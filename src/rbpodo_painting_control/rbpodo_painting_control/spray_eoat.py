"""Verified, content-addressed spray tool geometry in the robot TCP frame."""

from dataclasses import dataclass
from functools import lru_cache
import hashlib
import io
import json
from pathlib import Path
import struct

import numpy as np
from scipy.spatial.transform import Rotation

from .spray_geometry import resolve_spray_tool_axis


MAX_MESH_BYTES = 50_000_000
MAX_MESH_FACES = 250_000
MAX_TCP_REACH_M = 5.0


def _numeric_array(value, shape, label):
    if any(isinstance(item, (bool, np.bool_))
           for item in np.asarray(value, dtype=object).flat):
        raise ValueError(f"{label} must not contain booleans")
    raw = np.asarray(value)
    if raw.shape != shape or raw.dtype.kind not in "iuf":
        raise ValueError(f"{label} must be numeric with shape {shape}")
    result = raw.astype(float)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must be finite")
    return result


def _immutable(array, dtype):
    values = np.asarray(array, dtype=dtype)
    return np.frombuffer(values.tobytes(), dtype=dtype).reshape(values.shape)


@dataclass(frozen=True)
class SprayEoatProfile:
    sha256: str
    endpoint_tcp_m: np.ndarray
    vertices_tcp_m: np.ndarray
    faces: np.ndarray

    def metadata(self):
        return {
            "spray_eoat_profile_sha256": self.sha256,
            "spray_endpoint_tcp_m": self.endpoint_tcp_m.tolist(),
        }


def compensate_spray_endpoint(position, rotation, endpoint_tcp_m):
    """Convert a desired endpoint position into a TCP origin position."""
    position = _numeric_array(position, (3,), "endpoint position")
    endpoint = _numeric_array(endpoint_tcp_m, (3,), "TCP endpoint offset")
    rotation = _numeric_array(rotation, (3, 3), "TCP rotation")
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8, rtol=0)
            or not np.isclose(np.linalg.det(rotation), 1., atol=1e-8, rtol=0)):
        raise ValueError("TCP rotation must be a proper rigid rotation")
    return position - rotation @ endpoint


def _read_bounded(path, limit):
    with path.open("rb") as stream:
        data = stream.read(limit + 1)
    if not data or len(data) > limit:
        raise ValueError(f"empty or oversized spray EOAT file: {path}")
    return data


def _unique_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate EOAT profile key: {key}")
        result[key] = value
    return result


def _complete_mesh_triangle_count(data, file_type):
    """Reject incomplete records before the permissive mesh importer can drop them.

    Geometry decoding remains in trimesh. The accepted OBJ subset is an
    already-triangulated mesh, not curves, free-form surfaces or vertex colors.
    """
    if file_type == "stl" and len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * count:
            return count
    try:
        lines = [line.split("#", 1)[0].split() for line in data.decode("utf-8-sig").splitlines()]
    except UnicodeError as exc:
        raise ValueError("incomplete binary STL or invalid text mesh") from exc
    lines = [line for line in lines if line]

    def numbers(tokens, count):
        if len(tokens) != count or not all(np.isfinite(float(v)) for v in tokens):
            raise ValueError("incomplete or nonfinite mesh record")

    count = 0
    if file_type == "obj":
        counts = {"v": 0, "vt": 0, "vn": 0}
        for line in lines:
            kind, values = line[0], line[1:]
            if kind in counts:
                if kind == "vt":
                    if not 1 <= len(values) <= 3:
                        raise ValueError("invalid OBJ texture vertex")
                    numbers(values, len(values))
                else:
                    numbers(values, 3)
                counts[kind] += 1
            elif kind == "f":
                if len(values) != 3:
                    raise ValueError("OBJ must contain complete triangulated faces")
                for value in values:
                    parts = value.split("/")
                    if not 1 <= len(parts) <= 3 or not parts[0] or not parts[-1]:
                        raise ValueError("invalid OBJ face reference")
                    for index, part in enumerate(parts):
                        if not part and index == 1 and len(parts) == 3:
                            continue
                        reference = int(part)
                        available = counts[("v", "vt", "vn")[index]]
                        if reference <= 0:
                            raise ValueError("OBJ requires positive absolute face indices")
                        if reference > available:
                            raise ValueError("OBJ face index is out of range")
                count += 1
            elif kind not in {"o", "g", "s", "usemtl", "mtllib"}:
                raise ValueError(f"unsupported OBJ record: {kind}")
        return count

    index = 0
    while index < len(lines):
        if lines[index][0] != "solid":
            raise ValueError("ASCII STL requires complete solid blocks")
        index += 1
        while index < len(lines) and lines[index][0] != "endsolid":
            if index + 6 >= len(lines):
                raise ValueError("truncated ASCII STL facet")
            facet = lines[index:index + 7]
            if (facet[0][:2] != ["facet", "normal"] or facet[1] != ["outer", "loop"]
                    or facet[5] != ["endloop"] or facet[6] != ["endfacet"]):
                raise ValueError("invalid ASCII STL facet structure")
            numbers(facet[0][2:], 3)
            for vertex in facet[2:5]:
                if vertex[0] != "vertex":
                    raise ValueError("invalid ASCII STL vertex record")
                numbers(vertex[1:], 3)
            count += 1
            index += 7
        if index >= len(lines):
            raise ValueError("ASCII STL is missing endsolid")
        index += 1
    return count


def load_spray_eoat_profile(path, model_id, tool_axis):
    """Read a confirmed JSON profile and STL/OBJ mesh anew on every call.

    File units and the mesh-to-TCP mounting transform must be explicit. The
    axial extreme is only a candidate: endpoint_confirmed must be true even
    when the operator supplies an explicit, calibrated nozzle outlet center.
    Both files are read before any content-keyed geometry cache lookup. A
    deleted or unreadable file can never fall back to a previously loaded tool.
    """
    try:
        return _load_profile(path, model_id, tool_axis)
    except (OSError, TypeError, KeyError, ImportError, UnicodeError) as exc:
        raise ValueError(f"spray EOAT profile unavailable or invalid: {exc}") from exc


def _load_profile(path, model_id, tool_axis):
    if not isinstance(path, (str, Path)) or not str(path).strip():
        raise ValueError("spray_eoat_profile requires a confirmed EOAT JSON file")
    path = Path(path).expanduser().resolve()
    config = json.loads(_read_bounded(path, 1_000_000).decode("utf-8-sig"),
                        object_pairs_hook=_unique_keys)
    if not isinstance(config, dict) or type(config.get("schema_version")) is not int:
        raise ValueError("spray EOAT profile requires integer schema_version")
    if config["schema_version"] != 1:
        raise ValueError("unsupported spray EOAT profile schema_version")
    allowed = {"schema_version", "model_id", "spray_tool_axis", "mesh_file",
               "mesh_scale_to_m", "mesh_to_tcp", "endpoint_confirmed", "endpoint_tcp_m"}
    if set(config) - allowed:
        raise ValueError("unknown spray EOAT profile keys")
    axis = resolve_spray_tool_axis(model_id, tool_axis)
    if config.get("model_id") != model_id or config.get("spray_tool_axis") != axis:
        raise ValueError("spray EOAT profile model/tool axis does not match the robot")
    if config.get("endpoint_confirmed") is not True:
        raise ValueError("spray EOAT endpoint has not been physically confirmed")
    scale = config["mesh_scale_to_m"]
    if type(scale) not in (int, float) or not np.isfinite(scale) or scale <= 0:
        raise ValueError("mesh_scale_to_m must be finite and positive")
    mount = config["mesh_to_tcp"]
    if not isinstance(mount, dict) or set(mount) != {"translation_m", "quaternion_xyzw"}:
        raise ValueError("mesh_to_tcp requires translation_m and quaternion_xyzw")
    _numeric_array(mount["translation_m"], (3,), "mount translation")
    quaternion = _numeric_array(mount["quaternion_xyzw"], (4,), "mount quaternion")
    if not np.isclose(np.linalg.norm(quaternion), 1., atol=1e-6, rtol=0):
        raise ValueError("mesh mount quaternion must have unit length")
    resource = config["mesh_file"]
    if not isinstance(resource, str) or not resource.strip() or "://" in resource:
        raise ValueError("mesh_file must be a local STL or OBJ path")
    mesh_path = Path(resource).expanduser()
    if not mesh_path.is_absolute():
        mesh_path = path.parent / mesh_path
    file_type = mesh_path.suffix.lower().lstrip(".")
    if file_type not in ("stl", "obj"):
        raise ValueError("spray EOAT mesh must be STL or OBJ")
    mesh_bytes = _read_bounded(mesh_path, MAX_MESH_BYTES)
    canonical_config = json.dumps(config, sort_keys=True, separators=(",", ":"),
                                  allow_nan=False)
    return _decode_profile(canonical_config, mesh_bytes, file_type)


@lru_cache(maxsize=2)
def _decode_profile(canonical_config, mesh_bytes, file_type):
    # The cache key contains both complete file contents, never paths or mtimes.
    # Immutable outputs keep repeated active-motion checks inexpensive.
    config = json.loads(canonical_config)
    scale = config["mesh_scale_to_m"]
    mount = config["mesh_to_tcp"]
    translation = np.asarray(mount["translation_m"], dtype=float)
    rotation = Rotation.from_quat(mount["quaternion_xyzw"]).as_matrix()
    axis = config["spray_tool_axis"]
    expected_faces = _complete_mesh_triangle_count(mesh_bytes, file_type)
    # Decode the exact hashed bytes; do not let OBJ materials resolve other files.
    import trimesh
    try:
        mesh = trimesh.load_mesh(io.BytesIO(mesh_bytes), file_type=file_type,
                                 process=False, resolver={}, skip_materials=True)
    except Exception as exc:
        raise ValueError(f"cannot decode spray EOAT mesh: {exc}") from exc
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError("spray EOAT asset must contain a triangle mesh")
    vertices = np.asarray(mesh.vertices, dtype=float)
    faces = np.asarray(mesh.faces)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or not len(vertices)
            or not np.all(np.isfinite(vertices)) or faces.ndim != 2
            or faces.shape[1] != 3 or not 0 < len(faces) <= MAX_MESH_FACES
            or len(faces) != expected_faces
            or faces.dtype.kind not in "iu" or np.min(faces) < 0
            or np.max(faces) >= len(vertices)):
        raise ValueError("invalid or oversized spray EOAT triangle mesh")
    vertices = (vertices * scale) @ rotation.T + translation
    if (not np.all(np.isfinite(vertices))
            or np.max(np.linalg.norm(vertices, axis=1)) > MAX_TCP_REACH_M):
        raise ValueError("spray EOAT exceeds 5m TCP reach; check mesh units/mount")
    triangles = vertices[faces]
    areas = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                   triangles[:, 2] - triangles[:, 0]), axis=1)
    if np.any(areas <= 1e-18):
        raise ValueError("spray EOAT mesh has degenerate collision triangles")
    # Unreferenced OBJ vertices cannot define the working endpoint or its bounds.
    referenced = np.unique(vertices[np.unique(faces)], axis=0)
    signed_distances = referenced[:, "xyz".index(axis[1])] * (1 if axis[0] == "+" else -1)
    extreme = float(np.max(signed_distances))
    candidate = referenced[np.isclose(signed_distances, extreme, atol=1e-8, rtol=0)].mean(axis=0)
    endpoint = _numeric_array(config.get("endpoint_tcp_m", candidate), (3,), "endpoint_tcp_m")
    if (np.any(endpoint < referenced.min(axis=0) - .002)
            or np.any(endpoint > referenced.max(axis=0) + .002)):
        raise ValueError("confirmed endpoint is outside EOAT mesh bounds (2mm tolerance)")
    vertices = _immutable(vertices, "<f8")
    faces = _immutable(faces, "<i8")
    endpoint = _immutable(endpoint, "<f8")
    digest = hashlib.sha256()
    digest.update(canonical_config.encode("utf-8"))
    digest.update(hashlib.sha256(mesh_bytes).digest())
    digest.update(vertices.tobytes())
    digest.update(faces.tobytes())
    digest.update(endpoint.tobytes())
    return SprayEoatProfile(digest.hexdigest(), endpoint, vertices, faces)
