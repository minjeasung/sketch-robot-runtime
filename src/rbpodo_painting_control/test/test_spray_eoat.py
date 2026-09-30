import json

import numpy as np
import pytest
import trimesh

from rbpodo_painting_control.spray_eoat import (
    compensate_spray_endpoint, load_spray_eoat_profile,
)
from rbpodo_painting_control.spray_geometry import rotation_from_spray_path


def profile_file(tmp_path, **overrides):
    mesh = trimesh.creation.box(extents=[40., 60., 200.])
    mesh.apply_translation([10., 20., 100.])
    (tmp_path / "tool.stl").write_bytes(mesh.export(file_type="stl"))
    data = dict(schema_version=1, model_id="rb20_1900es", spray_tool_axis="+z",
                mesh_file="tool.stl", mesh_scale_to_m=.001,
                mesh_to_tcp=dict(translation_m=[.01, -.02, .05],
                                 quaternion_xyzw=[0., 0., 0., 1.]),
                endpoint_confirmed=True)
    data.update(overrides)
    path = tmp_path / "tool.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def load(path, model="rb20_1900es", axis="+z"):
    return load_spray_eoat_profile(str(path), model, axis)


def test_mesh_scale_mount_and_endpoint_centroid(tmp_path):
    profile = load(profile_file(tmp_path))
    np.testing.assert_allclose(profile.endpoint_tcp_m, [.02, 0., .25], atol=1e-12)
    assert profile.vertices_tcp_m.shape[1] == 3
    assert len(profile.faces) == 12
    assert len(profile.sha256) == 64
    assert profile.metadata() == dict(spray_eoat_profile_sha256=profile.sha256,
                                     spray_endpoint_tcp_m=profile.endpoint_tcp_m.tolist())
    with pytest.raises(ValueError):
        profile.endpoint_tcp_m[0] = 1.


def test_rb10_rotated_mount_maps_cad_z_to_tcp_minus_y(tmp_path):
    path = profile_file(tmp_path, model_id="rb10_1300e_u", spray_tool_axis="-y",
                        mesh_to_tcp=dict(translation_m=[0., 0., 0.],
                                         quaternion_xyzw=[2**-.5, 0., 0., 2**-.5]))
    np.testing.assert_allclose(load(path, "rb10_1300e_u", "-y").endpoint_tcp_m,
                               [.01, -.2, .02], atol=1e-12)


@pytest.mark.parametrize("axis", ["+z", "-y", "+x", "-z"])
def test_lateral_endpoint_compensated_for_tilted_wall(axis):
    normal = np.array([1., 2., 3.]) / np.sqrt(14)
    rotation = rotation_from_spray_path(normal, [0., 1., 0.], axis)
    endpoint = np.array([.03, -.24, .11])
    surface = np.array([.8, -.3, .2])
    tcp = compensate_spray_endpoint(surface + .5 * normal, rotation, endpoint)
    actual_end = tcp + rotation @ endpoint
    np.testing.assert_allclose(actual_end, surface + .5 * normal, atol=1e-12)
    assert np.dot(actual_end - surface, normal) == pytest.approx(.5)


def test_override_is_nozzle_outlet_not_guard_tip(tmp_path):
    profile = load(profile_file(tmp_path, endpoint_tcp_m=[.02, .0, .23]))
    np.testing.assert_allclose(profile.endpoint_tcp_m, [.02, .0, .23])


@pytest.mark.parametrize("change", [dict(endpoint_confirmed=False),
    dict(endpoint_confirmed="true"), dict(mesh_scale_to_m=0),
    dict(mesh_scale_to_m=True), dict(mesh_scale_to_m="0.001"),
    dict(mesh_scale_to_m=float("nan")), dict(schema_version=2),
    dict(model_id="rb10_1300e_u"), dict(spray_tool_axis="-y"),
    dict(endpoint_tcp_m=[.02, False, .23]),
    dict(endpoint_tcp_m=[0, 0]), dict(endpoint_tcp_m=[0, 0, float("inf")]),
    dict(endpoint_tcp_m=[0, 0, 1000]), dict(mesh_file="missing.stl"),
    dict(mesh_to_tcp=dict(translation_m=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 2]))])
def test_bad_or_unconfirmed_profile_fails_closed(tmp_path, change):
    with pytest.raises(ValueError):
        load(profile_file(tmp_path, **change))


def test_empty_path_rejected():
    with pytest.raises(ValueError):
        load("")


def test_mesh_and_mount_changes_change_identity_without_cache(tmp_path):
    path = profile_file(tmp_path)
    first = load(path)
    mesh_path = tmp_path / "tool.stl"
    mesh = trimesh.creation.box(extents=[40, 60, 210])
    mesh.apply_translation([10, 20, 105])
    mesh_path.write_bytes(mesh.export(file_type="stl"))
    second = load(path)
    assert first.sha256 != second.sha256
    assert second.endpoint_tcp_m[2] == pytest.approx(.26)
    data = json.loads(path.read_text())
    data["mesh_to_tcp"]["translation_m"][0] += .01
    path.write_text(json.dumps(data))
    third = load(path)
    assert third.sha256 != second.sha256
    assert third.endpoint_tcp_m[0] == pytest.approx(.03)


@pytest.mark.parametrize("file_type", ["obj", "stl_ascii"])
def test_supported_text_meshes(tmp_path, file_type):
    path = profile_file(tmp_path)
    mesh = trimesh.creation.box(extents=[40, 60, 200])
    mesh.apply_translation([10, 20, 100])
    target = tmp_path / ("tool.obj" if file_type == "obj" else "tool.stl")
    target.write_text(mesh.export(file_type=file_type))
    data = json.loads(path.read_text())
    data["mesh_file"] = target.name
    path.write_text(json.dumps(data))
    np.testing.assert_allclose(load(path).endpoint_tcp_m, [.02, 0, .25], atol=1e-12)


@pytest.mark.parametrize("bad", [b"", b"not a mesh", b"solid empty\nendsolid empty\n"])
def test_bad_mesh_fails_closed(tmp_path, bad):
    path = profile_file(tmp_path)
    (tmp_path / "tool.stl").write_bytes(bad)
    with pytest.raises(ValueError):
        load(path)


def test_non_rigid_rotation_rejected():
    with pytest.raises(ValueError):
        compensate_spray_endpoint([0, 0, 0], np.eye(3) * 2, [0, 0, .2])


@pytest.mark.parametrize("key", ["mesh_scale_to_m", "mesh_to_tcp", "mesh_file"])
def test_missing_mount_or_units_never_inferred(tmp_path, key):
    path = profile_file(tmp_path)
    data = json.loads(path.read_text())
    del data[key]
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load(path)


def test_duplicate_config_keys_fail_closed(tmp_path):
    path = profile_file(tmp_path)
    path.write_text(path.read_text().replace('"schema_version": 1',
                                            '"schema_version": 1, "schema_version": 1'))
    with pytest.raises(ValueError, match="duplicate"):
        load(path)


def test_degenerate_collision_mesh_rejected(tmp_path):
    path = profile_file(tmp_path)
    mesh = trimesh.Trimesh(vertices=[[0, 0, 0], [0, 0, 1], [0, 0, 2]],
                           faces=[[0, 1, 2]], process=False)
    (tmp_path / "tool.stl").write_bytes(mesh.export(file_type="stl"))
    with pytest.raises(ValueError, match="degenerate"):
        load(path)


def test_endpoint_confirmation_change_invalidates_identity(tmp_path):
    path = profile_file(tmp_path)
    first = load(path)
    data = json.loads(path.read_text())
    data["endpoint_tcp_m"] = [.02, 0., .23]
    path.write_text(json.dumps(data))
    assert load(path).sha256 != first.sha256
    data["endpoint_confirmed"] = False
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="confirmed"):
        load(path)


def test_unchanged_content_skips_decode_but_removed_mesh_never_uses_cache(tmp_path, monkeypatch):
    path = profile_file(tmp_path)
    data = json.loads(path.read_text())
    data["mesh_to_tcp"]["translation_m"][0] = .012345
    path.write_text(json.dumps(data))
    calls = []
    original = trimesh.load_mesh
    def counted(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)
    monkeypatch.setattr(trimesh, "load_mesh", counted)
    first = load(path)
    assert load(path).sha256 == first.sha256
    assert len(calls) == 1
    (tmp_path / "tool.stl").unlink()
    with pytest.raises(ValueError):
        load(path)


@pytest.mark.parametrize("file_type", ["obj", "stl"])
def test_partially_valid_mesh_must_not_drop_truncated_nozzle(tmp_path, file_type):
    path = profile_file(tmp_path)
    mesh = trimesh.creation.box(extents=[40, 60, 200])
    mesh.apply_translation([10, 20, 100])
    if file_type == "obj":
        content = mesh.export(file_type="obj") + "\nv 0 0 400\nv 10 0 400\nf 9 10\n"
    else:
        content = mesh.export(file_type="stl_ascii") + "\nsolid nozzle\nfacet normal 0 0 1\nouter loop\nvertex 0 0 400\n"
    target = tmp_path / ("tool." + file_type)
    target.write_text(content)
    data = json.loads(path.read_text())
    data["mesh_file"] = target.name
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load(path)


def test_obj_relative_indices_rejected_instead_of_rebased_by_importer(tmp_path):
    path = profile_file(tmp_path)
    (tmp_path / "tool.obj").write_text(
        "v 0 0 100\nv 100 0 100\nv 0 100 100\nf -3 -2 -1\n"
        "v 0 0 400\nv 100 0 400\nv 0 100 400\n")
    data = json.loads(path.read_text())
    data["mesh_file"] = "tool.obj"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="positive"):
        load(path)
