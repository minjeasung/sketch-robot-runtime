"""Finite measured plane geometry. No RANSAC or upstream algorithm copies."""
import hashlib
import copy
import json
import numpy as np
import shapely
from scipy.spatial import Delaunay, QhullError
from shapely.geometry import Polygon, box
from shapely.ops import unary_union


def fingerprint(data):
    return hashlib.sha256(json.dumps(data, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def vector(value, shape):
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.isfinite(a).all() or np.max(np.abs(a)) > 10:
        raise ValueError('invalid metric geometry (metres required)')
    return a


def basis(normal):
    n = vector(normal, (3,))
    if abs(np.linalg.norm(n)-1) > .001:
        raise ValueError('unit normal required')
    seed = np.eye(3)[np.argmin(abs(n))]
    u = np.cross(seed, n)
    u /= np.linalg.norm(u)
    return np.column_stack((u, np.cross(n, u)))


def measured_cells(support, normal, max_edge=.05):
    """Triangulate only measured samples, excluding triangles across depth gaps.

    max_edge is an explicit measurement resolution, not an inferred extension.
    No inferred_support/footprint expansion is consumed.
    """
    points = np.asarray(support, float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or not np.isfinite(points).all():
        return []
    if len(points) > 20000 or not .005 <= max_edge <= .05:
        raise ValueError('measured support exceeds configured resolution/size')
    uv = basis(normal)
    center = points.mean(axis=0)
    xy = (points-center) @ uv
    xy = np.unique(xy, axis=0)
    try:
        triangles = xy[Delaunay(xy).simplices]
    except QhullError:
        return []
    lengths = np.linalg.norm(triangles-np.roll(triangles, 1, axis=1), axis=2)
    triangles = triangles[np.max(lengths, axis=1) <= max_edge]
    if len(triangles) > 12000:
        raise ValueError('too many measured support cells')
    return (center + triangles @ uv.T).tolist()


def support_shape(cells, normal, origin):
    uv = basis(normal)
    if not isinstance(cells, list) or not 1 <= len(cells) <= 12000:
        raise ValueError('finite measured support required')
    groups = {}
    for cell in cells:
        if not isinstance(cell, (list, tuple)) or not 3 <= len(cell) <= 64:
            raise ValueError('invalid support cell')
        groups.setdefault(len(cell), []).append(cell)
    polygons = []
    for count, group in groups.items():
        a = np.asarray(group, float)
        if a.shape != (len(group), count, 3):
            raise ValueError('invalid support cell')
        if not np.isfinite(a).all() or np.max(abs(a)) > 10 or np.max(abs((a-origin) @ normal)) > .002:
            raise ValueError('nonfinite/nonplanar support')
        # NumPy/Shapely batch operations keep thousands of measured triangles
        # within the live source heartbeat budget. The geometric checks match
        # the scalar path, including concavity and invalid/self-crossing cells.
        group_polygons = shapely.polygons((a-origin) @ uv)
        areas = shapely.area(group_polygons)
        if (not np.all(shapely.is_valid(group_polygons)) or np.any(areas < 1e-10)
                or np.any(abs(shapely.area(shapely.convex_hull(group_polygons))-areas) > 1e-9)):
            raise ValueError('degenerate/nonconvex support cell')
        polygons.extend(group_polygons)
    return unary_union(polygons), uv


def require_supported_polygon(points, cells, normal):
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3 or not np.isfinite(points).all():
        raise ValueError('invalid work polygon')
    origin = np.asarray(cells[0][0], float)
    shape, uv = support_shape(cells, normal, origin)
    if np.max(abs((points-origin) @ normal)) > .002:
        raise ValueError('work polygon outside plane support')
    area = Polygon((points-origin) @ uv)
    if not area.is_valid or area.area < 1e-8 or not shape.buffer(1e-8).covers(area):
        raise ValueError('work polygon crosses unmeasured support')


class StableSupport:
    """Freeze a conservative footprint while a locked measurement still covers it.

    Upstream verified_hold accumulates/remeshes samples on an unchanged plane.
    More samples do not authorize extending an existing work region. Removal,
    changed plane geometry or lost coverage establishes a new region instead.
    """
    def __init__(self):
        self.session = None
        self.entries = {}
        self.shapes = {}
        self.inputs = {}

    def update(self, entries, session):
        previous = self.entries if session == self.session else {}
        stable, shapes, inputs = {}, {}, {}
        for entry in entries:
            value = copy.deepcopy(entry)
            ident = entry['plane_id']
            if ident in stable:
                raise ValueError('duplicate plane identity')
            old = previous.get(ident)
            inputs[ident] = fingerprint({k:entry[k] for k in ('center', 'normal', 'cells')})
            if old is not None and inputs[ident] == self.inputs.get(ident):
                value['cells'] = copy.deepcopy(old['cells'])
                value['center'], value['normal'] = old['center'], old['normal']
                shape = self.shapes[ident]
            else:
                shape, _ = support_shape(entry['cells'], entry['normal'], np.asarray(entry['center']))
                if (old is not None and old['center'] == entry['center'] and old['normal'] == entry['normal']
                        and shape.buffer(1e-8).covers(self.shapes[ident])):
                    value['cells'] = copy.deepcopy(old['cells'])
                    value['center'], value['normal'] = old['center'], old['normal']
                    shape = self.shapes[ident]
            stable[ident], shapes[ident] = value, shape
        self.session, self.entries = session, stable
        self.shapes, self.inputs = shapes, inputs
        return list(stable.values())


def snapshot(entries, source_session, stamp_ns, calibration_id, frame_id='link0'):
    if not source_session or not calibration_id or frame_id != 'link0' or type(stamp_ns) is not int or stamp_ns <= 0:
        raise ValueError('source session, calibration, link0 frame and positive stamp required')
    clean, ids = [], set()
    for entry in entries:
        ident = entry['plane_id']
        if not isinstance(ident, str) or not ident or ident in ids:
            raise ValueError('invalid/duplicate plane identity')
        ids.add(ident)
        center = vector(entry['center'], (3,))
        n = vector(entry['normal'], (3,))
        shape, uv = support_shape(entry['cells'], n, center)
        count, rms = entry['inlier_count'], entry['rms_m']
        if type(count) is not int or count < 80 or not np.isfinite(rms) or not 0 <= rms <= .015:
            raise ValueError('insufficient measured plane quality')
        left, bottom, right, top = shape.bounds
        if min(right-left, top-bottom) < .01:
            raise ValueError('support is too narrow')
        corners = center + np.array([[left, top], [right, top], [right, bottom], [left, bottom]]) @ uv.T
        clean.append(dict(id=ident, center=corners.mean(axis=0).tolist(), normal=n.tolist(),
                          corners=corners.tolist(), support_cells=entry['cells'],
                          support_polygon=(center+np.asarray(shape.convex_hull.exterior.coords[:-1])@uv.T).tolist(),
                          inlier_count=count, rms_m=float(rms)))
    clean.sort(key=lambda p:p['id'])
    identity = dict(schema_version=1, source_session=source_session, frame_id=frame_id,
                    calibration_id=calibration_id, planes=clean)
    # Passing quality is checked above; changing sample counts/residuals is an
    # observation update, not a changed geometric work authorization.
    geometric = dict(identity, planes=[{k:v for k, v in p.items() if k not in ('inlier_count', 'rms_m')}
                                       for p in clean])
    return dict(identity, revision=fingerprint(geometric), stamp_ns=stamp_ns)


class SnapshotBuilder:
    """Reuse validated immutable geometry, never freshness or quality checks."""
    def __init__(self):
        self.key, self.value = None, None

    def build(self, entries, source_session, stamp_ns, calibration_id):
        if type(stamp_ns) is not int or stamp_ns <= 0:
            raise ValueError('positive source timestamp required')
        for entry in entries:
            count, rms = entry['inlier_count'], entry['rms_m']
            if type(count) is not int or count < 80 or not np.isfinite(rms) or not 0 <= rms <= .015:
                raise ValueError('insufficient measured plane quality')
        key = fingerprint(dict(session=source_session, calibration=calibration_id,
            planes=[{k:v for k, v in e.items() if k not in ('inlier_count', 'rms_m')} for e in entries]))
        if key != self.key:
            self.value = snapshot(entries, source_session, stamp_ns, calibration_id)
            self.key = key
        quality = {e['plane_id']:e for e in entries}
        planes = [dict(p, inlier_count=quality[p['id']]['inlier_count'], rms_m=quality[p['id']]['rms_m'])
                  for p in self.value['planes']]
        return dict(self.value, stamp_ns=stamp_ns, planes=planes)


def project_catalogue(data, intrinsics, size, transform, generation, selection):
    K = np.asarray(intrinsics, float)
    T = np.asarray(transform, float)
    if (K.shape != (3, 3) or not np.isfinite(K).all() or min(K[0, 0], K[1, 1]) <= 0
            or T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1])
            or not np.allclose(T[:3, :3].T@T[:3, :3], np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(T[:3, :3]), 1, atol=1e-6)):
        raise ValueError('invalid camera geometry')
    points = np.asarray(selection, float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        raise ValueError('invalid pixel selection')
    roi = box(*points.min(axis=0), *points.max(axis=0)) if len(points) == 2 else Polygon(points)
    if not roi.is_valid or roi.area <= 0:
        raise ValueError('invalid pixel ROI')
    planes = []
    for plane in data['planes']:
        visible = []
        for cell in plane['support_cells']:
            xyz = np.asarray(cell)@T[:3, :3].T+T[:3, 3]
            if np.min(xyz[:, 2]) <= .01:
                continue
            image = xyz@K.T
            pixels = image[:, :2]/image[:, 2:]
            p = Polygon(pixels)
            if p.is_valid and p.intersects(roi) and p.intersects(box(0, 0, size[0]-1, size[1]-1)):
                visible.append(p)
        if visible:
            item = dict(plane)
            outline = unary_union(visible).convex_hull
            item['polygon_px'] = np.asarray(outline.exterior.coords[:-1]).tolist()
            item['source_revision'] = data['revision']
            planes.append(item)
    return dict(generation=str(generation), frame_id=data['frame_id'], image_width=size[0], image_height=size[1],
                planes=planes, extraction_method='snucem_humble_measured', source_revision=data['revision'])
