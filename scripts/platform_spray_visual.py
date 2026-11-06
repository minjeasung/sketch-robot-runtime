"""Simulation-only spray envelope and sampled, first-hit CAD surface coverage.

There is no output driver here. Coverage means geometric intersection, never
paint mass or thickness. The original STEP slope projects onto all first-hit
CAD surfaces, beyond its finite display cap. An optional display extension is
reported separately; no selected rectangle clips it. Cells stay on original
CAD triangles. A finite cell can straddle the analytical footprint boundary.

``SprayVisual(stage, envelope, collision_json).update(world_from_nozzle,
                                                    enabled, plan_id)``
uses column-vector rigid transforms in metres. Pass stage=None for geometry
without Isaac/USD. Spray illustrations intentionally appear in simulation
RGB/depth images as well as the viewport. They are visual geometry, not material
simulation or physical sensor observations.
"""
import heapq
import math

import numpy as np


def _rigid_pose(value):
    pose = np.asarray(value, dtype=float)
    if (pose.shape != (4, 4) or not np.isfinite(pose).all()
            or not np.allclose(pose[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(pose[:3, :3]), 1., atol=1e-5)):
        raise ValueError('Expected a finite rigid world_from_nozzle transform')
    return pose


def _surface_triangles(geometry):
    if geometry.get('frame_id') != 'World':
        raise ValueError('Surface geometry must use frame_id World in metres')
    result = []
    for mesh in geometry['meshes']:
        vertices = np.asarray(mesh['vertices'], dtype=float)
        indices = np.asarray(mesh['triangles'], dtype=int)
        if (vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all()
                or indices.ndim != 2 or indices.shape[1] != 3
                or np.any(indices < 0) or np.any(indices >= len(vertices))):
            raise ValueError('Invalid CAD triangle geometry')
        result.extend(vertices[indices])
    triangles = np.asarray(result, dtype=float).reshape(-1, 3, 3)
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    return triangles[np.linalg.norm(cross, axis=1) > 1e-12]


def _surface_cells(triangles, spacing, limit):
    """Longest-edge bisection keeps patches on CAD faces and bounds memory."""
    if spacing <= 0 or limit < len(triangles) or limit < 1:
        raise ValueError('Cell size must be positive and budget must cover all CAD triangles')
    heap = []
    serial = 0

    def push(triangle):
        nonlocal serial
        edges = triangle[[1, 2, 0]] - triangle
        lengths = np.einsum('ij,ij->i', edges, edges)
        longest = int(np.argmax(lengths))
        heapq.heappush(heap, (-float(lengths[longest]), serial, longest, triangle))
        serial += 1

    for triangle in triangles:
        push(triangle)
    while heap and -heap[0][0] > spacing * spacing and len(heap) < limit:
        _, _, edge, triangle = heapq.heappop(heap)
        a, b, c = triangle[[edge, (edge + 1) % 3, (edge + 2) % 3]]
        midpoint = (a + b) * .5
        push(np.array([a, midpoint, c]))
        push(np.array([midpoint, b, c]))
    return np.array([item[3] for item in heap], dtype=float).reshape(-1, 3, 3)


def _triangle_tree(triangles, indices):
    """Static CAD BVH; median splits bound ray/triangle work at render time."""
    selected = triangles[indices]
    bounds = (selected.min(axis=(0, 1)), selected.max(axis=(0, 1)))
    if len(indices) <= 8:
        return bounds, indices, None, None
    centers = selected.mean(axis=1)
    axis = int(np.argmax(np.ptp(centers, axis=0)))
    ordered = indices[np.argsort(centers[:, axis])]
    mid = len(ordered) // 2
    return bounds, None, _triangle_tree(triangles, ordered[:mid]), _triangle_tree(triangles, ordered[mid:])


class SprayVisual:
    """Batched USD visualization with bounded numpy-only geometric coverage."""

    def __init__(self, stage, envelope, surface_geometry, *,
                 root_path='/World/SimulationSpray', cell_size_m=.025, max_cells=6000,
                 projection_end_m=None):
        self.axial_start = float(envelope['axial_start_m'])
        self.start_radius = float(envelope['start_radius_m'])
        self.length = float(envelope['length_m'])
        self.end_radius = float(envelope['end_radius_m'])
        if (not np.isfinite([self.axial_start, self.start_radius, self.length, self.end_radius]).all()
                or self.length <= 0 or self.start_radius < 0 or self.end_radius <= self.start_radius):
            raise ValueError('Expected an expanding finite spray frustum')
        self.slope = (self.end_radius - self.start_radius) / self.length
        self.apex_z = self.axial_start - self.start_radius / self.slope
        self.source_axial_end = self.axial_start + self.length
        if projection_end_m is not None and not math.isfinite(float(projection_end_m)):
            raise ValueError('Expected a finite projection_end_m')
        self.visual_axial_end = max(self.source_axial_end, float(projection_end_m)
                                    if projection_end_m is not None else self.source_axial_end)
        self.triangles = _surface_triangles(surface_geometry)
        self.cell_vertices = _surface_cells(self.triangles, float(cell_size_m), int(max_cells))
        self.cell_centers = self.cell_vertices.mean(axis=1)
        crosses = np.cross(self.cell_vertices[:, 1] - self.cell_vertices[:, 0],
                           self.cell_vertices[:, 2] - self.cell_vertices[:, 0])
        magnitude = np.linalg.norm(crosses, axis=1)
        self.cell_areas = magnitude * .5
        self.cell_normals = crosses / magnitude[:, None]
        self.covered = np.zeros(len(self.cell_centers), dtype=bool)
        self.footprint = self.covered.copy()
        self._offset_sign = np.ones(len(self.covered))
        self._edge1 = self.triangles[:, 1] - self.triangles[:, 0]
        self._edge2 = self.triangles[:, 2] - self.triangles[:, 0]
        self._tree = (_triangle_tree(self.triangles, np.arange(len(self.triangles)))
                      if len(self.triangles) else None)
        self._plan_id = None
        self._sample_pose = None
        self._enabled = False
        self._sensor_capture = False
        self.root_path = root_path
        self._renderer = _UsdSprayRenderer(stage, root_path) if stage is not None else None

    def radius_at(self, distance_m):
        """Source linear envelope, including extrapolation beyond the finite end."""
        return self.start_radius + (np.asarray(distance_m) - self.axial_start) * self.slope

    def cone_points(self, world_from_nozzle, segments=48):
        pose = _rigid_pose(world_from_nozzle)
        angle = np.arange(segments) * (2 * math.pi / segments)
        circles = []
        for z, radius in ((self.axial_start, self.start_radius),
                          (self.visual_axial_end, float(self.radius_at(self.visual_axial_end)))):
            circles.append(np.column_stack((np.cos(angle) * radius, np.sin(angle) * radius,
                                            np.full(segments, z))))
        return np.concatenate(circles) @ pose[:3, :3].T + pose[:3, 3]

    def _unoccluded(self, apex, targets):
        """First-hit segment tests against a static CAD bounding volume tree."""
        clear = np.ones(len(targets), dtype=bool)
        if self._tree is None or not len(targets):
            return clear
        rays = targets - apex
        inverse = np.divide(1., rays, out=np.full_like(rays, 1e30), where=np.abs(rays) > 1e-15)
        stack = [(self._tree, np.arange(len(rays)))]
        while stack:
            node, ray_ids = stack.pop()
            ray_ids = ray_ids[clear[ray_ids]]
            if not len(ray_ids):
                continue
            bounds, triangle_ids, left, right = node
            first = (bounds[0] - apex - 1e-9) * inverse[ray_ids]
            last = (bounds[1] - apex + 1e-9) * inverse[ray_ids]
            near = np.minimum(first, last).max(axis=1)
            far = np.maximum(first, last).min(axis=1)
            ray_ids = ray_ids[(far >= np.maximum(near, 0)) & (near <= 1) & (far >= 0)]
            if not len(ray_ids):
                continue
            if triangle_ids is None:
                stack.extend(((left, ray_ids), (right, ray_ids)))
                continue
            edge1, edge2 = self._edge1[triangle_ids], self._edge2[triangle_ids]
            displacement = apex - self.triangles[triangle_ids, 0]
            qvec = np.cross(displacement, edge1)
            pvec = np.cross(rays[ray_ids, None, :], edge2[None, :, :])
            det = np.einsum('tj,rtj->rt', edge1, pvec)
            usable = np.abs(det) > 1e-12
            invdet = np.divide(1., det, out=np.zeros_like(det), where=usable)
            u = np.einsum('tj,rtj->rt', displacement, pvec) * invdet
            v = np.einsum('rj,tj->rt', rays[ray_ids], qvec) * invdet
            t = np.einsum('tj,tj->t', edge2, qvec)[None, :] * invdet
            blocking = (usable & (u >= -1e-9) & (v >= -1e-9) & (u + v <= 1 + 1e-9)
                        & (t > 1e-7) & (t < 1 - 1e-6))
            clear[ray_ids[blocking.any(axis=1)]] = False
        return clear

    def update(self, world_from_nozzle, enabled, plan_id):
        pose = _rigid_pose(world_from_nozzle)
        changed_plan = plan_id != self._plan_id
        if changed_plan:
            self.covered[:] = False
            self._sample_pose = None
            self._plan_id = plan_id
        enabled = bool(enabled)
        recompute = enabled and (self._sample_pose is None or not self._enabled)
        if enabled and self._sample_pose is not None:
            delta = pose[:3, :3].T @ self._sample_pose[:3, :3]
            angle = math.acos(float(np.clip((np.trace(delta) - 1) * .5, -1, 1)))
            recompute |= (np.linalg.norm(pose[:3, 3] - self._sample_pose[:3, 3]) >= .005
                          or angle >= math.radians(.5))
        if not enabled:
            self.footprint[:] = False
        elif recompute:
            local = (self.cell_centers - pose[:3, 3]) @ pose[:3, :3]
            radius = self.radius_at(local[:, 2])
            inside = ((local[:, 2] >= self.axial_start)
                      & (np.linalg.norm(local[:, :2], axis=1) <= radius))
            candidates = np.flatnonzero(inside)
            apex = pose[:3, :3] @ np.array([0., 0., self.apex_z]) + pose[:3, 3]
            self.footprint[:] = False
            hits = candidates[self._unoccluded(apex, self.cell_centers[candidates])]
            self.footprint[hits] = True
            self.covered |= self.footprint
            towards = apex - self.cell_centers[hits]
            self._offset_sign[hits] = np.where(
                np.einsum('ij,ij->i', towards, self.cell_normals[hits]) >= 0, 1., -1.)
            self._sample_pose = pose.copy()
        self._enabled = enabled
        if self._renderer:
            self._renderer.update(self, pose, meshes_changed=bool(recompute or changed_plan or not enabled))
        return dict(enabled=enabled, plan_id=plan_id, physical_output_enabled=False,
                    coverage_kind='geometric_surface_cells', coverage_is_approximate=True,
                    source_axial_end_m=self.source_axial_end,
                    visual_extrapolation_m=self.visual_axial_end - self.source_axial_end,
                    coverage_axial_limit_m=None,
                    footprint_area_m2=float(self.cell_areas[self.footprint].sum()),
                    covered_area_m2=float(self.cell_areas[self.covered].sum()),
                    covered_cells=int(self.covered.sum()), surface_cells=len(self.covered),
                    footprint_cells=int(self.footprint.sum()), geometry_recomputed=bool(recompute),
                    sensor_capture_hidden=self._sensor_capture,
                    sensor_exclusion='none; illustrations visible in simulation RGB/depth',
                    visualization_root=self.root_path)

    def set_sensor_capture(self, active):
        """Caller brackets sensor rendering to keep virtual overlays out of depth."""
        self._sensor_capture = bool(active)
        if self._renderer:
            self._renderer.set_hidden(self._sensor_capture)


class _UsdSprayRenderer:
    """One wireframe and two batched meshes; no rigid body or collision APIs."""

    def __init__(self, stage, path):
        from pxr import UsdGeom
        self.UsdGeom = UsdGeom
        self.root = UsdGeom.Xform.Define(stage, path)
        self.root.GetPrim().SetCustomDataByKey('simulationOnly', True)
        self.cone = UsdGeom.BasisCurves.Define(stage, path + '/Envelope')
        self.cone.CreateTypeAttr('linear')
        self.cone.CreateWrapAttr('nonperiodic')
        self.cone.CreateWidthsAttr([.0007])
        self.cone.SetWidthsInterpolation('constant')
        self.cone.CreateDisplayColorAttr([(0.37, .76, .88)])
        self.coverage = UsdGeom.Mesh.Define(stage, path + '/Coverage')
        self.footprint = UsdGeom.Mesh.Define(stage, path + '/Footprint')
        for mesh, color in ((self.coverage, (.30, .56, .65)), (self.footprint, (.52, .83, .88))):
            mesh.CreateSubdivisionSchemeAttr('none')
            mesh.CreateDoubleSidedAttr(True)
            mesh.CreateDisplayColorAttr([color])
            mesh.CreateDisplayOpacityAttr([1.])
        self.cone.CreateVisibilityAttr('invisible')

    def set_hidden(self, hidden):
        self.root.CreateVisibilityAttr('invisible' if hidden else 'inherited')

    def update(self, model, pose, *, meshes_changed):
        from pxr import Vt
        self.cone.CreateVisibilityAttr('inherited' if model._enabled else 'invisible')
        if model._enabled:
            rings = model.cone_points(pose)
            n = len(rings) // 2
            strokes = [np.concatenate((rings[:n], rings[:1])),
                       np.concatenate((rings[n:], rings[n:n + 1]))]
            strokes.extend(rings[[i, n + i]] for i in range(0, n, 6))
            points = np.concatenate(strokes).astype(np.float32)
            self.cone.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
            self.cone.CreateCurveVertexCountsAttr([len(stroke) for stroke in strokes])
        if meshes_changed:
            for mesh, mask, lift in ((self.coverage, model.covered & ~model.footprint, .00025),
                                     (self.footprint, model.footprint, .00035)):
                offset = model.cell_normals[mask] * model._offset_sign[mask, None] * lift
                points = (model.cell_vertices[mask] + offset[:, None, :]).reshape(-1, 3)
                mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
                mesh.CreateFaceVertexCountsAttr([3] * int(mask.sum()))
                mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))
