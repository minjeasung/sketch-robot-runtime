#!/usr/bin/env python3
"""Stage the supplied H-steel USD and export its exact static collision triangles.

Uses pxr from the installed Isaac Python environment; never starts a simulator.
Placement is explicit and shared by the USD and the ROS-ready triangle export.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path


def prepare(root, output, translation=(0., 0., 0.)):
    from pxr import Usd, UsdGeom, UsdPhysics, Gf

    root, output = Path(root).resolve(), Path(output).resolve()
    if len(translation) != 3 or not all(math.isfinite(v) for v in translation):
        raise ValueError('Expected finite xyz translation in meters')
    source = root/'assets/construct/h_steel/source/Steel Structure.usdc'
    original = Usd.Stage.Open(str(source))
    if not original or not original.GetDefaultPrim():
        raise ValueError('H-steel source needs a default prim')
    if (UsdGeom.GetStageMetersPerUnit(original) != 1.
            or UsdGeom.GetStageUpAxis(original) != UsdGeom.Tokens.z):
        raise ValueError('Revalidate placement: source must use meters and Z-up')
    output.mkdir(parents=True, exist_ok=True)
    scene = Usd.Stage.CreateNew(str(output/'h_steel.usda'))
    world = UsdGeom.Xform.Define(scene, '/World')
    scene.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageMetersPerUnit(scene, 1.)
    UsdGeom.SetStageUpAxis(scene, UsdGeom.Tokens.z)
    workpiece = UsdGeom.Xform.Define(scene, '/World/HSteel')
    workpiece.GetPrim().GetReferences().AddReference(os.path.relpath(source, output))
    # Wrap the referenced root so source xform ops are preserved.
    world.AddTranslateOp().Set(Gf.Vec3d(*translation))
    cache = UsdGeom.XformCache()
    parts = []
    for prim in scene.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise ValueError('Workpiece must be static')
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        if not points or not counts or any(n != 3 for n in counts):
            raise ValueError('Revalidate triangulation: ' + str(prim.GetPath()))
        if len(indices) != 3 * len(counts) or any(i < 0 or i >= len(points) for i in indices):
            raise ValueError('Invalid triangle indices')
        transform = cache.GetLocalToWorldTransform(prim)
        vertices = [list(transform.Transform(Gf.Vec3d(*p))) for p in points]
        if not all(math.isfinite(v) for p in vertices for v in p):
            raise ValueError('Non-finite mesh points')
        faces = [list(indices[i:i+3]) for i in range(0, len(indices), 3)]
        reverse = (mesh.GetOrientationAttr().Get() == UsdGeom.Tokens.leftHanded)
        reverse ^= transform.GetDeterminant() < 0
        if reverse:
            faces = [list(reversed(face)) for face in faces]
        parts.append(dict(prim=str(prim.GetPath()), vertices=vertices, triangles=faces))
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
    if not parts:
        raise ValueError('No workpiece meshes')
    vertices = [p for part in parts for p in part['vertices']]
    bounds = [[fn(p[i] for p in vertices) for i in range(3)] for fn in (min, max)]
    meta = dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                meters_per_unit=1., up_axis='Z', translation_m=list(translation),
                bounds_m=bounds, mesh_count=len(parts),
                triangle_count=sum(len(p['triangles']) for p in parts),
                placement_calibrated=False)
    scene.GetRootLayer().Save()
    (output/'h_steel_collision.json').write_text(json.dumps(dict(frame_id='World', meshes=parts)))
    (output/'h_steel_manifest.json').write_text(json.dumps(meta, indent=2)+'\n')
    return meta


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--translation', type=float, nargs=3, default=(0., 0., 0.))
    args = parser.parse_args()
    print(json.dumps(prepare(Path(__file__).resolve().parents[1], args.output, args.translation), indent=2))
