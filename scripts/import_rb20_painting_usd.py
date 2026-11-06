"""Isaac 6.1 URDF importer; fail if the physical F/T boundary is absent."""
import hashlib
import json
import os
from pathlib import Path
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdPhysics


def import_assets(root):
    assets=root/'.runtime/rb20_painting/assets'
    from prepare_rb20_painting_assets import verify_assets
    manifest=verify_assets(root,assets,require_usd=False)
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    imported=Path(URDFImporter(URDFImporterConfig(
        urdf_path=str(assets/'robot_physics.urdf'),usd_path=str(assets/'robot_generated'),
        fix_base=True,merge_fixed_joints=False,merge_mesh=False,
        collision_type='Convex Decomposition',allow_self_collision=True,
    )).import_urdf())
    stage=Usd.Stage.Open(str(imported))
    stage.GetDefaultPrim().GetVariantSet('Physics').SetVariantSelection('physx')
    stage.GetRootLayer().Save()
    revolute=[p.GetName() for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    if set(revolute)!=set(manifest['joint_names']):
        raise RuntimeError(f'Wrong six articulation joints: {revolute}')
    sensor=[UsdPhysics.FixedJoint(p) for p in stage.Traverse() if p.IsA(UsdPhysics.FixedJoint) and p.GetName()=='ft_sensor_joint']
    if len(sensor)!=1:
        raise RuntimeError('Importer did not retain physical ft_sensor_joint')
    child_paths=sensor[0].GetBody1Rel().GetTargets()
    if len(child_paths)!=1:
        raise RuntimeError('F/T joint has no single downstream body')
    body=stage.GetPrimAtPath(child_paths[0])
    if not body.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError('F/T child is not a physical body')
    mass=UsdPhysics.MassAPI(body).GetMassAttr().Get()
    if mass is None or abs(mass-manifest['ft_boundary']['mass_kg'])>1e-5:
        raise RuntimeError(f'F/T downstream tool mass changed: {mass}')
    colliders=[str(p.GetPath()) for p in Usd.PrimRange(body,Usd.TraverseInstanceProxies()) if p.HasAPI(UsdPhysics.CollisionAPI)]
    if not colliders or not all(any(kind in name for name in colliders) for kind in ['roller_contact','aft200_link','paint_eoat_no_camera_link']):
        raise RuntimeError(f'F/T body does not include roller contact geometry: {colliders}')
    output=Usd.Stage.CreateNew(str(assets/'robot.usd'))
    robot=output.DefinePrim('/rb20','Xform')
    robot.GetReferences().AddReference(os.path.relpath(imported,assets))
    robot.GetVariantSet('Physics').SetVariantSelection('physx')
    output.SetDefaultPrim(robot)
    UsdGeom.SetStageMetersPerUnit(output,1.)
    UsdGeom.SetStageUpAxis(output,UsdGeom.Tokens.z)
    output.GetRootLayer().Save()
    evidence={'physics_urdf_sha256':manifest['generated_hashes']['robot_physics.urdf'],'scene_revision':manifest['scene_revision'],'joint_names':revolute,
              'ft_joint_path':str(sensor[0].GetPath()),'ft_body_path':str(body.GetPath()),
              'ft_mass_kg':mass,'colliders':colliders}
    evidence['usd_hashes']={str(p.relative_to(assets)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [assets/'robot.usd',*sorted((assets/'robot_generated').rglob('*.usd')),*sorted((assets/'robot_generated').rglob('*.usda'))]}
    (assets/'usd-verification.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print('RB20_IMPORT_OK '+json.dumps(evidence),flush=True)
    return evidence

if __name__=='__main__':
    import traceback
    import omni.kit.app
    try:
        import_assets(Path(os.environ['SKETCH_WORKSPACE']))
        omni.kit.app.get_app().post_quit(0)
        os._exit(0)
    except Exception:
        traceback.print_exc(); os._exit(1)
