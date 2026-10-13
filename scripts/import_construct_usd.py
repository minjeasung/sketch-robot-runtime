"""Run inside Isaac 6.1 via setup_construct_isaac.sh."""
import os
from pathlib import Path
import omni.kit.app
from isaacsim.core.utils.extensions import enable_extension
from pxr import Usd, UsdGeom, UsdPhysics

try:
    root = Path(os.environ['SKETCH_WORKSPACE'])
    assets = root / 'isaac_assets'
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    imported = Path(URDFImporter(URDFImporterConfig(
        urdf_path=str(assets/'construct.urdf'), usd_path=str(assets/'construct_generated'),
        fix_base=True, merge_fixed_joints=False, merge_mesh=False,
    )).import_urdf())
    stage = Usd.Stage.Open(str(imported))
    stage.GetDefaultPrim().GetVariantSet('Physics').SetVariantSelection('physx')
    stage.GetRootLayer().Save()
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)]
    assert len(joints) == 14, f'Expected 14 joints, found {len(joints)}'
    output = Usd.Stage.CreateNew(str(assets/'construct.usd'))
    robot = output.DefinePrim('/construct', 'Xform')
    robot.GetReferences().AddReference(os.path.relpath(imported, assets))
    robot.GetVariantSet('Physics').SetVariantSelection('physx')
    output.SetDefaultPrim(robot)
    UsdGeom.SetStageMetersPerUnit(output, 1.0)
    UsdGeom.SetStageUpAxis(output, UsdGeom.Tokens.z)
    output.GetRootLayer().Save()
    print(f'CONSTRUCT_IMPORT_OK joints={len(joints)} path={assets / "construct.usd"}', flush=True)
    omni.kit.app.get_app().post_quit(0)
    os._exit(0)
except Exception:
    import traceback
    traceback.print_exc()
    os._exit(1)
