#!/usr/bin/env python3
"""Run with the installed Isaac Python; imports a selected robot into USD."""
import argparse
from pathlib import Path
import os
import xml.etree.ElementTree as ET



def configure_robot_collision_meshes(stage):
    """Author the requested decomposition on existing URDF collision meshes.

    Isaac 6.1's collision_type option is only consumed when generating collision
    from visuals. Native URDF colliders otherwise keep convexHull, which bridges
    robot link concavities that remain open in MoveIt's original triangle meshes.
    """
    from pxr import Sdf,Usd,UsdGeom,UsdPhysics
    root=stage.GetDefaultPrim()
    if not root:raise ValueError('Imported robot root is missing')
    paths=[p.GetPath() for p in Usd.PrimRange(root,Usd.TraverseInstanceProxies())
           if p.IsA(UsdGeom.Mesh) and p.HasAPI(UsdPhysics.CollisionAPI)]
    if not paths:raise ValueError('Imported robot has no collision meshes')
    settings={'errorPercentage':1.,'hullVertexLimit':64,'maxConvexHulls':64,
              'minThickness':.001,'shrinkWrap':True,'voxelResolution':500000}
    deinstanced=[]
    for path in paths:
        prim=stage.GetPrimAtPath(path)
        while prim.IsInstanceProxy():
            owner=prim.GetParent()
            while owner.IsInstanceProxy():owner=owner.GetParent()
            if not owner.IsInstance():raise ValueError('Cannot author collision instance: '+str(path))
            owner.SetInstanceable(False);deinstanced.append(str(owner.GetPath()))
            prim=stage.GetPrimAtPath(path)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexDecomposition')
        # Schema names/types follow the installed PhysX schema; authoring them
        # with core USD also permits verification without loading the simulator.
        prim.AddAppliedSchema('PhysxConvexDecompositionCollisionAPI')
        for name,value in settings.items():
            kind=Sdf.ValueTypeNames.Bool if type(value) is bool else Sdf.ValueTypeNames.Int if type(value) is int else Sdf.ValueTypeNames.Float
            prim.CreateAttribute('physxConvexDecompositionCollision:'+name,kind,custom=False).Set(value)
    for path in paths:
        if UsdPhysics.MeshCollisionAPI(stage.GetPrimAtPath(path)).GetApproximationAttr().Get()!='convexDecomposition':
            raise ValueError('Collision decomposition override did not compose: '+str(path))
    return {'schema_version':1,'mesh_count':len(paths),'approximation':'convexDecomposition',
            'settings':settings,'mesh_paths':list(map(str,paths)),
            'deinstanced_owners':sorted(set(deinstanced)),
            'provenance':'Explicit existing URDF collision mesh approximation; source geometry and SRDF exclusions unchanged'}

def import_robot(urdf,output):
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Usd,UsdGeom,UsdPhysics
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    imported=Path(URDFImporter(URDFImporterConfig(urdf_path=str(urdf),usd_path=str(output.parent/'robot_generated'),
                        fix_base=True,merge_fixed_joints=False,merge_mesh=False,
                        collision_type='Convex Decomposition',allow_self_collision=True)).import_urdf())
    stage=Usd.Stage.Open(str(imported))
    stage.GetDefaultPrim().GetVariantSet('Physics').SetVariantSelection('physx')
    collision_contract=configure_robot_collision_meshes(stage)
    stage.GetRootLayer().Save()
    import json
    (output.parent/'robot-collision-contract.json').write_text(json.dumps(collision_contract,indent=2)+'\n')
    print('PLATFORM_COLLISION_APPROXIMATION '+json.dumps(collision_contract),flush=True)
    expected={j.get('name') for j in ET.parse(urdf).getroot().findall("joint[@type='revolute']")}
    actual={p.GetName() for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)}
    if expected!=actual: raise RuntimeError('Imported articulation joints differ from URDF')
    result=Usd.Stage.CreateNew(str(output)); robot=result.DefinePrim('/robot','Xform')
    robot.GetReferences().AddReference(os.path.relpath(imported,output.parent))
    robot.GetVariantSet('Physics').SetVariantSelection('physx')
    result.SetDefaultPrim(robot);UsdGeom.SetStageMetersPerUnit(result,1.);UsdGeom.SetStageUpAxis(result,UsdGeom.Tokens.z)
    result.GetRootLayer().Save()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    import charset_normalizer
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True})
    exit_code=0
    try:
        import_robot(args.urdf.resolve(),args.output.resolve())
        import json
        assets=args.output.resolve().parent
        bundle=json.loads((assets/'bundle.json').read_text())
        if bundle['selection']['execution_mode']=='simulation' and bundle['selection']['process'] in ('spray','weld'):
            from prepare_h_steel import prepare
            scene=json.loads((assets/'scene.json').read_text())
            prepare(Path(__file__).resolve().parents[1],assets,translation=scene['h_steel_translation'])
            if bundle['selection']['process']=='weld':
                from platform_weld_fixture import write_fixture
                write_fixture(assets)
        print('PLATFORM_IMPORT_OK',flush=True)
    except Exception:
        # SimulationApp.close uses os._exit; preserve failures before shutdown
        # instead of allowing its default successful exit to hide the error.
        import traceback
        traceback.print_exc()
        exit_code=1
    finally:
        app.close(exit_code=exit_code)
    raise SystemExit(exit_code)
