"""Saved robot collision meshes must not silently retain importer convex hulls."""
import importlib.util
from pathlib import Path
import pytest

ROOT=Path(__file__).resolve().parents[2]


def module():
    spec=importlib.util.spec_from_file_location('platform_import_collision',ROOT/'scripts/import_platform_usd.py')
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result


def usd():
    pytest.importorskip('pxr.Usd');from pxr import Usd,UsdGeom,UsdPhysics
    return Usd,UsdGeom,UsdPhysics


def test_instanced_collision_override_persists_without_changing_source_or_visuals(tmp_path):
    Usd,Geom,Physics=usd()
    source=Usd.Stage.CreateNew(str(tmp_path/'source.usda'));root=Geom.Xform.Define(source,'/Part').GetPrim();source.SetDefaultPrim(root)
    mesh=Geom.Mesh.Define(source,'/Part/Collision');mesh.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0)]);mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2])
    Physics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
    Physics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
    source.GetRootLayer().Save();original=(tmp_path/'source.usda').read_bytes()
    stage=Usd.Stage.CreateNew(str(tmp_path/'robot.usda'));robot=Geom.Xform.Define(stage,'/Robot').GetPrim();stage.SetDefaultPrim(robot)
    owner=Geom.Xform.Define(stage,'/Robot/Collider').GetPrim();owner.GetReferences().AddReference('source.usda');owner.SetInstanceable(True)
    visual=Geom.Mesh.Define(stage,'/Robot/Visual').GetPrim()
    outside=Geom.Mesh.Define(stage,'/Outside').GetPrim();Physics.CollisionAPI.Apply(outside);Physics.MeshCollisionAPI.Apply(outside).CreateApproximationAttr('convexHull')
    Physics.FilteredPairsAPI.Apply(robot).CreateFilteredPairsRel().AddTarget('/Outside')
    result=module().configure_robot_collision_meshes(stage)
    assert result['mesh_count']==1
    stage.GetRootLayer().Save();reopened=Usd.Stage.Open(str(tmp_path/'robot.usda'))
    collider=reopened.GetPrimAtPath('/Robot/Collider/Collision')
    assert not collider.IsInstanceProxy()
    assert Physics.MeshCollisionAPI(collider).GetApproximationAttr().Get()=='convexDecomposition'
    assert Physics.CollisionAPI(collider).GetCollisionEnabledAttr().Get() is True
    assert collider.GetAttribute('physxConvexDecompositionCollision:errorPercentage').Get()==1.
    assert collider.GetAttribute('physxConvexDecompositionCollision:maxConvexHulls').Get()==64
    assert collider.GetAttribute('physxConvexDecompositionCollision:shrinkWrap').Get() is True
    assert collider.GetAttribute('physxConvexDecompositionCollision:voxelResolution').Get()==500000
    assert not reopened.GetPrimAtPath('/Robot/Visual').HasAPI(Physics.CollisionAPI)
    assert Physics.MeshCollisionAPI(reopened.GetPrimAtPath('/Outside')).GetApproximationAttr().Get()=='convexHull'
    assert list(Physics.FilteredPairsAPI(reopened.GetPrimAtPath('/Robot')).GetFilteredPairsRel().GetTargets())==['/Outside']
    assert (tmp_path/'source.usda').read_bytes()==original
    assert module().configure_robot_collision_meshes(reopened)['mesh_count']==1


def test_empty_robot_collision_import_fails_instead_of_claiming_decomposition():
    Usd,Geom,_=usd();stage=Usd.Stage.CreateInMemory();stage.SetDefaultPrim(Geom.Xform.Define(stage,'/Robot').GetPrim())
    with pytest.raises(ValueError,match='collision'):module().configure_robot_collision_meshes(stage)


def test_actual_generated_robot_colliders_are_explicit_after_saved_override(tmp_path):
    Usd,Geom,Physics=usd();path=ROOT/'.runtime/platform/matrix-noncontact-final/commercial-right-spray/robot.usd'
    if not path.exists():pytest.skip('local actual imported robot audit artifact unavailable')
    stage=Usd.Stage.CreateNew(str(tmp_path/'actual.usda'));robot=Geom.Xform.Define(stage,'/Robot').GetPrim();robot.GetReferences().AddReference(str(path));stage.SetDefaultPrim(robot)
    before=[p.GetPath() for p in Usd.PrimRange(robot,Usd.TraverseInstanceProxies()) if p.IsA(Geom.Mesh) and p.HasAPI(Physics.CollisionAPI)]
    assert len(before)==25
    report=module().configure_robot_collision_meshes(stage);stage.GetRootLayer().Save()
    actual=Usd.Stage.Open(str(tmp_path/'actual.usda'))
    assert report['mesh_count']==len(before)
    for path in before:
        prim=actual.GetPrimAtPath(path)
        assert Physics.MeshCollisionAPI(prim).GetApproximationAttr().Get()=='convexDecomposition'
        assert Physics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is True


def test_import_robot_persists_collision_contract_in_referenced_physics_usd(monkeypatch,tmp_path):
    """Exercise the production import flow using a real USD and a fake converter."""
    import json
    import sys
    from types import SimpleNamespace
    Usd,Geom,Physics=usd()
    imported=tmp_path/'converted.usda'
    stage=Usd.Stage.CreateNew(str(imported));root=Geom.Xform.Define(stage,'/Robot').GetPrim();stage.SetDefaultPrim(root)
    mesh=Geom.Mesh.Define(stage,'/Robot/Collision').GetPrim()
    Physics.CollisionAPI.Apply(mesh).CreateCollisionEnabledAttr(True)
    Physics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr('convexHull')
    Physics.RevoluteJoint.Define(stage,'/Robot/joint0')
    stage.GetRootLayer().Save()
    class Importer:
        def __init__(self,config):self.config=config
        def import_urdf(self):return str(imported)
    monkeypatch.setitem(sys.modules,'isaacsim.core.utils.extensions',SimpleNamespace(enable_extension=lambda name:None))
    monkeypatch.setitem(sys.modules,'isaacsim.asset.importer.urdf',SimpleNamespace(URDFImporter=Importer,URDFImporterConfig=SimpleNamespace))
    urdf=tmp_path/'robot.urdf';urdf.write_text('<robot name="test"><joint name="joint0" type="revolute"/></robot>')
    output=tmp_path/'robot.usd';module().import_robot(urdf,output)
    actual=Usd.Stage.Open(str(output));collider=actual.GetPrimAtPath('/robot/Collision')
    assert Physics.MeshCollisionAPI(collider).GetApproximationAttr().Get()=='convexDecomposition'
    assert Physics.CollisionAPI(collider).GetCollisionEnabledAttr().Get() is True
    contract=json.loads((tmp_path/'robot-collision-contract.json').read_text())
    assert contract['mesh_count']==1
    assert contract['approximation']=='convexDecomposition'
