"""Actual USD bindings preserve configured pair friction and one compliant surface."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))


def test_wall_and_roller_use_configured_pair_friction_without_second_spring():
    pytest.importorskip('pxr.Usd')
    from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Sdf
    import rb20_painting_scene as scene
    stage=Usd.Stage.CreateInMemory()
    roller=UsdGeom.Cylinder.Define(stage,'/World/Robot/ft/collisions/roller').GetPrim()
    wall=UsdGeom.Cube.Define(stage,'/World/TaskSurface').GetPrim()
    UsdPhysics.CollisionAPI.Apply(roller);UsdPhysics.CollisionAPI.Apply(wall)
    old=UsdShade.Material.Define(stage,'/World/DefaultWall')
    old_api=UsdPhysics.MaterialAPI.Apply(old.GetPrim());old_api.CreateDynamicFrictionAttr(1.)
    UsdShade.MaterialBindingAPI.Apply(wall).Bind(old,UsdShade.Tokens.strongerThanDescendants,'physics')
    friction=dict(static_friction=.4,dynamic_friction=.3,restitution=0.)
    existing=UsdShade.Material.Define(stage,'/World/RollerContactMaterial')
    api=UsdPhysics.MaterialAPI.Apply(existing.GetPrim())
    api.CreateStaticFrictionAttr(.4);api.CreateDynamicFrictionAttr(.3);api.CreateRestitutionAttr(0.)
    existing.GetPrim().CreateAttribute('physxMaterial:compliantContactStiffness',Sdf.ValueTypeNames.Float).Set(5000.)
    UsdShade.MaterialBindingAPI.Apply(roller).Bind(existing,UsdShade.Tokens.strongerThanDescendants,'physics')
    report=scene.configure_paint_wall_material(stage,'/World/TaskSurface',friction,
        material_path='/World/RollerContactMaterial_Wall')
    bound=[]
    for prim in (roller,wall):
        material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
        api=UsdPhysics.MaterialAPI(material.GetPrim())
        assert api.GetStaticFrictionAttr().Get()==pytest.approx(.4)
        assert api.GetDynamicFrictionAttr().Get()==pytest.approx(.3)
        assert api.GetRestitutionAttr().Get()==0.
        bound.append(material.GetPrim())
    assert bound[0]!=bound[1]
    assert bound[0].GetAttribute('physxMaterial:compliantContactStiffness').Get()==5000.
    assert not bound[1].GetAttribute('physxMaterial:compliantContactStiffness').HasAuthoredValueOpinion()
    assert report['friction']==friction
    assert report['contact_model']=='fixed_cylinder_sliding_approximation'
    assert report['wall_material']=='/World/RollerContactMaterial_Wall'
    assert report['effective_pair_dynamic_friction']==pytest.approx(.3)


@pytest.mark.parametrize('friction',[None,dict(static_friction=.4,dynamic_friction=.3,restitution=0.)])
def test_measured_surface_material_is_optional_and_does_not_change_geometry(friction):
    pytest.importorskip('pxr.Usd')
    from pxr import Usd,UsdGeom,UsdPhysics,UsdShade
    from platform_task_scene import apply_task_surface
    stage=Usd.Stage.CreateInMemory()
    payload=dict(surface_id='observed',revision='r1',frame_id='World',
                 vertices_m=[[.5,-1.,.5],[.7,-1.,.5],[.7,-1.,.8]],faces=[[0,1,2]])
    ack=apply_task_surface(stage,payload,expected_frame='World',friction=friction)
    assert ack['success'] and ack['revision']=='r1'
    mesh=UsdGeom.Mesh(stage.GetPrimAtPath('/World/MeasuredTaskSurface'))
    import numpy as np
    assert np.allclose(mesh.GetPointsAttr().Get(),payload['vertices_m'])
    assert list(mesh.GetFaceVertexIndicesAttr().Get())==[0,1,2]
    assert UsdPhysics.MeshCollisionAPI(mesh.GetPrim()).GetApproximationAttr().Get()=='none'
    material,_=UsdShade.MaterialBindingAPI(mesh.GetPrim()).ComputeBoundMaterial('physics')
    if friction is None:
        assert not material and 'contact_material' not in ack
    else:
        assert UsdPhysics.MaterialAPI(material.GetPrim()).GetDynamicFrictionAttr().Get()==pytest.approx(.3)
        assert not material.GetPrim().GetAttribute('physxMaterial:compliantContactStiffness').HasAuthoredValueOpinion()
        assert ack['contact_material']['friction']==friction
        assert ack['contact_material']['contact_model']=='fixed_cylinder_sliding_approximation'


def test_measured_surface_frame_rejection_does_not_author_material():
    pytest.importorskip('pxr.Usd')
    from pxr import Usd
    from platform_task_scene import apply_task_surface
    stage=Usd.Stage.CreateInMemory()
    ack=apply_task_surface(stage,dict(surface_id='observed',revision='r',frame_id='wrong'),
                           expected_frame='World',friction=dict(static_friction=.4,dynamic_friction=.3,restitution=0.))
    assert not ack['success'] and 'frame' in ack['error']
    assert not stage.GetPrimAtPath('/World/MeasuredTaskSurface')
    assert not stage.GetPrimAtPath('/World/MeasuredPaintMaterial')
