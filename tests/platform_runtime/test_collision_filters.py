"""SRDF filtering must never broaden robot/environment collision exclusions."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))


def description(names):
    robot=ET.Element('robot',name='test')
    for name in names:
        link=ET.SubElement(robot,'link',name=name)
        if name!='tcp':ET.SubElement(link,'collision')
    return robot


def fixture(tmp_path):
    paths={}
    for key,names in [('urdf',['base','wrist','ft','roller','camera','tcp','other_arm']),
                      ('physics_urdf',['base','wrist','ft','other_arm'])]:
        path=tmp_path/(key+'.xml');ET.ElementTree(description(names)).write(path);paths[key]=str(path)
    srdf=ET.Element('robot',name='test')
    for a,b in [('base','wrist'),('wrist','roller'),('wrist','camera'),('roller','camera'),('camera','tcp')]:
        ET.SubElement(srdf,'disable_collisions',link1=a,link2=b,reason='Adjacent')
    path=tmp_path/'robot.srdf';ET.ElementTree(srdf).write(path)
    return dict(paths,srdf=str(path),ft_boundaries={'left':{'child':'ft','downstream_links':['ft','roller','camera','tcp']}})


def test_mapping_coalesces_reduced_bodies_and_preserves_exact_pairs(tmp_path):
    from platform_collision import collision_filter_plan
    plan=collision_filter_plan(fixture(tmp_path))
    assert plan['pairs']==[['base','wrist'],['ft','wrist']]
    assert plan['source_pair_count']==5
    assert plan['coalesced_same_body_count']==2
    assert plan['duplicate_mapped_pair_count']==1
    assert plan['mapped_pair_count']==2
    assert all('other_arm' not in p for p in plan['pairs'])


@pytest.mark.parametrize('fault',['unknown_link','missing_physics_link','invalid_reduction','ambiguous_reduction'])
def test_invalid_mapping_cannot_expand_filters(tmp_path,fault):
    from platform_collision import collision_filter_plan
    bundle=fixture(tmp_path)
    if fault=='unknown_link':
        text=Path(bundle['srdf']).read_text().replace('link1="base"','link1="task_surface"');Path(bundle['srdf']).write_text(text)
    elif fault=='missing_physics_link':
        root=ET.parse(bundle['physics_urdf']);root.getroot().remove(root.find("link[@name='wrist']"));root.write(bundle['physics_urdf'])
    elif fault=='invalid_reduction':bundle['ft_boundaries']['left']['child']='task_surface'
    else:bundle['ft_boundaries']['right']={'child':'wrist','downstream_links':['roller']}
    with pytest.raises(ValueError):collision_filter_plan(bundle)


def test_collisionless_pairs_need_no_physics_body(tmp_path):
    from platform_collision import collision_filter_plan
    bundle=fixture(tmp_path)
    # A native TCP frame has no collision and need not have a USD rigid body.
    full=ET.parse(bundle['urdf']);ET.SubElement(full.getroot(),'link',name='sensor_optical');full.write(bundle['urdf'])
    physical=ET.parse(bundle['physics_urdf']);ET.SubElement(physical.getroot(),'link',name='sensor_optical');physical.write(bundle['physics_urdf'])
    srdf=ET.parse(bundle['srdf']);ET.SubElement(srdf.getroot(),'disable_collisions',link1='base',link2='sensor_optical');srdf.write(bundle['srdf'])
    plan=collision_filter_plan(bundle)
    assert plan['collisionless_pair_count']==1
    assert plan['mapped_pair_count']==2


def stage_with_bodies():
    from pxr import Usd,UsdGeom,UsdPhysics
    stage=Usd.Stage.CreateInMemory()
    for name in ('base','wrist','ft','other_arm'):
        prim=UsdGeom.Xform.Define(stage,'/World/Robot/'+name).GetPrim();UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,'/World/TaskSurface').GetPrim())
    return stage


def test_usd_adapter_authors_only_exact_robot_pairs_and_is_idempotent(tmp_path):
    pytest.importorskip('pxr.UsdPhysics')
    from pxr import UsdPhysics
    from platform_collision import apply_srdf_collision_filters
    stage=stage_with_bodies();bundle=fixture(tmp_path)
    original=UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath('/World/Robot/base')).CreateFilteredPairsRel()
    original.AddTarget('/World/ExistingFilter')
    report=apply_srdf_collision_filters(stage,bundle)
    assert report['applied_pair_count']==2
    assert report['rigid_body_count']==4
    assert report['srdf_sha256']
    assert list(map(str,original.GetTargets()))==['/World/ExistingFilter','/World/Robot/wrist']
    ft=UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath('/World/Robot/ft'))
    assert list(map(str,ft.GetFilteredPairsRel().GetTargets()))==['/World/Robot/wrist']
    assert not stage.GetPrimAtPath('/World/Robot/other_arm').HasAPI(UsdPhysics.FilteredPairsAPI)
    assert not stage.GetPrimAtPath('/World/TaskSurface').HasAPI(UsdPhysics.FilteredPairsAPI)
    apply_srdf_collision_filters(stage,bundle)
    assert len(original.GetTargets())==2


@pytest.mark.parametrize('fault',['missing','ambiguous'])
def test_usd_adapter_validates_all_bodies_before_any_mutation(tmp_path,fault):
    pytest.importorskip('pxr.UsdPhysics')
    from pxr import UsdGeom,UsdPhysics
    from platform_collision import apply_srdf_collision_filters
    stage=stage_with_bodies()
    if fault=='missing':stage.RemovePrim('/World/Robot/wrist')
    else:UsdPhysics.RigidBodyAPI.Apply(UsdGeom.Xform.Define(stage,'/World/Robot/nested/wrist').GetPrim())
    with pytest.raises(ValueError):apply_srdf_collision_filters(stage,fixture(tmp_path))
    assert not stage.GetPrimAtPath('/World/Robot/base').HasAPI(UsdPhysics.FilteredPairsAPI)
