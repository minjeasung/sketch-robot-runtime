import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]

def module():
    spec=importlib.util.spec_from_file_location('cad_fixture',ROOT/'scripts/platform_weld_fixture.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def geometry():return json.loads((Path(__file__).parent/'fixtures/h_steel_weld_bead.json').read_text())

def test_actual_cad_bead_seam_is_on_exposed_sloping_face_with_cad_normal():
    data=geometry();m=module();part=data['meshes'][0]
    # Observe the actual sloping CAD face directly, excluding any radial-up heuristic.
    center=np.asarray(part['vertices']).mean(0);normal=np.array([np.sqrt(3)/2,0.,-.5])
    result=m.generate_fixture(data,center+normal,scene_revision='r',frame_id='World')
    seam=result['seams'][0];points=np.array(seam['points_xyz']);up=np.array(seam['reference_up_vectors'][0])
    assert np.dot(up,normal)>.99999
    assert np.max(np.linalg.norm(np.diff(points,axis=0),axis=1))<=.02000000001
    assert np.linalg.norm(points[-1]-points[0])==pytest.approx(.2,abs=1e-6)
    triangles=np.array(part['vertices'])[np.array(part['triangles'])]
    face=seam['provenance']['triangle_indices']
    assert set(face)=={2,8}
    assert np.max(abs((points-triangles[face[0],0])@up))<1e-7
    assert abs(np.dot(points[0]-center,up))>.001  # Surface, not volume centerline.
    assert result['provenance']['source']=='simulation_cad_fixture'
    assert seam['provenance']['tcp_offset_applied_m']==0.


def test_fixture_is_rigid_transform_equivariant():
    m=module();data=geometry();view=np.array([1.,-1.3,.5]);original=m.generate_fixture(data,view,scene_revision='r',frame_id='World')['seams'][0]
    rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]]);offset=np.array([3.,2.,1.])
    for mesh in data['meshes']:mesh['vertices']=(np.array(mesh['vertices'])@rotation.T+offset).tolist()
    changed=m.generate_fixture(data,rotation@view+offset,scene_revision='r',frame_id='World')['seams'][0]
    assert np.allclose(changed['reference_up_vectors'],np.array(original['reference_up_vectors'])@rotation.T)
    assert np.allclose(changed['points_xyz'],np.array(original['points_xyz'])@rotation.T+offset)


def test_occluded_cad_face_is_not_selected():
    m=module();data=geometry();bead=np.asarray(data['meshes'][0]['vertices']);center=bead.mean(0);normal=np.array([np.sqrt(3)/2,0.,-.5]);view=center+normal
    # Enclose the view point behind a plane perpendicular to the viewing direction.
    side=np.array([0.,1.,0.]);cross=np.cross(normal,side);c=center+normal*.1
    vertices=[c+side*a+cross*b for a,b in [(-2,-2),(2,-2),(2,2),(-2,2)]]
    data['meshes'].append({'prim':'occluding_plate','vertices':[p.tolist() for p in vertices],'triangles':[[0,1,2],[0,2,3]]})
    with pytest.raises(ValueError,match='visible|occluded'):m.generate_fixture(data,view,scene_revision='r',frame_id='World')


def test_ambiguous_face_direction_requires_explicit_orientation():
    m=module();data=geometry();v=np.array(data['meshes'][0]['vertices']);center=v.mean(0)
    # Exact bisector between sloping and top faces, both equally view-facing.
    view=center+np.array([np.sqrt(3)/2,0.,.5])
    with pytest.raises(ValueError,match='ambiguous|orientation'):m.generate_fixture(data,view,scene_revision='r',frame_id='World',ambiguity_margin=.05)


def test_base_viewpoint_comes_from_selected_first_joint_origin():
    urdf='<robot name="r"><link name="World"/><link name="mount"/><link name="arm"/><joint name="fixed" type="fixed"><parent link="World"/><child link="mount"/><origin xyz="1 2 3"/></joint><joint name="j1" type="revolute"><parent link="mount"/><child link="arm"/><origin xyz="0 .5 0"/></joint></robot>'
    assert np.allclose(module().robot_base_point(urdf,'j1','World'),[1,2.5,3])


def test_fixture_writer_rejects_non_simulation(tmp_path):
    (tmp_path/'bundle.json').write_text(json.dumps({'selection':{'execution_mode':'mixed','process':'weld'}}))
    with pytest.raises(ValueError,match='explicit simulation'):module().write_fixture(tmp_path)


def test_actual_complete_export_selects_only_visible_named_cad_surfaces():
    assets=ROOT/'.runtime/platform/default-spray'
    if not (assets/'h_steel_collision.json').is_file():pytest.skip('full local CAD export unavailable; committed bead fixture tested separately')
    m=module();data=json.loads((assets/'h_steel_collision.json').read_text());bundle=json.loads((assets/'bundle.json').read_text())
    for arm in bundle['arms'].values():
        view=m.robot_base_point((assets/'robot.urdf').read_text(),arm['joints'][0],arm['base_frame'])
        fixture=m.generate_fixture(data,view,scene_revision=bundle['revision'],frame_id=arm['base_frame'])
        assert len(fixture['seams'])==2
        assert len(fixture['provenance']['rejected_features'])==4
        for seam in fixture['seams']:
            normal=np.array(seam['reference_up_vectors'][0]);points=np.array(seam['points_xyz'])
            assert normal[2]>.999
            assert np.max(np.linalg.norm(np.diff(points,axis=0),axis=1))<=.02+1e-10


def test_writer_records_source_and_target_frame_without_tool_offset(tmp_path):
    m=module();data=geometry();view=np.array(data['meshes'][0]['vertices']).mean(0)+[1.,0.,-.5]
    (tmp_path/'h_steel_collision.json').write_text(json.dumps(data))
    (tmp_path/'h_steel_manifest.json').write_text(json.dumps({'source':'Steel Structure.usdc','source_sha256':'source-sha','translation_m':[0,0,0]}))
    (tmp_path/'bundle.json').write_text(json.dumps({'revision':'revision','selection':{'execution_mode':'simulation','process':'weld','active_arm':'left'},'arms':{'left':{'base_frame':'World','joints':['j1']}}}))
    (tmp_path/'robot.urdf').write_text('<robot name="fixture"><link name="World"/><link name="arm"/><joint name="j1" type="revolute"><parent link="World"/><child link="arm"/><origin xyz="'+ ' '.join(map(str,view))+'"/></joint></robot>')
    camera=np.eye(4);camera[:3,3]=view
    (tmp_path/'scene.json').write_text(json.dumps({'zed_world_optical':camera.tolist(),'camera_names':['zed']}))
    # First joint remains below the bead; visibility must use the rendered eye.
    (tmp_path/'robot.urdf').write_text('<robot name="fixture"><link name="World"/><link name="arm"/><joint name="j1" type="revolute"><parent link="World"/><child link="arm"/></joint></robot>')
    written=m.write_fixture(tmp_path);result=json.loads((tmp_path/'weld_fixture.json').read_text())
    assert result['provenance']['viewpoint_source']=='generated_zed_optical_origin'
    assert result['provenance']['viewpoint_xyz']==view.tolist()
    assert result['provenance']['selected_robot_base_xyz']==[0.,0.,0.]
    assert result==written and result['frame_id']=='World' and result['scene_revision']=='revision'
    assert result['provenance']['source_sha256']=='source-sha'
    assert result['provenance']['selected_arm']=='left'
    assert all(s['provenance']['tcp_offset_applied_m']==0. for s in result['seams'])


@pytest.mark.parametrize('bad',[None,[[1,0],[0,1]],[[1,0,0,0],[0,2,0,0],[0,0,1,1],[0,0,0,1]]])
def test_generated_camera_viewpoint_requires_finite_rigid_transform(bad):
    with pytest.raises(ValueError,match='camera'):
        module().camera_viewpoint({'zed_world_optical':bad,'camera_names':['zed']})


def test_actual_rb10_cad_fixture_uses_render_camera_not_ground_level_base(tmp_path):
    import shutil
    assets=ROOT/'.runtime/platform/matrix-noncontact/rb10-single-weld'
    if not (assets/'h_steel_collision.json').is_file():pytest.skip('local full RB10 CAD export unavailable')
    for name in ('bundle.json','scene.json','robot.urdf','h_steel_collision.json','h_steel_manifest.json'):
        shutil.copyfile(assets/name,tmp_path/name)
    m=module();geometry=json.loads((tmp_path/'h_steel_collision.json').read_text())
    with pytest.raises(ValueError,match='No unambiguous visible'):
        m.generate_fixture(geometry,[0,0,0],scene_revision='r',frame_id='link0')
    result=m.write_fixture(tmp_path)
    assert [s['seam_id'] for s in result['seams']]==['IfcFastener_WELD_004']
    assert result['seams'][0]['reference_up_vectors'][0][2]>.999
    assert len(result['provenance']['rejected_features'])==5


def test_horizontal_lower_fillet_follows_front_web_and_lower_flange_intersection():
    data=json.loads((Path(__file__).parent/'fixtures/h_steel_horizontal_beam.json').read_text())
    seam=module().horizontal_lower_fillet(data)
    points=np.asarray(seam['points_xyz'])
    np.testing.assert_allclose(points[[0,-1]],[[.2,-1.496,.629],[-.2,-1.496,.629]],atol=2e-6)
    assert np.max(np.linalg.norm(np.diff(points,axis=0),axis=1))<=.02000001
    from scipy.spatial.transform import Rotation
    axis=Rotation.from_quat(seam['quaternions_xyzw']).apply([0,0,1])
    np.testing.assert_allclose(axis,[[0,-2**-.5,-2**-.5]],atol=1e-7)
    assert seam['provenance']['tool_roll_deg']==90.
    assert seam['provenance']['recognition'] is False
    assert seam['provenance']['path_source']=='web_lower_flange_plane_intersection'
    # The actual CAD surface, not a floating line through the beam volume.
    import trimesh
    mesh=data['meshes'][0]
    surface=trimesh.Trimesh(vertices=mesh['vertices'],faces=mesh['triangles'],process=False)
    _,distance,_=trimesh.proximity.closest_point_naive(surface,points)
    assert distance.max()<1e-5  # CAD tessellation differs by a few micrometres at the corner.
    for mesh in data['meshes']:
        mesh['vertices']=(np.asarray(mesh['vertices'])+[1,2,3]).tolist()
    moved=module().horizontal_lower_fillet(data)
    np.testing.assert_allclose(moved['points_xyz'],points+[1,2,3],atol=1e-6)
