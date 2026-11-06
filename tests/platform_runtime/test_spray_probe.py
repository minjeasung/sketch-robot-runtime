from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))


def cad():
    return {'frame_id':'World','meshes':[{'prim':'/World/HSteel/IfcPlate_PLATE/mesh','vertices':[[-.5,-.5,0],[.5,-.5,0],[.5,.5,0],[-.5,.5,0]],'triangles':[[0,1,2],[0,2,3]]}]}


def test_roi_is_inside_real_visible_cad_triangle_and_has_provenance():
    from check_platform_spray import cad_fixture
    camera=np.diag([1.,-1.,-1.,1.]);camera[:3,3]=[0,0,2]
    fixture=cad_fixture(cad(),camera,[[400,0,320],[0,400,240],[0,0,1]],640,480)
    assert fixture['source']=='simulation_cad_fixture' and fixture['recognition'] is False
    assert fixture['source_prim']==cad()['meshes'][0]['prim']
    assert fixture['normal']==[0.,0.,1.]
    assert np.allclose(np.asarray(fixture['corners'])[:,2],0.)
    assert len(fixture['pixels'])==4 and fixture['geometry_sha256']


def test_hidden_cad_face_cannot_be_substituted_with_fabricated_roi():
    from check_platform_spray import cad_fixture
    camera=np.eye(4);camera[:3,3]=[0,0,-2]
    with pytest.raises(ValueError,match='visible'):
        cad_fixture(cad(),camera,[[400,0,320],[0,400,240],[0,0,1]],640,480)


def test_read_only_never_authorizes_public_workflow_mutation():
    from check_platform_spray import authorize
    for topic in ('/target_selection_pixels','/painting_system/zed_sketch_request','/sketch_execute'):
        with pytest.raises(ValueError,match='execute'):authorize(topic,False)
        authorize(topic,True)


def test_workflow_publish_waits_for_intended_consumer_not_invalidation_only():
    from check_platform_spray import workflow_consumers_ready
    assert not workflow_consumers_ready('/target_selection_pixels',{'moveit_executor'},1)
    assert not workflow_consumers_ready('/target_selection_pixels',{'moveit_executor','target_selector'},1)
    assert workflow_consumers_ready('/target_selection_pixels',{'moveit_executor','target_selector'},2)
    assert not workflow_consumers_ready('/work_area_pixels',{'moveit_executor','wall_projector'},2)
    assert workflow_consumers_ready('/work_area_pixels',{'wall_projector','sketch_to_waypoints'},2)


def plan():
    return {'process_mode':'spray','frame_id':'World','path_id':'p','plan_hash':'h',
            'work_area_id':'a','plane_generation_id':'g','point_semantics':'surface_point',
            'spray_tool_axis':'+z','spray_endpoint_tcp_m':[0,0,.1],
            'rows':[dict(mode='SPRAY',x=x,y=0.,z=0.,nx=0.,ny=0.,nz=1.,offset_m=.5) for x in (0.,.1)]}


def feedback():
    return [dict(stamp=1.+i*.1,xyz=[i*.01,0.,.6],quaternion=[1.,0.,0.,0.]) for i in range(11)]


def test_measured_tip_tracking_applies_endpoint_once_and_requires_coverage():
    from check_platform_spray import tracking
    result=tracking(plan(),feedback())
    import json
    json.dumps(result)
    assert result['passed'] and result['max_endpoint_error_m']<1e-9 and result['coverage']>.99
    assert not tracking(plan(),feedback()[:3])['passed']
    wrong=feedback()
    for item in wrong:item['xyz'][2]+=.1
    assert not tracking(plan(),wrong)['passed']


def test_plan_gate_requires_fresh_exact_identity_and_validation():
    from check_platform_spray import ready_plan
    p=plan();status={**p,'ready':True,'checks':{'current_plan_validated':True}}
    assert ready_plan(status,p,age=.1)
    for stale in ({**status,'plan_hash':'old'},{**status,'ready':False},{**status,'checks':{}}):
        assert not ready_plan(stale,p,age=.1)
    assert not ready_plan(status,p,age=2.)


def test_roi_merges_coplanar_cad_triangles_to_retain_extraction_support():
    from check_platform_spray import cad_fixture
    camera=np.diag([1.,-1.,-1.,1.]);camera[:3,3]=[0,0,2]
    fixture=cad_fixture(cad(),camera,[[400,0,320],[0,400,240],[0,0,1]],640,480)
    pixels=np.asarray(fixture['pixels']);area=abs(np.dot(pixels[:,0],np.roll(pixels[:,1],1))-np.dot(pixels[:,1],np.roll(pixels[:,0],1)))*.5
    assert area>=1600 and fixture['triangle_indices']==[0,1]


def test_sparse_tcp_samples_do_not_claim_continuous_tracking():
    from check_platform_spray import tracking
    samples=feedback()
    for index,item in enumerate(samples):item['stamp']=float(index)
    assert not tracking(plan(),samples)['passed']


@pytest.mark.parametrize('side,expected_sign',[('right',1.)])
def test_simulated_spray_camera_exposes_selected_arm_side_of_actual_cad(tmp_path,side,expected_sign):
    import json
    from prepare_platform_assets import generate_urdf_assets
    from sketch_control.runtime_selection import normalize_selection
    from check_platform_spray import cad_fixture
    generate_urdf_assets(ROOT,normalize_selection(dict(platform='commercial',active_arm=side,process='spray')),tmp_path)
    scene=json.loads((tmp_path/'scene.json').read_text())
    camera=np.asarray(scene['zed_world_optical'])
    assert camera[0,3]*expected_sign>0
    geometry=json.loads((ROOT/'tests/platform_runtime/fixtures/h_steel_spray_column.json').read_text())
    for mesh in geometry['meshes']:
        mesh['vertices']=(np.asarray(mesh['vertices'])+np.asarray(scene['h_steel_translation'])-[-.515,-1.7,0.]).tolist()
    fixture=cad_fixture(geometry,camera,[432,0,320,0,432,240,0,0,1],640,480)
    assert np.asarray(fixture['normal'])[0]*expected_sign>.99
    assert fixture['source_prim']=='/World/HSteel/IfcColumn_COLUMN/_2_750'
    assert fixture['triangle_indices']==([6,28] if side=='left' else [0,22])


def test_robot_visibility_checks_interior_and_measured_joint_pose():
    import xml.etree.ElementTree as ET
    from platform_visibility import robot_occluders,patch_visibility
    robot=ET.fromstring('''<robot name="visibility"><link name="base"/><link name="arm">
    <visual><geometry><box size=".1 .1 .1"/></geometry></visual>
    <collision><geometry><box size=".1 .1 .1"/></geometry></collision></link>
    <joint name="slide" type="prismatic"><parent link="base"/><child link="arm"/>
    <origin xyz="0 0 1"/><axis xyz="1 0 0"/></joint></robot>''')
    corners=[[-.4,-.4,0],[.4,-.4,0],[.4,.4,0],[-.4,.4,0]]
    with pytest.raises(ValueError,match='measured'):robot_occluders(robot,{})
    result=patch_visibility(corners,[0,0,2],robot_occluders(robot,{'slide':0.}))
    assert result['blocked_rays']>0 and result['occluders']==['arm/collision','arm/visual']
    assert patch_visibility(corners,[0,0,2],robot_occluders(robot,{'slide':1.}))['clear']


def test_cad_selection_rejects_robot_occluded_roi():
    import trimesh
    from check_platform_spray import cad_fixture
    obstacle=trimesh.creation.box(extents=[2.,2.,.1]);obstacle.apply_translation([0,0,1.])
    camera=np.diag([1.,-1.,-1.,1.]);camera[:3,3]=[0,0,2]
    with pytest.raises(ValueError,match='visible'):
        cad_fixture(cad(),camera,[400,0,320,0,400,240,0,0,1],640,480,
                    robot_geometry=[('arm/visual',obstacle.bounds,obstacle.triangles)])


def test_left_spray_head_mount_sees_horizontal_member_top_at_demo_ready_pose(tmp_path):
    import json
    import xml.etree.ElementTree as ET
    from prepare_platform_assets import generate_urdf_assets
    from sketch_control.runtime_selection import normalize_selection
    from platform_visibility import robot_occluders,patch_visibility
    bundle=generate_urdf_assets(ROOT,normalize_selection(dict(platform='commercial',active_arm='left',process='spray')),tmp_path)
    obstacles=robot_occluders(ET.parse(tmp_path/'robot.urdf').getroot(),bundle['initial_positions'])
    # Rays start beyond the camera near clip, so the owning sensor housing is excluded.
    obstacles=[x for x in obstacles if not x[0].startswith(('zed2i_link/','zed_bracket_link/'))]
    scene=json.loads((tmp_path/'scene.json').read_text());camera=np.asarray(scene['zed_world_optical'])
    # Original H-steel horizontal member upper flange, 20 mm inset, after the recorded rigid placement.
    # The current demo fixture is 0.60 m farther along -World Y.
    corners=np.array([[-.27200491,-1.57999995,1.016],[.27200488,-1.57999995,1.016],
                      [.27200488,-1.41999994,1.016],[-.27200491,-1.41999994,1.016]])
    assert scene['zed_mount']['parent_link']=='zed2i_link'
    assert patch_visibility(corners,camera[:3,3],obstacles)['clear']
    optical=(np.linalg.inv(camera)@np.column_stack((corners,np.ones(4))).T).T[:,:3]
    assert np.all(optical[:,2]>.1)
    pixels=optical[:,:2]/optical[:,2,None]*444.444+[320,240]
    assert np.all((pixels>[0,0])&(pixels<[640,480]))


def test_empty_segment_invalidation_clears_old_plan_without_crashing():
    from check_platform_spray import receive_json
    seen={'segments':plan()};received={'segments':10.};events=[];diagnostics=[]
    assert not receive_json('segments','',seen,received,events,diagnostics,11.)
    assert seen['segments']=={} and received.get('segments',0)==0
    assert diagnostics[-1]['kind']=='cleared' and not events
    assert not ready_plan_for_test(seen)


def ready_plan_for_test(seen):
    from check_platform_spray import ready_plan
    return ready_plan({**plan(),'ready':True,'checks':{'current_plan_validated':True}},seen['segments'],.1)


@pytest.mark.parametrize('payload',['mode,x,y,z\nSPRAY,1,2,3','not json','[]','null','true','{"ready":NaN}'])
def test_non_json_or_non_object_status_never_retains_previous_readiness(payload):
    from check_platform_spray import receive_json
    seen={'readiness':{'ready':True}};received={'readiness':10.};diagnostics=[]
    assert not receive_json('readiness',payload,seen,received,[],diagnostics,11.)
    assert seen['readiness']=={} and received.get('readiness',0)==0
    assert diagnostics[-1]['kind']=='invalid'


def test_profile_rejection_detail_survives_segment_clear_for_report():
    from check_platform_spray import receive_json,plan_rejection
    seen={};received={};events=[];diagnostics=[]
    receive_json('segments','',seen,received,events,diagnostics,10.)
    payload='{"state":"rejected","reason":"SPRAY_EOAT_PROFILE_INVALID","detail":"No module named trimesh"}'
    assert receive_json('plan',payload,seen,received,events,diagnostics,10.1)
    reason=plan_rejection(seen,received,10.)
    assert 'SPRAY_EOAT_PROFILE_INVALID' in reason and 'trimesh' in reason
    assert plan_rejection(seen,received,11.)==''


@pytest.mark.parametrize('side,mode,expected', [
    ('left','simulation',[-.515,-1.9,0.]),
    ('right','simulation',[-.515,-1.7,0.]),
    ('left','mixed',[-.515,-1.7,0.]),
])
def test_left_spray_fixture_placement_is_scoped_rigid_translation(tmp_path,side,mode,expected):
    import json
    from prepare_platform_assets import generate_urdf_assets
    from sketch_control.runtime_selection import normalize_selection
    generate_urdf_assets(ROOT,normalize_selection(dict(platform='commercial',active_arm=side,
                         process='spray',execution_mode=mode)),tmp_path)
    scene=json.loads((tmp_path/'scene.json').read_text())
    assert scene['h_steel_translation']==expected
    if side=='left' and mode=='simulation':
        assert scene['h_steel_placement_source']=='simulation_fixture_rigid_translation'
        assert scene['h_steel_placement_delta_m']==[0.,-.2,0.]


def test_rb10_spray_camera_selects_visible_reachable_height_on_actual_column(tmp_path):
    import json
    import xml.etree.ElementTree as ET
    from prepare_platform_assets import generate_urdf_assets
    from sketch_control.runtime_selection import normalize_selection
    from platform_visibility import robot_occluders
    from check_platform_spray import cad_fixture
    bundle=generate_urdf_assets(ROOT,normalize_selection(dict(platform='rb10',active_arm='single',process='spray')),tmp_path)
    scene=json.loads((tmp_path/'scene.json').read_text());camera=np.asarray(scene['zed_world_optical'])
    assert camera[:3,3].tolist()==[.2,-1.4,.5]
    assert scene['h_steel_translation']==[.5,-.4,0.]
    geometry=json.loads((ROOT/'tests/platform_runtime/fixtures/h_steel_spray_column.json').read_text())
    for mesh in geometry['meshes']:
        mesh['vertices']=(np.asarray(mesh['vertices'])+np.asarray(scene['h_steel_translation'])-[-.515,-1.7,0.]).tolist()
    positions={'base':.0010476602474227548,'shoulder':-.9350494742393494,'elbow':2.4254813194274902,
               'wrist1':-1.6288317441940308,'wrist2':1.5675837993621826,'wrist3':-.00030892775976099074}
    obstacles=robot_occluders(ET.parse(tmp_path/'robot.urdf').getroot(),positions)
    fixture=cad_fixture(geometry,camera,[444.444,0,320,0,444.444,240,0,0,1],640,480,robot_geometry=obstacles)
    assert fixture['triangle_indices']==[3,25]
    assert np.allclose(fixture['point'],[.62408,-.004,.31])
    assert np.allclose(fixture['normal'],[0,-1,0])
    assert fixture['robot_visibility']['blocked_rays']==0
