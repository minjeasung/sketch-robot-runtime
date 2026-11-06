"""Simulation-only UI workflow input and evidence contracts."""
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))


def test_cleanup_failure_preserves_original_workflow_exception(tmp_path,monkeypatch):
    from types import SimpleNamespace as NS
    import check_platform_workflow as module
    for name in ('scene.json','objects.yaml'):(tmp_path/name).write_text('{}')
    def original_failure(*args):raise RuntimeError('original D405 target measurement timed out')
    def close_failure():raise FileNotFoundError('simulation-ready.json missing during cleanup')
    observer=NS(mutated=False,plane_evidence=None,free_space_proof=None,latest={},events=[],samples=[],
                wait_healthy=original_failure,close=close_failure)
    monkeypatch.setattr(module,'validate_probe_context',lambda assets,**kwargs:dict(bundle=dict(selection=dict(process='paint'),revision='r'),session_id='s'))
    monkeypatch.setattr(module,'wall_fixture',lambda *args:{})
    monkeypatch.setattr(module,'PlatformObserver',lambda *args:observer)
    report={'passed':False}
    with pytest.raises(RuntimeError,match='original D405 target measurement timed out'):
        module.run(NS(assets=tmp_path,roi_fraction=.3,execute=True,output=tmp_path/'report.json'),report)
    assert report['error']=='original D405 target measurement timed out'
    assert report['cleanup_errors']==[dict(stage='observer_close',error='simulation-ready.json missing during cleanup')]
    assert (tmp_path/'report.trace.json').exists()


def test_cleanup_failure_cannot_leave_successful_report_passed():
    from check_platform_workflow import cleanup_step
    def broken():raise OSError('disk full')
    report={'passed':True}
    cleanup_step(report,'trace_write',broken)
    assert report['passed'] is False
    assert report['error']=='cleanup failed (trace_write): disk full'


def test_runtime_disappearance_records_active_workflow_wait(monkeypatch):
    from check_platform_workflow import PlatformObserver,Observer
    observer=object.__new__(PlatformObserver)
    def gone():raise FileNotFoundError('simulation-ready.json')
    observer.guard=gone
    monkeypatch.setattr(Observer,'wait',lambda self,predicate,seconds,label:predicate())
    with pytest.raises(RuntimeError,match='original D405 target measurement: simulation-ready.json'):
        observer.wait(lambda:True,360.,'original D405 target measurement')
    assert observer.last_wait['limit_wall_s']==360.


def test_missing_marker_reports_matching_stack_failure_without_losing_original(tmp_path,monkeypatch):
    import json
    import check_platform_workflow as module
    observer=object.__new__(module.PlatformObserver)
    observer.assets=tmp_path;observer.session='current';observer.bundle=dict(revision='r',runtime_revision=2)
    observer.latest={}
    (tmp_path/'startup-error.json').write_text(json.dumps(dict(session_id='current',asset_revision='r',runtime_revision=2,error='Paint hardware motion inhibited; full stack restart required')))
    def gone(assets,**kwargs):raise FileNotFoundError('simulation-ready.json')
    monkeypatch.setattr(module,'validate_probe_context',gone)
    with pytest.raises(RuntimeError,match='Paint hardware motion inhibited') as caught:observer.guard()
    assert 'simulation-ready.json' in str(caught.value)
    assert isinstance(caught.value.__cause__,FileNotFoundError)
    observer.session='other'
    with pytest.raises(FileNotFoundError):observer.guard()


def test_wall_face_uses_registry_and_rejects_backside_camera():
    from check_platform_workflow import wall_fixture
    scene={'wall':{'id':'wall','center':[0,-1.25,.9],'size':[1,.06,1.2]},'zed_world_optical':np.eye(4).tolist()}
    objects={'active_target':'wall','objects':[{'name':'wall','sketch_face':'+y'}]}
    fixture=wall_fixture(scene,objects)
    np.testing.assert_allclose(fixture['point'],[0,-1.22,.9])
    np.testing.assert_allclose(fixture['normal'],[0,1,0])
    assert np.asarray(fixture['corners']).shape==(4,3)
    scene['zed_world_optical'][1][3]=-2.4
    with pytest.raises(ValueError,match='opposite face'):wall_fixture(scene,objects)


def test_projection_rejects_outside_pixels_and_behind_camera():
    from check_platform_workflow import project_fixture
    fixture={'corners':[[-.1,-.1,1],[.1,-.1,1],[.1,.1,1],[-.1,.1,1]]}
    k=[[100,0,50],[0,100,50],[0,0,1]]
    assert project_fixture(fixture,np.eye(4),k,100,100)==[[40.,40.],[60.,40.],[60.,60.],[40.,60.]]
    with pytest.raises(ValueError,match='visible'):project_fixture(fixture,np.eye(4),k,10,10)


def test_free_space_requires_actual_fresh_force_and_clearance():
    from check_platform_workflow import free_space_evidence
    sample=dict(valid=True,wrench=[0,0,10,0,0,0],gravity_wrench=[0,0,10,0,0,0],velocities=[0]*6)
    fixture={'point':[0,0,0],'normal':[0,0,1]}
    assert free_space_evidence(sample,.01,[0,0,.1],fixture)['passed']
    assert not free_space_evidence(sample,.01,[0,0,.001],fixture)['passed']
    assert not free_space_evidence(sample,.5,[0,0,.1],fixture)['passed']
    sample['wrench'][0]=20
    assert not free_space_evidence(sample,.01,[0,0,.1],fixture)['passed']


def test_topics_follow_selected_arm_and_execute_is_explicit():
    from check_platform_workflow import workflow_remappings,authorize_workflow
    pairs=dict(workflow_remappings('right'))
    assert pairs['/isaac/rb20_painting/state']=='/platform/right/isaac_painting/state'
    assert pairs['/painting_admittance/free_space_confirmed']=='/platform/right/painting_admittance/free_space_confirmed'
    assert '/motion_abort' not in pairs
    with pytest.raises(ValueError,match='execute'):authorize_workflow(False)
    authorize_workflow(True)


def test_plan_only_cannot_publish_any_motion_workflow_request(monkeypatch):
    from check_platform_workflow import PlatformObserver,Observer
    observer=object.__new__(PlatformObserver);observer.execute_authorized=False
    observer.guard=lambda:None;observer.mutated=False
    published=[];monkeypatch.setattr(Observer,'publish',lambda *args:published.append(args))
    for topic in ('/target_selection_pixels','/painting_system/select_planes','/refine_work_area','/sketch_execute'):
        with pytest.raises(ValueError,match='execute'):observer.publish(topic,object())
    assert published==[] and observer.mutated is False


@pytest.mark.parametrize("platform",["commercial","rb10"])
def test_generated_paint_roi_survives_actual_organized_extractor(tmp_path,platform):
    """Ideal rendered wall geometry; runtime occlusion is a separate live check."""
    import json
    import yaml
    from prepare_platform_assets import generate_urdf_assets
    from sketch_control.runtime_selection import normalize_selection
    from sketch_control.multi_plane_geometry import polygon_mask,extract_planes
    from check_platform_workflow import wall_fixture,project_fixture
    generate_urdf_assets(ROOT,normalize_selection(dict(platform=platform,process='paint')),tmp_path)
    scene=json.loads((tmp_path/'scene.json').read_text())
    fixture=wall_fixture(scene,yaml.safe_load((tmp_path/'objects.yaml').read_text()))
    transform=np.asarray(scene['zed_world_optical']);width,height=scene['camera_resolution']
    focal=width*2.5/3.6  # RenderedCameras focal length / horizontal aperture.
    k=np.array([[focal,0,width/2],[0,focal,height/2],[0,0,1.]])
    polygon=project_fixture(fixture,transform,k,width,height)
    vv,uu=np.mgrid[0:height:4,0:width:4];pixels=np.c_[uu.ravel(),vv.ravel()]
    pixels=pixels[polygon_mask(pixels,[polygon])]
    normal=transform[:3,:3].T@fixture['normal']
    point=transform[:3,:3].T@(np.asarray(fixture['point'])-transform[:3,3])
    rays=np.c_[(pixels[:,0]-width/2)/focal,(pixels[:,1]-height/2)/focal,np.ones(len(pixels))]
    depth=(normal@point)/(rays@normal)
    planes=extract_planes(rays*depth[:,None],pixels,pixel_stride=4,
                          rgb=np.full((height,width,3),200,dtype=np.uint8))
    assert len(planes)==1
    assert planes[0]['inlier_count']==len(pixels)>=80
    assert planes[0]['rms_m']<1e-8
    np.testing.assert_allclose(planes[0]['normal'],normal,atol=1e-8)


def test_slow_simulation_scales_only_long_workflow_waits():
    pytest.importorskip('rclpy')
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time
    from check_rb20_painting_isaac import run_workflow
    waits=[]
    identity=dict(path_id='p',plan_hash='hash',work_area_id='area',plane_generation_id='plane')
    observer=NS(latest=dict(zed_info=NS(k=np.eye(3).flatten().tolist()),front=dict(width=640,height=480),
        work_area=dict(work_area_id='area'),refinement=dict(plane_generation_id='plane'),
        plan=identity,catalog=dict(generation='1',planes=[dict(id='one',corners=[[0,0,0],[1,0,0],[1,1,0],[0,1,0]])])),
        events=[],node=NS(get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time(sec=1)))),
        wait_healthy=lambda seconds:waits.append(('health',seconds)),
        wait=lambda predicate,seconds,label:waits.append((label,seconds)),publish=lambda *args:None)
    run_workflow(observer,'manual',execute=True,scene=dict(zed_world_optical=np.eye(4).tolist()),
                 selection_polygon=[[1,1],[5,1],[5,5],[1,5]],plane_accuracy=lambda:None,workflow_time_scale=3.)
    durations=dict(waits)
    assert durations['original D405 target measurement']==360.
    assert durations['original D405 work-area measurement']==360.
    assert durations['original painting completion']==900.
    assert durations['health']==45.
    assert durations['original runtime readiness discovery']==10.
    assert durations['request-bound target catalog and executor acknowledgement']==20.
    assert durations['original path validation']==60.
    assert durations['original Run readiness']==20.


@pytest.mark.parametrize('scale',[0,-1,float('nan'),float('inf')])
def test_workflow_time_scale_rejects_unbounded_or_nonpositive(scale):
    from check_rb20_painting_isaac import run_workflow
    with pytest.raises(ValueError,match='time scale'):
        run_workflow(None,'manual',workflow_time_scale=scale)


@pytest.mark.parametrize('delay,allowed',[(.02,True),(.3,False)])
def test_free_space_waits_for_exact_tf_without_refreshing_state_age(monkeypatch,delay,allowed):
    from types import SimpleNamespace as NS
    import check_platform_workflow as module
    obs=object.__new__(module.PlatformObserver)
    obs.execute_authorized=True;obs.guard=lambda:None;obs.mutated=False
    obs.active=dict(base_frame='World',tcp_link='left_manipulator_ee_point')
    stamp=object();obs.rclpy=NS(time=NS(Time=NS(from_msg=lambda value:value)))
    obs.latest={'state':NS(header=NS(stamp=stamp))};obs.received={'state':99.99}
    obs.samples=[dict(time=t,valid=True,wrench=[0]*6,gravity_wrench=[0]*6,velocities=[0]*6) for t in (99.6,99.7,99.8,99.9,99.99)]
    obs.fixture=dict(point=[0,0,0],normal=[0,0,1]);clock=[100.];available=[False]
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    def lookup(base,tip,requested):
        assert available[0] and requested is stamp
        return NS(transform=NS(translation=NS(x=0,y=0,z=.1)))
    obs.tf=NS(can_transform=lambda base,tip,requested:available[0] and requested is stamp,lookup_transform=lookup)
    def wait(predicate,seconds,label):
        assert seconds==.15 and not predicate()
        available[0]=True;clock[0]+=delay
        obs.received['state']=clock[0]  # Later feedback must not renew the saved sample.
        assert predicate()
    obs.wait=wait;published=[]
    monkeypatch.setattr(module.Observer,'publish',lambda *args:published.append(args))
    if allowed:obs.publish('/painting_admittance/free_space_confirmed',object())
    else:
        with pytest.raises(RuntimeError,match='free-space confirmation refused'):
            obs.publish('/painting_admittance/free_space_confirmed',object())
    assert bool(published)==allowed
    assert obs.free_space_proof['state_age_s']==pytest.approx(.01+delay)


def test_longer_manual_strokes_remain_inside_selected_area():
    from check_rb20_painting_isaac import manual_strokes
    normal=np.asarray(manual_strokes(900,600))
    longer=np.asarray(manual_strokes(900,600,.15))
    np.testing.assert_allclose(np.abs(np.diff(normal[:,:,1],axis=1)),300.)
    np.testing.assert_allclose(np.abs(np.diff(longer[:,:,1],axis=1)),420.)
    assert np.all(longer[:,:,0]>90) and np.all(longer[:,:,0]<810)
    assert np.all(longer[:,:,1]>60) and np.all(longer[:,:,1]<540)
    assert longer[0,0,1]<longer[0,1,1] and longer[1,0,1]>longer[1,1,1]


@pytest.mark.parametrize('inset',[0,.1,.5,float('nan'),float('inf')])
def test_manual_stroke_inset_rejects_outside_work_area(inset):
    from check_rb20_painting_isaac import run_workflow
    with pytest.raises(ValueError,match='stroke inset'):
        run_workflow(None,'manual',stroke_inset_fraction=inset)
