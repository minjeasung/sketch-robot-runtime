from pathlib import Path
from types import SimpleNamespace
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts'))
from rb20_painting_sensors import CommandLease, transform_wrench, gravity_wrench

def command(**kw):
    return SimpleNamespace(**({**dict(session_id='session',scene_revision='revision',sequence=1,positions=[0.]*6,enabled=True,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=123456789))),**kw}))

def source_state(lease, now):
    lease.record_state(1,123456789,now)

def test_lease_rejects_foreign_duplicate_invalid_and_expired():
    lease = CommandLease('session','revision')
    source_state(lease,10.)
    assert not lease.accept(command(session_id='old'),10.)
    assert not lease.accept(command(positions=[float('nan')]*6),10.)
    assert lease.accept(command(),10.)
    assert lease.active(10.1)
    assert not lease.accept(command(),10.15)
    assert not lease.active(10.201)
    source_state(lease,11.)
    assert not lease.accept(command(sequence=2),11.), 'Duplicate source state must not refresh expiry'
    lease.record_state(2,0,11.)
    fresh=command(sequence=2,header=SimpleNamespace(stamp=SimpleNamespace(sec=2,nanosec=0)))
    assert lease.accept(fresh,11.)
    lease.invalidate()
    assert not lease.active(11.)
    assert not lease.accept(command(sequence=2),11.)
    assert lease.accept(command(sequence=3,enabled=False),11.)
    assert not lease.active(11.)

def test_pause_invalidates_old_target():
    lease = CommandLease('session','revision')
    source_state(lease,1.)
    lease.accept(command(),1.)
    lease.invalidate()
    assert lease.target is None
    assert not lease.active(1.01)
    assert not lease.accept(command(sequence=2),1.02)

def test_enabled_command_requires_known_unexpired_physics_state():
    lease=CommandLease('session','revision')
    assert not lease.accept(command(),10.)
    source_state(lease,10.)
    assert not lease.accept(command(),10.201)
    assert lease.target is None
    lease.record_state(2,0,10.21)
    assert not lease.accept(command(sequence=2),10.22), 'Old source stamp cannot borrow newer state freshness'

def test_command_expiry_is_bounded_by_original_physics_sample():
    lease=CommandLease('session','revision')
    source_state(lease,10.)
    assert lease.accept(command(),10.15)
    assert lease.active(10.19)
    assert not lease.active(10.201)
    assert not lease.accept(command(sequence=2),10.21)
    assert not lease.active(10.22)

def test_disabled_command_stops_without_finite_positions_or_source_stamp():
    lease=CommandLease('session','revision')
    source_state(lease,10.)
    assert lease.accept(command(),10.01)
    assert lease.accept(command(sequence=2,enabled=False,positions=[float('nan')],header=None),10.02)
    assert not lease.active(10.03)

def test_wrench_translation_and_gravity_are_physical():
    tf = np.eye(4); tf[:3,3] = [1,0,0]
    np.testing.assert_allclose(transform_wrench([0,2,0,0,0,0],tf),[0,2,0,0,0,2])
    np.testing.assert_allclose(gravity_wrench(2.,[1,0,0],np.eye(3)),[0,0,-19.62,0,19.62,0])

def test_isaac61_camera_frame_metadata_is_rational_time():
    from rb20_painting_sensors import camera_frame_index
    assert camera_frame_index({'referenceTimeNumerator':12,'referenceTimeDenominator':100}) == .12
    assert camera_frame_index(3) == 3
    assert camera_frame_index({'invalid':7}) is None

def test_camera_topics_match_original_native_ros_inputs():
    from rb20_painting_sensors import CAMERA_TOPICS
    assert CAMERA_TOPICS['zed']==('/zed/zed_node/rgb/color/rect/image','/zed/zed_node/depth/depth_registered','/zed/zed_node/rgb/color/rect/camera_info','/zed/zed_node/depth/camera_info')
    assert CAMERA_TOPICS['d405']==('/d405/d405/color/image_raw','/d405/d405/depth/image_rect_raw','/d405/d405/color/camera_info','/d405/d405/depth/camera_info')

def test_pause_state_never_marks_retained_data_valid():
    from rb20_painting_sensors import invalid_measurements
    q,v,e,w,g=invalid_measurements()
    assert all(np.isnan(x).all() for x in (q,v,e,w,g))
    assert q.shape==v.shape==e.shape==w.shape==g.shape==(6,)

def test_usd_angular_limit_uses_degrees_per_second():
    from rb20_painting_scene import usd_angular_speed
    assert usd_angular_speed(np.pi)==180.

def test_angular_drive_si_gain_is_authored_per_degree():
    from rb20_painting_scene import usd_angular_gain
    assert np.isclose(usd_angular_gain(300000.),5235.987755982989)
    assert np.isclose(usd_angular_gain(30000.),523.5987755982989)
    assert usd_angular_gain(0.)==0.

def test_roller_contact_selection_is_exact_and_unambiguous():
    import pytest
    from rb20_painting_scene import select_roller_collision
    roller='/World/RB20/Geometry/ft_link/paint_eoat_no_camera_roller_contact_link_paint_eoat_no_camera_roller_contact_collision'
    other=['/World/RB20/Geometry/ft_link/paint_eoat_no_camera_link_paint_eoat_no_camera_collision',
           roller+'_support', '/World/OtherRobot/'+roller.rsplit('/',1)[-1]]
    assert select_roller_collision(other+[roller])==roller
    with pytest.raises(ValueError,match='exactly one'):
        select_roller_collision(other)
    with pytest.raises(ValueError,match='exactly one'):
        select_roller_collision([roller,roller.replace('/Geometry/ft_link/','/Geometry/duplicate/')])

def test_loop_trace_attributes_source_stalls_without_refreshing_measurements():
    from rb20_painting_sensors import LoopTrace
    trace=LoopTrace(enabled=True)
    trace.record('camera_publish',.26,10.,step=55)
    trace.state_published(9.9,50)
    trace.state_published(10.3,55)
    report=trace.report()
    assert abs(report['state_max_gap_s']-.4)<1e-8
    assert report['phases']['camera_publish']['max_ms']==260.
    assert report['slow_events'][0]=={'phase':'camera_publish','duration_s':.26,'monotonic':10.,'step_index':55}
    assert report['state_gaps'][0]['previous_step_index']==50
    assert report['state_gaps'][0]['step_index']==55
    disabled=LoopTrace(enabled=False)
    assert disabled.call('none',lambda:42)==42
    assert disabled.report()['phases']=={}

def test_streaming_qos_never_waits_for_reliable_reader_ack():
    from rb20_painting_sensors import transport_qos_profiles
    fake=SimpleNamespace(QoSProfile=lambda **kw:SimpleNamespace(**kw),ReliabilityPolicy=SimpleNamespace(RELIABLE='reliable',BEST_EFFORT='best_effort'),DurabilityPolicy=SimpleNamespace(VOLATILE='volatile'))
    measured,command=transport_qos_profiles(fake)
    assert measured.reliability=='best_effort' and measured.depth==1
    assert measured.durability=='volatile'
    assert command.reliability=='reliable' and command.durability=='volatile'

def test_cleanup_always_closes_kit_when_ros_cleanup_errors():
    import pytest
    from rb20_painting_app import close_runtime
    called=[]
    def failed_shutdown():
        called.append('shutdown'); raise RuntimeError('context already down')
    with pytest.raises(RuntimeError,match='context already down'):
        close_runtime(SimpleNamespace(destroy_node=lambda:called.append('node')),failed_shutdown,lambda **kw:called.append(('app',kw['exit_code'])),0)
    assert called==['node','shutdown',('app',0)]

def test_trace_keeps_critical_startup_stall_after_render_ring_wraps():
    from rb20_painting_sensors import LoopTrace
    trace=LoopTrace(True)
    trace.record('clock_publish',.33,10.,1,cpu_duration=.001)
    for n in range(5000):trace.record('world_render',.07,11.+n,2)
    assert trace.report()['critical_events'][0]['duration_s']==.33
    assert trace.report()['critical_events'][0]['cpu_duration_s']==.001
    assert trace.report()['phases']['clock_publish']['cumulative_max_ms']==330.

def test_dedicated_simulator_uses_actual_loopback_dds_profile():
    launch=Path(__file__).resolve().parents[2]/'scripts/run_rb20_painting_isaac.sh'
    text=launch.read_text()
    assert 'source "$SCRIPT_DIR/rb20_painting_dds_env.sh"' in text
    assert text.index('source "$SCRIPT_DIR/isaac_env.sh"') < text.index('source "$SCRIPT_DIR/rb20_painting_dds_env.sh"')
    assert 'ROS_LOCALHOST_ONLY=1' not in text

def test_fresh_command_after_callback_uses_current_steady_time():
    lease=CommandLease('session','revision')
    source_state(lease,10.)
    loop_started=10.
    assert lease.accept(command(),10.01)
    assert not lease.active(loop_started)
    assert lease.active_at_current_time(lambda:10.011)
    assert not lease.active_at_current_time(lambda:10.211)

def test_participant_publication_mode_restores_process_environment(monkeypatch):
    import os
    from rb20_painting_sensors import publication_mode
    monkeypatch.setenv('RMW_FASTRTPS_PUBLICATION_MODE','ASYNCHRONOUS')
    with publication_mode('SYNCHRONOUS'):
        assert os.environ['RMW_FASTRTPS_PUBLICATION_MODE']=='SYNCHRONOUS'
    assert os.environ['RMW_FASTRTPS_PUBLICATION_MODE']=='ASYNCHRONOUS'
    monkeypatch.delenv('RMW_FASTRTPS_PUBLICATION_MODE')
    with publication_mode('ASYNCHRONOUS'):
        assert os.environ['RMW_FASTRTPS_PUBLICATION_MODE']=='ASYNCHRONOUS'
    assert 'RMW_FASTRTPS_PUBLICATION_MODE' not in os.environ

def ft_sample(window, step, raw=None, gravity=None, **kw):
    return window.add(np.full(6,step,dtype=float) if raw is None else raw,
                      np.full(6,step*.1,dtype=float) if gravity is None else gravity,
                      session_id=kw.pop('session_id','session'),scene_revision=kw.pop('scene_revision','revision'),
                      step_index=step,stamp=kw.pop('stamp',step*.002),**kw)

def test_five_physics_samples_average_raw_and_gravity_separately():
    from rb20_painting_sensors import WrenchWindowAverage
    window=WrenchWindowAverage()
    for step in range(1,5):assert ft_sample(window,step) is None
    result=ft_sample(window,5)
    np.testing.assert_allclose(result.raw,np.full(6,3.))
    np.testing.assert_allclose(result.gravity,np.full(6,.3))
    assert result.step_index==5 and result.stamp==.010
    assert result.sample_count==5
    result=ft_sample(window,6)
    np.testing.assert_allclose(result.raw,np.full(6,4.))
    assert result.step_index==6 and result.stamp==.012

def test_sensor_window_flushes_invalid_pause_and_explicit_reset():
    from rb20_painting_sensors import WrenchWindowAverage
    for fault in ['invalid','paused','nan_raw','nan_gravity','reset']:
        window=WrenchWindowAverage()
        for step in range(1,6):ft_sample(window,step)
        if fault=='reset':window.flush();start=6
        else:
            kwargs={'valid':False} if fault=='invalid' else {'playing':False} if fault=='paused' else {}
            raw=np.full(6,np.nan) if fault=='nan_raw' else None
            gravity=np.full(6,np.nan) if fault=='nan_gravity' else None
            assert ft_sample(window,6,raw,gravity,**kwargs) is None
            start=7
        for step in range(start,start+4):assert ft_sample(window,step) is None
        assert ft_sample(window,start+4) is not None

def test_sensor_window_flushes_step_gap_session_revision_and_time_rewind():
    from rb20_painting_sensors import WrenchWindowAverage
    for change in ['gap','session','revision','rewind','duplicate']:
        window=WrenchWindowAverage()
        for step in range(1,6):ft_sample(window,step)
        step=9 if change=='gap' else 5 if change=='duplicate' else 6
        kwargs={'session_id':'new'} if change=='session' else {'scene_revision':'new'} if change=='revision' else {'stamp':.001} if change=='rewind' else {}
        assert ft_sample(window,step,**kwargs) is None
        for offset in range(1,4):assert ft_sample(window,step+offset,**kwargs) is None
        # A rewound/frozen clock is invalid until advancing original sample stamps resume.
        if change=='rewind':
            for n in range(1,6):result=ft_sample(window,step+3+n,stamp=.001+n*.002)
            assert result is not None
        elif change=='duplicate':
            assert ft_sample(window,step+4,**kwargs) is None
            assert ft_sample(window,step+5,**kwargs) is not None
        else:assert ft_sample(window,step+4,**kwargs) is not None

def test_sensor_average_does_not_clip_physical_overload():
    from rb20_painting_sensors import WrenchWindowAverage
    window=WrenchWindowAverage()
    raw=np.array([1e6,-2e6,20.,40.,-100.,0.])
    gravity=np.array([4.,3.,2.,1.,0.,-1.])
    for step in range(1,6):result=ft_sample(window,step,raw,gravity)
    np.testing.assert_array_equal(result.raw,raw)
    np.testing.assert_array_equal(result.gravity,gravity)
