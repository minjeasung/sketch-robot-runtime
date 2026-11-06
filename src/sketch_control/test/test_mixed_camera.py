"""Physical frame identity must survive simulated time and calibration boundaries."""
import pytest
from sketch_control.camera_registration import CameraRegistration
from sketch_control.mixed_frame_sync import MixedFrameSync


def registration(**changes):
    args=dict(camera_id='zed',serial='123',optical_frame='physical_zed_optical',target_frame='World',translation_m=(1,2,3),quaternion_xyzw=(0,0,0,1),revision='cal-1',pose_source='fixed_measured')
    args.update(changes)
    return CameraRegistration(**args)


def frame(**changes):
    args=dict(camera_id='zed',serial='123',source_session='session-a',source_frame_id=1,capture_stamp=100,receipt_monotonic=10.,registration_revision='cal-1')
    args.update(changes)
    return args


def test_registration_transform_is_fixed_and_requires_measured_pose():
    r=registration()
    assert r.transform_points([[0,0,2]]).tolist()==[[1,2,5]]
    with pytest.raises(ValueError,match='pose_source'): registration(pose_source='virtual_wrist')
    with pytest.raises(ValueError): registration(quaternion_xyzw=(0,0,0,0))


def test_binding_preserves_identity_and_rejects_duplicate_reordered_or_reconnected_frames():
    sync=MixedFrameSync(registration())
    b=sync.bind(frame(),1000,10.,0)
    assert (b.source_session,b.source_frame_id,b.capture_stamp,b.sim_stamp,b.registration_revision)==('session-a',1,100,1000,'cal-1')
    for f in [frame(),frame(source_frame_id=0),frame(source_session='session-b',source_frame_id=2),frame(serial='wrong',source_frame_id=2),frame(registration_revision='old',source_frame_id=2)]:
        with pytest.raises(ValueError): sync.bind(f,2000,10.1,0)


def test_paused_reset_and_expired_frames_fail_closed():
    sync=MixedFrameSync(registration())
    sync.bind(frame(),1000,10.,0)
    with pytest.raises(ValueError,match='clock'): sync.bind(frame(source_frame_id=2),1000,10.1,0)
    with pytest.raises(ValueError): sync.bind(frame(source_frame_id=2),2000,10.2,0)
    with pytest.raises(ValueError,match='epoch'): sync.bind(frame(source_frame_id=3),100,10.3,1)
    with pytest.raises(ValueError): sync.bind(frame(source_frame_id=3),200,10.4,1)
    assert sync.bind(frame(source_frame_id=4,receipt_monotonic=10.5),300,10.5,1).clock_epoch==1
    with pytest.raises(ValueError,match='expired'): sync.bind(frame(source_frame_id=5),400,12.,1)


def test_selection_pin_requires_fresh_acquisition_and_expires_without_rebinding():
    from sketch_control.mixed_frame_sync import SelectionLease
    b=MixedFrameSync(registration()).bind(frame(),1000,10.,0)
    lease=SelectionLease('cal-1')
    lease.pin(b,10.5)
    lease.require(b,25.)
    with pytest.raises(ValueError,match='expired'): lease.require(b,40.6)
    with pytest.raises(ValueError): lease.pin(b,12.)
    lease.clear()
    with pytest.raises(ValueError): lease.require(b,25.)


def test_registration_cannot_publish_measured_pose_over_virtual_camera_link():
    with pytest.raises(ValueError,match='virtual'):
        registration(camera_id='d405',optical_frame='d405_color_optical_frame')


def test_selection_pin_cannot_extend_the_same_capture_forever():
    from sketch_control.mixed_frame_sync import SelectionLease
    b=MixedFrameSync(registration()).bind(frame(),1000,10.,0)
    lease=SelectionLease('cal-1')
    lease.pin(b,10.1)
    lease.pin(b,10.9)
    with pytest.raises(ValueError,match='expired'): lease.require(b,40.5)


def test_all_required_physical_camera_leases_must_be_current():
    from sketch_control.mixed_frame_sync import PhysicalCameraReadiness
    r=registration(); d=registration(camera_id='d405',serial='456',optical_frame='physical_d405_optical')
    cameras=PhysicalCameraReadiness({'zed':r,'d405':d})
    def identity(reg):
        return dict(camera_id=reg.camera_id,serial=reg.serial,frame_id=reg.optical_frame,valid=True,frame_binding=dict(source_session='s',source_frame_id=1,capture_stamp=100,receipt_monotonic=10.,sim_stamp=1000,clock_epoch=0,registration_revision='cal-1'))
    cameras.observe('zed',identity(r),10.)
    assert not cameras.ready(10.)
    cameras.observe('d405',identity(d),10.)
    assert cameras.ready(10.2)
    assert not cameras.ready(11.1)
    with pytest.raises(ValueError): cameras.observe('d405',identity(r),10.2)
    assert not cameras.ready(10.2)
