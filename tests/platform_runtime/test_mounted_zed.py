from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))


def test_nominal_zed_is_between_shoulders_and_follows_head_rotation():
    from platform_camera_mount import nominal_zed_mount, link_world_transform
    robot=ET.parse(ROOT/'assets/construct/hongik_v4/source/construct_description/urdf_0528/construct_robot_0528_hongik_v2.urdf').getroot()
    mount=nominal_zed_mount()
    assert mount['parent_link']=='zed2i_link'
    assert mount['calibrated'] is False
    fixed=np.array(mount['parent_from_optical'])
    np.testing.assert_allclose(fixed[:3,3],[-.06,0,0])
    zero=link_world_transform(robot,mount['parent_link'],{'robot_head_rev_joint1':0.,'robot_head_rev_joint2':0.})@fixed
    tilted=link_world_transform(robot,mount['parent_link'],{'robot_head_rev_joint1':0.,'robot_head_rev_joint2':.7})@fixed
    np.testing.assert_allclose(zero[:3,3],[.05600094,-.61690671,1.96749947],atol=1e-7)
    np.testing.assert_allclose(zero[:3,2],[0,-1,0],atol=2e-5)
    assert tilted[2,2]<-.64
    assert tilted[2,3]<zero[2,3]
    with pytest.raises(ValueError,match='Missing joint'):
        link_world_transform(robot,mount['parent_link'],{})


def test_spray_bundle_camera_uses_head_mount_and_same_tf_calibration(tmp_path):
    import json
    from prepare_platform_assets import generate_urdf_assets
    from platform_camera_mount import link_world_transform
    from sketch_control.runtime_selection import normalize_selection
    bundle=generate_urdf_assets(ROOT,normalize_selection(dict(platform='commercial',active_arm='left',process='spray')),tmp_path)
    scene=json.loads((tmp_path/'scene.json').read_text())
    mount=scene['zed_mount'];calibration=json.loads((tmp_path/'calibration/zed.json').read_text())
    assert calibration['parent_frame']=='zed2i_link'
    assert 'T_world_zed_optical' not in calibration
    robot=ET.parse(bundle['urdf']).getroot()
    expected=link_world_transform(robot,'zed2i_link',bundle['initial_positions'])@np.array(mount['parent_from_optical'])
    np.testing.assert_allclose(scene['zed_world_optical'],expected)
    assert abs(expected[0,3])<.10 and expected[2,3]>1.7
    # The fixture moved 0.60 m farther away; frame the horizontal top face
    # through the actual head joints instead of moving a free camera.
    target=np.array([0.,-1.5,1.016])
    direction=target-expected[:3,3];direction/=np.linalg.norm(direction)
    assert np.dot(expected[:3,2],direction)>.999999
    assert calibration['calibrated'] is False


def test_spray_standby_is_near_entry_and_keeps_top_face_visible(tmp_path):
    from prepare_platform_assets import generate_urdf_assets
    from platform_camera_mount import link_world_transform,nominal_zed_mount
    from platform_visibility import robot_occluders,patch_visibility
    from sketch_control.runtime_selection import normalize_selection
    b=generate_urdf_assets(ROOT,normalize_selection(dict(platform='commercial',active_arm='left',process='spray')),tmp_path)
    robot=ET.parse(b['urdf']).getroot();joints=b['initial_positions']
    tcp=link_world_transform(robot,'left_manipulator_ee_point',joints)
    # Standby is above/beside the intended entry, with output still OFF.
    entry=np.array([.24,-1.514,1.516])
    assert np.linalg.norm(tcp[:3,3]-entry)<.20
    assert tcp[2,3]>entry[2]+.10
    camera=link_world_transform(robot,'zed2i_link',joints)@np.array(nominal_zed_mount()['parent_from_optical'])
    corners=[[-.272,-1.58,1.016],[.272,-1.58,1.016],[.272,-1.42,1.016],[-.272,-1.42,1.016]]
    assert patch_visibility(corners,camera[:3,3],robot_occluders(robot,joints))['clear']


def test_right_weld_standby_preserves_fixture_and_frames_lower_joint(tmp_path):
    import json
    from prepare_platform_assets import generate_urdf_assets
    from platform_camera_mount import link_world_transform
    from platform_visibility import robot_occluders,patch_visibility
    from sketch_control.runtime_selection import normalize_selection
    b=generate_urdf_assets(ROOT,normalize_selection(dict(platform='commercial',active_arm='right',process='weld')),tmp_path)
    scene=json.loads((tmp_path/'scene.json').read_text());robot=ET.parse(b['urdf']).getroot()
    assert scene['h_steel_translation']==[-.515,-1.9,0.]
    assert scene['zed_mount']['parent_link']=='zed2i_link'
    assert scene['weld_demo']['seam_id']=='horizontal_lower_front_fillet'
    tcp=link_world_transform(robot,b['arms']['right']['tcp_link'],b['initial_positions'])
    entry=np.array([.2,-1.496,.629])+np.array([0,2**-.5,2**-.5])*.012
    assert np.linalg.norm(tcp[:3,3]-entry)<.13
    camera=np.asarray(scene['zed_world_optical'])
    corners=np.array([[-.2,-1.496,.629],[.2,-1.496,.629],[.2,-1.485,.65],[-.2,-1.485,.65]])
    assert patch_visibility(corners,camera[:3,3],robot_occluders(robot,b['initial_positions']))['clear']
    optical=(np.linalg.inv(camera)@np.c_[corners,np.ones(4)].T).T[:,:3]
    pixels=optical[:,:2]/optical[:,2,None]*444.444+[320,240]
    assert np.all((pixels>[0,0])&(pixels<[640,480]))


@pytest.mark.parametrize('arm', ['left', 'right'])
def test_commercial_welding_always_uses_head_zed(tmp_path, arm):
    import json
    from prepare_platform_assets import generate_urdf_assets
    from platform_camera_mount import link_world_transform
    from sketch_control.runtime_selection import normalize_selection
    b = generate_urdf_assets(ROOT, normalize_selection(dict(
        platform='commercial', execution_mode='simulation', active_arm=arm, process='weld')), tmp_path)
    scene = json.loads((tmp_path/'scene.json').read_text())
    calibration = json.loads((tmp_path/'calibration/zed.json').read_text())
    assert scene.get('zed_mount', {}).get('parent_link') == 'zed2i_link'
    assert calibration['parent_frame'] == 'zed2i_link'
    assert 'T_world_zed_optical' not in calibration
    robot = ET.parse(b['urdf']).getroot()
    expected = link_world_transform(robot, 'zed2i_link', b['initial_positions']) @ np.array(scene['zed_mount']['parent_from_optical'])
    np.testing.assert_allclose(scene['zed_world_optical'], expected)
