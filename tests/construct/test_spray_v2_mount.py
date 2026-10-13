"""The imported tool must replace the old geometry in both robot descriptions."""
import sys
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from prepare_construct_assets import prepare


def test_left_spray_mount_and_tcp_match_cad_without_changing_right_arm(tmp_path):
    manifest=prepare(ROOT,tmp_path)
    source=ET.parse(ROOT/'src/construct_description/urdf_0528/construct_robot_0528.urdf').getroot()
    for filename in ('construct.urdf','construct_control.urdf'):
        robot=ET.parse(tmp_path/filename).getroot()
        mesh=robot.find("link[@name='left_manipulator_link_ee']/visual/geometry/mesh")
        assert mesh.get('filename').endswith('spray_v2_visual.stl')
        assert robot.find("link[@name='left_D435_link']") is None
        mount=robot.find("joint[@name='left_manipulator_joint_ee']/origin")
        assert np.allclose(list(map(float,mount.get('xyz').split())),[0,0,.0543])
        assert np.allclose(list(map(float,mount.get('rpy').split())),[0,0,0])
        tcp=robot.find("joint[@name='left_ee_point_joint']/origin")
        assert np.allclose(list(map(float,tcp.get('xyz').split())),[-.110,0,.2822])
        assert np.allclose(list(map(float,tcp.get('rpy').split())),[0,0,0])
        for name in ('right_manipulator_joint_ee','right_ee_point_joint'):
            assert robot.find(f"joint[@name='{name}']/origin").attrib==source.find(f"joint[@name='{name}']/origin").attrib
    assert manifest['equipment']['left']['model']=='spray_v2'
    assert manifest['equipment']['left']['mass_calibrated'] is False


def test_new_tool_does_not_inherit_old_collision_exemptions(tmp_path):
    prepare(ROOT,tmp_path)
    robot=ET.parse(tmp_path/'construct.srdf').getroot()
    for pair in robot.findall('disable_collisions'):
        names={pair.get('link1'),pair.get('link2')}
        assert 'left_D435_link' not in names
        if 'left_manipulator_link_ee' in names:
            assert names=={'left_manipulator_link6','left_manipulator_link_ee'}


def test_rigid_tool_excludes_spray_envelope_and_flexible_hoses():
    import trimesh
    meta=json.loads((ROOT/'assets/construct/spray_v2/mount.json').read_text())
    assert not set(meta['included_parts']) & {'spray coat','hose_1','hose_2','hose_3','RB20-1900ES_LINK6'}
    mesh=trimesh.load_mesh(ROOT/'assets/construct/spray_v2/spray_v2_visual.stl')
    assert np.max(mesh.extents)<.4
    assert np.allclose(mesh.bounds[1,2],meta['tcp_in_flange_m'][2],atol=.0002)
