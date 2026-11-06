"""Imported vendor robot keeps its kinematics and the new left spray assembly."""
import sys
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from prepare_construct_assets import prepare
from prepare_construct_runtime import prepare as prepare_runtime


def test_hongik_model_preserves_arm_kinematics_and_replaces_right_tool(tmp_path):
    meta = prepare(ROOT, tmp_path, source_model='hongik_v4')
    generated = ET.parse(tmp_path / 'construct_control.urdf').getroot()
    original = ET.parse(ROOT / 'src/construct_description/urdf_0528/construct_robot_0528.urdf').getroot()
    for joint in original.findall("joint[@type='revolute']"):
        actual = generated.find(f"joint[@name='{joint.get('name')}']")
        assert ET.tostring(actual) == ET.tostring(joint)
    assert len(generated.findall("joint[@type='revolute']")) == 14
    assert generated.find("link[@name='right_D435_link']") is None
    assert generated.find("link[@name='left_D435_link']") is None
    assert generated.find("link[@name='hongik_eoat_base_link']/collision/geometry/mesh") is not None
    assert generated.find("link[@name='left_manipulator_link_ee']/visual/geometry/mesh").get('filename').endswith('spray_v2_visual.stl')
    np.testing.assert_allclose(np.fromstring(generated.find("joint[@name='right_ee_point_joint']/origin").get('xyz'), sep=' '), [.206216, .118535, -.300173])
    assert meta['source_model'] == 'hongik_v4'
    assert meta['equipment']['right']['tcp_basis'] == 'contact_tip_face_center'
    # The original testbed is replaced by the imported workpiece, not a hidden collider.
    assert not generated.findall("link[@name='testbed_link']/collision")


def test_hongik_semantics_reference_existing_links_only(tmp_path):
    prepare(ROOT, tmp_path, source_model='hongik_v4')
    links = {l.get('name') for l in ET.parse(tmp_path/'construct.urdf').getroot().findall('link')}
    pairs = {frozenset((p.get('link1'), p.get('link2'))) for p in ET.parse(tmp_path/'construct.srdf').getroot().findall('disable_collisions')}
    assert all(pair <= links for pair in pairs)
    assert frozenset(('hongik_eoat_base_link', 'right_manipulator_link6')) in pairs
    assert not any('hongik_eoat_base_link' in p and 'right_manipulator_link1' in p for p in pairs)


def test_hongik_equipment_includes_tool_on_sibling_fixed_branch(tmp_path):
    prepare(ROOT, tmp_path, source_model='hongik_v4')
    prepare_runtime(ROOT, tmp_path, urdf_path=tmp_path/'construct_control.urdf')
    meta = json.loads((tmp_path/'construct_right_equipment.json').read_text())
    assert 'hongik_eoat_base_link' in meta['links']
    assert meta['equipment_model'] == 'hongik_v4'
    assert meta['tcp_calibrated'] is False
    assert (tmp_path/'construct_right_eoat.stl').stat().st_size > 84
