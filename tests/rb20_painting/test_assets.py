import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from prepare_rb20_painting_assets import prepare, expand_source, link_transforms

@pytest.fixture(scope='module')
def assets(tmp_path_factory):
    out = tmp_path_factory.mktemp('painting')
    return out, prepare(ROOT, out)

def test_actual_model_and_hardware(assets):
    out, manifest = assets
    model = ET.parse(out/'robot.urdf').getroot()
    assert manifest['joint_names'] == ['base','shoulder','elbow','wrist1','wrist2','wrist3']
    assert [j.get('name') for j in model.findall('joint') if j.get('type') == 'revolute'] == manifest['joint_names']
    assert model.find('ros2_control/hardware/plugin').text == 'isaac_painting_hardware/IsaacPaintingSystem'
    assert 'robot_ip' not in (out/'robot.urdf').read_text()
    assert 'RBPodoHardware' not in (out/'robot.urdf').read_text()
    assert model.find("link[@name='paint_eoat_no_camera_roller_contact_link']/collision/geometry/cylinder").attrib == {'length':'0.175','radius':'0.026'}
    assert model.find("joint[@name='ft_sensor_joint']/child").get('link') == 'ft_link'
    for link in ('tcp','aft200_link','paint_eoat_no_camera_link','paint_eoat_no_camera_roller_contact_link','paint_d405_link'):
        assert link in manifest['ft_boundary']['downstream_links']

def test_preserved_geometry_and_transforms(assets):
    out, manifest = assets
    original = expand_source(ROOT)
    generated = ET.parse(out/'robot.urdf').getroot()
    for joint in original.findall('joint'):
        assert ET.tostring(joint) == ET.tostring(generated.find(f"joint[@name='{joint.get('name')}']"))
    orig_tf = link_transforms(original, manifest['initial_positions'])
    gen_tf = link_transforms(generated, manifest['initial_positions'])
    for name in orig_tf:
        np.testing.assert_allclose(orig_tf[name], gen_tf[name], atol=1e-12)
    physics = ET.parse(out/'robot_physics.urdf').getroot()
    assert len(physics.findall('joint')) == 7
    assert physics.find("joint[@name='ft_sensor_joint']") is not None
    expected = sum(float(l.find('inertial/mass').get('value')) for l in original.findall('link') if l.find('inertial/mass') is not None)
    actual = sum(float(l.find('inertial/mass').get('value')) for l in physics.findall('link') if l.find('inertial/mass') is not None)
    assert actual == pytest.approx(expected, abs=1e-10)
    assert manifest['ft_boundary']['mass_kg'] > 1.6
    assert manifest['inertia_overrides'][0]['reason'].startswith('Aggregate')

def test_manifest_consistency_and_camera_calibration(assets):
    out, manifest = assets
    assert json.loads((out/'manifest.json').read_text()) == manifest
    assert manifest['scene_revision']
    for name, digest in manifest['generated_hashes'].items():
        assert hashlib.sha256((out/name).read_bytes()).hexdigest() == digest
    for key, filename in manifest['calibration'].items():
        assert (out/filename).is_file()
    assert manifest['world_to_link0'] == {'translation':[0,0,0],'rotation_xyzw':[0,0,0,1]}

def test_scene_drive_has_explicit_si_units_and_roller_only_compliance(assets):
    out,_=assets
    scene=json.loads((out/'scene.json').read_text())
    assert scene['drive']=={'stiffness_nm_per_rad':300000.,'damping_nms_per_rad':30000.,'max_torque_nm':2000.}
    assert scene['roller_contact']=={'stiffness_n_per_m':5000.,'damping_ns_per_m':50.,'acceleration_spring':False}
    assert scene['material']=={'static_friction':.4,'dynamic_friction':.3,'restitution':0.}

def test_semantics_are_rb20_specific_and_objects_match_physics(assets):
    import yaml
    out, _ = assets
    srdf = ET.parse(out/'robot.srdf').getroot()
    assert not srdf.findall('group_state')
    assert not [p for p in srdf.findall('disable_collisions') if p.get('reason')=='Never']
    assert srdf.find("disable_collisions[@link1='paint_d405_link'][@link2='paint_eoat_no_camera_roller_contact_link']") is not None
    from sketch_control.robot_models import model_srdf
    original=ET.fromstring(model_srdf('rb20_1900es',ROOT/'src/sketch_control/config/rbpodo_named.srdf'))
    pairs=lambda root:{tuple(sorted((p.get('link1'),p.get('link2')))) for p in root.findall('disable_collisions')}
    # The original executor checks this exact policy before any real dispatch.
    assert pairs(srdf)==pairs(original)
    painting=yaml.safe_load((out/'painting.yaml').read_text())
    assert all(v['ros__parameters'].get('model_id','rb20_1900es')=='rb20_1900es' for v in painting.values() if isinstance(v,dict) and 'ros__parameters' in v)
    objects=yaml.safe_load((out/'objects.yaml').read_text())
    scene=json.loads((out/'scene.json').read_text())
    assert objects['active_target']==scene['wall']['id']
    assert objects['objects'][0]['position']==scene['wall']['center']
    assert objects['objects'][0]['size']==scene['wall']['size']

def test_no_physical_calibration_in_generated_profile(assets):
    import yaml
    out, _ = assets
    profile=yaml.safe_load((out/'painting.yaml').read_text())
    assert profile['d405_surface_refiner']['ros__parameters']['calibration_file']==str(out/'calibration/d405.json')
    assert '~/sketch_robot_ws' not in (out/'painting.yaml').read_text()

def test_stale_source_and_usd_rejected(assets):
    from prepare_rb20_painting_assets import verify_assets
    out,manifest=assets
    with pytest.raises(ValueError,match='USD verification'):
        verify_assets(ROOT,out,require_usd=True)
    usd=out/'robot.usd'; usd.write_text('test usd')
    evidence={'scene_revision':manifest['scene_revision'],'physics_urdf_sha256':manifest['generated_hashes']['robot_physics.urdf'],
              'usd_hashes':{'robot.usd':hashlib.sha256(usd.read_bytes()).hexdigest()}}
    (out/'usd-verification.json').write_text(json.dumps(evidence))
    verify_assets(ROOT,out,require_usd=True)
    usd.write_text('changed')
    with pytest.raises(ValueError,match='USD digest'):
        verify_assets(ROOT,out,require_usd=True)
    usd.write_text('test usd')
    evidence['scene_revision']='old'
    (out/'usd-verification.json').write_text(json.dumps(evidence))
    with pytest.raises(ValueError,match='USD scene revision'):
        verify_assets(ROOT,out,require_usd=True)
    (out/'robot_physics.urdf').write_text('changed')
    with pytest.raises(ValueError,match='Generated digest'):
        verify_assets(ROOT,out,require_usd=True)

def test_simulation_profile_uses_exact_named_execution_interlocks(assets):
    import yaml
    out, _ = assets
    profile=yaml.safe_load((out/'painting.yaml').read_text())
    for name in ('moveit_executor','sketch_to_waypoints'):
        assert profile[name]['ros__parameters']['real_painting_enabled'] is True
        assert profile[name]['ros__parameters']['dry_run'] is False
    assert profile['moveit_executor']['ros__parameters']['painting_force_enabled'] is True
    assert profile['painting_wrench_guard']['ros__parameters']['real_painting_enabled'] is True
    reference=profile['painting_wrench_reference']['ros__parameters']
    assert reference['dry_run'] is False
    assert reference['enable_force'] is False
    assert profile['painting_force_monitor']['ros__parameters']['startup_readiness_grace_s']==5.


def test_reprepare_after_mesh_change_requires_new_usd(tmp_path,monkeypatch):
    from prepare_rb20_painting_assets import verify_assets
    original=prepare(ROOT,tmp_path)
    usd=tmp_path/'robot.usd';usd.write_text('old imported geometry')
    evidence={'scene_revision':original['scene_revision'],
              'physics_urdf_sha256':original['generated_hashes']['robot_physics.urdf'],
              'usd_hashes':{'robot.usd':hashlib.sha256(usd.read_bytes()).hexdigest()}}
    (tmp_path/'usd-verification.json').write_text(json.dumps(evidence))
    mesh=next(ROOT/name for name in original['input_hashes'] if Path(name).suffix.lower()=='.stl')
    read_bytes=Path.read_bytes
    monkeypatch.setattr(Path,'read_bytes',lambda path:read_bytes(path)+(b'changed geometry' if path==mesh else b''))
    updated=prepare(ROOT,tmp_path)
    assert updated['generated_hashes']==original['generated_hashes']
    assert updated['scene_revision']!=original['scene_revision']
    with pytest.raises(ValueError,match='USD scene revision'):
        verify_assets(ROOT,tmp_path,require_usd=True)
