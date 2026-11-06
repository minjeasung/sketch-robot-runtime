"""RB10 weld fixture placement must expose a reachable original CAD seam."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/sketch_control')]
from prepare_platform_assets import generate_urdf_assets
from sketch_control.runtime_selection import normalize_selection


def test_rb10_simulated_weld_moves_intact_fixture_and_camera_together(tmp_path):
    bundle=generate_urdf_assets(ROOT,normalize_selection({'platform':'rb10','process':'weld'}),tmp_path)
    scene=json.loads((tmp_path/'scene.json').read_text())
    translation=np.asarray(scene['h_steel_translation'])
    camera=np.asarray(scene['zed_world_optical'])
    np.testing.assert_allclose(translation,[-1.5,0.,0.])
    np.testing.assert_allclose(camera[:3,3],[-1.8,-1.4,1.7])
    # Same relative rendered viewpoint preserves the visible original004 face.
    np.testing.assert_allclose(camera[:3,3]-translation,[-.3,-1.4,1.7])
    direction=np.asarray([-1.1,.4,.9])-camera[:3,3]
    np.testing.assert_allclose(camera[:3,2],direction/np.linalg.norm(direction))
    assert bundle['arms']['single']['tcp_basis']=='contact_tip_face_center'
    assert scene['rb10_weld_fixture_setup']=={
        'seam_id':'IfcFastener_WELD_004','work_angle_deg':60.,'travel_angle_deg':0.,
        'reverse':False,'verification':'offline_native_geometry_samples; live_process_validation_required'}


@pytest.mark.parametrize('mode,process',[('mixed','weld'),('simulation','spray'),('simulation','paint')])
def test_rb10_weld_fixture_setup_does_not_move_other_modes_or_processes(tmp_path,mode,process):
    generate_urdf_assets(ROOT,normalize_selection({'platform':'rb10','process':process,'execution_mode':mode}),tmp_path)
    scene=json.loads((tmp_path/'scene.json').read_text())
    assert scene['h_steel_translation']==[.5,-.4,0.]
    assert 'rb10_weld_fixture_setup' not in scene


def test_actual_rb10_cad_rigid_translation_preserves_seam_and_normal():
    from platform_weld_fixture import generate_fixture
    path=ROOT/'.runtime/platform/matrix-noncontact-final/rb10-single-weld/h_steel_collision.json'
    if not path.exists():pytest.skip('local actual CAD export unavailable')
    geometry=json.loads(path.read_text())
    original=generate_fixture(geometry,[.2,-1.8,1.7],scene_revision='original',frame_id='link0')
    shift=np.array([-2.,.4,0.])
    for mesh in geometry['meshes']:mesh['vertices']=(np.asarray(mesh['vertices'])+shift).tolist()
    shifted=generate_fixture(geometry,[-1.8,-1.4,1.7],scene_revision='shifted',frame_id='link0')
    assert [seam['seam_id'] for seam in shifted['seams']]==['IfcFastener_WELD_004']
    np.testing.assert_allclose(shifted['seams'][0]['points_xyz'],np.asarray(original['seams'][0]['points_xyz'])+shift)
    np.testing.assert_allclose(shifted['seams'][0]['reference_up_vectors'],original['seams'][0]['reference_up_vectors'],atol=1e-10)
    assert min(np.asarray(mesh['vertices'])[:,2].min() for mesh in geometry['meshes'])>=-1e-7
