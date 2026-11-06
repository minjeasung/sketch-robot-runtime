"""Offline check of the system-Python environment used by ROS entrypoints."""
import json
from pathlib import Path
import subprocess

import pytest
import trimesh

ROOT=Path(__file__).resolve().parents[2]


def test_ros_child_loads_spray_mesh_without_api_venv_packages(tmp_path):
    if not (ROOT/'.runtime/platform/install/local_setup.bash').exists():
        pytest.skip('built platform ROS overlay required; no ROS graph is started')
    (tmp_path/'tool.stl').write_bytes(trimesh.creation.box(extents=[40.,60.,200.]).export(file_type='stl'))
    profile=tmp_path/'tool.json'
    profile.write_text(json.dumps(dict(schema_version=1,model_id='rb20_1900es',spray_tool_axis='+z',
        mesh_file='tool.stl',mesh_scale_to_m=.001,endpoint_confirmed=True,
        mesh_to_tcp=dict(translation_m=[0.,0.,0.],quaternion_xyzw=[0.,0.,0.,1.]))))
    code='''import json,sys,numpy,trimesh
from rbpodo_painting_control.spray_eoat import load_spray_eoat_profile
p=load_spray_eoat_profile(sys.argv[1],"rb20_1900es","+z")
print(json.dumps(dict(endpoint=p.endpoint_tcp_m.tolist(),mesh=trimesh.__file__,numpy=numpy.__file__,version=trimesh.__version__)))
'''
    result=subprocess.run(['bash','-c','source scripts/platform_ros_env.sh && exec /usr/bin/python3 -c "$1" "$2"',
        'probe',code,str(profile)],cwd=ROOT,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
    payload=json.loads(result.stdout.splitlines()[-1])
    assert payload['endpoint']==pytest.approx([0.,0.,.1])
    assert str(ROOT/'.runtime/platform/python') in payload['mesh']
    assert 'trimesh=='+payload['version'] in (ROOT/'config/platform_ros.requirements.txt').read_text().splitlines()
    assert '.venv-api' not in payload['numpy']


def test_api_and_ros_share_mesh_dependency_contract():
    assert '-r platform_ros.requirements.txt' in (ROOT/'config/system_api.requirements.txt').read_text().splitlines()
    assert 'scripts/setup_platform_python.sh' in (ROOT/'scripts/build_platform_sim.sh').read_text()


def test_ros_python_decodes_actual_rb10_collada_visual():
    if not (ROOT/'.runtime/platform/install/local_setup.bash').exists():pytest.skip('built platform ROS overlay required')
    mesh=ROOT/'src/rbpodo_ros2/rbpodo_description/meshes/rb10_1300e_u/visual/link0.dae'
    code='import sys,numpy,trimesh; m=trimesh.load_mesh(sys.argv[1],process=False); assert len(m.faces)>0 and numpy.isfinite(m.vertices).all()'
    result=subprocess.run(['bash','-c','source scripts/platform_ros_env.sh && exec /usr/bin/python3 -c "$1" "$2"',
        'probe',code,str(mesh)],cwd=ROOT,capture_output=True,text=True)
    assert result.returncode==0,result.stderr
