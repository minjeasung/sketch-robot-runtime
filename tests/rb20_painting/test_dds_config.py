"""Verify actual loopback UDP transport, not just exported environment knobs."""
import json
import os
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
PROFILE = ROOT / 'config/rb20_painting_fastdds.xml'
HELPER = ROOT / 'scripts/rb20_painting_dds_env.sh'


def test_profile_is_udp_only_and_every_locator_is_loopback():
    root = ET.parse(PROFILE).getroot()
    ns = {'d': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    transports = root.findall('.//d:transport_descriptor', ns)
    assert len(transports) == 1
    assert transports[0].find('d:type', ns).text == 'UDPv4'
    assert [x.text for x in transports[0].findall('d:interfaceWhiteList/d:address', ns)] == ['127.0.0.1']
    participant = root.find('.//d:participant', ns)
    assert participant.attrib['is_default_profile'] == 'true'
    assert participant.find('d:rtps/d:useBuiltinTransports', ns).text == 'false'
    assert [x.text for x in participant.findall('.//d:address', ns)] == ['127.0.0.1', '127.0.0.1']
    assert participant.find('d:rtps/d:userTransports/d:transport_id', ns).text == transports[0].find('d:transport_id', ns).text


def test_environment_removes_localhost_override_and_selects_explicit_profile():
    code = 'import json,os; print(json.dumps(dict(os.environ)))'
    result = subprocess.run(['bash', '-c', f'source {shlex.quote(str(HELPER))}; python3 -c {shlex.quote(code)}'],
        env=dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST'),
        capture_output=True, text=True, check=True)
    env = json.loads(result.stdout)
    assert 'ROS_LOCALHOST_ONLY' not in env
    assert env['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'SYSTEM_DEFAULT'
    assert env['FASTDDS_DEFAULT_PROFILES_FILE'] == env['FASTRTPS_DEFAULT_PROFILES_FILE'] == str(PROFILE)
    assert env['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'


def test_native_participant_has_no_shared_memory_transport():
    code = '''import json,pathlib,rclpy
rclpy.init()
n = rclpy.create_node('rb20_udp_transport_regression')
tasks = pathlib.Path('/proc/self/task')
names = [(p/'comm').read_text().strip() for p in tasks.iterdir()]
maps = pathlib.Path('/proc/self/maps').read_text()
print(json.dumps({'threads':names,'shm':[s for s in maps.splitlines() if '/dev/shm/fastrtps' in s]}))
n.destroy_node()
rclpy.shutdown()
'''
    result = subprocess.run(['bash', '-c', f'source /opt/ros/jazzy/setup.bash; source {shlex.quote(str(HELPER))}; python3 -c {shlex.quote(code)}'],
        env=dict(os.environ, ROS_DOMAIN_ID='119', ROS_LOCALHOST_ONLY='1'),
        capture_output=True, text=True, check=True, timeout=15)
    assert 'Error' not in result.stderr, result.stderr
    report = json.loads(result.stdout)
    assert any(name.startswith('dds.udp.') for name in report['threads'])
    assert not any(name.startswith('dds.shm.') for name in report['threads'])
    assert report['shm'] == []
