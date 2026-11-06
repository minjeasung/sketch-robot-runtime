#!/usr/bin/env python3
"""Readiness and measured-evidence verifier; never builds assets or drives real robots."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
from uuid import uuid4

from rb20_painting_report import assess_painting, assess_stop, finite

ROOT = Path(__file__).resolve().parents[1]
SELECTION_KEYS = ('execution_mode', 'platform', 'active_arm', 'process')
IDENTITY_KEYS = ('session_id', 'asset_revision', 'runtime_revision', 'tool_revision', 'scene_revision')


def normalized_selection(selection):
    result = {key: selection.get(key) for key in SELECTION_KEYS}
    if (result['execution_mode'] not in ('simulation', 'mixed')
            or result['platform'] not in ('commercial', 'rb10')
            or result['active_arm'] not in (('left', 'right') if result['platform'] == 'commercial' else ('single',))
            or result['process'] not in ('spray', 'weld', 'paint')):
        raise ValueError('Expected a simulation/mixed platform, matching arm and supported process')
    return result


def check(passed, reason='', **details):
    return dict(status='passed' if passed else 'failed', reason='' if passed else reason, **details)


def same_sample_identity(sample, identity):
    return (sample.get('session_id', sample.get('session')) == identity.get('session_id')
            and sample.get('scene_revision') == identity.get('scene_revision'))


def assess_joint_tracking(samples, identity):
    errors = []
    if len(samples) < 2:
        return check(False, 'At least two measured controller feedback samples required')
    max_error = 0.
    previous = None
    reference = samples[0].get('desired_positions', [])
    moved = False
    for sample in samples:
        q, desired, names = sample.get('positions', []), sample.get('desired_positions', []), sample.get('joints', [])
        valid = (len(q) == len(desired) == len(names) == 6 and len(set(names)) == 6
                 and all(finite(v) for v in q + desired)
                 and sample.get('valid') is True and same_sample_identity(sample, identity)
                 and sample.get('source') in ('controller_state', 'isaac_measured_state')
                 and finite(sample.get('state_age_s')) and 0 <= sample['state_age_s'] <= .20
                 and finite(sample.get('stamp')))
        if not valid:
            errors.append('Invalid, stale, foreign or non-measured joint feedback')
            continue
        if previous is not None and (sample['stamp'] <= previous['stamp'] or names != previous['joints']):
            errors.append('Joint feedback clock or selected-joint order changed')
        previous = sample
        max_error = max(max_error, max(abs(a-b) for a, b in zip(q, desired)))
        moved |= len(reference) == 6 and any(abs(a-b) > .001 for a, b in zip(desired, reference))
    if not moved:
        errors.append('No changing trajectory reference was observed')
    if max_error > .02:
        errors.append('Measured joint tracking error exceeds 0.02 rad')
    return check(not errors, '; '.join(dict.fromkeys(errors)), max_error_rad=max_error, sample_count=len(samples))


def assess_observations(selection, evidence=None):
    selection = normalized_selection(selection)
    evidence = evidence or {}
    report = dict(schema_version=1, scope='execution_evidence', selection=selection,
                  status='not_run', identity=evidence.get('identity', {}), camera=evidence.get('camera', {}), checks={})
    if not evidence or not evidence.get('executed'):
        report['reason'] = 'No execution observations supplied'
        return report
    camera = report['camera']
    if selection['execution_mode'] == 'mixed' and camera.get('connected') is not True:
        report['reason'] = 'Real camera not connected; mixed integration not run'
        return report
    identity = report['identity']
    checks = report['checks']
    checks['selection'] = check(evidence.get('selection') == selection, 'Evidence belongs to a different selection')
    checks['identity'] = check(all(identity.get(key) is not None and identity.get(key) != '' for key in IDENTITY_KEYS)
        and (selection['execution_mode'] != 'mixed' or bool(identity.get('calibration_revision'))),
        'Session, asset, tool, scene, runtime or calibration revision missing')
    required_source = 'live_camera' if selection['execution_mode'] == 'mixed' else 'isaac_camera'
    checks['camera'] = check(camera.get('connected') is True and camera.get('source') == required_source
        and bool(camera.get('camera_id')) and bool(camera.get('frame_id'))
        and finite(camera.get('age_s')) and 0 <= camera['age_s'] <= 1.,
        'Camera source, identity or freshness does not match the execution mode')
    checks['command_accepted'] = check(evidence.get('command_accepted') is True, 'No controller acceptance observed')
    checks['execution_complete'] = check(evidence.get('terminal_state') in ('COMPLETE','COMPLETED')
        and not evidence.get('execution_error'), 'Execution did not complete successfully')
    controller = evidence.get('controller', {})
    checks['controller'] = check(controller.get('active') is True and bool(controller.get('name')), 'Selected controller not active')
    checks['joint_tracking'] = assess_joint_tracking(evidence.get('joint_samples', []), identity)
    geometry = evidence.get('geometry', {})
    checks['surface_geometry'] = check(geometry.get('source') == 'measured'
        and finite(geometry.get('normal_error_deg')) and 0 <= geometry['normal_error_deg'] <= 2.
        and finite(geometry.get('offset_error_m')) and abs(geometry['offset_error_m']) <= .005,
        'Measured normal/offset comparison missing or outside 2 degrees / 5 mm', evidence=geometry)
    stop = evidence.get('stop', {})
    try:
        result = assess_stop(*(stop.get(key) for key in ('last_input', 'detected', 'disabled', 'cancel_ack', 'stopped')))
    except (ValueError, TypeError):
        result = dict(passed=False)
    checks['cancel_and_stop'] = check(result.pop('passed'), 'Cancellation/disable/stationary feedback incomplete or late', **result)
    if selection['process'] == 'paint':
        samples = evidence.get('force_samples', [])
        genuine = bool(samples) and all(s.get('source') == 'isaac_measured_reaction' and same_sample_identity(s, identity) for s in samples)
        try:
            force = assess_painting(evidence.get('events', []), samples)
        except (KeyError, ValueError, TypeError, OverflowError) as exc:
            force = dict(passed=False, errors=['Malformed force evidence: '+str(exc)])
        checks['roller_force'] = check(genuine and force['passed'],
            'Fresh measured reaction, two strokes and five-second 95% hold in 18–32 N required',
            target_force_n=20., hold_band_n=[20., 30.], measured_acceptance=force)
    if selection['process'] == 'weld':
        weave = evidence.get('weaving', {})
        checks['weld_weaving'] = check(weave.get('source') == 'measured_tcp'
            and finite(weave.get('position_error_m')) and 0 <= weave['position_error_m'] <= .005
            and finite(weave.get('cycles_observed')) and weave['cycles_observed'] >= 2,
            'Measured TCP weaving over at least two cycles is missing or outside 5 mm', evidence=weave)
    report['status'] = 'passed' if all(c['status'] == 'passed' for c in checks.values()) else 'failed'
    return report


def verification_matrix(reports=()):
    observed = {tuple(r['selection'].get(k) for k in SELECTION_KEYS): r for r in reports}
    rows = []
    for mode in ('simulation', 'mixed'):
        for platform, arm in (('commercial', 'left'), ('commercial', 'right'), ('rb10', 'single')):
            for process in ('spray', 'weld', 'paint'):
                selection = dict(zip(SELECTION_KEYS, (mode, platform, arm, process)))
                result = observed.get((mode, platform, arm, process))
                rows.append(dict(selection=selection, status=result['status'] if result else 'not_run',
                                 reason=result.get('reason', '') if result else 'No execution evidence recorded'))
    return rows


def pid_is_alive(pid):
    if type(pid) is not int or pid <= 0:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def prepare_report(selection, bundle, simulation, managed, *, now=None, pid_alive=pid_is_alive):
    selection = normalized_selection(selection)
    now = time.monotonic() if now is None else now
    checks = {}
    checks['matching_assets'] = check(bool(bundle) and all(bundle.get('selection', {}).get(k) == v for k, v in selection.items()),
                                      'Applied asset bundle is absent or does not match the requested selection')
    for name, heartbeat in (('simulator', simulation), ('managed_stack', managed)):
        checks[name] = check(bool(heartbeat) and heartbeat.get('ready') is True
            and pid_alive(heartbeat.get('pid')) and finite(heartbeat.get('updated_monotonic'))
            and 0 <= now-heartbeat['updated_monotonic'] <= 1.
            and heartbeat.get('asset_revision') == bundle.get('revision')
            and heartbeat.get('runtime_revision') == bundle.get('runtime_revision')
            and bool(heartbeat.get('session_id')) and bool(heartbeat.get('scene_revision')),
            'Missing, stopped, stale or mismatched runtime heartbeat')
    checks['same_session_scene'] = check(bool(simulation) and all(simulation.get(k) == managed.get(k)
        for k in ('session_id', 'scene_revision', 'asset_revision', 'runtime_revision')), 'Simulator and managed stack identities differ')
    checks['measured_joints'] = check(simulation.get('joints_valid') is True, 'Measured joint state unavailable')
    if selection['process'] == 'paint':
        checks['measured_wrench'] = check(simulation.get('wrench_valid') is True, 'Measured roller wrench unavailable')
    checks['camera_ready'] = check(managed.get('cameras_ready') is True, 'Required camera input unavailable')
    arm = bundle.get('arms', {}).get(selection['active_arm'], {})
    identity = {key: managed.get(key) for key in IDENTITY_KEYS}
    identity['tool_revision'] = arm.get('tool_revision')
    identity['calibration_revision'] = managed.get('calibration_revision', bundle.get('calibration_revision'))
    return dict(schema_version=1, scope='readiness', selection=selection, status='not_run', identity=identity,
        reason='Readiness checks do not verify execution', readiness=dict(status='passed' if all(c['status']=='passed' for c in checks.values()) else 'failed', checks=checks))


def validate_execution_plan(selection, plan, identity):
    selection = normalized_selection(selection)
    if not isinstance(plan, dict) or plan.get('validated') is not True or not plan.get('plan_id'):
        raise ValueError('--execute requires an existing validated plan; no benchmark path is generated')
    if plan.get('selection') != selection or any(plan.get('identity', {}).get(key) != identity.get(key)
        or identity.get(key) is None for key in IDENTITY_KEYS):
        raise ValueError('Planned path session/asset/tool/scene/runtime revision or selection is stale')
    if selection['execution_mode'] == 'mixed' and (not identity.get('calibration_revision')
        or plan['identity'].get('calibration_revision') != identity['calibration_revision']):
        raise ValueError('Mixed execution plan requires matching camera calibration revision')
    return plan


def read_json(path):
    try:
        value = json.loads(Path(path).read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def json_safe(value):
    """Keep invalid sensor values explicit in standards-compliant evidence JSON."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['simulation', 'mixed'], required=True)
    parser.add_argument('--platform', choices=['commercial', 'rb10'], required=True)
    parser.add_argument('--arm', choices=['left', 'right', 'single'], required=True)
    parser.add_argument('--process', choices=['spray', 'weld', 'paint'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--assets', type=Path)
    parser.add_argument('--observations', type=Path, help='Recorded measured evidence to assess (never synthesized)')
    parser.add_argument('--execute', action='store_true', help='Trigger only an existing current simulator plan')
    parser.add_argument('--plan', type=Path, help='Validated plan envelope with selection and identity')
    parser.add_argument('--timeout', type=float, default=180.)
    args = parser.parse_args(argv)
    try:
        selected = normalized_selection(dict(execution_mode=args.mode, platform=args.platform, active_arm=args.arm, process=args.process))
    except ValueError as exc:
        parser.error(str(exc))
    assets = args.assets or Path(read_json(ROOT/'.runtime/platform/configuration.json').get('bundle_directory', ROOT/'.runtime/platform/unapplied'))
    bundle = read_json(assets/'bundle.json')
    simulation, managed = read_json(assets/'simulation-ready.json'), read_json(assets/'managed-ready.json')
    prepared = prepare_report(selected, bundle, simulation, managed)
    report = prepared
    exit_code = 0
    if args.observations:
        report = assess_observations(selected, read_json(args.observations))
        report['readiness'] = prepared['readiness']
        report['observations_file'] = str(args.observations.resolve())
    if args.execute:
        try:
            if prepared['readiness']['status'] != 'passed':
                raise ValueError('Simulator/managed stack are not ready for this selection')
            plan = validate_execution_plan(selected, read_json(args.plan) if args.plan else None, prepared['identity'])
            observations = execute_existing_plan(selected, plan, bundle, timeout=args.timeout)
            report = assess_observations(selected, observations)
            report['readiness'] = prepared['readiness']
            report['live_observations'] = observations
        except (ValueError, RuntimeError, ImportError) as exc:
            report = {**prepared, 'execution_refused': str(exc), 'status': 'not_run'}
            exit_code = 2
    report['verification_matrix'] = verification_matrix([report])
    report['summary'] = {status: sum(row['status']==status for row in report['verification_matrix']) for status in ('passed', 'failed', 'not_run')}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(json_safe(report), indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print(json.dumps(dict(status=report['status'], output=str(args.output), summary=report['summary'])))
    return exit_code or (1 if report['status']=='failed' else 0)


def validate_simulator_bundle(selection, bundle):
    """Check the actual selected control block before any ROS publisher exists."""
    import xml.etree.ElementTree as ET
    selection = normalized_selection(selection)
    if any(bundle.get('selection', {}).get(key) != value for key, value in selection.items()):
        raise ValueError('Selected bundle does not match the simulator request')
    try:
        robot = ET.parse(bundle['urdf']).getroot()
        joints = bundle['arms'][selection['active_arm']]['joints']
    except (OSError, KeyError, ET.ParseError) as exc:
        raise ValueError('Cannot verify generated simulator hardware: '+str(exc)) from exc
    if len(joints) != 6 or len(set(joints)) != 6:
        raise ValueError('Selected arm must have six unique joints')
    permitted = {'isaac_painting_hardware/IsaacPaintingSystem', 'topic_based_ros2_control/TopicBasedSystem'}
    covered = set()
    for block in robot.findall('ros2_control'):
        plugin = block.findtext('hardware/plugin')
        if plugin not in permitted:
            raise ValueError('Non-simulator hardware plugin in generated description')
        controlled = {j.get('name') for j in block.findall('joint')}
        selected = controlled.intersection(joints)
        if selection['process'] == 'paint' and selected and plugin != 'isaac_painting_hardware/IsaacPaintingSystem':
            raise ValueError('Selected roller joints require measured IsaacPaintingSystem')
        if covered.intersection(selected):
            raise ValueError('Selected joints have multiple control owners')
        covered.update(selected)
    if covered != set(joints):
        raise ValueError('Selected joints are not fully owned by simulator hardware')


def live_plan_matches(selection, plan, status):
    if selection['process'] == 'weld':
        return bool(status.get('runtime_ready') is True and status.get('state') == 'READY'
            and status.get('plan_id') == plan.get('plan_id')
            and status.get('server_session_id') == plan.get('server_session_id')
            and status.get('identity') == plan.get('expected') and bool(plan.get('expected')))
    return bool(status.get('ready') is True and status.get('process_mode') == selection['process']
        and status.get('path_id') == plan.get('plan_id')
        and status.get('plan_hash') == plan.get('plan_hash') and bool(plan.get('plan_hash')))


def require_current_runtime(bundle, identity, *, now=None):
    now = time.monotonic() if now is None else now
    heartbeat = read_json(Path(bundle.get('directory', ''))/'managed-ready.json')
    if (heartbeat.get('ready') is not True or not finite(heartbeat.get('updated_monotonic'))
            or not 0 <= now-heartbeat['updated_monotonic'] <= 1.
            or any(heartbeat.get(key) != identity.get(key)
                   for key in ('session_id', 'scene_revision', 'asset_revision', 'runtime_revision'))):
        raise RuntimeError('Runtime identity or heartbeat changed during plan observation')


def execute_existing_plan(selection, plan, bundle, *, timeout=180.):
    """Dispatch the existing executor request and record feedback, never generate a path.

    Full surface-error/stop/weave evidence must come from separate instrumented
    observations. Absence remains a failed check, even after successful dispatch.
    """
    validate_simulator_bundle(selection, bundle)
    if not finite(timeout) or not 1 <= timeout <= 3600:
        raise ValueError('Execution observation timeout must be between 1 and 3600 seconds')
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String, Bool
    from control_msgs.msg import JointTrajectoryControllerState
    from sensor_msgs.msg import Image
    from isaac_painting_msgs.msg import State
    context = rclpy.context.Context()
    context.init()
    node = rclpy.create_node('platform_runtime_verifier', context=context)
    latest, received = {}, {}
    arm = bundle['arms'][selection['active_arm']]
    prefix = arm.get('control_namespace', '/platform/'+selection['active_arm'] if selection['process']=='paint' else '')
    state_topic = arm.get('topic_namespace', '/platform/'+selection['active_arm']+'/isaac_painting')+'/state'
    controller = arm['controller']
    identity = dict(plan['identity'])
    observations = dict(selection=selection, identity=identity, executed=False, command_accepted=False,
        controller=dict(name=controller, active=False), joint_samples=[], force_samples=[], events=[],
        camera=dict(connected=False, source='unverified'))
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    dispatched = False
    started = None

    def on_json(key, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                return
        except ValueError:
            return
        latest[key], received[key] = value, time.monotonic()
        if key == 'event' and dispatched:
            observations['events'].append({**value, 'time': time.monotonic()})
            if value.get('state') not in ('IDLE', 'NOT_READY'):
                observations['command_accepted'] = True

    def on_joint(msg):
        if not dispatched:
            return
        names = list(msg.joint_names)
        if any(names.count(name)!=1 for name in arm['joints']):
            return
        order = [names.index(name) for name in arm['joints']]
        actual = getattr(msg, 'feedback', getattr(msg, 'actual', None))
        desired = getattr(msg, 'reference', getattr(msg, 'desired', None))
        if actual is None or desired is None or len(actual.positions)!=len(names) or len(desired.positions)!=len(names):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        age = node.get_clock().now().nanoseconds/1e9-stamp
        # The controller clock follows Isaac. Source time must advance; callback
        # arrival time is not used to rejuvenate paused physics samples.
        observations['joint_samples'].append(dict(time=time.monotonic(), stamp=stamp, state_age_s=age,
            valid=True, source='controller_state', session_id=identity['session_id'], scene_revision=identity['scene_revision'],
            joints=list(arm['joints']), positions=[actual.positions[i] for i in order],
            desired_positions=[desired.positions[i] for i in order]))
        observations['controller']['active'] = True

    def on_force(msg):
        if not dispatched:
            return
        now = time.monotonic()
        safety = latest.get('safety', {})
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        observations['force_samples'].append(dict(time=now,sim_time=stamp,step=msg.step_index,
            session=msg.session_id, session_id=msg.session_id,scene_revision=msg.scene_revision,
            state_age_s=node.get_clock().now().nanoseconds/1e9-stamp,
            force_age_s=now-received.get('safety', 0.), measured_contact_force_n=safety.get('contact_force_n'),
            source='isaac_measured_reaction',valid=bool(msg.joints_valid and msg.wrench_valid and msg.playing and safety.get('ft_valid'))))

    def on_camera(msg):
        # This only attests the selected live topic; the producer's camera
        # identity is obtained separately instead of deriving it from a TF.
        latest['camera_image'] = msg
        received['camera_image'] = time.monotonic()

    def refresh_camera():
        image = latest.get('camera_image')
        frame = latest.get('camera_identity', {})
        if image is not None and time.monotonic()-received.get('camera_identity',0.)<=1.:
            observations['camera'].update(connected=True,frame_id=image.header.frame_id,
                camera_id=frame.get('camera_id', frame.get('camera', '')),
                age_s=time.monotonic()-received['camera_image'])
            # Explicit producer provenance overrides mode expectation.
            source = frame.get('source')
            observations['camera']['producer_source'] = source
            if source == 'physical' and frame.get('valid') is True and frame.get('frame_binding'):
                source = 'live_camera'
            elif source == 'isaac' and frame.get('valid') is True:
                source = 'isaac_camera'
            if source in ('fixture','recording','live_camera','isaac_camera'):
                observations['camera']['source'] = source

    status_topic = '/welding/status' if selection['process']=='weld' else '/painting_system/readiness'
    node.create_subscription(String,status_topic,lambda msg:on_json('status',msg),latched)
    node.create_subscription(String,'/painting_system/execution_events',lambda msg:on_json('event',msg),100)
    node.create_subscription(String,prefix+'/painting_admittance/safety_status',lambda msg:on_json('safety',msg),10)
    node.create_subscription(String,'/perception/zed_frame_identity',lambda msg:on_json('camera_identity',msg),latched)
    node.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',on_camera,qos_profile_sensor_data)
    node.create_subscription(JointTrajectoryControllerState,prefix+'/'+controller+'/controller_state',on_joint,qos_profile_sensor_data)
    node.create_subscription(State,state_topic,on_force,qos_profile_sensor_data)
    # Explicit sim clock use prevents wall/simulator time mixing in age checks.
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter('use_sim_time', value=True)])
    abort = node.create_publisher(Bool,'/motion_abort',10)
    topic = '/welding/execute' if selection['process']=='weld' else '/sketch_execute'
    publisher = node.create_publisher(String if selection['process']=='weld' else Bool,topic,10)
    try:
        deadline = time.monotonic()+10.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if (publisher.get_subscription_count()>0 and time.monotonic()-received.get('status',0.)<=1.
                    and live_plan_matches(selection,plan,latest.get('status',{}))):
                break
        else:
            raise RuntimeError('Live executor did not acknowledge this exact current validated plan')
        if selection['process']=='weld':
            command_id = uuid4().hex
            message = String(data=json.dumps(dict(command_id=command_id,server_session_id=plan['server_session_id'],
                                                 expected=plan['expected'],plan_id=plan['plan_id'])))
        else:
            message = Bool(data=True)
        require_current_runtime(bundle,identity)
        dispatched = True
        started = time.monotonic()
        observations['executed'] = True
        publisher.publish(message)
        while time.monotonic()-started<timeout:
            try:
                require_current_runtime(bundle,identity)
            except RuntimeError as exc:
                observations['execution_error'] = str(exc)
                observations['terminal_state'] = 'ABORT'
                abort.publish(Bool(data=True))
                break
            rclpy.spin_once(node,timeout_sec=.02)
            refresh_camera()
            status = latest.get('status', {})
            if selection['process']=='weld':
                ack = status.get('command_ack') or {}
                if ack.get('command_id')==command_id:
                    observations['command_accepted'] = ack.get('ok') is True
                if observations['command_accepted'] and status.get('state') in ('COMPLETE','COMPLETED','FAILED','ERROR','STOPPED'):
                    observations['terminal_state'] = status['state']
                    break
            elif observations['events'] and observations['events'][-1].get('state') in ('COMPLETE','ABORT','FAILED','NOT_READY'):
                observations['terminal_state'] = observations['events'][-1]['state']
                break
        else:
            observations['execution_error'] = 'Observation deadline expired; abort requested'
            abort.publish(Bool(data=True))
            for _ in range(10):
                rclpy.spin_once(node,timeout_sec=.02)
        observations['elapsed_s'] = time.monotonic()-started
        return observations
    except BaseException:
        if dispatched:
            abort.publish(Bool(data=True))
            for _ in range(10):
                rclpy.spin_once(node,timeout_sec=.02)
        raise
    finally:
        node.destroy_node()
        context.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
