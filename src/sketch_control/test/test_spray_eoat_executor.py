"""Executor geometry and fail-closed gates with local ROS transport doubles."""
import ast
import copy
import json
import time
from concurrent.futures import Future
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from rbpodo_painting_control.spray_eoat import compensate_spray_endpoint, load_spray_eoat_profile
from rbpodo_painting_control.segment_path import SegmentWaypoint, segment_waypoint_position
from rbpodo_painting_control.spray_path import rotation_from_spray_path
from sketch_control.rotation_utils import quat_to_matrix, quat_from_matrix


def pose():
    return NS(position=NS(x=0., y=0., z=0.),
              orientation=NS(x=0., y=0., z=0., w=0.))


def collision():
    return NS(id='', header=NS(frame_id=''), meshes=[], mesh_poses=[],
              primitives=[], primitive_poses=[], operation=None)


@pytest.fixture
def executor():
    source = Path(__file__).parents[1] / 'sketch_control/moveit_executor.py'
    tree = ast.parse(source.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'MoveItExecutor')
    cls.bases = []
    methods = {'_build_segment_orientation_candidate', '_segment_tip_pose', '_mark_scene_dirty',
               '_defer_candidate_invalidation', '_apply_scene_done',
               'execute_trajectory_direct', 'publish_scene_periodic', '_spray_tick',
               '_execute_trajectory_follow_joint'}
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                and (n.name in methods or n.name.startswith('_spray_eoat')
                     or n.name == '_spray_endpoint_to_tcp')]
    scope = dict(np=np, copy=copy, time=time, replace=replace, SegmentPathError=ValueError,
                 EE_LINK='tcp', EOAT_TOUCH_LINKS=['tcp'], MOTION_MODES={'SPRAY'},
                 Pose=pose, Point=lambda: NS(x=0., y=0., z=0.),
                 Mesh=lambda: NS(vertices=[], triangles=[]),
                 MeshTriangle=lambda: NS(vertex_indices=[]),
                 AttachedCollisionObject=lambda: NS(object=collision()),
                 CollisionObject=type('CollisionObject', (), {'ADD': 0, 'REMOVE': 1, '__new__': lambda cls: collision()}),
                 quat_to_matrix=quat_to_matrix, quat_from_matrix=quat_from_matrix,
                 rotation_from_spray_path=rotation_from_spray_path,
                 segment_waypoint_position=segment_waypoint_position,
                 compensate_spray_endpoint=compensate_spray_endpoint)
    module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0), cls], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), scope)
    node = object.__new__(scope['MoveItExecutor'])
    node.test_scope = scope
    for item in tree.body:
        if isinstance(item, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'SPRAY_EOAT_TOUCH_LINKS' for t in item.targets):
            exec(compile(ast.Module(body=[item], type_ignores=[]), str(source), 'exec'), scope)
    node.process_mode = 'spray'
    node.model_id = 'rb20'
    node.spray_tool_axis = '+z'
    node.spray_eoat_profile = 'selected.json'
    node._scene_revision = 1
    node.scene_confirmed = False
    node.scene_initialized = False
    node.executing = False
    node.events = []
    node._spray_off = lambda: node.events.append('off')
    node._request_motion_abort = lambda reason: node.events.append(reason)
    node.get_logger = lambda: NS(error=lambda *a, **k: None, warn=lambda *a, **k: None, info=lambda *a, **k: None)
    profile = NS(sha256='a'*64, endpoint_tcp_m=np.array([.12, -.23, .34]),
                 vertices_tcp_m=np.array([[0., 0., 0.], [.1, 0., 0.], [0., .2, 0.]]), faces=np.array([[0, 1, 2]]))
    profile.metadata = lambda: dict(spray_eoat_profile_sha256=profile.sha256, spray_endpoint_tcp_m=profile.endpoint_tcp_m.tolist())
    node.disk_profile = profile
    scope['load_spray_eoat_profile'] = lambda *a: node.disk_profile
    metadata = dict(profile.metadata(), model_id=node.model_id, spray_tool_axis=node.spray_tool_axis)
    node.path = NS(process_mode='spray', raw_payload=copy.deepcopy(metadata), source=copy.deepcopy(metadata))
    node._segment_path = node.path
    return node


def test_rotated_full_endpoint_offset_and_safety_retreat(executor):
    node = executor
    @dataclass
    class Row:
        row_number: int = 1
        offset_m: float = .5
        mode: str = 'SPRAY'
    node.path.rows = [Row()]
    node.path.safety_approach_offset_m = .5
    node.path.final_retreat_offset_m = .5
    endpoint = pose()
    endpoint.position = NS(x=1., y=2., z=3.)
    endpoint.orientation = NS(x=0., y=0., z=2**-.5, w=2**-.5)
    node._segment_tip_pose = lambda *a: (copy.deepcopy(endpoint), np.array([0., 1., 0.]))
    candidate = node._build_segment_orientation_candidate(node.path, [0., 1., 0.], name='spray')
    for actual in [candidate['row_tcp_poses'][1], candidate['safety_tcp_pose'], candidate['retreat_tcp_pose']]:
        np.testing.assert_allclose([actual.position.x, actual.position.y, actual.position.z], [.77, 1.88, 2.66])
    assert candidate['row_tip_poses'][1].position.x == 1.


def test_missing_profile_blocks_without_zero_offset(executor):
    executor.spray_eoat_profile = ''
    assert executor._spray_eoat_blocker(executor.path)


@pytest.mark.parametrize('where,field,value', [
    ('raw_payload', 'spray_eoat_profile_sha256', 'b'*64),
    ('source', 'spray_endpoint_tcp_m', [0., 0., 0.]),
    ('source', 'spray_eoat_profile_sha256', None),
    ('raw_payload', 'spray_tool_axis', '-y'),
    ('source', 'model_id', 'different_robot'),
])
def test_metadata_must_match_disk(executor, where, field, value):
    getattr(executor.path, where)[field] = value
    assert executor._spray_eoat_blocker(executor.path)


def test_changed_mesh_invalidates_scene_aborts_without_mutating_snapshot(executor):
    node = executor
    assert node._spray_eoat_blocker(node.path) == ''
    frozen = {'segment_path': copy.deepcopy(node.path)}
    node._execution_snapshot = frozen
    node.executing = True
    node.scene_confirmed = True
    revision = node._scene_revision
    node.disk_profile = copy.copy(node.disk_profile)
    node.disk_profile.sha256 = 'b'*64
    assert node._spray_eoat_blocker(frozen['segment_path'])
    assert node._execution_snapshot is frozen
    assert frozen['segment_path'].raw_payload['spray_eoat_profile_sha256'] == 'a'*64
    assert node._scene_revision > revision
    assert not node.scene_confirmed
    assert 'off' in node.events
    assert any('SPRAY_EOAT' in event for event in node.events)


def test_mesh_attached_in_tcp_coordinates_without_primitives(executor):
    attached = executor._spray_eoat_collision_object(executor.disk_profile)
    assert attached.link_name == 'tcp'
    assert attached.object.header.frame_id == 'tcp'
    assert not attached.object.primitives
    assert len(attached.object.meshes) == 1
    mesh = attached.object.meshes[0]
    np.testing.assert_allclose([[v.x, v.y, v.z] for v in mesh.vertices], executor.disk_profile.vertices_tcp_m)
    assert mesh.triangles[0].vertex_indices == [0, 1, 2]
    origin = attached.object.mesh_poses[0]
    assert vars(origin.position) == dict(x=0., y=0., z=0.)
    assert vars(origin.orientation) == dict(x=0., y=0., z=0., w=1.)


def test_unconfirmed_scene_blocks_dispatch_gate(executor):
    assert executor._spray_eoat_blocker(executor.path, require_scene=True)


def test_paint_does_not_load_profile(executor):
    executor.process_mode = 'paint'
    executor.spray_eoat_profile = ''
    assert executor._spray_eoat_blocker(NS(process_mode='paint'), require_scene=True) == ''


def scene_transport(node):
    node.test_scope.update(
        PlanningScene=lambda: NS(world=None, robot_state=NS(attached_collision_objects=[], is_diff=False)),
        PlanningSceneWorld=lambda: NS(collision_objects=[]),
        ApplyPlanningScene=NS(Request=lambda: NS(scene=None)),
        PUBLISH_EOAT_ATTACHED_OBJECT=False)
    node._joint_command_timer = None
    node._stale_dynamic_obstacle_ids = set()
    node._lookup_transform_to_base = lambda *a, **kw: object()
    node.cfg = {'objects': []}
    node.dynamic_obstacles = []
    node.scene_pub = NS(publish=lambda ps: node.events.append(ps))
    node.requests = []
    node.response = Future()
    def send(request):
        node.requests.append(request)
        return node.response
    node.apply_scene_client = NS(wait_for_service=lambda **kw: True, call_async=send)


def test_scene_really_appends_mesh_and_only_matching_ack_unlocks(executor):
    node = executor
    scene_transport(node)
    node.publish_scene_periodic()
    assert len(node.requests) == 1
    assert not node.scene_confirmed
    attached = node.requests[0].scene.robot_state.attached_collision_objects
    assert [obj.object.id for obj in attached] == ['spray_eoat']
    assert not attached[0].object.primitives
    assert node._spray_eoat_blocker(node.path, require_scene=True)
    node.response.set_result(NS(success=True))
    assert node._spray_eoat_blocker(node.path, require_scene=True) == ''


@pytest.mark.parametrize('success', [True, False])
def test_changed_file_during_scene_apply_never_confirms(executor, success):
    node = executor
    scene_transport(node)
    node.publish_scene_periodic()
    node.disk_profile = copy.copy(node.disk_profile)
    node.disk_profile.sha256 = 'b'*64
    node.response.set_result(NS(success=success))
    assert not node.scene_confirmed
    assert node._spray_eoat_blocker(require_scene=True)


def test_invalid_mesh_never_publishes_or_dispatches(executor):
    node = executor
    scene_transport(node)
    def invalid(*args):
        raise ValueError('invalid mesh')
    node.test_scope['load_spray_eoat_profile'] = invalid
    node.publish_scene_periodic()
    assert not node.requests
    assert node.execute_trajectory_direct(NS()) is False
    assert 'SPRAY_EOAT' in node._last_dispatch_inhibit_reason


def test_changed_profile_at_dispatch_blocks_without_touching_transport(executor):
    node = executor
    scene_transport(node)
    node.publish_scene_periodic()
    node.response.set_result(NS(success=True))
    node.disk_profile = copy.copy(node.disk_profile)
    node.disk_profile.sha256 = 'b'*64
    assert node.execute_trajectory_direct(NS()) is False
    assert 'SPRAY_EOAT' in node._last_dispatch_inhibit_reason


def test_scene_hash_and_revision_are_both_required(executor):
    node = executor
    assert node._spray_eoat_blocker(node.path) == ''
    node.scene_confirmed = True
    node._scene_confirmed_revision = node._scene_revision
    node._spray_eoat_scene_hash = 'b'*64
    assert node._spray_eoat_blocker(require_scene=True)
    node._spray_eoat_scene_hash = 'a'*64
    node._scene_confirmed_revision -= 1
    assert node._spray_eoat_blocker(require_scene=True)


def test_only_fixed_tool_duplicates_are_touch_links(executor):
    links = set(executor._spray_eoat_collision_object(executor.disk_profile).touch_links)
    assert {'tcp', 'aft200_link', 'aft200_cable_guard_link',
            'paint_eoat_no_camera_link', 'paint_eoat_no_camera_roller_contact_link',
            'paint_d405_link'} <= links
    assert not links.intersection({'link0', 'link1', 'link2', 'link3', 'link4', 'link5', 'wall'})


def test_each_scene_publish_decodes_profile_only_once(executor):
    node = executor
    scene_transport(node)
    loads = []
    node.test_scope['load_spray_eoat_profile'] = lambda *args: (loads.append(args), node.disk_profile)[1]
    node.publish_scene_periodic()
    assert len(loads) == 1


def test_paint_transition_removes_only_owned_spray_object(executor):
    node = executor
    scene_transport(node)
    node.publish_scene_periodic()
    node.response.set_result(NS(success=True))
    node.process_mode = 'paint'
    node._paint_eoat_collision_object = lambda: NS()
    node.response = Future()
    node.publish_scene_periodic()
    assert len(node.requests) == 2
    attached = node.requests[-1].scene.robot_state.attached_collision_objects
    assert [(obj.object.id, obj.object.operation) for obj in attached] == [('spray_eoat', 1)]
    assert not node.scene_confirmed


def test_changed_profile_blocks_active_lease_renewal(executor):
    node = executor
    node.test_scope['SprayExecutionMixin'] = NS(_spray_tick=lambda self: self.events.append('lease'))
    assert node._spray_eoat_blocker() == ''
    node.executing = True
    node._active_trajectory_goal_token = object()
    node.disk_profile = copy.copy(node.disk_profile)
    node.disk_profile.sha256 = 'b'*64
    node._spray_tick()
    assert 'lease' not in node.events
    assert 'off' in node.events


def test_fjt_boundary_rechecks_after_server_readiness(executor):
    node = executor
    scene_transport(node)
    node.publish_scene_periodic()
    node.response.set_result(NS(success=True))
    node._motion_abort_requested = False
    node._active_trajectory_goal_token = None
    node._trajectory_within_joint_limits = lambda *args: True
    node._point_time_sec = lambda *args: 0.
    node.fjt_result_timeout_margin_s = 1.
    node._start_fjt_guard_timer = lambda *args: None
    node.test_scope['FollowJointTrajectory'] = NS(Goal=lambda: NS())
    sent = []
    def server_ready():
        node.disk_profile = copy.copy(node.disk_profile)
        node.disk_profile.sha256 = 'b'*64
        return True
    node.traj_action_client = NS(server_is_ready=server_ready,
        send_goal_async=lambda goal: (sent.append(goal), Future())[1])
    assert node._execute_trajectory_follow_joint(NS(joint_trajectory=NS(points=[NS()]))) is False
    assert not sent


@pytest.mark.parametrize('model,axis', [('rb20_1900es', '+z'), ('rb10_1300e', '-y')])
def test_real_mesh_endpoint_is_half_metre_from_wall(executor, tmp_path, model, axis):
    import trimesh
    node = executor
    mesh_path = tmp_path / 'tool.stl'
    mesh_path.write_bytes(trimesh.creation.box(extents=[.04, .08, .2]).export(file_type='stl'))
    config = dict(schema_version=1, model_id=model, spray_tool_axis=axis,
                  mesh_file='tool.stl', mesh_scale_to_m=1., endpoint_confirmed=True,
                  mesh_to_tcp=dict(translation_m=[.12, -.23, .34], quaternion_xyzw=[0., 0., 0., 1.]))
    profile_path = tmp_path / 'tool.json'
    profile_path.write_text(json.dumps(config), encoding='utf-8')
    node.model_id, node.spray_tool_axis = model, axis
    node.spray_eoat_profile = str(profile_path)
    node.test_scope['load_spray_eoat_profile'] = load_spray_eoat_profile
    profile = load_spray_eoat_profile(str(profile_path), model, axis)
    node.path.raw_payload = dict(profile.metadata(), model_id=model, spray_tool_axis=axis)
    node.path.source = dict(node.path.raw_payload)
    node.path.spray_standoff_m = .5
    node.path.safety_approach_offset_m = .5
    node.path.final_retreat_offset_m = .5
    normal = np.array([1., 2., 3.]) / np.sqrt(14.)
    row = SegmentWaypoint('SPRAY', (1., 2., 3.), tuple(normal), (0., 0., 1.), 0., .5, .05, 1)
    node.path.rows = [row]
    candidate = node._build_segment_orientation_candidate(node.path, [1., 0., 0.], name='spray')
    for tcp in [candidate['row_tcp_poses'][1], candidate['safety_tcp_pose'], candidate['retreat_tcp_pose']]:
        q = tcp.orientation
        rotation = quat_to_matrix([q.x, q.y, q.z, q.w])
        endpoint = np.array([tcp.position.x, tcp.position.y, tcp.position.z]) + rotation @ profile.endpoint_tcp_m
        np.testing.assert_allclose(endpoint, np.asarray(row.position) + .5 * normal, atol=1e-12)
        local_axis = np.array([0., 0., 1.]) if axis == '+z' else np.array([0., -1., 0.])
        np.testing.assert_allclose(rotation @ local_axis, -normal, atol=1e-12)
    mesh_path.unlink()
    assert node._spray_eoat_blocker(node.path)


@pytest.mark.parametrize('owned_present', [True, False])
def test_paint_restart_reconciles_existing_moveit_attachment(executor, owned_present):
    node = executor
    scene_transport(node)
    node.process_mode = 'paint'
    node._paint_eoat_collision_object = lambda: NS()
    node.test_scope['GetPlanningScene'] = NS(Request=lambda: NS(components=NS(components=0)))
    node.test_scope['PlanningSceneComponents'] = NS(ROBOT_STATE_ATTACHED_OBJECTS=4)
    query = Future()
    node.get_planning_scene_client = NS(service_is_ready=lambda: True, call_async=lambda req: query)
    node.publish_scene_periodic()
    assert not node.requests
    attached = [NS(object=NS(id='spray_eoat'))] if owned_present else []
    query.set_result(NS(scene=NS(robot_state=NS(attached_collision_objects=attached))))
    node.publish_scene_periodic()
    objects = node.requests[-1].scene.robot_state.attached_collision_objects
    assert [(obj.object.id, obj.object.operation) for obj in objects] == ([('spray_eoat', 1)] if owned_present else [])
    node.response.set_result(NS(success=True))
    assert node.scene_confirmed
    assert node._spray_eoat_published is False


def test_late_startup_inventory_cannot_erase_new_attachment(executor):
    node = executor
    node.process_mode = 'paint'
    node.test_scope['GetPlanningScene'] = NS(Request=lambda: NS(components=NS(components=0)))
    node.test_scope['PlanningSceneComponents'] = NS(ROBOT_STATE_ATTACHED_OBJECTS=4)
    query = Future()
    node.get_planning_scene_client = NS(service_is_ready=lambda: True, call_async=lambda req: query)
    node._spray_eoat_reconcile_paint_startup()
    # A later Spray publish, followed by a return to Paint, supersedes inventory.
    node._spray_eoat_published = True
    query.set_result(NS(scene=NS(robot_state=NS(attached_collision_objects=[]))))
    assert node._spray_eoat_published is True
