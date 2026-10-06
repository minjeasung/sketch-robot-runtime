"""Run only with real Humble messages on an isolated CI ROS domain.

No MoveIt/FJT server or physical driver is started. This checks constructors,
real message APIs and the command boundary, not robot integration success.
"""
import os
from pathlib import Path
import pytest
from test_contracts import ROOT, module
from test_model_guard import robot, semantic, limits

rclpy = pytest.importorskip('rclpy')


def test_real_humble_nodes_and_blocked_dispatch(tmp_path, monkeypatch):
    assert os.environ.get('SNUCEM_ISOLATED_ROS_TEST') == '1', 'run in the isolated CI domain only'
    from std_msgs.msg import String
    from moveit_msgs.msg import RobotTrajectory
    from snucem_spray_addon.execution import executor_class
    from sketch_control.wall_projector_node import WallProjectorNode
    from sketch_control.zed_preview_node import ZedPreviewNode
    from sketch_control import sketch_to_waypoints_node as generator
    bundle, _ = module('packaging').build_bundle(ROOT, tmp_path/'smoke.tar.gz')
    up = tmp_path/'upstream'
    up.mkdir()
    cfg = module('config').AddonConfig(str(up), str(tmp_path/'install'), str(tmp_path/'state'))
    installed = module('install').install_bundle(bundle, cfg)
    monkeypatch.setenv('AMENT_PREFIX_PATH', installed['version_root']+os.pathsep+os.environ.get('AMENT_PREFIX_PATH', ''))
    model = module('model')
    path = model.save_descriptor(model.parse_stack_model(robot(), semantic(), limits(), {}), tmp_path/'model')
    params = dict(external_stack_model=str(path), spray_eoat_profile=str(path),
                  process_mode='spray', model_id='rb20_1900es', spray_tool_axis='+z',
                  execution_backend='follow_joint_trajectory', dry_run=False,
                  painting_force_enabled=False, real_painting_enabled=True,
                  spray_motion_test=True)
    args = ['--ros-args']
    for key, value in params.items():
        args.extend(['-p', key+':='+(str(value).lower() if isinstance(value, bool) else str(value))])
    rclpy.init(args=args)
    nodes = []
    try:
        subject = executor_class()()
        nodes.append(subject)
        assert subject.cfg['objects'][0]['name'] == 'snucem_sketch_target'
        assert subject._external_stack_model['base_frame'] == 'link0'
        assert subject._spray_eoat_load().endpoint_tcp_m.tolist() == [0., 0., 0.]
        # Missing live source/model/ownership must stop before any action API.
        assert subject._execute_trajectory_follow_joint(RobotTrajectory()) is False
        assert subject._active_trajectory_goal_token is None
        subject._set_process_mode(String(data='paint'))
        assert subject.process_mode == 'spray'
        nodes.append(WallProjectorNode())
        monkeypatch.setattr(generator, 'WORLD_FRAME', 'link0')
        nodes.append(generator.SketchToWaypointsNode())
        nodes.append(ZedPreviewNode())
    finally:
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown()
