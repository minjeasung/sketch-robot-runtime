"""Construct camera adaptations keep the physical-camera contract unchanged."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.context import Context
from rclpy.node import Node
import yaml
from launch import LaunchContext
from sketch_control import outpost_bridge_node as bridge
from sketch_control.outpost_camera import FrameGuard


def test_capture_policy_preserves_source_timestamp():
    policy = bridge.FrameStampPolicy('capture', use_sim_time=False)
    assert policy.stamp(1_790_000_000_000_000_000, 12) == 1_790_000_000_000_000_000


def test_sim_receipt_clock_needs_advancing_clock_and_explicit_simulation():
    with pytest.raises(ValueError, match='use_sim_time'):
        bridge.FrameStampPolicy('ros_clock', use_sim_time=False)
    policy = bridge.FrameStampPolicy('ros_clock', use_sim_time=True)
    assert policy.stamp(1_790_000_000_000_000_000, 0) is None
    assert policy.stamp(1_790_000_000_000_000_000, 12_000_000_000) == 12_000_000_000
    assert policy.stamp(1_790_000_000_000_000_001, 12_000_000_000) is None
    assert policy.stamp(1_790_000_000_000_000_002, 11_000_000_000) is None
    assert policy.stamp(1_790_000_000_000_000_003, 13_000_000_000) == 13_000_000_000
    # Selecting simulation publication stamps must not relax wire freshness.
    assert not FrameGuard().accept(1, 1, now_ns=1_790_000_000_000_000_000)


def test_unknown_stamp_policy_rejected():
    with pytest.raises(ValueError, match='stamp_mode'):
        bridge.FrameStampPolicy('latest_tf', use_sim_time=True)


def test_simulation_publications_share_one_receipt_stamp():
    class Publisher:
        def __init__(self):
            self.messages = []

        def get_subscription_count(self):
            return 1

        def publish(self, message):
            self.messages.append(message)

    publishers = [Publisher() for _ in range(5)]
    previews = []
    node = SimpleNamespace(
        stamp_policy=bridge.FrameStampPolicy('ros_clock', use_sim_time=True),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=12_345_000_000)),
        _camera_info=bridge.OutpostBridge._camera_info,
        _zed_identity_pub=Publisher(),
    )
    camera = dict(name='zed', image_frame='zed_left_camera_frame_optical',
                  cloud_frame='zed_left_camera_frame', point_stride=1,
                  publishers=publishers, status={'generation': 1, 'intrinsics': dict(fx=2, fy=2, cx=0, cy=0)},
                  preview=SimpleNamespace(publish=lambda rgb, header: previews.append(header)))
    bridge.OutpostBridge.publish_frame(node, camera, (
        1_790_000_000_000_000_000, 1, np.zeros((2, 2, 3), np.uint8),
        np.ones((2, 2), np.float32), np.ones((2, 2, 3), np.float32)))
    headers = [publisher.messages[0].header for publisher in publishers] + previews
    assert len(headers) == 6
    assert all((header.stamp.sec, header.stamp.nanosec) == (12, 345_000_000) for header in headers)


def test_external_camera_tf_does_not_read_real_calibration():
    path = Path(__file__).resolve().parents[1] / 'launch/rb10_perception_sketch.launch.py'
    spec = importlib.util.spec_from_file_location('perception_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['publish_zed_static_tf'] = 'false'
    # No calibration or pose parameters exist: external TF must bypass them.
    assert module._make_zed_static_tfs(context) == []


def test_construct_node_specific_config_wins_in_actual_ros_parameter_resolution():
    root = Path(__file__).resolve().parents[3]
    path = root / 'scripts/construct_perception.launch.py'
    spec = importlib.util.spec_from_file_location('construct_perception_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original_path = root / 'src/sketch_control/config/painting_system_real.yaml'
    original_text = original_path.read_text()
    original = yaml.safe_load(original_text)
    values = dict(model_id='construct_left', spray_tool_axis='+z',
                  spray_eoat_profile='/tmp/construct-eoat.json',
                  spray_footprint_width_m=.35, spray_overlap=.3,
                  spray_speed_mps=.02, spray_standoff_m=.3)
    merged = module.construct_perception_config(original, values)
    launch_context = LaunchContext()
    launch_context.launch_configurations['process_mode'] = 'spray'
    launch_context.launch_configurations.update({key: str(value) for key, value in values.items()})
    actions = module._launch_perception(launch_context, root)
    # Read the very file the original included launch will receive, not a
    # separately assembled approximation of its ROS arguments.
    include_args = dict(actions[1].launch_arguments)
    config_path = Path(include_args['painting_config_file'])
    assert yaml.safe_load(config_path.read_text()) == merged
    context = Context()
    context.init(args=[], domain_id=232)
    nodes = []
    try:
        for name in ('wall_projector', 'sketch_to_waypoints'):
            node = Node(name, context=context, enable_rosout=False, start_parameter_services=False,
                        cli_args=['--ros-args', '--params-file', str(config_path)],
                        automatically_declare_parameters_from_overrides=True)
            nodes.append(node)
            assert node.get_parameter('process_mode').value == 'spray'
            assert node.get_parameter('use_sim_time').value is True
        projector, generator = nodes
        assert projector.get_parameter('front_view_source').value == 'zed'
        assert generator.get_parameter('model_id').value == 'construct_left'
        assert generator.get_parameter('eoat_segment_frame').value == 'World'
        assert generator.get_parameter('spray_standoff_m').value == .3
        assert generator.get_parameter('spray_eoat_profile').value == '/tmp/construct-eoat.json'
        assert generator.get_parameter('dry_run').value is False
        assert generator.get_parameter('real_painting_enabled').value is False
    finally:
        for node in nodes:
            node.destroy_node()
        context.shutdown()
        config_path.unlink()
    assert original_path.read_text() == original_text
    assert original['wall_projector']['ros__parameters']['front_view_source'] == 'd405'
    # Tuning unrelated to the Construct overrides survives the merge.
    assert merged['admittance_controller'] == original['admittance_controller']
