from geometry_msgs.msg import Pose, PoseArray

from sketch_control.moveit_executor import BASE_FRAME, MoveItExecutor


class _Logger:
    def info(self, *_args, **_kwargs):
        pass

    def warn(self, *_args, **_kwargs):
        pass

    def error(self, *_args, **_kwargs):
        pass


def _executor_stub():
    node = object.__new__(MoveItExecutor)
    node.executing = False
    node._accepted_plan_hash = "hash-1"
    node._accepted_plan_path_id = "1234"
    node._waypoints_path_id = ""
    node._waypoints_received_at = 0.0
    node.current_waypoints = []
    node._canonical_world_frame = lambda frame: frame
    node._lookup_transform_to_base = lambda *_args, **_kwargs: None
    node._transform_pose_msg_to_base = lambda pose, _transform: pose
    node.get_logger = lambda: _Logger()
    return node


def _waypoints(path_id: int):
    message = PoseArray()
    message.header.frame_id = BASE_FRAME
    message.header.stamp.sec = 0
    message.header.stamp.nanosec = path_id
    message.poses = [Pose()]
    return message


def test_posearray_arriving_after_plan_status_preserves_matching_acceptance():
    node = _executor_stub()

    node.on_waypoints(_waypoints(1234))

    assert node._waypoints_path_id == "1234"
    assert node._accepted_plan_path_id == "1234"
    assert node._accepted_plan_hash == "hash-1"


def test_posearray_with_new_identity_invalidates_previous_acceptance():
    node = _executor_stub()

    node.on_waypoints(_waypoints(5678))

    assert node._waypoints_path_id == "5678"
    assert node._accepted_plan_path_id == ""
    assert node._accepted_plan_hash == ""
