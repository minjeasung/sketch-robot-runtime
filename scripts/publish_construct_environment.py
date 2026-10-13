"""Register beam collisions and camera TF; optionally point head at the beam."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, TransformStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.action import MoveGroup
from controller_manager_msgs.srv import ListControllers
from tf2_ros import StaticTransformBroadcaster
from construct_scene_objects import BEAM_BOXES,SUPPORT_BOXES,ZED_MOUNT_ROTATION,ZED_LEFT_POSITION


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--look-at-beam',action='store_true')
    args,ros=parser.parse_known_args()
    rclpy.init(args=ros)
    node=rclpy.create_node('construct_camera_environment')
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    broadcaster=StaticTransformBroadcaster(node)
    def transform(parent,child,xyz,matrix):
        t=TransformStamped();t.header.frame_id=parent;t.child_frame_id=child
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=map(float,xyz)
        q=Rotation.from_matrix(matrix).as_quat()
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=map(float,q)
        return t
    mount=np.array(ZED_MOUNT_ROTATION)
    broadcaster.sendTransform([
        transform('zed2i_link','zed_left_camera_frame',mount@np.array(ZED_LEFT_POSITION),mount),
        transform('zed_left_camera_frame','zed_left_camera_frame_optical',[0,0,0],[[0,0,1],[-1,0,0],[0,-1,0]])])
    client=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    if not client.wait_for_service(timeout_sec=360):raise RuntimeError('Start Construct MoveIt first')
    request=ApplyPlanningScene.Request();request.scene.is_diff=True
    request.scene.robot_state.is_diff=True
    for name,size,center in BEAM_BOXES+SUPPORT_BOXES:
        obj=CollisionObject();obj.id=name;obj.header.frame_id='World';obj.operation=CollisionObject.ADD
        primitive=SolidPrimitive(type=SolidPrimitive.BOX,dimensions=list(size))
        pose=Pose();pose.position.x,pose.position.y,pose.position.z=center;pose.orientation.w=1.
        obj.primitives=[primitive];obj.primitive_poses=[pose];request.scene.world.collision_objects.append(obj)
    def result(future,timeout=60):
        end=time.monotonic()+timeout
        while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if not future.done():raise TimeoutError('Environment setup timed out')
        return future.result()
    assert result(client.call_async(request)).success
    print('CONSTRUCT_BEAM_COLLISIONS_READY',flush=True)
    if args.look_at_beam:
        controllers=node.create_client(ListControllers,'/controller_manager/list_controllers')
        assert controllers.wait_for_service(timeout_sec=90)
        deadline=time.monotonic()+90
        while time.monotonic()<deadline:
            active={c.name for c in result(controllers.call_async(ListControllers.Request())).controller if c.state=='active'}
            if all(n+'_controller' in active for n in ('left_manipulator','right_manipulator','robot_head')):break
            rclpy.spin_once(node,timeout_sec=.1)
        else:raise TimeoutError('Trajectory controllers are not active')
        action=ActionClient(node,MoveGroup,'/move_action')
        assert action.wait_for_server(timeout_sec=30)
        manifest=json.loads((Path(__file__).resolve().parents[1]/'isaac_assets/construct_manifest.json').read_text())
        # Move the left shoulder away before tilting the head: source pose occludes
        # the camera and left_link2 intersects the head's downward goal otherwise.
        for group,overrides in [('left_manipulator',{'left_manipulator_joint1':.6}),
                                ('robot_head',{'robot_head_rev_joint1':0.,'robot_head_rev_joint2':.7})]:
            target=dict(manifest['initial_positions'],**overrides)
            goal=MoveGroup.Goal();goal.request.group_name=group;goal.request.pipeline_id='ompl'
            goal.request.allowed_planning_time=10.;goal.request.num_planning_attempts=3
            goal.request.max_velocity_scaling_factor=.3;goal.request.max_acceleration_scaling_factor=.3
            goal.request.start_state.is_diff=True
            goal.request.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=j,position=float(target[j]),tolerance_above=.002,tolerance_below=.002,weight=1.) for j in manifest['groups'][group+'_controller']])]
            handle=result(action.send_goal_async(goal));assert handle.accepted
            outcome=result(handle.get_result_async(),120)
            assert outcome.result.error_code.val==1, f"{group} view move failed: {outcome.result.error_code.val}"
            print('CONSTRUCT_VIEW_MOVE_OK '+group,flush=True)
        print('CONSTRUCT_CAMERA_VIEW_POSE_READY',flush=True)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
