#!/usr/bin/env python3
"""Publish the exact imported H-steel triangles to MoveIt and acknowledge application."""
import argparse,json,time,os
from pathlib import Path


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--assets',type=Path,required=True);args=parser.parse_args()
    assets=args.assets.resolve();bundle=json.loads((assets/'bundle.json').read_text())
    import rclpy
    from rclpy.node import Node
    from moveit_msgs.srv import ApplyPlanningScene
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import Mesh,MeshTriangle,SolidPrimitive
    from geometry_msgs.msg import Point,Pose
    rclpy.init();node=Node('platform_environment');client=node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    try:
        if not client.wait_for_service(timeout_sec=90):raise RuntimeError('MoveIt scene service unavailable')
        selected=bundle['selection'];data={'meshes':[]}
        if selected['execution_mode']=='simulation' and selected['process'] in ('spray','weld'):
            data=json.loads((assets/'h_steel_collision.json').read_text())
        request=ApplyPlanningScene.Request();request.scene.is_diff=True
        ground=CollisionObject();ground.id='platform_ground';ground.header.frame_id=bundle['arms'][selected['active_arm']]['base_frame'];ground.operation=CollisionObject.ADD
        ground.primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[100.,100.,.05])]
        pose=Pose();pose.orientation.w=1.;pose.position.z=-.05;ground.primitive_poses=[pose]
        request.scene.world.collision_objects.append(ground)
        for i,part in enumerate(data['meshes']):
            mesh=Mesh(vertices=[Point(x=float(v[0]),y=float(v[1]),z=float(v[2])) for v in part['vertices']],triangles=[MeshTriangle(vertex_indices=t) for t in part['triangles']])
            obj=CollisionObject();obj.id='h_steel_'+str(i);obj.header.frame_id=bundle['arms'][bundle['selection']['active_arm']]['base_frame'];obj.operation=CollisionObject.ADD
            obj.meshes=[mesh];pose=Pose();pose.orientation.w=1.;obj.mesh_poses=[pose];request.scene.world.collision_objects.append(obj)
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=60)
        if not future.done() or not future.result().success:raise RuntimeError('MoveIt rejected H-steel geometry')
        (assets/'environment-ready.json').write_text(json.dumps(dict(asset_revision=bundle['revision'],runtime_revision=bundle.get('runtime_revision',0),applied=True,pid=os.getpid(),mesh_count=len(data['meshes']),ground_applied=True,collision_object_count=len(request.scene.world.collision_objects))))
        rclpy.spin(node)
    finally:
        (assets/'environment-ready.json').unlink(missing_ok=True);node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
