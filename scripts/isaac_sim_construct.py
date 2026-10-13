"""Construct dual-arm scene, executed inside Isaac Sim 6.1."""
import json
import os
from pathlib import Path
import numpy as np
import omni.graph.core as og
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
import usdrt.Sdf
from isaacsim.core.utils.extensions import enable_extension

enable_extension('isaacsim.ros2.bridge')
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view

ROOT = Path(os.environ['SKETCH_WORKSPACE'])
(ROOT/'logs').mkdir(parents=True, exist_ok=True)
manifest = json.loads((ROOT/'isaac_assets/construct_manifest.json').read_text())
world = World(stage_units_in_meters=1.0, physics_dt=1/100,
              rendering_dt=1/float(os.environ.get('CONSTRUCT_RENDER_HZ', '20')))
stage = omni.usd.get_context().get_stage()
world.scene.add_default_ground_plane(z_position=-0.02)
add_reference_to_stage(str(ROOT/'isaac_assets/construct.usd'), '/World/Construct')
light = UsdLux.DomeLight.Define(stage, '/World/Lighting')
light.CreateIntensityAttr(1500.0)
roots = []
for prim in stage.Traverse():
    if not str(prim.GetPath()).startswith('/World/Construct'):
        continue
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        roots.append(str(prim.GetPath()))
        api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        api.CreateSolverPositionIterationCountAttr(32)
        api.CreateSolverVelocityIterationCountAttr(4)
    if prim.IsA(UsdPhysics.RevoluteJoint):
        name = prim.GetName()
        PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(
            float(np.degrees(manifest['limits'][name]['velocity'])))
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateTypeAttr('force')
        # Weld's 1 Hz millimetre weave needs less position-drive lag.
        # Preserve the original gains for paint/spray.
        drive.CreateStiffnessAttr(100000.0 if os.environ.get('CONSTRUCT_PROCESS_MODE') == 'weld' else 50000.0)
        drive.CreateDampingAttr(3000.0)
        drive.CreateMaxForceAttr(manifest['limits'][name]['effort'])
        drive.CreateTargetPositionAttr(float(np.degrees(manifest['initial_positions'][name])))
        state = PhysxSchema.JointStateAPI.Apply(prim, 'angular')
        state.CreatePositionAttr(float(np.degrees(manifest['initial_positions'][name])))
        state.CreateVelocityAttr(0.0)
assert len(roots) == 1, roots
robot = world.scene.add(SingleArticulation(prim_path=roots[0], name='construct'))
if os.environ.get('CONSTRUCT_PROCESS_MODE') == 'weld':
    import sys
    sys.path.insert(0, str(ROOT/'scripts'))
    from construct_weld_scene import attach_weld_scene
    update_weld_scene=attach_weld_scene(stage,ROOT,os.environ.get('CONSTRUCT_WELD_MODEL','construct_left'))
    world.add_physics_callback('construct_welding',update_weld_scene)
world.reset()
assert set(robot.dof_names) == set(manifest['joint_names']), robot.dof_names
pose = np.array([manifest['initial_positions'][n] for n in robot.dof_names])
robot.set_joints_default_state(positions=pose, velocities=np.zeros(14))
robot.set_joint_positions(pose)
robot.set_joint_velocities(np.zeros(14))
robot.apply_action(ArticulationAction(joint_positions=pose))
keys = og.Controller.Keys
og.Controller.edit({'graph_path':'/World/ConstructROS','evaluator_name':'execution',
                    'pipeline_stage':og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND}, {
    keys.CREATE_NODES:[
        ('Tick','isaacsim.core.nodes.OnPhysicsStep'),
        ('Time','isaacsim.core.nodes.IsaacReadSimulationTime'),
        ('Context','isaacsim.ros2.bridge.ROS2Context'),
        ('State','isaacsim.ros2.bridge.ROS2PublishJointState'),
        ('Command','isaacsim.ros2.bridge.ROS2SubscribeJointState'),
        ('Control','isaacsim.core.nodes.IsaacArticulationController'),
        ('Clock','isaacsim.ros2.bridge.ROS2PublishClock'),
    ],
    keys.CONNECT:[
        ('Context.outputs:context','State.inputs:context'),
        ('Context.outputs:context','Command.inputs:context'),
        ('Context.outputs:context','Clock.inputs:context'),
        ('Tick.outputs:step','State.inputs:execIn'),
        ('Tick.outputs:step','Command.inputs:execIn'),
        ('Command.outputs:execOut','Control.inputs:execIn'),
        ('Tick.outputs:step','Clock.inputs:execIn'),
        ('Time.outputs:simulationTime','State.inputs:timeStamp'),
        ('Time.outputs:simulationTime','Clock.inputs:timeStamp'),
        ('Command.outputs:jointNames','Control.inputs:jointNames'),
        ('Command.outputs:positionCommand','Control.inputs:positionCommand'),
    ],
    keys.SET_VALUES:[
        ('Context.inputs:useDomainIDEnvVar', True),
        ('State.inputs:topicName','/joint_states'),
        ('State.inputs:targetPrim',[usdrt.Sdf.Path(roots[0])]),
        ('Command.inputs:topicName','/isaac_joint_command'),
        ('Control.inputs:robotPath',roots[0]),
        ('Clock.inputs:topicName','/clock'),
    ],
})
set_camera_view(eye=np.array([3.3,-4.0,2.7]), target=np.array([0.0,0.0,0.9]))
# Optional SDK camera profile preserves the lean robot-only launcher.
if os.environ.get('CONSTRUCT_ZED', '0') == '1':
    import sys
    sys.path.insert(0,str(ROOT/'scripts'))
    from construct_scene_objects import add_scene_objects
    zed_path,update_zed_pose=add_scene_objects(stage,ROOT)
    print('CONSTRUCT_ZED_READY '+zed_path,flush=True)
    set_camera_view(eye=np.array([3.3,-4.5,3.0]),target=np.array([0.,-.6,1.]))
scene = ROOT/'isaac_assets/construct_scene.usda'
stage.GetRootLayer().Export(str(scene))
omni.timeline.get_timeline_interface().play()
print('CONSTRUCT_SCENE_READY '+json.dumps({'articulation':roots[0], 'joints':robot.dof_names, 'scene':str(scene)}), flush=True)

_frame = 0
def _heartbeat(dt):
    global _frame
    _frame += 1
    if _frame % 100 == 0:
        (ROOT/'logs/construct-heartbeat.json').write_text(json.dumps({'frame':_frame,'time':world.current_time, 'positions':dict(zip(robot.dof_names,robot.get_joint_positions().tolist()))}))
world.add_physics_callback('construct_health', _heartbeat)
