"""Physical RB20 roller and front wall; called after SimulationApp initialization."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def usd_angular_speed(radians_per_second):
    # Installed PhysxSchema schema.usda defines angular maxJointVelocity in deg/s.
    return float(np.degrees(radians_per_second))


def usd_angular_gain(si_gain_per_radian):
    # USD angular DriveAPI gains are per degree; PhysX tensor gains are per radian.
    # Matches Isaac Articulation.set_gains(save_to_usd=True).
    return float(si_gain_per_radian*np.pi/180.)


ROLLER_COLLISION_NAME = 'paint_eoat_no_camera_roller_contact_link_paint_eoat_no_camera_roller_contact_collision'


def select_roller_collision(paths, *, robot_root='/World/RB20',
                            collision_name=ROLLER_COLLISION_NAME, body_name=None):
    root=robot_root.rstrip('/')+'/'
    matches=[str(p) for p in paths if str(p).startswith(root)
             and str(p).rsplit('/',1)[-1]==collision_name
             and (body_name is None or body_name in str(p)[len(root):].split('/')[:-1])]
    if len(matches)!=1:
        raise ValueError(f'Expected exactly one named roller collider, found {matches}')
    return matches[0]


def configure_roller_contact(stage, config, friction, *, robot_root='/World/RB20',
                             collision_name=ROLLER_COLLISION_NAME, body_name=None,
                             material_path='/World/RollerContactMaterial'):
    from pxr import Usd, UsdPhysics, UsdShade, PhysxSchema
    paths=[str(p.GetPath()) for p in Usd.PrimRange.Stage(stage,Usd.TraverseInstanceProxies())
           if p.HasAPI(UsdPhysics.CollisionAPI)]
    path=select_roller_collision(paths,robot_root=robot_root,collision_name=collision_name,body_name=body_name)
    prim=stage.GetPrimAtPath(path)
    uninstanced=[]
    # Author only required instance ancestors; unrelated geometry stays intact.
    while prim.IsInstanceProxy():
        ancestor=prim.GetParent()
        while ancestor and not (ancestor.IsInstance() and not ancestor.IsInstanceProxy()):
            ancestor=ancestor.GetParent()
        if not ancestor:
            raise RuntimeError(f'Cannot author roller collider material: {path}')
        uninstanced.append(str(ancestor.GetPath()))
        ancestor.SetInstanceable(False)
        prim=stage.GetPrimAtPath(path)
    material=UsdShade.Material.Define(stage,material_path)
    api=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(friction['static_friction'])
    api.CreateDynamicFrictionAttr(friction['dynamic_friction'])
    api.CreateRestitutionAttr(friction['restitution'])
    physx=PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateCompliantContactAccelerationSpringAttr(config['acceleration_spring'])
    physx.CreateCompliantContactStiffnessAttr(config['stiffness_n_per_m'])
    physx.CreateCompliantContactDampingAttr(config['damping_ns_per_m'])
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,UsdShade.Tokens.strongerThanDescendants,'physics')
    return {'collider':path,'material':str(material.GetPath()),'uninstanced':uninstanced,**config}



def configure_paint_wall_material(stage, wall_path, friction, *,
                                  material_path='/World/PaintWallMaterial'):
    """Match the roller's friction without adding a second compliant spring.

    FixedCuboid's implicit dynamic friction is 1.0. With PhysX's average
    combine rule that would turn the configured roller coefficient .3 into
    .65. The source roller joint is fixed: this remains a sliding contact
    approximation, not a simulated passive rolling bearing.
    """
    from pxr import UsdPhysics, UsdShade
    prim = stage.GetPrimAtPath(wall_path)
    if not prim or not prim.HasAPI(UsdPhysics.CollisionAPI):
        raise ValueError(f'Expected physical paint wall collider: {wall_path}')
    material = UsdShade.Material.Define(stage, material_path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(friction['static_friction'])
    api.CreateDynamicFrictionAttr(friction['dynamic_friction'])
    api.CreateRestitutionAttr(friction['restitution'])
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.strongerThanDescendants, 'physics')
    return dict(wall_material=str(material.GetPath()), friction=dict(friction),
                contact_model='fixed_cylinder_sliding_approximation',
                friction_combine_mode='average',
                effective_pair_dynamic_friction=friction['dynamic_friction'])


def build_scene(root):
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.core.api.objects import FixedCuboid
    from isaacsim.core.api.materials import PhysicsMaterial
    from isaacsim.core.utils.viewports import set_camera_view
    assets=root/'.runtime/rb20_painting/assets'
    from prepare_rb20_painting_assets import verify_assets
    manifest=verify_assets(root,assets)
    config=json.loads((assets/'scene.json').read_text())
    world=World(stage_units_in_meters=1.,physics_dt=1/config['physics_hz'],rendering_dt=1/config['render_hz'])
    stage=omni.usd.get_context().get_stage()
    world.scene.add_default_ground_plane(z_position=config['ground_z'])
    add_reference_to_stage(str(assets/'robot.usd'),'/World/RB20')
    light=UsdLux.DomeLight.Define(stage,'/World/Lighting'); light.CreateIntensityAttr(1500.)
    wall=config['wall']
    material=PhysicsMaterial('/World/PaintMaterial',**config['material'])
    surface=world.scene.add(FixedCuboid('/World/PaintingWall',name=wall['id'],position=np.array(wall['center']),
                                       scale=np.array(wall['size']),size=1.,color=np.array([.78,.82,.86])))
    surface.apply_physics_material(material)
    print('RB20_ROLLER_CONTACT '+json.dumps(configure_roller_contact(stage,config['roller_contact'],config['material'])),flush=True)
    roots=[]; ft_joint=None
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith('/World/RB20'):
            continue
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(str(prim.GetPath()))
            api=PhysxSchema.PhysxArticulationAPI.Apply(prim)
            api.CreateSolverPositionIterationCountAttr(64); api.CreateSolverVelocityIterationCountAttr(8)
            api.CreateEnabledSelfCollisionsAttr(True)
        if prim.IsA(UsdPhysics.RevoluteJoint):
            name=prim.GetName(); drive=UsdPhysics.DriveAPI.Apply(prim,'angular')
            drive.CreateTypeAttr('force')
            drive.CreateStiffnessAttr(usd_angular_gain(config['drive']['stiffness_nm_per_rad']))
            drive.CreateDampingAttr(usd_angular_gain(config['drive']['damping_nms_per_rad']))
            drive.CreateMaxForceAttr(config['drive']['max_torque_nm'])
            drive.CreateTargetPositionAttr(float(np.degrees(manifest['initial_positions'][name])))
            PhysxSchema.JointStateAPI.Apply(prim,'angular').CreatePositionAttr(float(np.degrees(manifest['initial_positions'][name])))
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(usd_angular_speed(manifest['limits'][name]['velocity']))
        if prim.IsA(UsdPhysics.FixedJoint) and prim.GetName()=='ft_sensor_joint':
            ft_joint=UsdPhysics.FixedJoint(prim)
    if len(roots)!=1 or ft_joint is None:
        raise RuntimeError(f'Invalid articulation or F/T boundary: {roots}')
    robot=world.scene.add(SingleArticulation(prim_path=roots[0],name='rb20'))
    world.reset()
    if set(robot.dof_names)!=set(manifest['joint_names']):
        raise RuntimeError(f'Unexpected articulation joints: {robot.dof_names}')
    pose=np.array([manifest['initial_positions'][name] for name in robot.dof_names])
    robot.set_joints_default_state(positions=pose,velocities=np.zeros(6))
    robot.set_joint_positions(pose); robot.set_joint_velocities(np.zeros(6))
    robot.apply_action(ArticulationAction(joint_positions=pose))
    metadata=robot._articulation_view._metadata
    if 'ft_sensor_joint' not in metadata.joint_indices or 'ft_link' not in metadata.link_indices:
        raise RuntimeError('PhysX metadata does not retain named F/T joint and child body')
    ft_row=metadata.joint_indices['ft_sensor_joint']+1
    ft_body=metadata.link_indices['ft_link']
    print('RB20_PHYSICS_BOUNDARY '+json.dumps({'joint_names':list(metadata.joint_names),'body_names':list(metadata.link_names),
                                              'ft_row':ft_row,'ft_body':ft_body}),flush=True)
    # API reports child incoming joint wrench: move from that child joint frame to ft_link.
    source=np.eye(4)
    source[:3,3]=np.array(ft_joint.GetLocalPos1Attr().Get())
    q=ft_joint.GetLocalRot1Attr().Get()
    source[:3,:3]=Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix()
    set_camera_view(eye=np.array([3.,-3.,2.5]),target=np.array([.6,0.,1.]))
    stage.GetRootLayer().Export(str(assets/'scene.usda'))
    return world,robot,manifest,config,ft_row,ft_body,source


def ft_world_transform(robot,body_index):
    from scipy.spatial.transform import Rotation
    raw=np.asarray(robot._articulation_view._physics_view.get_link_transforms())[0,body_index]
    tf=np.eye(4); tf[:3,3]=raw[:3]; tf[:3,:3]=Rotation.from_quat(raw[3:7]).as_matrix()
    return tf
