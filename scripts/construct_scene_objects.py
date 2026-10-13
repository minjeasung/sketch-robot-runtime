"""Shared metric scene geometry: (name, dimensions xyz, center xyz)."""
BEAM_BOXES = [
    ('h_beam_bottom', (1.5, .3, .02), (0., -1.30, .91)),
    ('h_beam_web', (1.5, .01, .26), (0., -1.30, 1.05)),
    ('h_beam_top', (1.5, .3, .02), (0., -1.30, 1.19)),
]
SUPPORT_BOXES = [
    ('support_left', (.10, .42, .88), (-.55, -1.30, .44)),
    ('support_right', (.10, .42, .88), (.55, -1.30, .44)),
    ('support_left_pad', (.20, .42, .02), (-.55, -1.30, .89)),
    ('support_right_pad', (.20, .42, .02), (.55, -1.30, .89)),
]
# Official +X forward,+Y left,+Z up -> CAD +Z forward,-X left,-Y up.
ZED_MOUNT_ROTATION = [[0.,-1.,0.],[0.,0.,-1.],[1.,0.,0.]]
ZED_LEFT_POSITION = [.02,.06,.015]
ZED_RIGHT_POSITION = [.02,-.06,.015]


def add_steel_surface_appearance(stage, root):
    """Appearance-only meshes over the unchanged physical beam boxes."""
    from pxr import UsdGeom, UsdShade, Sdf, Gf
    # Physical mill-scale patches/scratches give stereo real image features.
    # These are surface appearance only: original box collision geometry stays
    # unchanged, and depth continues to come from the SDK stereo estimator.
    material=UsdShade.Material.Define(stage,'/World/Workpiece/Steel')
    shader=UsdShade.Shader.Define(stage,'/World/Workpiece/Steel/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.24,.28,.32))
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.8)
    shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(.3)
    texture=UsdShade.Shader.Define(stage,'/World/Workpiece/Steel/ColorTexture')
    texture.CreateIdAttr('UsdUVTexture')
    texture.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(root/'assets/construct/steel_mill_scale.png')))
    texture.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
    texture.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('repeat')
    texture.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('repeat')
    uv=UsdShade.Shader.Define(stage,'/World/Workpiece/Steel/UV')
    uv.CreateIdAttr('UsdPrimvarReader_float2')
    uv.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
    texture.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(uv.ConnectableAPI(),'result')
    shader.GetInput('diffuseColor').ConnectToSource(texture.ConnectableAPI(),'rgb')
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    # Explicit UVs on a visual mesh avoid renderer-dependent implicit Cube UVs.
    # The tile covers 0.4 m, so mill scale has the same size on every beam face.
    points=[(-.5,-.5,-.5),(.5,-.5,-.5),(.5,.5,-.5),(-.5,.5,-.5),
            (-.5,-.5,.5),(.5,-.5,.5),(.5,.5,.5),(-.5,.5,.5)]
    faces=[0,3,2,1,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7]
    for name,size,center in BEAM_BOXES:
        UsdGeom.Imageable(stage.GetPrimAtPath('/World/Workpiece/'+name)).MakeInvisible()
        mesh=UsdGeom.Mesh.Define(stage,'/World/Workpiece/SteelSurfaces/'+name)
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4]*6)
        mesh.CreateFaceVertexIndicesAttr(faces)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.AddTranslateOp().Set(Gf.Vec3d(*center))
        mesh.AddScaleOp().Set(Gf.Vec3f(*size))
        sx,sy,sz=size
        coords=[]
        for width,height in ((sy,sx),(sx,sy),(sx,sz),(sy,sz),(sx,sz),(sy,sz)):
            coords.extend([(0.,0.),(width/.4,0.),(width/.4,height/.4),(0.,height/.4)])
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,
                                              UsdGeom.Tokens.faceVarying).Set(coords)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def add_scene_objects(stage, root):
    import os
    import numpy as np
    from pxr import UsdGeom, UsdPhysics, Gf, UsdShade, Sdf
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.extensions import enable_extension
    import omni.kit.app
    import omni.graph.core as og
    import usdrt.Sdf
    from pathlib import Path
    for name,size,center in BEAM_BOXES+SUPPORT_BOXES:
        cube=UsdGeom.Cube.Define(stage,'/World/Workpiece/'+name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*center))
        cube.AddScaleOp().Set(Gf.Vec3f(*size))
        cube.CreateDisplayColorAttr([Gf.Vec3f(.24,.31,.38) if name.startswith('h_beam') else Gf.Vec3f(.35,.16,.04)])
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    add_steel_surface_appearance(stage, root)
    ext=Path(os.environ.get('ZED_ISAAC_EXTENSION',root.parent/'zed-isaac-sim/exts'))
    omni.kit.app.get_app().get_extension_manager().add_path(str(ext))
    enable_extension('sl.sensor.camera')
    link=next(p for p in stage.Traverse() if p.GetName()=='zed2i_link' and str(p.GetPath()).startswith('/World/Construct/Geometry'))
    sensor_path='/World/ZEDSensor'
    sensor=add_reference_to_stage(str(ext/'sl.sensor.camera/data/usd/ZED_2i.usdc'),sensor_path)
    # Place sensor optical centers slightly in front of the CAD face; retain original robot mesh.
    m=np.eye(4);m[:3,:3]=np.asarray(ZED_MOUNT_ROTATION).T
    # Official standalone asset is a dynamic rigid body. The robot already owns
    # the physical camera link: a second unconstrained body would fall through it.
    sensor.RemoveAPI(UsdPhysics.RigidBodyAPI)
    sensor.RemoveAPI(UsdPhysics.MassAPI)
    mount=Gf.Matrix4d(m.tolist())
    camera_ops=[]
    camera_view=None
    def update_pose():
        world_pose=mount * UsdGeom.XformCache().GetLocalToWorldTransform(link)
        positions=[];orientations=[]
        for op,local_pose in camera_ops:
            transform=local_pose * world_pose
            op.Set(transform)
            positions.append(list(transform.ExtractTranslation()))
            rotation=transform.ExtractRotationQuat()
            orientations.append([rotation.GetReal(),*rotation.GetImaginary()])
        if camera_view is not None:
            # Keep RTX/Fabric and exported USD at the same post-physics pose.
            camera_view.set_world_poses(positions=np.asarray(positions),orientations=np.asarray(orientations),usd=False)
    for side,pos in [('Left',ZED_LEFT_POSITION),('Right',ZED_RIGHT_POSITION)]:
        camera=UsdGeom.Camera(stage.GetPrimAtPath(sensor_path+'/base_link/ZED_2i/Camera'+side))
        camera.ClearXformOpOrder()
        camera.SetResetXformStack(True)
        local_pose=Gf.Matrix4d().SetRotate(Gf.Quatd(.5,.5,-.5,-.5))
        local_pose.SetTranslateOnly(Gf.Vec3d(*pos))
        camera_ops.append((camera.AddTransformOp(),local_pose))
        camera.MakeVisible()
        camera.CreateClippingRangeAttr(Gf.Vec2f(.01,1000.))
    update_pose()
    from isaacsim.core.prims import XFormPrim
    camera_view=XFormPrim(prim_paths_expr=[sensor_path+'/base_link/ZED_2i/Camera'+side for side in ('Left','Right')],reset_xform_properties=False)
    update_pose()
    UsdGeom.Imageable(stage.GetPrimAtPath(sensor_path+'/base_link/ZED_2i/Meshes')).MakeInvisible()
    keys=og.Controller.Keys
    og.Controller.edit({'graph_path':'/World/ConstructZED','evaluator_name':'execution'}, {
        keys.CREATE_NODES:[('Tick','omni.graph.action.OnPlaybackTick'),('ZED','sl.sensor.camera.ZED_Camera')],
        keys.CONNECT:[('Tick.outputs:tick','ZED.inputs:execIn')],
        keys.SET_VALUES:[('ZED.inputs:cameraPrim',[usdrt.Sdf.Path(sensor_path)]),
            ('ZED.inputs:cameraModel','ZED_2i'),('ZED.inputs:resolution','HD720'),
            ('ZED.inputs:fps',15),('ZED.inputs:transportLayerMode','IPC'),
            ('ZED.inputs:streamingPort',30000),('ZED.inputs:streamDepth',False),
            ('ZED.inputs:applyZedSim2Real',False)]})
    return sensor_path,update_pose
