"""Isaac-only welded-path display and explicit demo torch (SI units)."""
import math
import json
import numpy as np


def tool_geometry(profile):
    tip=np.asarray(profile['tip_translation_m'],float); length=float(np.linalg.norm(tip))
    if length<1e-6: raise ValueError('torch needs nonzero length')
    direction=tip/length; axis=np.cross([0.,0.,1.],direction); dot=float(direction[2])
    if dot < -.999999: q=[1.,0.,0.,0.]
    else:
        q=np.r_[axis,1+dot]; q=(q/np.linalg.norm(q)).tolist()
    return (tip/2).tolist(),length,q,float(profile['radius_m'])


def mounted_tool_geometry(urdf,tcp,profile):
    import xml.etree.ElementTree as ET
    from scipy.spatial.transform import Rotation
    joint=next(j for j in ET.parse(urdf).getroot().findall('joint') if j.find('child').get('link')==tcp)
    if joint.get('type')!='fixed': raise ValueError('fixed TCP mount required')
    origin=joint.find('origin'); xyz=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
    r=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' '))
    center,length,q,radius=tool_geometry(profile)
    return joint.find('parent').get('link'),(xyz+r.apply(center)).tolist(),(r*Rotation.from_quat(q)).as_quat().tolist(),length,radius


class PreviewState:
    def __init__(self): self.payload=None; self.dirty=False
    def accept(self,payload):
        self.payload=None if payload.get('invalidated') else payload
        self.dirty=True


def attach_weld_scene(stage,root,model):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from pxr import UsdGeom,Gf,UsdPhysics
    if not rclpy.ok(): rclpy.init()
    node=Node('construct_weld_display'); node.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    profile=json.loads((root/'src/sketch_control/config/weld_sim_profile.json').read_text())
    parent,center,q,length,radius=mounted_tool_geometry(root/'isaac_assets/construct_control.urdf',model.replace('construct_','')+'_manipulator_ee_point',profile)
    bodies=[p for p in stage.Traverse() if p.GetName()==parent and p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if len(bodies)!=1: raise ValueError('unique EOAT rigid body required')
    # A collider under the existing rigid body moves with its articulation; no
    # independent static collider can push the mounting link via delayed TF.
    cylinder=UsdGeom.Cylinder.Define(stage,str(bodies[0].GetPath())+'/WeldTorch')
    cylinder.CreateRadiusAttr(radius); cylinder.CreateHeightAttr(length); cylinder.CreateAxisAttr('Z')
    cylinder.AddTranslateOp().Set(Gf.Vec3d(*center)); cylinder.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
    cylinder.CreateDisplayColorAttr([Gf.Vec3f(.8,.55,.15)])
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    pending=PreviewState(); subscriptions=[]
    def receive(msg):
        try: pending.accept(json.loads(msg.data))
        except (TypeError,ValueError): pass
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions.append(node.create_subscription(String,'/welding/preview',receive,qos))
    colors=[(.2,.8,1.),(1.,.6,.2),(.7,.4,1.)]
    def curve(name,points,color,width=.002):
        if len(points)<2: return
        c=UsdGeom.BasisCurves.Define(stage,'/World/WeldPreview/'+name)
        c.CreateTypeAttr('linear'); c.CreateCurveVertexCountsAttr([len(points)])
        c.CreatePointsAttr([Gf.Vec3f(*p) for p in points]); c.CreateWidthsAttr([width]); c.SetWidthsInterpolation('constant')
        c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    def update(dt):
        rclpy.spin_once(node,timeout_sec=0.)
        if pending.dirty:
            stage.RemovePrim('/World/WeldPreview'); pending.dirty=False
            if pending.payload:
                for i,segment in enumerate(pending.payload.get('segments',[])):
                    curve('Path'+str(i),segment.get('tip_xyz_m',[]),colors[i%3])
                    curve('Center'+str(i),segment.get('center_xyz_m',[]),(.5,.5,.5),.001)
                    for j,arrow in enumerate(segment.get('axes',[])): curve(f'Axis{i}_{j}',arrow,(.3,1.,.3),.001)
    return update
