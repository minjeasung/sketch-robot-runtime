"""Named CAD bead surface paths for explicit simulation fixtures, never recognition.

Only long rectangular coplanar surface patches are supported. The line is on the
CAD surface, and its reference vector is that face's outward triangle normal.
No tool-tip, TCP, clearance, or process standoff is applied here.
"""
from collections import Counter
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np


def robot_base_point(description,first_joint,frame_id):
    """Selected first-joint origin in the requested base frame, through fixed mounts."""
    root=ET.fromstring(description);joints={j.get('name'):j for j in root.findall('joint')}
    parents={j.find('child').get('link'):j for j in joints.values()}
    def origin(joint):
        element=joint.find('origin');tf=np.eye(4)
        if element is None:return tf
        tf[:3,3]=[float(x) for x in element.get('xyz','0 0 0').split()]
        r,p,y=[float(x) for x in element.get('rpy','0 0 0').split()]
        cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
        tf[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
        return tf
    def fixed_tf(link,seen=None):
        seen=set() if seen is None else seen
        if link in seen:raise ValueError('cyclic robot frame tree')
        if link not in parents:return np.eye(4)
        seen.add(link);joint=parents[link]
        if joint.get('type')!='fixed':raise ValueError('robot base viewpoint has a movable ancestor')
        return fixed_tf(joint.find('parent').get('link'),seen)@origin(joint)
    if first_joint not in joints:raise ValueError('selected first joint missing')
    if root.find("link[@name='%s']"%frame_id) is None:raise ValueError('fixture frame missing from robot')
    joint=joints[first_joint]
    world=fixed_tf(joint.find('parent').get('link'))@origin(joint)
    return (np.linalg.inv(fixed_tf(frame_id))@world)[:3,3]


def triangles(mesh):
    vertices=np.asarray(mesh['vertices'],dtype=float);faces=np.asarray(mesh['triangles'],dtype=int)
    if vertices.ndim!=2 or vertices.shape[1]!=3 or not np.isfinite(vertices).all():raise ValueError('invalid CAD vertices')
    if faces.ndim!=2 or faces.shape[1]!=3 or np.any(faces<0) or np.any(faces>=len(vertices)):raise ValueError('invalid CAD triangles')
    return vertices,faces,vertices[faces]


def surface_patches(mesh):
    vertices,faces,tri=triangles(mesh);cross=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);areas=np.linalg.norm(cross,axis=1)
    if np.any(areas<1e-14):raise ValueError('degenerate CAD bead triangle')
    normals=cross/areas[:,None];groups=[]
    for index,normal in enumerate(normals):
        for group in groups:
            first=group[0]
            if np.dot(normal,normals[first])>1.-1e-8 and np.max(abs((tri[index]-tri[first,0])@normals[first]))<1e-6:
                group.append(index);break
        else:groups.append([index])
    result=[]
    for group in groups:
        n=normals[group[0]];ids=sorted(set(faces[group].flatten()));points=vertices[ids]
        # Winding must describe an outward face of the CAD bead; never replace
        # the normal with the vector from the bead to the robot.
        if np.dot(points.mean(0)-vertices.mean(0),n)<=1e-8:continue
        counts=Counter(tuple(sorted((int(a),int(b)))) for face in faces[group] for a,b in zip(face,np.roll(face,-1)))
        edges=[e for e,count in counts.items() if count==1]
        if not edges:continue
        lengths=[np.linalg.norm(vertices[b]-vertices[a]) for a,b in edges];longest=max(lengths)
        if longest<.02:continue
        edge=next(e for e,length in zip(edges,lengths) if longest-length<1e-6)
        tangent=vertices[edge[1]]-vertices[edge[0]];tangent/=np.linalg.norm(tangent)
        across=np.cross(n,tangent);across/=np.linalg.norm(across)
        origin=points[0];x=(points-origin)@tangent;y=(points-origin)@across
        low,high=x.min(),x.max();bottom,top=y.min(),y.max();length=high-low;width=top-bottom
        if width<=1e-8 or length/width<4.:continue
        # Require all four corners and matching planar rectangle area; partial,
        # disconnected, notched, or curved patches do not imply a seam.
        corners=[(a,b) for a in (low,high) for b in (bottom,top)]
        if any(np.min(np.hypot(x-a,y-b))>2e-6 for a,b in corners):continue
        if abs(float(areas[group].sum())*.5-length*width)>max(1e-10,length*width*.001):continue
        start=origin+tangent*low+across*(bottom+top)*.5
        end=origin+tangent*high+across*(bottom+top)*.5
        result.append(dict(start=start,end=end,normal=n,triangle_indices=group,length=length))
    return result


def occluded(point,view,scene_triangles):
    direction=view-point;length=np.linalg.norm(direction)
    if length<=1e-8:return True
    direction/=length;a=scene_triangles[:,0];e1=scene_triangles[:,1]-a;e2=scene_triangles[:,2]-a
    h=np.cross(np.broadcast_to(direction,e2.shape),e2);det=np.einsum('ij,ij->i',e1,h)
    valid=abs(det)>1e-12;inv=np.zeros_like(det);inv[valid]=1./det[valid]
    s=point-a;u=inv*np.einsum('ij,ij->i',s,h);q=np.cross(s,e1);v=inv*(q@direction);distance=inv*np.einsum('ij,ij->i',e2,q)
    return bool(np.any(valid&(u>=-1e-9)&(v>=-1e-9)&(u+v<=1.+1e-9)&(distance>1e-5)&(distance<length-1e-5)))


def generate_fixture(geometry,viewpoint_xyz,*,scene_revision,frame_id,max_step_m=.02,ambiguity_margin=.03):
    if not 0<max_step_m<=.02:raise ValueError('fixture maximum step must be at most .02 m')
    view=np.asarray(viewpoint_xyz,dtype=float)
    if view.shape!=(3,) or not np.isfinite(view).all():raise ValueError('finite CAD visibility viewpoint required')
    # Geometry export is in World. The supported generated base frames are
    # colocated with World; caller records the exact selected target frame.
    all_triangles=np.concatenate([triangles(mesh)[2] for mesh in geometry['meshes']])
    seams=[];rejected=[]
    for mesh in sorted(geometry['meshes'],key=lambda m:m['prim']):
        if 'IfcFastener_WELD' not in mesh['prim']:continue
        candidates=[]
        for patch in surface_patches(mesh):
            midpoint=(patch['start']+patch['end'])*.5;direction=view-midpoint;distance=np.linalg.norm(direction)
            if distance<=1e-8:continue
            score=float(np.dot(patch['normal'],direction/distance))
            if score<=.05:continue
            if any(occluded(patch['start']+(patch['end']-patch['start'])*fraction+patch['normal']*1e-6,view,all_triangles) for fraction in (.05,.25,.5,.75,.95)):continue
            candidates.append((score,patch))
        candidates.sort(key=lambda item:item[0],reverse=True)
        if not candidates:
            rejected.append({'prim':mesh['prim'],'reason':'no supported visible exposed CAD face (occluded or unsupported geometry)'});continue
        if len(candidates)>1 and candidates[0][0]-candidates[1][0]<ambiguity_margin:
            rejected.append({'prim':mesh['prim'],'reason':'ambiguous CAD face; explicit orientation required'});continue
        score,patch=candidates[0];count=max(1,math.ceil(patch['length']/max_step_m))
        line=np.linspace(patch['start'],patch['end'],count+1)
        name=mesh['prim'].split('/')[-2]
        seams.append({'seam_id':name,'points_xyz':line.tolist(),'reference_up_vectors':[patch['normal'].tolist()],
            'provenance':{'source_prim':mesh['prim'],'triangle_indices':patch['triangle_indices'],'normal_source':'outward_coplanar_cad_triangles','path_source':'exposed_rectangular_face_midline','view_score':score,'tcp_offset_applied_m':0.,'max_step_m':max_step_m}})
    if not seams:raise ValueError('No unambiguous visible CAD seams: '+json.dumps(rejected))
    return {'schema_version':1,'scene_revision':scene_revision,'frame_id':frame_id,'length_unit':'m','seams':seams,
        'provenance':{'source':'simulation_cad_fixture','recognition':False,'viewpoint_xyz':view.tolist(),'collision_geometry_sha256':hashlib.sha256(json.dumps(geometry,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'rejected_features':rejected}}



def horizontal_lower_fillet(geometry):
    """Explicit demo seam on the front web/lower flange of the axis-aligned CAD beam.

    This is a user-selected simulation joint, not an existing CAD weld bead or
    image recognition result. Leave 100 mm at each end for the end connections.
    """
    matches=[m for m in geometry['meshes'] if '/IfcBeam_BEAM/' in m['prim']]
    if len(matches)!=1:raise ValueError('Expected one named horizontal CAD beam')
    mesh=matches[0];vertices,_,tri=triangles(mesh)
    cross=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);sizes=np.linalg.norm(cross,axis=1)
    valid=sizes>1e-12;normals=np.zeros_like(cross);normals[valid]=cross[valid]/sizes[valid,None]
    def planes(axis):
        groups={}
        for i in np.flatnonzero(valid & (normals[:,axis]>.999999)):
            key=round(float(tri[i,:,axis].mean()),5)
            groups.setdefault(key,[]).append(i)
        return groups
    front=planes(1);upper=planes(2)
    if not front or not upper:raise ValueError('Expected axis-aligned front web and upper flange faces')
    web=max(front,key=lambda key:float(sizes[front[key]].sum()))
    bottom=min(z for z in upper if z>vertices[:,2].min()+.001)
    low,high=vertices[:,0].min()+.1,vertices[:,0].max()-.1
    if high-low<.02:raise ValueError('Horizontal beam too short for demo end clearances')
    # Fit the actual almost-axis-aligned CAD planes, retaining micrometre-scale
    # source rotation rather than snapping the joint to rounded coordinates.
    normals_pair=[];offsets=[]
    for indices in (front[web],upper[bottom]):
        normal=cross[indices].sum(axis=0);normal/=np.linalg.norm(normal)
        center=np.average(tri[indices].mean(axis=1),axis=0,weights=sizes[indices])
        normals_pair.append(normal);offsets.append(float(normal@center))
    plane=np.asarray(normals_pair);offsets=np.asarray(offsets)
    def at(x):return np.r_[x,np.linalg.solve(plane[:,1:],offsets-plane[:,0]*x)]
    points=np.linspace(at(high),at(low),math.ceil((high-low)/.02)+1)
    from scipy.spatial.transform import Rotation
    tangent=points[-1]-points[0];tangent/=np.linalg.norm(tangent)
    normal=np.array([0.,2**-.5,2**-.5]);z=-normal
    y=np.cross(z,tangent);y/=np.linalg.norm(y);x=np.cross(y,z)
    rotation=Rotation.from_matrix(np.column_stack([x,y,z]))*Rotation.from_euler('z',90.,degrees=True)
    return dict(seam_id='horizontal_lower_front_fillet',points_xyz=points.tolist(),
        quaternions_xyzw=[rotation.as_quat().tolist()],
        provenance=dict(source_prim=mesh['prim'],recognition=False,
            path_source='web_lower_flange_plane_intersection',
            normal_source='front_web_and_lower_flange_outward_bisector',
            web_y_m=web,lower_flange_top_z_m=bottom,end_inset_m=.1,tcp_offset_applied_m=0.,tool_roll_deg=90.))


def camera_viewpoint(scene):
    """Use the same rendered ZED eye as the simulator, never an arm-base proxy."""
    try:
        transform=np.asarray(scene['zed_world_optical'],dtype=float)
    except (KeyError,TypeError,ValueError) as exc:
        raise ValueError('generated ZED camera transform required') from exc
    if ('zed' not in scene.get('camera_names',[]) or transform.shape!=(4,4)
        or not np.isfinite(transform).all() or not np.allclose(transform[3],[0,0,0,1],atol=1e-9)
        or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-7)
        or not np.isclose(np.linalg.det(transform[:3,:3]),1.,atol=1e-7)):
        raise ValueError('generated ZED camera must have a finite rigid transform')
    return transform[:3,3].copy()


def write_fixture(assets):
    """Write a fixture only for a generated, explicitly simulated welding bundle."""
    from pathlib import Path
    assets=Path(assets);bundle=json.loads((assets/'bundle.json').read_text());selection=bundle['selection']
    if selection['execution_mode']!='simulation' or selection['process']!='weld':raise ValueError('CAD fixture requires explicit simulation welding mode')
    arm=bundle['arms'][selection['active_arm']];description=(assets/'robot.urdf').read_text()
    geometry=json.loads((assets/'h_steel_collision.json').read_text())
    if geometry.get('frame_id')!='World' or arm['base_frame'] not in ('World','link0'):raise ValueError('unsupported generated fixture coordinate frame')
    # Platform importer places the URDF root at the World origin. The RB10 root
    # is link0; commercial root is World. Refuse a non-root relabeling.
    robot=ET.fromstring(description);children={j.find('child').get('link') for j in robot.findall('joint')}
    roots={link.get('name') for link in robot.findall('link')}-children
    if roots!={arm['base_frame']}:raise ValueError('fixture target must be the generated robot root at World origin')
    base=robot_base_point(description,arm['joints'][0],arm['base_frame'])
    scene=json.loads((assets/'scene.json').read_text());view=camera_viewpoint(scene)
    if scene.get('weld_demo',{}).get('seam_id')=='horizontal_lower_front_fillet':
        seam=horizontal_lower_fillet(geometry)
        fixture=dict(schema_version=1,scene_revision=bundle['revision'],frame_id=arm['base_frame'],
            length_unit='m',seams=[seam],provenance=dict(source='simulation_cad_fixture',
            recognition=False,viewpoint_xyz=view.tolist(),
            collision_geometry_sha256=hashlib.sha256(json.dumps(geometry,sort_keys=True,separators=(',',':')).encode()).hexdigest()))
    else:
        fixture=generate_fixture(geometry,view,scene_revision=bundle['revision'],frame_id=arm['base_frame'])
    fixture['provenance'].update(viewpoint_source='generated_zed_optical_origin',selected_robot_base_xyz=base.tolist(),selected_arm=selection['active_arm'],robot_base_joint=arm['joints'][0],geometry_frame='World',target_frame=arm['base_frame'],frame_binding='generated_robot_root_at_world_origin')
    manifest=assets/'h_steel_manifest.json'
    if manifest.is_file():
        source=json.loads(manifest.read_text());fixture['provenance'].update(source_usd=source.get('source'),source_sha256=source.get('source_sha256'),placement_translation_m=source.get('translation_m'))
    target=assets/'weld_fixture.json';temporary=target.with_suffix('.tmp')
    temporary.write_text(json.dumps(fixture,indent=2)+'\n');temporary.replace(target)
    return fixture


if __name__=='__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--assets',required=True);args=parser.parse_args()
    result=write_fixture(args.assets)
    print(json.dumps({'seams':[s['seam_id'] for s in result['seams']],'provenance':result['provenance']},indent=2))
