"""User-selected seam projection from an exact physical RGB-D exposure."""
from collections import OrderedDict
import time
import numpy as np
from .mixed_frame_sync import FrameBinding, SelectionLease
from .outpost_camera import intrinsics as checked_intrinsics
from .weld_contract import Seam, SeamCatalog


def pixel_points(polyline_px,depth_m,intrinsics,*,max_depth_step_m=.05):
    depth=np.asarray(depth_m,dtype=float)
    polyline=np.asarray(polyline_px,dtype=float)
    if depth.ndim!=2 or polyline.ndim!=2 or polyline.shape[1:]!=(2,) or not 2<=len(polyline)<=1024 or not np.isfinite(polyline).all():
        raise ValueError('finite selected pixel polyline and depth image required')
    h,w=depth.shape
    if np.any(polyline<0) or np.any(polyline[:,0]>w-1) or np.any(polyline[:,1]>h-1): raise ValueError('polyline outside selected image')
    samples=[]
    for a,b in zip(polyline,polyline[1:]):
        distance=np.linalg.norm(b-a)
        if distance<.5: raise ValueError('duplicate seam pixels')
        segment=np.linspace(a,b,int(np.ceil(distance))+1)
        samples.extend(segment if not samples else segment[1:])
    pixels=np.asarray(samples); u=np.rint(pixels[:,0]).astype(int); v=np.rint(pixels[:,1]).astype(int)
    z=depth[v,u]
    if not np.isfinite(z).all() or np.any(z<=0): raise ValueError('selected seam has missing depth')
    if np.any(np.abs(np.diff(z))>max_depth_step_m): raise ValueError('selected seam crosses a depth discontinuity')
    k=checked_intrinsics(intrinsics)
    xyz=np.column_stack(((pixels[:,0]-k['cx'])*z/k['fx'],(pixels[:,1]-k['cy'])*z/k['fy'],z))
    return xyz,pixels


def measured_normal(depth,intrinsics,pixels):
    """Require a locally planar patch; never invent a joint-face orientation."""
    h,w=depth.shape
    positions=set()
    for u,v in pixels:
        u,v=int(round(u)),int(round(v))
        for y in range(max(0,v-2),min(h,v+3)):
            for x in range(max(0,u-2),min(w,u+3)): positions.add((x,y))
    k=checked_intrinsics(intrinsics); xyz=[]
    for u,v in positions:
        z=float(depth[v,u])
        if np.isfinite(z) and z>0: xyz.append(((u-k['cx'])*z/k['fx'],(v-k['cy'])*z/k['fy'],z))
    if len(xyz)<6: raise ValueError('surface normal unavailable; specify process reference_up')
    xyz=np.asarray(xyz); _,s,vh=np.linalg.svd(xyz-xyz.mean(axis=0),full_matrices=False)
    if s[1]<1e-6 or s[-1]/max(np.sqrt(len(xyz)),1)>.005:
        raise ValueError('adjacent surface is not planar; specify process reference_up')
    normal=vh[-1]
    facing=float(np.dot(normal,xyz.mean(axis=0)))
    if abs(facing)<1e-6: raise ValueError('ambiguous camera-facing surface; specify process reference_up')
    if facing>0: normal=-normal
    return normal


def project_seam(polyline_px,depth_m,intrinsics,frame_binding,registration,*,target=None,steady_now=None,reference_up=None,selection_lease=None):
    if isinstance(frame_binding,dict): frame_binding=FrameBinding.from_dict(frame_binding)
    now=time.monotonic() if steady_now is None else steady_now
    if selection_lease is None: frame_binding.require_current(now,registration.revision)
    else: selection_lease.require(frame_binding,now)
    if target is not None:
        if target.provider!='external': raise ValueError('physical RGB-D requires external provider')
        if target.generation!=frame_binding.source_session: raise ValueError('source session changed')
        if target.stamp!=divmod(frame_binding.sim_stamp,1_000_000_000): raise ValueError('selected frame stamp changed')
        if target.frame_id!=registration.optical_frame or target.clock_domain!='ros': raise ValueError('selected camera frame mismatch')
        if np.shape(depth_m)!=(target.height,target.width): raise ValueError('selected image dimensions changed')
    points,pixels=pixel_points(polyline_px,depth_m,intrinsics)
    normal=measured_normal(np.asarray(depth_m),intrinsics,pixels) if reference_up is None else np.asarray(reference_up,dtype=float)
    if normal.shape!=(3,) or not np.isfinite(normal).all() or abs(np.linalg.norm(normal)-1)>.001:
        raise ValueError('normalized process reference_up required')
    normal=registration.rotation @ normal
    points=registration.transform_points(points)
    if np.any(np.linalg.norm(np.cross(np.diff(points,axis=0),normal),axis=1)<1e-8):
        raise ValueError('process reference_up parallel to seam')
    seam=Seam('user-seam-1',tuple(map(tuple,points)),(tuple(normal),))
    return SeamCatalog(target.request_id if target else str(frame_binding.source_frame_id),target.target_revision if target else str(frame_binding.source_frame_id),frame_binding.source_session,'external',registration.target_frame,(seam,))


class RGBDFrameStore:
    """Bounded cache; every component must have the same ROS stamp and identity."""
    def __init__(self,registration,limit=30):
        self.registration=registration; self.limit=limit
        self.depth=OrderedDict(); self.intrinsics=OrderedDict(); self.rgb=OrderedDict(); self.identities=OrderedDict()
        self.colors=OrderedDict(); self.pinned_rgb=None
        self.context=None; self.live_receipt=None
        self.lease=SelectionLease(registration.revision); self.pinned=None

    def _put(self,cache,stamp,value):
        cache[stamp]=value
        while len(cache)>self.limit: cache.popitem(last=False)

    def clear(self):
        self.pinned=None; self.pinned_rgb=None; self.lease.clear(); self.live_receipt=None
        for cache in (self.depth,self.intrinsics,self.rgb,self.identities,self.colors): cache.clear()

    def add_depth(self,stamp,depth): self._put(self.depth,stamp,np.asarray(depth).copy())
    def add_intrinsics(self,stamp,intrinsics): self._put(self.intrinsics,stamp,checked_intrinsics(intrinsics))
    def add_rgb(self,stamp,shape,rgb=None):
        self._put(self.rgb,stamp,tuple(shape))
        if rgb is not None:
            pixels=np.asarray(rgb)
            if pixels.shape!=(*shape,3): raise ValueError('RGB dimensions differ')
            self._put(self.colors,stamp,pixels.copy())

    def selected_rgb(self,identity,steady_now):
        self.selected(identity,steady_now)
        if self.pinned and self.pinned[0]==identity: pixels=self.pinned_rgb
        else: pixels=self.colors.get(identity['frame_binding']['sim_stamp'])
        if pixels is None: raise ValueError('matching RGB pixels unavailable')
        return pixels

    def selected_payload(self,payload,steady_now):
        binding=FrameBinding.from_dict(payload['frame_binding'])
        identity=self.pinned[0] if self.pinned and self.pinned[3]==binding else self.identities.get(binding.sim_stamp)
        if identity is None or identity.get('frame_binding')!=payload['frame_binding']: raise ValueError('selected physical frame expired')
        for key in ('generation','frame_id','stamp','width','height','clock_domain'):
            if payload.get(key)!=identity.get(key): raise ValueError('selected physical frame mismatch: '+key)
        return identity,self.selected(identity,steady_now)


    def add_identity(self,payload):
        binding=FrameBinding.from_dict(payload['frame_binding'])
        context=(binding.source_session,binding.clock_epoch,binding.registration_revision)
        if self.context is not None and self.context!=context: self.clear()
        self.context=context; self.live_receipt=binding.receipt_monotonic
        if binding.registration_revision!=self.registration.revision: raise ValueError('registration revision changed')
        self._put(self.identities,binding.sim_stamp,payload)

    def pin(self,identity,steady_now):
        depth,intrinsics,binding=self.selected(identity,steady_now)
        self.lease.pin(binding,steady_now)
        self.pinned=(identity,depth,intrinsics,binding)
        self.pinned_rgb=self.colors.get(binding.sim_stamp)

    def selected(self,identity,steady_now):
        binding=FrameBinding.from_dict(identity['frame_binding'])
        self.lease.require(binding,steady_now)
        if self.live_receipt is None or not 0<=steady_now-self.live_receipt<=1.: raise ValueError('live camera frame expired')
        if self.context and (binding.source_session,binding.clock_epoch,binding.registration_revision)!=self.context: raise ValueError('selected frame context changed')
        if self.pinned and self.pinned[0]==identity:
            return self.pinned[1:]
        stamp=binding.sim_stamp
        if self.identities.get(stamp)!=identity: raise ValueError('selected frame identity expired or changed')
        if stamp not in self.depth or stamp not in self.intrinsics or stamp not in self.rgb: raise ValueError('matching RGB-D and CameraInfo not yet received')
        depth=self.depth[stamp]
        if depth.shape!=self.rgb[stamp] or depth.shape!=(identity['height'],identity['width']): raise ValueError('RGB-D dimensions differ')
        return depth,self.intrinsics[stamp],binding


def project_surface_measurement(polygon,depth_m,intrinsics,frame_binding,surface_id):
    """Finite depth mesh clipped to the selected ROI, preserving holes and steps."""
    from dataclasses import asdict
    depth=np.asarray(depth_m,dtype=float); poly=np.asarray(polygon,dtype=float)
    if depth.ndim!=2 or poly.ndim!=2 or poly.shape[1:]!=(2,) or len(poly)<3 or not np.isfinite(poly).all(): raise ValueError('valid surface polygon required')
    h,w=depth.shape
    if np.any(poly<0) or np.any(poly[:,0]>w-1) or np.any(poly[:,1]>h-1): raise ValueError('surface polygon outside image')
    k=checked_intrinsics(intrinsics)
    low=np.floor(poly.min(axis=0)).astype(int); high=np.ceil(poly.max(axis=0)).astype(int)
    step=max(1,int(np.ceil(max(high-low)/64)))
    us=sorted(set(range(low[0],high[0]+1,step))|{int(high[0])})
    vs=sorted(set(range(low[1],high[1]+1,step))|{int(high[1])})
    def inside(x,y):
        result=False
        for a,b in zip(poly,np.roll(poly,-1,axis=0)):
            edge=b-a; rel=np.array([x,y])-a
            if abs(edge[0]*rel[1]-edge[1]*rel[0])<1e-6 and np.dot(rel,edge)>=0 and np.dot(rel,edge)<=np.dot(edge,edge): return True
            if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]: result=not result
        return result
    vertices=[]; indices={}
    for j,v in enumerate(vs):
        for i,u in enumerate(us):
            z=depth[v,u]
            if inside(u,v) and np.isfinite(z) and z>0:
                indices[i,j]=len(vertices); vertices.append([(u-k['cx'])*z/k['fx'],(v-k['cy'])*z/k['fy'],float(z)])
    faces=[]
    for j in range(len(vs)-1):
        for i in range(len(us)-1):
            keys=((i,j),(i+1,j),(i+1,j+1),(i,j+1))
            if not all(key in indices for key in keys): continue
            # Inspect every depth pixel inside a coarse cell before meshing it.
            block=depth[vs[j]:vs[j+1]+1,us[i]:us[i+1]+1]
            if not np.isfinite(block).all() or block.min()<=0 or np.ptp(block)>.05: continue
            if not all(inside(x,y) for x,y in ((us[i]+(us[i+1]-us[i])*.5,vs[j]+(vs[j+1]-vs[j])*.5),)): continue
            a,b,c,d=[indices[key] for key in keys]; faces.extend(((a,b,c),(a,c,d)))
    if not faces: raise ValueError('no continuous measured surface in selected region')
    points=np.asarray(vertices); triangles=points[np.asarray(faces)]
    normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]); normal=normals.sum(axis=0)
    if np.linalg.norm(normal)<1e-10: raise ValueError('degenerate measured surface')
    normal/=np.linalg.norm(normal)
    if np.dot(normal,triangles.mean(axis=(0,1)))>0:
        normal=-normal; faces=[(a,c,b) for a,b,c in faces]
    return dict(surface_id=surface_id,vertices_m=vertices,faces=faces,normal=normal.tolist(),frame_binding=asdict(frame_binding))
