"""Calibrated camera views and finite-plane support, independent of ROS.

All geometry passed to a function must be expressed in the same frame. Camera
+Z is the optical viewing axis; the roller's drawing orientation is unrelated.
"""
import numpy as np
from sketch_control.rotation_utils import quat_to_matrix, quat_from_matrix


def unit(value):
    value = np.asarray(value, float)
    if value.shape != (3,) or not np.isfinite(value).all() or np.linalg.norm(value) < 1e-9:
        raise ValueError('invalid direction')
    return value / np.linalg.norm(value)


def plane_basis(normal):
    n = unit(normal)
    seed = np.eye(3)[np.argmin(np.abs(n))]
    u = unit(np.cross(seed, n))
    return u, np.cross(n, u)


def support_coordinates(polygon, normal):
    polygon = np.asarray(polygon, float)
    if polygon.ndim != 2 or polygon.shape[1] != 3 or len(polygon) < 3 or not np.isfinite(polygon).all():
        raise ValueError('invalid support polygon')
    center = polygon.mean(axis=0)
    n = unit(normal)
    if np.max(np.abs((polygon-center) @ n)) > .005:
        raise ValueError('non-planar support polygon')
    u, v = plane_basis(n)
    basis = np.column_stack((u, v))
    uv = (polygon-center) @ basis
    area = np.sum(uv[:,0]*np.roll(uv[:,1],-1)-uv[:,1]*np.roll(uv[:,0],-1)) / 2
    if abs(area) < 1e-6:
        raise ValueError('empty support polygon')
    if area < 0:
        uv = uv[::-1]
    edges = np.roll(uv,-1,axis=0)-uv
    lengths = np.linalg.norm(edges,axis=1)
    if np.min(lengths) < 1e-8:
        raise ValueError('duplicate support vertices')
    inward = np.column_stack((-edges[:,1], edges[:,0])) / lengths[:,None]
    distances = np.einsum('ijk,ik->ij', uv[None,:,:]-uv[:,None,:], inward)
    if distances.min() < -1e-6:
        raise ValueError('support must be convex and ordered')
    return center, basis, uv, inward


def support_mask(points, polygon, normal, margin=0.):
    center,basis,uv,inward = support_coordinates(polygon,normal)
    points = np.asarray(points,float)
    projected = (points-center) @ basis
    mask = np.ones(len(projected),dtype=bool)
    for origin,axis in zip(uv,inward):
        mask &= (projected-origin) @ axis >= float(margin)-1e-8
    return mask


def inset_polygon(uv, inward, margin):
    result = np.array(uv,copy=True)
    for origin,axis in zip(uv,inward):
        if not len(result):
            break
        clipped = []
        for a,b in zip(result,np.roll(result,-1,axis=0)):
            da,db = (a-origin)@axis-margin,(b-origin)@axis-margin
            if da >= -1e-9:
                clipped.append(a)
            if (da >= 0) != (db >= 0):
                clipped.append(a+(b-a)*(da/(da-db)))
        result = np.asarray(clipped,float).reshape(-1,2)
    return result


def measurement_samples(camera, polygon, normal, margin=.08, max_samples=3):
    """Nearest point in an inset convex support, then interior alternatives."""
    camera = np.asarray(camera,float)
    if camera.shape != (3,) or not np.isfinite(camera).all():
        raise ValueError("invalid current camera position")
    center,basis,uv,inward = support_coordinates(polygon,normal)
    bounded = inset_polygon(uv,inward,margin)
    if len(bounded) < 3:
        raise ValueError('plane support too small for measurement margin')
    projected = (np.asarray(camera,float)-center) @ basis
    if np.all(np.einsum('ij,ij->i',projected-uv,inward) >= margin-1e-8):
        nearest = projected
    else:
        edges = np.roll(bounded,-1,axis=0)-bounded
        scale = np.clip(np.einsum('ij,ij->i',projected-bounded,edges) / np.maximum(np.sum(edges*edges,axis=1),1e-15),0,1)
        candidates = bounded+scale[:,None]*edges
        nearest = candidates[np.argmin(np.linalg.norm(candidates-projected,axis=1))]
    middle = bounded.mean(axis=0)
    result = []
    for p in (nearest,(nearest+middle)/2,middle):
        xyz = center+basis@p
        if not any(np.linalg.norm(xyz-old) < .025 for old in result):
            result.append(xyz)
    return result[:max_samples]


def camera_view(sample, normal, tcp_pose, mount, standoff=.32, flipped=False):
    """Return TCP position/quaternion which aims calibrated optical +Z at sample."""
    translation, mount_q = mount
    sample,translation = np.asarray(sample,float),np.asarray(translation,float)
    mount_q,current_q = np.asarray(mount_q,float),np.asarray(tcp_pose[1],float)
    if sample.shape != (3,) or translation.shape != (3,) or not np.isfinite(np.r_[sample,translation]).all():
        raise ValueError("invalid camera view position")
    if any(q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9 for q in (mount_q,current_q)):
        raise ValueError("invalid camera view rotation")
    if not np.isfinite(standoff) or standoff <= 0:
        raise ValueError("invalid camera standoff")
    mount_q,current_q = mount_q/np.linalg.norm(mount_q),current_q/np.linalg.norm(current_q)
    mount_r = quat_to_matrix(np.asarray(mount_q,float))
    current_r = quat_to_matrix(current_q) @ mount_r
    z = -unit(normal)
    x_ref = current_r[:,0]
    x = x_ref-z*(x_ref@z)
    if np.linalg.norm(x) < 1e-6:
        x = plane_basis(z)[0]
    x = unit(x)
    if flipped:
        x = -x
    camera_r = np.column_stack((x,np.cross(z,x),z))
    tcp_r = camera_r @ mount_r.T
    camera = np.asarray(sample,float)+unit(normal)*standoff
    return camera-tcp_r@np.asarray(translation,float), quat_from_matrix(tcp_r)
