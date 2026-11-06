"""Frame-bound rectified-image projection of validated 3D seam catalogs."""
import numpy as np


def project_catalog(catalog, frame, info, optical_from_catalog):
    if frame.get('valid') is not True or frame.get('generation') != catalog.generation:
        raise ValueError('Current camera generation required')
    if any(info.get(k) != frame.get(k) for k in ('stamp','frame_id','width','height')):
        raise ValueError('CameraInfo must match the displayed frame')
    projection=np.asarray(info['p'],dtype=float).reshape(3,4)
    transform=np.asarray(optical_from_catalog,dtype=float)
    if (not np.isfinite(projection).all() or projection[0,0]<=0 or projection[1,1]<=0
            or transform.shape!=(4,4) or not np.isfinite(transform).all()
            or not np.allclose(transform[3],[0,0,0,1])
            or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-6)):
        raise ValueError('Finite camera calibration and rigid transform required')
    seams=[]
    for seam in catalog.seams:
        xyz=np.asarray(seam.points_xyz,dtype=float)
        optical=np.c_[xyz,np.ones(len(xyz))]@transform.T
        image=optical@projection.T
        points=[]
        for point,pixel in zip(optical,image):
            if point[2]<=.01 or pixel[2]<=0 or not np.isfinite(pixel).all():points.append(None)
            else:points.append((pixel[:2]/pixel[2]).tolist())
        seams.append(dict(seam_id=seam.seam_id,points_px=points))
    return dict(header=dict(stamp=frame['stamp'],frame_id=frame['frame_id']),
        width=frame['width'],height=frame['height'],generation=catalog.generation,
        target_revision=catalog.target_revision,request_id=catalog.request_id,
        source=catalog.source,seams=seams)
