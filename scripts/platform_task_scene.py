"""Isaac adapter for /task_scene/apply. No guessed workpiece or back thickness."""
import json
import hashlib


def apply_task_surface(stage,payload,*,expected_frame,friction=None):
    """Caller must invoke on the simulation thread; returns an explicit ACK."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics
    ack=dict(engine='isaac',surface_id=payload.get('surface_id',''),revision=payload.get('revision',''),success=False)
    try:
        if payload['frame_id']!=expected_frame: raise ValueError('measured surface frame differs from simulation base')
        import numpy as np
        vertices=np.asarray(payload['vertices_m'],dtype=float); faces=np.asarray(payload['faces'])
        if vertices.ndim!=2 or vertices.shape[1:]!=(3,) or len(vertices)<3 or not np.isfinite(vertices).all(): raise ValueError('invalid measured vertices')
        if faces.ndim!=2 or faces.shape[1:]!=(3,) or len(faces)<1 or faces.dtype.kind not in 'iu' or np.any(faces<0) or np.any(faces>=len(vertices)): raise ValueError('invalid measured faces')
        path='/World/MeasuredTaskSurface'
        if stage.GetPrimAtPath(path): stage.RemovePrim(path)
        mesh=UsdGeom.Mesh.Define(stage,path)
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float,p)) for p in vertices])
        mesh.CreateFaceVertexCountsAttr([3]*len(faces))
        mesh.CreateFaceVertexIndicesAttr(faces.flatten().tolist())
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.35,.55,.65)])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
        mesh.GetPrim().CreateAttribute('taskScene:revision',Sdf.ValueTypeNames.String).Set(payload['revision'])
        if friction is not None:
            from rb20_painting_scene import configure_paint_wall_material
            ack['contact_material']=configure_paint_wall_material(
                stage,path,friction,material_path='/World/MeasuredPaintMaterial')
        ack['success']=True
    except Exception as exc: ack['error']=str(exc)
    return ack
