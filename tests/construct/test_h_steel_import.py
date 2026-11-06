"""USD workpiece retains the source geometry and exports matching collision triangles."""
import sys
import json
from pathlib import Path
import numpy as np
import pytest

pytest.importorskip('pxr')
from pxr import Usd, UsdGeom, UsdPhysics

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))
from prepare_h_steel import prepare


def test_workpiece_preserves_metric_size_and_uses_actual_mesh_collision(tmp_path):
    meta = prepare(ROOT, tmp_path, translation=(-.515, -1.7, 0.))
    assert meta['mesh_count'] == 23
    assert meta['triangle_count'] == 2352
    np.testing.assert_allclose(np.subtract(*meta['bounds_m'][::-1]), [1.03, .8, 1.516], atol=1e-5)
    scene = Usd.Stage.Open(str(tmp_path/'h_steel.usda'))
    meshes = [p for p in scene.Traverse() if p.IsA(UsdGeom.Mesh)]
    assert len(meshes) == 23
    assert all(p.HasAPI(UsdPhysics.CollisionAPI) for p in meshes)
    assert all(UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() == 'none' for p in meshes)
    assert not any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in scene.Traverse())
    data = json.loads((tmp_path/'h_steel_collision.json').read_text())
    points = np.concatenate([part['vertices'] for part in data['meshes']])
    np.testing.assert_allclose([points.min(axis=0), points.max(axis=0)], meta['bounds_m'])
    assert data['frame_id'] == 'World'
