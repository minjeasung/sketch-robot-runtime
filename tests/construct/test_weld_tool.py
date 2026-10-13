import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).parents[2]/'scripts'))
from construct_weld_scene import tool_geometry, PreviewState

def test_isaac_and_moveit_use_same_torch_transform():
    center,length,quat,radius=tool_geometry({'tip_translation_m':[0,0,.15],'radius_m':.006})
    np.testing.assert_allclose(center,[0,0,.075]); assert length==.15 and radius==.006
    np.testing.assert_allclose(quat,[0,0,0,1])

def test_preview_reset_removes_old_geometry():
    state=PreviewState(); state.accept({'plan_id':'p','segments':[{'seam_id':'S1'}]})
    assert state.payload['plan_id']=='p'
    state.accept({'invalidated':True}); assert state.payload is None

def test_torch_mount_uses_the_model_fixed_tcp_transform():
    from construct_weld_scene import mounted_tool_geometry
    import numpy as np
    from scipy.spatial.transform import Rotation
    root=Path(__file__).resolve().parents[2]
    profile={'tip_translation_m':[0,0,.15],'radius_m':.006}
    # Exercise the original torch fixture; generated left-arm assets now carry a spray gun.
    parent,xyz,q,length,radius=mounted_tool_geometry(root/'src/construct_description/urdf_0528/construct_robot_0528.urdf','left_manipulator_ee_point',profile)
    assert parent=='left_manipulator_link_ee'
    expected=np.array([-.14531,0,-.2122])+Rotation.from_euler('xyz',[0,3.1416,0]).apply([0,0,.075])
    np.testing.assert_allclose(xyz,expected)
    np.testing.assert_allclose(Rotation.from_quat(q).apply([0,0,1]),Rotation.from_euler('xyz',[0,3.1416,0]).apply([0,0,1]))
