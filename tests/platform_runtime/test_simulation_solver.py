"""The authored TGS scene must apply gravity on each internal solver step."""
import sys
from pathlib import Path

import pytest

pytest.importorskip('pxr', reason='USD authoring tests require the Isaac/USD Python environment')

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
import platform_simulation_app as app


def test_tgs_gravity_uses_each_solver_iteration_without_changing_timing(tmp_path):
    from pxr import Usd, UsdPhysics, Sdf
    stage = Usd.Stage.CreateInMemory()
    scene = UsdPhysics.Scene.Define(stage, '/physicsScene').GetPrim()
    scene.CreateAttribute('physxScene:solverType', Sdf.ValueTypeNames.Token).Set('TGS')
    scene.CreateAttribute('physxScene:timeStepsPerSecond', Sdf.ValueTypeNames.UInt).Set(500)
    other = UsdPhysics.Scene.Define(stage, '/otherScene').GetPrim()
    other.CreateAttribute('physxScene:enableExternalForcesEveryIteration', Sdf.ValueTypeNames.Bool).Set(False)

    contract = app.configure_simulator_solver(stage, '/physicsScene')
    # Re-open the authored asset: this is the value the physics engine consumes.
    path = tmp_path / 'scene.usda'
    stage.GetRootLayer().Export(str(path))
    loaded = Usd.Stage.Open(str(path))
    authored = loaded.GetPrimAtPath('/physicsScene')
    assert authored.GetAttribute('physxScene:enableExternalForcesEveryIteration').Get() is True
    assert authored.GetAttribute('physxScene:solverType').Get() == 'TGS'
    assert authored.GetAttribute('physxScene:timeStepsPerSecond').Get() == 500
    assert contract == dict(scene_path='/physicsScene', solver_type='TGS',
                            enable_external_forces_every_iteration=True)
    assert loaded.GetPrimAtPath('/otherScene').GetAttribute('physxScene:enableExternalForcesEveryIteration').Get() is False


def test_solver_configuration_requires_the_existing_physics_scene():
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, '/Robot')
    for path in ('/missing', '/Robot'):
        with pytest.raises(ValueError, match='physics scene'):
            app.configure_simulator_solver(stage, path)


@pytest.mark.parametrize('platform', ['rb10', 'commercial'])
@pytest.mark.parametrize('process', ['paint', 'spray', 'weld'])
@pytest.mark.parametrize('mode', ['simulation', 'mixed'])
def test_articulation_solver_override_is_only_rb10_paint(tmp_path, platform, process, mode):
    from pxr import Usd, UsdGeom, UsdPhysics, Sdf
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, '/World/Robot/base').GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root)
    other = UsdGeom.Xform.Define(stage, '/other').GetPrim()
    other.CreateAttribute('physxArticulation:solverVelocityIterationCount', Sdf.ValueTypeNames.Int).Set(8)
    root.CreateAttribute('physxArticulation:sleepThreshold', Sdf.ValueTypeNames.Float).Set(0.)
    selection = dict(platform=platform, process=process, execution_mode=mode)

    contract = app.configure_simulator_articulation_solver(root, selection)
    path = tmp_path / 'articulation.usda'
    stage.GetRootLayer().Export(str(path))
    loaded = Usd.Stage.Open(str(path))
    authored = loaded.GetPrimAtPath('/World/Robot/base')
    target = platform == 'rb10' and process == 'paint'
    assert authored.GetAttribute('physxArticulation:solverPositionIterationCount').Get() == 64
    assert authored.GetAttribute('physxArticulation:solverVelocityIterationCount').Get() == (1 if target else 8)
    assert authored.GetAttribute('physxArticulation:sleepThreshold').Get() == 0.
    assert loaded.GetPrimAtPath('/other').GetAttribute('physxArticulation:solverVelocityIterationCount').Get() == 8
    assert contract['articulation_path'] == '/World/Robot/base'
    assert contract['solver_position_iteration_count'] == 64
    assert contract['solver_velocity_iteration_count'] == (1 if target else 8)
    assert contract['selection'] == selection
    assert contract['rb10_paint_tgs_iteration_override'] is target
    assert '110.3.2' in contract['provenance'] if target else 'Existing' in contract['provenance']


def test_articulation_solver_rejects_real_mode_and_non_articulation():
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, '/Robot').GetPrim()
    selection = dict(platform='rb10', process='paint', execution_mode='simulation')
    with pytest.raises(ValueError, match='articulation'):
        app.configure_simulator_articulation_solver(root, selection)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    with pytest.raises(ValueError, match='simulation robot modes'):
        app.configure_simulator_articulation_solver(root, dict(selection, execution_mode='real'))
    assert not root.GetAttribute('physxArticulation:solverVelocityIterationCount')
