import asyncio
import json
import hashlib
import threading
import time
from pathlib import Path
import pytest
from sketch_control.runtime_apply import RuntimeApplyManager, RuntimeApplyError


def asset_bundle(selection,directory):
    directory.mkdir(parents=True,exist_ok=True)
    (directory/'robot.usd').write_text('#usda 1.0\ndef Xform "Robot" {}')
    (directory/'robot.urdf').write_text(f'<robot name="test"><ros2_control><hardware><param name="readiness_file">{directory}/simulation-ready.json</param></hardware></ros2_control></robot>')
    (directory/'robot_physics.urdf').write_text('<robot name="test"/>')
    (directory/'robot.srdf').write_text('<robot name="test"/>')
    (directory/'controllers.yaml').write_text('controller_manager: {}')
    (directory/'painting.yaml').write_text(f'generated_description_file: {directory}/robot.urdf\nobjects_config_file: {directory}/objects.yaml\n')
    (directory/'objects.yaml').write_text('objects: []')
    (directory/'scene.json').write_text('{}')
    (directory/'calibration').mkdir()
    (directory/'calibration/zed.json').write_text('{}')
    (directory/'calibration/d405.json').write_text('{}')
    (directory/'h_steel.usda').write_text('#usda 1.0')
    (directory/'h_steel_collision.json').write_text('{}')
    (directory/'h_steel_manifest.json').write_text('{}')
    (directory/'weld_fixture.json').write_text('{}')
    for side in selection.tools:
        if selection.tools[side]=='none':continue
        (directory/(side+'_eoat.json')).write_text(json.dumps(dict(mesh_file=side+'_eoat.stl')))
        (directory/(side+'_eoat.stl')).write_text('solid tool\nendsolid tool')
    (directory/'weld_tool.json').write_text(json.dumps(dict(collision_mode='native_urdf',robot_description_file=str(directory/'robot.urdf'),robot_description_sha256=hashlib.sha256((directory/'robot.urdf').read_bytes()).hexdigest())))
    return dict(directory=str(directory),usd=str(directory/'robot.usd'),urdf=str(directory/'robot.urdf'),physics_urdf=str(directory/'robot_physics.urdf'),srdf=str(directory/'robot.srdf'),controllers=str(directory/'controllers.yaml'),arms={side:dict(tool_id=tool) for side,tool in selection.tools.items()},revision='asset1',generation_stage='usd_complete')


def manager(tmp_path,build=None,start=None,stop=None,busy=None):
    async def noop(*args): return None
    async def default_start(bundle): return {'asset_revision':bundle['revision'],'ready':True}
    return RuntimeApplyManager(tmp_path,build or asset_bundle,stop or noop,start or default_start,busy=busy or (lambda:False))


def configure(m,**selection):
    return m.configure({'expected_revision':m.state()['revision'],'selection':selection})


def test_stale_client_and_state_survive_reload(tmp_path):
    m=manager(tmp_path)
    configure(m,process='weld',active_arm='right')
    with pytest.raises(RuntimeApplyError) as e:m.configure({'expected_revision':0,'selection':{'process':'paint'}})
    assert e.value.status==409
    asyncio.run(m.apply(1))
    restored=manager(tmp_path).state()
    assert restored['desired']['process']=='weld'
    assert restored['applied']['tools']['right']=='hongik_v4'
    assert restored['phase']=='idle' # A persisted ready flag cannot prove live processes.


def test_build_failure_keeps_previous_applied(tmp_path):
    m=manager(tmp_path); configure(m)
    asyncio.run(m.apply(1)); previous=m.state()['applied']
    configure(m,process='paint')
    def fail(*args):raise RuntimeError('conversion failed')
    m.build=fail
    with pytest.raises(RuntimeApplyError): asyncio.run(m.apply(2))
    assert m.state()['applied']==previous and m.state()['phase']=='failed'
    assert list((tmp_path/'assets').rglob('robot.usd'))


def test_failed_start_cleans_children_and_never_marks_applied(tmp_path):
    stopped=[]
    async def stop():stopped.append(True)
    async def start(bundle):raise RuntimeError('controller failed')
    m=manager(tmp_path,start=start,stop=stop); configure(m)
    with pytest.raises(RuntimeApplyError):asyncio.run(m.apply(1))
    assert len(stopped)==2 and m.state()['applied'] is None


def test_revision_ack_required(tmp_path):
    async def wrong(bundle):return {'asset_revision':'old','ready':True}
    m=manager(tmp_path,start=wrong);configure(m)
    with pytest.raises(RuntimeApplyError):asyncio.run(m.apply(1))
    assert m.state()['phase']=='failed'


def test_busy_execution_rejects_apply_and_edit(tmp_path):
    m=manager(tmp_path,busy=lambda:True)
    with pytest.raises(RuntimeApplyError):configure(m,process='paint')
    with pytest.raises(RuntimeApplyError):asyncio.run(m.apply(0))


def test_concurrent_configuration_during_build_rejected(tmp_path):
    async def scenario():
        entered=asyncio.Event(); release=asyncio.Event()
        async def build(selection,directory):
            entered.set(); await release.wait()
            return asset_bundle(selection,directory)
        m=manager(tmp_path,build=build);configure(m)
        task=asyncio.create_task(m.apply(1)); await entered.wait()
        with pytest.raises(RuntimeApplyError):configure(m,process='paint')
        with pytest.raises(RuntimeApplyError):await m.apply(1)
        release.set();await task
    asyncio.run(scenario())


@pytest.mark.parametrize('connections',[{'command':'sh'},{'robot_ip':'$(id)'},{'outpost_http':'file:///tmp/x'}, {'outpost_zed_serial':None}])
def test_connection_fields_are_validated(tmp_path,connections):
    with pytest.raises((ValueError,RuntimeApplyError)):
        manager(tmp_path).configure({'expected_revision':0,'selection':{},'connections':connections})


def test_relocation_updates_urdf_yaml_and_native_weld_hash(tmp_path):
    accepted=[]
    async def start(bundle):
        directory=Path(bundle['directory'])
        urdf=Path(bundle['urdf'])
        assert str(directory/'simulation-ready.json') in urdf.read_text()
        assert str(urdf) in (directory/'painting.yaml').read_text()
        assert '.building-' not in ''.join(p.read_text() for p in directory.iterdir() if p.suffix in ('.json','.yaml','.urdf'))
        tool=json.loads((directory/'weld_tool.json').read_text())
        assert tool['robot_description_file']==str(urdf)
        assert tool['robot_description_sha256']==hashlib.sha256(urdf.read_bytes()).hexdigest()
        accepted.append(bundle)
        return dict(ready=True,asset_revision=bundle['revision'])
    m=manager(tmp_path,start=start);configure(m,process='weld')
    asyncio.run(m.apply(1));assert accepted


@pytest.mark.parametrize('broken',['empty_usd','missing_profile','missing_mesh','missing_painting'])
def test_incomplete_bundle_never_reaches_start(tmp_path,broken):
    started=[]
    def build(selection,directory):
        bundle=asset_bundle(selection,directory)
        if broken=='empty_usd':Path(bundle['usd']).write_text('')
        else:(directory/{'missing_profile':'left_eoat.json','missing_mesh':'left_eoat.stl','missing_painting':'painting.yaml'}[broken]).unlink()
        return bundle
    async def start(bundle):started.append(True);return dict(ready=True,asset_revision=bundle['revision'])
    m=manager(tmp_path,build=build,start=start)
    with pytest.raises(RuntimeApplyError):asyncio.run(m.apply(0))
    assert not started and m.state()['phase']=='failed'


def test_real_rb10_reports_unintegrated_revision_path_before_stopping_or_generating(tmp_path):
    touched=[]
    async def stop():touched.append('stop')
    def build(*args):touched.append('build');raise AssertionError('Physical mode reached simulator')
    m=manager(tmp_path,build=build,stop=stop);configure(m,execution_mode='real',platform='rb10',active_arm='single',process='paint')
    assert m.state()['capabilities'][0]['supported'] is False
    with pytest.raises(RuntimeApplyError) as error:asyncio.run(m.apply(1))
    assert error.value.status==400 and touched==[]


def test_cancelled_thread_build_is_joined_before_cleanup_and_unlock(tmp_path):
    async def scenario():
        entered=threading.Event();release=threading.Event()
        def build(selection,directory):
            entered.set();assert release.wait(3);return asset_bundle(selection,directory)
        m=manager(tmp_path,build=build)
        task=asyncio.create_task(m.apply(0))
        while not entered.is_set():await asyncio.sleep(.001)
        task.cancel();await asyncio.sleep(.02)
        assert m.state()['phase']=='building'
        with pytest.raises(RuntimeApplyError):configure(m,process='paint')
        release.set()
        with pytest.raises(asyncio.CancelledError):await task
        assert m.state()['phase']=='failed'
        assert not list((tmp_path/'assets').glob('.building-*'))
        configure(m,process='paint')
    asyncio.run(scenario())


@pytest.mark.parametrize('missing',['h_steel.usda','h_steel_collision.json','weld_fixture.json'])
def test_simulation_scene_assets_must_be_complete_before_start(tmp_path,missing):
    def build(selection,directory):
        bundle=asset_bundle(selection,directory);(directory/missing).unlink();return bundle
    m=manager(tmp_path,build=build);configure(m,process='weld')
    with pytest.raises(RuntimeApplyError):asyncio.run(m.apply(1))


def test_binary_usd_relative_layers_survive_transaction_directory_rename(tmp_path):
    pytest.importorskip('pxr.Usd')
    from pxr import Usd,Sdf,UsdGeom,UsdUtils
    staged=[];started=[]
    def build(selection,directory):
        bundle=asset_bundle(selection,directory);staged.append(directory)
        # Match the importer layout: binary wrapper -> text layer -> binary geometry.
        payload=directory/'robot_generated/robot_physics/payloads';payload.mkdir(parents=True)
        geometry=Usd.Stage.CreateNew(str(payload/'geometry.usdc'))
        mesh=UsdGeom.Mesh.Define(geometry,'/Body');geometry.SetDefaultPrim(mesh.GetPrim())
        mesh.CreatePointsAttr([(0,0,0),(1,0,0),(0,1,0)])
        mesh.CreateFaceVertexCountsAttr([3]);mesh.CreateFaceVertexIndicesAttr([0,1,2]);geometry.GetRootLayer().Save()
        component=Usd.Stage.CreateNew(str(payload.parent/'robot_physics.usda'))
        body=component.DefinePrim('/Robot','Xform');body.GetReferences().AddReference('payloads/geometry.usdc')
        component.SetDefaultPrim(body);component.GetRootLayer().Save()
        root=Path(bundle['usd']);root.unlink()
        wrapper=Usd.Stage.CreateNew(str(root));prim=wrapper.DefinePrim('/Robot','Xform')
        prim.GetReferences().AddReference('robot_generated/robot_physics/robot_physics.usda')
        wrapper.SetDefaultPrim(prim);wrapper.GetRootLayer().Save()
        assert root.read_bytes().startswith(b'PXR-USDC')
        return bundle
    async def start(bundle):
        assert not staged[0].exists()
        layers,assets,unresolved=UsdUtils.ComputeAllDependencies(Sdf.AssetPath(bundle['usd']))
        assert len(layers)==3 and not unresolved
        assert all(Path(layer.realPath).is_relative_to(bundle['directory']) for layer in layers)
        assert all(not Path(ref).is_absolute() for layer in layers for ref in layer.GetExternalReferences())
        stage=Usd.Stage.Open(bundle['usd'])
        assert UsdGeom.Mesh(stage.GetPrimAtPath('/Robot')).GetPointsAttr().Get()==[(0,0,0),(1,0,0),(0,1,0)]
        for layer in layers:
            relative=str(Path(layer.realPath).relative_to(bundle['directory']))
            assert bundle['file_sha256'][relative]==hashlib.sha256(Path(layer.realPath).read_bytes()).hexdigest()
        started.append(True)
        return dict(ready=True,asset_revision=bundle['revision'])
    m=manager(tmp_path,build=build,start=start);configure(m,process='paint')
    asyncio.run(m.apply(1))
    assert started and m.state()['phase']=='ready'


def test_saved_two_tool_desired_migrates_without_rewriting_applied_evidence(tmp_path):
    m=manager(tmp_path)
    state=m.state();state.pop('capabilities')
    state.update(revision=8,desired=dict(schema_version=2,execution_mode='simulation',
        platform='commercial',active_arm='right',process='paint',tools={'left':'roller','right':'roller'}))
    state['applied']=dict(state['desired'])
    (tmp_path/'configuration.json').write_text(json.dumps(state))
    restored=manager(tmp_path).state()
    assert restored['desired']['tools']=={'left':'none','right':'roller'}
    assert restored['applied']['tools']=={'left':'roller','right':'roller'}
    assert restored['revision']==9 and restored['phase']=='idle'
    assert json.loads((tmp_path/'configuration.json').read_text())['desired']['tools']['left']=='none'
    assert manager(tmp_path).state()['revision']==9
