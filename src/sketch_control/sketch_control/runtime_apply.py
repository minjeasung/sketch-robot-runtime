"""Revision-checked configuration and transactional simulation asset activation."""
import asyncio
from copy import deepcopy
import inspect
import hashlib
import ipaddress
import json
from pathlib import Path
import shutil
import threading
from urllib.parse import urlsplit
from uuid import uuid4
from .runtime_selection import normalize_selection, capabilities


class RuntimeApplyError(ValueError):
    def __init__(self,message,status=409):
        super().__init__(message); self.status=status


CONNECTION_FIELDS={'robot_ip','outpost_http','outpost_zed_hw_id','outpost_zed_serial',
                   'outpost_d405_hw_id','outpost_d405_serial','zed_calibration_file','d405_calibration_file'}


def validate_connections(value):
    if not isinstance(value,dict) or set(value)-CONNECTION_FIELDS:
        raise RuntimeApplyError('Unknown connection fields',400)
    for key,text in value.items():
        if not isinstance(text,str) or len(text)>4096 or any(ord(c)<32 or 127<=ord(c)<=159 for c in text):
            raise RuntimeApplyError('Invalid connection value: '+key,400)
        if key=='robot_ip':
            try:ipaddress.ip_address(text)
            except ValueError:raise RuntimeApplyError('Invalid robot IP',400) from None
        if key=='outpost_http':
            url=urlsplit(text)
            if url.scheme not in ('http','https') or not url.hostname or url.username or url.password or url.query or url.fragment:
                raise RuntimeApplyError('Invalid Outpost HTTP address',400)
    return dict(value)


class RuntimeApplyManager:
    def __init__(self,root,build,stop,start,*,busy=lambda:False,initial=None):
        self.root=Path(root).resolve();self.build=build;self.stop=stop;self.start=start;self.busy=busy
        self._lock=threading.RLock()
        self._cancel_requested=False
        self._state=dict(revision=0,desired=(initial or normalize_selection({})).to_dict(),applied=None,
                         connections={},applied_connections={},phase='idle',asset_revision=None,error=None)
        file=self.root/'configuration.json'
        if file.is_file():
            saved=json.loads(file.read_text())
            desired=normalize_selection(saved['desired']).to_dict()
            validate_connections(saved.get('connections',{}))
            migrated=desired != saved['desired']
            if migrated:
                saved['desired']=desired
                saved['revision']+=1
            self._state.update(saved)
            self._state['phase']='idle'
            self._state['error']='Supervisor restarted; apply configuration to verify live readiness.'
            if migrated:self._save()

    def state(self):
        with self._lock:
            result=deepcopy(self._state)
            result['capabilities']=self._capabilities(normalize_selection(result['desired']))
            return result

    @staticmethod
    def _capabilities(selection):
        result=capabilities(selection)
        if selection.execution_mode=='real' and result[0]['supported']:
            result[0].update(supported=False,reason='실환경의 자동 EOAT 적용 경로는 연결되지 않았습니다. 고급 설정의 기존 work / spray_motion_test 구성을 사용하세요.')
        return result

    def check_process_stop(self):
        with self._lock:
            if self._state['phase'] in ('stopping','building','starting'):
                raise RuntimeApplyError('Use system shutdown to cancel the active runtime transaction')

    def check_legacy_start(self):
        with self._lock:
            if self._state['phase'] in ('stopping','building','starting'):
                raise RuntimeApplyError('Runtime apply owns process startup; wait or shut down')
            if (self._state['revision']>0 or self._state['applied'] is not None) and not self._state.get('legacy_configuration'):
                raise RuntimeApplyError('Use revisioned runtime apply for the selected configuration')

    def check_legacy_configuration(self,expected_revision=None):
        with self._lock:
            owned=self._state['revision']>0 or self._state['applied'] is not None
            self._check(expected_revision if owned or expected_revision is not None else self._state['revision'])

    def adopt_legacy(self,configuration,expected_revision=None):
        with self._lock:
            self.check_legacy_configuration(expected_revision)
            if self._state['revision']>0 or self._state['applied'] is not None or expected_revision is not None:
                self._state.update(revision=self._state['revision']+1,applied=None,applied_connections={},asset_revision=None,
                                   phase='idle',error=None,legacy_configuration=deepcopy(configuration))
                self._save()

    def request_stop(self):
        with self._lock:
            if self._state['phase'] in ('stopping','building','starting'):self._cancel_requested=True

    def mark_unready(self,error=None):
        with self._lock:
            if self._state['phase']=='ready':
                self._state.update(phase='failed' if error else 'idle',error=error)
                self._save()

    def _check_cancelled(self):
        if self._cancel_requested:raise RuntimeApplyError('Runtime apply cancelled by shutdown')

    def _save(self):
        self.root.mkdir(parents=True,exist_ok=True)
        temp=self.root/('configuration.'+uuid4().hex+'.tmp')
        temp.write_text(json.dumps(self._state,indent=2));temp.replace(self.root/'configuration.json')

    def _check(self,revision):
        if type(revision) is not int or revision!=self._state['revision']:
            raise RuntimeApplyError('Configuration revision changed; refresh before applying')
        if self._state['phase'] in ('stopping','building','starting') or self.busy():
            raise RuntimeApplyError('Configuration cannot change while applying or executing')

    def configure(self,request):
        with self._lock:
            if not isinstance(request,dict) or set(request)-{'selection','connections','expected_revision'}:
                raise RuntimeApplyError('Unknown configuration request fields',400)
            self._check(request.get('expected_revision'))
            previous=normalize_selection(self._state['desired'])
            selection=normalize_selection(request.get('selection',{}),previous)
            connections=validate_connections(request.get('connections',self._state['connections']))
            self._state.update(revision=self._state['revision']+1,desired=selection.to_dict(),connections=connections,
                               phase='idle',error=None,legacy_configuration=None)
            self._save();return self.state()

    async def _call(self,fn,*args):
        if inspect.iscoroutinefunction(fn):return await fn(*args)
        # A cancelled waiter must not leave a writer modifying the build directory
        # after cleanup or after a new transaction has acquired the same manager.
        worker=asyncio.create_task(asyncio.to_thread(fn,*args))
        try:value=await asyncio.shield(worker)
        except asyncio.CancelledError:
            try:await asyncio.shield(worker)
            except Exception:pass
            raise
        return await value if inspect.isawaitable(value) else value

    @staticmethod
    def _validate_bundle(bundle,directory,selection):
        if not isinstance(bundle,dict) or bundle.get('generation_stage')!='usd_complete':
            raise RuntimeError('Complete verified USD bundle is required')
        if Path(bundle.get('directory','')).resolve()!=directory.resolve():
            raise RuntimeError('Generated bundle directory does not match the transaction')
        required=[]
        for key in ('usd','urdf','physics_urdf','srdf','controllers'):
            path=Path(bundle.get(key,''))
            if not path.resolve().is_relative_to(directory.resolve()):raise RuntimeError('Generated file escapes bundle: '+key)
            required.append(path)
        required.extend(directory/name for name in ('scene.json','painting.yaml','objects.yaml'))
        for side in selection.tools:
            if selection.tools[side]!='none':
                required.extend((directory/(side+'_eoat.json'),directory/(side+'_eoat.stl')))
        if selection.process=='weld':required.append(directory/'weld_tool.json')
        if selection.execution_mode=='simulation':
            required.append(directory/'calibration/zed.json')
            if selection.process=='paint':required.append(directory/'calibration/d405.json')
            else:required.extend(directory/name for name in ('h_steel.usda','h_steel_collision.json','h_steel_manifest.json'))
            if selection.process=='weld':required.append(directory/'weld_fixture.json')
        for path in required:
            if not path.is_file() or path.stat().st_size==0:raise RuntimeError('Missing or empty generated asset: '+str(path))
        for side in selection.tools:
            if selection.tools[side]=='none':continue
            profile=json.loads((directory/(side+'_eoat.json')).read_text())
            mesh=Path(profile.get('mesh_file',''))
            if not mesh.is_absolute():mesh=directory/mesh
            if not mesh.is_file() or not mesh.resolve().is_relative_to(directory.resolve()):raise RuntimeError('Invalid EOAT mesh reference')

    def _phase(self,phase):
        with self._lock:self._state['phase']=phase;self._save()

    async def apply(self,expected_revision):
        with self._lock:
            self._check(expected_revision)
            selection=normalize_selection(self._state['desired'])
            cap=self._capabilities(selection)[0]
            if not cap['supported']:raise RuntimeApplyError(cap['reason'],400)
            self._cancel_requested=False
            self._phase('stopping')
            connections=deepcopy(self._state['connections'])
        temp=self.root/'assets'/('.building-'+uuid4().hex)
        try:
            await self._call(self.stop)
            self._check_cancelled()
            self._phase('building')
            bundle=await self._call(self.build,selection,temp)
            self._check_cancelled()
            self._validate_bundle(bundle,temp,selection)
            revision=bundle['revision']
            if not isinstance(revision,str) or not revision.isalnum():raise RuntimeError('Invalid asset revision')
            # Unique instance path prevents replacing assets still referenced by prior readers.
            destination=temp.parent/(revision+'-'+uuid4().hex)
            def relocate(value):
                if isinstance(value,str):return value.replace(str(temp),str(destination))
                if isinstance(value,dict):return {k:relocate(v) for k,v in value.items()}
                if isinstance(value,list):return [relocate(v) for v in value]
                return value
            bundle=relocate(bundle)
            for path in temp.rglob('*'):
                if path.is_file() and path.suffix in ('.json','.yaml','.yml','.urdf','.srdf','.xml','.usda'):
                    path.write_text(path.read_text().replace(str(temp),str(destination)))
            temp.rename(destination)
            # The native collision validator hashes the final URDF bytes, including
            # the relocated readiness marker path used by painting hardware.
            for path in destination.rglob('*.json'):
                profile=json.loads(path.read_text())
                if isinstance(profile,dict) and profile.get('collision_mode')=='native_urdf':
                    description=Path(profile['robot_description_file'])
                    if not description.is_file():raise RuntimeError('Native weld robot description is missing')
                    profile['robot_description_sha256']=hashlib.sha256(description.read_bytes()).hexdigest()
                    path.write_text(json.dumps(profile,indent=2))
            self._validate_bundle(bundle,destination,selection)
            bundle['file_sha256']={str(path.relative_to(destination)):hashlib.sha256(path.read_bytes()).hexdigest()
                                   for path in destination.rglob('*') if path.is_file() and path.name!='bundle.json'}
            bundle.update(runtime_revision=expected_revision,connections=connections,selection=selection.to_dict())
            (destination/'bundle.json').write_text(json.dumps(bundle,indent=2))
            self._check_cancelled()
            self._phase('starting')
            ack=await self._call(self.start,bundle)
            self._check_cancelled()
            if not isinstance(ack,dict) or ack.get('ready') is not True or ack.get('asset_revision')!=revision:
                raise RuntimeError('Processes did not acknowledge the selected asset revision')
            with self._lock:
                self._state.update(applied=selection.to_dict(),applied_connections=connections,phase='ready',
                                   asset_revision=revision,error=None,bundle_directory=str(destination),legacy_configuration=None)
                self._save()
            return self.state()
        except BaseException as exc:
            cleanup_error=None
            try:await asyncio.shield(self._call(self.stop))
            except BaseException as stop_error:cleanup_error=str(stop_error)
            if temp.exists():shutil.rmtree(temp)
            with self._lock:
                self._state.update(phase='failed',error=(str(exc) or 'Runtime apply cancelled')+(f'; cleanup: {cleanup_error}' if cleanup_error else ''))
                self._save()
            if isinstance(exc,asyncio.CancelledError):raise
            raise RuntimeApplyError(self._state['error'],503) from exc
