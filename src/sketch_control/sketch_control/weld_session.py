"""Revision-bound welding lifecycle. Planning responses cannot revive old work."""
from dataclasses import asdict
from uuid import uuid4
from .weld_contract import PlanIdentity, validate_settings

class WeldSession:
    def __init__(self,profile):
        self.profile=profile; self.server_session_id=uuid4().hex
        self.target=None; self.catalog=None; self.selection=(); self.settings=None
        self.selection_revision=0; self.settings_revision=0; self.scene_revision='initial'
        self.plan=None; self.state='EMPTY'; self.error=''; self.current_segment=0
        self.planning_attempt=''

    @property
    def busy(self): return self.state in ('RUNNING','STOPPING')

    def _editable(self):
        if self.busy: raise ValueError('execution or stop confirmation in progress')

    def identity(self):
        return PlanIdentity(self.server_session_id,self.target.target_revision if self.target else '',
            self.catalog.generation if self.catalog else '',self.selection_revision,self.settings_revision,
            self.profile.revision,self.scene_revision,self.profile.model_id,self.planning_attempt)

    def invalidate(self,reason):
        self.plan=None; self.error=reason
        if self.busy: self.state='STOPPING'
        else: self.state='SELECTED' if self.selection else ('SEAMS_READY' if self.catalog else ('TARGET_READY' if self.target else 'EMPTY'))

    def request_target(self,target):
        self._editable(); self.target=target; self.catalog=None; self.selection=(); self.selection_revision+=1
        self.invalidate('target changed')

    def accept_catalog(self,catalog):
        self._editable()
        if not self.target or any(getattr(catalog,k)!=getattr(self.target,k) for k in ('request_id','target_revision','generation')):
            raise ValueError('stale extraction result')
        self.catalog=catalog; self.selection=(); self.selection_revision+=1; self.invalidate('new seams')

    def select(self,items):
        self._editable()
        ids={s.seam_id for s in self.catalog.seams} if self.catalog else set()
        if not isinstance(items,list): raise ValueError('selection list required')
        result=[]
        for item in items:
            sid=item.get('seam_id'); reverse=item.get('reverse',False)
            if sid not in ids or sid in [v[0] for v in result] or type(reverse) is not bool: raise ValueError('invalid seam selection')
            result.append((sid,reverse))
        self.selection=tuple(result); self.selection_revision+=1; self.invalidate('selection changed')

    def apply_settings(self,payload):
        self._editable(); validated=validate_settings(payload,self.profile)
        self.settings=validated; self.settings_revision+=1; self.invalidate('settings changed')
        return validated

    def begin_plan(self):
        self._editable()
        if not self.selection or self.settings is None: raise ValueError('select seams and apply settings first')
        self.planning_attempt=uuid4().hex
        self.plan=None; self.current_segment=0; self.state='PLANNING'; self.error=''; return self.identity()

    def accept_plan(self,identity,plan):
        if self.state!='PLANNING' or identity!=self.identity() or plan.identity!=identity: return False
        self.plan=plan; self.state='READY'; return True

    def begin_execution(self,plan_id,identity):
        if self.state!='READY' or not self.plan or self.plan.plan_id!=plan_id or identity!=self.identity(): raise ValueError('current verified plan required')
        self.state='RUNNING'; self.current_segment=0; return self.plan

    def complete_segment(self):
        if self.state!='RUNNING': return False
        self.current_segment+=1
        if self.current_segment>=len(self.plan.segments): self.state='COMPLETED'; self.plan=None; return False
        return True

    def request_stop(self):
        self.state='STOPPING'; self.error='stop requested'

    def confirm_stop(self):
        self.plan=None; self.state='STOPPED'; self.error='stopped'

    def snapshot(self):
        segment=self.plan.segments[self.current_segment] if self.plan and self.current_segment<len(self.plan.segments) else None
        return dict(state=self.state,error=self.error,identity=asdict(self.identity()),server_session_id=self.server_session_id,
                    ready=self.state=='READY',running=self.busy,plan_id=self.plan.plan_id if self.plan else '',
                    selection=[dict(seam_id=s,reverse=r) for s,r in self.selection],segment_index=self.current_segment,
                    current_seam_id=segment.seam_id if segment else '',current_phase=segment.phase if segment else '',
                    settings=asdict(self.settings) if self.settings else None,settings_revision=self.settings_revision)
