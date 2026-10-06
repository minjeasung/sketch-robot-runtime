"""Continuous external state interlock, with explicit rearming after any loss."""
class ExecutionGuard:
    def __init__(self):
        self.model = self.source = ''
        self.controllers = {}
        self.competitors = []
        self.updated_at = float('-inf')
        self.armed = None
        self.latched = False

    def update(self, *, model, source, controllers, competitors, now):
        self.model, self.source = model, source
        self.controllers, self.competitors = dict(controllers), list(competitors)
        self.updated_at = now
        if self.armed and self.blockers(now, model=self.armed[0], source=self.armed[1]):
            self.latched = True

    def blockers(self, now, *, model=None, source=None):
        reasons = []
        if not 0 <= now-self.updated_at <= 1.0:
            reasons.append('EXTERNAL_STATE_STALE')
        if not self.model:
            reasons.append('EXTERNAL_MODEL_MISSING')
        if not self.source:
            reasons.append('EXTERNAL_SOURCE_MISSING')
        if model is not None and model != self.model:
            reasons.append('EXTERNAL_MODEL_CHANGED')
        if source is not None and source != self.source:
            reasons.append('EXTERNAL_SOURCE_CHANGED')
        if self.controllers.get('joint_trajectory_controller') != 'active':
            reasons.append('POSITION_CONTROLLER_NOT_ACTIVE')
        if any(state == 'active' and name not in ('joint_trajectory_controller', 'joint_state_broadcaster') for name, state in self.controllers.items()):
            reasons.append('COMPETING_CONTROLLER')
        if self.competitors:
            reasons.append('EXTERNAL_MOTION_OWNER')
        if self.latched:
            reasons.append('REARM_REQUIRED')
        return tuple(reasons)

    def arm(self, model, source, *, now):
        reasons = self.blockers(now, model=model, source=source)
        if reasons:
            raise ValueError(','.join(reasons))
        self.armed = (model, source)

    def reset(self):
        self.armed = None
        self.latched = False
