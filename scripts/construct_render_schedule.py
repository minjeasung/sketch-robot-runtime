"""Keep physics steps independent of expensive camera/viewport renders."""
import math


class RenderSchedule:
    def __init__(self, physics_hz=100, render_hz=20):
        if not (math.isfinite(physics_hz) and math.isfinite(render_hz)
                and 0 < render_hz <= physics_hz):
            raise ValueError('render rate must be finite and between 0 and physics rate')
        self.physics_hz = physics_hz
        self.render_hz = render_hz
        self.phase = 0

    def advance(self, playing=True):
        if not playing:
            self.phase = 0
            return True
        self.phase += self.render_hz
        if self.phase >= self.physics_hz:
            self.phase -= self.physics_hz
            return True
        return False
