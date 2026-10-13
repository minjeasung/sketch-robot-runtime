"""Simulation process output; never owns a physical welding power supply."""
import time
from dataclasses import asdict

class SimWeldOutput:
    def __init__(self,clock=time.monotonic):
        self.clock=clock; self.on=False; self.deadline=0.; self.settings={}; self.plan_id=''; self.seam_id=''; self.reason='OFF'; self.start_sim_time_s=None
    def configure(self,settings): self.settings=asdict(settings)
    def start_segment(self,plan_id,seam_id,start_sim_time_s=None):
        self.plan_id=plan_id; self.seam_id=seam_id; self.on=True; self.deadline=self.clock()+.5; self.reason='WELD'; self.start_sim_time_s=start_sim_time_s
    def renew(self):
        if self.on: self.deadline=self.clock()+.5
    def stop_segment(self,reason): self.on=False; self.reason=reason
    def status(self):
        if self.on and self.clock()>self.deadline: self.stop_segment('feedback timeout')
        return dict(simulation=True,on=self.on,reason=self.reason,settings=self.settings,plan_id=self.plan_id,seam_id=self.seam_id,start_sim_time_s=self.start_sim_time_s)
