"""Pure acceptance calculations over recorded measured RB20 simulation data."""
import math
import statistics

def finite(value):
    return isinstance(value,(int,float)) and not isinstance(value,bool) and math.isfinite(value)

def assess_plane(point, normal, surface_x):
    """Compare a measured base-frame plane with the fixture's front wall."""
    if len(point)!=3 or len(normal)!=3 or not all(finite(v) for v in [*point,*normal,surface_x]):
        return {'passed':False,'reason':'nonfinite or incomplete plane'}
    norm=math.sqrt(sum(v*v for v in normal))
    if norm<=1e-9:return {'passed':False,'reason':'zero plane normal'}
    distance=abs(point[0]-surface_x)
    angle=math.degrees(math.acos(max(-1.,min(1.,-normal[0]/norm))))
    return {'passed':distance<=.005 and angle<=2.,'distance_error_m':distance,
            'normal_error_deg':angle,'point_link0':list(point),'normal_link0':list(normal)}

def assess_painting(events, samples, *, clock_domain='simulation'):
    if clock_domain not in ('simulation','steady'):
        raise ValueError('painting report clock domain must be simulation or steady')
    errors=[]
    if clock_domain=='simulation':
        if any(not finite(e.get('timestamp_ns')) or e['timestamp_ns']<0 for e in events):
            return dict(passed=False,errors=['simulation event timestamp missing or invalid'],strokes=[],clock_domain=clock_domain)
        if any(not finite(s.get('sim_time')) or s['sim_time']<=0 for s in samples):
            return dict(passed=False,errors=['simulation sample timestamp missing or invalid'],strokes=[],clock_domain=clock_domain)
    event_time=lambda event: event['timestamp_ns']*1e-9 if clock_domain=='simulation' else event['time']
    sample_time=lambda sample: sample['sim_time'] if clock_domain=='simulation' else sample['time']
    states=[e.get('state') for e in events]
    if not states or states[-1]!='COMPLETE':errors.append('final COMPLETE missing')
    if 'PRECONTACT_TARE_COMPLETE' not in states:errors.append('runtime tare completion missing')
    if any(s in {'ABORT','FAILED','NOT_READY'} for s in states):errors.append('execution fault present')
    paints=[i for i,s in enumerate(states) if s=='PAINT' and (i==0 or states[i-1]!='PAINT')]
    if len(paints)<2:errors.append('two independent PAINT strokes required')
    strokes=[];previous=-1
    for index in paints:
        prefix=events[previous+1:index]
        required=['APPROACH_PRECONTACT','PRECONTACT_TARE_COMPLETE','CONTACT_SEARCH','RAMP_UP']
        identity={key:events[index].get(key) for key in ('plan_hash','work_area_id','plane_generation_id')}
        if not all(identity.values()):errors.append('stroke execution identity missing')
        cursor=0
        for event in prefix:
            if cursor<len(required) and event.get('state')==required[cursor]:
                if any(event.get(key)!=value for key,value in identity.items()):
                    errors.append('stroke phase execution identity mismatch')
                cursor+=1
        if cursor!=len(required):errors.append('stroke approach/tare/search/ramp sequence incomplete')
        end=next((j for j in range(index+1,len(states)) if states[j]!='PAINT'),len(states))
        if end==len(states) or states[end]!='RAMP_DOWN':errors.append('stroke RAMP_DOWN missing')
        tail=states[end:next((j for j in paints if j>index),len(states))]
        if not any(s in tail for s in ('RETRACT','FINAL_RETRACT')):errors.append('stroke retract missing')
        start=float(event_time(events[index]));stop=float(event_time(events[end])) if end<len(events) else start
        feedback=[s for s in samples if start+1<=sample_time(s)<stop]
        invalid=any(not s.get('valid') or not finite(s.get('measured_contact_force_n')) or
                    not 0<=s.get('state_age_s',math.inf)<=.20 or not 0<=s.get('force_age_s',math.inf)<=.20 for s in feedback)
        identities={s.get('session') for s in samples}
        if len(identities)!=1 or not next(iter(identities),None):invalid=True
        if any(b['step']<=a['step'] or sample_time(b)<=sample_time(a) for a,b in zip(feedback,feedback[1:])):invalid=True
        if any(not 0<b['time']-a['time']<=.20 for a,b in zip(feedback,feedback[1:])):invalid=True
        duration=sample_time(feedback[-1])-sample_time(feedback[0]) if feedback else 0.
        wall_duration=feedback[-1]['time']-feedback[0]['time'] if feedback else 0.
        simulation_duration=(feedback[-1]['sim_time']-feedback[0]['sim_time'] if feedback else 0.) if clock_domain=='simulation' else None
        forces=[s['measured_contact_force_n'] for s in feedback if finite(s.get('measured_contact_force_n'))]
        fraction=sum(18<=f<=32 for f in forces)/len(forces) if forces else 0.
        passed=bool(not invalid and duration>=5 and fraction>=.95)
        if not passed:errors.append('stroke lacks 5 seconds of fresh in-band measured force')
        sorted_forces=sorted(forces)
        strokes.append({'passed':passed,'duration_s':duration,'simulation_duration_s':simulation_duration,'wall_duration_s':wall_duration,'samples':len(feedback),'fraction_18_32_n':fraction,
            'mean_n':statistics.mean(forces) if forces else None,'p95_n':sorted_forces[int(.95*(len(forces)-1))] if forces else None,
            'max_n':max(forces) if forces else None})
        previous=index
    if 'FINAL_RETRACT' not in states:errors.append('FINAL_RETRACT missing')
    return {'passed':not errors,'errors':list(dict.fromkeys(errors)),'strokes':strokes,'clock_domain':clock_domain}


def assess_stop(last_input, detected, disabled, cancel_ack, stopped):
    values=(last_input,detected,disabled,cancel_ack,stopped)
    finite=all(v is not None and math.isfinite(v) for v in values)
    passed=bool(finite and last_input<=detected<=disabled and detected<=cancel_ack<=stopped and
                detected-last_input<=.22 and disabled-detected<=.10 and stopped-detected<=.10)
    return {'passed':passed,'detection_s':detected-last_input if finite else None,
            'disable_s':disabled-detected if finite else None,'stop_s':stopped-detected if finite else None,
            'cancel_acknowledged':cancel_ack is not None}
