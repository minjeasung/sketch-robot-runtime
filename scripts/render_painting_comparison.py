#!/usr/bin/env python3
"""Export synchronized measured joints and an observation-video path overlay."""
import argparse
import csv
import json
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
import cv2
import numpy as np
from painting_comparison import contact_class, motion_metrics, stroke_displacement


def lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def render(directory):
    directory=Path(directory)
    meta=json.loads((directory/'camera.json').read_text())
    data=lines(directory/'measured.jsonl');frames=lines(directory/'frames.jsonl')
    events=lines(directory/'events.jsonl')
    trace=json.loads((directory/'trace.json').read_text()) if (directory/'trace.json').exists() else dict(samples=[])
    report=json.loads((directory/'report.json').read_text()) if (directory/'report.json').exists() else dict(passed=False)
    if not data or not frames:raise ValueError('No measured frames or samples')
    if report.get('session_id') != meta['session'] or any(s.get('session') != meta['session'] for s in trace['samples']):
        raise ValueError('Video, workflow and force samples must share one simulation session')
    times=np.array([d['sim_time'] for d in data]);qs=np.array([d['positions'] for d in data])
    # The model's TCP is proximal to the roller. Project the collision cylinder
    # center using its fixed URDF offset, rather than drawing a wrist trail.
    from scipy.spatial.transform import Rotation
    bundle=json.loads((directory.parent/'assets/bundle.json').read_text())
    robot=ET.parse(bundle['urdf']).getroot();arm=bundle['arms']['right']
    parents={j.find('child').get('link'):j for j in robot.findall('joint')}
    def fixed_to_ft(link):
        result=np.eye(4)
        while link!=arm['wrench_frame']:
            joint=parents[link]
            if joint.get('type')!='fixed':raise ValueError('Roller/TCP offset must be rigid')
            origin=joint.find('origin');tf=np.eye(4)
            if origin is not None:
                tf[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
                tf[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
            result=tf@result;link=joint.find('parent').get('link')
        return result
    collision=robot.find(f"link[@name='{arm['contact_link']}']/collision")
    offset=np.eye(4);offset[:3,3]=np.fromstring(collision.find('origin').get('xyz'),sep=' ')
    tcp_from_roller=np.linalg.inv(fixed_to_ft(arm['tcp_link']))@fixed_to_ft(arm['contact_link'])@offset
    roller_points=np.array([(np.array(d['tcp_world'])@tcp_from_roller)[:3,3] for d in data])
    (directory/'overlay_geometry.json').write_text(json.dumps(dict(tcp_from_roller_center=tcp_from_roller.tolist(),
        urdf_sha256=hashlib.sha256(Path(bundle['urdf']).read_bytes()).hexdigest(),path_reference='measured rigid roller cylinder center'),indent=2))
    event_times=np.array([e.get('timestamp_ns',0)*1e-9 for e in events])
    feedback=trace['samples'];feedback_times=np.array([s['sim_time'] for s in feedback])
    def state(stamp):
        i=np.searchsorted(event_times,stamp,side='right')-1
        phase=events[i].get('state','UNKNOWN') if i>=0 else 'PREPARATION'
        j=np.searchsorted(feedback_times,stamp,side='right')-1
        sample=feedback[j] if j>=0 else {}
        valid=(bool(sample.get('valid')) and 0<=stamp-sample.get('sim_time',-1)<=.10
               and 0<=sample.get('force_age_s',float('inf'))<=.20
               and 0<=sample.get('state_age_s',float('inf'))<=.20)
        return phase,contact_class(phase,sample.get('measured_contact_force_n'),valid),sample.get('measured_contact_force_n')
    phases=[state(t)[0] for t in times]
    checks=[]
    for i,event in enumerate(events):
        if event.get('state')!='PAINT' or (i and events[i-1].get('state')=='PAINT'):continue
        end=next((e for e in events[i+1:] if e.get('state')!='PAINT'),None)
        if end is None:continue
        points=[np.array(item['tcp_world'])[:3,3] for item in data
                if event['timestamp_ns']*1e-9<=item['sim_time']<end['timestamp_ns']*1e-9]
        checks.append(stroke_displacement(points,report['direction'],report['side_m']))
    measured_passed=report['passed'] and len(checks)==3 and all(c['passed'] for c in checks)
    (directory/'measured_path_validation.json').write_text(json.dumps(dict(passed=measured_passed,strokes=checks),indent=2))
    report['passed']=measured_passed
    with (directory/'joint_states.csv').open('w') as stream:
        fields=['sim_time','phase','contact_class','force_n']+[n+'_rad' for n in meta['joint_names']]+[n+'_rad_s' for n in meta['joint_names']]+['tcp_x_m','tcp_y_m','tcp_z_m','roller_x_m','roller_y_m','roller_z_m']
        writer=csv.writer(stream);writer.writerow(fields)
        for i,item in enumerate(data):
            phase,kind,force=state(item['sim_time'])
            writer.writerow([item['sim_time'],phase,kind,force,*item['positions'],*item['velocities'],*np.array(item['tcp_world'])[:3,3],*roller_points[i]])
    active=[i for i,n in enumerate(meta['joint_names']) if n.startswith('right_manipulator_joint')]
    metrics=motion_metrics(times,qs[:,active],phases)
    groups=[]
    paint_starts=[e['timestamp_ns']*1e-9 for i,e in enumerate(events)
                  if e.get('state')=='PAINT' and (i==0 or events[i-1].get('state')!='PAINT')]
    for t,phase in zip(times,phases):
        kind=state(t)[1]
        between_strokes=len(paint_starts)==3 and paint_starts[0]<t<paint_starts[2]
        groups.append('contact_stroke' if phase=='PAINT' else 'noncontact_transfer' if kind=='noncontact' and between_strokes else 'other')
    grouped=motion_metrics(times,qs[:,active],groups)
    (directory/'joint_metrics.json').write_text(json.dumps(dict(passed=report['passed'],joint_names=[meta['joint_names'][i] for i in active],phases=metrics,groups=grouped),indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(13,8),sharex=True)
    for i in active:
        axes[0].plot(times-times[0],np.degrees(qs[:,i]-qs[0,i]),label=meta['joint_names'][i])
        axes[1].plot(times-times[0],np.degrees([d['velocities'][i] for d in data]))
    for e,nxt in zip(events,events[1:]):
        if e.get('state')=='PAINT':
            for ax in axes:ax.axvspan(e['timestamp_ns']*1e-9-times[0],nxt['timestamp_ns']*1e-9-times[0],color='green',alpha=.12)
    axes[0].set_ylabel('Angle change (deg)');axes[1].set_ylabel('Velocity (deg/s)');axes[1].set_xlabel('Simulation time since recording start (s)')
    axes[0].legend(fontsize=8);fig.suptitle(directory.name+(' — COMPLETE' if report['passed'] else ' — INCOMPLETE TRIAL'))
    fig.tight_layout();fig.savefig(directory/'joint_states.png',dpi=150);plt.close(fig)
    k=np.array(meta['K']);inverse=np.linalg.inv(np.array(meta['world_from_camera']))
    colors={'contact':(60,220,60),'noncontact':(0,165,255),'transition':(220,100,220),'unknown':(160,160,160)}
    def project(point):
        local=inverse@np.r_[point,1.]
        if local[2]<=.05:return None
        uv=k@local[:3];return tuple(np.rint(uv[:2]/uv[2]).astype(int))
    writer=cv2.VideoWriter(str(directory/'annotated.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),15,(1280,720))
    if not writer.isOpened():raise RuntimeError('MP4 writer unavailable')
    canvas=np.zeros((720,1280,3),dtype=np.uint8);cursor=0;previous=None
    frame_times=np.array([f['sim_time'] for f in frames])
    if np.any(np.diff(frame_times)<=0):raise ValueError('Frame timestamps must increase')
    output_times=np.arange(frame_times[0],frame_times[-1]+1e-8,1/15)
    for output_stamp in output_times:
        frame=frames[max(0,np.searchsorted(frame_times,output_stamp,side='right')-1)]
        stamp=frame['sim_time']
        while cursor<len(data) and times[cursor]<=stamp:
            item=data[cursor];phase,kind,_=state(times[cursor]);point=project(roller_points[cursor])
            # Preparatory D405 movements are recorded but excluded from paint trails.
            if phase not in ('PREPARATION','IDLE','UNKNOWN') and previous is not None and point is not None:
                old,old_kind,old_time=previous
                if old is not None and old_kind==kind and times[cursor]-old_time<=.05 and (kind!='noncontact' or cursor//5%2==0):
                    cv2.line(canvas,old,point,colors[kind],3,cv2.LINE_AA)
            previous=(point,kind,times[cursor]);cursor+=1
        img=cv2.imread(str(directory/'frames'/f"{frame['frame']:07d}.jpg"))
        if img is None:raise ValueError('Missing recorded frame')
        raw=img.copy()
        if output_stamp-stamp>.20:
            img[:]=0
            cv2.putText(img,'MISSING CAMERA FRAME',(350,350),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),2)
        mask=np.any(canvas,axis=2);img[mask]=canvas[mask]
        phase,kind,force=state(stamp)
        cv2.rectangle(img,(0,0),(1280,76),(25,25,25),-1)
        stroke_count=sum(e.get('state')=='PAINT' and (i==0 or events[i-1].get('state')!='PAINT') for i,e in enumerate(events) if e.get('timestamp_ns',0)*1e-9<=stamp)
        force_text=f'{force:.1f} N' if force is not None else 'unknown force'
        cv2.putText(img,f'{directory.name.upper()} | sim {stamp:.2f}s | {phase} | stroke {stroke_count}/3 | {force_text}',(18,28),cv2.FONT_HERSHEY_SIMPLEX,.60,(255,255,255),1,cv2.LINE_AA)
        cv2.putText(img,'GREEN: measured contact  ORANGE dashed: noncontact  PURPLE: transition  GRAY: unknown',(18,58),cv2.FONT_HERSHEY_SIMPLEX,.48,(230,230,230),1,cv2.LINE_AA)
        if cursor and phase not in ('PREPARATION','IDLE','UNKNOWN'):
            point=project(roller_points[cursor-1])
            if point is not None and 0<=point[0]<1280 and 0<=point[1]<720:
                x=int(np.clip(point[0]-100,0,1080));y=int(np.clip(point[1]-100,0,520))
                detail=cv2.resize(raw[y:y+200,x:x+200],(320,320),interpolation=cv2.INTER_CUBIC)
                img[110:430,950:1270]=detail
                cv2.rectangle(img,(950,110),(1270,430),(220,220,220),1)
                cv2.putText(img,'ROLLER / WALL DETAIL',(950,100),cv2.FONT_HERSHEY_SIMPLEX,.48,(30,30,30),1,cv2.LINE_AA)
        if not report['passed']:cv2.putText(img,'INCOMPLETE TRIAL',(18,700),cv2.FONT_HERSHEY_SIMPLEX,.7,(70,70,255),2)
        writer.write(img)
    writer.release()
    return dict(source_frames=len(frames),output_frames=len(output_times),samples=len(data),passed=report['passed'])


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    print(json.dumps(render(parser.parse_args().directory)))
