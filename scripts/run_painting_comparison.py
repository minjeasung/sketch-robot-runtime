#!/usr/bin/env python3
"""Execute one three-stroke trial through the existing measured ROS workflow."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from painting_comparison import strokes
from check_platform_workflow import PlatformObserver, validate_probe_context, wall_fixture, project_fixture, live_model_proof
from check_rb20_painting_isaac import run_workflow
from rb20_painting_report import assess_painting


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--direction',choices=['horizontal','vertical'],required=True)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--wait-scale',type=float,default=12.,help='Observation deadline multiplier; control/safety deadlines unchanged')
    args=parser.parse_args()
    if not args.execute:parser.error('--execute required for simulated wrist and painting motion')
    if not np.isfinite(args.wait_scale) or args.wait_scale<=0:parser.error('wait-scale must be finite and positive')
    args.output.mkdir(parents=True,exist_ok=True)
    context=validate_probe_context(args.assets)
    selection=context['bundle']['selection']
    if selection != dict(schema_version=2,execution_mode='simulation',platform='commercial',active_arm='right',process='paint',tools=dict(left='none',right='roller')):
        raise ValueError('trial requires right commercial simulation paint')
    parameters=yaml.safe_load((args.assets/'painting.yaml').read_text())['sketch_to_waypoints']['ros__parameters']
    paths=strokes(args.direction,parameters['roller_length_m'],parameters['fill_overlap'])
    side=parameters['roller_length_m']*(3-2*parameters['fill_overlap'])
    scene=json.loads((args.assets/'scene.json').read_text())
    fixture=wall_fixture(scene,yaml.safe_load((args.assets/'objects.yaml').read_text()),.50)
    center=np.asarray(fixture['point'])
    report=dict(passed=False,direction=args.direction,side_m=side,stroke_count_required=3,
                session_id=context['session_id'],parameters=parameters,fixture=fixture,joint_names=context['bundle']['arms']['right']['joints'])
    obs=PlatformObserver(args.assets,context,True);obs.fixture=fixture
    from geometry_msgs.msg import PoseArray
    from rclpy.qos import QoSProfile,DurabilityPolicy
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    obs.node.create_subscription(PoseArray,'/perception/wall_front_extent',lambda msg:obs.store('extent',msg),qos)
    obs.node.create_subscription(PoseArray,'/perception/work_area_corners',lambda msg:obs.store('accepted_corners',msg),qos)

    def geometry(kind,observer,width,height):
        if kind=='area_request':
            area=dict(fixture)
            area['corners']=[(center+np.array([u*(side+.02)/2,0,v*(side+.02)/2])).tolist() for u,v in [(-1,1),(1,1),(1,-1),(-1,-1)]]
            info=observer.latest['zed_info']
            return 'zed_raw',project_fixture(area,scene['zed_world_optical'],info.k,info.width,info.height)
        def matched():
            extent=observer.latest.get('accepted_corners');front=observer.latest.get('front',{})
            return extent is not None and abs(extent.header.stamp.sec+extent.header.stamp.nanosec*1e-9-front.get('stamp',-1))<1e-7
        observer.wait(matched,10.,'timestamp-matched measured front extent')
        msg=observer.latest['accepted_corners']
        width=observer.latest['front']['width'];height=observer.latest['front']['height']
        if kind=='front_area':
            return [[0.,0.],[float(width-1),0.],[float(width-1),float(height-1)],[0.,float(height-1)]]
        extent=np.array([[p.position.x,p.position.y,p.position.z] for p in msg.poses])
        if msg.header.frame_id != observer.active['base_frame']:
            from scipy.spatial.transform import Rotation
            tf=observer.tf.lookup_transform(observer.active['base_frame'],msg.header.frame_id,observer.rclpy.time.Time.from_msg(msg.header.stamp))
            q=tf.transform.rotation;t=tf.transform.translation
            extent=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(extent)+[t.x,t.y,t.z]
        if extent.shape != (4,3):raise ValueError('invalid measured front extent')
        def pixel(pair):
            from scipy.optimize import least_squares
            point=center+np.array([pair[0],0.,pair[1]])
            def residual(uv):
                u,v=uv
                return (1-u)*(1-v)*extent[0]+u*(1-v)*extent[1]+u*v*extent[2]+(1-u)*v*extent[3]-point
            uv=least_squares(residual,[.5,.5]).x
            if np.any(uv < -.005) or np.any(uv > 1.005):
                raise ValueError(f'trial area outside measured view: {uv.tolist()}')
            return (np.clip(uv,0,1)*[width-1,height-1]).tolist()
        report['accepted_work_area_corners']=extent.tolist()
        report['world_strokes']=[[(center+np.array([p[0],0,p[1]])).tolist() for p in path] for path in paths]
        return [[pixel(p) for p in path] for path in paths]

    try:
        report['health']=obs.wait_healthy(45.)
        report['collision_check']=live_model_proof(obs)
        obs.wait(lambda:'zed_info' in obs.latest,10.,'ZED intrinsics')
        info=obs.latest['zed_info']
        polygon=project_fixture(fixture,scene['zed_world_optical'],info.k,info.width,info.height)
        report['identity']=run_workflow(obs,'manual',execute=True,scene=scene,selection_polygon=polygon,
            plane_accuracy=obs.record_fixture_accuracy,workflow_time_scale=args.wait_scale,experiment_geometry=geometry)
        report['painting']=assess_painting(obs.events,obs.samples)
        report['passed']=report['painting']['passed'] and len(report['painting']['strokes'])==3
    except Exception as exc:
        report['error']=str(exc)
        report['last_wait']=getattr(obs,'last_wait',None)
        if obs.mutated:
            from std_msgs.msg import Bool
            try:obs.publish('/motion_abort',Bool(data=True))
            except Exception as abort_error:report['abort_error']=str(abort_error)
    finally:
        report['last_statuses']={k:v for k,v in obs.latest.items() if isinstance(v,dict)}
        (args.output/'trace.json').write_text(json.dumps(dict(events=obs.events,samples=obs.samples)))
        (args.output/'report.json').write_text(json.dumps(report,indent=2))
        obs.close()
    print(json.dumps({k:report[k] for k in ('passed','direction','error') if k in report}),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
