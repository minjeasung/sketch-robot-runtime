#!/usr/bin/env python3
"""Offline raw-IPC to ROS smoke test. Publishes synthetic cameras in domain 227 only.

Source ROS setup and add src/sketch_control to PYTHONPATH before running.
No camera SDK, robot controller, action client or device-control HTTP request is used.
"""
import os
os.environ['ROS_DOMAIN_ID']='227'
import json, threading, time, tempfile, pathlib
from http.server import HTTPServer, BaseHTTPRequestHandler
import numpy as np
import zmq
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from sensor_msgs_py import point_cloud2
from sketch_control.outpost_bridge_node import OutpostBridge

with tempfile.TemporaryDirectory(prefix='sketch-outpost-') as tmp:
 ctx=zmq.Context(); sockets={}; statuses={}; done=threading.Event(); seq=0
 rgb=np.arange(12,dtype=np.uint8).reshape(2,2,3);depth=np.array([[1000,0],[2000,3000]],np.uint16)
 points=np.array([[0,0,1000.25],[0,1000,2000.5],[1500,1500,3000.75]],np.float32)
 for name,kind in [('zed','zed'),('d405','realsense')]:
  ep='ipc://'+str(pathlib.Path(tmp)/(name+'.raw'));sock=ctx.socket(zmq.PUB);sock.bind(ep);sockets[name]=sock
  statuses[name]=dict(hw_id=name,camera_id='serial-'+name,camera_type=kind,state='streaming',generation=1,
   resolution=[2,2],intrinsics=dict(fx=2,fy=2,cx=0,cy=0),local_raw_endpoint=ep)
 class Handler(BaseHTTPRequestHandler):
  def do_GET(self):
   data=json.dumps(statuses[self.path.rsplit('/',1)[-1]]).encode();self.send_response(200);self.end_headers();self.wfile.write(data)
  def log_message(self,*args): pass
 server=HTTPServer(('127.0.0.1',0),Handler);threading.Thread(target=server.serve_forever,daemon=True).start()
 rclpy.init();params=[Parameter('outpost_http',value=f'http://127.0.0.1:{server.server_port}')]
 for name in sockets:params.extend([Parameter(f'outpost_{name}_hw_id',value=name),Parameter(f'outpost_{name}_serial',value='serial-'+name)])
 bridge=OutpostBridge(parameter_overrides=params);observer=rclpy.create_node('outpost_test_observer');executor=SingleThreadedExecutor();executor.add_node(observer);received={}
 subscriptions=[]
 for topic,typ in [('/zed/zed_node/depth/depth_registered',Image),('/zed/zed_node/rgb/color/rect/camera_info',CameraInfo),('/d405/d405/depth/color/points',PointCloud2),('/d405/d405/color/image_raw',Image)]:
  subscriptions.append(observer.create_subscription(typ,topic,lambda m,t=topic:received.__setitem__(t,m),10))
 def publish():
  seq=0
  while not done.wait(.06):
   seq+=1
   for name,sock in sockets.items():
    arrays=[('rgb',rgb),('depth',depth)]+([('point_cloud',points)] if name=='d405' else [])
    h=dict(hw_id=name,seq=seq,generation=1,capture_timestamp_ns=time.time_ns(),frame_size=[2,2],channels=[dict(kind=k,encoding='raw',dtype=str(a.dtype),shape=list(a.shape),part=i+2) for i,(k,a) in enumerate(arrays)])
    sock.send_multipart([name.encode(),json.dumps(h).encode()]+[a.tobytes() for _,a in arrays])
 publisher=threading.Thread(target=publish);publisher.start();errors=[]
 def run():
  try:bridge.run()
  except Exception as e:errors.append(e)
 worker=threading.Thread(target=run);worker.start()
 try:
  deadline=time.monotonic()+5
  while len(received)<4 and time.monotonic()<deadline and not errors:executor.spin_once(timeout_sec=.1)
  assert len(received)==4,(received.keys(),errors)
  msg=received['/zed/zed_node/depth/depth_registered'];assert msg.encoding=='32FC1' and msg.header.frame_id=='zed_left_camera_frame_optical';assert np.frombuffer(bytes(msg.data),np.float32)[0]==1.
  msg=received['/d405/d405/depth/color/points'];xyz=point_cloud2.read_points_numpy(msg,field_names=['x','y','z']);assert abs(xyz.reshape(-1,3)[0,2]-1.00025)<1e-6
  assert msg.header.frame_id=='d405_color_optical_frame'
  done.set();publisher.join();worker.join(timeout=6)
  assert errors and isinstance(errors[0],TimeoutError),errors
  print('PASS: raw IPC for both cameras -> ROS RGB/depth/CameraInfo/SDK XYZ; frame IDs and units; 3s stream-loss failure')
 finally:
  done.set();publisher.join();rclpy.shutdown();worker.join(3);bridge.close();bridge.destroy_node();observer.destroy_node();executor.shutdown();server.shutdown();server.server_close()
  for sock in sockets.values():sock.close(0)
  ctx.term()
