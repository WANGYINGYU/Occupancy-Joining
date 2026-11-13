#!/usr/bin/env python3
"""Publish initial/optimized/reintegrated OccupancyGrid and Path topics for RViz2."""
import argparse,os,time,math,subprocess
import yaml
from pathlib import Path
import numpy as np
from grid_io import read_grid,probabilities
from project_paths import ROOT

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--result',type=Path,required=True);p.add_argument('--no-rviz',action='store_true');p.add_argument('--duration',type=float,default=0);p.add_argument('--domain-id',type=int,default=int(os.environ.get('ROS_DOMAIN_ID','88')));p.add_argument('--output',type=Path,help='Viewer settings/log directory')
    from rclpy.utilities import remove_ros_args
    a=p.parse_args(remove_ros_args()[1:])
    if not np.isfinite(a.duration) or a.duration<0 or not 0<=a.domain_id<=232:p.error('Invalid duration/domain')
    os.environ['ROS_DOMAIN_ID']=str(a.domain_id);os.environ['RMW_IMPLEMENTATION']='rmw_fastrtps_cpp'
    os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']=str(ROOT/'config/fastdds_udp.xml');os.environ['FASTDDS_DEFAULT_PROFILES_FILE']=str(ROOT/'config/fastdds_udp.xml')
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid,Path as PathMessage
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from tf2_msgs.msg import TFMessage
    rclpy.init();node=rclpy.create_node('occupancy_joining_results');qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publishers=[];bounds=[]
    states = ('initial','optimized','reintegrated')
    if (a.result/'maps/input/map.grid').is_file():states = ('input',)+states
    for state in states:
        folder=a.result/'maps'/state;path=next((folder/name for name in ('display.npz','display.grid','map.grid') if (folder/name).is_file()),folder/'map.grid');g=read_grid(path)
        observed=np.argwhere(g['counts']>0)
        if len(observed):
            xy=observed[:,::-1]*g['resolution']+g['origin'];bounds.extend([xy.min(axis=0),xy.max(axis=0)])
        m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=node.get_clock().now().to_msg();m.info.resolution=float(g['resolution']);m.info.width=g['width'];m.info.height=g['height'];m.info.origin.orientation.w=1.
        m.info.origin.position.x=float(g['origin'][0]-.5*g['resolution']);m.info.origin.position.y=float(g['origin'][1]-.5*g['resolution'])
        cells=np.rint(probabilities(g)*100).astype(np.int8);cells[g['counts']<=0]=-1;m.data=cells.ravel().tolist()
        pub=node.create_publisher(OccupancyGrid,f'/occupancy_joining/{state}/map',qos);publishers.append((pub,m))
        for name,filename in [('trajectory','trajectory.txt'),('submap_frames','submap_poses.txt')]:
            file=folder/filename
            if not file.exists():continue
            msg=PathMessage();msg.header=m.header
            for row in np.loadtxt(file,ndmin=2):
                pose=PoseStamped();pose.header=m.header;pose.pose.position.x=float(row[0]);pose.pose.position.y=float(row[1]);pose.pose.orientation.z=math.sin(row[2]/2);pose.pose.orientation.w=math.cos(row[2]/2);msg.poses.append(pose)
            pub=node.create_publisher(PathMessage,f'/occupancy_joining/{state}/{name}',qos);publishers.append((pub,msg))
    reference=TransformStamped();reference.header.frame_id='map';reference.header.stamp=node.get_clock().now().to_msg();reference.child_frame_id='occupancy_joining_reference';reference.transform.rotation.w=1.
    static_pub=node.create_publisher(TFMessage,'/tf_static',qos);publishers.append((static_pub,TFMessage(transforms=[reference])))
    process=None
    try:
        if not a.no_rviz:
            if a.output:out=a.output.resolve()
            elif os.environ.get('OCCUPANCY_JOINING_RUN_DIR'):out=Path(os.environ['OCCUPANCY_JOINING_RUN_DIR'])/'viewer'
            elif os.environ.get('ROS_LOG_DIR'):out=Path(os.environ['ROS_LOG_DIR']).resolve().parent/'viewer'
            else:out=Path(os.environ.get('OCCUPANCY_JOINING_WORKSPACE',str(Path.home()/'occupancy-joining-workspace')))/'runs'/f'view_{int(time.time())}_{os.getpid()}'
            out.mkdir(parents=True,exist_ok=True)
            config=yaml.safe_load((ROOT/'config/joining.rviz').read_text())
            if bounds:
                lo=np.min(bounds,axis=0);hi=np.max(bounds,axis=0);center=(lo+hi)/2;size=np.maximum(hi-lo,1.)
                config['Visualization Manager']['Views']['Current'].update(X=float(center[0]),Y=float(center[1]),Scale=float(min(720/size[0],640/size[1])*.75))
            path=out/'joining.rviz';path.write_text(yaml.safe_dump(config,sort_keys=False))
            viewer_env=dict(os.environ)
            process=subprocess.Popen(['rviz2','-d',str(path),'-t','Occupancy Joining - RViz2'],env=viewer_env)
        started=time.monotonic();last=0
        print(f'Publishing {len(publishers)-1} standard ROS2 map/path topics + a static frame reference; domain {a.domain_id}',flush=True)
        while rclpy.ok() and (not a.duration or time.monotonic()-started<a.duration) and (process is None or process.poll() is None):
            if time.monotonic()-last>1:
                for pub,msg in publishers:pub.publish(msg)
                last=time.monotonic()
            rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:pass
    finally:
        if process and process.poll() is None:process.terminate();process.wait(timeout=10)
        node.destroy_node();rclpy.try_shutdown()
    if process and process.returncode not in (0,-15):raise RuntimeError(f'RViz exited with status {process.returncode}')
if __name__=='__main__':main()
