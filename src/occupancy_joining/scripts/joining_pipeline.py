#!/usr/bin/env python3
"""Prepare local submaps, call the C++ backend, and export three ROS2 map states."""
import argparse,json,os,sys,subprocess,time
from pathlib import Path
import numpy as np
import yaml
from geometry import se2,pose2
from grid_io import build_map,write_grid,read_grid,export_image
from project_paths import ROOT

DEFAULTS=dict(input=dict(type='txt',dataset=None,ranges=None,poses=None,odom=None,start=0,stride=1,max_scans=0,angle_min=-2.356194490192345,angle_max=2.356194490192345,min_range=.1,max_range=30.,scan_period=.1,ground_truth=None),
 submaps=dict(partition='frames',max_extent=30.,max_travel=60.,overlap_travel=6.,scans_per_submap=1000,overlap=0,local_resolution=.1,cell_update_policy='first_sample',anchor_poses=None),
 optimizer=dict(model='matlab_inverse',max_iterations=70,tolerance=1e-7,global_resolution=.1,padding=5.,minimum_count=1e-9,max_cells=2000000,huber_delta=1.,n_smoothing=1e-6,inverse_delete_threshold=.5,count_delete_threshold=.001,inverse_gradient='matlab',max_backtracks=10,continuation_sigmas=[0.]),
 display=dict(resolution=.05,cell_update_policy='first_sample',max_cells=None,grid_format='text'),bag=dict(pose_estimate='causal',graph_exporter=None,scan_topic='auto',odom_source='auto',odom_topic='/odom',tf_topic='/tf',tf_static_topic='/tf_static',odom_frame='odom',tf_max_gap=.2,projection_frame=None,min_height=-.2,max_height=.2,bins=720,icp_distance=.8,beam_stride=1,scan_stride=1,start=0,max_scans=0,min_range=.1,max_range=30.,slam_config=None,frontend_resolution=None,scan_timeout=3.,startup_timeout=30.,min_accepted_fraction=.8,domain_id=87))
PATHS={'dataset','ranges','poses','odom','anchor_poses','slam_config','graph_exporter','ground_truth'}

def settings(path):
    import copy
    cfg=copy.deepcopy(DEFAULTS)
    if path:
        raw=yaml.safe_load(path.read_text())
        if not isinstance(raw,dict) or raw.keys()-cfg.keys():raise ValueError('Unknown configuration section')
        for section,items in raw.items():
            if not isinstance(items,dict) or items.keys()-cfg[section].keys():raise ValueError(f'Unknown {section} key')
            cfg[section].update(items)
            for key,val in items.items():
                if key in PATHS and val is not None:cfg[section][key]=str((path.parent/Path(val).expanduser()).resolve())
    return cfg

def manifest_optimizer(manifest_options, config_options):
    """Keep stored optimizer settings, then apply only explicitly supplied YAML keys."""
    if isinstance(manifest_options,dict):
        manifest_options={k:v for k,v in manifest_options.items() if k!='map_regularization'}
    if not isinstance(manifest_options,dict) or manifest_options.keys()-DEFAULTS['optimizer'].keys():
        raise ValueError('Unknown optimizer key in submap manifest')
    return dict(DEFAULTS['optimizer'],**manifest_options) | config_options

def odom_to_pose(odom):
    odom=np.asarray(odom,float)
    result=np.empty_like(odom);result[0]=odom[0]
    for i in range(1,len(odom)):result[i]=pose2(se2(result[i-1])@se2(odom[i]))
    return result

def matrix(path):return np.loadtxt(path,ndmin=2)

def save_timestamps(path,times):
    Path(path).write_text(''.join(f'{int(t)//1_000_000_000}.{int(t)%1_000_000_000:09d}\n' for t in times))

def load_input(cfg,out):
    a=cfg['input'];gt=None;timestamps=None
    if a['type']=='txt':
        dataset=Path(a['dataset']) if a['dataset'] else None
        def file(key,default):return Path(a[key]) if a[key] else dataset/default if dataset else None
        rp=file('ranges','Range.txt');pp=file('poses','Pose.txt');op=file('odom','Odom.txt')
        if rp is None:raise ValueError('Supply --dataset or input.ranges')
        ranges=matrix(rp)
        if pp and pp.is_file():poses=matrix(pp)
        elif op and op.is_file():poses=odom_to_pose(matrix(op))
        else:raise ValueError('TXT needs Pose.txt or incremental Odom.txt')

    elif a['type']=='bag':
        from argparse import Namespace
        from scan_io import bag_inputs
        from slam_frontend import run_slam
        bag=dict(cfg['bag'],bag=a['dataset']);bag['slam_config']=Path(bag['slam_config']) if bag['slam_config'] else ROOT/'config/slam_toolbox.yaml'
        if not bag['bag']:raise ValueError('Supply --dataset with a ROS1 bag or ROS2 bag directory')
        args=Namespace(**bag);scans,priors,source,diagnostics=bag_inputs(args)
        front=out/'frontend';front.mkdir()
        accepted,poses=run_slam(scans,priors,args,front)
        scans=[scans[i] for i in accepted];ranges=np.asarray([s.ranges for s in scans])
        a.update(angle_min=scans[0].angle_min,angle_max=scans[0].angle_min+(len(scans[0].ranges)-1)*scans[0].angle_increment,min_range=scans[0].range_min,max_range=scans[0].range_max)
        timestamps=np.asarray([s.time for s in scans],dtype=np.int64)
        (front/'input_topics.json').write_text(json.dumps(args.resolved_input_topics,indent=2))
        (front/'odometry.json').write_text(json.dumps(dict(source=source,diagnostics=diagnostics),indent=2))
    else:raise ValueError('Input type must be txt, bag or submaps')
    if a['type']=='txt' and a['ground_truth']:
        gt=matrix(a['ground_truth'])
        if gt.shape!=(len(ranges),3) or not np.isfinite(gt).all():raise ValueError('Ground truth rows do not match the input')
    if ranges.ndim!=2 or poses.ndim!=2 or poses.shape!=(len(ranges),3) or len(ranges)<2 or not np.isfinite(poses).all():raise ValueError('Scan/pose rows do not match or contain invalid poses')
    sel=np.arange(len(ranges))[a['start']::a['stride']]
    if a['max_scans']:sel=sel[:a['max_scans']]
    if len(sel)<2:raise ValueError('Selection contains fewer than two scans')
    ranges,poses=ranges[sel],poses[sel]
    if timestamps is not None:save_timestamps(out/'timestamps.txt',timestamps[sel])
    if gt is not None:gt=gt[sel];np.savetxt(out/'groundtruth.txt',gt,fmt='%.17g')
    np.savetxt(out/'frame_indices.txt',sel,fmt='%d')
    return ranges,poses,gt

def backend_path(override):
    if override:return Path(override).resolve()
    if os.environ.get('OCCUPANCY_JOINING_BACKEND'):return Path(os.environ['OCCUPANCY_JOINING_BACKEND'])
    candidates=[Path(__file__).parent/'joining_backend',ROOT.parents[1]/'build/src/occupancy_joining/joining_backend',ROOT.parents[1]/'build/occupancy_joining/joining_backend']
    try:
        from ament_index_python.packages import get_package_prefix
        candidates.insert(0,Path(get_package_prefix('occupancy_joining'))/'lib/occupancy_joining/joining_backend')
    except (ImportError,LookupError):pass
    for p in candidates:
        if p.is_file():return p
    raise ValueError('Build joining_backend first, or set OCCUPANCY_JOINING_BACKEND')

def relative_blocks(poses,size,overlap):
    if not isinstance(size,int) or not isinstance(overlap,int) or not 0<=overlap<size or size<2:raise ValueError('Need 0 <= overlap < scans_per_submap, with size >= 2')
    result=[]
    for start in range(0,len(poses),size):
        begin=max(0,start-overlap);end=min(len(poses),start+size);anchor=poses[start]
        local=np.asarray([pose2(np.linalg.inv(se2(anchor))@se2(p)) for p in poses[begin:end]])
        result.append((begin,end,anchor.copy(),local,start))
    return result

def partition_blocks(poses, settings):
    if settings['partition']=='frames':return relative_blocks(poses,settings['scans_per_submap'],settings['overlap'])
    from partitioning import spatial_blocks
    return spatial_blocks(poses,settings['scans_per_submap'],settings['max_extent'],settings['max_travel'],settings['overlap_travel'])

def trajectory(anchors,blocks,n):
    result=np.zeros((n,3));written=np.zeros(n,bool)
    for anchor,(begin,end,_,local,_) in zip(anchors,blocks):
        for j,p in enumerate(local):
            k=begin+j
            if not written[k]:result[k]=pose2(se2(anchor)@se2(p));written[k]=True
    if not written.all():raise ValueError('Unassigned trajectory frames')
    return result

def validate(cfg):
    a=cfg['input'];o=cfg['optimizer'];s=cfg['submaps']
    if a['type'] not in ('txt','bag','submaps'):raise ValueError('Input type must be txt, bag or submaps')
    for section,items in cfg.items():
        for key,val in items.items():
            if isinstance(val,float) and not np.isfinite(val):raise ValueError(f'Nonfinite {section}.{key}')
    if a['stride']<1 or a['start']<0 or a['max_scans']<0 or not 0<=a['min_range']<a['max_range'] or a['angle_min']>=a['angle_max']:raise ValueError('Invalid input selection or geometry')
    if min(s['local_resolution'],o['global_resolution'],cfg['display']['resolution'])<=0:raise ValueError('Resolutions must be positive')
    d=cfg['display'];limit=d['max_cells']
    if d['grid_format'] not in ('text','npz') or (limit is not None and (not isinstance(limit,int) or isinstance(limit,bool) or limit<4)):raise ValueError('Invalid display storage/cell budget')
    if s['partition'] not in ('frames','spatial') or not isinstance(s['scans_per_submap'],int) or isinstance(s['scans_per_submap'],bool) or s['scans_per_submap']<2:raise ValueError('Invalid submap partition')
    if s['partition']=='spatial' and (min(s['max_extent'],s['max_travel'])<=0 or not 0<=s['overlap_travel']<s['max_travel']):raise ValueError('Invalid spatial bounds/overlap')
    if s['cell_update_policy'] not in ('first_sample','hit_priority') or cfg['display']['cell_update_policy'] not in ('first_sample','hit_priority'):raise ValueError('Invalid cell update policy')
    if o['inverse_gradient'] not in ('matlab','metric'):raise ValueError('Invalid inverse gradient mode')
    if o['model'] not in ('matlab_inverse','globalized_inverse') or o['max_iterations']<0:raise ValueError('Invalid optimizer model/iteration limit')
    sigmas=o['continuation_sigmas']
    if not isinstance(o['max_backtracks'],int) or not 1<=o['max_backtracks']<=30 or not isinstance(sigmas,list) or not sigmas or sigmas[-1]!=0 or any(not isinstance(x,(int,float)) or not np.isfinite(x) or not 0<=x<=128 for x in sigmas):raise ValueError('Invalid globalization schedule')
    b=cfg['bag']
    if b['pose_estimate'] not in ('causal','final_graph'):raise ValueError('Invalid bag pose_estimate')
    if min(b['tf_max_gap'],b['scan_timeout'],b['startup_timeout'],b['icp_distance'])<=0 or not 0<b['min_accepted_fraction']<=1 or not 0<=b['domain_id']<=232 or b['bins']<2 or min(b['scan_stride'],b['beam_stride'])<1 or b['min_height']>=b['max_height']:raise ValueError('Invalid bag geometry/timeouts')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path)
    p.add_argument('--input-type',choices=('txt','bag','submaps'))
    p.add_argument('--dataset',type=Path);p.add_argument('--manifest',type=Path)
    p.add_argument('--output',type=Path);p.add_argument('--backend',type=Path)
    p.add_argument('--max-scans',type=int);p.add_argument('--iterations',type=int);p.add_argument('--display-resolution',type=float)
    args=p.parse_args();cfg=settings(args.config.resolve() if args.config else None)
    for key,attr in [('type','input_type'),('dataset','dataset'),('max_scans','max_scans')]:
        val=getattr(args,attr)
        if val is not None:cfg['input'][key]=str(val.resolve()) if isinstance(val,Path) else val
    supplied_submaps=None
    if cfg['input']['type']=='submaps':
        if not args.manifest:raise ValueError('submaps input requires --manifest')
        supplied_submaps=yaml.safe_load(args.manifest.read_text())
        if not isinstance(supplied_submaps,dict) or not isinstance(supplied_submaps.get('submaps'),list):
            raise ValueError('Manifest requires a submaps list')
        explicit=yaml.safe_load(args.config.read_text()).get('optimizer',{}) if args.config else {}
        cfg['optimizer']=manifest_optimizer(supplied_submaps.get('optimizer',{}),explicit)
    if args.iterations is not None:cfg['optimizer']['max_iterations']=args.iterations
    if args.display_resolution is not None:cfg['display']['resolution']=args.display_resolution
    validate(cfg)
    default_out=Path(os.environ['OCCUPANCY_JOINING_RUN_DIR'])/'result' if os.environ.get('OCCUPANCY_JOINING_RUN_DIR') else Path(os.environ.get('OCCUPANCY_JOINING_WORKSPACE',str(Path.home()/'occupancy-joining-workspace')))/'runs'/f'{time.strftime("%Y%m%d_%H%M%S")}_{os.getpid()}'
    out=args.output.resolve() if args.output else default_out
    if out.exists() and any(out.iterdir()):raise ValueError('Output must be empty')
    out.mkdir(parents=True,exist_ok=True)
    # Keep caches and ROS logs with this run; standalone commands also remain portable.
    for key,sub in [('ROS_LOG_DIR','logs/ros'),('MPLCONFIGDIR','scratch/matplotlib')]:
        folder=out/sub;folder.mkdir(parents=True,exist_ok=True);os.environ[key]=str(folder)
    (out/'effective_config.yaml').write_text(yaml.safe_dump(cfg));status={'status':'running'}
    try:
        ranges=poses=gt=blocks=None
        if cfg['input']['type']=='submaps':
            data=supplied_submaps
            for item in data['submaps']:item['grid']=str((args.manifest.resolve().parent/Path(item['grid'])).resolve())
            data['optimizer']=cfg['optimizer']
        else:
            ranges,poses,gt=load_input(cfg,out);a=cfg['input'];s=cfg['submaps']
            angles=np.linspace(a['angle_min'],a['angle_max'],ranges.shape[1]);blocks=partition_blocks(poses,s)
            anchors=np.array([b[2] for b in blocks])
            if s['anchor_poses']:
                anchors=matrix(s['anchor_poses'])
                if anchors.shape!=(len(blocks),3) or not np.isfinite(anchors).all():raise ValueError('Wrong anchor_poses shape')
            np.savetxt(out/'input_poses.txt',poses);np.savetxt(out/'initial_anchors.txt',anchors)
            data=dict(optimizer=cfg['optimizer'],submaps=[])
            for i,(begin,end,_,local,anchor_index) in enumerate(blocks):
                print(f'Building local submap {i+1}/{len(blocks)} ({end-begin} scans)',flush=True)
                g=build_map(ranges[begin:end],local,angles,s['local_resolution'],a['min_range'],a['max_range'],cfg['optimizer']['max_cells'],s['cell_update_policy'])
                path=out/'submaps'/f'{i:04d}.grid';write_grid(g,path)
                data['submaps'].append(dict(grid=str(path),pose=anchors[i].tolist(),begin=begin,end=end,anchor_index=anchor_index))
        manifest=out/'submaps.yaml';manifest.write_text(yaml.safe_dump(data))
        command=[str(backend_path(args.backend)),'--manifest',str(manifest),'--output',str(out/'maps')]
        (out/'backend_command.json').write_text(json.dumps(command,indent=2))
        with (out/'backend.log').open('w') as log:subprocess.run(command,check=True,stdout=log,stderr=subprocess.STDOUT)
        metrics={}
        for method in ('initial','optimized','reintegrated'):
            folder=out/'maps'/method;g=read_grid(folder/'map.grid');export_image(g,folder)
            if blocks is not None:
                optimized=matrix(folder/'submap_poses.txt');robot=trajectory(optimized,blocks,len(poses));np.savetxt(folder/'trajectory.txt',robot,fmt='%.17g')
                if gt is not None:
                    # The first anchor is fixed: all states share the input gauge, with no refitted alignment.
                    alignment=se2(poses[0])@np.linalg.inv(se2(gt[0]));aligned=np.asarray([pose2(alignment@se2(t)) for t in gt])
                    error=robot-aligned;error[:,2]=(error[:,2]+np.pi)%(2*np.pi)-np.pi
                    metrics[method]=dict(translation_rmse_m=float(np.sqrt(np.mean(np.sum(error[1:,:2]**2,axis=1)))),yaw_rmse_rad=float(np.sqrt(np.mean(error[1:,2]**2))))
                if method!='optimized':
                    display=build_map(ranges,robot,angles,cfg['display']['resolution'],a['min_range'],a['max_range'],cfg['display']['max_cells'] or cfg['optimizer']['max_cells'],cfg['display']['cell_update_policy'])
                    write_grid(display,folder/('display.npz' if cfg['display']['grid_format']=='npz' else 'display.grid'));export_image(display,folder/'display')
            else:write_grid(g,folder/'display.grid') # no raw rays: preserve native resolution explicitly
        (out/'metrics.json').write_text(json.dumps(metrics,indent=2,allow_nan=False))
        (out/'effective_config.yaml').write_text(yaml.safe_dump(cfg))
        status=dict(status='completed',input=cfg['input']['type'],model=cfg['optimizer']['model'],display_resolution=cfg['display']['resolution'] if blocks else 'native: raw scans unavailable')
        print(f'Completed. Result: {out}',flush=True)
    except Exception as exc:
        status=dict(status='failed',error=str(exc));raise
    finally:(out/'status.json').write_text(json.dumps(status,indent=2))

if __name__=='__main__':
    try:main()
    except (ValueError,KeyError,OSError,RuntimeError,subprocess.CalledProcessError) as exc:sys.exit(str(exc))
