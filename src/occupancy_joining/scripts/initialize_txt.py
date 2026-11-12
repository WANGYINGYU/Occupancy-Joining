#!/usr/bin/env python3
"""Create an explicit SLAM-toolbox trajectory initialization from original 2D TXT data."""
import argparse,json,os,time,copy,shutil
import yaml
from decimal import Decimal
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from joining_pipeline import settings,validate,load_input,save_timestamps
from scan_io import Scan
from slam_frontend import run_slam
from project_paths import ROOT

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path)
    p.add_argument('--timestamps',type=Path,help='Original full-input timestamp rows, in seconds')
    p.add_argument('--graph-poses',type=Path,help='Associate an existing final/refined graph CSV instead of replaying ROS')
    p.add_argument('--pose-estimate',choices=('causal','final_graph'),default='final_graph')
    a=p.parse_args()
    if a.graph_poses and not a.timestamps:p.error('--graph-poses requires original --timestamps')
    cfg=settings(a.config.resolve());validate(cfg)
    if cfg['input']['type']!='txt':p.error('This adapter requires input.type: txt; bags use bag.pose_estimate')
    default_out=Path(os.environ['OCCUPANCY_JOINING_RUN_DIR'])/'result' if os.environ.get('OCCUPANCY_JOINING_RUN_DIR') else Path(os.environ.get('OCCUPANCY_JOINING_WORKSPACE',str(Path.home()/'occupancy-joining-workspace')))/'runs'/f'{time.strftime("%Y%m%d_%H%M%S")}_{os.getpid()}'
    out=a.output.resolve() if a.output else default_out
    if out.exists() and any(out.iterdir()):p.error('Output must be empty')
    out.mkdir(parents=True,exist_ok=True)
    # Ground truth is not an initialization source or a frontend input.
    cfg['input']['ground_truth']=None
    ranges,priors,_=load_input(cfg,out);frames=np.loadtxt(out/'frame_indices.txt',dtype=int,ndmin=1)
    if a.timestamps:
        all_times=np.array([int(Decimal(s.strip())*1000000000) for s in a.timestamps.read_text().splitlines() if s.strip()],np.int64)
        if len(all_times)<=frames.max():p.error('Timestamp file must cover original input frame indices')
        times=all_times[frames];time_source='supplied timestamp file'
    else:
        times=np.rint((frames+1)*cfg['input']['scan_period']*1e9).astype(np.int64);time_source='synthetic, original frame index and configured scan_period'
    if np.any(np.diff(times)<=0):p.error('Timestamps must increase')
    inp=cfg['input'];increment=(inp['angle_max']-inp['angle_min'])/(ranges.shape[1]-1)
    scans=[Scan(int(t),'occupancy_laser',rr,inp['angle_min'],increment,inp['min_range'],inp['max_range']) for t,rr in zip(times,ranges)]
    if a.graph_poses:
        from graph_poses import associate_final_graph
        accepted,poses,receipts=associate_final_graph(a.graph_poses,scans)
        if len(accepted)<2 or len(accepted)/len(scans)<cfg['bag']['min_accepted_fraction']:
            raise ValueError('Insufficient timestamp-associated graph poses')
        (out/'graph_association.json').write_text(json.dumps(receipts,indent=2))
        shutil.copy2(a.graph_poses,out/'graph_poses.csv')
        estimate='external_graph';source='timestamp-associated graph CSV, no GT'
    else:
        front=out/'frontend';front.mkdir();args=SimpleNamespace(**cfg['bag']);args.pose_estimate=a.pose_estimate
        args.slam_config=Path(args.slam_config) if args.slam_config else ROOT/'config/slam_toolbox.yaml'
        accepted,poses=run_slam(scans,priors,args,front)
        estimate=a.pose_estimate;source='slam_toolbox, no GT'
    np.savetxt(out/'Range.txt',ranges[accepted],fmt='%.17g');np.savetxt(out/'Pose.txt',poses,fmt='%.17g');np.savetxt(out/'input_priors.txt',priors[accepted],fmt='%.17g');np.savetxt(out/'accepted_frame_indices.txt',frames[accepted],fmt='%d');save_timestamps(out/'timestamps.txt',times[accepted])
    (out/'initialization.json').write_text(json.dumps(dict(status='completed',pose_estimate=estimate,source=source,input_prior='provided correlated scan-matching/odometric poses, not independent ground truth',input_frames=len(ranges),output_frames=len(accepted),timestamps=time_source,missing_frames='not fabricated; output rows contain only explicitly associated scans'),indent=2))
    # This recipe consumes the exported subset once; do not reapply start/stride.
    recipe={key:copy.deepcopy(cfg[key]) for key in ['input','submaps','optimizer','display']}
    recipe['input'].update(type='txt',dataset='.',ranges=None,poses=None,odom=None,
                           ground_truth=None,start=0,stride=1,max_scans=0)
    (out/'joining.yaml').write_text(yaml.safe_dump(recipe,sort_keys=False))
    print(f'Initialization exported: {out}',flush=True)
if __name__=='__main__':main()
