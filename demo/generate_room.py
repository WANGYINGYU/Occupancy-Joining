#!/usr/bin/env python3
"""Generate an independent synthetic 2D room example in a chosen output directory."""
from pathlib import Path
import argparse,sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src/occupancy_joining/scripts"))
import numpy as np
import yaml
from geometry import se2,pose2
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
i=np.arange(60);truth=np.column_stack((.8*np.cos(i/20),.6*np.sin(i/18),.02*np.sin(i/9)))
angles=np.linspace(-3*np.pi/4,3*np.pi/4,361);ranges=[];initial=[]
bias=[np.array([0.,0.,0.]),np.array([.16,-.09,.025]),np.array([-.13,.11,-.02])]
for j,(x,y,yaw) in enumerate(truth):
    dx,dy=np.cos(angles+yaw),np.sin(angles+yaw)
    with np.errstate(divide='ignore'):
        ranges.append(np.minimum(np.where(dx>0,(5-x)/dx,(-4-x)/dx),np.where(dy>0,(4-y)/dy,(-3-y)/dy)))
    initial.append(pose2(se2(bias[j//20])@se2(truth[j])))
np.savetxt(a.output/'Range.txt',ranges,fmt='%.17g');np.savetxt(a.output/'Pose.txt',initial,fmt='%.17g');np.savetxt(a.output/'GT.txt',truth,fmt='%.17g')
cfg=dict(input=dict(type='txt',dataset='.',ground_truth='GT.txt'),submaps=dict(scans_per_submap=20,local_resolution=.15),optimizer=dict(model='matlab_inverse',global_resolution=.15,max_iterations=70),display=dict(resolution=.05))
(a.output/'demo.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
print(a.output.resolve())
