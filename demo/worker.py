#!/usr/bin/env python3
"""Prepare the published demo inputs, run the unchanged backend, and export maps."""
import argparse,json,subprocess,sys,time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/occupancy_joining/scripts'))
from geometry import se2,pose2
from grid_io import build_map,write_grid,read_grid,export_image
from joining_pipeline import trajectory

def fixed_grid(ranges,poses,angles,spec,min_range,max_range):
    """Preserve the prepared nodal lattice and per-scan hit-priority evidence."""
    h=spec['resolution'];w,height=spec['width'],spec['height'];origin=np.asarray(spec['reference_origin']);shift=np.asarray(spec['index_offset'],dtype=int)
    if w*height>8_000_000 or min(w,height)<=0:raise ValueError('Prepared local grid exceeds the demo limit')
    values=np.zeros(w*height);counts=np.zeros(w*height)
    for scan,pose in zip(ranges,poses):
        valid=np.isfinite(scan)&(scan>min_range)&(scan<=max_range);beam=scan[valid];theta=angles[valid]+pose[2]
        n=(np.ceil(beam/h).astype(int)-1).clip(0)+1;offsets=np.repeat(np.cumsum(n)-n,n);k=np.arange(n.sum())-offsets
        d=(k+1)*h;last=k==np.repeat(n-1,n);d[last]=np.repeat(beam,n)[last];a=np.repeat(theta,n);xy=np.column_stack([d*np.cos(a)+pose[0],d*np.sin(a)+pose[1]])
        ij=np.floor((xy-origin)/h+.5).astype(int)-shift
        if not ((ij>=0).all() and (ij[:,0]<w).all() and (ij[:,1]<height).all()):raise ValueError('A valid ray leaves the prepared lattice')
        ids=ij[:,1]*w+ij[:,0];cells,first=np.unique(ids,return_index=True);evidence=np.where(last[first],.847297860387203,-.405465108108164);evidence[np.searchsorted(cells,np.unique(ids[last]))]=.847297860387203
        np.add.at(values,cells,evidence);np.add.at(counts,cells,1.)
    return dict(width=w,height=height,resolution=h,origin=np.asarray(spec['origin']),values=values.reshape(height,w),counts=counts.reshape(height,w))

def read_b2(bag,data,prepared,meta):
    from rosbags.highlevel import AnyReader
    expected=np.loadtxt(data/'timestamps_ns.txt',dtype=np.int64);path=prepared/'ranges.bin';ranges=np.memmap(path,mode='w+',dtype='<f8',shape=(meta['frames'],meta['beams']));topic=json.loads((ROOT/'demo/b2.json').read_text())['raw']['topic'];count=0
    with AnyReader([bag]) as reader:
        selected=[c for c in reader.connections if c.topic==topic and c.msgtype.endswith('/MultiEchoLaserScan')]
        if len(selected)!=1:raise ValueError(f'Expected one MultiEchoLaserScan connection for {topic}')
        for connection,_,raw in reader.messages(connections=selected):
            msg=reader.deserialize(raw,connection.msgtype);ns=int(msg.header.stamp.sec)*1_000_000_000+int(msg.header.stamp.nanosec)
            if count>=len(expected) or ns!=expected[count] or len(msg.ranges)!=meta['beams']:raise ValueError('Raw scan identities do not match the prepared initialization')
            if msg.angle_min!=meta['angle_min'] or msg.angle_max!=meta['angle_max']:raise ValueError('Unexpected beam geometry')
            ranges[count]=[echo.echoes[0] if len(echo.echoes) else np.inf for echo in msg.ranges];count+=1
    if count!=meta['frames']:raise ValueError('Incomplete b2 scan stream')
    ranges.flush();(prepared/'input_receipt.json').write_text(json.dumps(dict(scans=count,beams=meta['beams'],topic=topic,all_header_times_exact=True,ground_truth=False),indent=2));return np.memmap(path,mode='r',dtype='<f8',shape=(meta['frames'],meta['beams']))

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--demo',choices=['room','b2'],required=True);p.add_argument('--data',type=Path,required=True);p.add_argument('--bag',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--jobs',type=int,default=2);a=p.parse_args();a.data=a.data.resolve(strict=True);a.output=a.output.resolve();prepared=a.output.parent/'prepared';prepared.mkdir(exist_ok=False);pipeline=ROOT/'src/occupancy_joining/scripts/joining_pipeline.py';started=time.monotonic()
    if a.demo=='room':
        command=[sys.executable,str(pipeline),'--config',str(a.data/'demo.yaml'),'--output',str(a.output)]
        subprocess.run(command,check=True)
    else:
        if not a.bag:raise ValueError('b2 requires its original bag')
        meta=json.loads((a.data/'preparation.json').read_text());poses=np.loadtxt(a.data/'initial_poses.txt');assert poses.shape==(meta['frames'],3) and np.isfinite(poses).all();ranges=read_b2(a.bag,a.data,prepared,meta);angles=np.linspace(meta['angle_min'],meta['angle_max'],meta['beams']);specs=meta['submaps'];blocks=[];covered=np.zeros(len(poses),bool)
        for spec in specs:
            begin,end,index=spec['begin'],spec['end'],spec['anchor_index'];anchor=poses[index];c,s=np.cos(anchor[2]),np.sin(anchor[2]);rotation=np.array([[c,-s],[s,c]]);local=np.empty((end-begin,3));local[:,:2]=(poses[begin:end,:2]-anchor[:2])@rotation;delta=poses[begin:end,2]-anchor[2];local[:,2]=np.arctan2(np.sin(delta),np.cos(delta));blocks.append((begin,end,anchor,local,index));covered[begin:end]=True
        if not covered.all():raise ValueError('Prepared submaps do not cover the entire sequence')
        def make(index):
            spec=specs[index];begin,end,anchor,local,_=blocks[index];g=fixed_grid(ranges[begin:end],local,angles,spec,meta['min_range'],meta['max_range']);file=prepared/f'local{index}.grid';write_grid(g,file);print(f'Built submap {index+1}/{len(specs)}',flush=True);return dict(grid=str(file),pose=spec['pose'],begin=begin,end=end,anchor_index=spec['anchor_index'])
        with ThreadPoolExecutor(max_workers=a.jobs) as pool:items=list(pool.map(make,range(len(specs))))
        manifest=prepared/'submaps.yaml';manifest.write_text(yaml.safe_dump(dict(optimizer=meta['optimizer'],submaps=items)));command=[sys.executable,str(pipeline),'--input-type','submaps','--manifest',str(manifest),'--output',str(a.output)];subprocess.run(command,check=True)
        for state in ['initial','optimized','reintegrated']:
            folder=a.output/'maps'/state;anchors=np.loadtxt(folder/'submap_poses.txt');robot=trajectory(anchors,blocks,len(poses));np.savetxt(folder/'trajectory.txt',robot,fmt='%.17g')
            if state!='optimized':
                print(f'Rebuilding full {state} display at 0.1 m...',flush=True);g=build_map(ranges,robot,angles,.1,meta['min_range'],meta['max_range'],8_000_000,'hit_priority');write_grid(g,folder/'display.grid');export_image(g,folder/'display')
    images=[]
    for state,title in [('initial','Initial'),('optimized','Direct optimized map'),('reintegrated','Rebuilt with optimized poses')]:
        folder=a.output/'maps'/state;png=f'maps/{state}/'+('display/map.png' if (folder/'display/map.png').exists() else 'map.png');images.append(f'<section><h2>{title}</h2><a href="{png}"><img src="{png}" alt="{title}"></a></section>')
    note='Synthetic functional example; ground truth is evaluation-only.' if a.demo=='room' else 'Complete 51,833-scan prepared-input example. Supplied range-only Cartographer initialization; explicit globalized_inverse preset. No ground truth or complete-overlap guarantee.'
    metric_html=''
    metrics=json.loads((a.output/'metrics.json').read_text())
    if metrics:
        rows=''.join(f'<tr><td>{state}</td><td>{value["translation_rmse_m"]:.6f}</td><td>{value["yaw_rmse_rad"]:.6f}</td></tr>' for state,value in metrics.items())
        metric_html='<h2>Synthetic GT evaluation</h2><table><tr><th>State</th><th>Translation RMSE [m]</th><th>Yaw RMSE [rad]</th></tr>'+rows+'</table>'
    (a.output/'index.html').write_text('<!doctype html><html lang="en"><meta charset="utf-8"><title>Occupancy Joining demo</title><style>body{font:16px/1.6 sans-serif;margin:24px}main{display:flex;gap:18px}section{flex:1;min-width:0}img{width:100%;height:auto}h2{font-size:18px}</style><h1>Occupancy Joining: '+a.demo+'</h1><p>'+note+'</p><p>Maps share the same world frame. Open individual images at full resolution, or use the ROS 2 RViz Map topic selector.</p><main>'+''.join(images)+'</main>'+metric_html+'</html>')
    (a.output/'demo.json').write_text(json.dumps(dict(demo=a.demo,seconds=time.monotonic()-started,frames=60 if a.demo=='room' else 51833,notes=note,optimizer_source='demo.yaml' if a.demo=='room' else 'preparation.json',display_resolution=.05 if a.demo=='room' else .1),indent=2));print(a.output/'index.html',flush=True)

if __name__=='__main__':main()
