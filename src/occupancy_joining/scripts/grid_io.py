"""Row-major nodal grids: raw log-odds sums and independent hit counts."""
from pathlib import Path
import numpy as np
from PIL import Image
import yaml

def read_grid(path):
    with Path(path).open() as f:
        w,h,res,x,y=map(float,f.readline().split()); data=np.loadtxt(f,ndmin=2)
    w,h=int(w),int(h)
    if data.shape!=(w*h,2) or not np.isfinite(data).all() or np.any(data[:,1]<0):
        raise ValueError(f'Invalid grid: {path}')
    return dict(width=w,height=h,resolution=res,origin=np.array([x,y]),values=data[:,0].reshape(h,w),counts=data[:,1].reshape(h,w))

def write_grid(g,path):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w') as f:
        f.write(f'{g["width"]} {g["height"]} {g["resolution"]:.17g} {g["origin"][0]:.17g} {g["origin"][1]:.17g}\n')
        np.savetxt(f,np.column_stack((g['values'].ravel(),g['counts'].ravel())),fmt='%.17g')

def probabilities(g):
    return 1/(1+np.exp(-np.clip(g['values'],-60,60)))

def export_image(g,folder):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    p=probabilities(g);img=np.rint(255*(1-p)).astype(np.uint8)
    img[g['counts']<=0]=128
    Image.fromarray(img[::-1]).save(folder/'map.png')
    origin=g['origin']-.5*g['resolution']
    (folder/'map.yaml').write_text(yaml.safe_dump(dict(image='map.png',resolution=g['resolution'],origin=[float(origin[0]),float(origin[1]),0.],negate=0,occupied_thresh=.65,free_thresh=.196)))

def build_map(ranges,poses,angles,scale,min_range,max_range,max_cells=2000000,cell_update_policy='first_sample'):
    """One count per scan/cell; selectable original or endpoint-preserving evidence."""
    if cell_update_policy not in ('first_sample','hit_priority'):raise ValueError('Unknown cell update policy')
    if len(ranges)!=len(poses) or len(ranges)==0: raise ValueError('No corresponding scan/pose rows')
    lo=np.array([np.inf,np.inf]);hi=-lo
    for scan,pose in zip(ranges,poses):
        good=np.isfinite(scan)&(scan>min_range)&(scan<=max_range)
        a=angles[good]+pose[2];p=np.column_stack((scan[good]*np.cos(a)+pose[0],scan[good]*np.sin(a)+pose[1]))
        p=np.vstack((p,pose[:2]));lo=np.minimum(lo,p.min(axis=0));hi=np.maximum(hi,p.max(axis=0))
    lo-=5*scale;hi+=5*scale;w,h=(np.ceil((hi-lo)/scale).astype(int)+1)
    if w*h>max_cells: raise ValueError('Local/display grid exceeds max_cells; change resolution or split the sequence')
    values=np.zeros(w*h);counts=values.copy()
    for scan,pose in zip(ranges,poses):
        good=np.isfinite(scan)&(scan>min_range)&(scan<=max_range)
        beam=scan[good];a=angles[good]+pose[2]
        n=(np.ceil(beam/scale).astype(int)-1).clip(0)+1
        # Beam order and within-beam sample order match MATLAB stable unique.
        offsets=np.repeat(np.cumsum(n)-n,n);k=np.arange(n.sum())-offsets
        d=(k+1)*scale;last=k==np.repeat(n-1,n);d[last]=np.repeat(beam,n)[last]
        theta=np.repeat(a,n);q=np.column_stack((d*np.cos(theta)+pose[0],d*np.sin(theta)+pose[1]))
        xy=np.floor((q-lo)/scale+.5).astype(int);ids=xy[:,1]*w+xy[:,0]
        cells,first=np.unique(ids,return_index=True)
        evidence=np.where(last[first],.847297860387203,-.405465108108164)
        if cell_update_policy=='hit_priority':
            # A free sample may round into its own/later beam's endpoint cell.
            # Keep one update, but never let free-space traversal erase a hit.
            evidence[np.searchsorted(cells,np.unique(ids[last]))]=.847297860387203
        np.add.at(values,cells,evidence)
        np.add.at(counts,cells,1.)
    return dict(width=int(w),height=int(h),resolution=float(scale),origin=lo,values=values.reshape(h,w),counts=counts.reshape(h,w))
