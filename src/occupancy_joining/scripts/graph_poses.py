"""Associate final native SLAM graph poses to scan times without guessing node order."""
import json
from pathlib import Path
import numpy as np


def associate_final_graph(path, scans):
    rows = np.genfromtxt(path, delimiter=',', names=True, ndmin=1)
    if not len(rows) or not all(name in rows.dtype.names for name in ('time_seconds','id','x','y','yaw')):
        raise ValueError('Invalid exported graph columns')
    seconds = rows['time_seconds']
    if not np.isfinite(np.column_stack([seconds,rows['x'],rows['y'],rows['yaw']])).all():
        raise ValueError('Nonfinite final graph pose')
    times = np.rint(seconds*1e9).astype(np.int64)
    order = np.argsort(times);times = times[order];rows = rows[order]
    if len(np.unique(times))!=len(times) or len(np.unique(rows['id']))!=len(rows):
        raise ValueError('Duplicate final graph timestamps or node IDs')
    indices, poses, receipts, used = [], [], [], set()
    for index, scan in enumerate(scans):
        at = int(np.searchsorted(times,scan.time))
        candidates = [k for k in (at-1,at) if 0<=k<len(times)]
        if not candidates:continue
        k = min(candidates,key=lambda k:abs(int(times[k])-scan.time))
        error = abs(int(times[k])-scan.time)
        # Karto stores seconds as double. Bound only its floating-point rounding;
        # this is not an adjustable temporal synchronization window.
        bound = max(1,int(np.ceil(2*abs(np.spacing(rows['time_seconds'][k]))*1e9)))+1
        if error>bound:continue
        node = int(rows['id'][k])
        if node in used:raise ValueError('One graph node matched multiple input scans')
        used.add(node);indices.append(index);poses.append([rows['x'][k],rows['y'][k],rows['yaw'][k]])
        receipts.append(dict(index=index,node=node,input_ns=scan.time,graph_ns=int(times[k]),error_ns=error,rounding_bound_ns=bound))
    return indices,np.asarray(poses,dtype=float).reshape(-1,3),receipts


def graph_exporter(override=None):
    if override:
        path=Path(override).expanduser().resolve()
        if path.is_file():return path
        raise ValueError('Configured graph exporter does not exist')
    candidates=[Path(__file__).parent/'joining_export_slam_graph']
    try:
        from ament_index_python.packages import get_package_prefix
        candidates.insert(0,Path(get_package_prefix('occupancy_joining'))/'lib/occupancy_joining/joining_export_slam_graph')
    except (ImportError,LookupError):pass
    for path in candidates:
        if path.is_file():return path
    raise ValueError('Final graph initialization requires joining_export_slam_graph; install slam_toolbox development dependencies and rebuild the ROS2 package')
