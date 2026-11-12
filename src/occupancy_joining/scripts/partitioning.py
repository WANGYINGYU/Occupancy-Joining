"""Spatial submap boundaries; core ownership and anchor coordinates stay explicit."""
import numpy as np


def spatial_blocks(poses, max_scans, max_extent, max_travel, overlap_travel):
    """Return (begin,end,anchor,local,anchor_index), with unique contiguous cores.

    Limits include overlap. At most20% of the frame budget is reused when a
    stationary segment cannot supply the requested traveled-distance overlap.
    A single input motion exceeding a spatial limit is represented, not dropped.
    """
    poses = np.asarray(poses, dtype=float)
    if poses.ndim != 2 or poses.shape[1] != 3 or len(poses) < 2 or not np.isfinite(poses).all():
        raise ValueError('Need at least two finite x/y/yaw poses')
    if not isinstance(max_scans, int) or isinstance(max_scans, bool) or max_scans < 2:
        raise ValueError('max_scans must be an integer >=2')
    if not all(np.isfinite(x) for x in (max_extent, max_travel, overlap_travel)) or min(max_extent, max_travel) <= 0 or not 0 <= overlap_travel < max_travel:
        raise ValueError('Invalid spatial bounds or overlap distance')
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1))]
    result = []
    start = 0
    overlap_limit = min(max_scans-2, max(1, max_scans//5))
    while start < len(poses):
        begin = start
        if start and overlap_travel:
            begin = int(np.searchsorted(distance, distance[start]-overlap_travel, side='left'))
            begin = max(0, start-overlap_limit, min(begin, start))
        end = start+1
        lower = poses[begin:end, :2].min(axis=0)
        upper = poses[begin:end, :2].max(axis=0)
        while end < len(poses):
            lo = np.minimum(lower, poses[end, :2])
            hi = np.maximum(upper, poses[end, :2])
            if end-begin >= max_scans or np.max(hi-lo) > max_extent or distance[end]-distance[begin] > max_travel:
                break
            lower, upper = lo, hi
            end += 1
        anchor = poses[start].copy()
        c, s = np.cos(anchor[2]), np.sin(anchor[2])
        rotation = np.array([[c, -s], [s, c]])
        local = np.empty((end-begin, 3))
        local[:, :2] = (poses[begin:end, :2]-anchor[:2]) @ rotation
        yaw = poses[begin:end, 2]-anchor[2]
        local[:, 2] = np.arctan2(np.sin(yaw), np.cos(yaw))
        result.append((begin, end, anchor, local, start))
        start = end
    return result
