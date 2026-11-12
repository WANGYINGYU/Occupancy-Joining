# Input and configuration reference

All numerical values use metres and radians. YAML paths are relative to the
file; CLI paths are relative to the caller. Unknown keys are rejected. Ordinary
input configuration is in `src/occupancy_joining/config/`.

## Text and MATLAB

TXT input accepts `Range.txt` plus absolute sensor-to-world `Pose.txt` rows
`x y yaw`. Alternatively, `Odom.txt` contains the absolute first pose followed
by SE(2) increments expressed in the preceding sensor frame. Optional
`input.ground_truth` names a separate evaluation-only pose file.

`input.start`, `stride` and `max_scans` select rows. Set `angle_min`,
`angle_max`, `min_range` and `max_range` for the actual sensor. Timestamps are
not required for joining prepared TXT poses.

MAT v5/v7 and v7.3 inputs use SciPy/h5py. Copy `simulation.yaml` and set
`input.mat`. Configure `mat_range_key`, `mat_pose_key` and `mat_odom_key` for
other field names. The default `pose_source: odometry` initializes from
odometry. Ground-truth poses can be supplied separately for evaluation.

```bash
bash tools/occupancy-joining run --config /absolute/simulation.yaml \
  --mat /absolute/sequence.mat --output /absolute/results
```

## Bags

Copy the LaserScan or PointCloud2 profile and `slam_toolbox.yaml` together.
`bag.scan_topic`, `odom_topic`, `tf_topic`, `tf_static_topic` and `odom_frame`
are configurable; scan topic `auto` requires an unambiguous supported stream.
MultiEchoLaserScan uses the first recorded echo; an empty echo is missing.

`odom_source` selects automatic, recorded odometry, TF or scan-matching motion
priors. `projection_frame`, `min_height`, `max_height` and `bins` control the
PointCloud2-to-2D conversion. `slam_config` controls the ROS 2 frontend.

The default pose estimate is causal. Set `bag.pose_estimate: final_graph` to
extract the final serialized `slam_toolbox` graph. Missing timestamp-associated
poses are excluded and reported. The final-graph helper requires the `slam_toolbox`
development libraries included by the documented Docker/native dependencies.

## Submap partitioning

`submaps.scans_per_submap`, `overlap` and `local_resolution` control frame-based
partitioning. `partition: spatial` also uses `max_extent`, `max_travel` and
`overlap_travel`; `spatial.yaml` is a template. The first frame of each unique
core defines its anchor. Overlap contributes observations while each trajectory
row is assigned to its first owning submap.

The local `cell_update_policy` is `first_sample` by default. `hit_priority`
keeps one update per scan/cell but preserves endpoint evidence when a free
sample lands in the same cell. This changes mapping evidence and must be kept
consistent when comparing runs. It is explicitly selected by the prepared b2 demo.

## Prebuilt submaps

```bash
bash tools/occupancy-joining run --input-type submaps \
  --manifest /absolute/submaps.yaml --output /absolute/results
```

```yaml
optimizer:
  model: matlab_inverse
  global_resolution: 0.1
  max_iterations: 70
submaps:
  - grid: local0.grid
    pose: [0.0, 0.0, 0.0]
  - grid: local1.grid
    pose: [3.0, 0.1, 0.02]
```

Grid files start with `width height resolution origin_x origin_y`, followed by
`width*height` rows of `log_odds_sum hit_count`, in row-major order. Counts must
be nonnegative. Vertex `(x,y)` is `origin + resolution*[x,y]`; this is a nodal
grid. Grid paths resolve relative to the
manifest. Settings order is defaults, manifest optimizer, explicitly supplied
YAML optimizer, then CLI overrides. Raw-ray reconstruction is unavailable if
only local grids are supplied.

## Optimization and display

`optimizer.global_resolution` and `submaps.local_resolution` define numerical
lattices. `display.resolution` independently controls initial/final-pose raw-ray
map reconstruction. `max_cells` bounds dense allocation. `max_iterations` is a
solve budget.

The default is `model: matlab_inverse`, Huber delta 1, inverse raw-value cutoff
0.5, count cutoff 0.001, N smoothing 1e-6 and the MATLAB cell-coordinate gradient
approximation. `inverse_gradient: metric` explicitly changes the chain factor.
The optional `globalized_inverse` model adds accepted-step control; the b2 preset
uses no continuation (`continuation_sigmas: [0]`). Numerical definitions and
the count/gradient conventions are summarized in [method](METHOD.md).

## Optional corrected TXT trajectories

The initializer accepts a timestamp-associated external graph CSV with header
`time_seconds,id,x,y,yaw`. Each pose is an absolute sensor-to-world SE(2) pose.
The original TXT `timestamps.txt` must have one seconds timestamp per scan.

```bash
bash tools/occupancy-joining initialize --config /absolute/sequence.yaml \
  --timestamps /absolute/timestamps.txt --graph-poses /absolute/final_graph.csv \
  --output /absolute/initialized
bash tools/occupancy-joining run --config /absolute/initialized/joining.yaml \
  --output /absolute/joined
```

The generated recipe resets row selection for the exported subset and disables
GT evaluation. Without `--graph-poses`, the initializer runs `slam_toolbox`;
`--pose-estimate final_graph` requests the final graph. The exported configuration can be used directly by the joining backend.
