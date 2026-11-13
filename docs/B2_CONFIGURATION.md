# B2 configuration

The tested full-rate input contains 51,833 first-echo scans and frontend poses from Cartographer's final pose graph. The profile builds spatial submaps with a 30 m extent, 60 m travel limit, 6 m overlap and at most 1,500 scans. It uses 0.1 m optimization grids and an 8/4/2/1/0-cell continuation schedule.

From the repository root, with ROS 2 and the workspace sourced:

```bash
ros2 run occupancy_joining joining_pipeline.py \
  --config src/occupancy_joining/config/b2.yaml \
  --dataset /path/to/b2 --output /path/to/new-results
ros2 run occupancy_joining joining_viewer.py --result /path/to/new-results
```

The input directory contains `Range.txt` and `Pose.txt` (`x y yaw`, metres and radians). Compressed text ranges are supported through `input.ranges: Range.txt.gz` in a dataset-local configuration file. Relative file paths are resolved against that configuration file.

`display.resolution`, `display.max_cells` and `display.grid_format` control reconstruction separately from optimization. The b2 profile stores lossless `display.npz` grids at 0.05 m; the viewer publishes standard ROS 2 OccupancyGrid topics. The directly optimized map retains its native 0.1 m grid.

For a controlled initialization test, `submaps.anchor_poses` accepts one `x y yaw` row per generated submap. Keep the scan poses used to build each local map fixed and provide the modified submap-frame poses separately. Record the imposed errors with the dataset.
