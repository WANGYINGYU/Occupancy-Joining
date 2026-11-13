# Grid-based Submap Joining: An Efficient Algorithm for Simultaneously Optimizing Global Occupancy Map and Local Submap Frames

Yingyu Wang, Liang Zhao, and Shoudong Huang
**IROS 2024** · [Paper](https://arxiv.org/abs/2501.12764) · [DOI](https://doi.org/10.1109/IROS58592.2024.10802536)

ROS 2 / C++17 implementation for jointly optimizing a global 2D occupancy map
and local submap frames using global-to-local projection and a pose-only Schur solve.

![Submap joining on b2 laser scans](media/b2.gif)

*b2: 51,833 laser scans, 44 submaps, and nominal 1 m / 2° initial submap-frame perturbations.*

## Installation

Ubuntu 24.04 with Docker or ROS 2 Jazzy.

```bash
git clone https://github.com/WANGYINGYU/Occupancy-Joining.git
cd Occupancy-Joining
```

### Docker

Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/), then build:

```bash
bash tools/docker.sh build
```

The image includes ROS 2, slam_toolbox, RViz and all build/runtime dependencies.

### Native

Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html), then run:

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-colcon-common-extensions \
  python3-venv python3-pip python3-numpy python3-scipy python3-yaml \
  python3-pil libeigen3-dev libyaml-cpp-dev \
  libboost-serialization-dev ros-jazzy-ament-cmake \
  ros-jazzy-slam-toolbox ros-jazzy-rviz2 ros-jazzy-rmw-fastrtps-cpp
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2
bash tools/build.sh
source install/setup.bash
```

In each new native terminal:

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Demo

Run from the repository root with a new work directory outside the checkout.
Inputs download automatically.

```bash
python3 demo/run.py --demo b2 --work /absolute/path/to/b2-demo
```

Add `--native` to use a native build. Select a dataset with `--demo`:

| Dataset | Scans | Submaps |
| --- | ---: | ---: |
| `b2` | 51,833 | 44 |
| `room` | 90 | 3 |
| `intel` | 13,631 | 14 |

Open `<work>/results/index.html` for the output maps or
`<work>/results/demo.gif` for the animation.
[Dataset inputs and cached downloads](docs/DATASETS.md).

## Use Your Own Data

Copy a profile from `src/occupancy_joining/config/` and edit its input settings.
YAML paths resolve relative to the configuration file.

| Input | Profile | Settings |
| --- | --- | --- |
| TXT | `txt.yaml` | Dataset path, scan angles and range limits |
| LaserScan / MultiEchoLaserScan bag | `rosbag_laserscan.yaml` | Bag path and input topics |
| PointCloud2 bag | `rosbag_pointcloud.yaml` | Topics, projection frame and height interval |
| Local grids | [Submap manifest](docs/CONFIGURATION.md#prebuilt-submaps) | Grid files and initial frame poses |

TXT input uses `Range.txt` (one scan per row, metres) and `Pose.txt`
(one sensor-to-world `x y yaw` per row, metres/radians).
For bags, copy `slam_toolbox.yaml` alongside the selected profile and set
`bag.scan_topic`, `bag.odom_topic`, `bag.tf_topic` and `bag.tf_static_topic`.
ROS1 bags are files; ROS2 bags are directories.

```bash
# Inspect bag topics
bash tools/docker.sh inspect /absolute/path/to/input.bag

# Docker
bash tools/docker.sh run --config /absolute/path/to/sequence.yaml

# Native
bash tools/occupancy-joining run --config /absolute/path/to/sequence.yaml \
  --output /absolute/path/to/results
```

Set `OCCUPANCY_JOINING_WORKSPACE` to choose the Docker output root.
Use `--mount-input /absolute/path` before `run` for additional input directories.

[Configuration](docs/CONFIGURATION.md) · [b2 profile](docs/B2_CONFIGURATION.md) · [Method](docs/METHOD.md)

## RViz

```bash
# Docker: Linux X11/XWayland, with DISPLAY and XAUTHORITY set
bash tools/docker.sh --gui view --result /absolute/path/to/results

# Native
bash tools/occupancy-joining view --result /absolute/path/to/results
```

Select a topic in the RViz **Map** display:

| Topic | Map |
| --- | --- |
| `/occupancy_joining/initial/map` | Map at initial poses |
| `/occupancy_joining/optimized/map` | Direct optimized occupancy field |
| `/occupancy_joining/reintegrated/map` | Rebuilt with optimized poses |

Maps use `nav_msgs/OccupancyGrid` with black occupied, white free and gray unknown.
Trajectories and submap frames use `nav_msgs/Path` under each state's
`/trajectory` and `/submap_frames`. Set fixed frame to `map` and QoS to Reliable/Transient Local.

`display.resolution` controls the initial/rebuilt map resolution independently
of the optimization grid; use 0.05 or 0.1 m. The optimized field retains its native resolution.

## Citation

```bibtex
@inproceedings{wang2024grid,
  title={Grid-based Submap Joining: An Efficient Algorithm for Simultaneously Optimizing Global Occupancy Map and Local Submap Frames},
  author={Wang, Yingyu and Zhao, Liang and Huang, Shoudong},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  year={2024},
  pages={10121--10128},
  doi={10.1109/IROS58592.2024.10802536}
}
```

## License

[MIT](LICENSE). Dependency and dataset terms are listed in
[third-party notices](THIRD_PARTY_NOTICES.md) and each demo archive's `DATA_NOTICE.txt`.
