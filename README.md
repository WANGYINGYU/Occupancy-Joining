# Grid-based Submap Joining: An Efficient Algorithm for Simultaneously Optimizing Global Occupancy Map and Local Submap Frames

Yingyu Wang, Liang Zhao, and Shoudong Huang
**IROS 2024** · [Paper](https://arxiv.org/abs/2501.12764) · [DOI](https://doi.org/10.1109/IROS58592.2024.10802536)

ROS 2 / C++17 implementation for jointly optimizing a global 2D occupancy map
and local submap frames using global-to-local projection and a pose-only Schur solve.

![Submap joining on the Intel Research Lab dataset](media/intel.gif)

**Intel Research Lab:** 13,631 real laser scans and 14 submaps. The animation
shows the initial alignment, recorded optimizer updates and the three output maps
in a fixed whole-map view. Initial poses use local scan tracking.

Independent evaluation against 2,984 manually matched relative-pose references:

| State | Translation RMSE [m] | Rotation RMSE [deg] |
| --- | ---: | ---: |
| Initial | 1.327 | 2.311 |
| Optimized | 0.586 | 2.329 |

![Joining three initially displaced synthetic submaps](media/room.gif)

**Synthetic alignment:** three internally exact submaps with imposed frame errors;
pose RMSE decreases from 0.632 m to below 0.001 m. Both animations use a fixed
view of the entire map throughout.

## Installation

Ubuntu 24.04; Python 3.11 or later. Choose [Docker](https://docs.docker.com/engine/install/ubuntu/)
or a native [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) installation.

```bash
git clone https://github.com/WANGYINGYU/Occupancy-Joining.git
cd Occupancy-Joining
```

### Docker

```bash
bash tools/docker.sh build
```

The image includes ROS 2, slam_toolbox, RViz and the C++/Python dependencies.
Builds use two compiler jobs; use `--build-arg BUILD_JOBS=1` to reduce this.

### Native

Install ROS 2 Jazzy, then run:

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

Source these environments in each new native terminal:

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Run a Demo

Run from the repository root with a new `--work` directory outside the checkout.
Inputs download automatically; Docker demos also build the image automatically.

```bash
python3 demo/run.py --demo room --work /absolute/path/to/room-demo
python3 demo/run.py --demo intel --work /absolute/path/to/intel-demo
```

For a native build, add `--native`. Open `<work>/results/index.html` for map
comparisons or `<work>/results/demo.gif` for the animation.

| Demo | Input | Submaps | Backend | Display grid |
| --- | --- | ---: | --- | --- |
| room | 90 synthetic scans; imposed submap frame errors; evaluation GT | 3 | `globalized_inverse` | 0.05 m |
| intel | 13,631 real laser scans; local tracking poses; manual evaluation relations | 14 | `globalized_inverse` | 0.05 m |

The [Intel input archive](https://github.com/WANGYINGYU/Occupancy-Joining/releases/tag/v0.1.4)
and [room input archive](https://github.com/WANGYINGYU/Occupancy-Joining/releases/tag/v0.1.2)
include the inputs, parameters and dataset notices. See
[dataset preparation and cached downloads](docs/DATASETS.md).

## Use Your Own Data

Copy a profile from `src/occupancy_joining/config/` and edit its input settings.
YAML paths resolve relative to the configuration file.

| Input | Profile | Required settings |
| --- | --- | --- |
| TXT | `txt.yaml` | `input.dataset`; scan angles and range limits |
| LaserScan / MultiEchoLaserScan bag | `rosbag_laserscan.yaml` | Bag path and topic names |
| PointCloud2 bag | `rosbag_pointcloud.yaml` | Topics, projection frame and height interval |
| Local grids | [Submap manifest](docs/CONFIGURATION.md#prebuilt-submaps) | Grid files and initial frame poses |

TXT datasets contain `Range.txt` (one scan per row, metres) and `Pose.txt`
(one sensor-to-world `x y yaw` per row, metres/radians). ROS1 bags are files;
ROS2 bags are directories. Copy `slam_toolbox.yaml` alongside a bag profile,
and set `bag.scan_topic`, `odom_topic`, `tf_topic` and `tf_static_topic`.

```bash
# Inspect bag topics
bash tools/docker.sh inspect /absolute/path/to/input.bag

# Run with your configuration
bash tools/docker.sh run --config /absolute/path/to/sequence.yaml

# Native
bash tools/occupancy-joining run --config /absolute/path/to/sequence.yaml \
  --output /absolute/path/to/results
```

Set `OCCUPANCY_JOINING_WORKSPACE` to choose the Docker output root. Additional
input folders can be mounted with `--mount-input /absolute/path` before `run`.
See [configuration](docs/CONFIGURATION.md) for odometry, submap
partitioning and optimizer settings, and [method](docs/METHOD.md) for the two backends.

## RViz

```bash
# Docker: Linux X11/XWayland, with DISPLAY and XAUTHORITY set
bash tools/docker.sh --gui view --result /absolute/path/to/results

# Native
bash tools/occupancy-joining view --result /absolute/path/to/results
```

Select a topic in the RViz **Map** display to compare:

| Topic | Output |
| --- | --- |
| `/occupancy_joining/initial/map` | Map at the initial poses |
| `/occupancy_joining/optimized/map` | Direct optimized occupancy field |
| `/occupancy_joining/reintegrated/map` | Map rebuilt at the optimized poses |

Maps use `nav_msgs/OccupancyGrid`: black occupied, white free, gray unknown.
Trajectories and submap frames use `nav_msgs/Path` under each state's
`/trajectory` and `/submap_frames`. Use fixed frame `map` and Reliable/Transient Local QoS.

`display.resolution` sets the initial/rebuilt raw-scan map resolution independently
of the optimizer; 0.05 or 0.1 m is typical. The optimized field uses its native grid.
For headless publication, add `--no-rviz --duration 5` and omit `--gui`.

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
