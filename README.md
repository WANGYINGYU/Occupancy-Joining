# Grid-based Submap Joining: An Efficient Algorithm for Simultaneously Optimizing Global Occupancy Map and Local Submap Frames

Yingyu Wang, Liang Zhao, and Shoudong Huang

**IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2024.**
[Paper](https://arxiv.org/abs/2501.12764) · [DOI](https://doi.org/10.1109/IROS58592.2024.10802536)

ROS 2 / C++17 implementation of 2D occupancy submap joining. The backend
optimizes local submap frames and a global occupancy map using global-to-local
projection and a pose-only Schur solve. Robot poses within each submap remain
fixed. The MATLAB-style robust inverse model is the default; N derivatives are
omitted and N is recomputed after pose updates.

Inputs can be prepared local grids, 2D text scans and poses, MATLAB data, or
ROS1/ROS2 bags. ROS bag preparation uses `slam_toolbox`; RViz displays standard
occupancy maps and trajectories. This package implements **2D joining**;
PointCloud2 input is projected into 2D. See [method and implementation scope](docs/METHOD.md).

## 1. Prerequisites

- Ubuntu 24.04 and Python 3.11 or later.
- **Docker:** install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/)
  and configure [user access](https://docs.docker.com/engine/install/linux-postinstall/).
- **Native:** install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

## 2. Build

Clone the repository, then choose Docker or a native installation.

```bash
sudo apt update
sudo apt install -y git python3
git clone https://github.com/WANGYINGYU/Occupancy-Joining.git
cd Occupancy-Joining
```

### 2.1 Docker

```bash
docker info
```

The demo commands build the image automatically. For your own data, build it with:

```bash
bash tools/docker.sh build
```

The image includes ROS 2, `slam_toolbox`, RViz and all build/Python dependencies.
No host ROS installation or GPU is required for headless runs. The image builds
with two compiler jobs by default; reduce this with
`bash tools/docker.sh build --build-arg BUILD_JOBS=1` on memory-constrained machines.

### 2.2 Native (without Docker)

After installing ROS 2 Jazzy, install the dependencies and build:

```bash
sudo apt update
sudo apt install -y build-essential cmake python3-colcon-common-extensions \
  python3-venv python3-pip python3-numpy python3-scipy python3-yaml \
  python3-h5py python3-pil libeigen3-dev libyaml-cpp-dev \
  libboost-serialization-dev libceres-dev ros-jazzy-ament-cmake \
  ros-jazzy-slam-toolbox ros-jazzy-rviz2 ros-jazzy-rmw-fastrtps-cpp
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
source /opt/ros/jazzy/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
bash tools/build.sh
source install/setup.bash
```

In each new terminal, source the environments from the repository root:

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## 3. Run the Demos

Run from the repository root. Use a **new** directory for `--work`, outside the
checkout. Inputs, results, temporary files and logs remain in that directory.

### 3.1 Synthetic room

```bash
# Docker
python3 demo/run.py --demo room --work /absolute/path/to/room-demo

# Native
python3 demo/run.py --native --demo room --work /absolute/path/to/room-demo-native
```

This independent 60-scan example is a functional demonstration, not a paper benchmark.
Its comparison page reports synthetic GT metrics. Floating-point changes to grid
origins/support can change its optimization path; it is not an accuracy guarantee.

### 3.2 Museum b2

```bash
# Docker
python3 demo/run.py --demo b2 --work /absolute/path/to/b2-demo

# Native
python3 demo/run.py --native --demo b2 --work /absolute/path/to/b2-demo-native
```

The complete 51,833-scan demo uses a supplied range-only Cartographer
initialization and 35 fixed submaps. It retains the explicit
`globalized_inverse` preset, distinct from the default MATLAB-style solver.
It does not rerun Cartographer, use ground truth, or reproduce the complete
paper local-map optimization pipeline. Complete alignment is not guaranteed.

Inputs are downloaded automatically. The small preparation archives are
[release assets](https://github.com/WANGYINGYU/Occupancy-Joining/releases/tag/v0.1.0);
the original b2 bag is downloaded directly from the
[official provider](https://google-cartographer-ros.readthedocs.io/en/latest/data.html).
For existing downloads, add `--archive /path/to/demo.tar.gz` and, for b2,
`--bag /path/to/b2-2015-07-07-11-27-05.bag`.

Open `<work>/results/index.html` for the initial, direct optimized and
pose-rebuilt maps. The full-resolution images and ROS map files remain in
`<work>/results/maps/`. See [demo input definitions](docs/DATASETS.md).

## 4. Prepare Your Own Data

### 4.1 Text scans and poses

```text
sequence/
├── Range.txt
├── Pose.txt
└── sequence.yaml
```

`Range.txt` contains one scan per row, in metres, with beams ordered by
increasing angle. `Pose.txt` contains one absolute sensor-to-world pose per row:

```text
x y yaw
```

Translations are in metres and yaw is in radians. Scan and pose counts must
match. Copy `src/occupancy_joining/config/txt.yaml` to `sequence/sequence.yaml`,
set `input.dataset: .`, and adjust the beam angles and valid range for your sensor.
YAML paths resolve relative to the YAML file.

```bash
# Docker
bash tools/docker.sh run --config /absolute/path/to/sequence/sequence.yaml

# Native
bash tools/occupancy-joining run \
  --config /absolute/path/to/sequence/sequence.yaml \
  --output /absolute/path/to/results
```

Docker run directories are printed on startup; set
`OCCUPANCY_JOINING_WORKSPACE` to choose their output root. Keep results separate
from inputs. Use [configuration](docs/CONFIGURATION.md) for incremental
odometry, MATLAB files, spatial submap partitioning, prebuilt grids and
optional external trajectory initialization.

### 4.2 ROS1 and ROS2 bags

```bash
bash tools/docker.sh inspect /absolute/path/to/input.bag
bash tools/docker.sh run \
  --config /absolute/path/to/rosbag_laserscan.yaml \
  --dataset /absolute/path/to/input.bag
```

Copy `src/occupancy_joining/config/rosbag_laserscan.yaml` and its sibling
`slam_toolbox.yaml` into your configuration directory. Set `bag.scan_topic`,
`odom_topic`, `tf_topic` and `tf_static_topic` to the names in your bag.
LaserScan and MultiEchoLaserScan (first echo) are supported. For PointCloud2,
start from `rosbag_pointcloud.yaml` and set the projection frame, height interval
and angular bins. ROS2 bag inputs are directories. No ROS1 installation or
bridge is required.

For native execution, replace `bash tools/docker.sh` with
`bash tools/occupancy-joining` and provide an output directory with `--output`.
Additional YAML-referenced input folders can be mounted with
`--mount-input /absolute/path` before the Docker command.

## 5. RViz and Outputs

Open the demo or custom result directory:

```bash
# Docker: Linux X11/XWayland with DISPLAY and XAUTHORITY
bash tools/docker.sh --gui view --result /absolute/path/to/results

# Native
bash tools/occupancy-joining view --result /absolute/path/to/results
```

In the RViz **Map** display, change **Topic** to compare:

| Topic | Map definition |
| --- | --- |
| `/occupancy_joining/initial/map` | Input submaps at their initial frame poses. |
| `/occupancy_joining/optimized/map` | Direct optimized occupancy field. |
| `/occupancy_joining/reintegrated/map` | Reintegrated input submaps at optimized poses; raw-ray display reconstruction when scans are available. |

Maps are `nav_msgs/OccupancyGrid`, with black occupied, white free and gray
unknown cells. Trajectories and submap frames are `nav_msgs/Path` on each
state's `/trajectory` and `/submap_frames` topics. Subscriptions use
Reliable/Transient Local QoS, with fixed frame `map`.

`display.resolution` controls raw-scan display rebuilding independently of the
optimizer grid; use 0.05 or 0.1 m. The direct optimized field retains its native
resolution. For prebuilt grids without scans, displays preserve native resolution.
On a headless machine, add `--no-rviz --duration 5` to `view` and omit `--gui`.

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

[MIT](LICENSE) for project code. Demo input terms are separate; see
`DATA_NOTICE.txt` in each downloaded preparation archive and
[third-party notices](THIRD_PARTY_NOTICES.md). The code license grants no
additional rights to the original b2 sensor data.
