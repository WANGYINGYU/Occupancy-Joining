# Native ROS 2 installation

Ubuntu 24.04 with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Run from the repository root.

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

## Run

```bash
python3 demo/run.py --demo b2 --native --work ../occupancy-joining-native-demo
bash tools/occupancy-joining view --result ../occupancy-joining-native-demo/results
```

For a new terminal:

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

For configured inputs:

```bash
bash tools/occupancy-joining run --config /path/to/config.yaml \
  --output /path/to/results
```
