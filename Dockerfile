FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends build-essential cmake git libeigen3-dev libyaml-cpp-dev libboost-serialization-dev libceres-dev python3-colcon-common-extensions python3-venv python3-pip python3-numpy python3-scipy python3-yaml python3-h5py python3-pil ros-jazzy-slam-toolbox ros-jazzy-rviz2 ros-jazzy-rmw-fastrtps-cpp libgl1-mesa-dri libglx-mesa0 xauth && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/occupancy-joining
COPY requirements.txt .
RUN /usr/bin/python3 -m venv --system-site-packages /opt/joining-venv && /opt/joining-venv/bin/pip install --no-cache-dir -r requirements.txt
ENV PATH="/opt/joining-venv/bin:${PATH}" PYTHONDONTWRITEBYTECODE=1
COPY src src
COPY tools tools
COPY docker docker
COPY demo demo
ARG BUILD_JOBS=2
RUN source /opt/ros/jazzy/setup.bash && CMAKE_BUILD_PARALLEL_LEVEL=${BUILD_JOBS} MAKEFLAGS="-j${BUILD_JOBS}" colcon build --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
ENTRYPOINT ["/bin/bash", "/opt/occupancy-joining/docker/entrypoint.sh"]
CMD ["--help"]
