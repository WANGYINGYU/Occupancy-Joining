# Third-party notices

The bag/TF interfaces (`scan_io.py`, `geometry.py`, `slam_frontend.py`,
`inspect_bag.py`), grayscale RViz Map extension, and frontend/Fast DDS
configuration are adapted from WANGYINGYU/Occupancy-SLAM, MIT License,
Copyright (c) 2023 Yingyu WANG. The MIT notice is included in `LICENSE`.

ROS 2, slam_toolbox, RViz, Ceres and other external dependencies retain their
respective licenses. They are installed by the dependency instructions or
Dockerfile; the project license does not relicense these dependencies.

The room demo is generated independently. Intel Research Lab laser data was
provided by Dirk Haehnel through Radish and the University of Freiburg
SLAM Benchmarking archive, under Creative Commons Attribution 1.0.
Its manual relative-pose relations are Creative Commons Attribution 3.0.
The demo input archive includes attribution, source links and preparation details
in `DATA_NOTICE.txt`; see `docs/DATASETS.md`.
