# Grid-based Submap Joining: An Efficient Algorithm for Simultaneously Optimizing Global Occupancy Map and Local Submap Frames

Yingyu Wang, Liang Zhao, and Shoudong Huang
**IROS 2024** · [Paper](https://arxiv.org/abs/2501.12764) · [DOI](https://doi.org/10.1109/IROS58592.2024.10802536)

ROS 2 / C++17 implementation for jointly optimizing a global 2D occupancy map
and local submap frames using global-to-local projection and a pose-only Schur solve.

![Submap joining on b2 laser scans](media/b2.gif)

*b2 with controlled initial submap frames.*

## Installation

Ubuntu 24.04 and [Docker Engine](https://docs.docker.com/engine/install/ubuntu/).

```bash
git clone https://github.com/WANGYINGYU/Occupancy-Joining.git
cd Occupancy-Joining
bash tools/docker.sh build
```

[Native ROS 2 installation](docs/INSTALLATION.md).

## Run

```bash
python3 demo/run.py --demo b2 --work ../occupancy-joining-demo
```

For your own data, copy the [configuration profiles](src/occupancy_joining/config/)
and set `input.dataset`, input topics and sensor parameters.
[Configuration reference](docs/CONFIGURATION.md).

```bash
bash tools/docker.sh --mount-input /path/to/data run --config /path/to/config.yaml
```

## Visualization

```bash
bash tools/docker.sh --gui view --result ../occupancy-joining-demo/results
```

Select a map topic in RViz:

| Topic | Map |
| --- | --- |
| `/occupancy_joining/initial/map` | Initial map |
| `/occupancy_joining/optimized/map` | Optimized occupancy field |
| `/occupancy_joining/reintegrated/map` | Rebuilt with optimized poses |

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
