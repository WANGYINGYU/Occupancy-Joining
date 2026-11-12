# Demo inputs

`demo/run.py` downloads the versioned input archive into `--work` and runs
Docker or the sourced native build. Each archive includes `DATA_NOTICE.txt`.

## Room

60 scans, 361 beams and three temporal submaps. Inputs contain perturbed
sensor poses and separate evaluation GT. The optimizer grid is 0.15 m;
the raw-scan display grid is 0.05 m.

```bash
python3 demo/generate_room.py --output /absolute/new/room-inputs
```

## Museum b2

[Cartographer public data](https://google-cartographer-ros.readthedocs.io/en/latest/data.html):
`b2-2015-07-07-11-27-05.bag`, approximately 361 MB. The runner downloads
the original bag directly from the provider under its dataset terms.

The preparation archive contains 51,833 initial sensor poses, exact scan-header
timestamps, 35 overlapping submap ranges/anchors, 0.1 m lattice geometry and
the `globalized_inverse` preset with a 70-solve budget.

Initial poses come from a range-only Cartographer final optimization using
instantaneous first-echo scans and identity sensor/tracking extrinsics.
All 51,833 horizontal MultiEchoLaserScan frames enter local mapping;
empty echoes are missing. Mapping uses one count per scan/cell and hit-priority
evidence. Initial/reintegrated displays rebuild the full sequence at 0.1 m.

## Cached downloads

```bash
python3 demo/run.py --demo b2 --work /absolute/new/b2-demo \
  --archive /absolute/Occupancy_Joining_B2_Inputs_v0.1.0.tar.gz \
  --bag /absolute/b2-2015-07-07-11-27-05.bag
```

Add `--native` for a native build. `--jobs 1` reduces preparation concurrency;
`--timeout` sets the process time limit. Docker's image storage is managed
by the Docker daemon. Extra input folders use `--mount-input /absolute/path`.

## Animation

Demos write `results/demo.gif`. To render an existing result:

```bash
python3 demo/animate.py --result /absolute/b2-demo/results \
  --output /absolute/b2-demo/results/demo.gif --roi 50 150 -20 30
```

The GIF uses recorded submap updates with fixed map extents and a fixed detail
window, followed by the three exported occupancy grids. `--frames` controls
the number of recorded updates shown.
