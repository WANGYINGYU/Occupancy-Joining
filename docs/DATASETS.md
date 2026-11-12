# Demo inputs

`demo/run.py` downloads the versioned input archive into `--work` and runs
Docker or the sourced native build. Each archive includes `DATA_NOTICE.txt`.

## Room

90 ray-cast scans, 720 beams and three overlapping submaps of an asymmetric
room. Each submap contains the same 30-pose loop with exact internal geometry.
The three initial world-frame biases are `[0,0,0]`, `[0.65,-0.45,0.06]` and
`[-0.55,0.5,-0.05]` in metres/radians. The first submap fixes the gauge;
`GT.txt` is used only to evaluate the sensor trajectory.

This controlled alignment example uses `globalized_inverse`, continuation
`[8,4,2,1,0]`, a 70-solve limit, 0.1 m local/global grids and 0.05 m raw-ray
display grids. The animation follows the actual committed frame poses.

Regenerate the versioned TXT inputs with:

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
  --output /absolute/b2-demo/results/demo.gif
```

The GIF shows recorded submap updates in a fixed whole-map viewport, followed
by the initial, directly optimized and pose-rebuilt grayscale occupancy maps.
All frames use the same world bounds and scale. When evaluation GT is present,
it displays pose RMSE in the fixed input gauge. `--frames` controls the number
of recorded updates shown.
