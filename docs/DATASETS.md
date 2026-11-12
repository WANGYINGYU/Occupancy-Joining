# Demo datasets

Run from the repository root. Inputs download into a new directory outside the checkout:

```bash
python3 demo/run.py --demo intel --work /absolute/new/intel-demo
python3 demo/run.py --demo room --work /absolute/new/room-demo
```

Docker builds automatically. Add `--native` to use the sourced native build.
Open `results/index.html`, `results/demo.gif`, or view the result directory in RViz.
Both presets use 0.1 m local/global grids, 0.05 m raw-ray display grids and the
existing `globalized_inverse` continuation `[8,4,2,1,0]` with a 70-solve limit.

## Intel Research Lab

13,631 scans with 180 laser beams from the real Intel Research Lab (Seattle)
dataset, provided by Dirk Haehnel through [Radish](https://radish.sourceforge.net/)
and the [Freiburg SLAM Benchmarking archive](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php).
The archive supplies `Range.txt`, `Pose.txt`, original timestamps,
`demo.yaml`, `preparation.json`, manual relative-pose references and `DATA_NOTICE.txt`.

Ranges retain the original recorded scan order. Angles are `-90 + i` degrees
for `i=0..179`, with 0.1–30 m range filtering and zero laser offset.
Initial poses are range-only Cartographer local tracking poses, before global
pose-graph correction. The legacy log has nonmonotonic timestamps; frontend
replay uses a separate 0.2 s logical clock while retaining original timestamps
for evaluation association. Local tracking poses match exactly with global
graph computation enabled or disabled.

The original project partition uses 1,000 scans per submap without overlap,
giving 14 submaps. All scans participate in mapping. The 2,984 manually matched
relative-pose references are evaluated independently, with exact original
timestamp association. Results are in `relative_pose_metrics.json`; the animation
shows translation and rotation RMSE at recorded iterations.

Raw data is CC BY 1.0; manual relations are CC BY 3.0. Attribution and source
links are included in `DATA_NOTICE.txt`. The MIT license covers project code.

## Synthetic room

90 ray-cast scans, 720 beams and three overlapping submaps of an asymmetric room.
Each submap contains the same 30-pose loop with exact internal geometry.
Initial frame biases are `[0,0,0]`, `[0.65,-0.45,0.06]` and
`[-0.55,0.5,-0.05]` in metres/radians. The first frame fixes the gauge;
`GT.txt` supplies the independent trajectory evaluation in `metrics.json`.

Regenerate the TXT input:

```bash
python3 demo/generate_room.py --output /absolute/new/room-inputs
```

## Cached downloads

```bash
python3 demo/run.py --demo intel --work /absolute/new/intel-demo \
  --archive /absolute/Occupancy_Joining_Intel_v0.1.4.tar.gz
```

`--timeout` sets the demo time limit. Docker image storage is managed by the
daemon. Additional input folders use `--mount-input /absolute/path`.

## Animation

```bash
python3 demo/animate.py --result /absolute/intel-demo/results \
  --output /absolute/intel-demo/results/demo.gif
```

The GIF uses a single whole-map viewport: recorded submap-frame updates,
then initial, directly optimized and pose-rebuilt grayscale occupancy maps.
All frames share world bounds and scale. `--frames` controls how many actual
updates are shown. Intel metrics use manual relative-pose references; room
metrics use the fixed-gauge synthetic trajectory GT.
