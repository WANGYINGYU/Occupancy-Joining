# Demo datasets

Run from the repository root. Inputs download into a new directory outside the checkout:

```bash
python3 demo/run.py --demo b2 --work /absolute/new/b2-demo
python3 demo/run.py --demo room --work /absolute/new/room-demo
```

Docker builds automatically. Add `--native` to use the sourced native build.
Open `results/index.html`, `results/demo.gif`, or view the result directory in RViz.
All three presets use 0.1 m local/global grids, 0.05 m raw-ray display grids and the
existing `globalized_inverse` continuation `[8,4,2,1,0]` with a 70-solve limit.

## b2

51,833 horizontal scans with 1,079 beams from the Google Cartographer backpack
sequence `b2-2015-07-07-11-27-05.bag`. The demo downloads the 361 MB bag from
[the original provider](https://google-cartographer-ros.readthedocs.io/en/latest/data.html).
The release archive contains prepared poses, timestamps, parameters and evaluation
references. Scan ranges are extracted locally from the first echo, with exact
scan timestamp association and lossless TXT conversion.

`Pose.txt` is a range-only Cartographer final pose-graph estimate used to build
local geometry. `InitialSubmapPoses.txt` adds a fixed controlled error field at
the occupancy centroid of each submap, keeping the first frame fixed. Its nominal
continuous-field RMS is 1 m / 2°; the 44 sampled centroids have RMS 0.986 m / 1.856°.
These are estimated poses with imposed frame errors, not ground truth or raw odometry.

Spatial partitioning uses at most 1,500 scans, 30 m trajectory extent, 60 m travel
and 6 m overlap, producing 44 submaps. The [b2 configuration](B2_CONFIGURATION.md)
uses 0.1 m local/global grids and 0.05 m raw-ray initial/rebuilt maps.

`repeat_metrics.json` reports direction-weighted point-to-plane and point-to-point
consistency on 28 scan pairs (56 directed sets). The archive fixes 11,731 endpoint
correspondences and target normals, including 9,805 central-hall endpoints;
references were estimated by registration of repeated observations. Evaluation
keeps these identities fixed for every output. The 33 rejected pair candidates
are excluded from these conditional consistency statistics.

To use the frontend estimate without controlled frame perturbations, remove
`submaps.anchor_poses` from the generated `prepared/demo.yaml` and run that
configuration into a new output directory.

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
python3 demo/run.py --demo b2 --work /absolute/new/b2-demo \
  --archive /absolute/Occupancy_Joining_B2_Inputs_v0.2.0.tar.gz \
  --bag /absolute/b2-2015-07-07-11-27-05.bag
```

`--timeout` sets the demo time limit. Docker image storage is managed by the
daemon. Additional input folders use `--mount-input /absolute/path`.

## Animation

```bash
python3 demo/animate.py --result /absolute/b2-demo/results \
  --output /absolute/b2-demo/results/demo.gif --style occupancy --frames 42 \
  --title "b2 real scans | controlled initial frame errors: nominal 1 m / 2 deg"
```

The GIF uses a single whole-map viewport: recorded submap-frame updates,
then initial, directly optimized and pose-rebuilt grayscale occupancy maps.
All frames share world bounds and scale. `--frames` controls how many actual
updates are shown. The b2 occupancy style projects observed local cells in black/white/gray.
Intel metrics use manual relative-pose references; room
metrics use the fixed-gauge synthetic trajectory GT.
