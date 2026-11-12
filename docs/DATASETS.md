# Demo inputs

Demo archives are separate from the source repository. `demo/run.py` downloads
and checks the versioned archive, then runs Docker or the sourced native build.
Use a new output directory. Each archive contains `DATA_NOTICE.txt`.

## Room

The independent room contains 60 scans with 361 beams, perturbed absolute
sensor poses and separate synthetic GT. Three temporal submaps are joined.
The default MATLAB-style inverse model is used. The display resolution is
0.05 m, independent of the 0.15 m local/global optimizer grids. No paper result
is asserted for this small example. To regenerate its uncompressed inputs:

```bash
python3 demo/generate_room.py --output /absolute/new/room-inputs
```

## Museum b2

Source: [Cartographer public data](https://google-cartographer-ros.readthedocs.io/en/latest/data.html),
`b2-2015-07-07-11-27-05.bag`. The original bag is approximately 361 MB and is
downloaded directly from the official provider; it is not mirrored in this
repository or its preparation archive. The upstream dataset terms still apply.

The preparation archive contains:

- 51,833 absolute initial sensor poses from a recorded full-sequence,
  range-only Cartographer C++-library final optimization;
- exact raw scan-header timestamps;
- 35 fixed overlapping submap ranges, frame anchors and nodal lattice geometry;
- the explicit numerical preset used for this prepared-input example.

The initializer used no supplied odometry, IMU or GT. It treated first-echo
scans as instantaneous, with identity sensor/tracking extrinsics. This is not
the official backpack IMU/deskewing sensor configuration. The initializer is
supplied, not rerun by the demo; optimized joining poses or maps are not inputs.

All 51,833 horizontal MultiEchoLaserScan observations enter local mapping.
The reader verifies every scan-header timestamp against the prepared input,
uses first echoes, and treats empty echoes as missing. The local-map builder
preserves the prepared 0.1 m lattice phase and per-scan hit-priority evidence.
N is the actual observation count; it is not synthesized from a smoothed field.

The b2 preset is `globalized_inverse`, with the recorded 70-solve budget and
no continuation. Both direct optimized and independently raw-ray rebuilt maps
are exported. Initial/reintegrated displays use the complete sequence at 0.1 m.
This demonstrates the prepared joining pipeline and its outputs, not the full
paper frontend/local optimization procedure or an absolute-accuracy benchmark.

For cached downloads:

```bash
python3 demo/run.py --demo b2 --work /absolute/new/b2-demo \
  --archive /absolute/Occupancy_Joining_B2_Inputs_v0.1.0.tar.gz \
  --bag /absolute/b2-2015-07-07-11-27-05.bag
```

Add `--native` after building/sourcing the native installation. `--jobs 1`
reduces preparation concurrency. `--timeout` limits the optimization/preparation
process; timeout is a failure, with logs retained. Docker image storage uses
the Docker daemon's configured storage directory, separately from `--work`.

For Docker, repeat `--mount-input /absolute/input-parent` to make an additional
input directory available read-only at its original path. The selected `--work`
directory remains the writable location for all generated files.
