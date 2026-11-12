# Method

The state consists of a global occupancy field and SE(2) submap frames.
Global grid samples project into each local submap. A diagonal map block
allows pose-only Schur elimination followed by map back-substitution.
The first submap frame fixes the gauge; relative poses inside submaps stay fixed.

## Backends

- `matlab_inverse` (default): MATLAB inverse residual, Huber weighting,
  raw-value filtering and N refresh/smoothing. The default gradient uses the
  MATLAB cell-coordinate approximation; `inverse_gradient: metric` includes
  the inverse local-resolution chain factor.
- `globalized_inverse`: count-supported local interpolation, conditional map
  elimination and accepted-step control. The b2 demo uses this backend with
  `continuation_sigmas: [0]`.

N and robust weights remain fixed within each linearization. N is recomputed
after pose updates; its pose derivative is omitted.

## Maps

Local grids carry log-odds sums and raw observation counts. Count support
determines visibility; an observed zero value differs from an unknown cell.
The backend exports initial integration, the optimized occupancy field, and
reintegration at the final submap poses. With raw scans, initial/reintegrated
display maps are rebuilt from those scans at `display.resolution`.
