# Method and implementation scope

The reference is Wang, Zhao and Huang, *Grid-based Submap Joining: An Efficient
Algorithm for Simultaneously Optimizing Global Occupancy Map and Local Submap
Frames*, IROS 2024, [DOI](https://doi.org/10.1109/IROS58592.2024.10802536).

The state contains a global occupancy field and SE(2) local-submap frames. Each
global sample is projected into local coordinates. The map block of the frozen
linear system is diagonal, allowing a pose-only Schur solve and map
back-substitution. The first submap frame fixes the gauge. Relative robot poses
inside local submaps are not optimization variables in this backend.

## Default robust inverse implementation

`matlab_inverse` follows the supplied MATLAB inverse residual, Huber weighting,
raw-value filtering and N refresh/smoothing, with the paper's global-to-local
projection and pose-only elimination. It preserves the original cell-coordinate
gradient approximation by default. `inverse_gradient: metric` changes the
explicit inverse-local-resolution chain factor and is a separate setting.

N and robust weights remain fixed during each linearization. N is recomputed
after pose updates. **No derivative of N with respect to poses is added.**
Map elimination equivalence and independence statements refer to that frozen
system; changing weights/support can change the nonlinear trajectory.

`paper` is the unrobust equation-(4) reference. `matlab` is a forward
local-to-global/full-joint reference and is not the pose-only paper algorithm.
The count-supported inverse variants are optional alternative observation
models, not literal MATLAB parity or default replacements.

## Optional accepted-step model

`globalized_inverse` uses the existing accepted-step inverse extension with
conditional map elimination, fixed-domain support checks and trial N refresh.
A continuation schedule is optional; the prepared b2 demo explicitly sets
`continuation_sigmas: [0]`. This model is distinguished from the default and
from an unmodified paper reproduction. It is not claimed to improve every input.

## Inputs and outputs

Prepared local-grid inputs supply both occupancy evidence and observation
counts. Unknown cells, observed zero values and smoothed counts have different
meanings. Smoothing is not a new physical observation. Local-map preparation,
initial trajectory quality and sensor handling can materially affect joining.

Three states are exported: initial integration, the direct optimized field,
and reintegration with final frame poses. With raw scans, initial/reintegrated
display maps are independently rebuilt from those scans; the optimized field
is displayed at its true numerical resolution. Comparing these outputs separates
field changes from pose-based reconstruction, without treating either as GT.

The included room is a synthetic functional example. The b2 example uses a
prepared Cartographer initialization and fixed raw-ray local maps; it does not
run the paper's Occupancy-SLAM local-map optimizer. It has no independent GT,
and complete overlap or paper benchmark accuracy is not certified. The source
release does not claim online real-time operation, all-scene accuracy,
full unmodified-MATLAB numerical identity or a 3D joining algorithm.
