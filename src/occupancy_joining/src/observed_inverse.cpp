#include "occupancy_joining/core.hpp"
#include <cmath>

namespace joining {
bool interpolate_observed(const Grid& grid, const Eigen::Vector2d& point,
                          Sample& sample, double minimum_count, bool normalized) {
  if (!interpolate(grid, point, sample)) return false;
  // Occupancy zero is a valid observation. Visibility comes only from raw counts.
  for (int k = 0; k < 4; ++k)
    if (!(grid.counts[sample.ids[k]] > minimum_count)) return false;
  if (!normalized) return true;
  double value[4];
  for (int k = 0; k < 4; ++k)
    value[k] = grid.values[sample.ids[k]] / grid.counts[sample.ids[k]];
  sample.value = 0;
  for (int k = 0; k < 4; ++k) sample.value += sample.weights[k] * value[k];
  const Eigen::Vector2d q = (point - grid.origin) / grid.resolution;
  const double u = q.x() - std::floor(q.x());
  const double v = q.y() - std::floor(q.y());
  // Exact derivative of the same bilinear field that supplies the residual.
  // The four vertices are observed, so no zero-filled unknown stencil is used.
  sample.gradient.x() = ((1-v)*(value[2]-value[0]) + v*(value[3]-value[1])) / grid.resolution;
  sample.gradient.y() = ((1-u)*(value[1]-value[0]) + u*(value[3]-value[2])) / grid.resolution;
  return true;
}

std::vector<Row> observed_rows(const std::vector<Grid>& locals,
                               const std::vector<Pose>& poses,
                               const Grid& global, const Options& options,
                               bool normalized) {
  std::vector<Row> rows;
  std::vector<Eigen::Matrix2d> rotations;
  for (const auto& pose : poses) rotations.push_back(rotation(pose.z()).transpose());
  for (int cell = 0; cell < global.size(); ++cell) {
    // Raw mode needs a valid count ratio. Normalized mode has no global divisor.
    if (!normalized && global.counts[cell] <= options.minimum_count) continue;
    const Eigen::Vector2d world = global.origin + global.resolution *
        Eigen::Vector2d(cell % global.width, cell / global.width);
    for (size_t i = 0; i < locals.size(); ++i) {
      const auto& R = rotations[i];
      const Eigen::Vector2d local = R * (world - poses[i].head<2>());
      Sample sample;
      if (!interpolate_observed(locals[i], local, sample, options.minimum_count, normalized)) continue;
      const double amplitude = normalized ?
          (options.model == "information_inverse" ? std::sqrt(sample.count) : sample.count) : 1.;
      const double coefficient = normalized ? amplitude : sample.count / global.counts[cell];
      const double observation = amplitude * sample.value;
      const double error = coefficient * global.values[cell] - observation;
      const double scale = std::sqrt(std::abs(error) > options.huber_delta ?
                                      options.huber_delta / std::abs(error) : 1.);
      Row row;
      row.cell = cell; row.submap = int(i); row.count = sample.count;
      row.h = scale * observation; row.weight = scale * coefficient;
      const Eigen::Vector2d gradient = scale * amplitude * sample.gradient;
      row.jacobian.head<2>() = R.transpose() * gradient;
      row.jacobian.z() = gradient.y() * local.x() - gradient.x() * local.y();
      rows.push_back(row);
    }
  }
  return rows;
}

Eigen::VectorXd observation_counts(const std::vector<Row>& rows, int cells) {
  Eigen::VectorXd counts = Eigen::VectorXd::Zero(cells);
  for (const auto& row : rows) counts[row.cell] += row.count;
  return counts;
}
}  // namespace joining
