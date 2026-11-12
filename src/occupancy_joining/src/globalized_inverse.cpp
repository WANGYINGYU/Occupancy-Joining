#include "occupancy_joining/core.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace joining {
void inverse_trace(const Options&, int, const Grid&, const std::vector<Pose>&,
                   const std::vector<Row>&);
namespace {
double huber(double error, double delta) {
  const double a = std::abs(error);
  return a <= delta ? .5 * error * error : delta * (a - .5 * delta);
}
double robust_cost(const Grid& field, const std::vector<Row>& rows, double delta) {
  double cost = 0;
  for (const auto& row : rows)
    cost += huber(row.count * (field.values[row.cell] - row.h / row.weight), delta);
  return cost;
}
Eigen::VectorXd convolve(const Eigen::VectorXd& input, int width, int height,
                         const std::vector<double>& kernel) {
  const int radius = int(kernel.size()) / 2;
  Eigen::VectorXd horizontal = Eigen::VectorXd::Zero(input.size());
  Eigen::VectorXd output = horizontal;
  for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x)
    for (int k = -radius; k <= radius; ++k) if (x+k >= 0 && x+k < width)
      horizontal[y*width+x] += kernel[k+radius] * input[y*width+x+k];
  for (int y = 0; y < height; ++y) for (int x = 0; x < width; ++x)
    for (int k = -radius; k <= radius; ++k) if (y+k >= 0 && y+k < height)
      output[y*width+x] += kernel[k+radius] * horizontal[(y+k)*width+x];
  return output;
}
}  // namespace

Grid smooth_observed_field(const Grid& raw, double sigma, double minimum_count) {
  if (!std::isfinite(sigma) || sigma < 0 || sigma > 128)
    throw std::runtime_error("Continuation sigma must be finite in [0,128] cells");
  if (sigma == 0) return raw;
  const int radius = int(4*sigma + .5);
  std::vector<double> kernel(2*radius+1);
  double sum = 0;
  for (int k = -radius; k <= radius; ++k) {
    kernel[k+radius] = std::exp(-.5 * k*k / (sigma*sigma));
    sum += kernel[k+radius];
  }
  for (auto& value : kernel) value /= sum;
  Eigen::VectorXd known = Eigen::VectorXd::Zero(raw.size()), field = known;
  for (int j = 0; j < raw.size(); ++j) if (raw.counts[j] > minimum_count) {
    known[j] = 1; field[j] = raw.values[j] / raw.counts[j];
  }
  const auto numerator = convolve(field, raw.width, raw.height, kernel);
  const auto denominator = convolve(known, raw.width, raw.height, kernel);
  Grid result = raw;
  // Counts and visibility remain raw. Unknown cells never become observations.
  for (int j = 0; j < raw.size(); ++j)
    result.values[j] = known[j] && denominator[j] > 0 ?
        raw.counts[j] * numerator[j] / denominator[j] : 0;
  return result;
}

void fit_robust_observed_map(Grid& state, const std::vector<Row>& rows, double delta) {
  size_t begin = 0;
  while (begin < rows.size()) {
    size_t end = begin+1;
    while (end < rows.size() && rows[end].cell == rows[begin].cell) ++end;
    double lower = INFINITY, upper = -INFINITY;
    for (size_t k = begin; k < end; ++k) {
      const double value = rows[k].h / rows[k].weight;
      lower = std::min(lower, value); upper = std::max(upper, value);
    }
    // Convex scalar Huber problem. Its derivative is monotone even where the
    // minimizer is an interval. No pose or count derivative is introduced.
    for (int iteration = 0; iteration < 48 && upper > lower; ++iteration) {
      const double middle = .5 * (lower + upper);
      double derivative = 0;
      for (size_t k = begin; k < end; ++k) {
        const auto& row = rows[k];
        derivative += row.count * std::clamp(
            row.count * (middle - row.h / row.weight), -delta, delta);
      }
      if (derivative > 0) upper = middle; else lower = middle;
    }
    state.values[rows[begin].cell] = .5 * (lower + upper);
    begin = end;
  }
}

Result optimize_globalized(const std::vector<Grid>& raw, const std::vector<Pose>& input,
                           const Options& options) {
  const auto& sigmas = options.continuation_sigmas;
  if (sigmas.empty() || sigmas.back() != 0 || options.max_backtracks < 1 ||
      options.max_backtracks > 30 || options.max_iterations < 0)
    throw std::runtime_error("Invalid globalized inverse schedule");
  Result result;
  result.initial_poses = input; result.poses = input;
  result.pose_history.push_back(input);
  const Grid domain = allocate_global(raw, input, options);
  result.initial = integrate(raw, input, domain);
  int solves = 0;
  for (size_t level = 0; level < sigmas.size(); ++level) {
    std::vector<Grid> locals;
    for (const auto& grid : raw)
      locals.push_back(smooth_observed_field(grid, sigmas[level], options.minimum_count));
    Grid state = integrate(locals, result.poses, domain);
    for (int j = 0; j < state.size(); ++j)
      state.values[j] = state.counts[j] > 0 ? state.values[j] / state.counts[j] : 0;
    const int budget = options.max_iterations / int(sigmas.size()) +
        (int(level) < options.max_iterations % int(sigmas.size()) ? 1 : 0);
    result.stop_reason = "solve_budget";
    for (int iteration = 0; iteration < budget; ++iteration, ++solves) {
      auto rows = observed_rows(locals, result.poses, state, options);
      if (rows.empty()) throw std::runtime_error("No observed inverse residuals");
      fit_robust_observed_map(state, rows, options.huber_delta);
      rows = observed_rows(locals, result.poses, state, options);
      state.counts = observation_counts(rows, state.size());
      if (iteration == 0 || solves < 3 || solves % options.trace_stride == 0)
        inverse_trace(options, solves, state, result.poses, rows);
      const auto delta = solve(reduce_blocks(rows, int(input.size()), state.size()));
      const double before = robust_cost(state, rows, options.huber_delta);
      bool accepted = false;
      double alpha = 1;
      for (int trial = 0; trial < options.max_backtracks; ++trial, alpha *= .5) {
        auto candidate_poses = result.poses;
        for (size_t i = 1; i < input.size(); ++i) {
          candidate_poses[i] += alpha * delta.segment<3>(3*(i-1));
          candidate_poses[i].z() = std::atan2(std::sin(candidate_poses[i].z()),
                                            std::cos(candidate_poses[i].z()));
        }
        try { integrate(raw, candidate_poses, domain); }
        catch (const std::runtime_error&) {
          result.trials.push_back({solves, int(level), trial, sigmas[level], alpha,
                                   before, INFINITY, 0, rows.size(), false});
          continue;
        }
        auto trial_rows = rows;
        size_t missing = 0;
        for (auto& row : trial_rows) {
          const Eigen::Vector2d world = state.origin + state.resolution *
              Eigen::Vector2d(row.cell % state.width, row.cell / state.width);
          const auto& pose = candidate_poses[row.submap];
          const Eigen::Vector2d local = rotation(pose.z()).transpose() *
              (world - pose.head<2>());
          Sample sample;
          double value = row.h / row.weight;
          if (interpolate_observed(locals[row.submap], local, sample,
                                   options.minimum_count)) value = sample.value;
          else ++missing;
          // Old Ni and row keys define this conditional trial. Missing rows keep
          // their observation; disappearing alone cannot reduce the objective.
          row.h = row.count * value; row.weight = row.count;
        }
        Grid candidate = state;
        fit_robust_observed_map(candidate, trial_rows, options.huber_delta);
        const double after = robust_cost(candidate, trial_rows, options.huber_delta);
        const bool take = after < before - 1e-12 * std::max(1., before);
        result.trials.push_back({solves, int(level), trial, sigmas[level], alpha,
                                 before, after, missing, rows.size(), take});
        if (take) {
          double frozen_cost = 0;
          for (const auto& row : rows) {
            const double error = row.weight*state.values[row.cell]-row.h;
            frozen_cost += error*error;
          }
          result.poses = std::move(candidate_poses); state = std::move(candidate);
          result.iterations.push_back({solves, frozen_cost, alpha*alpha*delta.squaredNorm(), rows.size()});
          result.pose_history.push_back(result.poses);
          accepted = true; break;
        }
      }
      if (!accepted) { result.stop_reason = "no_conditional_decrease"; ++solves; break; }
      if (alpha*alpha*delta.squaredNorm()/std::max(1, int(input.size())-1) <= options.tolerance) {
        result.stop_reason = "small_accepted_step"; ++solves; break;
      }
    }
  }
  Grid field = integrate(raw, result.poses, domain);
  for (int j = 0; j < field.size(); ++j)
    field.values[j] = field.counts[j] > 0 ? field.values[j]/field.counts[j] : 0;
  auto rows = observed_rows(raw, result.poses, field, options);
  fit_robust_observed_map(field, rows, options.huber_delta);
  rows = observed_rows(raw, result.poses, field, options);
  field.counts = observation_counts(rows, field.size());
  inverse_trace(options, solves, field, result.poses, rows);
  result.optimized = field;
  result.optimized.values.array() *= field.counts.array();
  result.reintegrated = integrate(raw, result.poses, domain);
  return result;
}
}  // namespace joining
