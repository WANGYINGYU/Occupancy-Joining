#include "occupancy_joining/core.hpp"
#include <cmath>
#include <algorithm>

namespace joining {
bool interpolate_observed(const Grid& grid, const Eigen::Vector2d& point,
                          Sample& sample, double minimum_count) {
  if (!interpolate(grid, point, sample)) return false;
  // Occupancy zero is a valid observation. Visibility comes only from raw counts.
  for (int k = 0; k < 4; ++k)
    if (!(grid.counts[sample.ids[k]] > minimum_count)) return false;
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
                               const Grid& global, const Options& options) {
  std::vector<Row> rows;
  // Enumerate only the conservative world AABB of each local rectangle.
  for(size_t i=0;i<locals.size();++i){
    const auto& g=locals[i];
    const Eigen::Matrix2d R=rotation(poses[i].z()).transpose();
    Eigen::Vector2d lo=Eigen::Vector2d::Constant(INFINITY),hi=-lo;
    for(int yy:{0,g.height-1})for(int xx:{0,g.width-1}){
      const Eigen::Vector2d corner=g.origin+g.resolution*Eigen::Vector2d(xx,yy);
      const Eigen::Vector2d world=R.transpose()*corner+poses[i].head<2>();
      lo=lo.cwiseMin(world);hi=hi.cwiseMax(world);
    }
    const Eigen::Vector2d a=(lo-global.origin)/global.resolution,b=(hi-global.origin)/global.resolution;
    const int x0=std::max(0,int(std::floor(a.x()))-1),x1=std::min(global.width-1,int(std::ceil(b.x()))+1);
    const int y0=std::max(0,int(std::floor(a.y()))-1),y1=std::min(global.height-1,int(std::ceil(b.y()))+1);
    for(int y=y0;y<=y1;++y)for(int x=x0;x<=x1;++x){
      const int cell=y*global.width+x;
      const Eigen::Vector2d world=global.origin+global.resolution*Eigen::Vector2d(cell%global.width,cell/global.width);
      const Eigen::Vector2d local=R*(world-poses[i].head<2>());
      Sample sample;
      if (!interpolate_observed(locals[i], local, sample, options.minimum_count)) continue;
      const double amplitude = sample.count;
      const double coefficient = amplitude;
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
  // Restore the original cell/submap order for identical Schur accumulation.
  std::sort(rows.begin(),rows.end(),[](const Row&a,const Row&b){return a.cell<b.cell||(a.cell==b.cell&&a.submap<b.submap);});
  return rows;
}

Eigen::VectorXd observation_counts(const std::vector<Row>& rows, int cells) {
  Eigen::VectorXd counts = Eigen::VectorXd::Zero(cells);
  for (const auto& row : rows) counts[row.cell] += row.count;
  return counts;
}
}  // namespace joining
