#include "occupancy_joining/core.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCholesky>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <filesystem>
namespace joining {
Eigen::Matrix2d rotation(double t) {Eigen::Matrix2d R;R<<cos(t),-sin(t),sin(t),cos(t);return R;}
bool interpolate(const Grid& g,const Eigen::Vector2d& p,Sample& s,bool matlab_gradient){
  Eigen::Vector2d q=(p-g.origin)/g.resolution;
  if(!q.allFinite()||q.x()<0||q.y()<0||q.x()>=g.width-1||q.y()>=g.height-1)return false;
  int x=int(floor(q.x())),y=int(floor(q.y()));double u=q.x()-x,v=q.y()-y;
  int id=y*g.width+x;s.ids[0]=id;s.ids[1]=id+g.width;s.ids[2]=id+1;s.ids[3]=id+g.width+1;
  s.weights[0]=(1-u)*(1-v);s.weights[1]=(1-u)*v;s.weights[2]=u*(1-v);s.weights[3]=u*v;
  s.value=0;s.count=0;s.gradient.setZero();
  for(int k=0;k<4;++k){s.value+=s.weights[k]*g.values[s.ids[k]];s.count+=s.weights[k]*g.counts[s.ids[k]];}
  s.gradient<<((1-v)*(g.values[id+1]-g.values[id])+v*(g.values[id+g.width+1]-g.values[id+g.width]))/g.resolution,
    ((1-u)*(g.values[id+g.width]-g.values[id])+u*(g.values[id+g.width+1]-g.values[id+1]))/g.resolution;
  if(matlab_gradient){s.gradient.setZero();for(int k=0;k<4;++k){int j=s.ids[k],xx=j%g.width,yy=j/g.width;
    int a=std::max(0,xx-1),b=std::min(g.width-1,xx+1),c=std::max(0,yy-1),d=std::min(g.height-1,yy+1);
    s.gradient.x()+=s.weights[k]*(g.values[yy*g.width+b]-g.values[yy*g.width+a])/((b-a)*g.resolution);
    s.gradient.y()+=s.weights[k]*(g.values[d*g.width+xx]-g.values[c*g.width+xx])/((d-c)*g.resolution);}}
  return true;
}
Grid allocate_global(const std::vector<Grid>& locals,const std::vector<Pose>& poses,const Options& o){
  Eigen::Vector2d lo=Eigen::Vector2d::Constant(INFINITY),hi=-lo;
  for(size_t i=0;i<locals.size();++i){const auto& g=locals[i];auto R=rotation(poses[i].z());
    for(int j=0;j<g.size();++j)if(g.counts[j]>0){Eigen::Vector2d p=R*(g.origin+g.resolution*Eigen::Vector2d(j%g.width,j/g.width))+poses[i].head<2>();lo=lo.cwiseMin(p);hi=hi.cwiseMax(p);}}
  if(!lo.allFinite())throw std::runtime_error("No observed local grid cells");
  double pad=std::max(o.padding,.05*(hi-lo).maxCoeff());lo.array()-=pad;hi.array()+=pad;
  Grid g;g.resolution=o.global_resolution;g.origin=lo;
  double w=ceil((hi.x()-lo.x())/g.resolution)+1,h=ceil((hi.y()-lo.y())/g.resolution)+1;
  if(w<2||h<2||w*h>o.max_cells)throw std::runtime_error("Global grid exceeds max_cells or has invalid dimensions");
  g.width=int(w);g.height=int(h);g.values=Eigen::VectorXd::Zero(g.size());g.counts=g.values;return g;
}
Grid integrate(const std::vector<Grid>& locals,const std::vector<Pose>& poses,const Grid& domain,double threshold){
  Grid g=domain;g.values.setZero();g.counts.setZero();
  for(size_t i=0;i<locals.size();++i){const auto& l=locals[i];auto R=rotation(poses[i].z());
    for(int j=0;j<l.size();++j)if(l.counts[j]>0&&(threshold<0||abs(l.values[j])>=threshold)){
      Eigen::Vector2d q=(R*(l.origin+l.resolution*Eigen::Vector2d(j%l.width,j/l.width))+poses[i].head<2>()-g.origin)/g.resolution;
      if(!q.allFinite()||q.x()<=-.5||q.y()<=-.5||q.x()>=g.width-.5||q.y()>=g.height-.5)throw std::runtime_error("Submap moved outside fixed global domain; increase padding");
      int x=int(std::lround(q.x())),y=int(std::lround(q.y()));int k=y*g.width+x;g.values[k]+=l.values[j];g.counts[k]+=l.counts[j];}}
  return g;
}
std::vector<Row> paper_rows(const std::vector<Grid>& locals,const std::vector<Pose>& poses,const Grid& g,const Options& o){
  std::vector<Row> rows;
  for(int j=0;j<g.size();++j)if(g.counts[j]>o.minimum_count){Eigen::Vector2d world=g.origin+g.resolution*Eigen::Vector2d(j%g.width,j/g.width);
    for(size_t i=0;i<locals.size();++i){auto R=rotation(poses[i].z()).transpose().eval();Eigen::Vector2d local=R*(world-poses[i].head<2>());Sample s;
      if(!interpolate(locals[i],local,s)||s.count<=o.minimum_count)continue;
      Row row;row.cell=j;row.submap=int(i);row.h=s.value;row.weight=s.count/g.counts[j];
      row.jacobian.head<2>()=R.transpose()*s.gradient;
      row.jacobian.z()=s.gradient.x()*(-local.y())+s.gradient.y()*local.x();
      rows.push_back(row);}}
  return rows;
}
// MATLAB inverse residuals. Default preserves the source cell-coordinate
// gradient approximation. Metric mode explicitly adds the 1/LocalScale factor.
std::vector<Row> inverse_rows(const std::vector<Grid>& locals,
                              const std::vector<Pose>& poses,
                              const Grid& global, const Options& options) {
  std::vector<Row> rows;
  std::vector<Eigen::Matrix2d> rotations;
  for (const auto& pose : poses) rotations.push_back(rotation(pose.z()).transpose());
  for (int cell = 0; cell < global.size(); ++cell) {
    const Eigen::Vector2d world = global.origin + global.resolution *
        Eigen::Vector2d(cell % global.width, cell / global.width);
    for (size_t i = 0; i < locals.size(); ++i) {
      const auto& R = rotations[i];
      const Eigen::Vector2d local = R * (world - poses[i].head<2>());
      Sample sample;
      if (!interpolate(locals[i], local, sample, true) ||
          std::abs(sample.value) <= options.inverse_delete_threshold) continue;
      // The original inverse routine keeps finite zero-N cells and sets a
      // nonfinite N_local/N_global ratio to zero. Do not silently drop them.
      double weight = global.counts[cell] != 0 ? sample.count / global.counts[cell] : 0;
      if (!std::isfinite(weight)) weight = 0;
      const double error = weight * global.values[cell] - sample.value;
      if (!std::isfinite(error)) throw std::runtime_error("Nonfinite inverse residual");
      const double scale = std::sqrt(std::abs(error) > options.huber_delta ?
                                      options.huber_delta / std::abs(error) : 1.);
      if (options.inverse_gradient == "matlab")
        sample.gradient *= locals[i].resolution;
      Row row;
      row.cell = cell; row.submap = int(i);
      row.h = scale * sample.value; row.weight = scale * weight;
      row.jacobian.head<2>() = scale * R.transpose() * sample.gradient;
      row.jacobian.z() = scale * (sample.gradient.y() * local.x() -
                                   sample.gradient.x() * local.y());
      rows.push_back(row);
    }
  }
  return rows;
}

// Recover the map increment of the very same frozen-IRLS joint GN system.
// Unobserved variables retain their previous value; no spatial map prior is used.
Eigen::VectorXd back_substitute(const std::vector<Row>& rows,
                                const Eigen::VectorXd& pose_delta,
                                const Eigen::VectorXd& previous_map) {
  Eigen::VectorXd result = previous_map;
  size_t begin = 0;
  while (begin < rows.size()) {
    size_t end = begin; const int cell = rows[begin].cell;
    double diagonal = 0, rhs = 0;
    while (end < rows.size() && rows[end].cell == cell) {
      const auto& row = rows[end++];
      const double pose_term = row.submap ?
          row.jacobian.dot(pose_delta.segment<3>(3 * (row.submap - 1))) : 0.;
      diagonal += row.weight * row.weight;
      rhs += row.weight * (row.h - pose_term);
    }
    if (diagonal > 0) result[cell] = rhs / diagonal;
    begin = end;
  }
  return result;
}

Linearization reduce(const std::vector<Row>& rows,int poses,int cells){
  int n=3*(poses-1);Linearization out;out.S=Eigen::MatrixXd::Zero(n,n);out.b=Eigen::VectorXd::Zero(n);out.map=Eigen::VectorXd::Zero(cells);out.rows=rows.size();
  // Rows are grouped by global cell. Each scalar map variable can be eliminated independently.
  size_t a=0;while(a<rows.size()){size_t b=a;int cell=rows[a].cell;double v=0,z=0;Eigen::VectorXd w=Eigen::VectorXd::Zero(n);
    while(b<rows.size()&&rows[b].cell==cell){const auto& r=rows[b];v+=r.weight*r.weight;z+=r.weight*r.h;
      if(r.submap>0){int p=3*(r.submap-1);out.S.block<3,3>(p,p).noalias()+=r.jacobian*r.jacobian.transpose();out.b.segment<3>(p)+=r.jacobian*r.h;w.segment<3>(p)+=r.jacobian*r.weight;}++b;}
    if(v>0){out.S.noalias()-=w*w.transpose()/v;out.b.noalias()-=w*(z/v);out.map[cell]=z/v;
      for(size_t k=a;k<b;++k){double e=rows[k].weight*out.map[cell]-rows[k].h;out.cost+=e*e;}}
    a=b;}
  out.S=(out.S+out.S.transpose()).eval()*.5;return out;
}
Eigen::VectorXd solve(const Linearization& l){
  if(l.S.rows()==0)return Eigen::VectorXd();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen(l.S);
  if(eigen.info()!=Eigen::Success)throw std::runtime_error("Pose eigensolve failed");
  double largest=eigen.eigenvalues().maxCoeff(),smallest=eigen.eigenvalues().minCoeff();
  if(!(largest>0)||smallest<=largest*1e-11)throw std::runtime_error("Pose system rank deficient: insufficient overlapping geometry");
  Eigen::VectorXd d=eigen.eigenvectors()*eigen.eigenvalues().cwiseInverse().asDiagonal()*eigen.eigenvectors().transpose()*l.b;
  if(!d.allFinite()||(l.S*d-l.b).norm()>1e-6*std::max(1.,l.b.norm()))throw std::runtime_error("Invalid pose solve");return d;
}
using Sparse=Eigen::SparseMatrix<double>;
Sparse laplacian(int w,int h){std::vector<Eigen::Triplet<double>> t;for(int y=0;y<h;++y)for(int x=0;x<w;++x){int a=y*w+x;for(int b:{x+1<w?a+1:-1,y+1<h?a+w:-1})if(b>=0){t.emplace_back(a,a,1);t.emplace_back(b,b,1);t.emplace_back(a,b,-1);t.emplace_back(b,a,-1);}}Sparse L(w*h,w*h);L.setFromTriplets(t.begin(),t.end());return L;}
Eigen::VectorXd matlab_step(const std::vector<Grid>& locals,const std::vector<Pose>& poses,const Grid& g,const Options& o,double* cost){
  int np=3*(int(poses.size())-1),nv=np+g.size(),nrow=0;std::vector<Eigen::Triplet<double>> t;std::vector<double> residual;
  if(cost)*cost=0;
  for(size_t i=0;i<locals.size();++i){const auto& l=locals[i];auto R=rotation(poses[i].z());for(int j=0;j<l.size();++j)if(l.counts[j]>0){
    Eigen::Vector2d local=l.origin+l.resolution*Eigen::Vector2d(j%l.width,j/l.width),rp=R*local;Sample s;
    if(!interpolate(g,rp+poses[i].head<2>(),s,true)||s.count<=0)throw std::runtime_error("MATLAB reference sample outside positive N support");
    double e=s.value/s.count-l.values[j]/l.counts[j];double scale=std::sqrt(abs(e)>o.huber_delta?o.huber_delta/abs(e):1.);
    residual.push_back(e*scale);if(cost)*cost+=e*e*scale*scale;
    if(i){Pose jac;jac.head<2>()=s.gradient/s.count;jac.z()=s.gradient.dot(Eigen::Vector2d(-rp.y(),rp.x()))/s.count;
      for(int k=0;k<3;++k)t.emplace_back(nrow,3*(i-1)+k,scale*jac[k]);}
    for(int k=0;k<4;++k)t.emplace_back(nrow,np+s.ids[k],scale*s.weights[k]/s.count);++nrow;}}
  Sparse J(nrow,nv);J.setFromTriplets(t.begin(),t.end());Eigen::Map<Eigen::VectorXd> e(residual.data(),nrow);
  Sparse H=J.transpose()*J;Eigen::VectorXd rhs=-J.transpose()*e;Sparse L=laplacian(g.width,g.height);
  std::vector<Eigen::Triplet<double>> t2;for(int k=0;k<L.outerSize();++k)for(Sparse::InnerIterator it(L,k);it;++it)t2.emplace_back(np+it.row(),np+it.col(),o.map_regularization*it.value());
  Sparse reg(nv,nv);reg.setFromTriplets(t2.begin(),t2.end());H+=reg;rhs.tail(g.size())-=o.map_regularization*(L*g.values);
  Eigen::SimplicialLDLT<Sparse> solver;solver.compute(H);if(solver.info()!=Eigen::Success)throw std::runtime_error("MATLAB joint normal matrix factorization failed");
  Eigen::VectorXd d=solver.solve(rhs);if(solver.info()!=Eigen::Success||!d.allFinite())throw std::runtime_error("MATLAB joint solve failed");return d;
}
Grid refresh_counts(const std::vector<Grid>& locals, const std::vector<Pose>& poses,
                    const Grid& domain, const Options& options) {
  Grid next = integrate(locals, poses, domain, options.count_delete_threshold);
  if (options.n_smoothing <= 0) return next;
  Sparse L = laplacian(next.width, next.height);
  Sparse A = options.n_smoothing * L;
  Eigen::VectorXd rhs = Eigen::VectorXd::Zero(next.size());
  for (int j = 0; j < next.size(); ++j) if (next.counts[j] != 0) {
    A.coeffRef(j,j) += 1.; rhs[j] = next.counts[j];
  }
  Eigen::SimplicialLDLT<Sparse> solver;
  solver.compute(A);
  if (solver.info() != Eigen::Success) throw std::runtime_error("N smoothing factorization failed");
  next.counts = solver.solve(rhs);
  if (solver.info() != Eigen::Success || !next.counts.allFinite())
    throw std::runtime_error("N smoothing solve failed");
  return next;
}

void inverse_trace(const Options& options, int iteration, const Grid& state,
                   const std::vector<Pose>& poses, const std::vector<Row>& rows) {
  if (options.trace_directory.empty()) return;
  const auto path = std::filesystem::path(options.trace_directory) / std::to_string(iteration);
  std::filesystem::create_directories(path);
  write_grid(state, (path / "map.grid").string());
  std::ofstream pf(path / "poses.txt"), rf(path / "rows.txt");
  pf << std::setprecision(17); rf << std::setprecision(17);
  for (const auto& pose : poses) pf << pose.transpose() << '\n';
  for (const auto& row : rows) {
    rf << row.cell << ' ' << row.submap << ' ' << row.h << ' '
       << row.weight << ' ' << row.jacobian.transpose();
    if (options.model == "observed_inverse" || options.model == "information_inverse" || options.model == "supported_inverse" || options.model == "globalized_inverse") rf << ' ' << row.count;
    rf << '\n';
  }
}

Result optimize_inverse(const std::vector<Grid>& locals, const std::vector<Pose>& input,
                        const Options& options) {
  Result result; result.initial_poses = input; result.poses = input;
  result.pose_history.push_back(input);
  const bool normalized = options.model == "observed_inverse" || options.model == "information_inverse";
  const bool supported = normalized || options.model == "supported_inverse";
  const Grid domain = allocate_global(locals, input, options);
  result.initial = integrate(locals, input, domain);
  Grid state = result.initial;
  if (normalized) for (int j = 0; j < state.size(); ++j)
    state.values[j] = state.counts[j] > 0 ? state.values[j] / state.counts[j] : 0.;
  auto sample_rows = [&]() {
    return supported ? observed_rows(locals, result.poses, state, options, normalized) :
                       inverse_rows(locals, result.poses, state, options);
  };
  result.stop_reason = "iteration_limit";
  for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
    const auto rows = sample_rows();
    if (normalized) state.counts = observation_counts(rows, state.size());
    if (rows.empty()) throw std::runtime_error("No supported inverse residuals");
    if (iteration < 3 || iteration % options.trace_stride == 0)
      inverse_trace(options, iteration, state, result.poses, rows);
    const auto linearization = reduce(rows, int(input.size()), state.size());
    const Eigen::VectorXd delta = solve(linearization);
    double cost = 0;
    for (const auto& row : rows) {
      const double error = row.weight * state.values[row.cell] - row.h;
      cost += error * error;
    }
    state.values = back_substitute(rows, delta, state.values);
    for (size_t i = 1; i < result.poses.size(); ++i) {
      result.poses[i] += delta.segment<3>(3 * (i - 1));
      result.poses[i].z() = std::atan2(std::sin(result.poses[i].z()),
                                     std::cos(result.poses[i].z()));
    }
    if (!normalized) state.counts = refresh_counts(locals, result.poses, domain, options).counts;
    result.iterations.push_back({iteration, cost, delta.squaredNorm(), rows.size()});
    result.pose_history.push_back(result.poses);
    // Match MATLAB's mean squared step per free submap, not the total norm.
    const double mean_step = delta.squaredNorm() / std::max(1, int(input.size()) - 1);
    if (mean_step <= options.tolerance) {
      result.stop_reason = "small_mean_pose_step"; break;
    }
  }
  const auto final_rows = sample_rows();
  if (normalized) state.counts = observation_counts(final_rows, state.size());
  inverse_trace(options, int(result.iterations.size()), state, result.poses, final_rows);
  result.optimized = state;
  // Weighted equation (15) at final poses, with the final IRLS weights frozen.
  result.optimized.values = back_substitute(final_rows,
      Eigen::VectorXd::Zero(3 * (int(input.size()) - 1)), state.values);
  if (normalized) result.optimized.values.array() *= state.counts.array();
  result.optimized.counts.setZero();
  for (const auto& row : final_rows) if (row.weight != 0)
    result.optimized.counts[row.cell] = state.counts[row.cell];
  result.reintegrated = integrate(locals, result.poses, domain);
  return result;
}

Result optimize(const std::vector<Grid>& locals,const std::vector<Pose>& input,const Options& o){
  if(locals.empty()||locals.size()!=input.size())throw std::runtime_error("Need corresponding submaps and poses");
  if(o.model=="globalized_inverse")return optimize_globalized(locals,input,o);
  if(o.model=="matlab_inverse"||o.model=="supported_inverse"||o.model=="observed_inverse"||o.model=="information_inverse")return optimize_inverse(locals,input,o);
  Result r;r.initial_poses=input;r.poses=input;Grid domain=allocate_global(locals,input,o);r.initial=integrate(locals,input,domain);Grid state=r.initial;
  r.stop_reason="iteration_limit";
  for(int iter=0;iter<o.max_iterations;++iter){Eigen::VectorXd d;double cost=0;size_t nrows=0;
    if(o.model=="paper"){auto l=reduce(paper_rows(locals,r.poses,state,o),int(input.size()),state.size());d=solve(l);cost=l.cost;nrows=l.rows;}
    else {d=matlab_step(locals,r.poses,state,o,&cost);nrows=0;}
    double step=d.head(3*(int(input.size())-1)).squaredNorm();
    for(size_t i=1;i<r.poses.size();++i)r.poses[i]+=d.segment<3>(3*(i-1));
    Grid next=integrate(locals,r.poses,domain,o.model=="matlab"?.001:-1);
    if(o.model=="matlab"){
      state.values+=d.tail(state.size());state.counts=next.counts;
      if(o.n_smoothing>0){Sparse L=laplacian(state.width,state.height);Sparse A=o.n_smoothing*L;Eigen::VectorXd rhs=Eigen::VectorXd::Zero(state.size());
        for(int j=0;j<state.size();++j)if(state.counts[j]!=0){A.coeffRef(j,j)+=1;rhs[j]=state.counts[j];}
        Eigen::SimplicialLDLT<Sparse> s;s.compute(A);if(s.info()!=Eigen::Success)throw std::runtime_error("N smoothing factorization failed");state.counts=s.solve(rhs);}}
    else state=std::move(next);
    r.iterations.push_back({iter,cost,step,nrows});
    if(step<=o.tolerance){r.stop_reason="small_pose_step";break;}
  }
  r.reintegrated=integrate(locals,r.poses,domain);r.optimized=state;
  if(o.model=="paper"){auto l=reduce(paper_rows(locals,r.poses,state,o),int(input.size()),state.size());r.optimized.values=l.map;
    // Mark only cells with residual support, rather than interpreting unsupported zero values as free.
    r.optimized.counts.setZero();for(const auto& row:paper_rows(locals,r.poses,state,o))r.optimized.counts[row.cell]=state.counts[row.cell];}
  return r;
}
Grid read_grid(const std::string& path){std::ifstream f(path);Grid g;if(!(f>>g.width>>g.height>>g.resolution>>g.origin.x()>>g.origin.y())||g.width<2||g.height<2||double(g.width)*g.height>20000000||!std::isfinite(g.resolution)||g.resolution<=0||!g.origin.allFinite())throw std::runtime_error("Invalid grid header: "+path);
  g.values.resize(g.size());g.counts.resize(g.size());for(int j=0;j<g.size();++j)if(!(f>>g.values[j]>>g.counts[j])||!std::isfinite(g.values[j])||!std::isfinite(g.counts[j])||g.counts[j]<0)throw std::runtime_error("Invalid grid value/count: "+path);return g;}
void write_grid(const Grid& g,const std::string& path){std::ofstream f(path);if(!f)throw std::runtime_error("Cannot write grid: "+path);f<<std::setprecision(17)<<g.width<<' '<<g.height<<' '<<g.resolution<<' '<<g.origin.x()<<' '<<g.origin.y()<<'\n';for(int j=0;j<g.size();++j)f<<g.values[j]<<' '<<g.counts[j]<<'\n';}
}
