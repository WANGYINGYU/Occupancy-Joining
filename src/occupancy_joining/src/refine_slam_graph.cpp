#include <karto_sdk/Mapper.h>
#include <ceres/ceres.h>
#include <Eigen/Cholesky>
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/resource.h>

namespace {
struct Constraint {
  int source, target;
  Eigen::Vector3d measurement;
  Eigen::Matrix3d information;
  bool supplied = false;
};
struct RelativePoseCost {
  Eigen::Vector3d measurement;
  Eigen::Matrix3d whitening;
  template<class T> bool operator()(const T* source, const T* target, T* output) const {
    const T c = ceres::cos(source[2]), s = ceres::sin(source[2]);
    const T dx = target[0]-source[0], dy = target[1]-source[1];
    const T angle = target[2]-source[2]-T(measurement[2]);
    Eigen::Matrix<T,3,1> error;
    error << c*dx+s*dy-T(measurement[0]), -s*dx+c*dy-T(measurement[1]),
        ceres::atan2(ceres::sin(angle), ceres::cos(angle));
    Eigen::Map<Eigen::Matrix<T,3,1>> residual(output);
    residual = whitening.cast<T>()*error;
    return true;
  }
};
void validateInformation(const Eigen::Matrix3d& information) {
  if (!information.allFinite() || !information.isApprox(information.transpose(), 1e-9) ||
      Eigen::LLT<Eigen::Matrix3d>(information).info() != Eigen::Success)
    throw std::runtime_error("Constraint information must be finite, symmetric and positive definite");
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
      std::cout << "Usage: joining_refine_slam_graph graph_prefix constraints.txt output.csv summary.txt [initial.csv]\n"
                << "Constraint rows: source_id target_id dx dy yaw H00 H01 H02 H10 H11 H12 H20 H21 H22\n"
                << "Each measurement is T_source_inverse * T_target; information is symmetric positive definite.\n";
      return 0;
    }
    if (argc != 5 && argc != 6)
      throw std::runtime_error("Usage: joining_refine_slam_graph graph_prefix constraints.txt output.csv summary.txt [initial.csv]");
    const std::filesystem::path prefix(argv[1]), output(argv[3]), summary_path(argv[4]);
    if (std::filesystem::exists(output) || std::filesystem::exists(summary_path) ||
        std::filesystem::absolute(output) == std::filesystem::absolute(summary_path))
      throw std::runtime_error("Output CSV and summary must be distinct new files");
    struct rlimit limit;
    if (getrlimit(RLIMIT_STACK, &limit) != 0) throw std::runtime_error("Cannot read stack limit");
    limit.rlim_cur = std::max<rlim_t>(limit.rlim_cur, 64*1024*1024);
    if (setrlimit(RLIMIT_STACK, &limit) != 0)
      throw std::runtime_error("Native graph loading requires a 64 MiB process stack");

    // Sensors owned by Dataset must outlive all Mapper scans.
    karto::Dataset dataset;
    karto::Mapper mapper;
    mapper.LoadFromFile(prefix.string()+".posegraph");
    dataset.LoadFromFile(prefix.string()+".data");
    for (auto* object : dataset.GetLasers())
      if (auto* sensor = dynamic_cast<karto::Sensor*>(object))
        karto::SensorManager::GetInstance()->RegisterSensor(sensor, true);
    std::map<int,std::array<double,3>> nodes;
    std::map<int,double> times;
    for (auto* scan : mapper.GetAllProcessedScans()) {
      const auto base = scan->GetCorrectedPose(), sensor = scan->GetSensorPose();
      const double angle = sensor.GetHeading()-base.GetHeading();
      if (std::hypot(sensor.GetX()-base.GetX(), sensor.GetY()-base.GetY()) > 1e-9 ||
          std::abs(std::atan2(std::sin(angle),std::cos(angle))) > 1e-9)
        throw std::runtime_error("Expected an Occupancy Joining virtual-laser graph: base and sensor frames must coincide");
      const int id = scan->GetUniqueId();
      const std::array<double,3> pose{base.GetX(),base.GetY(),base.GetHeading()};
      if (!std::all_of(pose.begin(),pose.end(),[](double x){return std::isfinite(x);}) ||
          !std::isfinite(scan->GetTime()) || !nodes.emplace(id,pose).second)
        throw std::runtime_error("Invalid or duplicate native graph node");
      times[id] = scan->GetTime();
    }
    if (nodes.empty()) throw std::runtime_error("Empty native graph");
    if (argc == 6) {
      std::ifstream file(argv[5]);
      std::string line;
      if (!std::getline(file,line) || line != "time_seconds,id,x,y,yaw")
        throw std::runtime_error("Initial CSV needs time_seconds,id,x,y,yaw columns");
      std::set<int> seen;
      while (std::getline(file,line)) {
        std::replace(line.begin(),line.end(),',',' ');
        std::istringstream row(line);
        double time, x, y, yaw; int id;
        if (!(row>>time>>id>>x>>y>>yaw) || !nodes.count(id) || !seen.insert(id).second ||
            !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw) || !std::isfinite(time) ||
            std::abs(time-times.at(id)) > std::max(1e-9,2*std::numeric_limits<double>::epsilon()*std::abs(time)))
          throw std::runtime_error("Initial CSV contains an invalid node, pose or timestamp");
        nodes[id] = {x,y,yaw};
      }
      if (seen.size() != nodes.size()) throw std::runtime_error("Initial CSV must cover every graph node");
    }
    const auto initial = nodes;
    std::vector<Constraint> edges;
    std::set<std::pair<int,int>> node_pairs;
    for (auto* edge : mapper.GetGraph()->GetEdges()) {
      auto* link = dynamic_cast<karto::LinkInfo*>(edge->GetLabel());
      if (!link) throw std::runtime_error("Unsupported native graph edge label");
      Constraint c;
      c.source = edge->GetSource()->GetObject()->GetUniqueId();
      c.target = edge->GetTarget()->GetObject()->GetUniqueId();
      const auto pose = link->GetPoseDifference();
      c.measurement << pose.GetX(),pose.GetY(),pose.GetHeading();
      const auto covariance = link->GetCovariance();
      Eigen::Matrix3d matrix;
      for (int i=0;i<3;++i) for (int j=0;j<3;++j) matrix(i,j)=covariance(i,j);
      validateInformation(matrix);
      c.information = matrix.llt().solve(Eigen::Matrix3d::Identity());
      edges.push_back(c);
      node_pairs.insert(std::minmax(c.source,c.target));
    }
    const size_t native_edges = edges.size();
    std::ifstream file(argv[2]);
    if (!file) throw std::runtime_error("Cannot open supplied constraints file");
    std::string line;
    while (std::getline(file,line)) {
      if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
      std::istringstream row(line); Constraint c;
      if (!(row>>c.source>>c.target>>c.measurement[0]>>c.measurement[1]>>c.measurement[2]))
        throw std::runtime_error("Invalid supplied constraint row");
      for (int i=0;i<3;++i) for (int j=0;j<3;++j)
        if (!(row>>c.information(i,j))) throw std::runtime_error("Expected nine information coefficients");
      std::string extra;
      if (row>>extra || !nodes.count(c.source) || !nodes.count(c.target) || c.source==c.target ||
          !c.measurement.allFinite() || !node_pairs.insert(std::minmax(c.source,c.target)).second)
        throw std::runtime_error("Invalid, repeated or already existing supplied constraint");
      validateInformation(c.information); c.supplied=true; edges.push_back(c);
    }
    ceres::Problem problem;
    for (auto& [id,pose] : nodes) problem.AddParameterBlock(pose.data(),3);
    for (const auto& c : edges) {
      if (!nodes.count(c.source) || !nodes.count(c.target)) throw std::runtime_error("Missing edge node");
      const Eigen::Matrix3d whitening = c.information.llt().matrixU();
      auto* cost = new ceres::AutoDiffCostFunction<RelativePoseCost,3,3,3>(
          new RelativePoseCost{c.measurement,whitening});
      problem.AddResidualBlock(cost,c.supplied ? new ceres::HuberLoss(1.) : nullptr,
                               nodes.at(c.source).data(),nodes.at(c.target).data());
    }
    const auto first = std::min_element(times.begin(),times.end(),[](const auto& a,const auto& b){return a.second<b.second;});
    problem.SetParameterBlockConstant(nodes.at(first->first).data());
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;
    options.sparse_linear_algebra_library_type=ceres::SUITE_SPARSE;
    options.trust_region_strategy_type=ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations=50; options.num_threads=1;
    options.function_tolerance=1e-3; options.parameter_tolerance=1e-3; options.gradient_tolerance=1e-6;
    options.use_nonmonotonic_steps=true; options.min_relative_decrease=1e-3; options.dynamic_sparsity=true;
    options.min_lm_diagonal=1e-6; options.max_lm_diagonal=1e32; options.max_num_consecutive_invalid_steps=3;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    std::cout<<summary.FullReport()<<std::endl;
    if (!summary.IsSolutionUsable()) throw std::runtime_error("Unusable native graph refinement");
    std::ofstream out(output);
    out<<std::setprecision(17)<<"time_seconds,id,x,y,yaw\n";
    double maximum=0;
    for (auto& [id,pose] : nodes) {
      pose[2]=std::atan2(std::sin(pose[2]),std::cos(pose[2]));
      out<<times.at(id)<<','<<id<<','<<pose[0]<<','<<pose[1]<<','<<pose[2]<<'\n';
      for (int k=0;k<3;++k) maximum=std::max(maximum,std::abs(pose[k]-initial.at(id)[k]));
    }
    out.close();
    std::ofstream stats(summary_path);
    stats<<std::setprecision(17)<<"nodes "<<nodes.size()<<"\nnative_edges "<<native_edges
         <<"\nnew_loops "<<edges.size()-native_edges<<"\ninitial_cost "<<summary.initial_cost
         <<"\nfinal_cost "<<summary.final_cost<<"\nmax_component_change "<<maximum
         <<"\niterations "<<summary.iterations.size()<<"\nusable 1\n";
    stats.close();
    if (!out || !stats) throw std::runtime_error("Could not write graph refinement outputs");
    return 0;
  } catch (const std::exception& error) {
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}
