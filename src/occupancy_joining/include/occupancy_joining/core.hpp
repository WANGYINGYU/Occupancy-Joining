#pragma once
#include <Eigen/Core>
#include <string>
#include <vector>
namespace joining {
using Pose = Eigen::Vector3d;
struct Grid {
  int width=0,height=0;
  double resolution=.1;
  Eigen::Vector2d origin=Eigen::Vector2d::Zero();
  Eigen::VectorXd values, counts;
  int size() const {return width*height;}
};
struct Sample {double value=0,count=0;Eigen::Vector2d gradient=Eigen::Vector2d::Zero();int ids[4];double weights[4];};
struct Row {int cell=0,submap=0;double h=0,weight=0,count=0;Pose jacobian=Pose::Zero();};
struct Options {
  std::string model="matlab_inverse";
  std::string trace_directory;
  std::string inverse_gradient="matlab";
  int max_iterations=70;
  int trace_stride=1;
  int max_backtracks=10;
  std::vector<double> continuation_sigmas={0.};
  double tolerance=1e-7,global_resolution=.1,padding=5,minimum_count=1e-9;
  double map_regularization=.001,huber_delta=1.,n_smoothing=1e-6;
  double inverse_delete_threshold=.5,count_delete_threshold=.001;
  int max_cells=2000000;
};
struct Linearization {Eigen::MatrixXd S;Eigen::VectorXd b, map;double cost=0;size_t rows=0;};
struct Iteration {int index;double cost,step_squared;size_t rows;};
struct StepTrial {int solve,level,trial;double sigma,alpha,before,after;size_t missing,rows;bool accepted;};
struct Result {std::vector<StepTrial> trials;Grid initial,optimized,reintegrated;std::vector<Pose> initial_poses,poses;std::vector<Iteration> iterations;std::vector<std::vector<Pose>> pose_history;std::string stop_reason;};
Eigen::Matrix2d rotation(double theta);
bool interpolate(const Grid&,const Eigen::Vector2d&,Sample&,bool matlab_gradient=false);
bool interpolate_observed(const Grid&,const Eigen::Vector2d&,Sample&,double minimum_count=1e-9,bool normalized=true);
std::vector<Row> observed_rows(const std::vector<Grid>&,const std::vector<Pose>&,const Grid&,const Options&,bool normalized=true);
Eigen::VectorXd observation_counts(const std::vector<Row>&,int cells);
Grid allocate_global(const std::vector<Grid>&,const std::vector<Pose>&,const Options&);
Grid integrate(const std::vector<Grid>&,const std::vector<Pose>&,const Grid&,double delete_threshold=-1);
std::vector<Row> paper_rows(const std::vector<Grid>&,const std::vector<Pose>&,const Grid&,const Options&);
std::vector<Row> inverse_rows(const std::vector<Grid>&,const std::vector<Pose>&,const Grid&,const Options&);
Eigen::VectorXd back_substitute(const std::vector<Row>&,const Eigen::VectorXd& pose_delta,const Eigen::VectorXd& previous_map);
Linearization reduce(const std::vector<Row>&,int poses,int cells);
Linearization reduce_blocks(const std::vector<Row>&,int poses,int cells);
Eigen::VectorXd solve(const Linearization&);
Eigen::VectorXd matlab_step(const std::vector<Grid>&,const std::vector<Pose>&,const Grid&,const Options&,double* cost=nullptr);
Result optimize(const std::vector<Grid>&,const std::vector<Pose>&,const Options&);
Grid smooth_observed_field(const Grid&,double sigma,double minimum_count=1e-9);
void fit_robust_observed_map(Grid&,const std::vector<Row>&,double huber_delta);
Result optimize_globalized(const std::vector<Grid>&,const std::vector<Pose>&,const Options&);
Grid read_grid(const std::string&);
void write_grid(const Grid&,const std::string&);
}
