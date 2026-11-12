#include "occupancy_joining/core.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
using namespace joining;namespace fs=std::filesystem;
int main(int argc,char** argv){fs::path output;try{
  if(argc==2&&std::string(argv[1])=="--help"){std::cout<<"joining_backend --manifest submaps.yaml --output directory\n";return 0;}
  fs::path manifest;for(int i=1;i<argc;++i){std::string key=argv[i];if(i+1>=argc)throw std::runtime_error("Missing option value");if(key=="--manifest")manifest=argv[++i];else if(key=="--output")output=argv[++i];else throw std::runtime_error("Unknown option "+key);}
  if(manifest.empty()||output.empty())throw std::runtime_error("--manifest and --output required");
  if(fs::exists(output)&&!fs::is_empty(output))throw std::runtime_error("Output must be empty");fs::create_directories(output);
  auto cfg=YAML::LoadFile(manifest.string());Options o;auto p=cfg["optimizer"];
#define SET(name) if(p&&p[#name])o.name=p[#name].as<decltype(o.name)>();
  SET(model) SET(max_iterations) SET(tolerance) SET(global_resolution) SET(padding) SET(minimum_count) SET(max_cells) SET(huber_delta) SET(n_smoothing) SET(inverse_delete_threshold) SET(count_delete_threshold) SET(inverse_gradient) SET(max_backtracks) SET(continuation_sigmas)
#undef SET
  if((o.model!="matlab_inverse"&&o.model!="globalized_inverse")||(o.inverse_gradient!="matlab"&&o.inverse_gradient!="metric")||o.max_iterations<0||!std::isfinite(o.tolerance)||o.tolerance<0||!std::isfinite(o.global_resolution)||o.global_resolution<=0||!std::isfinite(o.padding)||o.padding<=0||o.max_cells<4||!std::isfinite(o.minimum_count)||o.minimum_count<0||!std::isfinite(o.huber_delta)||o.huber_delta<=0||!std::isfinite(o.n_smoothing)||o.n_smoothing<0||!std::isfinite(o.inverse_delete_threshold)||o.inverse_delete_threshold<0||!std::isfinite(o.count_delete_threshold)||o.count_delete_threshold<0)throw std::runtime_error("Invalid optimizer options");
  if(o.max_backtracks<1||o.max_backtracks>30||o.continuation_sigmas.empty()||o.continuation_sigmas.back()!=0)throw std::runtime_error("Invalid globalization schedule");
  for(double sigma:o.continuation_sigmas)if(!std::isfinite(sigma)||sigma<0||sigma>128)throw std::runtime_error("Invalid continuation sigma");
  std::vector<Grid> grids;std::vector<Pose> poses;for(const auto& item:cfg["submaps"]){fs::path path=item["grid"].as<std::string>();if(path.is_relative())path=manifest.parent_path()/path;grids.push_back(read_grid(path.string()));auto v=item["pose"].as<std::vector<double>>();if(v.size()!=3)throw std::runtime_error("Pose needs x y yaw");poses.emplace_back(v[0],v[1],v[2]);if(!poses.back().allFinite())throw std::runtime_error("Nonfinite pose");}
  auto r=optimize(grids,poses,o);
  for(const auto& s:{"initial","optimized","reintegrated"})fs::create_directory(output/s);
  write_grid(r.initial,(output/"initial/map.grid").string());write_grid(r.optimized,(output/"optimized/map.grid").string());write_grid(r.reintegrated,(output/"reintegrated/map.grid").string());
  for(const auto& name:{"initial","optimized","reintegrated"}){std::ofstream f(output/name/"submap_poses.txt");f<<std::setprecision(17);for(const auto& t:std::string(name)=="initial"?r.initial_poses:r.poses)f<<t.transpose()<<'\n';}
  std::ofstream log(output/"iterations.csv");log<<"iteration,frozen_model_cost,pose_step_squared,residual_rows\n";for(const auto& v:r.iterations)log<<v.index<<','<<std::setprecision(17)<<v.cost<<','<<v.step_squared<<','<<v.rows<<'\n';
  if(!r.pose_history.empty()){
    std::ofstream history(output/"pose_history.csv");history<<"updates,submap,x,y,yaw\n"<<std::setprecision(17);
    for(size_t k=0;k<r.pose_history.size();++k)for(size_t i=0;i<r.pose_history[k].size();++i){const auto& v=r.pose_history[k][i];history<<k<<','<<i<<','<<v.x()<<','<<v.y()<<','<<v.z()<<'\n';}
  }
  if(!r.trials.empty()){std::ofstream f(output/"acceptance.csv");f<<std::setprecision(17)<<"solve,level,sigma,trial,alpha,base_huber,trial_huber,missing_rows,residual_rows,accepted\n";for(const auto& a:r.trials)f<<a.solve<<','<<a.level<<','<<a.sigma<<','<<a.trial<<','<<a.alpha<<','<<a.before<<','<<a.after<<','<<a.missing<<','<<a.rows<<','<<a.accepted<<'\n';}
  std::ofstream summary(output/"status.yaml");summary<<"status: completed\nmodel: "<<o.model<<"\nstop_reason: "<<r.stop_reason<<"\niterations: "<<r.iterations.size()<<"\nsubmaps: "<<poses.size()<<"\n";
  std::cout<<"Completed "<<o.model<<" joining: "<<r.stop_reason<<" ("<<r.iterations.size()<<" updates)\n";
  return 0;
}catch(const std::exception& e){if(!output.empty()&&fs::is_directory(output)){std::ofstream f(output/"failure.txt");f<<e.what()<<'\n';}std::cerr<<e.what()<<'\n';return 1;}}
