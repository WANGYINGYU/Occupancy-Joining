#include "occupancy_joining/core.hpp"
#include <algorithm>
namespace joining {
// Same Schur equations, touching only pose blocks present at each map vertex.
Linearization reduce_blocks(const std::vector<Row>& rows,int poses,int cells){
  int n=3*(poses-1);Linearization out;out.S=Eigen::MatrixXd::Zero(n,n);out.b=Eigen::VectorXd::Zero(n);out.map=Eigen::VectorXd::Zero(cells);out.rows=rows.size();
  // Rows are grouped by global cell. Each scalar map variable can be eliminated independently.
  std::vector<int> active;active.reserve(poses);
  size_t a=0;while(a<rows.size()){size_t b=a;int cell=rows[a].cell;double v=0,z=0;Eigen::VectorXd w=Eigen::VectorXd::Zero(n);active.clear();
    while(b<rows.size()&&rows[b].cell==cell){const auto& r=rows[b];v+=r.weight*r.weight;z+=r.weight*r.h;
      if(r.submap>0){int p=3*(r.submap-1);if(std::find(active.begin(),active.end(),p)==active.end())active.push_back(p);out.S.block<3,3>(p,p).noalias()+=r.jacobian*r.jacobian.transpose();out.b.segment<3>(p)+=r.jacobian*r.h;w.segment<3>(p)+=r.jacobian*r.weight;}++b;}
    if(v>0){for(int p:active){out.b.segment<3>(p).noalias()-=w.segment<3>(p)*(z/v);for(int q:active)out.S.block<3,3>(p,q).noalias()-=w.segment<3>(p)*w.segment<3>(q).transpose()/v;}out.map[cell]=z/v;
      for(size_t k=a;k<b;++k){double e=rows[k].weight*out.map[cell]-rows[k].h;out.cost+=e*e;}}
    a=b;}
  out.S=(out.S+out.S.transpose()).eval()*.5;return out;
}
}
