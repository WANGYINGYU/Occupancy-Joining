#include <karto_sdk/Mapper.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <sys/resource.h>

int main(int argc, char** argv) {
  try {
    if (argc != 4)
      throw std::runtime_error("Expected graph_prefix output.csv stats.txt");
    // Boost deserialization of large native graphs can exceed the default stack.
    struct rlimit limit;
    if (getrlimit(RLIMIT_STACK, &limit) != 0)
      throw std::runtime_error("Cannot read the process stack limit");
    constexpr rlim_t needed = 64 * 1024 * 1024;
    if (limit.rlim_cur < needed) {
      limit.rlim_cur = needed;
      if (setrlimit(RLIMIT_STACK, &limit) != 0)
        throw std::runtime_error("Native graph export requires a 64 MiB stack; raise the process stack limit");
    }

    // Dataset owns the deserialized sensors and must outlive Mapper scans.
    karto::Dataset dataset;
    karto::Mapper mapper;
    mapper.LoadFromFile(std::string(argv[1]) + ".posegraph");
    dataset.LoadFromFile(std::string(argv[1]) + ".data");
    for (auto* object : dataset.GetLasers())
      if (auto* sensor = dynamic_cast<karto::Sensor*>(object))
        karto::SensorManager::GetInstance()->RegisterSensor(sensor, true);

    std::map<int, karto::LocalizedRangeScan*> graph;
    for (auto* scan : mapper.GetAllProcessedScans())
      if (!graph.emplace(scan->GetUniqueId(), scan).second)
        throw std::runtime_error("Duplicate native graph scan ID");
    std::ofstream out(argv[2]);
    out << std::setprecision(17) << "time_seconds,id,x,y,yaw\n";
    double maximum = 0;
    int exported = 0;
    for (const auto& item : dataset.GetData()) {
      auto* scan = dynamic_cast<karto::LocalizedRangeScan*>(item.second);
      if (!scan) continue;
      auto at = graph.find(scan->GetUniqueId());
      if (at == graph.end())
        throw std::runtime_error("Dataset scan absent from graph");
      const auto pose = scan->GetSensorPose();
      const auto other = at->second->GetSensorPose();
      if (!std::isfinite(pose.GetX()) || !std::isfinite(pose.GetY()) ||
          !std::isfinite(pose.GetHeading()) || !std::isfinite(scan->GetTime()))
        throw std::runtime_error("Nonfinite final graph pose or time");
      maximum = std::max(maximum, std::abs(pose.GetX() - other.GetX()));
      maximum = std::max(maximum, std::abs(pose.GetY() - other.GetY()));
      const double angle = pose.GetHeading() - other.GetHeading();
      maximum = std::max(maximum, std::abs(std::atan2(std::sin(angle), std::cos(angle))));
      if (scan->GetTime() != at->second->GetTime())
        throw std::runtime_error("Dataset/graph time mismatch");
      out << scan->GetTime() << ',' << scan->GetUniqueId() << ','
          << pose.GetX() << ',' << pose.GetY() << ',' << pose.GetHeading() << '\n';
      ++exported;
    }
    out.close();
    if (!out) throw std::runtime_error("Cannot write final graph CSV");
    std::ofstream stats(argv[3]);
    stats << "exported " << exported << "\ngraph_nodes " << graph.size()
          << "\ngraph_edges " << mapper.GetGraph()->GetEdges().size()
          << "\ndataset_graph_pose_max " << std::setprecision(17) << maximum << '\n';
    stats.close();
    if (!stats) throw std::runtime_error("Cannot write graph statistics");
    if (maximum > 1e-9 || exported != int(graph.size()))
      throw std::runtime_error("Inconsistent final poses");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
