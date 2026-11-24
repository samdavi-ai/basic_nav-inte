// Private recovery planner. NavFn searches an owned map of feasible centers;
// the controller and guard still validate the oriented padded footprint.
#include <algorithm>
#include "nav2_costmap_2d/footprint.hpp"
#include <cmath>
#include <mutex>
#include <vector>
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vira_validation {
class ClearanceNavfn : public nav2_navfn_planner::NavfnPlanner {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> grid) override {
    NavfnPlanner::configure(parent,name,tf,grid);
    const auto polygon=grid->getRobotFootprint();
    const auto [inner,outer] = nav2_costmap_2d::calculateMinAndMaxDistances(polygon);
    clearance_=name=="TurningClearance" ? outer : inner;
    if (!(clearance_>0 && std::isfinite(clearance_))) throw std::runtime_error("Invalid padded footprint");
  }

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
      const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancelled) override {
    std::lock_guard<std::mutex> serial(serial_);
    auto * live = costmap_;
    nav2_costmap_2d::Costmap2D snapshot;
    {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live->getMutex());
      snapshot = *live;
    }
    const int width = snapshot.getSizeInCellsX(), height = snapshot.getSizeInCellsY();
    const double resolution = snapshot.getResolution();
    // Any orientation of the padded rectangle contains a disk of radius 0.35 m.
    // Reject a center when that disk intersects an occupied cell square.
    const double radius = clearance_;
    const int reach = static_cast<int>(std::ceil(radius / resolution + 0.5));
    std::vector<std::pair<int,int>> offsets;
    for (int y=-reach; y<=reach; ++y) {
      for (int x=-reach; x<=reach; ++x) {
        const double dx=std::max(0., std::abs(x)-0.5)*resolution;
        const double dy=std::max(0., std::abs(y)-0.5)*resolution;
        if (std::hypot(dx,dy)<=radius) offsets.emplace_back(x,y);
      }
    }
    const auto * cells = snapshot.getCharMap();
    std::vector<unsigned char> original(cells,cells+width*height);
    for (int y=0; y<height; ++y) {
      if (cancelled()) throw std::runtime_error("Recovery planning cancelled");
      for (int x=0; x<width; ++x) {
        if (original[y*width+x]<254) continue;
        for (auto [dx,dy] : offsets) {
          const int xx=x+dx, yy=y+dy;
          if (xx>=0 && xx<width && yy>=0 && yy<height) snapshot.setCost(xx,yy,254);
        }
      }
    }
    struct Restore {
      nav2_costmap_2d::Costmap2D *& pointer;
      nav2_costmap_2d::Costmap2D * live;
      ~Restore() { pointer=live; }
    } restore{costmap_,live};
    costmap_=&snapshot;
    return NavfnPlanner::createPlan(start,goal,cancelled);
  }
private:
  double clearance_ = 0.;
  std::mutex serial_;
};
}
PLUGINLIB_EXPORT_CLASS(vira_validation::ClearanceNavfn,nav2_core::GlobalPlanner)
