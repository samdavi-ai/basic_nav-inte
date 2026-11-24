// Hard geometric veto for the simulation candidate. Valid trajectories retain
// their original DWB score. Use the same conservative cell and footprint bounds
// as the downstream validator, so DWB can select a different sample itself.
#include <algorithm>
#include <cmath>
#include <vector>

#include "dwb_core/exceptions.hpp"
#include "dwb_core/trajectory_critic.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vira_validation
{
class SweptFootprintCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    const auto node = node_.lock();
    const auto param = dwb_plugin_name_ + "." + name_ + ".planning_padding";
    if (!node->has_parameter(param)) { node->declare_parameter(param, 0.0); }
    const double padding = node->get_parameter(param).as_double();
    if (!std::isfinite(padding) || padding < 0.) {
      throw std::invalid_argument("Invalid braking footprint padding");
    }
    brake_margin_ = std::sqrt(2.) * padding;
  }

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override
  {
    map_ = costmap_ros_->getCostmap();
    pose_ = pose;
    measured_ = velocity;
    half_ = hx_ = hy_ = 0;
    for (const auto & p : costmap_ros_->getRobotFootprint()) {
      hx_ = std::max(hx_,std::abs(p.x));
      hy_ = std::max(hy_,std::abs(p.y));
      half_ = std::max({half_, std::abs(p.x), std::abs(p.y)});
    }
    width_ = map_->getSizeInCellsX();
    height_ = map_->getSizeInCellsY();
    res_ = map_->getResolution();
    // A summed-area table skips free rectangles during scoring. DWB holds the
    // costmap mutex across its evaluation cycle.
    prefix_.assign((width_ + 1) * (height_ + 1), 0);
    for (unsigned y = 0; y < height_; ++y) {
      unsigned row = 0;
      for (unsigned x = 0; x < width_; ++x) {
        row += map_->getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE;
        prefix_[(y+1)*(width_+1)+x+1] = prefix_[y*(width_+1)+x+1] + row;
      }
    }
    return half_ > 0 && res_ > 0;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (half_ <= 0 || res_ <= 0 || trajectory.poses.empty()) {
      throw dwb_core::IllegalTrajectoryException(name_, "Missing footprint, grid or trajectory");
    }
    auto previous = pose_;
    for (const auto & p : trajectory.poses) {
      const double dx = p.x-previous.x;
      const double dy = p.y-previous.y;
      const double dh = std::remainder(p.theta-previous.theta, 2*M_PI);
      const double span = std::hypot(dx, dy) + std::sqrt(2.)*half_*std::abs(dh);
      const int steps = std::max(1, static_cast<int>(std::ceil(span/(res_/2))));
      for (int i = 0; i <= steps; ++i) {
        const double f = static_cast<double>(i)/steps;
        check(previous.x+f*dx, previous.y+f*dy, previous.theta+f*dh, res_/4);
      }
      previous = p;
    }
    // The guard may see a local map up to one publication period older.
    // Validate the reaction sweep for both axes' measured and sampled speeds.
    for (double v : {measured_.x, trajectory.velocity.x}) {
      for (double w : {measured_.theta, trajectory.velocity.theta}) {
        checkBraking(v,w);
      }
    }
    return 0.0;
  }

protected:
  virtual bool rectangular() const {return false;}

private:
  void checkBraking(double v, double w) const
  {
    double x=pose_.x, y=pose_.y, h=pose_.theta;
    constexpr double step=.05;
    constexpr double reaction=1.05;  // 2 Hz local map, one further publish period and one mux cycle
    const double duration=reaction+std::max(std::abs(v)/2.5,std::abs(w)/3.2);
    for (double t=0; t<=duration+step; t+=step) {
      check(x,y,h,brake_margin_+(std::abs(v)+std::sqrt(2.)*half_*std::abs(w))*step);
      x+=v*std::cos(h+w*step/2)*step;
      y+=v*std::sin(h+w*step/2)*step;
      h+=w*step;
      if (t>=reaction) {
        v=std::copysign(std::max(0.,std::abs(v)-2.5*step),v);
        w=std::copysign(std::max(0.,std::abs(w)-3.2*step),w);
      }
    }
  }

  void check(double x,double y,double h,double margin) const
  {
    const double c=std::cos(h), s=std::sin(h);
    const double limit=half_+res_/std::sqrt(2.)+margin;
    const double extent=(std::abs(c)+std::abs(s))*limit;
    unsigned left,bottom,right,top;
    const double ex=rectangular() ? std::abs(c)*hx_+std::abs(s)*hy_+margin : extent;
    const double ey=rectangular() ? std::abs(s)*hx_+std::abs(c)*hy_+margin : extent;
    if (!map_->worldToMap(x-ex,y-ey,left,bottom) ||
      !map_->worldToMap(x+ex,y+ey,right,top))
    {
      throw dwb_core::IllegalTrajectoryException(name_, "Footprint outside observed grid");
    }
    // Include the neighboring cell when a footprint edge falls exactly on a
    // grid line. The narrow phase treats touching as occupied.
    if (rectangular()) {
      if (left>0) --left;
      if (bottom>0) --bottom;
      right=std::min(right+1,width_-1);
      top=std::min(top+1,height_-1);
    }
    const unsigned stride=width_+1;
    if (prefix_[(top+1)*stride+right+1]+prefix_[bottom*stride+left] ==
      prefix_[(top+1)*stride+left]+prefix_[bottom*stride+right+1])
    {
      return;
    }
    for (auto row=bottom; row<=top; ++row) {
      for (auto col=left; col<=right; ++col) {
        if (map_->getCost(col,row)<nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
        double cx,cy;
        map_->mapToWorld(col,row,cx,cy);
        const double dx=cx-x,dy=cy-y;
        const double bx=std::abs(c*dx+s*dy), by=std::abs(-s*dx+c*dy);
        const double cell=res_/2, projected=cell*(std::abs(c)+std::abs(s));
        const bool overlap=rectangular() ?
          (std::abs(dx)<=ex+cell+1e-12 && std::abs(dy)<=ey+cell+1e-12 &&
          bx<=hx_+projected+margin+1e-12 && by<=hy_+projected+margin+1e-12) :
          std::max(bx,by)<=limit;
        if (overlap) {
          throw dwb_core::IllegalTrajectoryException(name_, "Swept padded footprint intersects cell");
        }
      }
    }
  }

  nav2_costmap_2d::Costmap2D * map_{nullptr};
  geometry_msgs::msg::Pose2D pose_;
  nav_2d_msgs::msg::Twist2D measured_;
  std::vector<unsigned> prefix_;
  unsigned width_{0},height_{0};
  double brake_margin_{0};
  double half_{0},hx_{0},hy_{0},res_{0};
};
class RectangularSweepCritic : public SweptFootprintCritic {
protected:
  bool rectangular() const override {return true;}
};
}
PLUGINLIB_EXPORT_CLASS(vira_validation::RectangularSweepCritic, dwb_core::TrajectoryCritic)
PLUGINLIB_EXPORT_CLASS(vira_validation::SweptFootprintCritic, dwb_core::TrajectoryCritic)
