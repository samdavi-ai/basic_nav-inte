// Private simulation critic. A suggestion can only add a nonnegative cost.
#include <algorithm>
#include <cmath>
#include <mutex>
#include "dwb_core/trajectory_critic.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vira_validation
{
class PolicyPreferenceCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    auto node = node_.lock();
    clock_ = node->get_clock();
    max_v_ = node->get_parameter(dwb_plugin_name_ + ".max_vel_x").as_double();
    max_w_ = node->get_parameter(dwb_plugin_name_ + ".max_vel_theta").as_double();
    if (!(max_v_ > 0 && max_w_ > 0)) {
      throw std::invalid_argument("PolicyPreference requires positive controller limits");
    }
    subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/validation/policy_proposal", rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lock(mutex_);
        stamp_ = clock_->now().nanoseconds();
        proposed_v_ = m->linear.x;
        proposed_w_ = m->angular.z;
      });
  }

  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto age = clock_->now().nanoseconds() - stamp_;
    available_ = stamp_ > 0 && age >= 0 && age <= 150000000 &&
      std::isfinite(proposed_v_) && std::isfinite(proposed_w_) &&
      proposed_v_ >= 0 && proposed_v_ <= max_v_ && std::abs(proposed_w_) <= max_w_;
    v_ = proposed_v_;
    w_ = proposed_w_;
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (!available_) { return 0.; }
    const double dv = (trajectory.velocity.x-v_)/max_v_;
    const double dw = (trajectory.velocity.theta-w_)/(2*max_w_);
    return (dv*dv + dw*dw)/2;
  }

private:
  std::mutex mutex_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  int64_t stamp_{0};
  double max_v_{0}, max_w_{0}, proposed_v_{0}, proposed_w_{0}, v_{0}, w_{0};
  bool available_{false};
};
}
PLUGINLIB_EXPORT_CLASS(vira_validation::PolicyPreferenceCritic, dwb_core::TrajectoryCritic)
