// Test scheduling hook around the installed Jazzy NavFn implementation.
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "nav2_navfn_planner/navfn_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace vira_validation
{
class SynchronizedNavfn : public nav2_navfn_planner::NavfnPlanner
{
public:
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancelled) override
  {
    const char * dir = std::getenv("VIRA_FIXTURE_BARRIER");
    const char * partition = std::getenv("GZ_PARTITION");
    if (!dir || !partition || std::string(partition).find("vira-dynamic-") != 0) {
      throw std::runtime_error("Test planner needs its private Gazebo fixture directory");
    }
    if (++calls_ == 2) {
      const std::filesystem::path folder(dir);
      // Persist both ends, including cancellation, so an old marker cannot
      // qualify an insertion during a later invocation.
      struct ExitMarker {
        std::filesystem::path path;
        ~ExitMarker() {
          std::ofstream(path) << std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        }
      } exit_marker{folder / "replan.exited"};
      std::ofstream(folder / "replan.entered") << "Second createPlan entered\n";
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
      while (!std::filesystem::exists(folder / "fixture.confirmed")) {
        if (cancelled() || std::chrono::steady_clock::now() > deadline) {
          throw std::runtime_error("Timed out waiting for the Gazebo fixture");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      return NavfnPlanner::createPlan(start, goal, cancelled);
    }
    return NavfnPlanner::createPlan(start, goal, cancelled);
  }

private:
  unsigned int calls_{0};
};
}
PLUGINLIB_EXPORT_CLASS(vira_validation::SynchronizedNavfn, nav2_core::GlobalPlanner)
