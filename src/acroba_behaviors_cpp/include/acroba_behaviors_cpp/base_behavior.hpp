/**
 * @file base_behavior.hpp
 * @brief Abstract interface for C++ reactive behaviors in the Acroba framework.
 *
 * C++ behaviors share the same contract as Python behaviors:
 *   - on_enter(): initialize when activated
 *   - on_tick(): execute one cycle, return status
 *   - on_exit(): clean up on completion or preemption
 *
 * C++ is used for behaviors requiring low-latency sensor processing
 * (e.g. direct LiDAR reading for reactive obstacle avoidance).
 */

#ifndef ACROBA_BEHAVIORS_CPP__BASE_BEHAVIOR_HPP_
#define ACROBA_BEHAVIORS_CPP__BASE_BEHAVIOR_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace acroba
{

/**
 * @brief Status codes matching BehaviorStatus.msg constants.
 */
enum class Status : uint8_t
{
  IDLE = 0,
  RUNNING = 1,
  SUCCEEDED = 2,
  FAILED = 3,
  PREEMPTED = 4,
};

/**
 * @brief Shared context passed to behaviors on each tick.
 */
struct BehaviorContext
{
  std::unordered_map<std::string, std::string> parameters;
  std::unordered_map<std::string, double> shared_data;
  double robot_x = 0.0;
  double robot_y = 0.0;
  double robot_theta = 0.0;
  std::vector<float> scan_ranges;
};

/**
 * @brief Abstract base class for all C++ Acroba behaviors.
 */
class BaseBehavior
{
public:
  explicit BaseBehavior(const std::string & name, const std::string & description = "")
  : name_(name), description_(description), status_(Status::IDLE)
  {
  }

  virtual ~BaseBehavior() = default;

  /** @brief Called once when the behavior is activated. */
  virtual void on_enter(BehaviorContext & context) = 0;

  /** @brief Called repeatedly while RUNNING. Must return a Status. */
  virtual Status on_tick(BehaviorContext & context) = 0;

  /** @brief Called once on completion or preemption. */
  virtual void on_exit(BehaviorContext & context) = 0;

  /** @brief Reset to IDLE for reuse. */
  void reset() { status_ = Status::IDLE; }

  const std::string & name() const { return name_; }
  const std::string & description() const { return description_; }
  Status status() const { return status_; }

protected:
  std::string name_;
  std::string description_;
  Status status_;
};

}  // namespace acroba

#endif  // ACROBA_BEHAVIORS_CPP__BASE_BEHAVIOR_HPP_
