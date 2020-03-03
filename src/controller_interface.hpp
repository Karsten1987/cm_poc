#pragma once

#include <memory>
#include <string>
#include <unordered_map>

using StateMap = std::unordered_map<std::string, double>;

static constexpr char const * const pos_key = "pos";
static constexpr char const * const vel_key = "vel";
static constexpr char const * const eff_key = "eff";
static constexpr char const * const pos_cmd_key = "pos_cmd";
static constexpr char const * const vel_cmd_key = "vel_cmd";
static constexpr char const * const torque_cmd_key = "torque_cmd";

namespace details
{
inline void throw_on_missing_key(
  const std::shared_ptr<StateMap> state_map, const std::vector<std::string> & keys)
{
  std::string err_msg = "missing key in state map: ";
  for (auto & key : keys) {
    if (state_map->find(key) == state_map->end()) {
      throw std::runtime_error(err_msg + key);
    }
  }
}
}  // namespace details

class ControllerInterface
{
public:
  virtual ~ControllerInterface() = default;

  // don't differentiate between input/output because.
  // in the serial alignment the output of one controller becomes the input of the next one.
  // having all set in one big k-v map makes things easier and less ambigious.
  virtual void attach_state_map(std::shared_ptr<StateMap> state_map)
  {
    if (NULL == state_map) {
      throw std::runtime_error("state map is null");
    }

    state_map_ = state_map;
    details::throw_on_missing_key(state_map_, get_input_keys());
    details::throw_on_missing_key(state_map_, get_output_keys());
  }

  // virtual function to force users to implement the used keys
  // The flexible joint state map (statemap) has to be implemented in a way
  // to mark the input keys as read-only to not invalidate the states during runtime.
  virtual std::vector<std::string> get_input_keys() const = 0;

  virtual std::vector<std::string> get_output_keys() const = 0;

  virtual void update(double dt) = 0;

protected:
  std::shared_ptr<StateMap> state_map_;
};
