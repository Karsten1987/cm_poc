#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <utility>

#include "./controller_interface.hpp"
#include "./pid_controllers.hpp"
#include "./simple_controllers.hpp"

// A ParallelSystem can be a controller itself
class ParallelSystem : public ControllerInterface
{
public:
  ParallelSystem(std::vector<std::unique_ptr<ControllerInterface>> && controllers)
  : controllers_(std::move(controllers))
  {
    controller_state_maps_.reserve(controllers.size());
    for (auto i = 0u; i < controllers_.size(); ++i) {
      controller_state_maps_.emplace_back(std::make_shared<StateMap>());
    }
  }

  void attach_state_map(std::shared_ptr<StateMap> state_map) override
  {
    // set your own state map
    ControllerInterface::attach_state_map(state_map);

    // update values from input state map
    populate_controller_state_maps();

    // attach all controllers with a copy of the system's state map
    for (auto i = 0u; i < controllers_.size(); ++i) {
      controllers_[i]->attach_state_map(controller_state_maps_[i]);
    }
  }

  void populate_controller_state_maps()
  {
    for (auto state_map : controller_state_maps_) {
      // explicit copy
      *state_map = *state_map_;
    }
  }

  void fuse_controller_state_maps()
  {
    // should take a function/lamda to modularize operation on how to fuse
    // using `+=` for demo purposes
    for (const auto & key : get_output_keys()) {
      auto output = 0.0;
      for (auto i = 0u; i < controllers_.size(); ++i) {
        output += controller_state_maps_[i]->at(key);
      }
      state_map_->at(key) = output;
    }
  }

  std::vector<std::string> get_input_keys() const override
  {
    std::vector<std::string> input_keys;
    for (const auto & controller : controllers_) {
      auto controller_keys = controller->get_input_keys();
      input_keys.insert(input_keys.end(), controller_keys.begin(), controller_keys.end());
    }
    return input_keys;
  }

  std::vector<std::string> get_output_keys() const override
  {
    std::vector<std::string> output_keys;
    for (const auto & controller : controllers_) {
      auto controller_keys = controller->get_output_keys();
      output_keys.insert(output_keys.end(), controller_keys.begin(), controller_keys.end());
    }
    return output_keys;
  }

  void update(double dt) override {
    populate_controller_state_maps();

    for (auto & controller : controllers_) {
      controller->update(dt);
    }

    fuse_controller_state_maps();
  }

private:
  // associate array between controllers and their statemaps
  std::vector<std::unique_ptr<ControllerInterface>> controllers_;
  std::vector<std::shared_ptr<StateMap>> controller_state_maps_;
};

// A SequentialSystem can be a controller itself
class SequentialSystem : public ControllerInterface
{
public:
  SequentialSystem(std::vector<std::unique_ptr<ControllerInterface>> && controllers)
  : controllers_(std::move(controllers)),
    // A SequentialSystem only has one state map which gets successively passed
    // to all controllers
    controller_state_map_(std::make_shared<StateMap>())
  {}

  void attach_state_map(std::shared_ptr<StateMap> state_map) override
  {
    // set your own state map
    ControllerInterface::attach_state_map(state_map);

    // update values from input state map
    populate_controller_state_maps();

    // attach all controllers with the same copy of the system's state map
    for (auto i = 0u; i < controllers_.size(); ++i) {
      controllers_[i]->attach_state_map(controller_state_map_);
    }
  }

  void populate_controller_state_maps()
  {
    // explicit copy
    *controller_state_map_ = *state_map_;
  }

  void fuse_controller_state_map()
  {
    // explicitly copy back
    *state_map_ = *controller_state_map_;
  }

  std::vector<std::string> get_input_keys() const override
  {
    std::vector<std::string> input_keys;
    for (const auto & controller : controllers_) {
      auto controller_keys = controller->get_input_keys();
      input_keys.insert(input_keys.end(), controller_keys.begin(), controller_keys.end());
    }
    return input_keys;
  }

  std::vector<std::string> get_output_keys() const override
  {
    std::vector<std::string> output_keys;
    for (const auto & controller : controllers_) {
      auto controller_keys = controller->get_output_keys();
      output_keys.insert(output_keys.end(), controller_keys.begin(), controller_keys.end());
    }
    return output_keys;
  }

  void update(double dt) override {
    populate_controller_state_maps();

    for (auto & controller : controllers_) {
      controller->update(dt);
    }

    fuse_controller_state_map();
  }

private:
  // associate array between controllers and their statemaps
  std::vector<std::unique_ptr<ControllerInterface>> controllers_;
  std::shared_ptr<StateMap> controller_state_map_;
};
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  std::vector<std::unique_ptr<ControllerInterface>> controllers;
  //controllers.emplace_back(std::make_unique<DuplicateController>());
  controllers.emplace_back(std::make_unique<BisectController>());
  controllers.emplace_back(std::make_unique<CeilController>());

  //ParallelSystem system(std::move(controllers));
  SequentialSystem system(std::move(controllers));

  auto robot_state_map = std::make_shared<StateMap>();
  robot_state_map->insert({pos_key, 1.0});
  robot_state_map->insert({pos_cmd_key, 0.0});
  system.attach_state_map(robot_state_map);

  for (auto i = 0u; i < 10; ++i) {
    fprintf(stdout, "Current position: %f\n", robot_state_map->at(pos_key));

    system.update(1.0);

    fprintf(stdout, "Current pos cmd: %f\n", robot_state_map->at(pos_cmd_key));

    // update the robot's state map
    robot_state_map->at(pos_key) = robot_state_map->at(pos_cmd_key);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}
