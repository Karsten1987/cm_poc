#pragma once

#include <cmath>
#include <string>
#include <unordered_map>

#include "./controller_interface.hpp"

class DuplicateController : public ControllerInterface
{
public:
  std::vector<std::string> get_input_keys() const override
  {
    return {pos_key};
  }

  std::vector<std::string> get_output_keys() const override
  {
    return {pos_cmd_key};
  }

  void update(double dt) override
  {
    (void) dt;

    auto curr_pos = state_map_->at(pos_key);
    state_map_->at(pos_cmd_key) = 2 * curr_pos;
  }
};

class BisectController : public ControllerInterface
{
public:
  std::vector<std::string> get_input_keys() const override
  {
    return {pos_key};
  }

  std::vector<std::string> get_output_keys() const override
  {
    return {pos_cmd_key};
  }

  void update(double dt) override
  {
    (void) dt;

    auto curr_pos = state_map_->at(pos_key);
    state_map_->at(pos_cmd_key) = 0.5 * curr_pos;
  }
};

class FloorController : public ControllerInterface
{
public:
  std::vector<std::string> get_input_keys() const override
  {
    // Note that this input key is an output key of another controller
    return {pos_cmd_key};
  }

  std::vector<std::string> get_output_keys() const override
  {
    return {pos_cmd_key};
  }

  void update(double dt) override
  {
    (void) dt;

    auto curr_pos = state_map_->at(pos_key);
    state_map_->at(pos_cmd_key) = floor(curr_pos);
  }
};

class CeilController : public ControllerInterface
{
public:
  std::vector<std::string> get_input_keys() const override
  {
    // Note that this input key is an output key of another controller
    return {pos_cmd_key};
  }

  std::vector<std::string> get_output_keys() const override
  {
    return {pos_cmd_key};
  }

  void update(double dt) override
  {
    (void) dt;

    auto curr_pos = state_map_->at(pos_key);
    state_map_->at(pos_cmd_key) = ceil(curr_pos);
  }
};
