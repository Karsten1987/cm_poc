#pragma once

#include <string>
#include <unordered_map>

#include "./controller_interface.hpp"

class PController : public ControllerInterface
{
public:
  PController(double p_gain, double goal_pos)
  : p_gain_(p_gain),
    goal_pos_(goal_pos)
  {}

  std::vector<std::string> get_input_keys() const override
  {
    return {pos_key};
  }

  std::vector<std::string> get_output_keys() const override
  {
    return {torque_cmd_key};
  }

  void update(double dt) override
  {
    (void) dt;

    auto curr_pos = state_map_->at(pos_key);
    auto err = goal_pos_ - curr_pos;
    state_map_->at(torque_cmd_key) = p_gain_ * err;
  }

private:
  double p_gain_;
  double goal_pos_;
};

class DController : public ControllerInterface
{
public:
  DController(double d_gain, double goal_pos)
  : d_gain_(d_gain),
    goal_pos_(goal_pos),
    prev_err_(0.0)
  {}

  std::vector<std::string> get_input_keys() const override
  {
    return {pos_key};
  }

  std::vector<std::string> get_output_keys() const override
  {
    return {torque_cmd_key};
  }

  void update(double dt) override
  {
    auto curr_pos = state_map_->at(pos_key);
    auto err = goal_pos_ - curr_pos;
    auto err_dot = (prev_err_ - err) / dt;
    state_map_->at(torque_cmd_key) = d_gain_ * err_dot;
    prev_err_ = err;
  }

private:
  double d_gain_;
  double goal_pos_;

  double prev_err_;
};

class IController : public ControllerInterface
{
public:
  IController(double i_gain, double goal_pos)
  : i_gain_(i_gain),
    goal_pos_(goal_pos),
    acc_err_(0.0)
  {}

  std::vector<std::string> get_input_keys() const override
  {
    return {pos_key};
  }

  std::vector<std::string> get_output_keys() const override
  {
    return {torque_cmd_key};
  }

  void update(double dt) override
  {
    (void) dt;

    auto curr_pos = state_map_->at(pos_key);
    auto err = goal_pos_ - curr_pos;
    acc_err_ += err;
    state_map_->at(torque_cmd_key) = i_gain_ * acc_err_;
  }

private:
  double i_gain_;
  double goal_pos_;

  double acc_err_;
};
