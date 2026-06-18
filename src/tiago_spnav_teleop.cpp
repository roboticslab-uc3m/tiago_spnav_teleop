#include "tiago_spnav_teleop/tiago_spnav_teleop.hpp"

#include <algorithm> // std::copy
#include <string>
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <urdf/model.h>

using namespace spnav_controller;

constexpr auto UPDATE_LOG_THROTTLE = 1000.0; // [ms]

controller_interface::CallbackReturn SpnavController::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener = std::make_shared<ParamListener>(get_node());
    params = param_listener->get_params();
  }
  catch (const std::exception & e)
  {
    std::fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  const std::string robot_desc_string = get_robot_description();

  KDL::Tree tree;

  if (!kdl_parser::treeFromString(robot_desc_string, tree))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to construct KDL tree");
    return controller_interface::CallbackReturn::ERROR;
  }

  KDL::Chain chain;

  if (!tree.getChain(params.start_link, params.end_link, chain))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get chain from KDL tree");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Got chain with " << chain.getNrOfJoints() << " joints and " << chain.getNrOfSegments() << " segments");

  q.resize(chain.getNrOfJoints());

  ik_solver_vel = std::make_unique<KDL::ChainIkSolverVel_pinv>(chain, params.ik_solver_vel_eps, params.ik_solver_vel_max_iter);

  urdf::Model model;

  if (!model.initString(robot_desc_string))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse robot description");
    return controller_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint_name : params.arm_joint_names)
  {
    auto joint = model.getJoint(joint_name);
    arm_joint_limits.emplace_back(joint->limits->lower, joint->limits->upper);
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpnavController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  // update the dynamic map parameters
  param_listener->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params = param_listener->get_params();

  spnav_subscription = get_node()->create_subscription<sensor_msgs::msg::Joy>(
    "spacenav/joy", 1, std::bind(&SpnavController::spnavCallback, this, std::placeholders::_1));

  if (!spnav_subscription)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not subscribe to /spacenav/joy");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SpnavController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(params.arm_joint_names.size() + params.gripper_joint_names.size());

  for (const auto & joint_name : params.arm_joint_names)
  {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  for (const auto & joint_name : params.gripper_joint_names)
  {
    conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
  }

  return conf;
}

controller_interface::InterfaceConfiguration SpnavController::state_interface_configuration() const
{
  return command_interface_configuration();
}

controller_interface::CallbackReturn SpnavController::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpnavController::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpnavController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  std::string out = "Initial arm pose:";

  for (int i = 0; i < state_interfaces_.size(); i++)
  {
    const auto op = state_interfaces_[i].get_optional();

    if (op.has_value())
    {
      q(i) = op.value();
      out += " " + std::to_string(q(i));
    }
    else
    {
      RCLCPP_WARN_STREAM(get_node()->get_logger(), "State interface " << state_interfaces_[i].get_name() << " has no value");
      return controller_interface::CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO_STREAM(get_node()->get_logger(), out);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpnavController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SpnavController::on_error(const rclcpp_lifecycle::State & previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SpnavController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  KDL::JntArray qdot(params.arm_joint_names.size());
  KDL::Twist tw;

  {
    std::lock_guard lock(mtx);
    tw.vel = {joy_axes[0], joy_axes[1], joy_axes[2]};
    tw.rot = {joy_axes[3], joy_axes[4], joy_axes[5]};
  }

  if (!checkReturnCode(ik_solver_vel->CartToJnt(q, tw, qdot)))
  {
    return controller_interface::return_type::ERROR;
  }

  KDL::JntArray q_temp(q); // look ahead in case we may have ended up in a singular point at `q`

  for (int i = 0; i < params.arm_joint_names.size(); i++)
  {
    q_temp(i) += qdot(i) * period.seconds() * params.joy_arm_scale;

    if (q_temp(i) < arm_joint_limits[i].first || q_temp(i) > arm_joint_limits[i].second)
    {
      RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), UPDATE_LOG_THROTTLE,
      "Joint " << i << " out of limits: " << q_temp(i) <<
      " not in [" << arm_joint_limits[i].first << ", " << arm_joint_limits[i].second << "]");

      return controller_interface::return_type::ERROR;
    }
  }

  KDL::JntArray qdot_temp(params.arm_joint_names.size());

  if (!checkReturnCode(ik_solver_vel->CartToJnt(q_temp, tw, qdot_temp)))
  {
    return controller_interface::return_type::ERROR;
  }

  q = q_temp; // no singular point, so update the calculated pose

  for (int i = 0; i < params.arm_joint_names.size(); i++)
  {
    if (!command_interfaces_[i].set_value(q(i), 1))
    {
      RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), UPDATE_LOG_THROTTLE,
                                  "Failed to set command for joint " << state_interfaces_[i].get_name());
    }
  }

  if (joy_buttons[0])
  {
    for (int i = params.arm_joint_names.size(); i < params.gripper_joint_names.size(); i++)
    {
      auto value = state_interfaces_[i].get_optional().value() + params.joy_gripper_increment;

      if (!command_interfaces_[i].set_value(value, 1))
      {
        RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), UPDATE_LOG_THROTTLE,
                                    "Failed to set command for joint " << state_interfaces_[i].get_name());
      }
    }
  }
  else if (joy_buttons[1])
  {
    for (int i = params.arm_joint_names.size(); i < params.gripper_joint_names.size(); i++)
    {
      auto value = state_interfaces_[i].get_optional().value() - params.joy_gripper_increment;

      if (!command_interfaces_[i].set_value(value, 1))
      {
        RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), UPDATE_LOG_THROTTLE,
                                    "Failed to set command for joint " << state_interfaces_[i].get_name());
      }
    }
  }

  return controller_interface::return_type::OK;
}

bool SpnavController::checkReturnCode(int ret)
{
  switch (ret)
  {
    case KDL::ChainIkSolverVel_pinv::E_CONVERGE_PINV_SINGULAR:
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), UPDATE_LOG_THROTTLE, "Convergence issue: pseudo-inverse is singular");
      return false;
    case KDL::SolverI::E_SVD_FAILED:
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), UPDATE_LOG_THROTTLE, "Convergence issue: SVD failed");
      return false;
    case KDL::SolverI::E_NOERROR:
      return true;
    default:
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), UPDATE_LOG_THROTTLE, "Convergence issue: unknown error");
      return false;
  }
}

void SpnavController::spnavCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  std::lock_guard lock(mtx);
  std::copy(msg->axes.cbegin(), msg->axes.cend(), joy_axes.begin());
  std::copy(msg->buttons.cbegin(), msg->buttons.cend(), joy_buttons.begin());
}

PLUGINLIB_EXPORT_CLASS(spnav_controller::SpnavController, controller_interface::ControllerInterface);
