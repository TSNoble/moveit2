/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sebastian Jahr
   Description: The hybrid planning manager component node that serves as the control unit of the whole architecture.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/local_planner.hpp>
#include <moveit_msgs/action/global_planner.hpp>
#include <moveit_msgs/action/hybrid_planner.hpp>

#include <moveit/hybrid_planning_manager/planner_logic_interface.hpp>

#include <pluginlib/class_loader.hpp>

namespace moveit::hybrid_planning
{
/**
 * Class HybridPlanningManager - ROS 2 component node that implements the hybrid planning manager.
 */
class HybridPlanningManager
{
public:
  /** \brief Constructor */
  HybridPlanningManager(const rclcpp::NodeOptions& options);

  /** \brief Destructor */
  ~HybridPlanningManager()
  {
    // Join the thread used for long-running callbacks
    if (long_callback_thread_.joinable())
    {
      long_callback_thread_.join();
    }
  }

  /**
   * Load and initialized planner logic plugin and ROS 2 action and topic interfaces
   * @return Initialization successful yes/no
   */
  bool initialize();

  // This function is required to make this class a valid NodeClass
  // see https://docs.ros2.org/latest/api/rclcpp_components/register__node__macro_8hpp.html
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()  // NOLINT
  {
    return node_->get_node_base_interface();  // NOLINT
  }

  /**
   * Cancel any active action goals, including global and local planners
   */
  void cancelHybridManagerGoals() noexcept;

  /**
   * This handles execution of a hybrid planning goal in a new thread, to avoid blocking the executor.
   * @param goal_handle The action server goal
   */
  void executeHybridPlannerGoal(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanner>> goal_handle);

  /**
   * Send global planning request to global planner component
   * @return Global planner successfully started yes/no
   */
  bool sendGlobalPlannerAction();

  /**
   * Send local planning request to local planner component
   * @return Local planner successfully started yes/no
   */
  bool sendLocalPlannerAction();

  /**
   * Send back hybrid planning response
   * @param success Indicates whether hybrid planning was successful
   */
  void sendHybridPlanningResponse(bool success);

  /**
   * @brief Process the action result and do an action call if necessary
   *
   * @param result Result to an event
   */
  void processReactionResult(const ReactionResult& result);

private:
  std::shared_ptr<rclcpp::Node> node_;

  // Planner logic plugin loader
  std::unique_ptr<pluginlib::ClassLoader<PlannerLogicInterface>> planner_logic_plugin_loader_;

  // Planner logic instance to implement reactive behavior
  std::shared_ptr<PlannerLogicInterface> planner_logic_instance_;

  // Flag that indicates hybrid planning has been canceled
  std::atomic<bool> stop_hybrid_planning_;

  // Shared hybrid planning goal handle
  std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::HybridPlanner>> hybrid_planning_goal_handle_;

  // Frequently updated feedback for the hybrid planning action requester
  std::shared_ptr<moveit_msgs::action::HybridPlanner_Feedback> hybrid_planning_progess_;

  // Planning request action clients
  rclcpp_action::Client<moveit_msgs::action::LocalPlanner>::SharedPtr local_planner_action_client_;
  rclcpp_action::Client<moveit_msgs::action::GlobalPlanner>::SharedPtr global_planner_action_client_;

  // Hybrid planning request action server
  rclcpp_action::Server<moveit_msgs::action::HybridPlanner>::SharedPtr hybrid_planning_request_server_;

  // Global solution subscriber
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_solution_sub_;

  // This thread is used for long-running callbacks. It's a member so they do not go out of scope.
  std::thread long_callback_thread_;

  // A unique callback group, to avoid mixing callbacks with other action servers
  rclcpp::CallbackGroup::SharedPtr cb_group_;
};
}  // namespace moveit::hybrid_planning
