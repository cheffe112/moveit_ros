/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include "get_execution_state_service_capability.h"
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/move_group/capability_names.h>

move_group::MoveGroupGetExecutionStateService::MoveGroupGetExecutionStateService():
  MoveGroupCapability("GetExecutionStateService")
{
}

void move_group::MoveGroupGetExecutionStateService::initialize()
{
  get_execution_state_service_ = root_node_handle_.advertiseService(GET_EXECUTION_STATE_SERVICE_NAME, &MoveGroupGetExecutionStateService::getExecutionStateService, this);
}

bool move_group::MoveGroupGetExecutionStateService::getExecutionStateService(moveit_msgs::GetTrajectoryExecutionState::Request &req, moveit_msgs::GetTrajectoryExecutionState::Response &res)
{
  ROS_INFO("Received new get trajectory execution state service request...");
  if (!context_->trajectory_execution_manager_)
  {
    ROS_ERROR("Cannot determine execution state since ~allow_trajectory_execution was set to false");
    res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
    return true;
  }

  // \todo unwind trajectory before execution
  //    robot_trajectory::RobotTrajectory to_exec(planning_scene_monitor_->getRobotModel(), ;

  res.trajectory_execution_complete = context_->trajectory_execution_manager_->isExecutionComplete();

  moveit_controller_manager::ExecutionStatus::Value status = context_->trajectory_execution_manager_->getLastExecutionStatus();

  switch(status) {
  case moveit_controller_manager::ExecutionStatus::RUNNING:
      res.controller_execution_state.val = moveit_msgs::MoveItControllerExecutionState::RUNNING;
      break;
  case moveit_controller_manager::ExecutionStatus::SUCCEEDED:
      res.controller_execution_state.val = moveit_msgs::MoveItControllerExecutionState::SUCCEEDED;
      break;
  case moveit_controller_manager::ExecutionStatus::PREEMPTED:
      res.controller_execution_state.val = moveit_msgs::MoveItControllerExecutionState::PREEMPTED;
      break;
  case moveit_controller_manager::ExecutionStatus::TIMED_OUT:
      res.controller_execution_state.val = moveit_msgs::MoveItControllerExecutionState::TIMED_OUT;
      break;
  case moveit_controller_manager::ExecutionStatus::ABORTED:
      res.controller_execution_state.val = moveit_msgs::MoveItControllerExecutionState::ABORTED;
      break;
  case moveit_controller_manager::ExecutionStatus::FAILED:
      res.controller_execution_state.val = moveit_msgs::MoveItControllerExecutionState::FAILED;
      break;
  default:
      res.controller_execution_state.val = moveit_msgs::MoveItControllerExecutionState::UNKNOWN;
      break;
  }

  res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  return true;
}


#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupGetExecutionStateService, move_group::MoveGroupCapability)
