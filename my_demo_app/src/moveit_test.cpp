/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#if 0
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#endif

moveit::planning_interface::MoveGroupInterface* move_group;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  move_group = new moveit::planning_interface::MoveGroupInterface("arm");//group);

  std::map<std::string, double> target = move_group->getNamedTargetValues("home");

  moveit_msgs::Constraints constraints;
  std::map<std::string, double>::iterator it = target.begin();
  while(it != target.end())
  {
      moveit_msgs::JointConstraint joint_constraint;

      std::cout<<it->first<<" = "<<it->second<<std::endl;
      // Constrain the position of a joint to be within a certain bound
      joint_constraint.joint_name = it->first;

      // the bound to be achieved is [position - tolerance_below, position + tolerance_above]
      joint_constraint.position = it->second;
      joint_constraint.tolerance_above = 0.1;
      joint_constraint.tolerance_below = 0.1;

      // A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
      joint_constraint.weight = 1.0;

      constraints.joint_constraints.push_back(joint_constraint);

      it++;
  }
  move_group->setJointValueTarget(target);

  //move_group->setPathConstraints(constraints);

  //move_group->setMaxVelocityScalingFactor(0.5);
  //move_group->setMaxAccelerationScalingFactor(0.5);
  //move_group->setPlanningTime(10.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

#if 1
//  move_group->move();

  bool succes = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (succes)
    {
    move_group->execute(my_plan);
    //move_group->asyncExecute(my_plan);
    }
#endif



  ros::shutdown();
  return 0;
}
