/*******************************************************************************
File: state_srdf_to_moveit.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#include "my_app/states/state_srdf_to_moveit.h"

#define DEBUG_LEVEL       DEBUG_LEVEL_1//DEBUG_LEVEL_NONE//DEBUG_LEVEL_1

state_srdf_to_moveit::state_srdf_to_moveit(const std::string& state_object_name){//, const std::string& group/* define own paramters here*/){

  this->state_object_name = state_object_name;
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::construcor\n", state_object_name.c_str());

  move_group = new moveit::planning_interface::MoveGroupInterface("arm");
  move_group_state = move_group->getCurrentState(1.0);

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::construcor\n", state_object_name.c_str());
}

state_srdf_to_moveit::~state_srdf_to_moveit(){
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::destrucor\n", state_object_name.c_str());

    /* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::destrucor\n", state_object_name.c_str());
}

state_srdf_to_moveit::status state_srdf_to_moveit::onEnter(input_keys_& input_keys){

  state_srdf_to_moveit::status return_code = success;
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onEnter\n", state_object_name.c_str());

  user_data.input_keys = input_keys;


  // positie 1
    std::map<std::string, double> target = move_group->getNamedTargetValues(user_data.input_keys.config_name);

    move_group->setJointValueTarget(target);


#if 0
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

    move_group->setPathConstraints(constraints);
#endif

    move_group->setMaxVelocityScalingFactor(0.5);
    move_group->setMaxAccelerationScalingFactor(0.5);
    move_group->setPlanningTime(10.0);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success)
    {
      //move_group->execute(my_plan);
      move_group->asyncExecute(my_plan);
    }
    else{
      ROS_ERROR("Unable to start move_group");
    }

  state_ = state_srdf_to_moveit::running;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::onEnter\n", state_object_name.c_str());
  return(return_code);
}


state_srdf_to_moveit::outcomes state_srdf_to_moveit::execute(void){

  state_srdf_to_moveit::outcomes return_value = busy;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::execute\n", state_object_name.c_str());

  // nog timeout inbouwen
  {
    // Dit is nog niet de netste methode
    static int execute_status;
		moveit_msgs::ExecuteTrajectoryActionResult::ConstPtr msg = ros::topic::waitForMessage<moveit_msgs::ExecuteTrajectoryActionResult>("/execute_trajectory/result", ros::Duration(0.5));
	  if (msg == NULL){
				//ROS_INFO("No ExecuteTrajectoryAction result received");
		}
	  else{
      execute_status=msg->status.status;
      //ROS_INFO("%i execute status",execute_status);
      if(execute_status==3){ // Wat als andere status?
        return_value = done;
      }
    }
	}


  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::execute\n", state_object_name.c_str());
  return(return_value);
}


/* do not modify this member function */
state_srdf_to_moveit::outcomes state_srdf_to_moveit::simpleEexecute(input_keys_& input_keys, output_keys_& output_keys){
  outcomes return_value = busy;

  switch(execution_state_){
    case execution_wait_for_start:
      {
        status on_enter_status_ = onEnter(input_keys);
        if(on_enter_status_ != success){
          return_value = failed;
          break;
        }
        execution_state_ = execution_execute;
      }
      break;
    case execution_execute:
      execution_return_value = execute();
      if(execution_return_value != busy){
        execution_state_ = execution_exit;
      }
      break;
    case execution_exit:
      output_keys = onExit();
      return_value = execution_return_value;
      execution_state_ = execution_wait_for_start;
      break;
    default:
      break;

  }
  return(return_value);
}


state_srdf_to_moveit::output_keys_ state_srdf_to_moveit::onExit(){

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onExit\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::onExit\n", state_object_name.c_str());
  return(user_data.output_keys);
}

state_srdf_to_moveit::status state_srdf_to_moveit::onStop(){

  state_srdf_to_moveit::status return_code = success;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onStop\n", state_object_name.c_str());

    /* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::onStop\n", state_object_name.c_str());
  return(return_code);
}

state_srdf_to_moveit::status state_srdf_to_moveit::onPause(){

  state_srdf_to_moveit::status return_code = success;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onPause\n", state_object_name.c_str());

  /* Write here your code */

  state_ = state_srdf_to_moveit::paused;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::onPause\n", state_object_name.c_str());
  return(return_code);
}

state_srdf_to_moveit::status state_srdf_to_moveit::onResume(){

  state_srdf_to_moveit::status return_code = success;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onResume\n", state_object_name.c_str());

  /* Write here your code */

  state_ = state_srdf_to_moveit::running;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::onResume\n", state_object_name.c_str());
  return(return_code);
}

state_srdf_to_moveit::state state_srdf_to_moveit::getState(void){
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::getState\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::getState\n", state_object_name.c_str());
  return(state_);
}
