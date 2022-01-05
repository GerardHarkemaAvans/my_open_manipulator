/*******************************************************************************
File: state_move_joints.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#include "../include/state_move_joints.h"

#define DEBUG_ITEMS       DEBUG_NONE //| DEBUG_STATES | DEBUG_CUSTOM

state_move_joints::state_move_joints(const std::string& state_object_name, const std::string& group/* define own paramters here*/)
: node_handle("")
{
  this->state_object_name = state_object_name;
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::construcor\n", state_object_name.c_str());

  move_group = new moveit::planning_interface::MoveGroupInterface(group);
  move_group_state = move_group->getCurrentState(1.0);

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Leaving %s::construcor\n", state_object_name.c_str());
}

state_move_joints::~state_move_joints(){
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::destrucor\n", state_object_name.c_str());

    /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Leaving %s::destrucor\n", state_object_name.c_str());
}

state_move_joints::status_enum state_move_joints::onEnter(input_keys_type& input_keys){

  state_move_joints::status_enum return_code = status_succes;
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onEnter\n", state_object_name.c_str());

  user_data.input_keys = input_keys;

  std::map<std::string, double> target = user_data.input_keys.joints;

  #if(DEBUG_ITEMS & DEBUG_CUSTOM)
    std::map<std::string, double>::iterator it = target.begin();
    while(it != target.end())
    {
        std::cout << it->first << " :: " << it->second << std::endl;
        it++;
    }
  #endif

  move_group->setJointValueTarget(user_data.input_keys.joints);


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

    bool succes = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (succes)
    {
      //move_group->execute(my_plan);
      move_group->asyncExecute(my_plan);
    }
    else{
      ROS_ERROR("Unable to start move_group");
    }

  state = state_move_joints::running;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Leaving %s::onEnter\n", state_object_name.c_str());
  return(return_code);
}


state_move_joints::outcomes_enum state_move_joints::execute(void){

  state_move_joints::outcomes_enum return_value = outcomes_busy;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::execute\n", state_object_name.c_str());

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
      //ROS_INFO("%i execute status_enum",execute_status);
      if(execute_status==3){ // Wat als andere status_enum?
        return_value = outcomes_done;
      }
    }
	}


  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Leaving %s::execute\n", state_object_name.c_str());
  return(return_value);
}


/* do not modify this member function */
state_move_joints::outcomes_enum state_move_joints::simpleEexecute(input_keys_type& input_keys, output_keys_type& output_keys){
  outcomes_enum return_value = outcomes_busy;

  switch(execution_state_){
    case execution_wait_for_start:
      {
        status_enum on_enter_status_ = onEnter(input_keys);
        if(on_enter_status_ != status_succes){
          return_value = outcomes_failed;
          break;
        }
        execution_state_ = execution_execute;
      }
      break;
    case execution_execute:
      execution_return_value = execute();
      if(execution_return_value != outcomes_busy){
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


state_move_joints::output_keys_type state_move_joints::onExit(){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onExit\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Leaving %s::onExit\n", state_object_name.c_str());
  return(user_data.output_keys);
}

state_move_joints::status_enum state_move_joints::onStop(){

  state_move_joints::status_enum return_code = status_succes;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onStop\n", state_object_name.c_str());

    /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Leaving %s::onStop\n", state_object_name.c_str());
  return(return_code);
}

state_move_joints::status_enum state_move_joints::onPause(){

  state_move_joints::status_enum return_code = status_succes;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onPause\n", state_object_name.c_str());

  /* Write here your code */

  state = state_move_joints::paused;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Leaving %s::onPause\n", state_object_name.c_str());
  return(return_code);
}

state_move_joints::status_enum state_move_joints::onResume(){

  state_move_joints::status_enum return_code = status_succes;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onResume\n", state_object_name.c_str());

  /* Write here your code */

  state = state_move_joints::running;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Leaving %s::onResume\n", state_object_name.c_str());
  return(return_code);
}

state_move_joints::state_enum state_move_joints::getState(void){
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::getState\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Leaving %s::getState\n", state_object_name.c_str());
  return(state);
}
