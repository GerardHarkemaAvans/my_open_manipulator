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

#define DEBUG_LEVEL_NONE  0 // Nu Debugging
#define DEBUG_LEVEL_1     1 // Own debug messages
#define DEBUG_LEVEL_2     2 // State debug messages
#define DEBUG_LEVEL_3     3 // All state debug messages (not implemented yet!)


#define DEBUG_LEVEL       DEBUG_LEVEL_2 //DEBUG_LEVEL_NONE

state_srdf_to_moveit::state_srdf_to_moveit(const std::string& state_object_name){//, const std::string& group/* define own paramters here*/){

  this->state_object_name = state_object_name;
#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::construcor" << endl;
#endif


//  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//  robot_model = robot_model_loader.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
//  robot_state = new robot_state::RobotState(robot_model);
//  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
//  joint_model_group = robot_state->getJointModelGroup(group);
  move_group = new moveit::planning_interface::MoveGroupInterface("arm");//group);

  move_group_state = move_group->getCurrentState(1.0);

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::construcor" << endl;
#endif
}

state_srdf_to_moveit::~state_srdf_to_moveit(){
#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::destrucor" << endl;
#endif

    /* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::destrucor" << endl;
#endif
}

state_srdf_to_moveit::status state_srdf_to_moveit::onEnter(input_keys_& input_keys){

  state_srdf_to_moveit::status return_code = success;
#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::onEnter" << endl;
#endif

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

  state_ = state_srdf_to_moveit::running;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::onEnter" << endl;
#endif
  return(return_code);
}


state_srdf_to_moveit::outcomes state_srdf_to_moveit::execute(void){

  state_srdf_to_moveit::outcomes return_value = busy;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::execute" << endl;
#endif

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


#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::execute" << endl;
#endif
  return(return_value);
}

state_srdf_to_moveit::output_keys_ state_srdf_to_moveit::onExit(){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::onExit" << endl;
#endif

  /* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::onExit" << endl;
#endif
  return(user_data.output_keys);
}

state_srdf_to_moveit::status state_srdf_to_moveit::onStop(){

  state_srdf_to_moveit::status return_code = success;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::onStop" << endl;
#endif

    /* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::onStop" << endl;
#endif
  return(return_code);
}

state_srdf_to_moveit::status state_srdf_to_moveit::onPause(){

  state_srdf_to_moveit::status return_code = success;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::onPause" << endl;
#endif

  /* Write here your code */

  state_ = state_srdf_to_moveit::paused;


#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::onPause" << endl;
#endif
  return(return_code);
}

state_srdf_to_moveit::status state_srdf_to_moveit::onResume(){

  state_srdf_to_moveit::status return_code = success;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::onResume" << endl;
#endif

  /* Write here your code */

  state_ = state_srdf_to_moveit::running;


#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::onResume" << endl;
#endif
  return(return_code);
}

state_srdf_to_moveit::state state_srdf_to_moveit::getState(void){
#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::getState" << endl;
#endif

  /* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::getState" << endl;
#endif
  return(state_);
}
