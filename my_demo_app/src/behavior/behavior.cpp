/*******************************************************************************
File: behavior.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een behaivior definitie welke gebruikt kan worden
bij een statemachine.
*******************************************************************************/
#include "my_app/behavior/behavior.h"

#define DEBUG_LEVEL       DEBUG_LEVEL_NONE//DEBUG_LEVEL_2 //DEBUG_LEVEL_NONE


behavior::behavior(const std::string& behavior_object_name)
: node_handle(""),
  priv_node_handle("~")
{

  this->behavior_object_name = behavior_object_name;
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::construcor\n", behavior_object_name.c_str());

  /* Write here your code */
  string tmp = "srdf_to_moveit";
  srdf_to_moveit = new state_srdf_to_moveit("arm");
  state_timer = node_handle.createTimer(ros::Duration(0.100)/*100ms*/,
                                          &behavior::stateCallback,
                                          this);

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::construcor\n", behavior_object_name.c_str());
}

behavior::~behavior()
{

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::destructor\n", behavior_object_name.c_str());

/* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::destructor\n", behavior_object_name.c_str());
}

void behavior::stateCallback(const ros::TimerEvent&){

  switch(_state){
    case state_idle:
      break;
    case state_start:
      {
        state_srdf_to_moveit::input_keys_ input_key;
        input_key.config_name = "home";
        if(srdf_to_moveit->onEnter(input_key)){_state = state_failed; break;}
        ROS_INFO("Executing go_home");
      }
      _state = go_home;
      break;
    case go_home:
      if(srdf_to_moveit->execute() != state_srdf_to_moveit::busy){
        srdf_to_moveit->onExit();
        {
          state_srdf_to_moveit::input_keys_ input_key;
          input_key.config_name = "left";
          if(srdf_to_moveit->onEnter(input_key)){_state = state_failed; break;}
          ROS_INFO("Executing go_left");
        }
        _state = go_left;
      }
      break;
    case go_left:
      if(srdf_to_moveit->execute() != state_srdf_to_moveit::busy){
        srdf_to_moveit->onExit();
        {
          state_srdf_to_moveit::input_keys_ input_key;
          input_key.config_name = "right";
          if(srdf_to_moveit->onEnter(input_key)){_state = state_failed; break;}
          ROS_INFO("Executing go_right");
        }
        _state = go_right;
      }
      break;
    case go_right:
      if(srdf_to_moveit->execute() != state_srdf_to_moveit::busy){
        srdf_to_moveit->onExit();
        {
          state_srdf_to_moveit::input_keys_ input_key;
          input_key.config_name = "resting";
          if(srdf_to_moveit->onEnter(input_key)){_state = state_failed; break;}
          ROS_INFO("Executing go_resting");
        }
        _state = go_resting;
      }
      break;
    case go_resting:
      if(srdf_to_moveit->execute() != state_srdf_to_moveit::busy){
        srdf_to_moveit->onExit();
        _state = state_finshed;
      }
      break;
    case state_failed:
      ROS_INFO("Ending with outcome failed");
      _outcomes = outcomes::status_failed;
      _state = state_wait_for_reset;
    break;
    case state_finshed:
      ROS_INFO("Ending with outcome finsihed");
      _outcomes = outcomes::status_finshed;
      _state = state_wait_for_reset;
      break;
    case state_wait_for_reset:
      break;
    default:
      break;
  }
}

void behavior::onEnter(input_keys_ &input_keys){

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onEnter\n", behavior_object_name.c_str());

  /* Write here your code */

  user_data.input_keys = input_keys;
  ROS_INFO("Starting");
  _state = state_start;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::onEnter\n", behavior_object_name.c_str());
}

behavior::outcomes behavior::execute(){

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::execute\n", behavior_object_name.c_str());

/* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::execute\n", behavior_object_name.c_str());
  return(_outcomes);
}

behavior::output_keys_ behavior::onExit(){

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onExit\n", behavior_object_name.c_str());

/* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::onExit\n", behavior_object_name.c_str());
  return(user_data.output_keys);
}

void behavior::abort(){

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::abort\n", behavior_object_name.c_str());

/* Write here your code */

_state = state_abort;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::abort\n", behavior_object_name.c_str());
}

void behavior::reset(){

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::reset\n", behavior_object_name.c_str());

  ROS_INFO("Resetiing");
  /* Write here your code */

  _state = state_idle;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Leaving %s::reset\n", behavior_object_name.c_str());
}
