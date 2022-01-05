/*******************************************************************************
File: behavior_go_pose_ik.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een behavior definitie welke gebruikt kan worden
bij een statemachine.
*******************************************************************************/
#include "../include/behavior_go_pose_ik.h"

#define DEBUG_ITEMS       DEBUG_NONE | DEBUG_BEHAVIORS_STATES//| DEBUG_BEHAVIORS | DEBUG_CUSTOM

behavior_go_pose_ik::behavior_go_pose_ik(const std::string& behavior_object_name, bool simple_execution_mode)
: node_handle(""),
  priv_node_handle("~")
{

  this->behavior_object_name = behavior_object_name;
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::construcor\n", behavior_object_name.c_str());
  this->simple_execution_mode = simple_execution_mode;

  /* Write here your code */
  move_joints = new state_move_joints("move_joints", "arm");
  get_tf_transform = new state_get_tf_transform("get_tf_transform");
  ik_get_joints_from_pose = new state_ik_get_joints_from_pose("ik_get_joints_from_pose", "arm");

  if(!simple_execution_mode)
    state_timer = node_handle.createTimer(ros::Duration(0.100)/*100ms*/,
                                            &behavior_go_pose_ik::stateCallback,
                                            this);
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::construcor\n", behavior_object_name.c_str());
}

behavior_go_pose_ik::~behavior_go_pose_ik()
{

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::destructor\n", behavior_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::destructor\n", behavior_object_name.c_str());
}

void behavior_go_pose_ik::stateHandler(){
  switch(state){
    case state_idle:
      break;
    case state_start:
      state = state_get_pose_transform;
      break;
    case state_get_pose_transform:
      {
        state_get_tf_transform::input_keys_type input_key;
        input_key.target_frame = user_data.input_keys.target_frame;
        input_key.source_frame = user_data.input_keys.source_frame;
        state_get_tf_transform::output_keys_type output_key;
        DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Executing state_get_pose_transform");
        switch(get_tf_transform->simpleEexecute(input_key, output_key)){
          case state_get_tf_transform::outcomes_busy:
            /* Do nothing */
            break;
          case state_get_tf_transform::outcomes_done:
            object_pose = output_key.transform;
#if 0
            DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "x = %f/n", object_pose.pose.position.x);
            DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "y = %f/n", output_key.transform.pose.position.y);
            DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "z = %f/n", output_key.transform.pose.position.z);
            DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "header = %s/n", output_key.transform.header.frame_id.c_str());
#endif
            state = state_ik_calculate_pose_joints;
            break;
          case state_get_tf_transform::outcomes_failed:
            state = state_failed;
            break;
        }
      }
      break;
    case state_ik_calculate_pose_joints:
      {
        state_ik_get_joints_from_pose::input_keys_type input_key;
        input_key.pose = object_pose;
        input_key.tool_link = "end_effector_link";
        input_key.group_name = "arm";
        input_key.offset = 0.01;
        input_key.rotation = 1.00;//1.57;
        state_ik_get_joints_from_pose::output_keys_type output_key;
        DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Executing state_ik_calculate_pose_joints");
        switch(ik_get_joints_from_pose->simpleEexecute(input_key, output_key)){
          case state_ik_get_joints_from_pose::outcomes_busy:
            /* Do nothing */
            break;
          case state_ik_get_joints_from_pose::outcomes_done:
            this->object_pose_joints = output_key.joints;
            state = state_go_pose;
            break;
          case state_ik_get_joints_from_pose::outcomes_failed:
            state = state_failed;
            break;
          default:
            break;
        }
      }
      break;
    case state_go_pose:
      {
        state_move_joints::input_keys_type input_key;
        input_key.joints = this->object_pose_joints;
        state_move_joints::output_keys_type output_key;
        DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Executing state_go_pose");
        switch(move_joints->simpleEexecute(input_key, output_key)){
          case state_move_joints::outcomes_busy:
            /* Do nothing */
            break;
          case state_move_joints::outcomes_done:
            state = state_finshed;
            break;
          case state_move_joints::outcomes_failed:
            state = state_failed;
            break;
        }
      }
      break;
    case state_failed:
      DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Ending with outcome outcomes_failed");
      outcomes = outcomes_enum::outcomes_failed;
      state = state_wait_for_reset;
      break;
    case state_finshed:
      DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Ending with outcome finsihed");
      outcomes = outcomes_enum::outcomes_finshed;
      state = state_wait_for_reset;
      break;
    case state_wait_for_reset:
      break;
    default:
      break;
  }
}

void behavior_go_pose_ik::stateCallback(const ros::TimerEvent&){
  if(!simple_execution_mode)
    behavior_go_pose_ik::stateHandler();
}

behavior_go_pose_ik::status_enum behavior_go_pose_ik::onEnter(input_keys_type &input_keys){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::onEnter\n", behavior_object_name.c_str());
  behavior_go_pose_ik::status_enum return_code = status_succes;

  /* Write here your code */

  user_data.input_keys = input_keys;
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Starting");
  state = state_start;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::onEnter\n", behavior_object_name.c_str());
  return(return_code);

}

behavior_go_pose_ik::outcomes_enum behavior_go_pose_ik::execute(){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::execute\n", behavior_object_name.c_str());

  /* Write here your code */
  if(simple_execution_mode) stateHandler();

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::execute\n", behavior_object_name.c_str());
  return(outcomes);
}

behavior_go_pose_ik::output_keys_type behavior_go_pose_ik::onExit(){
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::onExit\n", behavior_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::onExit\n", behavior_object_name.c_str());
  return(user_data.output_keys);
}

/* do not modify this member function */
behavior_go_pose_ik::outcomes_enum behavior_go_pose_ik::simpleEexecute(input_keys_type& input_keys, output_keys_type& output_keys){
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

#if 0 // not implemted yet
status_enum behavior_go_pose_ik::abort(){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::abort\n", behavior_object_name.c_str());

  /* Write here your code */

  state = state_abort;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::abort\n", behavior_object_name.c_str());
}

status_enum behavior_go_pose_ik::reset(){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::reset\n", behavior_object_name.c_str());

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s\n", "Resetiing");
  /* Write here your code */

  state = state_idle;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::reset\n", behavior_object_name.c_str());
}
#endif
