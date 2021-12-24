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

#define DEBUG_LEVEL       DEBUG_LEVEL_NONE//DEBUG_LEVEL_2


behavior::behavior(const std::string& behavior_object_name)
: node_handle(""),
  priv_node_handle("~")
{

  this->behavior_object_name = behavior_object_name;
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::construcor\n", behavior_object_name.c_str());

  /* Write here your code */
  srdf_to_moveit = new state_srdf_to_moveit("srdf_to_moveit", "arm");
  move_joints = new state_move_joints("move_joints", "arm");
  get_tf_transform = new state_get_tf_transform("get_tf_transform");
  ik_get_joints_from_pose = new state_ik_get_joints_from_pose("ik_get_joints_from_pose", "arm");

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
        state_srdf_to_moveit::output_keys_ output_key;
        ROS_INFO("Executing go_home");
        switch(srdf_to_moveit->simpleEexecute(input_key, output_key)){
          case state_srdf_to_moveit::busy:
            /* Do nothing */
            break;
          case state_srdf_to_moveit::done:
            _state = go_left;
            break;
          case state_srdf_to_moveit::failed:
            _state = state_failed;
            break;
        }
      }
      break;
    case go_left:
      {
        state_srdf_to_moveit::input_keys_ input_key;
        input_key.config_name = "left";
        state_srdf_to_moveit::output_keys_ output_key;
        ROS_INFO("Executing go_left");
        switch(srdf_to_moveit->simpleEexecute(input_key, output_key)){
          case state_srdf_to_moveit::busy:
            /* Do nothing */
            break;
          case state_srdf_to_moveit::done:
            _state = get_transform;
            break;
          case state_srdf_to_moveit::failed:
            _state = state_failed;
            break;
        }
      }
    break;
    case get_transform:
      {
        state_get_tf_transform::input_keys_ input_key;
        input_key.target_frame = "world";
        input_key.source_frame = "ik_testpoint";
        state_get_tf_transform::output_keys_ output_key;
        ROS_INFO("Executing get transform");
        switch(get_tf_transform->simpleEexecute(input_key, output_key)){
          case state_get_tf_transform::busy:
            /* Do nothing */
            break;
          case state_get_tf_transform::done:
            object_pose = output_key.transform;
#if 0
            ROS_INFO("x = %f", object_pose.pose.position.x);
            ROS_INFO("y = %f", output_key.transform.pose.position.y);
            ROS_INFO("z = %f", output_key.transform.pose.position.z);
            ROS_INFO("header = %s", output_key.transform.header.frame_id.c_str());
#endif
            _state = ik_calculate_joits;
            break;
          case state_get_tf_transform::failed:
            _state = state_failed;
            break;
        }
      }
      break;
    case ik_calculate_joits:
      {
        state_ik_get_joints_from_pose::input_keys_ input_key;
        input_key.pose = object_pose;
        input_key.tool_link = "end_effector_link";
        input_key.group_name = "arm";
        input_key.offset = 0.01;
        input_key.rotation = 1.57;
        state_ik_get_joints_from_pose::output_keys_ output_key;
        ROS_INFO("Executing get transform");
        switch(ik_get_joints_from_pose->simpleEexecute(input_key, output_key)){
          case state_ik_get_joints_from_pose::busy:
            /* Do nothing */
            break;
          case state_ik_get_joints_from_pose::done:
            object_pose_joints = output_key.joints;
            _state = go_pose;
            break;
          case state_ik_get_joints_from_pose::failed:
            _state = state_failed;
            break;
        }
      }
      break;
    case go_pose:
      {
        state_move_joints::input_keys_ input_key;
        input_key.joints = object_pose_joints;
        state_move_joints::output_keys_ output_key;
        ROS_INFO("Executing go_pose");
        switch(move_joints->simpleEexecute(input_key, output_key)){
          case state_move_joints::busy:
            /* Do nothing */
            break;
          case state_move_joints::done:
            _state = go_right;
            break;
          case state_move_joints::failed:
            _state = state_failed;
            break;
        }
      }
      break;
    case go_right:
      {
        state_srdf_to_moveit::input_keys_ input_key;
        input_key.config_name = "right";
        state_srdf_to_moveit::output_keys_ output_key;
        ROS_INFO("Executing go_right");
        switch(srdf_to_moveit->simpleEexecute(input_key, output_key)){
          case state_srdf_to_moveit::busy:
            /* Do nothing */
          break;
          case state_srdf_to_moveit::done:
          _state = go_resting;
          break;
          case state_srdf_to_moveit::failed:
            _state = state_failed;
          break;
        }
      }
    break;
    case go_resting:
      {
        state_srdf_to_moveit::input_keys_ input_key;
        input_key.config_name = "resting";
        state_srdf_to_moveit::output_keys_ output_key;
        ROS_INFO("Executing go_resting");
        switch(srdf_to_moveit->simpleEexecute(input_key, output_key)){
          case state_srdf_to_moveit::busy:
            /* Do nothing */
          break;
          case state_srdf_to_moveit::done:
            _state = state_finshed;
            break;
          case state_srdf_to_moveit::failed:
            _state = state_failed;
            break;
        }
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
