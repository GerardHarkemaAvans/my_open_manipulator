/*******************************************************************************
File: state_ik_get_joints_from_pose.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#include "my_app/states/state_ik_get_joints_from_pose.h"

#define DEBUG_LEVEL       DEBUG_LEVEL_2 //DEBUG_LEVEL_NONE

state_ik_get_joints_from_pose::state_ik_get_joints_from_pose(const std::string& state_object_name, const std::string& group/* define own paramters here*/)
: node_handle("")
{
  this->state_object_name = state_object_name;
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::construcor\n", state_object_name.c_str());

  /* Write here your code */
  ik_service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

  while (!ik_service_client.exists())
  {
    ROS_INFO("Waiting for service");
    ros::Duration(1.0).sleep();
  }

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::construcor\n", state_object_name.c_str());
}

state_ik_get_joints_from_pose::~state_ik_get_joints_from_pose(){
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::destructor\n", state_object_name.c_str());

    /* Write here your code */

    DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::destructor\n", state_object_name.c_str());
}


state_ik_get_joints_from_pose::status state_ik_get_joints_from_pose::onEnter(input_keys_& input_keys){

  state_ik_get_joints_from_pose::status return_code = success;
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onEnter\n", state_object_name.c_str());

  user_data.input_keys = input_keys;
  /* Write here your code */
  moveit_msgs::GetPositionIK::Request service_request;
  moveit_msgs::GetPositionIK::Response service_response;

  service_request.ik_request.group_name = "arm";
  /*
  service_request.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
  service_request.ik_request.pose_stamped.pose.position.x = 0.75;
  service_request.ik_request.pose_stamped.pose.position.y = 0.188;
  service_request.ik_request.pose_stamped.pose.position.z = 0.0;

  service_request.ik_request.pose_stamped.pose.orientation.x = 0.0;
  service_request.ik_request.pose_stamped.pose.orientation.y = 0.0;
  service_request.ik_request.pose_stamped.pose.orientation.z = 0.0;
  service_request.ik_request.pose_stamped.pose.orientation.w = 1.0;
  */
  /* Call the service */
  ik_service_client.call(service_request, service_response);
  ROS_INFO_STREAM(
      "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                 << service_response.error_code.val);



  state_ = state_ik_get_joints_from_pose::running;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::onEnter\n", state_object_name.c_str());
  return(return_code);
}


state_ik_get_joints_from_pose::outcomes state_ik_get_joints_from_pose::execute(void){

  state_ik_get_joints_from_pose::outcomes return_value = busy;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::execute\n", state_object_name.c_str());

  /* Write here your code */
    return_value = done;
    state_ = state_ik_get_joints_from_pose::idle;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::execute\n", state_object_name.c_str());
  return(return_value);
}

/* do not modify this member function */
state_ik_get_joints_from_pose::outcomes state_ik_get_joints_from_pose::simpleEexecute(input_keys_& input_keys, output_keys_& output_keys){
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


state_ik_get_joints_from_pose::output_keys_ state_ik_get_joints_from_pose::onExit(){

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onExit\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::onExit\n", state_object_name.c_str());
  return(user_data.output_keys);
}

state_ik_get_joints_from_pose::status state_ik_get_joints_from_pose::onStop(){

  state_ik_get_joints_from_pose::status return_code = success;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onStop\n", state_object_name.c_str());

    /* Write here your code */

    DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::onStop\n", state_object_name.c_str());
  return(return_code);
}

state_ik_get_joints_from_pose::status state_ik_get_joints_from_pose::onPause(){

  state_ik_get_joints_from_pose::status return_code = success;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onPause\n", state_object_name.c_str());

  /* Write here your code */

  state_ = state_ik_get_joints_from_pose::paused;


  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::onPause\n", state_object_name.c_str());
  return(return_code);
}

state_ik_get_joints_from_pose::status state_ik_get_joints_from_pose::onResume(){

  state_ik_get_joints_from_pose::status return_code = success;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onResume\n", state_object_name.c_str());

  /* Write here your code */

  state_ = state_ik_get_joints_from_pose::running;


  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::onResume\n", state_object_name.c_str());
  return(return_code);
}

state_ik_get_joints_from_pose::state state_ik_get_joints_from_pose::getState(void){

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::getState\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::getState\n", state_object_name.c_str());
  return(state_);
}
