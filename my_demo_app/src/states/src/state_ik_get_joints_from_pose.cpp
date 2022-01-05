/*******************************************************************************
File: state_ik_get_joints_from_pose.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#include "../include/state_ik_get_joints_from_pose.h"

#define DEBUG_ITEMS       DEBUG_NONE //| DEBUG_CUSTOM | DEBUG_STATES | DEBUG_CUSTOM

state_ik_get_joints_from_pose::state_ik_get_joints_from_pose(const std::string& state_object_name, const std::string& group/* define own paramters here*/)
: node_handle("")
{
  this->state_object_name = state_object_name;
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::construcor\n", state_object_name.c_str());

  /* Write here your code */
  ik_service_client = node_handle.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");

  while (!ik_service_client.exists())
  {
    ROS_INFO("Waiting for service");
    ros::Duration(1.0).sleep();
  }

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::construcor\n", state_object_name.c_str());
}

state_ik_get_joints_from_pose::~state_ik_get_joints_from_pose(){
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::destructor\n", state_object_name.c_str());

    /* Write here your code */

    DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::destructor\n", state_object_name.c_str());
}


state_ik_get_joints_from_pose::status_enum state_ik_get_joints_from_pose::onEnter(input_keys_type& input_keys){

  state_ik_get_joints_from_pose::status_enum return_code = status_succes;
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onEnter\n", state_object_name.c_str());

  user_data.input_keys = input_keys;
  /* Write here your code */
  moveit_msgs::GetPositionIK::Request service_request;
  moveit_msgs::GetPositionIK::Response service_response;

  service_request.ik_request.group_name = input_keys.group_name;
  service_request.ik_request.pose_stamped = input_keys.pose;
  service_request.ik_request.pose_stamped.pose.position.z += input_keys.offset;

  tf2::Quaternion q_orig, q_rot, q_new;
  q_orig.setRPY( 0, 0, 0 );  // Create this quaternion from roll/pitch/yaw (in radians)
  q_rot.setRPY(0, input_keys.rotation, 0);

  q_new = q_rot * q_orig;  // Calculate the new orientation
  q_new.normalize();

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "q_new x: %f\n", q_new.x());
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "q_new y: %f\n", q_new.y());
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "q_new z: %f\n", q_new.z());
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "q_new w: %f\n", q_new.w());

  service_request.ik_request.pose_stamped.pose.orientation.x = q_new.x();
  service_request.ik_request.pose_stamped.pose.orientation.y = q_new.y();
  service_request.ik_request.pose_stamped.pose.orientation.z = q_new.z();
  service_request.ik_request.pose_stamped.pose.orientation.w = q_new.w();

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "header = %s\n", service_request.ik_request.pose_stamped.header.frame_id.c_str());
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "position.x = %f\n", service_request.ik_request.pose_stamped.pose.position.x);
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "position.y = %f\n", service_request.ik_request.pose_stamped.pose.position.y);
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "position.z = %f\n", service_request.ik_request.pose_stamped.pose.position.z);
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "orientation.x = %f\n", service_request.ik_request.pose_stamped.pose.orientation.x);
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "orientation.y = %f\n", service_request.ik_request.pose_stamped.pose.orientation.y);
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "orientation.z = %f\n", service_request.ik_request.pose_stamped.pose.orientation.z);
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "orientation.w = %f\n", service_request.ik_request.pose_stamped.pose.orientation.w);

  service_request.ik_request.ik_link_name = input_keys.tool_link;
  {
		sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(2));
	  if (msg == NULL){
				ROS_ERROR("No joint states received");
				return status_error;
		}
	  else{
	      service_request.ik_request.robot_state.joint_state = *msg;
    }
	}

  {
    int number_of_joints = service_request.ik_request.robot_state.joint_state.position.size();
    for(int i = 0; i < number_of_joints; i++){
      DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "Current Joints: %s, %f\n", service_request.ik_request.robot_state.joint_state.name[i].c_str(),
                                service_request.ik_request.robot_state.joint_state.position[i]);
    }
  }

  service_request.ik_request.avoid_collisions = true;
  service_request.ik_request.attempts = 500;
  service_request.ik_request.timeout = ros::Duration(5.0);

  /* Call the service */
  ik_service_client.call(service_request, service_response);
  ROS_INFO_STREAM(
      "GetPositionIK: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                 << service_response.error_code.val);
  if(service_response.error_code.val != service_response.error_code.SUCCESS) return status_error;


  int number_of_joints = service_response.solution.joint_state.position.size();
  for(int i = 0; i < number_of_joints; i++){
    DEBUG_PRINT(DEBUG_ITEMS & DEBUG_CUSTOM, "Ik Joints: %s, %f\n", service_response.solution.joint_state.name[i].c_str(),
                                service_response.solution.joint_state.position[i]);
    user_data.output_keys.joints.insert( std::pair<std::string,double>(service_response.solution.joint_state.name[i].c_str(),
                                  service_response.solution.joint_state.position[i]));
  }


  state = state_ik_get_joints_from_pose::running;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onEnter\n", state_object_name.c_str());
  return(return_code);
}


state_ik_get_joints_from_pose::outcomes_enum state_ik_get_joints_from_pose::execute(void){

  state_ik_get_joints_from_pose::outcomes_enum return_value = outcomes_busy;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::execute\n", state_object_name.c_str());

  /* Write here your code */
    return_value = outcomes_done;
    state = state_ik_get_joints_from_pose::idle;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::execute\n", state_object_name.c_str());
  return(return_value);
}

/* do not modify this member function */
state_ik_get_joints_from_pose::outcomes_enum state_ik_get_joints_from_pose::simpleEexecute(input_keys_type& input_keys, output_keys_type& output_keys){
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


state_ik_get_joints_from_pose::output_keys_type state_ik_get_joints_from_pose::onExit(){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onExit\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onExit\n", state_object_name.c_str());
  return(user_data.output_keys);
}

state_ik_get_joints_from_pose::status_enum state_ik_get_joints_from_pose::onStop(){

  state_ik_get_joints_from_pose::status_enum return_code = status_succes;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onStop\n", state_object_name.c_str());

    /* Write here your code */

    DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onStop\n", state_object_name.c_str());
  return(return_code);
}

state_ik_get_joints_from_pose::status_enum state_ik_get_joints_from_pose::onPause(){

  state_ik_get_joints_from_pose::status_enum return_code = status_succes;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onPause\n", state_object_name.c_str());

  /* Write here your code */

  state = state_ik_get_joints_from_pose::paused;


  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onPause\n", state_object_name.c_str());
  return(return_code);
}

state_ik_get_joints_from_pose::status_enum state_ik_get_joints_from_pose::onResume(){

  state_ik_get_joints_from_pose::status_enum return_code = status_succes;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onResume\n", state_object_name.c_str());

  /* Write here your code */

  state = state_ik_get_joints_from_pose::running;


  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onResume\n", state_object_name.c_str());
  return(return_code);
}

state_ik_get_joints_from_pose::state_enum state_ik_get_joints_from_pose::getState(void){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::getState\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::getState\n", state_object_name.c_str());
  return(state);
}
