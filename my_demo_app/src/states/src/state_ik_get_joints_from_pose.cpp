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

#define DEBUG_ITEMS       DEBUG_NONE | DEBUG_CUSTOM//| DEBUG_STATES | DEBUG_CUSTOM

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


state_ik_get_joints_from_pose::status state_ik_get_joints_from_pose::onEnter(input_keys_& input_keys){

  state_ik_get_joints_from_pose::status return_code = success;
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

#if (DEBUG_ITEMS & DEBUG_CUSTOM)
  ROS_INFO("q_new x: %f", q_new.x());
  ROS_INFO("q_new y: %f", q_new.y());
  ROS_INFO("q_new z: %f", q_new.z());
  ROS_INFO("q_new w: %f", q_new.w());
#endif

  service_request.ik_request.pose_stamped.pose.orientation.x = q_new.x();
  service_request.ik_request.pose_stamped.pose.orientation.y = q_new.y();
  service_request.ik_request.pose_stamped.pose.orientation.z = q_new.z();
  service_request.ik_request.pose_stamped.pose.orientation.w = q_new.w();

#if (DEBUG_ITEMS & DEBUG_CUSTOM)
  ROS_INFO("header = %s", service_request.ik_request.pose_stamped.header.frame_id.c_str());
  ROS_INFO("x = %f", service_request.ik_request.pose_stamped.pose.position.x);
  ROS_INFO("y = %f", service_request.ik_request.pose_stamped.pose.position.y);
  ROS_INFO("z = %f", service_request.ik_request.pose_stamped.pose.position.z);
  ROS_INFO("x = %f", service_request.ik_request.pose_stamped.pose.orientation.x);
  ROS_INFO("y = %f", service_request.ik_request.pose_stamped.pose.orientation.y);
  ROS_INFO("z = %f", service_request.ik_request.pose_stamped.pose.orientation.z);
  ROS_INFO("w = %f", service_request.ik_request.pose_stamped.pose.orientation.w);
#endif


  service_request.ik_request.ik_link_name = input_keys.tool_link;
  {
		sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(2));
	  if (msg == NULL){
				ROS_INFO("No joint states received");
				return error;
		}
	  else{
	      service_request.ik_request.robot_state.joint_state = *msg;
    }
	}

#if (DEBUG_ITEMS & DEBUG_CUSTOM)
  {
    int number_of_joints = 5;//sizeof(service_request.ik_request.robot_state.joint_state.position)/sizeof(service_request.ik_request.robot_state.joint_state.position[0]);
    for(int i = 0; i < number_of_joints; i++){
      ROS_INFO("Current Joints: %s, %f", service_request.ik_request.robot_state.joint_state.name[i].c_str(),
                                service_request.ik_request.robot_state.joint_state.position[i]);
    }
  }
#endif

  service_request.ik_request.avoid_collisions = true;
  service_request.ik_request.attempts = 500;
  service_request.ik_request.timeout = ros::Duration(5.0);

  /* Call the service */
  ik_service_client.call(service_request, service_response);
  ROS_INFO_STREAM(
      "GetPositionIK: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                 << service_response.error_code.val);
  if(service_response.error_code.val != service_response.error_code.SUCCESS) return error;


  int number_of_joints = 5;//sizeof(service_response.solution.joint_state.position)/sizeof(service_response.solution.joint_state.position[0]);
  //int number_of_joints = service_response.solution.joint_state.name.len();
  for(int i = 0; i < number_of_joints; i++){
    #if (DEBUG_ITEMS & DEBUG_CUSTOM)
      ROS_INFO("Ik Joints: %s, %f", service_response.solution.joint_state.name[i].c_str(),
                                service_response.solution.joint_state.position[i]);
    #endif
    user_data.output_keys.joints.insert( std::pair<std::string,double>(service_response.solution.joint_state.name[i].c_str(),
                                  service_response.solution.joint_state.position[i]));
  }


  state_ = state_ik_get_joints_from_pose::running;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onEnter\n", state_object_name.c_str());
  return(return_code);
}


state_ik_get_joints_from_pose::outcomes state_ik_get_joints_from_pose::execute(void){

  state_ik_get_joints_from_pose::outcomes return_value = busy;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::execute\n", state_object_name.c_str());

  /* Write here your code */
    return_value = done;
    state_ = state_ik_get_joints_from_pose::idle;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::execute\n", state_object_name.c_str());
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

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onExit\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onExit\n", state_object_name.c_str());
  return(user_data.output_keys);
}

state_ik_get_joints_from_pose::status state_ik_get_joints_from_pose::onStop(){

  state_ik_get_joints_from_pose::status return_code = success;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onStop\n", state_object_name.c_str());

    /* Write here your code */

    DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onStop\n", state_object_name.c_str());
  return(return_code);
}

state_ik_get_joints_from_pose::status state_ik_get_joints_from_pose::onPause(){

  state_ik_get_joints_from_pose::status return_code = success;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onPause\n", state_object_name.c_str());

  /* Write here your code */

  state_ = state_ik_get_joints_from_pose::paused;


  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onPause\n", state_object_name.c_str());
  return(return_code);
}

state_ik_get_joints_from_pose::status state_ik_get_joints_from_pose::onResume(){

  state_ik_get_joints_from_pose::status return_code = success;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onResume\n", state_object_name.c_str());

  /* Write here your code */

  state_ = state_ik_get_joints_from_pose::running;


  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onResume\n", state_object_name.c_str());
  return(return_code);
}

state_ik_get_joints_from_pose::state state_ik_get_joints_from_pose::getState(void){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::getState\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::getState\n", state_object_name.c_str());
  return(state_);
}
