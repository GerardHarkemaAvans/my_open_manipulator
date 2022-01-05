/*******************************************************************************
File: state_ik_get_joints_from_pose.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Interface header (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#ifndef _STATE_IK_GET_JOINTS_FROM_POSE_H_
#define _STATE_IK_GET_JOINTS_FROM_POSE_H_
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include "my_app/debug.h"

using namespace std;


class state_ik_get_joints_from_pose{

public:
  typedef enum{
    status_succes = 0,
    status_error
    // append other errors here
  }status_enum;

  typedef enum{
    execution_wait_for_start = 0,
    execution_execute,
    execution_exit
    // append other errors here
  }execution_state_enum;

  typedef enum{
    idle = 0,
    running,
    paused
    // append other errors here
  }state_enum;

  typedef enum{
    outcomes_busy = 0,
    outcomes_done,
    outcomes_failed
    // append other outcomes here
  }outcomes_enum;


  typedef struct input_keys_struct{
    geometry_msgs::PoseStamped pose;
    string group_name;
    string tool_link;
    float offset;
    float rotation;
  }input_keys_type;

  typedef struct output_keys_struct{
    std::map<std::string, double> joints;
    // append other keys here
  }output_keys_type;

  typedef struct user_data_struct{
    input_keys_type input_keys;
    output_keys_type output_keys;
  }user_data_type;

protected:
  ros::NodeHandle node_handle;
  state_enum  state = idle;
  user_data_type user_data;
  string state_object_name;
  execution_state_enum execution_state_ = execution_wait_for_start;
  outcomes_enum execution_return_value;
  ros::ServiceClient ik_service_client;


public:
  // constructor
  state_ik_get_joints_from_pose(const std::string& state_object_name, const std::string& group/* define own paramters here*/);
  // destructor
  ~state_ik_get_joints_from_pose();

  // Starten van de state
  status_enum onEnter(input_keys_type& input_keys);
  // Executeren van de state, state is actief zolang outcome == outcomes_busy
  outcomes_enum execute(void);

  outcomes_enum simpleEexecute(input_keys_type& input_keys, output_keys_type& output_keys);
  // Einde van de state
  output_keys_type onExit(void);
  // Afbeken van de state
  status_enum onStop(void);
  // Tijdelijk de state stopzetten
  status_enum onPause(void);
  // Voortzetten na pauze
  status_enum onResume(void);
  // Stte van de toestand
  state_enum getState(void);

};

#endif // _STATE_IK_GET_JOINTS_FROM_POSE_H_
