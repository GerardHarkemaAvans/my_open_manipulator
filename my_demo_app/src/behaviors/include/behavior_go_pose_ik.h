/*******************************************************************************
File: behavior_go_pose_ik.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Interface header (template) voor een behaivior definitie welke gebruikt kan
worden bij een statemachine.
*******************************************************************************/
#ifndef _BEHAVIOR_GO_POSE_IK_H_
#define _BEHAVIOR_GO_POSE_IK_H_

#include "src/states/include/state_srdf_to_moveit.h"
#include "src/states/include/state_move_joints.h"
#include "src/states/include/state_get_tf_transform.h"
#include "src/states/include/state_ik_get_joints_from_pose.h"
#include "src/states/include/state_template.h"
#include "my_app/debug.h"

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>
#include <iostream>
#include <string>

using namespace std;

class behavior_go_pose_ik{

public:

  typedef enum{
    status_succes = 0,
    status_error
    // append other errors here
  }status_enum;

  typedef enum{
    outcomes_busy = 0,
    outcomes_finshed,
    outcomes_failed
  }outcomes_enum;

  typedef enum{
    state_idle = 0,
    state_start,
    // add states here
    state_get_pose_transform,
    state_ik_calculate_pose_joints,
    state_go_pose,
    // add states here
    state_finshed,
    state_failed,
    state_abort,
    state_wait_for_reset
  }state_enum;

  typedef enum{
    execution_wait_for_start = 0,
    execution_execute,
    execution_exit
    // append other errors here
  }execution_state_enum;


  typedef struct input_keys_struct{
    string target_frame;
    string source_frame;
  }input_keys_type;

  typedef struct output_keys_struct{
    int dummy;
    // append other keys here
  }output_keys_type;

  typedef struct user_data_struct{
    input_keys_type input_keys;
    output_keys_type output_keys;
  }user_data_type;

protected:
  ros::NodeHandle node_handle;
  ros::NodeHandle priv_node_handle;

  ros::Timer state_timer;
  void stateCallback(const ros::TimerEvent&);
  void stateHandler(void);
  execution_state_enum execution_state_ = execution_wait_for_start;
  outcomes_enum execution_return_value;

  string behavior_object_name;
  bool simple_execution_mode;

  outcomes_enum outcomes = outcomes_enum::outcomes_busy;
  state_enum state = state_enum::state_idle;

  // enter here your states type
  state_move_joints* move_joints;
  state_get_tf_transform* get_tf_transform;
  state_ik_get_joints_from_pose* ik_get_joints_from_pose;
  user_data_type user_data;

  geometry_msgs::PoseStamped object_pose;
  std::map<std::string, double> object_pose_joints;

public:
  behavior_go_pose_ik(const std::string& behavior_object_name, bool simple_execution_mode);
  ~behavior_go_pose_ik();

  status_enum onEnter(input_keys_type &input_keys);
  outcomes_enum simpleEexecute(input_keys_type& input_keys, output_keys_type& output_keys);
  outcomes_enum execute();
  output_keys_type onExit();

#if 0 // not implmented yet
  status_enum abort();
  status_enum reset();
#endif
};

#endif // _BEHAVIOR_GO_POSE_IK_H_
