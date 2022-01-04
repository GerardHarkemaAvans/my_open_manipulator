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
  }status;

  typedef enum{
    outcomes_busy = 0,
    outcomes_finshed,
    outcomes_failed
  }outcomes;

  typedef enum{
    state_idle = 0,
    state_start,
    // add states here
    state_get_transform,
    state_ik_calculate_joints,
    state_go_pose,
    // add states here
    state_finshed,
    state_failed,
    state_abort,
    state_wait_for_reset
  }state;

  typedef enum{
    execution_wait_for_start = 0,
    execution_execute,
    execution_exit
    // append other errors here
  }execution_state;


  typedef struct input_keys_struct{
    string target_frame;
    string source_frame;
  }input_keys_;

  typedef struct output_keys_struct{
    int dummy;
    // append other keys here
  }output_keys_;

  typedef struct user_data_struct{
    input_keys_ input_keys;
    output_keys_ output_keys;
  }user_data_;

protected:
  ros::NodeHandle node_handle;
  ros::NodeHandle priv_node_handle;

  ros::Timer state_timer;
  void stateCallback(const ros::TimerEvent&);
  void stateHandler(void);
  execution_state execution_state_ = execution_wait_for_start;
  outcomes execution_return_value;

  string behavior_object_name;
  bool simple_execution_mode;

  outcomes _outcomes = outcomes::outcomes_busy;
  state _state = state::state_idle;

  // enter here your states type
  state_move_joints* move_joints;
  state_get_tf_transform* get_tf_transform;
  state_ik_get_joints_from_pose* ik_get_joints_from_pose;
  user_data_ user_data;

  geometry_msgs::PoseStamped object_pose;
  std::map<std::string, double> object_pose_joints;

public:
  behavior_go_pose_ik(const std::string& state_object_name, bool simple_execution_mode);
  ~behavior_go_pose_ik();

  status onEnter(input_keys_ &input_keys);
  outcomes simpleEexecute(input_keys_& input_keys, output_keys_& output_keys);
  outcomes execute();
  output_keys_ onExit();

#if 0 // not implmented yet
  status abort();
  status reset();
#endif
};

#endif // _BEHAVIOR_GO_POSE_IK_H_
