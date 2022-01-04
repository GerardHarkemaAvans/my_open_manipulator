/*******************************************************************************
File: behavior_main.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Interface header (template) voor een behaivior definitie welke gebruikt kan
worden bij een statemachine.
*******************************************************************************/
#ifndef _BEHAVIOR_MAIN_H_
#define _BEHAVIOR_MAIN_H_

#include "../include/behavior_go_pose_ik.h"
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

class behavior_main{

public:

  typedef enum{
    status_succes = 0,
    status_error
    // append other errors here
  }status;

  typedef enum{
    status_busy = 0,
    status_finshed,
    status_failed
  }outcomes;

  typedef enum{
    state_idle = 0,
    state_start,
    // add states here
    state_go_home,
    state_go_left,
    state_go_right,
    state_go_pose_ik,
    state_go_resting,
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
    int dummy;
    // append other keys here
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

  outcomes _outcomes = outcomes::status_busy;
  state _state = state::state_idle;

  // enter here your states type
  state_srdf_to_moveit* srdf_to_moveit;
  state_template* s_template;
  behavior_go_pose_ik *go_pose_ik;
  user_data_ user_data;


public:
  behavior_main(const std::string& state_object_name, bool simple_execution_mode);
  ~behavior_main();

  status onEnter(input_keys_ &input_keys);
  outcomes simpleEexecute(input_keys_& input_keys, output_keys_& output_keys);
  outcomes execute();
  output_keys_ onExit();

#if 0 // not implmented yet
  status abort();
  status reset();
#endif
};

#endif // _BEHAVIOR_MAIN_H_
