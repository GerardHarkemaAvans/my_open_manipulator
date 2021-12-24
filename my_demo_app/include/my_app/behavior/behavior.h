/*******************************************************************************
File: behavior.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Interface header (template) voor een behaivior definitie welke gebruikt kan
worden bij een statemachine.
*******************************************************************************/
#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include "my_app/states/state_srdf_to_moveit.h"
#include "my_app/states/state_move_joints.h"
#include "my_app/states/state_get_tf_transform.h"
#include "my_app/states/state_ik_get_joints_from_pose.h"
#include "my_app/states/state_template.h"
#include "my_app/debug.h"

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>
#include <iostream>
#include <string>

using namespace std;

class behavior{

public:
  typedef enum{
    status_busy = 0,
    status_finshed,
    status_failed
  }outcomes;

  typedef enum{
    state_idle = 0,
    state_start,
    // add states here
    go_home,
    go_left,
    go_right,
    get_transform,
    ik_calculate_joits,
    go_pose,
    go_resting,
    state_finshed,
    state_failed,
    state_abort,
    state_wait_for_reset
    // add states here
  }state;

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

  string behavior_object_name;

  outcomes _outcomes = outcomes::status_busy;
  state _state = state::state_idle;

  // enter here your states type
  state_srdf_to_moveit* srdf_to_moveit;
  state_move_joints* move_joints;
  state_get_tf_transform* get_tf_transform;
  state_ik_get_joints_from_pose* ik_get_joints_from_pose;
  state_template* s_template;
  user_data_ user_data = {0};

  geometry_msgs::PoseStamped object_pose;
  std::map<std::string, double> object_pose_joints;



public:
  behavior(const std::string& state_object_name);
  ~behavior();

  void onEnter();
  void onEnter(input_keys_ &input_keys);
  output_keys_ onExit();
  void abort();
  outcomes execute();
  void reset();
};

#endif // _BEHAVIOR_H_
