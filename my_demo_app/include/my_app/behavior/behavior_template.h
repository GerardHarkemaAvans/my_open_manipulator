/*******************************************************************************
File: behavior_template.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Interface header (template) voor een behaivior definitie welke gebruikt kan
worden bij een statemachine.
*******************************************************************************/
#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include "my_app/states/state_template.h"
#include <ros/ros.h>
#include <iostream>
#include <string>

using namespace std;

class behavior_template{

public:
  typedef enum{
    status_busy = 0,
    status_finshed
  }outcomes;

  typedef enum{
    state_idle = 0,
    state_start,
    // add states here
    state_1,
    state_2,
    state_finshed,
    state_abort
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

  //const std::string state_1_1 = "state 1";
  //const std::string state_2_1 = "state 2";
  // enter here your states
  state_template *state1;
  state_template *state2;

  user_data_ user_data = {0};


public:
  behavior_template(const std::string& state_object_name);
  ~behavior_template();

  void onEnter();
  void onEnter(input_keys_ &input_keys);
  output_keys_ onExit();
  void abort();
  outcomes execute();
  void reset();
};

#endif // _BEHAVIOR_H_
