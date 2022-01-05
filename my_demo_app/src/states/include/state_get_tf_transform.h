/*******************************************************************************
File: state_get_tf_transform.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Interface header (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#ifndef _STATE_GET_TF_TRANSFORM_H_
#define _STATE_GET_TF_TRANSFORM_H_
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "my_app/debug.h"

using namespace std;


class state_get_tf_transform{

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
    string target_frame;
    string source_frame;
  }input_keys_type;

  typedef struct output_keys_struct{
    geometry_msgs::PoseStamped transform;
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

public:
  // constructor
  state_get_tf_transform(const std::string& state_object_name/* define own paramters here*/);
  // destructor
  ~state_get_tf_transform();

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

#endif // _STATE_GET_TF_TRANSFORM_H_
