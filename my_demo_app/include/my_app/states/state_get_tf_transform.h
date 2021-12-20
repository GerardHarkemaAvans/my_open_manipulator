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
#include <iostream>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include "my_app/debug.h"

using namespace std;


class state_get_tf_transform{

public:
  typedef enum{
    success = 0,
    error
    // append other errors here
  }status;

  typedef enum{
    idle = 0,
    running,
    paused
    // append other errors here
  }state;

  typedef enum{
    busy = 0,
    done,
    failed
    // append other outcomes here
  }outcomes;


  typedef struct input_keys_struct{
    string target_frame;
    string source_frame;
  }input_keys_;

  typedef struct output_keys_struct{
    geometry_msgs::PoseStamped transform;
    // append other keys here
  }output_keys_;

  typedef struct user_data_struct{
    input_keys_ input_keys;
    output_keys_ output_keys;
  }user_data_;

protected:
  state  state_ = idle;
  user_data_ user_data;
  string state_object_name;

public:
  // constructor
  state_get_tf_transform(const std::string& state_object_name/* define own paramters here*/);
  // destructor
  ~state_get_tf_transform();

  // Starten van de state
  status onEnter(input_keys_& input_keys);
  // Executeren van de state, state is actief zolang outcome == busy
  outcomes execute(void);
  // Einde van de state
  output_keys_ onExit(void);
  // Afbeken van de state
  status onStop(void);
  // Tijdelijk de state stopzetten
  status onPause(void);
  // Voortzetten na pauze
  status onResume(void);
  // Stte van de toestand
  state getState(void);

};

#endif // _STATE_GET_TF_TRANSFORM_H_
