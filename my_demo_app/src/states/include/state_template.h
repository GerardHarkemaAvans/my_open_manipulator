/*******************************************************************************
File: state_template.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Interface header (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#ifndef _STATE_TEMPLATE_H_
#define _STATE_TEMPLATE_H_
#include <ros/ros.h>
#include <iostream>
#include <string>
#include "my_app/debug.h"

using namespace std;


class state_template{

public:
  typedef enum{
    status_succes = 0,
    status_error
    // append other errors here
  }status;

  typedef enum{
    execution_wait_for_start = 0,
    execution_execute,
    execution_exit
    // append other errors here
  }execution_state;

  typedef enum{
    idle = 0,
    running,
    paused
    // append other errors here
  }state;

  typedef enum{
    outcomes_busy = 0,
    outcomes_done,
    outcomes_failed
    // append other outcomes here
  }outcomes;


  typedef struct input_keys_struct{
    int dummy;
    // append other keys here
    int repeat_count; /* Example of key */
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
  state  state_ = idle;
  user_data_type user_data;
  string state_object_name;
  execution_state execution_state_ = execution_wait_for_start;
  outcomes execution_return_value;

  int remaining_count;


public:
  // constructor
  state_template(const std::string& state_object_name/* define own paramters here*/);
  // destructor
  ~state_template();

  // Starten van de state
  status onEnter(input_keys_type& input_keys);
  // Executeren van de state, state is actief zolang outcome == outcomes_busy
  outcomes execute(void);

  outcomes simpleEexecute(input_keys_type& input_keys, output_keys_type& output_keys);
  // Einde van de state
  output_keys_type onExit(void);
  // Afbeken van de state
  status onStop(void);
  // Tijdelijk de state stopzetten
  status onPause(void);
  // Voortzetten na pauze
  status onResume(void);
  // Stte van de toestand
  state getState(void);

};

#endif // _STATE_TEMPLATE_H_
