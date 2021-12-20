/*******************************************************************************
File: state_srdf_to_moveit.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Interface header (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#ifndef _STATE_SRDF_TO_MOVEIT_H_
#define _STATE_SRDF_TO_MOVEIT_H_
#include <iostream>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include "my_app/debug.h"

using namespace std;

class state_srdf_to_moveit{
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
    std::string config_name;
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
  state  state_ = idle;
  user_data_ user_data;
  string state_object_name;
  moveit::planning_interface::MoveGroupInterface* move_group;
  robot_state::RobotStatePtr move_group_state;

public:
  // constructor
  state_srdf_to_moveit(const std::string& state_object_name);//, const std::string& group/* define own paramters here*/);
  // destructor
  ~state_srdf_to_moveit();

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

#endif // _STATE_SRDF_TO_MOVEIT_H_
