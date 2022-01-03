/*******************************************************************************
File: state_move_joints.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Interface header (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#ifndef _STATE_MOVE_JOINTS_H_
#define _STATE_MOVE_JOINTS_H_
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/ExecuteTrajectoryActionResult.h>
#include "my_app/debug.h"

using namespace std;

class state_move_joints{
public:
  typedef enum{
    status_succes = 0,
    status_error
    // append other errors here
  }status;

  typedef enum{
    idle = 0,
    running,
    paused
    // append other errors here
  }state;


  typedef enum{
    execution_wait_for_start = 0,
    execution_execute,
    execution_exit
    // append other errors here
  }execution_state;

  typedef enum{
    outcomes_busy = 0,
    outcomes_done,
    outcomes_failed
    // append other outcomes here
  }outcomes;


  typedef struct input_keys_struct{
    std::map<std::string, double> joints;
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
  state  state_ = idle;
  user_data_ user_data;
  string state_object_name;
  moveit::planning_interface::MoveGroupInterface* move_group;
  robot_state::RobotStatePtr move_group_state;
  execution_state execution_state_ = execution_wait_for_start;
  outcomes execution_return_value;

public:
  // constructor
  state_move_joints(const std::string& state_object_name, const std::string& group/* define own paramters here*/);
  // destructor
  ~state_move_joints();

  // Starten van de state
  status onEnter(input_keys_& input_keys);
  // Executeren van de state, state is actief zolang outcome == outcomes_busy
  outcomes execute(void);

  outcomes simpleEexecute(input_keys_& input_keys, output_keys_& output_keys);
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

#endif // _STATE_MOVE_JOINTS_H_
