#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include "my_app/states/state_template.h"
#include <ros/ros.h>
#include <string>

using namespace std;

class behavior{

  typedef enum{
    status_idle = 0,
    status_running
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

  outcomes _outcomes = outcomes::status_idle;
  state _state = state::state_idle;

  // enter here your states
  state_template state1("state 1");
  state_template state2("state 2");

  user_data_ user_data = {0};


public:
  behavior();
  ~behavior();

  void onEnter();
  void onEnter(input_keys_ &input_keys);
  output_keys_ onExit();
  void abort();
  outcomes execute();
  void reset();
};

#endif // _BEHAVIOR_H_
