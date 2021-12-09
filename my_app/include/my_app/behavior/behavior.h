#ifndef _BEHAVIOR_H_
#define _BEHAVIOR_H_

#include "my_app/states/state_template.h"
#include <ros/ros.h>


class behavior{

  typedef enum{
    status_idle = 0,
    status_running
  }status;

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
protected:
  ros::NodeHandle node_handle;
  ros::NodeHandle priv_node_handle;

  ros::Timer state_timer;
  void stateCallback(const ros::TimerEvent&);

  status _status = status::status_idle;
  state _state = state::state_idle;

  // enter here your states
  state_template state1;
  state_template state2;


public:
  behavior();
  ~behavior();

  void start();
  void abort();
  status getStatus();
  void reset();
};

#endif // _BEHAVIOR_H_
