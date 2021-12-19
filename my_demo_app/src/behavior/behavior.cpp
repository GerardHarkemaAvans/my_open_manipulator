/*******************************************************************************
File: behavior.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een behaivior definitie welke gebruikt kan worden
bij een statemachine.
*******************************************************************************/
#include "my_app/behavior/behavior.h"

#define DEBUG_LEVEL_NONE  0 // Nu Debugging
#define DEBUG_LEVEL_1     1 // Own debug messages
#define DEBUG_LEVEL_2     2 // Behavior debug messages
#define DEBUG_LEVEL_3     3 // All Behavior debug messages


#define DEBUG_LEVEL       DEBUG_LEVEL_2 //DEBUG_LEVEL_NONE


behavior::behavior(const std::string& behavior_object_name)
: node_handle(""),
  priv_node_handle("~")
{

  this->behavior_object_name = behavior_object_name;
#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::construcor" << endl;
#endif

  /* Write here your code */
  string tmp = "srdf_to_moveit";
  srdf_to_moveit = new state_srdf_to_moveit("arm");
  state_timer = node_handle.createTimer(ros::Duration(0.100)/*100ms*/,
                                          &behavior::stateCallback,
                                          this);

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::construcor" << endl;
#endif


}

behavior::~behavior()
{

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::destrucor" << endl;
#endif

/* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::destrucor" << endl;
#endif

}

void behavior::stateCallback(const ros::TimerEvent&){

  switch(_state){
    case state_idle:
      break;
    case state_start:
      {
        state_srdf_to_moveit::input_keys_ input_key;
        input_key.config_name = "home";
        srdf_to_moveit->onEnter(input_key);
      }
      _state = go_home;
      break;
    case go_home:
      if(srdf_to_moveit->execute() != state_srdf_to_moveit::busy){
        srdf_to_moveit->onExit();
        {
          state_srdf_to_moveit::input_keys_ input_key;
          input_key.config_name = "left";
          srdf_to_moveit->onEnter(input_key);
        }
        _state = go_left;
      }
      break;
    case go_left:
      if(srdf_to_moveit->execute() != state_srdf_to_moveit::busy){
        srdf_to_moveit->onExit();
        {
          state_srdf_to_moveit::input_keys_ input_key;
          input_key.config_name = "right";
          srdf_to_moveit->onEnter(input_key);
        }
        _state = go_right;
      }
      break;
    case go_right:
      if(srdf_to_moveit->execute() != state_srdf_to_moveit::busy){
        srdf_to_moveit->onExit();
        {
          state_srdf_to_moveit::input_keys_ input_key;
          input_key.config_name = "resting";
          srdf_to_moveit->onEnter(input_key);
        }
        _state = go_resting;
      }
      break;
    case go_resting:
      if(srdf_to_moveit->execute() != state_srdf_to_moveit::busy){
        srdf_to_moveit->onExit();
        _state = state_finshed;
      }
      break;
    case state_finshed:
      _outcomes = outcomes::status_finshed;
      break;
    default:
      break;
  }
}

void behavior::onEnter(input_keys_ &input_keys){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::onEnter" << endl;
#endif

/* Write here your code */

user_data.input_keys = input_keys;
_state = state_start;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::onEnter" << endl;
#endif

}

behavior::outcomes behavior::execute(){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_3)
  cout << "Entering "  << behavior_object_name << "::execute" << endl;
#endif

/* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_3)
  cout << "Leaving "  << behavior_object_name << "::execute" << endl;
#endif
  return(_outcomes);
}

behavior::output_keys_ behavior::onExit(){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::onExit" << endl;
#endif

/* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::onExit" << endl;
#endif
  return(user_data.output_keys);
}

void behavior::abort(){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::abort" << endl;
#endif

/* Write here your code */

_state = state_abort;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::abort" << endl;
#endif
}

void behavior::reset(){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::reset" << endl;
#endif

  /* Write here your code */

  _state = state_finshed;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::reset" << endl;
#endif
}
