/*******************************************************************************
File: behavior_template.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een behaivior definitie welke gebruikt kan worden
bij een statemachine.
*******************************************************************************/
#include "my_app/behavior/behavior_template.h"
#include "my_app/states/state_template.h"

#define DEBUG_LEVEL_NONE  0 // Nu Debugging
#define DEBUG_LEVEL_1     1 // Own debug messages
#define DEBUG_LEVEL_2     2 // Behavior debug messages
#define DEBUG_LEVEL_3     3 // All Behavior debug messages


#define DEBUG_LEVEL       DEBUG_LEVEL_2 //DEBUG_LEVEL_NONE


behavior_template::behavior_template(const std::string& behavior_object_name)
: node_handle(""),
  priv_node_handle("~")
{

  this->behavior_object_name = behavior_object_name;
#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::construcor" << endl;
#endif

  /* Write here your code */


  state1 = new state_template("state1");
  state2 = new state_template("state2");


  state_timer = node_handle.createTimer(ros::Duration(0.100)/*100ms*/,
                                          &behavior_template::stateCallback,
                                          this);

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::construcor" << endl;
#endif


}

behavior_template::~behavior_template()
{

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::destrucor" << endl;
#endif

/* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::destrucor" << endl;
#endif

}

void behavior_template::stateCallback(const ros::TimerEvent&){

  switch(_state){
    case state_idle:
      break;
    case state_start:
      {
        state_template::input_keys_ input_key;
        input_key.dummy = 0;
        input_key.repeat_count = 1;
        state1->onEnter(input_key);
      }
      _state = state_1;
      break;
    case state_1:
      if(state1->execute() != state_template::busy){
        state1->onExit();
        {
          state_template::input_keys_ input_key;
          input_key.dummy = 0;
          input_key.repeat_count = 3;
          state2->onEnter(input_key);
        }
        _state = state_2;
      }
      break;
    case state_2:
      if(state2->execute() != state_template::busy){
        state2->onExit();
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

void behavior_template::onEnter(input_keys_ &input_keys){

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

behavior_template::outcomes behavior_template::execute(){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_3)
  cout << "Entering "  << behavior_object_name << "::execute" << endl;
#endif

/* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_3)
  cout << "Leaving "  << behavior_object_name << "::execute" << endl;
#endif
  return(_outcomes);
}

behavior_template::output_keys_ behavior_template::onExit(){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::onExit" << endl;
#endif

/* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::onExit" << endl;
#endif
  return(user_data.output_keys);
}

void behavior_template::abort(){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::abort" << endl;
#endif

/* Write here your code */

_state = state_abort;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::abort" << endl;
#endif
}

void behavior_template::reset(){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Entering "  << behavior_object_name << "::reset" << endl;
#endif

  /* Write here your code */

  _state = state_finshed;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_2)
  cout << "Leaving "  << behavior_object_name << "::reset" << endl;
#endif
}
