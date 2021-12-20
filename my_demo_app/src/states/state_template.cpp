/*******************************************************************************
File: state_template.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#include "my_app/states/state_template.h"

#define DEBUG_LEVEL       DEBUG_LEVEL_2 //DEBUG_LEVEL_NONE

state_template::state_template(const std::string& state_object_name/* define own paramters here*/){
  this->state_object_name = state_object_name;
#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::construcor" << endl;
#endif

  /* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::construcor" << endl;
#endif
}

state_template::~state_template(){
#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::destrucor" << endl;
#endif

    /* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::destrucor" << endl;
#endif
}


state_template::status state_template::onEnter(input_keys_& input_keys){

  state_template::status return_code = success;
#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::onEnter" << endl;
#endif

  user_data.input_keys = input_keys;
  remaining_count = user_data.input_keys.repeat_count;

  /* Write here your code */

  state_ = state_template::running;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::onEnter" << endl;
#endif
  return(return_code);
}


state_template::outcomes state_template::execute(void){

  state_template::outcomes return_value = busy;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::execute" << endl;
#endif

  /* Write here your code */
  if(--remaining_count == 0){ /* Example */
    return_value = done;
    state_ = state_template::idle;
  }

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::execute" << endl;
#endif
  return(return_value);
}

state_template::output_keys_ state_template::onExit(){

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::onExit" << endl;
#endif

  /* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::onExit" << endl;
#endif
  return(user_data.output_keys);
}

state_template::status state_template::onStop(){

  state_template::status return_code = success;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::onStop" << endl;
#endif

    /* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::onStop" << endl;
#endif
  return(return_code);
}

state_template::status state_template::onPause(){

  state_template::status return_code = success;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::onPause" << endl;
#endif

  /* Write here your code */

  state_ = state_template::paused;


#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::onPause" << endl;
#endif
  return(return_code);
}

state_template::status state_template::onResume(){

  state_template::status return_code = success;

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::onResume" << endl;
#endif

  /* Write here your code */

  state_ = state_template::running;


#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::onResume" << endl;
#endif
  return(return_code);
}

state_template::state state_template::getState(void){
#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Entering "  << state_object_name << "::getState" << endl;
#endif

  /* Write here your code */

#if (DEBUG_LEVEL >= DEBUG_LEVEL_1)
  cout << "Leaving "  << state_object_name << "::getState" << endl;
#endif
  return(state_);
}
