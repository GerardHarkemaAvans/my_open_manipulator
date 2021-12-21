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
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::construcor\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::construcor\n", state_object_name.c_str());
}

state_template::~state_template(){
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::destructor\n", state_object_name.c_str());

    /* Write here your code */

    DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::destructor\n", state_object_name.c_str());
}


state_template::status state_template::onEnter(input_keys_& input_keys){

  state_template::status return_code = success;
  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onEnter\n", state_object_name.c_str());

  user_data.input_keys = input_keys;
  remaining_count = user_data.input_keys.repeat_count;

  /* Write here your code */

  state_ = state_template::running;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::onEnter\n", state_object_name.c_str());
  return(return_code);
}


state_template::outcomes state_template::execute(void){

  state_template::outcomes return_value = busy;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::execute\n", state_object_name.c_str());

  /* Write here your code */
  if(--remaining_count == 0){ /* Example */
    return_value = done;
    state_ = state_template::idle;
  }

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::execute\n", state_object_name.c_str());
  return(return_value);
}

/* do not modify this member function */
state_template::outcomes state_template::simpleEexecute(input_keys_& input_keys, output_keys_& output_keys){
  outcomes return_value = busy;

  switch(execution_state_){
    case execution_wait_for_start:
      {
        status on_enter_status_ = onEnter(input_keys);
        if(on_enter_status_ != success){
          return_value = failed;
          break;
        }
        execution_state_ = execution_execute;
      }
      break;
    case execution_execute:
      execution_return_value = execute();
      if(execution_return_value != busy){
        execution_state_ = execution_exit;
      }
      break;
    case execution_exit:
      output_keys = onExit();
      return_value = execution_return_value;
      execution_state_ = execution_wait_for_start;
      break;
    default:
      break;
  }
  return(return_value);
}


state_template::output_keys_ state_template::onExit(){

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onExit\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::onExit\n", state_object_name.c_str());
  return(user_data.output_keys);
}

state_template::status state_template::onStop(){

  state_template::status return_code = success;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onStop\n", state_object_name.c_str());

    /* Write here your code */

    DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::onStop\n", state_object_name.c_str());
  return(return_code);
}

state_template::status state_template::onPause(){

  state_template::status return_code = success;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onPause\n", state_object_name.c_str());

  /* Write here your code */

  state_ = state_template::paused;


  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::onPause\n", state_object_name.c_str());
  return(return_code);
}

state_template::status state_template::onResume(){

  state_template::status return_code = success;

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::onResume\n", state_object_name.c_str());

  /* Write here your code */

  state_ = state_template::running;


  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::onResume\n", state_object_name.c_str());
  return(return_code);
}

state_template::state state_template::getState(void){

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Entering %s::getState\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_LEVEL >= DEBUG_LEVEL_1, "Laeving %s::getState\n", state_object_name.c_str());
  return(state_);
}
