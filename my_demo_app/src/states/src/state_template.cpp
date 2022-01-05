/*******************************************************************************
File: state_template.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#include "../include/state_template.h"

#define DEBUG_ITEMS       DEBUG_NONE//| DEBUG_STATES | DEBUG_CUSTOM

state_template::state_template(const std::string& state_object_name/* define own paramters here*/)
: node_handle("")
{
  this->state_object_name = state_object_name;
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::construcor\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::construcor\n", state_object_name.c_str());
}

state_template::~state_template(){
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::destructor\n", state_object_name.c_str());

    /* Write here your code */

    DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::destructor\n", state_object_name.c_str());
}


state_template::status_enum state_template::onEnter(input_keys_type& input_keys){

  state_template::status_enum return_code = status_succes;
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onEnter\n", state_object_name.c_str());

  user_data.input_keys = input_keys;
  remaining_count = user_data.input_keys.repeat_count;

  /* Write here your code */

  state = state_template::running;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onEnter\n", state_object_name.c_str());
  return(return_code);
}


state_template::outcomes_enum state_template::execute(void){

  state_template::outcomes_enum return_value = outcomes_busy;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::execute\n", state_object_name.c_str());

  /* Write here your code */
  if(--remaining_count == 0){ /* Example */
    return_value = outcomes_done;
    state = state_template::idle;
  }

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::execute\n", state_object_name.c_str());
  return(return_value);
}

/* do not modify this member function */
state_template::outcomes_enum state_template::simpleEexecute(input_keys_type& input_keys, output_keys_type& output_keys){
  outcomes_enum return_value = outcomes_busy;

  switch(execution_state_){
    case execution_wait_for_start:
      {
        status_enum on_enter_status_ = onEnter(input_keys);
        if(on_enter_status_ != status_succes){
          return_value = outcomes_failed;
          break;
        }
        execution_state_ = execution_execute;
      }
      break;
    case execution_execute:
      execution_return_value = execute();
      if(execution_return_value != outcomes_busy){
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


state_template::output_keys_type state_template::onExit(){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onExit\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onExit\n", state_object_name.c_str());
  return(user_data.output_keys);
}

state_template::status_enum state_template::onStop(){

  state_template::status_enum return_code = status_succes;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onStop\n", state_object_name.c_str());

    /* Write here your code */

    DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onStop\n", state_object_name.c_str());
  return(return_code);
}

state_template::status_enum state_template::onPause(){

  state_template::status_enum return_code = status_succes;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onPause\n", state_object_name.c_str());

  /* Write here your code */

  state = state_template::paused;


  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onPause\n", state_object_name.c_str());
  return(return_code);
}

state_template::status_enum state_template::onResume(){

  state_template::status_enum return_code = status_succes;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::onResume\n", state_object_name.c_str());

  /* Write here your code */

  state = state_template::running;


  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::onResume\n", state_object_name.c_str());
  return(return_code);
}

state_template::state_enum state_template::getState(void){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Entering %s::getState\n", state_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_STATES, "Laeving %s::getState\n", state_object_name.c_str());
  return(state);
}
