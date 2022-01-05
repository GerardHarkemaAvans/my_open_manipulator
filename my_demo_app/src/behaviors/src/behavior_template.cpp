/*******************************************************************************
File: behavior_template.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een behavior definitie welke gebruikt kan worden
bij een statemachine.
*******************************************************************************/
#include "../include/behavior_template.h"

#define DEBUG_ITEMS       DEBUG_NONE | DEBUG_BEHAVIORS_STATES//| DEBUG_BEHAVIORS | DEBUG_CUSTOM

behavior_template::behavior_template(const std::string& behavior_object_name, bool simple_execution_mode)
: node_handle(""),
  priv_node_handle("~")
{

  this->behavior_object_name = behavior_object_name;
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::construcor\n", behavior_object_name.c_str());
  this->simple_execution_mode = simple_execution_mode;

  /* Write here your code */
  state1 = new state_template("state1");
  state2 = new state_template("state2");

  if(!simple_execution_mode)
    state_timer = node_handle.createTimer(ros::Duration(0.100)/*100ms*/,
                                            &behavior_template::stateCallback,
                                            this);
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::construcor\n", behavior_object_name.c_str());
}

behavior_template::~behavior_template()
{

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::destructor\n", behavior_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::destructor\n", behavior_object_name.c_str());
}

void behavior_template::stateHandler(){
  switch(state){
    case state_idle:
      break;
    case state_start:
      state = state_1;
      break;
    case state_1:
      {
        state_template::input_keys_type input_key;
        input_key.dummy = 0;
        input_key.repeat_count = 3;
        state_template::output_keys_type output_key;
        DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Executing state_go_left");
        switch(state1->simpleEexecute(input_key, output_key)){
          case state_template::outcomes_busy:
            /* Do nothing */
            break;
          case state_template::outcomes_done:
            state = state_2;
            break;
          case state_template::outcomes_failed:
            state = state_failed;
            break;
        }
      }
      case state_2:
        {
          state_template::input_keys_type input_key;
          input_key.dummy = 0;
          input_key.repeat_count = 1;
          state_template::output_keys_type output_key;
          DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Executing state_go_left");
          switch(state2->simpleEexecute(input_key, output_key)){
            case state_template::outcomes_busy:
              /* Do nothing */
              break;
            case state_template::outcomes_done:
              state = state_finshed;
              break;
            case state_template::outcomes_failed:
              state = state_failed;
              break;
          }
        }
    case state_failed:
      DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Ending with outcome outcomes_failed");
      outcomes = outcomes_enum::outcomes_failed;
      state = state_wait_for_reset;
      break;
    case state_finshed:
      DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Ending with outcome finsihed");
      outcomes = outcomes_enum::outcomes_finshed;
      state = state_wait_for_reset;
      break;
    case state_wait_for_reset:
      break;
    default:
      break;
  }
}

void behavior_template::stateCallback(const ros::TimerEvent&){
  if(!simple_execution_mode)
    behavior_template::stateHandler();
}

behavior_template::status_enum behavior_template::onEnter(input_keys_type &input_keys){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::onEnter\n", behavior_object_name.c_str());
  behavior_template::status_enum return_code = status_succes;

  /* Write here your code */

  user_data.input_keys = input_keys;
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s: %s\n", behavior_object_name.c_str(), "Starting");
  state = state_start;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::onEnter\n", behavior_object_name.c_str());
  return(return_code);

}

behavior_template::outcomes_enum behavior_template::execute(){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::execute\n", behavior_object_name.c_str());

  /* Write here your code */
  if(simple_execution_mode) stateHandler();

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::execute\n", behavior_object_name.c_str());
  return(outcomes);
}

behavior_template::output_keys_type behavior_template::onExit(){
  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::onExit\n", behavior_object_name.c_str());

  /* Write here your code */

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::onExit\n", behavior_object_name.c_str());
  return(user_data.output_keys);
}

/* do not modify this member function */
behavior_template::outcomes_enum behavior_template::simpleEexecute(input_keys_type& input_keys, output_keys_type& output_keys){
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

#if 0 // not implemted yet
status_enum behavior_template::abort(){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::abort\n", behavior_object_name.c_str());

  /* Write here your code */

  state = state_abort;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::abort\n", behavior_object_name.c_str());
}

status_enum behavior_template::reset(){

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Entering %s::reset\n", behavior_object_name.c_str());

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS_STATES, "%s\n", "Resetiing");
  /* Write here your code */

  state = state_idle;

  DEBUG_PRINT(DEBUG_ITEMS & DEBUG_BEHAVIORS, "Leaving %s::reset\n", behavior_object_name.c_str());
}
#endif
