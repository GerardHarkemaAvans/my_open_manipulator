/*******************************************************************************
File: state_template.h
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Implementation (template) voor een state definitie welke gebruikt kan worden
bij een behavior.
*******************************************************************************/
#include "my_app/states/state_template.h"

state_template::state_template(/* define own paramters here*/){
}

state_template::~state_template(){
}

state_template::status state_template::onEnter(void){

  state_ = state_template::running;
  return(success);
}

state_template::status state_template::onEnter(input_keys_* input_keys){

  state_ = state_template::running;
  user_data.input_keys = input_keys;
  return(success);
}


state_template::outcomes state_template::execute(void){

  state_ = state_template::idle;
  return(done);
}

state_template::outcomes state_template::execute(output_keys_* output_keys){

  //output_keys = NULL;
  state_ = state_template::idle;
  output_keys = user_data.output_keys;
  return(done);
}

state_template::status state_template::onExit(){

  state_ = state_template::idle;
  return(success);
}

state_template::status state_template::onStop(){

  state_ = state_template::idle;
  return(success);
}

state_template::status state_template::onPause(){

  state_ = state_template::paused;
  return(success);
}

state_template::status state_template::onResume(){

  state_ = state_template::running;
  return(success);
}

state_template::state state_template::getState(void){

  return(state_);
}
