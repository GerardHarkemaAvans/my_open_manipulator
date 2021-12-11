#include "my_app/behavior/behavior.h"
#include "my_app/states/state_template.h"


behavior::behavior()
: node_handle(""),
  priv_node_handle("~")
{
  state_timer = node_handle.createTimer(ros::Duration(0.100)/*100ms*/,
                                          &behavior::stateCallback,
                                          this);
  state_timer.stop();

}

behavior::~behavior()
{
}

void behavior::stateCallback(const ros::TimerEvent&){

  switch(_state){
    case state_idle:
      break;
    case state_start:
      state1.onEnter();
      _state = state_1;
      break;
    case state_1:
      if(state1.execute() != state_template::busy){
        state1.onExit();
        state2.onEnter();
        _state = state_2;
      }
      break;
    case state_2:
      if(state2.execute() != state_template::busy){
        state2.onExit();
        _state = state_finshed;
      }
      break;
    case state_finshed:
      state_timer.stop();
      break;
    default:
      break;
  }
}

void behavior::onEnter(){
  _state = state_start;
}

void behavior::onEnter(input_keys_ &input_keys){
  user_data.input_keys = input_keys;
  _state = state_start;
}


behavior::output_keys_ behavior::onExit(){
  return(user_data.output_keys);
}

void behavior::abort(){
  _state = state_abort;
}

void behavior::reset(){
  _state = state_finshed;
}

behavior::outcomes behavior::execute(){
  return _outcomes;
}
