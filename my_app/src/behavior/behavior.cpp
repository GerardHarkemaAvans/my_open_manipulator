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
        state1.onEnter();
        _state = state_2;
      }
      break;
    case state_2:
      if(state1.execute() != state_template::busy){
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

void behavior::start(){
  _state = state_start;
  state_timer.start();
}


void behavior::abort(){
  state_timer.stop();
  _state = state_abort;
}

void behavior::reset(){
  _state = state_finshed;
}

behavior::status behavior::getStatus(){
  return _status;
}
