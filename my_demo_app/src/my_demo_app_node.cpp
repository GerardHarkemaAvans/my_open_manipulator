/*******************************************************************************
File: my_app_node.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Voorbeeld implementation van een state machine
*******************************************************************************/
#include "behaviors/include/behavior_main.h"

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "behavior_node");
  ros::NodeHandle node_handle("");

  behavior_main::input_keys_type input_key;
  behavior_main::output_keys_type output_key;

  input_key.dummy = 0;
  bool abort_flag = false;

  // Start AsyncSpinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::cout << "my_app started" << std::endl;
  behavior_main my_behavior("my_behavior", true); // start behaivior in simple execution mode

  while (ros::ok())
  {
    ros::spinOnce();
    switch(my_behavior.simpleEexecute(input_key, output_key)){
      case behavior_main::outcomes_busy:
        // Do nothing
        break;
      case behavior_main::outcomes_finshed:
        abort_flag = true;
        std::cout << "my_app finished" << std::endl;
        break;
      case behavior_main::outcomes_failed:
        abort_flag = true;
        std::cout << "my_app finished with status_enum outcomes_failed" << std::endl;
        break;
    }
    if(abort_flag) break;
  }
  return 0;
}
