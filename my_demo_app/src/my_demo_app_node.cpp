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

  // Start AsyncSpinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  behavior_main::input_keys_ input_key;
  behavior_main::output_keys_ output_key;

  std::cout << "my_app started" << std::endl;
  behavior_main my_behavior("my_behavior", true); // start behaivior in simple exuecution mode

  input_key.dummy = 0;
  bool abort_flag = false;

  while (ros::ok())
  {
    ros::spinOnce();
    switch(my_behavior.simpleEexecute(input_key, output_key)){
      case behavior_main::status_busy:
        // Do nothing
        break;
      case behavior_main::status_finshed:
        abort_flag = true;
        std::cout << "my_app finished" << std::endl;
        break;
      case behavior_main::status_failed:
        abort_flag = true;
        std::cout << "my_app finished with status failed" << std::endl;
        break;
    }
    if(abort_flag) break;
  }
  return 0;
}
