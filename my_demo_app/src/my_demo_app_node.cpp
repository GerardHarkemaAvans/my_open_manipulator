/*******************************************************************************
File: my_app_node.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Voorbeeld implementation van een state machine
*******************************************************************************/
#include "behaviors/include/behavior.h"

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "behavior_node");
  ros::NodeHandle node_handle("");

  // Start AsyncSpinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::cout << "my_app started" << std::endl;
  behavior my_behavior("my_behavior");

  {
    behavior::input_keys_ input_key;
    input_key.dummy = 0;
    my_behavior.onEnter(input_key);
  }

  while (ros::ok())
  {
    ros::spinOnce();
    if(my_behavior.execute()){
      my_behavior.onExit();
      break;
    }
  }

  return 0;
}
