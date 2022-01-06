/*******************************************************************************
File: my_app_node.cpp
Version: 1.0
Authour: G A Harkema (ga.harkeme@avans.nl)
Date: december 2021
Purpose:
Voorbeeld implementation van een state machine
*******************************************************************************/
#include "behaviors/include/behavior_template.h"

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "behavior_node");
  ros::NodeHandle node_handle("");
  std::cout << "my_app started" << std::endl;
  behavior_template my_behavior("my_behavior", true);

  behavior_template::input_keys_type input_key;
  input_key.dummy = 0;
  behavior_template::output_keys_type output_key;

  bool abort_flag = false;

  while (ros::ok())
  {
    ros::spinOnce();
    switch(my_behavior.simpleEexecute(input_key, output_key)){
      case behavior_template::outcomes_busy:
        // Do nothing
        break;
      case behavior_template::outcomes_finshed:
        abort_flag = true;
        std::cout << "my_app finished" << std::endl;
        break;
      case behavior_template::outcomes_failed:
        abort_flag = true;
        std::cout << "my_app finished with status outcomes_faild" << std::endl;
        break;
    }
    if(abort_flag) break;
  }

  return 0;
}
