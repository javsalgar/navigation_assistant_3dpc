#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "navigation_assistant_3dpc/navigation_assistant_3dpc.h"

int main(int argc, char** argv) {
    
  ros::init(argc, argv, "navigation_assistant");
  
  navigation_assistant_3dpc::NavigationAssistant3DPC nav_assistant;
  nav_assistant.start();

  ros::spin();  
}
