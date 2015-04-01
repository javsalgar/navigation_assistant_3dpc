#include <ros/ros.h>
#include "navigation_assistant_3dpc.h"

int main (int argc, char** argv)
{
    // Initialize ROS

    ros::init (argc, argv, "navigation_assistant");

    navigation_assistant_3dpc::NavigationAssistant3DPC nav_assist;

    ros::MultiThreadedSpinner spinnerGen(0);
    spinnerGen.spin();
}

