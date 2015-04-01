#ifndef NAVIGATION_ASSISTANT_3DPC_H
#define NAVIGATION_ASSISTANT_3DPC_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/callback_queue.h>
#include <cstdlib>
#include "pcl_ros/point_cloud.h"
#include <cmath>
#include <cfloat>
#include <limits>
#include <iostream>
#include <string>
#include <fstream>



#define PI 3.1416
#define MAX_THREADS 64


namespace navigation_assistant_3dpc {

class NavigationAssistant3DPC
{

private:

    double a_max;
    double previous_vj;
    double last_vj, last_wj,last_sj;
    double max_v_max;
    double last_updated_vj;
    double k_min, k_max;
    double distance_driven_prev, distance_driven;
    double dist_camera_axis, dist_axis_border;
    double distance_to_stop;
    double C_a, far;
    double t_update_vj;
    int freq, ticks;
    bool reverse_driven;
    bool measure_time;

    std::string save_time_file_name_proc;
    std::string save_time_file_name_comm;

    std::ofstream time_file_proc;
    std::ofstream time_file_comm;

    ros::Duration difTotal;
    ros::Publisher vel_pub_;
    ros::CallbackQueue my_callback_queue;
    ros::CallbackQueue my_callback_queue2;

    std::map<double,double> s1[MAX_THREADS];
    std::map<double,double> s2[MAX_THREADS];
    std::map<double,double> *p_s_pre[MAX_THREADS];
    std::map<double,double> *p_s_pre2[MAX_THREADS];

    std::map<double,double> *p_s;
    std::map<boost::thread::id, int> thread_id_map;

    ros::Time last_update_time;

    bool verbose;

public:

    NavigationAssistant3DPC();

    void timerCallback(const ros::TimerEvent& t_event);
    void timerCallback2(const ros::TimerEvent& t_event);
    double interpolate(double k, std::map<double,double> *m);
    void twist_cb (const geometry_msgs::TwistStampedConstPtr& input);
    void cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input);
};

}
#endif // NAVIGATION_ASSISTANT_3DPC_H
