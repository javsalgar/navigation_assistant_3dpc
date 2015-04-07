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
    double last_vj, last_wj, last_sj;
    double max_v_max;
    double last_updated_vj;
    double k_min, k_max;
    double distance_driven_prev, distance_driven;
    double dist_camera_axis, dist_axis_border;
    double distance_to_start_brakes;
    double C_a, far;
    double t_update_vj;
    int freq, ticks;
    bool reverse_driven;
    bool measure_time;

    std::string save_time_file_name_proc, save_time_file_name_comm;

    std::ofstream time_file_proc;
    std::ofstream time_file_comm;

    ros::Duration difTotal;
    ros::Publisher vel_pub_;
    ros::CallbackQueue my_callback_queue, my_callback_queue2;

    std::map<double,double> d_coll_1[MAX_THREADS], d_coll_2[MAX_THREADS];
    std::map<double,double> *p_d_coll_pre[MAX_THREADS], *p_d_coll_pre2[MAX_THREADS], *p_d_coll;

    std::map<boost::thread::id, int> thread_id_map;

    ros::Time last_update_time;

    bool verbose;

public:

    NavigationAssistant3DPC();

    void apply_acceleration(const ros::TimerEvent& t_event);
    void update_previous_velocity(const ros::TimerEvent& t_event);
    double interpolate_dcoll(double k, std::map<double,double> *m);
    void correct_joy_command (const geometry_msgs::TwistStampedConstPtr& input);
    void update_with_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input);
    void calculate_dcoll(int threadId, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input);
    double calculate_v_max(double sj);
    void send_joy_command(double v_max, double wj);
};

}
#endif // NAVIGATION_ASSISTANT_3DPC_H
