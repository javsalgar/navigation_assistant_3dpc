#ifndef NAVIGATION_ASSISTANT_3DPC_H
#define NAVIGATION_ASSISTANT_3DPC_H

/*********************************************************************************
 *
 *  Navigation Assistant based on 3D Point Clouds
 *
 *  @author Javier J. Salmeron-Garcia (jsalmeron2@us.es)
 *
 *  University of Seville
 *
 *  This file is part of 3D Point Cloud Navigation Assistant
 *
 *  3D Point Cloud Navigation Assistant is free software:
 *  you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  3D Point Cloud Navigation Assistant is distributed in the hope
 *  that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with 3D Point Cloud Navigation Assistant.  If not, see <http://www.gnu.org/licenses/>.
 *
 * *****************************************************************************/

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

    /*
        Kinematics
    */

    /**
     * @brief a_max
     * Maximum desired acceleration
     */
    double a_max;

    /**
     * @brief previous_vj
     * Velocity of the robot in tk-1
     */
    double previous_vj;

    /**
     * @brief last_vj
     */
    double last_vj, last_wj, last_sj;

    /**
     * @brief max_v_max
     * Maximum desired velocity for the robot
     */
    double max_v_max;

    /**
     * @brief last_updated_vj
     * Last velocity used by the robot
     */
    double last_updated_vj;

    /**
     * @brief distance_driven_prev
     */
    double distance_driven_prev, distance_driven;

    /**
     * @brief Distance of the camera and the border to the axis O
     */
    double dist_camera_axis, dist_axis_border;

    /**
     * @brief Distance after which the braking system should be activated
     */
    double distance_to_start_brakes;

    /**
     * @brief C_a Constants used for interpolation
     */
    double C_a, far;

    /**
     * @brief reverse_driven Indicates whether the robot is being driven forward or backwards
     */
    bool reverse_driven;

    /*
     *  Times
     */

    /**
     * @brief t_update_vj Period to update the robot's velocity
     */
    double t_update_vj;

    /**
     * @brief freq Frequency of the system
     */
    int freq;

    /**
     * @brief last_update_time Timestamp of the last 3DPC received
     */
    ros::Time last_update_time;

    /*
     *  Profiling
     */

    /**
     * @brief measure_time Option to save time measurements (use for profiling)
     */
    bool measure_time;


    /**
     * @brief Filenames where the measurements are saved
     */
    std::string save_time_file_name_proc, save_time_file_name_comm;

    /**
     * @brief File writing buffers
     */
    std::ofstream time_file_proc, time_file_comm;

    /*
     * Publishers, Subscribers and Callback Queues
     */

    ros::Publisher vel_cmd_pub;

    ros::CallbackQueue joy_reception_callback_queue, timers_callback_queue;

    ros::AsyncSpinner *joy_reception_spinner;
    ros::AsyncSpinner *timers_spinner;

    ros::Timer *apply_acceleration_timer;
    ros::Timer *previous_vj_update_timer;

    ros::Subscriber subscriber_3dpc;
    ros::Subscriber subscriber_joy_command;

    /*
     * Maps
     */

    /**
     * @brief d_coll functions maps. A double-buffer is used and each thread has its own one.
     */
    std::map<double,double> d_coll_1[MAX_THREADS], d_coll_2[MAX_THREADS];
    std::map<double,double> *p_d_coll_pre[MAX_THREADS], *p_d_coll_pre2[MAX_THREADS], *p_d_coll;

    /**
     * @brief Minimum and maximum values of current k
     */
    double k_min = DBL_MAX, k_max = -DBL_MAX;

    /**
     * @brief thread_id_map Maps thread number to d_coll map
     */
    std::map<boost::thread::id, int> thread_id_map;

    /**
     * @brief verbose Allows writing information in the console
     */
    bool verbose;


    /**
     * @brief apply_acceleration Update the robot's velocity applying current velocity
     * @param t_event Timer information
     */
    void apply_acceleration(const ros::TimerEvent& t_event);

    /**
     * @brief update_previous_velocity After a certain time, store previous velocity for time correction
     * @param t_event Timer information
     */
    void update_previous_velocity(const ros::TimerEvent& t_event);

    /**
     * @brief interpolate_dcoll Obtains s=d_coll(k) using the available information
     * @param k Value obtained from the received joystick command
     * @param m d_coll function
     * @return Distance to a obstacle
     */
    double interpolate_dcoll(double k, std::map<double,double> *m);

    /**
     * @brief correct_joy_command Given a joystick command, correct the velocity for a safer navigation
     * @param input
     */
    void correct_joy_command (const geometry_msgs::TwistStampedConstPtr& input);

    /**
     * @brief update_with_cloud Callback when a new 3DPC arrives
     * @param input
     */
    void update_with_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input);

    /**
     * @brief calculate_dcoll Obtain new d_coll function using the 3DPC information
     * @param threadId
     * @param input
     */
    void calculate_dcoll(int threadId, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input);

    /**
     * @brief calculate_v_max Obtain the maximum allowed velocity
     * @param sj Distance to obstacle
     * @return  Maximum velocity v_max
     */
    double calculate_v_max(double sj);

    /**
     * @brief send_joy_command Send a velocity command to the robot
     * @param v_max linear velocity
     * @param wj angular velocity
     */
    void send_vel_command(double v_max, double wj);

public:

    NavigationAssistant3DPC();

    /**
     * @brief start Start navigation assistant
     */
    void start();

};

}
#endif // NAVIGATION_ASSISTANT_3DPC_H
