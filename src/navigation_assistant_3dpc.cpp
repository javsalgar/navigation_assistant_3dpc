#include <ros/ros.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/callback_queue.h>
#include "pcl_ros/point_cloud.h"
#include <cstdlib>
#include <cmath>
#include <cfloat>
#include <limits>
#include "navigation_assistant_3dpc/navigation_assistant_3dpc.h"

namespace navigation_assistant_3dpc {

NavigationAssistant3DPC::NavigationAssistant3DPC()
{
    ros::NodeHandle nh_3dpc_reception;
    ros::NodeHandle nh2_joy_reception;
    ros::NodeHandle nh3_timers;
    ros::NodeHandle local_nh_parameters("~");
    double interp_factor;

    // Default parameters initialize

    p_d_coll = &d_coll_1[0];
    last_updated_vj = 0;
    last_vj = 0;
    last_wj = 0;
    distance_driven = 0;
    distance_driven_prev = 0;
    distance_to_start_brakes = 0;
    previous_vj = 0.0;
    last_update_time = ros::Time::now();


    for (int i = 0; i < MAX_THREADS; i++) {
        p_d_coll_pre[i] = &d_coll_1[i];
        p_d_coll_pre2[i] = &d_coll_2[i];
    }

    // Get Parameters

    local_nh_parameters.param<double>("dist_camera_axis", dist_camera_axis, 0.21);
    local_nh_parameters.param<int>("freq", freq, 10);
    local_nh_parameters.param<double>("dist_axis_border", dist_axis_border, 0.20);
    local_nh_parameters.param<double>("max_v_max", max_v_max, 0.5);
    local_nh_parameters.param<double>("t_update_vj", t_update_vj, 0.8);
    local_nh_parameters.param<std::string>("output_filename_proc", save_time_file_name_proc,
                           "time_fk_shared_control_planner_proc.csv");
    local_nh_parameters.param<std::string>("output_filename_comm", save_time_file_name_comm,
                           "time_fk_shared_control_planner_comm.csv");
    local_nh_parameters.param<bool>("measure_time", measure_time, "true");
    local_nh_parameters.param<double>("a_max", a_max , -0.4);
    local_nh_parameters.param<double>("far", far, 50);
    local_nh_parameters.param<bool>("reverse_driven", reverse_driven, "false");
    local_nh_parameters.param<double>("interp_factor", interp_factor, 0.7);
    local_nh_parameters.param<bool>("verbose", verbose, "false");

    C_a = far * interp_factor;

    if (measure_time) {
        time_file_proc.open(save_time_file_name_proc.c_str(), std::fstream::out | std::fstream::app);
        time_file_comm.open(save_time_file_name_comm.c_str(), std::fstream::out | std::fstream::app);
        time_file_proc.precision(std::numeric_limits< double >::digits10);
        time_file_comm.precision(std::numeric_limits< double >::digits10);

    }

    // Create subscription

    nh2_joy_reception.setCallbackQueue(&my_callback_queue);

    nh3_timers.setCallbackQueue(&my_callback_queue2);

    ros::AsyncSpinner spinner(1, &my_callback_queue);
    ros::AsyncSpinner spinner2(1, &my_callback_queue2);

    // Create a ROS subscriber for the input point cloud

    ros::Subscriber sub = nh_3dpc_reception.subscribe ("/points2", 1,  &NavigationAssistant3DPC::update_with_cloud, this);

    ros::Subscriber sub2 = nh2_joy_reception.subscribe ("/cmd_vel_pre", 1, &NavigationAssistant3DPC::correct_joy_command, this);
    vel_pub_ = nh2_joy_reception.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Timer timer = nh3_timers.createTimer(ros::Duration((double)1/freq),
                                       &NavigationAssistant3DPC::apply_acceleration, this);
    ros::Timer timer2 = nh3_timers.createTimer(ros::Duration(t_update_vj),
                                       &NavigationAssistant3DPC::update_previous_velocity, this);

    // Start spinners

    ROS_INFO_STREAM(std::endl << std::boolalpha <<
                    "==============================================================="    << std::endl <<
                    "Started 3DPC Navigation Assistant with the following parameters"    << std::endl <<
                    "==============================================================="    << std::endl <<
                    "Max braking acceleration (m/s^2)    : " << a_max                    << std::endl <<
                    "System status update frequency (Hz) : " << freq                     << std::endl <<
                    "Distance axis-border (m)            : " << dist_axis_border         << std::endl <<
                    "Maximum velocity limit              : " << max_v_max                << std::endl <<
                    "Linear velocity register period (s) : " << t_update_vj              << std::endl <<
                    "Far value (m)                       : " << far                      << std::endl <<
                    "Measure time                        : " << measure_time             << std::endl <<
                    "Time File (Proc)                    : " << save_time_file_name_proc << std::endl <<
                    "Time File (Comm)                    : " << save_time_file_name_comm << std::endl <<
                    "Drive reverse                       : " << reverse_driven           << std::endl <<
                    "Interpolation Factor                : " << interp_factor            << std::endl <<
                    "=============================================================="     << std::endl);


    ticks = 0;

    spinner.start();
    spinner2.start();

}

void NavigationAssistant3DPC::apply_acceleration(const ros::TimerEvent& t_event) {

    ros::Duration t_dif = t_event.current_real - t_event.current_expected;
    double t = ((double)1/freq) + t_dif.toSec();

    if (distance_to_start_brakes <= 0) { // We are applying brakes
        distance_driven += 1/2 * a_max * t * t + last_updated_vj * t;
    } else { // The speed is maintained
        distance_driven += last_updated_vj * t;
    }

    if ((last_updated_vj > 0) && ((distance_driven + distance_driven_prev) > distance_to_start_brakes)) {

        last_updated_vj += t * a_max;

        if (last_updated_vj <= 0) {
            last_updated_vj = 0;
        }

        send_joy_command(last_updated_vj, last_wj);

        if (verbose) {
            ROS_INFO(" DECREASING SPEED to %lf", last_updated_vj);
        }

    }
}


void NavigationAssistant3DPC::update_previous_velocity(const ros::TimerEvent& t_event) {

    (void)t_event;
    previous_vj = last_vj;
}

double NavigationAssistant3DPC::interpolate_dcoll(double k, std::map<double,double> *m) {

    double klow, kupper, res;

    if (m->size() == 0) {
        res = far;
    } else if (k < k_min) {
        res = (*m)[k_min] + fabs((k_min - k)) * C_a;
    } else if  (k > k_max) {
        res = (*m)[k_max] + fabs((k - k_max)) * C_a;
    } else {

        for (std::map<double,double>::iterator it=m->begin(); it!=m->end(); ++it) {
            if (it->first < k) {
                klow = it->first;
            } else if (it->first == k) {
                res =  it->second;
            } else if (it->first > k) {
                kupper = it->first;
                res = ((*m)[kupper] + (*m)[klow]) / 2
                        + fabs((kupper - klow)) * C_a;
                break;
            }
        }
    }
    return res;
}



void NavigationAssistant3DPC::correct_joy_command (const geometry_msgs::TwistStampedConstPtr& input)
{
    double kj, sj, v_max, vj, wj;

    if (input->twist.angular.z == -0.0) {
        wj = 0.0;
    } else {
        wj = input->twist.angular.z;
    }

    if (input->twist.linear.x == -0.0) {
        vj = 0.0;
    } else {
        vj = input->twist.linear.x;
    }

    kj = wj / vj;
    sj = interpolate_dcoll(kj, p_d_coll) - dist_axis_border;

    v_max = calculate_v_max(sj);
    send_joy_command(v_max, wj);

    if (v_max < vj) {
        distance_to_start_brakes = 0;
    } else {
        distance_to_start_brakes = sj - ((vj * vj)/(-2 * a_max)) * 1.2
                - distance_driven - distance_driven_prev;
    }

    last_wj = wj;
    last_vj = vj;
    last_updated_vj = last_vj;

    last_sj = sj;
    ticks = 0;

}

void NavigationAssistant3DPC::calculate_dcoll(int threadId, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
    std::map<double,double> *aux;

    double x_input, y_input, xp, xc, yp, s, r, k;

    p_d_coll_pre[threadId]->clear();

    for (unsigned int i = 0; i < input->points.size(); i++) {
        if (input->points[i].x == input->points[i].x &&
                input->points[i].z == input->points[i].z) {

            x_input = input->points[i].x;
            y_input = input->points[i].z;

            if (x_input == -0.0) {
                xp = 0.0;
            } else {
                xp = x_input;
            }

            if (xp == 0.0) {
                k = 0.0;
                s = y_input - dist_camera_axis;
                if (s <= 0.0) {
                    s = 0.0;
                }
            } else {
                yp = y_input - dist_camera_axis;
                if (yp <= 0.0) {
                    yp = 0.0;
                }
                xc = (xp * xp + yp * yp)/(2 * xp);
                if (xc < 0) {
                    r = -xc;
                } else {
                    r = xc;
                }
                k = - 2 * xp / (xp * xp + yp * yp);

                if (xp > 0) {
                    if (xp > xc) {
                        s = r * (PI - atan2(yp, xp - xc));

                    } else {
                        s = r * atan2(yp, xc - xp);

                    }
                } else {
                    if (xp > xc) {
                        s = r * (PI  - atan2(yp, xc - xp));
                    } else {
                        s = r * (atan2(yp, xp - xc));
                    }
                }
            }

            if (p_d_coll_pre[threadId]->count(k) > 0) {
                if (p_d_coll_pre[threadId]->at(k) > s) {
                    (*p_d_coll_pre[threadId])[k] = s;
                }
            } else {
                (*p_d_coll_pre[threadId])[k] = s;
            }
        }
    }

    aux = p_d_coll_pre[threadId];
    p_d_coll_pre[threadId] = p_d_coll_pre2[threadId];
    p_d_coll_pre2[threadId] = aux;

    p_d_coll = aux;
}

double NavigationAssistant3DPC::calculate_v_max(double sj) {

    double v_max;

    double total_driven = distance_driven + distance_driven_prev;

    if (sj > 0 && distance_driven <= sj) {
        v_max = sqrt(-2 * a_max) * sqrt(sj - total_driven) * 0.8;
    } else {
        v_max = 0;
    }

    if (v_max >  max_v_max) {
        v_max = max_v_max;
    }

    return v_max;

}

void NavigationAssistant3DPC::send_joy_command(double v_max, double wj)
{
    geometry_msgs::Twist final_vel;
    final_vel.angular.z = wj;
    final_vel.linear.x = v_max;
    last_vj = v_max;
    last_updated_vj = v_max;

    if (reverse_driven) {
        final_vel.linear.x = -final_vel.linear.x;
    }

    vel_pub_.publish(final_vel);
}


void NavigationAssistant3DPC::update_with_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
    double kj;
    double sj;
    double v_max;

    boost::thread::id this_thread_id = boost::this_thread::get_id();
    ros::Duration pcl_delay;
    ros::Time pcl_header_time;
    int this_thread_dcoll_index;
    double distance_to_start_brakes_aux;

    pcl_header_time = pcl_header_time.fromNSec(input->header.stamp * 1000);

    if (pcl_header_time < last_update_time) {
        return;
    } else {
        last_update_time = pcl_header_time;
    }

    if (thread_id_map.count(this_thread_id) > 0) {
        this_thread_dcoll_index = thread_id_map[this_thread_id];
    } else {
        this_thread_dcoll_index = thread_id_map.size();
        thread_id_map[this_thread_id] = this_thread_dcoll_index;
    }

    if (measure_time) {
        ros::Duration comm_time = ros::Time::now() - pcl_header_time;
        time_file_comm << std::fixed << std::setprecision(4) <<
                          pcl_header_time.toSec() << " tcomm_{pclproc_planner} = "
                       << std::fixed << std::setprecision(6) << comm_time.toSec() << std::endl;
    }

    calculate_dcoll(this_thread_dcoll_index, input);

    pcl_delay = ros::Time::now() - pcl_header_time;

    distance_driven_prev = distance_driven;
    distance_driven = 0;

    kj = last_wj / last_vj;
    sj = interpolate_dcoll(kj, p_d_coll) - dist_axis_border;

    v_max = calculate_v_max(sj);

    if (v_max < last_updated_vj) {
        send_joy_command(v_max, last_wj);
        distance_to_start_brakes = 0;
    } else {        
        distance_to_start_brakes -= distance_driven_prev;

        distance_to_start_brakes_aux = sj - ((last_updated_vj * last_updated_vj)/(-2 * a_max)) * 1.2
                - distance_driven_prev;

        if (distance_to_start_brakes_aux < distance_to_start_brakes) {
            distance_to_start_brakes = distance_to_start_brakes_aux;
        }
    }

    if (measure_time) {
        geometry_msgs::Twist final_vel2;
        ros::Duration comm_time = ros::Time::now() - pcl_header_time;
        time_file_proc << std::fixed << std::setprecision(4) <<
                          pcl_header_time.toSec() << " tproc_{planner} = " <<
                          std::fixed << std::setprecision(6) << comm_time.toSec() << std::endl;
    }
}

}
