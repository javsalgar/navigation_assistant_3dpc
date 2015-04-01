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
#include "navigation_assistant_3dpc.h"

namespace navigation_assistant_3dpc {

NavigationAssistant3DPC::NavigationAssistant3DPC()
{
    ros::NodeHandle nh;
    ros::NodeHandle nh4("~");
    double interp_factor;

    for (int i = 0; i < MAX_THREADS; i++) {
        p_s_pre[i] = &s1[i];
        p_s_pre2[i] = &s2[i];
    }

    p_s = &s1[0];

    // Globals

    last_updated_vj = 0;
    last_vj = 0;
    last_wj = 0;
    distance_driven = 0;
    distance_driven_prev = 0;
    distance_to_stop = 0;
    previous_vj = 0.0;
    last_update_time = ros::Time::now();

    // Parameters

    nh4.param<double>("dist_camera_axis", dist_camera_axis, 0.21);
    nh4.param<int>("freq", freq, 10);
    nh4.param<double>("dist_axis_border", dist_axis_border, 0.20);
    nh4.param<double>("max_v_max", max_v_max, 0.5);
    nh4.param<double>("t_update_vj", t_update_vj, 0.8);
    nh4.param<std::string>("output_filename_proc", save_time_file_name_proc, "time_fk_shared_control_planner_proc.csv");
    nh4.param<std::string>("output_filename_comm", save_time_file_name_comm, "time_fk_shared_control_planner_comm.csv");
    nh4.param<bool>("measure_time", measure_time, "true");
    nh4.param<double>("a_max", a_max , -0.4);
    nh4.param<double>("far", far, 50);
    nh4.param<bool>("reverse_driven", reverse_driven, "false");
    nh4.param<double>("interp_factor", interp_factor, 0.7);
    nh4.param<bool>("verbose", verbose, "false");

    C_a = far * interp_factor;

    if (measure_time) {
        time_file_proc.open(save_time_file_name_proc.c_str(), std::fstream::out | std::fstream::app);
        time_file_comm.open(save_time_file_name_comm.c_str(), std::fstream::out | std::fstream::app);
        time_file_proc.precision(std::numeric_limits< double >::digits10);
        time_file_comm.precision(std::numeric_limits< double >::digits10);

    }

    ros::NodeHandle nh2;
    nh2.setCallbackQueue(&my_callback_queue);

    ros::NodeHandle nh3;
    nh3.setCallbackQueue(&my_callback_queue2);

    ros::AsyncSpinner spinner(1, &my_callback_queue);
    ros::AsyncSpinner spinner2(1, &my_callback_queue2);

    // Create a ROS subscriber for the input point cloud

    ros::Subscriber sub = nh.subscribe ("/points2", 1,  &NavigationAssistant3DPC::cloud_cb, this);

    ros::Subscriber sub2 = nh2.subscribe ("/cmd_vel_pre", 1, &NavigationAssistant3DPC::twist_cb, this);
    vel_pub_ = nh2.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Timer timer = nh3.createTimer(ros::Duration((double)1/freq),
                                       &NavigationAssistant3DPC::timerCallback, this);
    ros::Timer timer2 = nh3.createTimer(ros::Duration(t_update_vj),
                                       &NavigationAssistant3DPC::timerCallback2, this);
    // Spin

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

    // Loop to decrease velocity

    ticks = 0;

    spinner.start();
    spinner2.start();

}

void NavigationAssistant3DPC::timerCallback(const ros::TimerEvent& t_event) {

    ros::Duration t_dif = t_event.current_real - t_event.current_expected;
    double t = ((double)1/freq) + t_dif.toSec();
    int should_print = 0;

    if (distance_to_stop <= 0) {
        distance_driven += 1/2 * a_max * t * t + last_updated_vj * t;
    } else {
        distance_driven += last_updated_vj * t;
    }

    if (distance_driven > 0) {
        printf("LAST VJ: %lf, LAST SJ: %lf, DISTANCE DRIVEN + PREV: %lf, DISTANCE TO STOP: %lf",
               last_vj, last_sj, distance_driven + distance_driven_prev, distance_to_stop);
        should_print = 1;
    }

    if ((last_updated_vj > 0) && ((distance_driven + distance_driven_prev) > distance_to_stop)) {

        last_updated_vj += t * a_max;

        if (last_updated_vj <= 0) {
            last_updated_vj = 0;
        }

        geometry_msgs::Twist final_vel;


        if (reverse_driven) {

            final_vel.linear.x = -last_updated_vj;

        } else {
            final_vel.linear.x = last_updated_vj;
        }

        printf(" DECREASING SPEED to %lf", last_updated_vj);
        should_print = 1;
        vel_pub_.publish(final_vel);

    } else if (last_updated_vj < 0) {
        distance_driven += last_updated_vj * t;
    }

    if (should_print) {
        printf("\n");
    }
}


void NavigationAssistant3DPC::timerCallback2(const ros::TimerEvent& t_event) {
    previous_vj = last_vj;
}

double NavigationAssistant3DPC::interpolate(double k, std::map<double,double> *m) {

    std::map<double,double>::iterator it;

    double klow;
    double kupper;
    double res;

    if (m->size() == 0) {
        res = far;
    } else if (k < k_min) {

        res = (*m)[k_min] + fabs((k_min - k)) * C_a;
    } else if  (k > k_max) {

        res = (*m)[k_max] + fabs((k - k_max)) * C_a;
    } else {

        for (it=m->begin(); it!=m->end(); ++it) {
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

void NavigationAssistant3DPC::twist_cb (const geometry_msgs::TwistStampedConstPtr& input)
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


    sj = interpolate(kj, p_s) - dist_axis_border;

    geometry_msgs::Twist final_vel;
    final_vel.angular.z = wj;

    //ROS_INFO("Received twist with vj = %lf and wj = %lf, kj = %lf and sj = %lf amax = %.2lf",
    //	   vj, wj, kj, sj, a_max);

    if (sj > 0 && distance_driven <= 0) {
        v_max = sqrt(-2 * a_max) * sqrt(sj - distance_driven_prev) * 0.8;
    } else if (sj > 0 && distance_driven > 0 && (distance_driven + distance_driven_prev) < sj ) {
        v_max = sqrt(-2 * a_max) * sqrt(sj - distance_driven - distance_driven_prev) * 0.8;
    } else if (sj > 0 && distance_driven > 0 && (distance_driven + distance_driven_prev) >= sj ) {
        v_max = 0;
    } else {
        v_max = 0;
    }

    if (v_max >  max_v_max) {
        v_max = max_v_max;
    }

    if (vj > max_v_max) {
        vj = max_v_max;
    }

    if (v_max < vj) {
        final_vel.linear.x = v_max;
        distance_to_stop = 0;
    } else {
        final_vel.linear.x = vj;
        distance_to_stop = sj - ((vj * vj)/(-2 * a_max)) * 1.2
                - distance_driven - distance_driven_prev;
    }

    if (reverse_driven) {
        final_vel.linear.x = -final_vel.linear.x;
    }


    vel_pub_.publish(final_vel);

    last_wj = wj;
    last_vj = vj;

    last_updated_vj = last_vj;

    //   ROS_INFO("TWIST: vj: %lf --> %lf , wj = %lf, kj = %lf, sj = %lf, d = %lf, dp = %lf, dtoStop = %lf",
    //	   vj, final_vel.linear.x, final_vel.angular.z, kj, sj, distance_driven, distance_driven_prev, distance_to_stop);

    last_sj = sj;

    ticks = 0;

}

void NavigationAssistant3DPC::cloud_cb (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& input)
{
    double r;
    double x_input;
    double y_input;
    double xp;
    double yp;
    double k;
    double s;
    double xc;
    double k_min_aux = DBL_MAX, k_max_aux = -10000;
    double s_min_aux = DBL_MAX, s_max_aux = -10000, s_avg_aux = 0;

    boost::thread::id thisId = boost::this_thread::get_id();
    ros::Time now = ros::Time::now();
    ros::Duration dif;
    ros::Duration dif2;
    ros::Duration dif3;
    ros::Time pcl_header_time;
    pcl_header_time = pcl_header_time.fromNSec(input->header.stamp * 1000);
    int threadId;


    if (pcl_header_time < last_update_time) {
        return;
    } else {
        last_update_time = pcl_header_time;
    }


    if (thread_id_map.count(thisId) > 0) {
        threadId = thread_id_map[thisId];
    } else {
        threadId = thread_id_map.size();
        thread_id_map[thisId] = threadId;
    }

    if (measure_time) {
        ros::Duration comm_time = ros::Time::now() - pcl_header_time;
        time_file_comm << std::fixed << std::setprecision(4) << pcl_header_time.toSec() << " tcomm_{pclproc_planner} = " << std::fixed << std::setprecision(6) << comm_time.toSec() << std::endl;
        //ROS_INFO("Planner --> %lf --> %lf", pcl_header_time.toSec(), comm_time.toSec());
    }


    std::map<double,double> *aux;

    p_s_pre[threadId]->clear();

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

                if (k < k_min_aux) {
                    k_min_aux = k;
                }

                if (k > k_max_aux) {
                    k_max_aux = k;
                }

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



            if (p_s_pre[threadId]->count(k) > 0) {
                if (p_s_pre[threadId]->at(k) > s) {
                    (*p_s_pre[threadId])[k] = s;
                }
            } else {
                (*p_s_pre[threadId])[k] = s;
            }

            if (s > s_max_aux) {
                s_max_aux = s;
            }

            if (s < s_min_aux) {
                s_min_aux = s;
            }


            s_avg_aux += s;

        }
    }

    s_avg_aux = s_avg_aux/p_s_pre[threadId]->size();

    aux = p_s_pre[threadId];
    p_s_pre[threadId] = p_s_pre2[threadId];
    p_s_pre2[threadId] = aux;
    k_min = k_min_aux;
    k_max = k_max_aux;

    p_s = aux;

    dif2 = ros::Time::now() - pcl_header_time;

    if (previous_vj * dif2.toSec() > distance_driven) {
        distance_driven_prev = previous_vj * dif2.toSec();
    } else {
        distance_driven_prev = distance_driven;
    }

    distance_driven = 0;

    double kj;
    double sj;
    double v_max;

    kj = last_wj / last_vj;
    sj = interpolate(kj, p_s) - dist_axis_border;

    geometry_msgs::Twist final_vel;
    final_vel.angular.z = last_wj;

    if (sj > 0 && distance_driven <= 0) {
        v_max = sqrt(-2 * a_max) * sqrt(sj - distance_driven_prev) * 0.8;
    } else if (sj > 0 && distance_driven > 0 && (distance_driven + distance_driven_prev) < sj ) {
        v_max = sqrt(-2 * a_max) * sqrt(sj - distance_driven - distance_driven_prev) * 0.8;
    } else if (sj > 0 && distance_driven > 0 && (distance_driven + distance_driven_prev) >= sj ) {
        v_max = 0;
    } else {
        v_max = 0;
    }

    if (v_max >  max_v_max) {
        v_max = max_v_max;
    }

    double distance_to_stop_aux;

    if (v_max < last_updated_vj) {

        geometry_msgs::Twist final_vel;
        final_vel.angular.z = last_wj;
        final_vel.linear.x = v_max;

        last_vj = v_max;
        last_updated_vj = v_max;
        distance_to_stop = 0;

        if (reverse_driven) {
            final_vel.linear.x = -final_vel.linear.x;
        }

        if (verbose) {

            ROS_INFO("UPDATING twist  with vj = %lf and wj = %f",
                  final_vel.linear.x, final_vel.angular.z);
        }

        vel_pub_.publish(final_vel);

    } else {
        distance_to_stop -= distance_driven_prev;

        distance_to_stop_aux = sj - ((last_updated_vj * last_updated_vj)/(-2 * a_max)) * 1.2 - distance_driven - distance_driven_prev;
        if (distance_to_stop_aux < distance_to_stop) {
            distance_to_stop = distance_to_stop_aux;
        }
    }

    ticks = 0;
    last_sj = sj;


    if (verbose) {
        ROS_INFO("Information about s: MIN = %lf MAX = %lf AVG = %lf",s_min_aux, s_max_aux, s_avg_aux);
    }

    if (measure_time) {
        geometry_msgs::Twist final_vel2;
        ros::Duration comm_time = ros::Time::now() - pcl_header_time;
        time_file_proc << std::fixed << std::setprecision(4) << pcl_header_time.toSec() << " tproc_{planner} = " << std::fixed << std::setprecision(6) << comm_time.toSec() << std::endl;
        //      final_vel2.angular.z = 0;
        //     final_vel2.linear.x = 0;
        //     final_vel2.header.stamp = pcl_header_time;
        //      vel_pub_.publish(final_vel2);
    }
}

}
