#include "../include/px4_zed_bridge/px4_zed_bridge.h"


#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

namespace bridge {
    PX4_ZED_Bridge::PX4_ZED_Bridge(const ros::NodeHandle& nh) : nh_(nh) {
        odom_sub_ = nh_.subscribe<const nav_msgs::Odometry&>("/zedm/zed_node/odom_throttled", 10, &PX4_ZED_Bridge::odomCallback, this);
        mavros_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
        mavros_system_status_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("/mavros/companion_process/status", 1);

        last_callback_time = ros::Time::now();

        status_mutex_.reset(new std::mutex);

        worker_ = std::thread(&PX4_ZED_Bridge::publishSystemStatus, this);
    };

    PX4_ZED_Bridge::~PX4_ZED_Bridge() { }

    void PX4_ZED_Bridge::odomCallback(const nav_msgs::Odometry& msg) {
        // publish odometry msg
        geometry_msgs::PoseStamped output;
        output.header.stamp = ros::Time::now();
        output.header.frame_id = msg.header.frame_id;
        output.pose = msg.pose.pose;
        mavros_pose_pub_.publish(output);

        flag_first_pose_received = true;

        {   // lock mutex
            std::lock_guard<std::mutex> status_guard(*(status_mutex_));

            last_system_status_ = system_status_;

            if (msg.pose.covariance[0] > 0.1)
                system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;

            else if (0.01 <= msg.pose.covariance[0] <= 0.1)
                system_status_ = MAV_STATE::MAV_STATE_CRITICAL;

            else if (msg.pose.covariance[0] < 0.01)
                system_status_ = MAV_STATE::MAV_STATE_ACTIVE;

            else
                ROS_WARN_STREAM("Unexpected vision sensor variance");

            if (last_system_status_ != system_status_)
            {
                mavros_msgs::CompanionProcessStatus status_msg;

                status_msg.header.stamp = ros::Time::now();
                status_msg.component = 197; // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY

                status_msg.state = (int)system_status_;

                mavros_system_status_pub_.publish(status_msg);
            }
        }

        last_callback_time = ros::Time::now();
    }

    void PX4_ZED_Bridge::publishSystemStatus() {

        while (ros::ok())
        {
            ros::Duration(1).sleep();
            
            if (flag_first_pose_received == true)
            {
                if ((ros::Time::now()-last_callback_time) > ros::Duration(0.5))
                {
                    ROS_WARN_STREAM("Stopped receiving data from ZED Mini");
                    system_status_ = MAV_STATE::MAV_STATE_FLIGHT_TERMINATION;
                }

                mavros_msgs::CompanionProcessStatus status_msg;

                status_msg.header.stamp = ros::Time::now();
                status_msg.component = 197; // MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY

                {   // lock mutex
                    std::lock_guard<std::mutex> status_guard(*(status_mutex_));
                    status_msg.state = (int)system_status_;
                    mavros_system_status_pub_.publish(status_msg);
                }
            }
        }
    }
}
