#!/usr/bin/python3

import rospy

from nav_msgs.msg import Odometry
from mavros_msgs.msg import CompanionProcessStatus
from enum import Enum


class MAV_STATE(Enum):
    MAV_STATE_UNINIT = 0
    MAV_STATE_BOOT = 1
    MAV_STATE_CALIBRATING = 2
    MAV_STATE_STANDBY = 3
    MAV_STATE_ACTIVE = 4
    MAV_STATE_CRITICAL = 5
    MAV_STATE_EMERGENCY = 6
    MAV_STATE_POWEROFF = 7
    MAV_STATE_FLIGHT_TERMINATION = 8


class OdomBridge:
    def __init__(self):
        rospy.Subscriber("/zedm/zed_node/odom", Odometry, self.odom_callback)
        self.mavros_odom_pub = rospy.Publisher("/mavros/odometry/out", Odometry, queue_size=10)
        self.mavros_system_status_pub = rospy.Publisher("/mavros/companion_process/status", CompanionProcessStatus,
                                                         queue_size=1)

        self.system_status = MAV_STATE.MAV_STATE_UNINIT
        self.last_system_status = MAV_STATE.MAV_STATE_UNINIT
        # rospy.spin()

    def odom_callback(self, msg):
        # Publish odometry data
        output = msg
        output.header.frame_id = msg.header.frame_id
        output.child_frame_id = msg.child_frame_id
        self.mavros_odom_pub.publish(output)
        # print(msg)

        # Publish system status
        self.last_system_status = self.system_status
        if msg.pose.covariance[0] > 0.1:
            self.system_status = MAV_STATE.MAV_STATE_FLIGHT_TERMINATION
        elif msg.pose.covariance[0] == 0.1:
            self.system_status = MAV_STATE.MAV_STATE_CRITICAL
        elif msg.pose.covariance[0] < 0.01:
            self.system_status = MAV_STATE.MAV_STATE_ACTIVE
        else:
            rospy.logwarn("Unexpected vision sensor variance")

        if self.last_system_status != self.system_status:
            self.publish_system_status()

    def publish_system_status(self):
        status_msg = CompanionProcessStatus()

        status_msg.header.stamp = rospy.Time.now()
        status_msg.component = 197  # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
        status_msg.state = self.system_status.value

        self.mavros_system_status_pub.publish(status_msg)


if __name__ == '__main__':
    rospy.init_node('px4_zed_bridge')
    bridge = OdomBridge()
    #rospy.spin()
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        bridge.publish_system_status()
        r.sleep()
