#include "../include/px4_zed_bridge/px4_zed_bridge.h"

using namespace bridge;

int main(int argc, char** argv) {
    ros::init(argc, argv, "PX4_ZED_Bridge_node");
    ros::NodeHandle nh("~");
    PX4_ZED_Bridge Bridge(nh);

    ros::spin();

    Bridge.worker_.join();

    return 0;
}