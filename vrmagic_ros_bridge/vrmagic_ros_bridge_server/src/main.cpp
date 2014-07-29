#include <ros/ros.h>


#include "VrMagicRosBridge_host.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vrmagic_ros_bridge_server_node");
    ros::NodeHandle nh("~");


    VrMagicRosBridge_host node;
    node.start(10);

}
