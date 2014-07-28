#include <ros/ros.h>


#include "VrMagicRosBridge_server.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vrmagic_ros_bridge_server_node");
    ros::NodeHandle nh("~");


    VrMagicRosBridge_server node;
    node.start(10);

}
