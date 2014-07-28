
#include "VrMagicRosBridge_server.h"

VrMagicRosBridge_server::VrMagicRosBridge_server() : _rate(0)
{
    _loopRate = 0;

    //init publisher
    //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);

    //inti subscriber
    //_sub = _nh.subscribe("topicName", 1, &Template::subCallback, this);
}

VrMagicRosBridge_server::~VrMagicRosBridge_server()
{
    delete _rate;
}

void VrMagicRosBridge_server::start(const unsigned int rate)
{
    delete _rate;
    _loopRate = rate;
    _rate = new ros::Rate(_loopRate);
    this->run();
}

void VrMagicRosBridge_server::run()
{
    unsigned int cnt = 0;

    while(ros::ok())
    {
        //do stuff;

        //publish data;
        //_pub.publish(msg);

        ros::spinOnce();
        _rate->sleep();
    }
}

