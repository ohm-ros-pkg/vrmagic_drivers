
#ifndef VRMAGICROSBRIDGE_SERVER_H_
#define VRMAGICROSBRIDGE_SERVER_H_

#include <ros/ros.h>
#include "TCP/TCP.h"

class VrMagicRosBridge_server
{
private:    //dataelements
    unsigned int _loopRate;

    ros::Rate* _rate;
    ros::NodeHandle _nh;

    ros::Publisher _pub;
    ros::Subscriber _sub;
public:
    VrMagicRosBridge_server();
    virtual ~VrMagicRosBridge_server();

    /**
     * @fn void start(const unsigned int frames = 10)
     *
     * @brief
     *
     *
     * @param[in] const unsigned int rate  ->  rate of the working loop in [1/s]
     *
     *
     * @return  void
     */
    void start(const unsigned int rate = 10);

private:    //functions

    /**
     * @fn void run()
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void run();

    //void subCallback(const ROS_PACK::MESSAGE& msg);
};

#endif /* VRMAGICROSBRIDGE_SERVER_H_ */
