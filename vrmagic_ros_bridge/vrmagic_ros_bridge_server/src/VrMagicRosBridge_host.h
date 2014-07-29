
#ifndef VRMAGICROSBRIDGE_SERVER_H_
#define VRMAGICROSBRIDGE_SERVER_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <sstream>
#include <map>

#include "VrMagicHandler_roshost/VrMagicHandler_roshost.h"

class VrMagicRosBridge_host
{

public:
    VrMagicRosBridge_host();
    virtual ~VrMagicRosBridge_host();

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

    int addPublisher(unsigned int id);

    int removePublischer(unsigned int id);

    bool provePublisherExist(unsigned int id);

    void setMsgImage(unsigned int id);

private:    //dataelements
    unsigned int _loopRate;
    std::string _pubName_base;

    ohm::ImageType _imgSmarcam;

    ohm::VrMagicHandler_roshost* _smartcamHandler;

    ros::Rate* _rate;
    ros::NodeHandle _nh;

    std::map<unsigned int, ros::Publisher> _publishers;
    std::map<unsigned int, sensor_msgs::Image*> _msgImgs;
    std::map<unsigned int, unsigned int> _seq;
};

#endif /* VRMAGICROSBRIDGE_SERVER_H_ */
