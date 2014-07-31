
#include "VrMagicRosBridge_host.h"

VrMagicRosBridge_host::VrMagicRosBridge_host() : _rate(0)
{
    _loopRate = 0;

    //get launch param:
    ros::NodeHandle privNh("~");
    std::string pub_name;
    std::string ip_smartcam;
    int port_smartcam;

    privNh.param("pub_name",pub_name,std::string("vrmagic_image"));
    privNh.param("ip_smartcam",ip_smartcam,std::string("192.168.3.100"));
    privNh.param<int>("port_smartcam",port_smartcam, 1234);

    _pubName_base = pub_name;
    _smartcamHandler = new ohm::VrMagicHandler_roshost(ip_smartcam, (unsigned int)port_smartcam);
}

VrMagicRosBridge_host::~VrMagicRosBridge_host()
{
    delete _smartcamHandler;
    delete _rate;
    //todo delete msg objs...
}

void VrMagicRosBridge_host::start(const unsigned int rate)
{
    delete _rate;
    _loopRate = rate;
    _rate = new ros::Rate(_loopRate);
    this->run();
}

void VrMagicRosBridge_host::run()
{

    //connect to camera
    ROS_INFO("Waiting for connenction to Smartcam ...");
    _smartcamHandler->connect();
    ROS_INFO("Connected to Smartcam: IP: %s, PORT: %d",_smartcamHandler->getIp().c_str(),_smartcamHandler->getPort());


    while(ros::ok())
    {
        //trigger smartcam
        if(_smartcamHandler->triggerImage() == 0)
        {
            if(_smartcamHandler->readImage(_imgSmarcam) == 0)
            {
                ROS_INFO("Read image");
                unsigned int id = _imgSmarcam.id;
                ROS_INFO("Read image id: %d",id);
                if(!this->provePublisherExist(id))
                {
                    ROS_INFO("Added Publischer with id: %d", id);
                    this->addPublisher(id);
                }
                ROS_INFO("Fill msg");
                this->setMsgImage(id);
                ROS_INFO("Publish msg");
                _publishers[id].publish(*_msgImgs[id]);
                ROS_INFO("Published... rdy");
            }
            else
            {
                ROS_ERROR("ERROR WHILE READING LAST IMAGE... now will continue");
                _rate->sleep();
            }
        }
        //only publishing
        //ros::spinOnce();
       //todo .... prove if need
       //_rate->sleep();
    }
}

int VrMagicRosBridge_host::addPublisher(unsigned int id)
{
    std::stringstream sstr;
    sstr << id;
    std::string id_str = sstr.str();

    //init publisher
    ros::Publisher pub = _nh.advertise<sensor_msgs::Image>(_pubName_base + id_str,1);
    _publishers.insert(std::make_pair(id, pub));

    //create ros-msg
    sensor_msgs::Image* msg = new sensor_msgs::Image;
    msg->width = 0;
    msg->height = 0;
    msg->step = 0;
    msg->header.frame_id = id_str;
    _msgImgs.insert(std::make_pair(id,msg));

    //create seq for ros-msg
    unsigned int seq = 0;
    _seq.insert(std::make_pair(id,seq));

    return 0;
}

int VrMagicRosBridge_host::removePublischer(unsigned int id)
{
    //todo
    return 0;
}

bool VrMagicRosBridge_host::provePublisherExist(unsigned int id)
{
    //prof if key exists
    if(_publishers.count(id) > 0)
    {
        return true;
    }
    return false;
}

void VrMagicRosBridge_host::setMsgImage(unsigned int id)
{
    //set ROS-Time and seq counter
    _msgImgs[id]->header.stamp = ros::Time::now();
    _msgImgs[id]->header.seq   = _seq[id]++;

    /**
     * @note it is not proved if XXX8 or XXX16 encoding has changed
     */
    if( _msgImgs[id]->width == _imgSmarcam.width &&
        _msgImgs[id]->height == _imgSmarcam.height &&
        _msgImgs[id]->step == _imgSmarcam.bytePerPixel * _imgSmarcam.width)
    {
        //copy new image in ros-msg
        memcpy(&_msgImgs[id]->data[0],_imgSmarcam.data,_imgSmarcam.dataSize);
        return;
    }


    //further code is executed if the image dimensions chaged.

    _msgImgs[id]->width = _imgSmarcam.width;
    _msgImgs[id]->height = _imgSmarcam.height;
    _msgImgs[id]->step = _imgSmarcam.bytePerPixel * _imgSmarcam.width;
    _msgImgs[id]->is_bigendian = 0;

    if(_imgSmarcam.channels == 1)
    {
        if(_imgSmarcam.bytePerPixel == 1)
            _msgImgs[id]->encoding = sensor_msgs::image_encodings::MONO8;
        else if(_imgSmarcam.bytePerPixel == 2)
            _msgImgs[id]->encoding = sensor_msgs::image_encodings::MONO16;
        else
        {
            ROS_ERROR("FALSE ENCODING GIVEN at 1 channel... will set to MONO8");
            _msgImgs[id]->encoding = sensor_msgs::image_encodings::MONO8;
        }
    }
    else if(_imgSmarcam.channels == 3)
    {
        if(_imgSmarcam.bytePerPixel == 3)
            _msgImgs[id]->encoding = sensor_msgs::image_encodings::BGR8;
        else if(_imgSmarcam.bytePerPixel == 6)
            _msgImgs[id]->encoding = sensor_msgs::image_encodings::BGR16;
        else
        {
            ROS_ERROR("FALSE ENCODING GIVEN at 3 channel... will set to BGR8");
            _msgImgs[id]->encoding = sensor_msgs::image_encodings::BGR8;
        }
    }
    else if(_imgSmarcam.channels == 4)
    {
        if(_imgSmarcam.bytePerPixel == 4)
            _msgImgs[id]->encoding = sensor_msgs::image_encodings::RGBA8;
        else if(_imgSmarcam.bytePerPixel == 8)
            _msgImgs[id]->encoding = sensor_msgs::image_encodings::RGBA16;
        else
        {
            ROS_ERROR("FALSE ENCODING GIVEN at 4 channel... will set to RGBA8");
            _msgImgs[id]->encoding = sensor_msgs::image_encodings::RGBA8;
        }
    }
    else
    {
        ROS_ERROR("FALSE ENCODING GIVEN: %d channel, %d Byte per Pixel... will set to RGB8", _imgSmarcam.channels, _imgSmarcam.bytePerPixel);
        _msgImgs[id]->encoding = sensor_msgs::image_encodings::RGB8;
    }

    ROS_INFO("selected Encoding: %s", _msgImgs[id]->encoding.c_str());

    ROS_INFO("resize data to: %d",_imgSmarcam.width * _imgSmarcam.height * _imgSmarcam.bytePerPixel);
    _msgImgs[id]->data.resize(_imgSmarcam.width * _imgSmarcam.height * _imgSmarcam.bytePerPixel);
    //copy image in ros-msg
    ROS_INFO("Copy data size: %d", _imgSmarcam.dataSize);
    memcpy(&_msgImgs[id]->data[0],_imgSmarcam.data,_imgSmarcam.dataSize);
    ROS_INFO("Copy data rdy");
}
