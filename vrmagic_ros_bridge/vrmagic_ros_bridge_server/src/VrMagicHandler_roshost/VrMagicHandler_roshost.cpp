/*
 * VrMagicHandlerroshost.cpp
 *
 *  Created on: 29.07.2014
 *      Author: m1ch1
 */

#include "VrMagicHandler_roshost.h"

namespace ohm
{

ohm::VrMagicHandler_roshost::VrMagicHandler_roshost(std::string ip_smartcam,
        unsigned int port_smartcam)
{
}

ohm::VrMagicHandler_roshost::~VrMagicHandler_roshost()
{
}

void ohm::VrMagicHandler_roshost::connect()
{
}

int ohm::VrMagicHandler_roshost::readImage(ImageType& image)
{
    return 0;
}

const std::string& ohm::VrMagicHandler_roshost::getIp() const
{
    return _ip;
}

unsigned int ohm::VrMagicHandler_roshost::getPort() const
{
    return _port;
}

} /* namespace ohm */


