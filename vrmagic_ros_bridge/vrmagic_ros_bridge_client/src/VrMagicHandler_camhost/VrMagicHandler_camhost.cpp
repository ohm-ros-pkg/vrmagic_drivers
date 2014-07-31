/*
 * VrMagicHandlercamhost.cpp
 *
 *  Created on: 29.07.2014
 *      Author: m1ch1
 */

#include "VrMagicHandler_camhost.h"

namespace ohm
{

VrMagicHandler_camhost::VrMagicHandler_camhost(unsigned int port_smartcam)
{
    _port = port_smartcam;

    _tcpServer = new apps::TCP(_port);

    _imgHeader = new OHM_HEADER_TYPE[ohm::HEADER_SIZE];
}

VrMagicHandler_camhost::~VrMagicHandler_camhost()
{
    delete _tcpServer;
    delete _imgHeader;
}

void VrMagicHandler_camhost::connect()
{
    _tcpServer->connectOnce();
}

int VrMagicHandler_camhost::writeImage(ohm::ImageType& image)
{
    //fill Header
    _imgHeader[ohm::ID]               = image.id;
    _imgHeader[ohm::DATA_SIZE]        = image.dataSize;
    _imgHeader[ohm::DATA_TYPE]        = image.dataType;
    _imgHeader[ohm::COMPRESSION_TYPE] = image.compressionType;
    _imgHeader[ohm::WIDTH]            = image.width;
    _imgHeader[ohm::HEIGHT]           = image.height;
    _imgHeader[ohm::CHANNELS]         = image.channels;
    _imgHeader[ohm::BYTE_PER_PIXEL]   = image.bytePerPixel;

    //write img-Header
    if(_tcpServer->write(_imgHeader,(unsigned int)ohm::HEADER_SIZE * sizeof(OHM_HEADER_TYPE)) != 0)
    {
        return -1;
    }

    //write img-data
    if(_tcpServer->write(image.data, image.dataSize) != 0)
    {
        return -1;
    }

    return 0;
}

unsigned int VrMagicHandler_camhost::getPort() const
{
    return _port;
}

} /* namespace ohm */
