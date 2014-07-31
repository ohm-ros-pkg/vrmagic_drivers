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
    _ip = ip_smartcam;
    _port = port_smartcam;

    _tcpClient = new apps::TCP(_ip, _port);

    _imgHeader = new OHM_HEADER_TYPE[ohm::HEADER_SIZE];
    _imgData = NULL;
    _currentDataSize = 0;
}

ohm::VrMagicHandler_roshost::~VrMagicHandler_roshost()
{
    delete _tcpClient;
    delete _imgHeader;
    if(_imgData != NULL)    delete _imgData;
}

void ohm::VrMagicHandler_roshost::connect()
{
    //wait for connection to smartcam
    _tcpClient->connectOnce();
}

int ohm::VrMagicHandler_roshost::readImage(ImageType& image)
{
    //read Header
    if(_tcpClient->readAll(_imgHeader, (unsigned int)ohm::HEADER_SIZE * sizeof(OHM_HEADER_TYPE)) != 0)
    {
        return -1;
    }
    unsigned int dataSize = _imgHeader[ohm::DATA_SIZE];

    //prove and allocate databuffer if needed
    if(dataSize > _currentDataSize)
    {//allocate new dataSize;
        if(_imgData != NULL)
            delete _imgData;
        //todo: if compressed data allocate more buffer for safety
        _imgData = new OHM_DATA_TYPE[dataSize];
    }

    //read image data
    if(_tcpClient->readAll(_imgData, dataSize) != 0)
    {
        return -1;
    }

    //set data in ImageType image
    image.id                = _imgHeader[ohm::ID];
    image.dataSize          = _imgHeader[ohm::DATA_SIZE];
    image.dataType          = (ohm::Data)_imgHeader[ohm::DATA_TYPE];
    image.compressionType   = (ohm::Compression)_imgHeader[ohm::COMPRESSION_TYPE];
    image.width             = _imgHeader[ohm::WIDTH];
    image.height            = _imgHeader[ohm::HEIGHT];
    image.channels          = _imgHeader[ohm::CHANNELS];
    image.bytePerPixel      = _imgHeader[ohm::BYTE_PER_PIXEL];
    //just set pointer to data
    image.data = _imgData;

    return 0;
}

int VrMagicHandler_roshost::triggerImage()
{
    OHM_TRIGGER_TYPE trigger = ohm::IMAGE_REQUEST;
    if(_tcpClient->write(&trigger,sizeof(OHM_TRIGGER_TYPE)) != 0)
    {
        return -1;
    }
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


