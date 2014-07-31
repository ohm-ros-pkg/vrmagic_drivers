/*
 * main .cpp
 *
 *  Created on: 30.07.2014
 *      Author: m1ch1
 */




#include <iostream>
#include <cstdlib>
#include <cstring>
#include <boost/thread.hpp>

#include <vrmusbcam2.h>
#include "../VrMagicHandler_camhost/VrMagicHandler_camhost.h"

#define PORT    1234

bool IMAGE_REQUEST = false;
boost::mutex mutex;

void thread(ohm::VrMagicHandler_camhost* rosBrige)
{
    while(1)
    {
        rosBrige->wait();
        mutex.lock();
        IMAGE_REQUEST = true;
        mutex.unlock();
    }
}

int main(int argc, char *argv[])
{
//===================================================================================================================
    ohm::VrMagicHandler_camhost _rosBrige(PORT);
    ohm::ImageType _rosImage;
//===================================================================================================================
    // at first, be sure to call VRmUsbCamCleanup() at exit, even in case
    // of an error
    atexit(VRmUsbCamCleanup);


    //-- scan for devices --

    //update
    VRmUsbCamUpdateDeviceKeyList();
    VRmDWORD size = 0;
    VRmUsbCamGetDeviceKeyListSize(&size);

    std::cout << "Found "<< size << " devices" << std::endl;

    if(size != 1)
    {
        std::cout << "found more or none devie..." << std::endl;
        std::cout << "this programm ist just for hanlde with one device pleace change code..." << std::endl;
        std::cout << "will exit now...." << std::endl;
        exit(EXIT_FAILURE);
    }

    VRmUsbCamDevice device=0;
    VRmDeviceKey* p_device_key=0;
    //open device
    VRmUsbCamGetDeviceKeyListEntry(0, &p_device_key);
    if(!p_device_key->m_busy)
    {
        VRmUsbCamOpenDevice(p_device_key, &device);
    }

    // display error when no camera has been found
    if(!device)
    {
        std::cerr << "No suitable VRmagic device found!" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Open deivce succesful" << std::endl;




    //init camera
    VRmImageFormat target_format;
    VRmDWORD port = 0; //id of Sensor //for multisensorunits

    VRmBOOL supported;
    // check number of connected sensors
    VRmDWORD num_sensorports=0;
    VRmUsbCamGetSensorPortListSize(device, &num_sensorports);

    // for this demo we switch off all connected sensor but the first one in the port list
    for(VRmDWORD ii=0; ii<num_sensorports;ii++)
    {
        VRmUsbCamGetSensorPortListEntry(device, ii, &port);
        std::cout << "found PORT: " << port << std::endl;

        // on single sensor devices this property does not exist
        VRmPropId sensor_enable = (VRmPropId)(VRM_PROPID_GRAB_SENSOR_ENABLE_1_B-1+port);
        VRmUsbCamGetPropertySupported(device, sensor_enable, &supported);
        if(supported)
        {
            //enable first sensor in port list
            VRmBOOL enable = 1;
            if(ii)
                enable = 0;
            VRmUsbCamSetPropertyValueB(device, sensor_enable, &enable);
        }
    }

    //now get the first sensor port
    VRmUsbCamGetSensorPortListEntry(device, 0, &port);
    std::cout << "PORT: " << port << " is used" << std::endl;

    //check if exposure time can be set
    VRmUsbCamGetPropertySupported(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &supported);
    float value=0.f;
    VRmUsbCamGetPropertyValueF(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &value);
    std::cout << "ExposureTime: " << value << "ms, changeable: "<< (supported ? "true" : "false") << std::endl;
    if(supported)
    {
        value=10.f;
        // uncomment the following lines to change exposure time to 25ms
        // when camera supports this feature
        VRmUsbCamSetPropertyValueF(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &value);
        std::cout << "ExposureTime changed to : " << value << "ms" << std::endl;
    }

    //prepare imageformat
    VRmDWORD number_of_target_formats, i;
    VRmUsbCamGetTargetFormatListSizeEx2( device, port, &number_of_target_formats );
    for ( i = 0; i < number_of_target_formats; ++i )
    {
        VRmUsbCamGetTargetFormatListEntryEx2(device, port, i, &target_format);
        if(target_format.m_color_format == VRM_BGR_3X8)
        {
            std::cout << "BGR8 found !" << std::endl;
            break;
        }
    }

//===================================================================================================================
    std::cout << "------------------------------------------------------------" << std::endl;
    std::cout << "--- Waiting for ROS-HOST... --------------------------------" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;
    _rosBrige.connect();
    std::cout << "Connected to ROS-HOST" << std::endl;
//===================================================================================================================


    //start grab
    VRmUsbCamResetFrameCounter(device);
    VRmUsbCamStart(device);

    VRmImage* p_target_img = 0;
    VRmUsbCamNewImage(&p_target_img,target_format);

    std::cout << "target img data: "
    << "color_format "  << p_target_img->m_image_format.m_color_format
    << " width " << p_target_img->m_image_format.m_width
    << " heigt "  << p_target_img->m_image_format.m_height
    << " modifier "    << p_target_img->m_image_format.m_image_modifier << std::endl;
    
    //p_target_img->m_pitch = 3 * p_target_img->m_image_format.m_width;
    std::cout << "target img pitch: " << p_target_img->m_pitch << std::endl;

//===================================================================================================================
    //set rosimage
    _rosImage.id                = port;
    _rosImage.dataSize          = p_target_img->m_image_format.m_width * 3 * p_target_img->m_image_format.m_height;
    _rosImage.dataType          = ohm::IMAGE;
    _rosImage.compressionType   = ohm::NONE;
    _rosImage.width             = p_target_img->m_image_format.m_width;
    _rosImage.height            = p_target_img->m_image_format.m_height;
    _rosImage.channels          = 3;
    _rosImage.bytePerPixel      = 3;
    _rosImage.data = NULL;

    OHM_DATA_TYPE* _rosImgBuffer = new OHM_DATA_TYPE[_rosImage.dataSize];


//===================================================================================================================

    //start thread for waiting:
    boost::thread _thread(thread, &_rosBrige);

    bool err_loop = false;
    //source img
    VRmImage* p_source_img = 0;

    while(!err_loop)
    {
        VRmDWORD frames_dropped;
        if(!VRmUsbCamLockNextImageEx(device,port,&p_source_img,&frames_dropped))
        {
            std::cerr << "Error at locking next image" << std::endl;
            err_loop = true;
            break;
        }

        mutex.lock();
        if(IMAGE_REQUEST)
        {
            IMAGE_REQUEST = false;
            if(!VRmUsbCamConvertImage(p_source_img,p_target_img))
            {
                std::cerr << "Error at converting image: " << VRmUsbCamGetLastError()  << std::endl;
                err_loop = true;
                break;
            }

            std::cout << "pitch of srcImg: " << p_source_img->m_pitch << std::endl;
            std::cout << "succesfully grabed and converted image" << std::endl;
            //-- work on image --


            //-- end work on image --
            //-- transmitt to ros --
//===================================================================================================================
            //copy data to _rosImgBuffer:
            for(unsigned int y = 0; y < _rosImage.height; y++)
            {
                for(unsigned int x = 0; x < _rosImage.width * 3; x++)
                {
                    _rosImgBuffer[y*_rosImage.width*3 + x] = p_target_img->mp_buffer[y*p_target_img->m_pitch + x];
                }
            }

            _rosImage.data = _rosImgBuffer;
            std::cout << "ImageSize: " << _rosImage.dataSize << std::endl;
            _rosBrige.writeImage(_rosImage);
//===================================================================================================================
            // -- end trasmitt to ros --
        }
        mutex.unlock();
        if(!VRmUsbCamUnlockNextImage(device,&p_source_img))
        {
            std::cerr << "Error at unlocking next image" << std::endl;
            err_loop = true;
            break;
        }

        //droped images:
        if(frames_dropped)
        {
            std::cout << frames_dropped <<" frame(s) dropped" << std::endl;
        }
    }

    if(err_loop)
    {
        std::cerr << "exit with error" << std::endl;
    }
    //free target image
    VRmUsbCamFreeImage(&p_target_img);
    //stop grab
    VRmUsbCamStop(device);
    //close device
    VRmUsbCamCloseDevice(device);
    _thread.join();
    return 0;
}
