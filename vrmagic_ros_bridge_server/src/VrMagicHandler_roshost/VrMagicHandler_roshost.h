/**
 *  @file   VrMagicHandler_roshost.h
 *  @brief  This class handles the vrmagic smartcam, the memory
 *          managment is included in the class.
 *
 *
 *
 *  @author	  m1ch1
 *
 *  @version  1.00
 *
 *  @date     29.07.2014
 *
 *
 * History: \n
 * -----------------
 *  - Version 1.00:    m1ch1          29.07.2014\n
 *			- initial Filecreation
 *
 */


#ifndef VRMAGICHANDLERROSHOST_H_
#define VRMAGICHANDLERROSHOST_H_

#include "../../include/VrMagicHandler_base.h"
#include "../TCP/TCP.h"

namespace ohm
{

class VrMagicHandler_roshost
{
public: //functions

    /**
     * @brief Constructor
     *
     * @param[in]  ip_smartcam	 ->	ip from vrmagic smartcam as std::string
     * @param[in]  port_smartcam -> port from vrmagic smartcam as unsigned int
     */
    VrMagicHandler_roshost(std::string ip_smartcam, unsigned int port_smartcam);

    /**
     * @brief Destructor
     *
     */
    virtual ~VrMagicHandler_roshost();

    /**
     * @brief connects to vrmagic smartcam blocks until a connection is established
     *
     * @note if wrong ip or port is given the function will block endless
     *       the andvantage is, that the roshost must not restart if connection to
     *       the camera borkes(because of a cam restart e.g)
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void connect();

    /**
     * @brief reads an image from the vrmagic smartcam, this function blocks untill an image
     *        is received
     *
     * @param[out]  image	->	reference of an ImageType object
     *
     * @return 		   todo
     */
    int readImage(ImageType& image);

    /**
     * @brief triggers vrmagic scmartcam for next image
     *
     * @param[in,out]  void
     *
     * @return 		   0 on succes -1 on connection error
     */
    int triggerImage();

    const std::string& getIp() const;
    unsigned int getPort() const;

private: //functions

private: //dataelements
    std::string _ip;
    unsigned int _port;
    apps::TCP* _tcpClient;

    OHM_HEADER_TYPE* _imgHeader;
    OHM_DATA_TYPE* _imgData;
    unsigned int _currentDataSize;
};

} /* namespace ohm */

#endif /* VRMAGICHANDLERROSHOST_H_ */
