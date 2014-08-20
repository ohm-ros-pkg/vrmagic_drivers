/*
 * VrMagicHandlercamhost.h
 *
 *  Created on: 29.07.2014
 *      Author: m1ch1
 */

#ifndef VRMAGICHANDLERCAMHOST_H_
#define VRMAGICHANDLERCAMHOST_H_

#include "TCP/TCP.h"
#include "../../../include/VrMagicHandler_base.h"

namespace ohm
{

class VrMagicHandler_camhost
{
public: //functions
    /**
     * @brief Constructor
     *
     * @param[in]  port	->	port of tcp-server on the smartcam
     *
     */
    VrMagicHandler_camhost(unsigned int port_smartcam);

    /**
     * @brief Destructor
     *
     */
    virtual ~VrMagicHandler_camhost();

    /**
     * @brief connects to ros-server-node, this function is blocking
     *        untill a connection is established
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void connect();

    /**
     * @brief writes image to ros-server
     *
     * @param[in]  image  ->  contains information and a pointer to data from the image
     *
     * @return 		   0 on succes and -1 on error
     */
    int writeImage(ohm::ImageType& image);

    /**
     * @brief waits for an imagerequest from ros
     *
     * @param[in,out]  void
     *
     * @return 		   0 on succes -1 on connection error and -2 if trigger is not an image request
     */
    //int wait();

    unsigned int getPort() const;

private: //functions

private: //dataelements
    unsigned int _port;
    apps::TCP* _tcpServer;

    OHM_HEADER_TYPE* _imgHeader;

};

} /* namespace ohm */

#endif /* VRMAGICHANDLERCAMHOST_H_ */
