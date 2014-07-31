/**
 *  @file   ImageBase.h
 *  @brief  Contains a Baseclass for ROS-Brige (for server and client) and defines for Networkmanagment
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

#ifndef IMAGEBASE_H_
#define IMAGEBASE_H_

#include <stdint.h>


#define OHM_DATA_TYPE       uint8_t
#define OHM_HEADER_TYPE     uint32_t
#define OHM_TRIGGER_TYPE    uint32_t
#define OHM_CONFIG_TYPE     uint32_t
//todo more stuff for header config if needed


namespace ohm
{

enum HeaderIndex{ ID = 0,
                  DATA_SIZE,        ///< for network managment, size of transmitted data in [bytes]
                  DATA_TYPE,
                  COMPRESSION_TYPE,
                  WIDTH,
                  HEIGHT,
                  CHANNELS,         //e.g. rgb(3) rgba(4)
                  BYTE_PER_PIXEL,
                  HEADER_SIZE
                   };

enum Data{ IMAGE = 0,
           STRING       ///< not implemented yet
            };

enum Compression{ NONE = 0,
                  TODO
                   };

enum Trigger{ IMAGE_REQUEST = 0
               };



typedef struct{
    unsigned int id;                        ///< id of an data packet
    unsigned int dataSize;                  ///< size of data in [byte]
    ohm::Data dataType;                     ///< type of data e.g IMAGE
    ohm::Compression compressionType;       ///< type of compression
    unsigned int width;                     ///< width of data (2dim)
    unsigned int height;                    ///< heigt of data (2dim)
    unsigned int channels;                  ///< channeld of data (for imges.. rgb(3))
    unsigned int bytePerPixel;              ///< pate per pixel (channels * sizeof(imagetype))
    OHM_DATA_TYPE* data;
}ImageType;


class VrMagicHandler_base
{
public:
    VrMagicHandler_base();
    virtual ~VrMagicHandler_base();
};

} /* namespace ohm */

#endif /* IMAGEBASE_H_ */
