// ==============================================================================================
// This file is part of the VRmagic USB2.0 Camera Demo Application
// ==============================================================================================
#ifndef VRMUSBCAMDEMO_H
#define VRMUSBCAMDEMO_H

// include the api of the library
#include  <vrmusbcam2.h>
#ifdef WIN32
#include <vrmusbcam2win32.h>
#endif
#include <vm_lib/vm_lib.h>
#include <iostream>
#include <sstream>
#include <iomanip>

// function prototypes "main.cpp"
void LogExit();

// small helper macro to check function for success and call LogExit when function fails
#define VRMEXECANDCHECK(function)\
{\
	if (VRM_SUCCESS!=function)\
	LogExit();\
}

// function prototypes "readcamera.cpp"
void adaptTemplateROI(char ch, VRmRectI* rect, int width, int height);
void teach_template(VD_IMAGE* in_image, VD_IMAGE* out_image, VD_IMAGE* template_image, OR_HND* or_hnd, char ch, VRmRectI* p_template_roi);
void show_template(VD_IMAGE* out_image, VD_IMAGE* template_image, VRmRectI template_roi, char ch);
void search_template(VD_IMAGE* in_image, VD_IMAGE* out_image, OR_HND* or_hnd, char ch);
void readCamera(VRmUsbCamDevice device, VRmDWORD port, VRmImageFormat target_format, VRmRectI src_cropping_region);

// function prototypes "initcamera.cpp"
void initCamera(VRmUsbCamDevice device, VRmDWORD& port, VRmImageFormat& target_format, VRmSizeI screen_size, VRmColorFormat screen_colorformat, VRmRectI& src_cropping_region );

// function prototypes "sdlwindow.cpp"
bool SDLQueryDisplayCaps(  VRmSizeI& screen_size, VRmColorFormat& screen_colorformat);
bool SDLWindowInit( const char* fp_caption, VRmImageFormat f_format);
void SDLWindowClose();
unsigned char* SDLLockBuffer( unsigned int& f_pitch );
void SDLUnlockBuffer();
void SDLUpdate();
char GetKey();

#endif //VRMUSBCAMDEMO_H
