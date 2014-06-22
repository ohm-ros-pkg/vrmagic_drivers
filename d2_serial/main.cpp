// ==============================================================================================
// This file is part of the VRmagic VRmUsbCam2 C API Demo Application
// ==============================================================================================
// Main Function
// ----------------------------------------------------------------------------------------------

#include "demo.h"


#include <iostream>
#include <cstdlib>
#include <cstring>

void LogExit()
{
	std::cerr << "VRmUsbCam Error: " << VRmUsbCamGetLastError() << "\nApplication exit" << std::endl;
	exit(-1);
}

int main(int argc, char** argv)
{
	// at first, be sure to call VRmUsbCamCleanup() at exit, even in case
	// of an error
	atexit(VRmUsbCamCleanup);

	// read libversion (for informational purposes only)
	VRmDWORD libversion;
	VRMEXECANDCHECK(VRmUsbCamGetVersion(&libversion));

	std::cout << "========================================================" << std::endl
		<< "===        VRmagic VRmUsbCam C API v2 Demo           ===" << std::endl
		<< "========================================================" << std::endl
		<< "(v." << libversion << ")" << std::endl << std::endl;

	// uncomment one of the following lines to enable logging features of VRmUsbCam (for customer support)
	//VRmUsbCamEnableLogging(); // save logfile to default location
	//VRmUsbCamEnableLoggingEx("mylogfile.log"); //save logfile to user defined location

	// check for connected devices
	VRmDWORD size=0;
	VRMEXECANDCHECK(VRmUsbCamGetDeviceKeyListSize(&size));

	// open first usable device
	VRmUsbCamDevice device=0;
	VRmDeviceKey* p_device_key=0;
	for(VRmDWORD i=0; i<size && !device; ++i)
	{
		VRMEXECANDCHECK(VRmUsbCamGetDeviceKeyListEntry(i, &p_device_key));
		if(!p_device_key->m_busy) {
			VRMEXECANDCHECK(VRmUsbCamOpenDevice(p_device_key, &device));
		}
		VRMEXECANDCHECK(VRmUsbCamFreeDeviceKey(&p_device_key));
	}

	// display error when no camera has been found
	if(!device)
	{
		std::cerr << "No suitable VRmagic device found!" << std::endl;
		exit(-1);
	}

	// NOTE:
	// from now on, the "device" handle can be used to access the camera board.
	// use VRmUsbCamCloseDevice to end the usage

	VRmSizeI screen_size = {0, 0};
	VRmColorFormat screen_colorformat;
	if(!SDLQueryDisplayCaps(screen_size, screen_colorformat))
	{
		std::cerr << "Could not obtain screen info from SDL." << std::endl;
		VRmUsbCamCloseDevice(device);
		exit(-1);
	}

	// init camera, change some settings...
	// we get a target_format in return, which is necessary to initialize our
	// viewer window
	VRmImageFormat target_format;
	VRmRectI src_cropping_region;
	VRmDWORD port=0;
	initCamera(device, port, target_format, screen_size, screen_colorformat, src_cropping_region);

	// initialize viewer
	std::stringstream buf;
	VRmDeviceKey* p_key;
	VRMEXECANDCHECK(VRmUsbCamGetDeviceKey(device, &p_key));
	VRmSTRING l_serial;
	VRMEXECANDCHECK(VRmUsbCamGetSerialString(p_key, &l_serial));
	buf << "VRmagic VRmUsbCam C API v2 Demo - Device: "
		<< p_key->mp_product_str
		<< " #" << l_serial;
	if (!SDLWindowInit(buf.str().c_str(), target_format))
	{
		std::cerr << "Could not initialize SDL output window." << std::endl;
		VRmUsbCamCloseDevice(device);
		exit(-1);
	}
	VRMEXECANDCHECK(VRmUsbCamFreeDeviceKey(&p_key));

	// to be able to use the image processing library VM_LIB, we have to create a key
	// a valid key is returned if a VRmagic device is attached
	VRmDWORD key;
	VRmCreateVMLIBKey(&key);
	//use the key to open VM_LIB
	if(vm_lib_open(key))
	{
		std::cout << "VM_LIB key not valid!" << std::endl;
		exit(-1);
	}

	std::cout << "VM_LIB (v" << vm_lib_vers_str() << ") initialized" << std::endl;

	// and read pictures...
	readCamera(device, port, target_format, src_cropping_region);

	// close VM_LIB
	vm_lib_close();

	// ...and the device
	VRMEXECANDCHECK(VRmUsbCamCloseDevice(device));

	std::cout << "exit." << std::endl;

	return 0;
}
