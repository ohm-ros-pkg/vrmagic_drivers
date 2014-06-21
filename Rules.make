# Global path definitions for all demos

#Specify target architecture here or give as command line option
# allowed values Linux_x86, Linux_x64, D2, D3

#TARGET_ARCH=Linux_x86
TARGET_ARCH=D2

# Path to your installation of the VRmagic SDK 
# for D2 Platform: /your/path/to/VRmUsbCamDistributionForDevelopers-$(SDK_VERSION)
# for Linux_x86/x64 and D3: set SDK_VERSION to version you want to build against 
SDK_VERSION = 4.1.0
VRM_DIR = /opt/vrmagic/sdk-4.1.0

#Only for D2 platform: Specify path to Code Sourcery Toolchain
D2_ARM_BUILD_SYSTEM = /opt/CodeSourcery/Sourcery_G++_Lite/bin

############### DO NOT EDIT BELOW THIS LINE ###########

# Check Target Architecture

ALLOWED_ARCHS = Linux_x86 Linux_x64 D2 D3

ifndef TARGET_ARCH
  $(error Variable TARGET_ARCH must be set in Rules.make of on the command line! Allowed target architectures are: $(ALLOWED_ARCHS))
endif

_TARGET_ARCH := $(filter $(TARGET_ARCH),$(ALLOWED_ARCHS))

$(info Building for $(_TARGET_ARCH))

ifndef _TARGET_ARCH
  $(error "Variable TARGET_ARCH not set correctly! Possible target architectures: Linux_x86, Linux_x64, D2, D3")
endif

## Linux_x86
VRMSDK_INSTALL_DIR_Linux_x86 = $(VRM_DIR)/x86/development_kit
CROSS_COMPILE_PREFIX_Linux_x86 =
VRM_INCPATH_Linux_x86 = -I$(VRMSDK_INSTALL_DIR_Linux_x86)/include
VRM_LIBPATH_Linux_x86 = -L$(VRMSDK_INSTALL_DIR_Linux_x86)/lib
VRM_LFLAGS_Linux_x86 =
VRM_CFLAGS_Linux_x86 =

## Linux_x64
VRMSDK_INSTALL_DIR_Linux_x64 = $(VRM_DIR)/x64/development_kit
CROSS_COMPILE_PREFIX_Linux_x64 =
VRM_INCPATH_Linux_x64 = -I$(VRMSDK_INSTALL_DIR_Linux_x64)/include
VRM_LIBPATH_Linux_x64 = -L$(VRMSDK_INSTALL_DIR_Linux_x64)/lib/
VRM_LFLAGS_Linux_x64 =
VRM_CFLAGS_Linux_x64 =

## D2
VRMSDK_INSTALL_DIR_D2 = $(VRM_DIR)/D2/development_kit
CROSS_COMPILE_PREFIX_D2 = $(D2_ARM_BUILD_SYSTEM)/arm-none-linux-gnueabi-
VRM_INCPATH_D2 = -I$(VRMSDK_INSTALL_DIR_D2)/include
VRM_LIBPATH_D2 = -L$(VRMSDK_INSTALL_DIR_D2)/lib 
VRM_LFLAGS_D2 =
VRM_CFLAGS_D2 = -DD2_PLATFORM

## D3
CROSS_COMPILE_PREFIX_D3 = arm-linux-gnueabihf-
VRMSDK_INSTALL_DIR_D3 = $(VRM_DIR)/D3/development_kit
VRM_INCPATH_D3 = -I$(VRMSDK_INSTALL_DIR_D3)/include \
-I/usr/arm-linux-gnueabihf/include \
-I/usr/arm-linux-gnueabihf/vrmagic/usr/include

VRM_LIBPATH_D3 = -L$(VRMSDK_INSTALL_DIR_D3)/lib \
-L/usr/arm-linux-gnueabihf/lib \
-L/usr/arm-linux-gnueabihf/vrmagic/lib \
-L/usr/arm-linux-gnueabihf/vrmagic/lib/arm-linux-gnueabihf \
-L/usr/arm-linux-gnueabihf/vrmagic/usr/lib \
-L/usr/arm-linux-gnueabihf/vrmagic/usr/lib/arm-linux-gnueabihf

VRM_LFLAGS_D3 = -Wl,--rpath-link=$(VRMSDK_INSTALL_DIR_D3)/lib:/usr/arm-linux-gnueabihf/lib:/usr/arm-linux-gnueabihf/vrmagic/lib/:/usr/arm-linux-gnueabihf/vrmagic/lib/arm-linux-gnueabihf:/usr/arm-linux-gnueabihf/vrmagic/usr/lib:/usr/arm-linux-gnueabihf/vrmagic/usr/lib/arm-linux-gnueabihf

VRM_CFLAGS_D3 = -DD3_PLATFORM

################ Put things together ####################

CROSS_COMPILE_PREFIX = ${CROSS_COMPILE_PREFIX_${_TARGET_ARCH}}
VRMSDK_INSTALL_DIR = ${VRMSDK_INSTALL_DIR_${_TARGET_ARCH}}
VRM_INCPATH          = ${VRM_INCPATH_${_TARGET_ARCH}}
VRM_LIBPATH          = ${VRM_LIBPATH_${_TARGET_ARCH}}
VRM_LFLAGS           = ${VRM_LFLAGS_${_TARGET_ARCH}}
VRM_CFLAGS           = ${VRM_CFLAGS_${_TARGET_ARCH}}
