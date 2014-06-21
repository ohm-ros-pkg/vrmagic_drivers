# include Rules file with architecture dependent definitions of 
# VRMSDK_INSTALL_DIR, CROSS_COMPILE_PREFIX, VRM_INCPATH, VRM_LIBPATH, VRM_LFLAGS, VRM_CFLAGS
include ../Rules.make

CXX      = $(CROSS_COMPILE_PREFIX)g++
CXXFLAGS = -pipe -O2 -fmessage-length=0 -fPIC -Wno-deprecated -Wall -W -Wno-unused $(VRM_CFLAGS)

# append your additional includes with -I<path_to_new_dir>
INCPATH  = $(VRM_INCPATH) -I$(VRMSDK_INSTALL_DIR)/vm_lib/include -I.
# append your additional library search dirs with -L<path_to_new_dir>
LIBPATH  = $(VRM_LIBPATH) -L$(VRMSDK_INSTALL_DIR)/vm_lib/lib
# add additional linker flags as needed 
LFLAGS= $(VRM_LFLAGS)

LIBS = -lvrmusbcam2 -lpthread -lSDL -lrt

ifeq ($(TARGET_ARCH),Linux_x86)
LIBS := $(LIBS) -lvm_lib_x86
CXXFLAGS := $(CXXFLAGS) -D_VR_LINUX_X86
endif

ifeq ($(TARGET_ARCH),D2)
LIBS := $(LIBS) -lvm_lib_arm
CXXFLAGS := $(CXXFLAGS) -D_VR_ARM
endif

ifeq ($(TARGET_ARCH),D3)
LIBS := $(LIBS) -lvm_lib_armhf
CXXFLAGS := $(CXXFLAGS) -D_VR_ARM
endif

DEL_FILE = rm -f

####### Files

OBJECTS = initcamera.o \
		readcamera.o \
		main.o \
                sdlwindow.o
TARGET   = d2_node

first: all
####### Implicit rules

.SUFFIXES: .o .cpp

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)
	$(CXX) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBPATH) $(LIBS)

clean:
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) *~ core *.core
	-$(DEL_FILE) $(TARGET)
	
