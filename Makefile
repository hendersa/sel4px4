#
# Copyright 2014, NICTA
#
# This software may be distributed and modified according to the terms of
# the BSD 2-Clause license. Note that NO WARRANTY is provided.
# See "LICENSE_BSD2.txt" for details.
#
# @TAG(NICTA_BSD)
#

# Targets
TARGETS := $(notdir $(SOURCE_DIR)).bin

# Set custom entry point as we are a rootserver and will not be
# started in a standard way
ENTRY_POINT := _sel4_start

# Source files required to build the target
CFILES   := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/*.c))
CFILES   += $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/plat/${PLAT}/*.c))
CFILES   += $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/arch/${ARCH}/*.c))

# AWH - Platforms
PLAT_CFILES := src/platforms/common/px4_getopt.c

# AWH - uORB module
UORB_PATH := src/modules/uORB
UORB_CXXFILES := $(UORB_PATH)/uORBDevices_posix.cpp 
UORB_CXXFILES += $(UORB_PATH)/uORBManager_posix.cpp
UORB_CXXFILES += $(UORB_PATH)/uORBTest_UnitTest.cpp
UORB_CXXFILES += $(UORB_PATH)/objects_common.cpp
UORB_CXXFILES += $(UORB_PATH)/Publication.cpp
UORB_CXXFILES += $(UORB_PATH)/Subscription.cpp
UORB_CXXFILES += $(UORB_PATH)/uORBUtils.cpp
UORB_CXXFILES += $(UORB_PATH)/uORB.cpp
UORB_CXXFILES += $(UORB_PATH)/uORBMain.cpp

# AWH - Drivers module
DRIVER_PATH := src/drivers/device
DRIVER_FILES := $(DRIVER_PATH)/vdev.cpp
DRIVER_FILES += $(DRIVER_PATH)/vdev_posix.cpp
DRIVER_FILES += $(DRIVER_PATH)/vfile.cpp
DRIVER_FILES += $(DRIVER_PATH)/device_posix.cpp
# #DRIVER_FILES += $(DRIVER_PATH)/i2c_posix.cpp
# #DRIVER_FILES += $(DRIVER_PATH)/pio.cpp
DRIVER_FILES += $(DRIVER_PATH)/ringbuffer.cpp
DRIVER_FILES += $(DRIVER_PATH)/sim.cpp

# AWH - POSIX layer
POSIX_LAYER_CFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/platforms/posix/px4_layer/*.c))
POSIX_LAYER_CXXFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/platforms/posix/px4_layer/*.cpp))

# AWH - libuavcan - UAVCAN <--> uORB bridge
UAVCAN_PATH := src/modules/uavcan
UAVCAN_CFILES := $(UAVCAN_PATH)/uavcan_params.c
UAVCAN_CXXFILES := $(UAVCAN_PATH)/uavcan_main.cpp
UAVCAN_CXXFILES += $(UAVCAN_PATH)/uavcan_clock.cpp
UAVCAN_CXXFILES += $(UAVCAN_PATH)/actuators/esc.cpp
UAVCAN_CXXFILES += $(UAVCAN_PATH)/sensors/sensor_bridge.cpp
UAVCAN_CXXFILES += $(UAVCAN_PATH)/sensors/gnss.cpp
UAVCAN_CXXFILES += $(UAVCAN_PATH)/sensors/mag.cpp
UAVCAN_CXXFILES += $(UAVCAN_PATH)/sensors/baro.cpp

# AWH - The autopilot app
AP_PATH := src/examples/px4_simple_app
AP_FILES := $(AP_PATH)/px4_simple_app.c

# AWH - Sensor module
SENSOR_PATH := src/modules/sensors
SENSOR_CFILES := $(SENSOR_PATH)/sensor_params.c
SENSOR_CXXFILES := $(SENSOR_PATH)/sensors.cpp

# AWH - Navigation controller module
NAV_CFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/modules/navigator/*.c))
NAV_CXXFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/modules/navigator/*.cpp))

# AWH - Commander module
COMMAND_CFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/modules/commander/*.c))
COMMAND_TEMP_CXXFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/modules/commander/*.cpp))
COMMAND_FILTER := src/modules/commander/state_machine_helper_posix.cpp
COMMAND_CXXFILES := $(filter-out $(COMMAND_FILTER),$(COMMAND_TEMP_CXXFILES))

# AWH - Controllib module
CLIB_PATH := src/modules/controllib
CLIB_CFILES := $(CLIB_PATH)/test_params.c
CLIB_CXXFILES := $(CLIB_PATH)/block/Block.cpp $(CLIB_PATH)/blocks.cpp
CLIB_CXXFILES += $(CLIB_PATH)/block/BlockParam.cpp
CLIB_CXXFILES += $(CLIB_PATH)/uorb/blocks.cpp

# AWH - Systemlib
SYSLIB_PATH := src/modules/systemlib
SYSLIB_TEMP_CFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/modules/systemlib/*.c))
SYSLIB_FILTER := $(SYSLIB_PATH)/err.c $(SYSLIB_PATH)/up_cxxinitialize.c
SYSLIB_FILTER += $(SYSLIB_PATH)/hx_stream.c
SYSLIB_CFILES := $(filter-out $(SYSLIB_FILTER),$(SYSLIB_TEMP_CFILES))
SYSLIB_CFILES += $(SYSLIB_PATH)/param/param.c $(SYSLIB_PATH)/pid/pid.c
SYSLIB_CFILES += $(SYSLIB_PATH)/pwm_limit/pwm_limit.c
SYSLIB_CFILES += $(SYSLIB_PATH)/bson/tinybson.c
SYSLIB_CXXFILES := $(SYSLIB_PATH)/circuit_breaker.cpp

# AWH - MathLib
MATHLIB_PATH := src/lib/mathlib
MATHLIB_CXXFILES := $(MATHLIB_PATH)/math/test/test.cpp
MATHLIB_CXXFILES += $(MATHLIB_PATH)/math/Limits.cpp

# AWH - VTOL attitude controller module
VTOL_PATH := src/modules/vtol_att_control
VTOL_CXXFILES := $(VTOL_PATH)/vtol_att_control_main.cpp
VTOL_CFILES := $(VTOL_PATH)/vtol_att_control_params.c

# AWH - sdlog2 application
SDLOG2_CFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/modules/sdlog2/*.c))

# AWH - MAVLink protocol to uORB interface process
MAVLINK_PATH := src/modules/mavlink
MAVLINK_CFILES := $(MAVLINK_PATH)/mavlink.c
MAVLINK_CXXFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/modules/mavlink/*.cpp))

# AWH - Position estimator Inav module
POS_EST_CFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/modules/position_estimator_inav/*.c))

# AWH - DataMan module
DM_CFILES := src/modules/dataman/dataman.c

# AWH - Geo library
GEO_CFILES := src/lib/geo/geo.c

#AWH - Conversion library
CONV_CXXFILES := src/lib/conversion/rotation.cpp

# AWH - ARM-specific asm
ARM_ASMFILES := src/arm/clone.S src/arm/__set_thread_area.S 
ARM_ASMFILES += src/arm/syscall_cp.S src/arm/tls.S 
ARM_ASMFILES += src/arm/__unmapself.S

# AWH - All of the module files that currently build
CPPFILES += $(UORB_CXXFILES) $(DRIVER_FILES) $(POSIX_LAYER_CXXFILES)
CPPFILES += $(MAVLINK_CXXFILES) $(SYSLIB_CXXFILES) $(CONV_CXXFILES)
CPPFILES += $(SENSOR_CXXFILES) $(NAV_CXXFILES) $(COMMAND_CXXFILES)
CPPFILES += $(CLIB_CXXFILES) $(MATHLIB_CXXFILES)
CFILES += $(POSIX_LAYER_CFILES) $(AP_FILES) $(SENSOR_CFILES)
CFILES += $(MAVLINK_CFILES) $(SDLOG2_CFILES) $(SYSLIB_CFILES)
CFILES += $(POS_EST_CFILES) $(UAVCAN_CFILES) $(NAV_CFILES)
CFILES += $(COMMAND_CFILES) $(GEO_CFILES) $(DM_CFILES) $(PLAT_CFILES)
CFILES += $(CLIB_CFILES)

#AWH - Modules that shouldn't be built in (Pixhawk/NuttX-specific)
#CFILES += $(UAVCAN_CFILES) $(VTOL_CFILES)
#CPPFILES += $(UAVCAN_CXXFILES) $(VTOL_CXXFILES)

# CPIO archive
OFILES := archive.o

ASMFILES := $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/arch/${ARCH}/*.S))
ASMFILES += $(patsubst $(SOURCE_DIR)/%,%,$(wildcard $(SOURCE_DIR)/src/plat/${PLAT}/*.S))

# AWH - ASM from other sources for ARM
ASMFILES += $(ARM_ASMFILES)

# Libraries required to build the target
LIBS = c sel4 cpio elf sel4muslcsys sel4vka sel4allocman \
       platsupport sel4platsupport sel4test sel4vspace \
       sel4utils sel4simple utils uclibc++
ifdef CONFIG_KERNEL_STABLE
LIBS += sel4simple-stable
else
LIBS += sel4simple-default
endif

# extra cflag for sel4test
#CFLAGS += -Werror -g
ifdef CONFIG_X86_64
CFLAGS += -mno-sse
endif

include $(SEL4_COMMON)/common.mk
OUR_FLAGS += -D__PX4_POSIX -D__PX4_LINUX -Dnoreturn_function= -Dnullptr=0
OUR_FLAGS += -DMAVLINK_DIALECT=common
OUR_FLAGS += -fvisibility=hidden -include $(SOURCE_DIR)/src/include/visibility.h
OUR_FLAGS += -fno-builtin -Wno-sign-compare
OUR_FLAGS += -I$(SOURCE_DIR)/src -I$(SOURCE_DIR)/src/platforms
OUR_FLAGS += -I$(SOURCE_DIR)/src/modules -I$(SOURCE_DIR)/src/include
OUR_FLAGS += -I$(SOURCE_DIR)/../../../../libs/libuclibc++/include
OUR_FLAGS += -I$(SOURCE_DIR)/src/platforms/posix/include
OUR_FLAGS += -I$(SOURCE_DIR)/src/include/mavlink
#OUR_FLAGS += -DUAVCAN_CPP_VERSION=UAVCAN_CPP03 -DUAVCAN_NO_ASSERTIONS
#OUR_FLAGS += -I$(SOURCE_DIR)/src/lib/uavcan/libuavcan_drivers/posix/include
OUR_FLAGS += -I$(SOURCE_DIR)/src/lib -I$(SOURCE_DIR)/src/lib/eigen
CXXFLAGS += -std=c++11 -nostdinc++ -fno-exceptions -fno-rtti $(OUR_FLAGS)
CFLAGS += $(OUR_FLAGS)

# AWH - Add in linker script for syslib params
LDFLAGS += ${COMMON_PATH}/../../projects/px4/apps/px4/px4_makefiles/posix/ld.script

# targets to generate CPIO archive of elf files
${COMPONENTS}:
	false

archive.o: ${COMPONENTS}
	$(Q)mkdir -p $(dir $@)
	${COMMON_PATH}/files_to_obj.sh $@ _cpio_archive $^

# AWH - PX4 build includes
#include makefiles/posix/firmware_posix.mk

