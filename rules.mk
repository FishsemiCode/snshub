#
# Copyright (C) 2016 Pinecone
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

LOCAL_DIR := $(GET_LOCAL_DIR)

MODULE := $(LOCAL_DIR)

MODULE_INCLUDES += \
	$(LOCAL_DIR) \
	$(LOCAL_DIR)/sensor_driver \

MODULE_SRCS += \
	$(LOCAL_DIR)/algo_manager.c \
	$(LOCAL_DIR)/circ_buffer.c \
	$(LOCAL_DIR)/client_manager.c \
	$(LOCAL_DIR)/host_interface.c \
	$(LOCAL_DIR)/host_interface_rpmsg.c \
	$(LOCAL_DIR)/hw_sensors.c \
	$(LOCAL_DIR)/sensor_board.c \
	$(LOCAL_DIR)/sensor_driver/icm20602.c \
	$(LOCAL_DIR)/sensor_driver/icm20690.c \
	$(LOCAL_DIR)/sensor_driver/ltr579.c \
	$(LOCAL_DIR)/sensor_driver/spl06.c \
	$(LOCAL_DIR)/sensor_driver/ist8307a_imp.c \
	$(LOCAL_DIR)/sensor_manager.c \
	$(LOCAL_DIR)/sensor_port.c \
	$(LOCAL_DIR)/sensor_port_i2c.c \
	$(LOCAL_DIR)/sensor_port_spi.c \
	$(LOCAL_DIR)/utils.c \

ifeq ($(ARCH),ceva)
EXTRA_OBJS += $(call TOBUILDDIR, $(LOCAL_DIR))/sensor_board_bumblebee.o
EXTRA_OBJS += $(call TOBUILDDIR, $(LOCAL_DIR))/client_manager.o
EXTRA_OBJS += $(call TOBUILDDIR, $(LOCAL_DIR))/sensor_driver/icm20602.o
EXTRA_OBJS += $(call TOBUILDDIR, $(LOCAL_DIR))/sensor_driver/spl06.o
EXTRA_OBJS += $(call TOBUILDDIR, $(LOCAL_DIR))/sensor_driver/ist8307a_imp.o
endif

include make/module.mk
