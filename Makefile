#################################################################################
# Copyright Statement:
#
# This software/firmware and related documentation ("Fishsemi Software") are
# protected under relevant copyright laws. The information contained herein is
# confidential and proprietary to Fishsemi Inc. and/or its licensors. Without
# the prior written permission of Fishsemi inc. and/or its licensors, any
# reproduction, modification, use or disclosure of Fishsemi Software, and
# information contained herein, in whole or in part, shall be strictly
# prohibited.
#
# Fishsemi Inc. (C) 2019. All rights reserved.
#
# BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
# THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("PINECONE SOFTWARE")
# RECEIVED FROM PINECONE AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
# ON AN "AS-IS" BASIS ONLY. PINECONE EXPRESSLY DISCLAIMS ANY AND ALL
# WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
# WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
# NONINFRINGEMENT. NEITHER DOES PINECONE PROVIDE ANY WARRANTY WHATSOEVER WITH
# RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
# INCORPORATED IN, OR SUPPLIED WITH THE PINECONE SOFTWARE, AND RECEIVER AGREES
# TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
# RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
# OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN PINECONE
# SOFTWARE. PINECONE SHALL ALSO NOT BE RESPONSIBLE FOR ANY PINECONE SOFTWARE
# RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
# STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND PINECONE'S
# ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE PINECONE SOFTWARE
# RELEASED HEREUNDER WILL BE, AT PINECONE'S OPTION, TO REVISE OR REPLACE THE
# PINECONE SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
# CHARGE PAID BY RECEIVER TO PINECONE FOR SUCH PINECONE SOFTWARE AT ISSUE.
#
# The following software/firmware and/or related documentation ("Fishsemi
# Software") have been modified by Fishsemi Inc. All revisions are subject to
# any receiver's applicable license agreements with Fishsemi Inc.
#
#################################################################################

include $(TOPDIR)/Make.defs

APPNAME = snshub
PRIORITY = $(CONFIG_SNSHUB_PRIORITY)
STACKSIZE = $(CONFIG_SNSHUB_STACKSIZE)

ASRCS =
CSRCS += \
	algo_manager.c \
	circ_buffer.c \
	client_manager.c \
	hw_sensors.c \
	sensor_manager.c \
	sensor_port.c \
	sensor_port_i2c.c \
	sensor_port_spi.c \
	utils.c

ifeq ($(CONFIG_SNSHUB_BOARD_DUMMY),y)
CSRCS += sensor_board_dummy.c
endif

ifeq ($(CONFIG_SNSHUB_BOARD_U1_BOX),y)
CSRCS += sensor_board_u1_box.c
endif

ifeq ($(CONFIG_SNSHUB_DRIVER_ICM42605),y)
CSRCS += icm42605.c
endif

CFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" $(SRCDIR)/}

MAINSRC = client_manager.c

VPATH =
DEPPATH = --dep-path .

PROGNAME = snshub

include $(APPDIR)/Application.mk
