/* Copyright Statement:
 *
 * This software/firmware and related documentation ("Fishsemi Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to Fishsemi Inc. and/or its licensors. Without
 * the prior written permission of Fishsemi inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of Fishsemi Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 *
 * Fishsemi Inc. (C) 2019. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("PINECONE SOFTWARE")
 * RECEIVED FROM PINECONE AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. PINECONE EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES PINECONE PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE PINECONE SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN PINECONE
 * SOFTWARE. PINECONE SHALL ALSO NOT BE RESPONSIBLE FOR ANY PINECONE SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND PINECONE'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE PINECONE SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT PINECONE'S OPTION, TO REVISE OR REPLACE THE
 * PINECONE SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO PINECONE FOR SUCH PINECONE SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("Fishsemi
 * Software") have been modified by Fishsemi Inc. All revisions are subject to
 * any receiver's applicable license agreements with Fishsemi Inc.
 */

#include "sensor_driver.h"
#include "hw_sensors.h"
#include "icm42605.h"
#include "utils.h"

extern struct sensor_driver icm42605_drv;

static const struct icm42605_platform_data icm42605_spdata = {
  .irq_pin = 10,
  .trigger_type = IOEXPANDER_VAL_RISING,
  .place = 1,
};
static const struct sensor_platform_data icm42605_pdata = {
  .name = "invn,icm42605",
  .bus_info = {
    .bus_type = BUS_I2C,
    .u = {
      .i2c_info = {
        .master_id = 0,
        .slave_addr = 0x68,
      },
    },
  },
  .spdata = &icm42605_spdata,
};

const struct sensor_platform_data *g_sensor_pdata[] = {
  &icm42605_pdata,
  NULL,
};

const struct sensor_driver *g_sensor_drv[] = {
  &icm42605_drv,
  NULL,
};
