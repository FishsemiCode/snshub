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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <nuttx/i2c/i2c_master.h>
#include "sensor_port_i2c.h"
#include "utils.h"

#define MAX_WRITE_LENGTH  16

static int get_max_master_id(void)
{
  int i;

  for (i = 0; g_i2c[i]; i++);

  return i - 1;
}

static int _i2c_read(FAR struct sns_port *port, uint8_t reg, FAR void *buff, size_t size)
{
  FAR const struct sensor_i2c_info *i2c_info = &port->binfo->u.i2c_info;
  FAR struct i2c_master_s *drv_i2c = port->master;

  return i2c_writeread(drv_i2c, (FAR const struct i2c_config_s *)&(i2c_info->frequency), &reg, 1, buff, size);
}

static int _i2c_write(FAR struct sns_port *port, uint8_t reg, FAR void *buff, size_t size)
{
  FAR const struct sensor_i2c_info *i2c_info = &port->binfo->u.i2c_info;
  FAR struct i2c_master_s *drv_i2c = port->master;
  uint8_t data[MAX_WRITE_LENGTH + 1];

  if (size > MAX_WRITE_LENGTH) {
    snshuberr("too much write data\n");
    return -EINVAL;
  }

  data[0] = reg;
  memcpy(&data[1], buff, size);

  return i2c_write(drv_i2c, (FAR struct i2c_config_s *)&(i2c_info->frequency), data, size + 1);
}

static struct port_ops i2c_ops = {
  .read = _i2c_read,
  .write = _i2c_write,
};

int port_init_i2c(FAR const struct sensor_bus_info *info, FAR struct sns_port *port)
{
  FAR const struct sensor_i2c_info *i2c_info = &info->u.i2c_info;
  int max_id;

  max_id = get_max_master_id();
  if (max_id < 0 || max_id < i2c_info->master_id) {
    snshuberr("no i2c master found\n");
    return -ENODEV;
  }

  port->master = g_i2c[i2c_info->master_id];
  port->ops = &i2c_ops;
  port->binfo = info;

  return 0;
}
