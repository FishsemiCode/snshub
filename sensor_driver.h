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

#ifndef __SENSOR_DRIVER_H__
#define __SENSOR_DRIVER_H__

#include <stdbool.h>
#include <stdint.h>
#include "sensor_port.h"
#include "sensor.h"

#define SENSOR_MATCHING_NAME_SIZE 20

struct sensor_dev;

struct sensor_matching_data {
  char name[SENSOR_MATCHING_NAME_SIZE];
  FAR void *priv; /* for the predefined binded data for this specifical sensor device */
};

struct sensor_platform_data {
  char name[SENSOR_MATCHING_NAME_SIZE];
  const struct sensor_bus_info bus_info;
  FAR const void *spdata; /* for the sensor specifical platform data */
};

struct board_info {
  /* used to match with the board_version by the hw_sensors_init(...) to
   * be dynamically probed */
  FAR const struct hw_version *version;
  FAR const struct sensor_platform_data **sensor_pdata;
};

struct sensor_ops {
  int (*activate)(FAR const struct sensor_dev *dev, bool enable, snshub_data_mode mode);
  int (*set_delay)(FAR const struct sensor_dev *dev, uint32_t us);
  int (*selftest)(FAR const struct sensor_dev *dev, FAR void *data);

  /* 1. the offset will be passed from AP at startup;
   * 2. the snshub may update the offset in need;
   * 3. AP may need to get the latest the offset for a new
   *  calibration or storing the offset to file */
  int (*set_offset)(FAR const struct sensor_dev *dev, FAR void *offset);
  int (*get_offset)(FAR const struct sensor_dev *dev, FAR void *offset);

  /*XXX: is it better to add get_module/range/delay/resolution
   * to replace the members in the sensor_dev? then it will
   * be not duplicated with the ones in the 'struct snshub_sensor_t' */
  FAR char* (*get_vendor)(FAR const struct sensor_dev *dev);
  FAR char* (*get_module)(FAR const struct sensor_dev *dev);
  int (*get_type)(FAR const struct sensor_dev *dev);
  float (*get_max_range)(FAR const struct sensor_dev *dev);
  float (*get_power)(FAR const struct sensor_dev *dev);
  int (*get_min_delay)(FAR const struct sensor_dev *dev);
  int (*get_max_delay)(FAR const struct sensor_dev *dev);
  float (*get_resolution)(FAR const struct sensor_dev *dev);

  int (*read_data)(FAR const struct sensor_dev *dev, FAR struct sensor_event *event);
};


/* basic sensor device structure, the individual drivers would derive their
 * specifical ones based on this structure by the private data filed */
struct sensor_dev {
  FAR char *name;
  int type;
  FAR struct sensor_ops *ops;
  FAR const struct sensor_platform_data *pdata;
  FAR void *rtdata; /* for the runtime data(eg. offsets, status) of this sensor device */
  FAR const struct sensor_matching_data *mdata;
};

static inline void init_sensor_dev(FAR struct sensor_dev *sdev,
    FAR char *name,
    int type,
    FAR struct sensor_ops *ops,
    FAR const struct sensor_platform_data *pdata,
    FAR void *rtdata,
    FAR const struct sensor_matching_data *mdata)
{
  sdev->name = name;
  sdev->type = type;
  sdev->ops = ops;
  sdev->pdata = pdata;
  sdev->rtdata = rtdata;
  sdev->mdata = mdata;
}

struct sensor_driver {
  FAR const char *name;
  FAR const struct sensor_matching_data **mdata;
  int (*probe)(FAR const struct sensor_platform_data *pdata,
      FAR const struct sensor_matching_data *mdata,
      FAR struct sensor_dev **dev, FAR uint32_t *num_sensors);
};

#endif
