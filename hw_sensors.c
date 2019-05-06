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

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sensor_driver.h"
#include "hw_sensors.h"
#include "utils.h"

#if 0
static struct list_node hw_sensors = LIST_INITIAL_VALUE(hw_sensors);
#else
/* here we just store the pointers of the sensor_dev, so a static array will be more efficent */
#define MAX_HW_SENSORS    10
static FAR struct sensor_dev *hw_sensors[MAX_HW_SENSORS];
static int hw_sensor_num = 0;
#endif

extern FAR const struct sensor_driver *g_sensor_drv[];
extern FAR const struct sensor_platform_data *g_sensor_pdata[];

static FAR const struct sensor_driver *hw_sensors_get_match_driver(FAR const struct sensor_platform_data *pdata,
      FAR const struct sensor_matching_data **matching_data)
{
  FAR const struct sensor_matching_data **mdata;
  int i, j;

  for (i = 0; g_sensor_drv[i] != NULL; i++) {
    mdata = g_sensor_drv[i]->mdata;
    for (j = 0; mdata[j] != NULL; j++)
      if (strcmp(mdata[j]->name, pdata->name) == 0) {
        *matching_data = mdata[j];
        return g_sensor_drv[i];
      }
  }

  return NULL;
}

int hw_sensors_init(void)
{
  /* 1. get the version of the hardware, e.g. board_id
   * 2. iterate all the sensor boards to get the one matches the board_id
   * 3. iterate all the sensor drivers in the section ".sensor_driver_section",
   *  and get the matched driver
   * 4. call the probe of the driver with platform data
   * 5. if the reture value is 0, then put the dev into the hw_sensors[]
   * */
  FAR const struct sensor_driver *drv;
  FAR const struct sensor_matching_data *mdata = NULL;
  FAR struct sensor_dev *sdev;
  int i, j;
  uint32_t num;
  int ret = -ENODEV;

  /*XXX: maybe we can remove this limitation, a succeeded init call needed? */
  if (hw_sensor_num) {
    snshuberr("hw_sensors_init is a oneshot routine!!!\n");
    return -EINVAL;
  }

  if (!g_sensor_pdata[0] || !g_sensor_drv[0]) {
    snshuberr("hw_sensors_init failed to get right platform or driver data\n");
    return -ENOENT;
  }

  for (i = 0; g_sensor_pdata[i] != NULL; i++) {
    snshubinfo("start to init for %s\n", g_sensor_pdata[i]->name);
    drv = hw_sensors_get_match_driver(g_sensor_pdata[i], &mdata);
    if (!drv) {
      snshuberr("no drivers for %s\n", g_sensor_pdata[i]->name);
      continue;
    }

    ret = drv->probe(g_sensor_pdata[i], mdata, &sdev, &num);
    if (ret) {
      snshuberr("probe for %s failed:%d\n", g_sensor_pdata[i]->name, ret);
      continue;
    }

    for (j = 0; j < num; j++) {
      if (j + hw_sensor_num >= MAX_HW_SENSORS) {
        /*XXX*/
        snshuberr("too many sensor devices\n");
        break;
      }

      hw_sensors[j + hw_sensor_num] = sdev + j;
    }

    hw_sensor_num += j;
  }

  snshubinfo("hw_sensors: %d hw sensors probed:\n", hw_sensor_num);
  for (i = 0; i < hw_sensor_num; i++) {
    snshubinfo("\tname=%s, type=%d\n", hw_sensors[i]->name, hw_sensors[i]->type);
  }

  return 0;
}

int hw_sensors_deinit(void)
{
  /*TODO:
   * call the remove functions of all the sensor devices, in which the
   * sensor_dev structure will be freed and the devices will be shutdown
   * */
  memset(hw_sensors, 0, sizeof(struct sensor_dev *) * MAX_HW_SENSORS);
  hw_sensor_num = 0;
  return 0;
}

/* get all of the hardware sensors */
int hw_sensors_get_list(FAR const struct sensor_dev ***sensor_list)
{
  *sensor_list = (const struct sensor_dev **)&hw_sensors[0];
  return hw_sensor_num;
}
