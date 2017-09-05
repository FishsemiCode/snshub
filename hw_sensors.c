/* Copyright Statement:
 *
 * This software/firmware and related documentation ("Pinecone Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to Pinecone Inc. and/or its licensors. Without
 * the prior written permission of Pinecone inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of Pinecone Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 *
 * Pinecone Inc. (C) 2017. All rights reserved.
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
 * The following software/firmware and/or related documentation ("Pinecone
 * Software") have been modified by Pinecone Inc. All revisions are subject to
 * any receiver's applicable license agreements with Pinecone Inc.
 */

#include <cmsis_os2.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sensor_driver.h"
#include "hw_sensors.h"

#if 0
static struct list_node hw_sensors = LIST_INITIAL_VALUE(hw_sensors);
#else
/* here we just store the pointers of the sensor_dev, so a static array will be more efficent */
#define MAX_HW_SENSORS      10
static struct sensor_dev *hw_sensors[MAX_HW_SENSORS];
static int hw_sensor_num = 0;
#endif

extern const struct sensor_driver __sensor_driver_start;
extern const struct sensor_driver __sensor_driver_end;
extern const struct board_info __sensor_board_start;
extern const struct board_info __sensor_board_end;

static bool hw_sensors_is_match_hw(const struct hw_version *v0, const struct hw_version *v1)
{
    if (strcmp(v0->system, v1->system))
        return false;

    if (strcmp(v0->board, v1->board))
        return false;

    return true;
}

static const struct sensor_platform_data **hw_sensors_get_pdata()
{
    struct hw_version hv;
    const struct hw_version *phv;
    const struct board_info *pbrd;

#if 0
    hv.system = getenv("/chosen/device");
    hv.board = getenv("/chosen/board-id");
#else
    hv.system = "0xfd7bbda9fd0300";
    hv.board = "0x00000006";
#endif
    printf("env: system=%s, board=%s\n", hv.system, hv.board);
    if (!hv.system || !hv.board)
        return NULL;

    for (pbrd = &__sensor_board_start; pbrd < &__sensor_board_end; pbrd++) {
        phv = pbrd->version;
        while (phv && phv->system) {
            if (hw_sensors_is_match_hw(&hv, phv))
                return pbrd->sensor_pdata;

            phv++;
        }
    }

    return NULL;
}

static const struct sensor_driver *hw_sensors_get_match_driver(const struct sensor_platform_data *pdata,
            const struct sensor_matching_data **matching_data)
{
    const struct sensor_driver *drv;
    const struct sensor_matching_data **mdata;
    int i;

    for (drv = &__sensor_driver_start; drv < &__sensor_driver_end; drv++) {
        mdata = drv->mdata;
        for (i = 0; mdata[i]->name[0]; i++)
            if (strcmp(mdata[i]->name, pdata->name) == 0) {
                *matching_data = mdata[i];
                return drv;
            }
    }

    return NULL;
}

int hw_sensors_init()
{
    /* 1. get the version of the hardware, e.g. board_id
     * 2. iterate all the sensor boards to get the one matches the board_id
     * 3. iterate all the sensor drivers in the section ".sensor_driver_section",
     *    and get the matched driver
     * 4. call the probe of the driver with platform data
     * 5. if the reture value is 0, then put the dev into the hw_sensors[]
     * */
    const struct sensor_driver *drv;
    const struct sensor_platform_data **pdata = NULL;
    const struct sensor_matching_data *mdata = NULL;
    struct sensor_dev *sdev;
    int i, j;
    uint32_t num;
    int ret = -ENODEV;

    /*XXX: maybe we can remove this limitation, a succeeded init call needed? */
    if (hw_sensor_num) {
        printf("hw_sensors_init is a oneshot routine!!!\n");
        return -EINVAL;
    }

    pdata = hw_sensors_get_pdata();
    if (!pdata) {
        printf("hw_sensors_init failed to get right platform data\n");
        return -ENOENT;
    }

    for (i = 0; pdata[i]; i++) {
        printf("start to init for %s\n", pdata[i]->name);
        drv = hw_sensors_get_match_driver(pdata[i], &mdata);
        if (!drv) {
            printf("no drivers for %s\n", pdata[i]->name);
            continue;
        }

        ret = drv->probe(pdata[i], mdata, &sdev, &num);
        if (ret) {
            printf("probe for %s failed:%d\n", pdata[i]->name, ret);
            continue;
        }

        for (j = 0; j < num; j++) {
            if (j + hw_sensor_num >= MAX_HW_SENSORS) {
                /*XXX*/
                printf("too many sensor devices\n");
                break;
            }

            hw_sensors[j + hw_sensor_num] = sdev + j;
        }

        hw_sensor_num += j;
    }

    printf("hw_sensors: %d hw sensors probed:\n", hw_sensor_num);
    for (i = 0; i < hw_sensor_num; i++) {
        printf("\tname=%s, type=%d\n", hw_sensors[i]->name, hw_sensors[i]->type);
    }

    return 0;
}

int hw_sensors_deinit()
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
int hw_sensors_get_list(const struct sensor_dev ***sensor_list)
{
    *sensor_list = (const struct sensor_dev **)&hw_sensors[0];
    return hw_sensor_num;
}
