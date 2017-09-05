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

#ifndef __SENSOR_DRIVER_H__
#define __SENSOR_DRIVER_H__

#include <stdbool.h>
#include <stdint.h>
#include "sensor_port.h"

#define SENSOR_MATCHING_NAME_SIZE 20

#if defined ARCH_CEVA
#define __define_brd__  __attribute__((__used__)) __attribute__((section(".DSECT sensor_board_section")))
#define __define_drv__  __attribute__((__used__)) __attribute__((section(".DSECT sensor_driver_section")))
#else
#define __define_brd__  __attribute__((__used__)) __attribute__((section(".sensor_board_section")))
#define __define_drv__  __attribute__((__used__)) __attribute__((section(".sensor_driver_section")))
#endif

struct sensor_dev;

struct sensor_matching_data {
    char name[SENSOR_MATCHING_NAME_SIZE];
    void *priv; /* for the predefined binded data for this specifical sensor device */
};

struct sensor_platform_data {
    char name[SENSOR_MATCHING_NAME_SIZE];
    const struct sensor_bus_info bus_info;
    const void *spdata; /* for the sensor specifical platform data */
};

struct hw_version {
    const char *system;
    const char *board;
};

struct board_info {
    /* used to match with the board_version by the hw_sensors_init(...) to
     * be dynamically probed */
    const struct hw_version *version;
    const struct sensor_platform_data **sensor_pdata;
};

struct sensor_ops {
    int (*activate)(const struct sensor_dev *dev, bool enable);
    int (*set_delay)(const struct sensor_dev *dev, uint32_t us);
    int (*selftest)(const struct sensor_dev *dev, void *data);

    /* 1. the offset will be passed from AP at startup;
     * 2. the snshub may update the offset in need;
     * 3. AP may need to get the latest the offset for a new
     *    calibration or storing the offset to file */
    int (*set_offset)(const struct sensor_dev *dev, void *offset);
    int (*get_offset)(const struct sensor_dev *dev, void *offset);

    /*XXX: is it better to add get_module/range/delay/resolution
     * to replace the members in the sensor_dev? then it will
     * be not duplicated with the ones in the 'struct sensor_t' */
    char* (*get_vendor)(const struct sensor_dev *dev);
    char* (*get_module)(const struct sensor_dev *dev);
    int (*get_type)(const struct sensor_dev *dev);
    float (*get_max_range)(const struct sensor_dev *dev);
    float (*get_power)(const struct sensor_dev *dev);
    int (*get_min_delay)(const struct sensor_dev *dev);
    int (*get_max_delay)(const struct sensor_dev *dev);
    float (*get_resolution)(const struct sensor_dev *dev);
};


/* basic sensor device structure, the individual drivers would derive their
 * specifical ones based on this structure by the private data filed */
struct sensor_dev {
    char *name;
    int type;
    struct sensor_ops *ops;
    const struct sensor_platform_data *pdata;
    void *rtdata; /* for the runtime data(eg. offsets, status) of this sensor device */
    const struct sensor_matching_data *mdata;
};

static inline void init_sensor_dev(struct sensor_dev *sdev,
        char *name,
        int type,
        struct sensor_ops *ops,
        const struct sensor_platform_data *pdata,
        void *rtdata,
        const struct sensor_matching_data *mdata)
{
    sdev->name = name;
    sdev->type = type;
    sdev->ops = ops;
    sdev->pdata = pdata;
    sdev->rtdata = rtdata;
    sdev->mdata = mdata;
}

struct sensor_driver {
    const char *name;
    const struct sensor_matching_data **mdata;
    int (*probe)(const struct sensor_platform_data *pdata,
            const struct sensor_matching_data *mdata,
            struct sensor_dev **dev, uint32_t *num_sensors);
};

#endif
