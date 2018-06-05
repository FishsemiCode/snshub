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
#include <Driver_GPIO.h>
#include "sensor_driver.h"
#include "hw_sensors.h"
#include "ltr579.h"
#include "yas537.h"
#include "icm20690.h"
#include "utils.h"

/* data for probe of ltr579 */
static const struct ltr579_platform_data ltr579_spdata = {
    .als_trans_ratio = 25,
    .als_hysteresis_ratio = 20,
    .als_auto_high_res = false,
    .prox_thres_near = 150,
    .prox_thres_far = 80,
    .prox_default_offset = 200,
};
static const struct sensor_platform_data ltr579_pdata = {
    .name = "liteon,ltr579",
    .bus_info = {
        .bus_type = BUS_I2C,
        .u = {
            .i2c_info = {
                .master_id = 0,
                .slave_addr = 0x53,
            },
        },
    },
    .spdata = &ltr579_spdata,
};

/* data for probe of yas537 */
static const struct yas537_platform_data yas537_spdata = {
    .position = 3,
    .delay = 50,
    .matrix = {1, 2, 3, 4, 5, 6, 7, 8, 9},
};
static const struct sensor_platform_data yas537_pdata = {
    .name = "yamaha,yas537",
    .bus_info = {
        .bus_type = BUS_I2C,
        .u = {
            .i2c_info = {
                .master_id = 0,
                .slave_addr = 0x2e,
            },
        },
    },
    .spdata = &yas537_spdata,
};

static const struct icm20690_platform_data icm20690_spdata = {
    .irq_pin = 13,
    .trigger_type = ARM_GPIO_TRIGGER_HIGH,
    .place = 1,
    .acc_range = 4,
    .acc_bw = 20,
    .gyro_range = 2000,
    .gyro_bw = 20,
};
static const struct sensor_platform_data icm20690_pdata = {
    .name = "invn,icm20690",
    .bus_info = {
        .bus_type = BUS_SPI,
        .u = {
            .spi_info = {
                .master_id = 1,
                .chip_select = 0,
                .mode = SPI_CPOL1_CPHA1,
                .max_frequency = 2000000,
            },
        },
    },
    .spdata = &icm20690_spdata,
};

static const struct hw_version bd_versions[] = {
    {
        .system = "any",
        .board = "banks-evb",
    },
    {
        .system = NULL,
    },
};

static const struct sensor_platform_data *sensor_pdata[] = {
    &ltr579_pdata,
    &yas537_pdata,
    &icm20690_pdata,
    NULL,
};

static struct board_info __define_brd__ binfo = {
    .version = bd_versions,
    .sensor_pdata = &sensor_pdata[0],
};
