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
#include "sensor_driver.h"
#include "hw_sensors.h"
#include "icm20602.h"
#include "spl06.h"
#include "ist8307a_imp.h"
#include "utils.h"

static const struct icm20602_platform_data icm20602_spdata = {
    .irq_pin = 75,
    .trigger_type = 1,
    .place = 4,
    .acc_range = 4,
    .acc_bw = 20,
    .gyro_range = 2000,
    .gyro_bw = 20,
};
static const struct sensor_platform_data icm20602_pdata = {
    .name = "invn,icm20602",
    .bus_info = {
        .bus_type = BUS_SPI,
        .u = {
            .spi_info = {
                .master_id = 0,
                .chip_select = 0,
                .mode = SPI_CPOL1_CPHA1,
                .max_frequency = 8000000,
            },
        },
    },
    .spdata = &icm20602_spdata,
};

static const struct spl0601_platform_data spl0601_spdata = {
    .press_smplrate = 32,
    .press_oversmpl = 16,
    .temp_smplrate = 32,
    .temp_oversmpl = 1,
};
static const struct sensor_platform_data spl0601_pdata = {
    .name = "goertek,spl0601",
    .bus_info = {
        .bus_type = BUS_I2C,
        .u = {
            .i2c_info = {
                .master_id = 0,
                .slave_addr = 0x77,
            },
        },
    },
    .spdata = &spl0601_spdata,
};

static const struct ist8307a_platform_data ist8307a_spdata = {
    .place = 7,
};
static const struct sensor_platform_data ist8307a_pdata = {
    .name = "isentek,ist8307a",
    .bus_info = {
        .bus_type = BUS_I2C,
        .u = {
            .i2c_info = {
                .master_id = 0,
                .slave_addr = 0x0C,
            },
        },
    },
    .spdata = &ist8307a_spdata,
};

static const struct hw_version bd_versions[] = {
    {
        .system = "0xfd7bbda9fd0300",
        .board = "0x00000006",
    },
    {
        .system = NULL,
    },
};

static const struct sensor_platform_data *sensor_pdata[] = {
    &icm20602_pdata,
    &spl0601_pdata,
    &ist8307a_pdata,
    NULL,
};

static struct board_info __define_brd__ binfo = {
    .version = bd_versions,
    .sensor_pdata = &sensor_pdata[0],
};
