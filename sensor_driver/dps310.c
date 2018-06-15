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
#include "sensor.h"
#include "sensor_manager.h"
#include "sensor_driver.h"
#include "sensor_port.h"
#include "utils.h"
#include "dps310.h"

#define PRESSURE_SENSOR    "pressure"

#define PRESS_DELAY_DEFAULT              (1000000/128)
#define ODR_IDEX_DEFAULT                 4
#define DPS310_TEMP_STD_MR               2
#define DPS310_TEMP_STD_OSR              3
#define DPS310_PRS_STD_MR                2
#define DPS310_PRS_STD_OSR               3
#define DPS310_OSR_SE                    3

#define DPS310_LSB                       0x01

#define DPS310_PRODUCT_ID                0x00
#define DPS310_REVISION_ID               0x01

#define DPS310_REG_INFO_PROD_ID          DPS310_REG_ADR_PROD_ID, \
                                         DPS310_REG_MASK_PROD_ID, \
                                         DPS310_REG_SHIFT_PROD_ID
#define DPS310_REG_ADR_PROD_ID           0x0D
#define DPS310_REG_MASK_PROD_ID          0x0F
#define DPS310_REG_SHIFT_PROD_ID         0U

#define DPS310_REG_INFO_REV_ID           DPS310_REG_ADR_REV_ID, \
                                         DPS310_REG_MASK_REV_ID, \
                                         DPS310_REG_SHIFT_REV_ID
#define DPS310_REG_ADR_REV_ID            0x0D
#define DPS310_REG_MASK_REV_ID           0xF0
#define DPS310_REG_SHIFT_REV_ID          4

#define DPS310_REG_INFO_OPMODE           DPS310_REG_ADR_OPMODE, \
                                         DPS310_REG_MASK_OPMODE, \
                                         DPS310_REG_SHIFT_OPMODE
#define DPS310_REG_ADR_OPMODE            0x08
#define DPS310_REG_MASK_OPMODE           0x07
#define DPS310_REG_SHIFT_OPMODE          0

#define DPS310_REG_INFO_TEMP_MR          DPS310_REG_ADR_TEMP_MR, \
                                         DPS310_REG_MASK_TEMP_MR, \
                                         DPS310_REG_SHIFT_TEMP_MR
#define DPS310_REG_ADR_TEMP_MR           0x07
#define DPS310_REG_MASK_TEMP_MR          0x70
#define DPS310_REG_SHIFT_TEMP_MR         4

#define DPS310_REG_INFO_TEMP_OSR         DPS310_REG_ADR_TEMP_OSR, \
                                         DPS310_REG_MASK_TEMP_OSR, \
                                         DPS310_REG_SHIFT_TEMP_OSR
#define DPS310_REG_ADR_TEMP_OSR          0x07
#define DPS310_REG_MASK_TEMP_OSR         0x07
#define DPS310_REG_SHIFT_TEMP_OSR        0

#define DPS310_REG_INFO_TEMP_SENSOR      DPS310_REG_ADR_TEMP_SENSOR, \
                                         DPS310_REG_MASK_TEMP_SENSOR, \
                                         DPS310_REG_SHIFT_TEMP_SENSOR
#define DPS310_REG_ADR_TEMP_SENSOR       0x07
#define DPS310_REG_MASK_TEMP_SENSOR      0x80
#define DPS310_REG_SHIFT_TEMP_SENSOR     7

#define DPS310_REG_INFO_TEMP_SENSORREC   DPS310_REG_ADR_TEMP_SENSORREC, \
                                         DPS310_REG_MASK_TEMP_SENSORREC, \
                                         DPS310_REG_SHIFT_TEMP_SENSORREC
#define DPS310_REG_ADR_TEMP_SENSORREC    0x28
#define DPS310_REG_MASK_TEMP_SENSORREC   0x80
#define DPS310_REG_SHIFT_TEMP_SENSORREC  7

#define DPS310_REG_INFO_TEMP_SE          DPS310_REG_ADR_TEMP_SE, \
                                         DPS310_REG_MASK_TEMP_SE, \
                                         DPS310_REG_SHIFT_TEMP_SE
#define DPS310_REG_ADR_TEMP_SE           0x09
#define DPS310_REG_MASK_TEMP_SE          0x08
#define DPS310_REG_SHIFT_TEMP_SE         3

#define DPS310_REG_INFO_PRS_MR           DPS310_REG_ADR_PRS_MR, \
                                         DPS310_REG_MASK_PRS_MR, \
                                         DPS310_REG_SHIFT_PRS_MR
#define DPS310_REG_ADR_PRS_MR            0x06
#define DPS310_REG_MASK_PRS_MR           0x70
#define DPS310_REG_SHIFT_PRS_MR          4

#define DPS310_REG_INFO_PRS_OSR          DPS310_REG_ADR_PRS_OSR, \
                                         DPS310_REG_MASK_PRS_OSR, \
                                         DPS310_REG_SHIFT_PRS_OSR
#define DPS310_REG_ADR_PRS_OSR           0x06
#define DPS310_REG_MASK_PRS_OSR          0x07
#define DPS310_REG_SHIFT_PRS_OSR         0

#define DPS310_REG_INFO_PRS_SE           DPS310_REG_ADR_PRS_SE, \
                                         DPS310_REG_MASK_PRS_SE, \
                                         DPS310_REG_SHIFT_PRS_SE
#define DPS310_REG_ADR_PRS_SE            0x09
#define DPS310_REG_MASK_PRS_SE           0x04
#define DPS310_REG_SHIFT_PRS_SE          2

#define DPS310_REG_INFO_TEMP_RDY         DPS310_REG_ADR_TEMP_RDY, \
                                         DPS310_REG_MASK_TEMP_RDY, \
                                         DPS310_REG_SHIFT_TEMP_RDY
#define DPS310_REG_ADR_TEMP_RDY          0x08
#define DPS310_REG_MASK_TEMP_RDY         0x20
#define DPS310_REG_SHIFT_TEMP_RDY        5

#define DPS310_REG_INFO_PRS_RDY          DPS310_REG_ADR_PRS_RDY, \
                                         DPS310_REG_MASK_PRS_RDY, \
                                         DPS310_REG_SHIFT_PRS_RDY
#define DPS310_REG_ADR_PRS_RDY           0x08
#define DPS310_REG_MASK_PRS_RDY          0x10
#define DPS310_REG_SHIFT_PRS_RDY         4

#define DPS310_REG_ADR_PRS               0x00
#define DPS310_REG_LEN_PRS               3

#define DPS310_REG_ADR_TEMP              0x03
#define DPS310_REG_LEN_TEMP              3

#define DPS310_REG_ADR_COEF              0x10
#define DPS310_REG_LEN_COEF              18

#define DPS310_REG_INFO_FIFO_EN          DPS310_REG_ADR_FIFO_EN, \
                                         DPS310_REG_MASK_FIFO_EN, \
                                         DPS310_REG_SHIFT_FIFO_EN
#define DPS310_REG_ADR_FIFO_EN           0x09
#define DPS310_REG_MASK_FIFO_EN          0x02
#define DPS310_REG_SHIFT_FIFO_EN         1
#define DPS310_FIFO_DISABLE              0x0
#define DPS310_FIFO_ENABLE               0x1

#define DPS310_REG_INFO_FIFO_FL          DPS310_REG_ADR_FIFO_FL, \
                                         DPS310_REG_MASK_FIFO_FL, \
                                         DPS310_REG_SHIFT_FIFO_FL
#define DPS310_REG_ADR_FIFO_FL           0x0C
#define DPS310_REG_MASK_FIFO_FL          0x80
#define DPS310_REG_SHIFT_FIFO_FL         7
#define DPS310_FIFO_FLUSH                0x1

#define DPS310_REG_INFO_FIFO_EMPTY       DPS310_REG_ADR_FIFO_EMPTY, \
                                         DPS310_REG_MASK_FIFO_EMPTY, \
                                         DPS310_REG_SHIFT_FIFO_EMPTY
#define DPS310_REG_ADR_FIFO_EMPTY        0x0B
#define DPS310_REG_MASK_FIFO_EMPTY       0x01
#define DPS310_REG_SHIFT_FIFO_EMPTY      0

#define DPS310_REG_INFO_FIFO_FULL        DPS310_REG_ADR_FIFO_FULL, \
                                         DPS310_REG_MASK_FIFO_FULL, \
                                         DPS310_REG_SHIFT_FIFO_FULL
#define DPS310_REG_ADR_FIFO_FULL         0x0B
#define DPS310_REG_MASK_FIFO_FULL        0x02
#define DPS310_REG_SHIFT_FIFO_FULL       1

#define DPS310_REG_INFO_INT_HL           DPS310_REG_ADR_INT_HL, \
                                         DPS310_REG_MASK_INT_HL, \
                                         DPS310_REG_SHIFT_INT_HL
#define DPS310_REG_ADR_INT_HL            0x09
#define DPS310_REG_MASK_INT_HL           0x80
#define DPS310_REG_SHIFT_INT_HL          7

#define DPS310_REG_INFO_INT_EN_FIFO      DPS310_REG_ADR_INT_EN_FIFO, \
                                         DPS310_REG_MASK_INT_EN_FIFO, \
                                         DPS310_REG_SHIFT_INT_EN_FIFO
#define DPS310_REG_ADR_INT_EN_FIFO       0x09
#define DPS310_REG_MASK_INT_EN_FIFO      0x40
#define DPS310_REG_SHIFT_INT_EN_FIFO     6

#define DPS310_REG_INFO_INT_EN_TEMP      DPS310_REG_ADR_INT_EN_TEMP, \
                                         DPS310_REG_MASK_INT_EN_TEMP, \
                                         DPS310_REG_SHIFT_INT_EN_TEMP
#define DPS310_REG_ADR_INT_EN_TEMP       0x09
#define DPS310_REG_MASK_INT_EN_TEMP      0x20
#define DPS310_REG_SHIFT_INT_EN_TEMP     5

#define DPS310_REG_INFO_INT_EN_PRS       DPS310_REG_ADR_INT_EN_PRS, \
                                         DPS310_REG_MASK_INT_EN_PRS, \
                                         DPS310_REG_SHIFT_INT_EN_PRS
#define DPS310_REG_ADR_INT_EN_PRS        0x09
#define DPS310_REG_MASK_INT_EN_PRS       0x10
#define DPS310_REG_SHIFT_INT_EN_PRS      4

#define DPS310_REG_INFO_INT_FLAG_FIFO    DPS310_REG_ADR_INT_FLAG_FIFO, \
                                         DPS310_REG_MASK_INT_FLAG_FIFO, \
                                         DPS310_REG_SHIFT_INT_FLAG_FIFO
#define DPS310_REG_ADR_INT_FLAG_FIFO     0x0A
#define DPS310_REG_MASK_INT_FLAG_FIFO    0x04
#define DPS310_REG_SHIFT_INT_FLAG_FIFO   2

#define DPS310_REG_INFO_INT_FLAG_TEMP    DPS310_REG_ADR_INT_FLAG_TEMP, \
                                         DPS310_REG_MASK_INT_FLAG_TEMP, \
                                         DPS310_REG_SHIFT_INT_FLAG_TEMP
#define DPS310_REG_ADR_INT_FLAG_TEMP     0x0A
#define DPS310_REG_MASK_INT_FLAG_TEMP    0x02
#define DPS310_REG_SHIFT_INT_FLAG_TEMP   1

#define DPS310_REG_INFO_INT_FLAG_PRS     DPS310_REG_ADR_INT_FLAG_PRS, \
                                         DPS310_REG_MASK_INT_FLAG_PRS, \
                                         DPS310_REG_SHIFT_INT_FLAG_PRS
#define DPS310_REG_ADR_INT_FLAG_PRS      0x0A
#define DPS310_REG_MASK_INT_FLAG_PRS     0x01
#define DPS310_REG_SHIFT_INT_FLAG_PRS    0

struct dps310_odr_info {
    int32_t odr;
    int32_t pre_rate;
    int32_t pre_osr;
    int32_t temp_rate;
    int32_t temp_osr;
};

const static struct dps310_odr_info dps310_odr_array[] = {
    {8,   3, 6, 3, 0},
    {16,  4, 5, 4, 0},
    {32,  5, 3, 5, 0},
    {64,  6, 2, 6, 0},
    {128, 7, 1, 6, 0},
};

const int32_t scaling_facts[] = {
    524288,
    1572864,
    3670016,
    7864320,
    253952,
    516096,
    1040384,
    2088960
};

typedef enum {
    IDLE_MODE        = 0x00,
    PRS_CMD_MODE     = 0x01,
    TEMP_CMD_MODE    = 0x02,
    INVAL_CMD_MODE1  = 0x03,
    INVAL_CMD_MODE2  = 0x04,
    PRS_BG_MODE      = 0x05,
    TEMP_BG_MODE     = 0x06,
    PRS_TEMP_BG_MODE = 0x07,
} DPS310_MODE;

struct dps310_rtdata {
    struct sns_port port;
    osMutexId_t mutex;
    osTimerId_t timer;

    float pressure;
    float temperature;
    uint32_t prs_raw1;
    uint32_t prs_raw0;
    uint32_t udelay;
    bool press_enabled;
    DPS310_MODE opMode;

    int32_t c0Half;
    int32_t c1;
    int32_t c00;
    int32_t c10;
    int32_t c01;
    int32_t c11;
    int32_t c20;
    int32_t c21;
    int32_t c30;
    uint8_t tempMr;
    uint8_t tempOsr;
    uint8_t tempSensor;
    uint8_t prsMr;
    uint8_t prsOsr;
    uint8_t lastTempScal;
};

static inline int dps310_read_data(struct sns_port *port, uint8_t cmd, uint8_t *pdata, size_t num)
{
    return sns_port_read(port, cmd, pdata, num);
}

static inline int dps310_read_byte_field(struct sns_port *port, uint8_t *pdata, uint8_t cmd, uint8_t mask, uint8_t shift)
{
    int ret;
    ret = sns_port_read(port, cmd, pdata, 1);
    if (ret < 0)
        return ret;
    *pdata = (*pdata & mask) >> shift;
    return ret;
}

static inline int dps310_write_reg(struct sns_port *port, uint8_t cmd, uint8_t val)
{
    return sns_port_write(port, cmd, &val, 1);
}

static inline int dps310_write_byte_field(struct sns_port *port, uint8_t pdata, uint8_t cmd, uint8_t mask, uint8_t shift)
{
    int ret;
    uint8_t reg;
    ret = sns_port_read(port, cmd, &reg, 1);
    if (ret < 0)
        return ret;
    reg = (reg & ~mask) | ((pdata << shift) & mask);
    return sns_port_write(port, cmd, &reg, 1);
}

static int dps310_find_odr(int expect)
{
    int i;

    for (i = 0; i < __countof(dps310_odr_array); i++) {
        if (expect <= dps310_odr_array[i].odr)
            return i;
    }

    return i - 1;
}

int dps310_get_calib_param(struct dps310_rtdata *rtdata)
{
    struct sns_port *sport = &rtdata->port;
    uint8_t buffer[DPS310_REG_LEN_COEF];

    int ret = dps310_read_data(sport, DPS310_REG_ADR_COEF, buffer, DPS310_REG_LEN_COEF);
    if (ret) {
        printf("DPS310 read DPS310_REG_ADR_COEF err, ret:%d\n", ret);
        return ret;
    }

    /* compose coefficients from buffer content */
    rtdata->c0Half = ((uint32_t)buffer[0] << 4) | (((uint32_t)buffer[1] >> 4) & 0x0F);
    /*
     * this construction recognizes non-32-bit negative numbers
     * and converts them to 32-bit negative numbers with 2's complement
     */
    if(rtdata->c0Half & ((uint32_t)1 << 11)) {
        rtdata->c0Half -= (uint32_t)1 << 12;
    }
    /* c0 is only used as c0*0.5, so c0_half is calculated immediately */
    rtdata->c0Half = rtdata->c0Half / 2U;

    /* now do the same thing for all other coefficients */
    rtdata->c1 = (((uint32_t)buffer[1] & 0x0F) << 8) | (uint32_t)buffer[2];
    if(rtdata->c1 & ((uint32_t)1 << 11)) {
        rtdata->c1 -= (uint32_t)1 << 12;
    }
    rtdata->c00 = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | (((uint32_t)buffer[5] >> 4) & 0x0F);
    if(rtdata->c00 & ((uint32_t)1 << 19)) {
        rtdata->c00 -= (uint32_t)1 << 20;
    }

    rtdata->c10 = (((uint32_t)buffer[5] & 0x0F) << 16) | ((uint32_t)buffer[6] << 8) | (uint32_t)buffer[7];
    if(rtdata->c10 & ((uint32_t)1 << 19)) {
        rtdata->c10 -= (uint32_t)1 << 20;
    }

    rtdata->c01 = ((uint32_t)buffer[8] << 8) | (uint32_t)buffer[9];
    if(rtdata->c01 & ((uint32_t)1 << 15)) {
        rtdata->c01 -= (uint32_t)1 << 16;
    }

    rtdata->c11 = ((uint32_t)buffer[10] << 8) | (uint32_t)buffer[11];
    if(rtdata->c11 & ((uint32_t)1 << 15)) {
        rtdata->c11 -= (uint32_t)1 << 16;
    }

    rtdata->c20 = ((uint32_t)buffer[12] << 8) | (uint32_t)buffer[13];
    if(rtdata->c20 & ((uint32_t)1 << 15)) {
        rtdata->c20 -= (uint32_t)1 << 16;
    }

    rtdata->c21 = ((uint32_t)buffer[14] << 8) | (uint32_t)buffer[15];
    if(rtdata->c21 & ((uint32_t)1 << 15)) {
        rtdata->c21 -= (uint32_t)1 << 16;
    }

    rtdata->c30 = ((uint32_t)buffer[16] << 8) | (uint32_t)buffer[17];
    if(rtdata->c30 & ((uint32_t)1 << 15)) {
        rtdata->c30 -= (uint32_t)1 << 16;
    }

    return 0;
}

uint32_t dps310_calc_busy_time(uint16_t mr, uint16_t osr)
{
    /* mask parameters first */
    mr &= DPS310_REG_MASK_TEMP_MR >> DPS310_REG_SHIFT_TEMP_MR;
    osr &= DPS310_REG_MASK_TEMP_OSR >> DPS310_REG_SHIFT_TEMP_OSR;
    /* formula from datasheet (optimized) */
    return ((uint32_t)20U << mr) + ((uint32_t)16U << (osr + mr));
}

/*
 * Calculates a scaled and compensated pressure value from raw data
 * raw: raw pressure value read from Dps310
 * returns: pressure value in Pa
 */
float dps310_calc_pressure(struct dps310_rtdata *rtdata, int32_t raw)
{
    double prs = raw;

    /* scale pressure according to scaling table and oversampling */
    prs /= scaling_facts[rtdata->prsOsr];

    /* Calculate compensated pressure */
    prs = rtdata->c00 + prs * (rtdata->c10 + prs * (rtdata->c20 + prs * rtdata->c30)) +
        rtdata->lastTempScal * (rtdata->c01 + prs * (rtdata->c11 + prs * rtdata->c21));

    return (float)prs;
}

int dps310_get_pressure(struct dps310_rtdata *rtdata, float *result)
{
    struct sns_port *sport = &rtdata->port;
    uint8_t buffer[3] = {0};

    int ret  = dps310_read_data(sport, DPS310_REG_ADR_PRS, buffer, DPS310_REG_LEN_PRS);
    if(ret) {
        return ret;
    }

    rtdata->prs_raw0 = (uint32_t)buffer[0] << 8 | (uint32_t)buffer[1];
    rtdata->prs_raw1 = (uint32_t)buffer[2];

    /* compose raw pressure value from buffer */
    int32_t prs = rtdata->prs_raw0 << 8 | rtdata->prs_raw1;
    /*
     * recognize non-32-bit negative numbers
     * and convert them to 32-bit negative numbers using 2's complement
     */
    if(prs & ((uint32_t)1 << 23)) {
        prs -= (uint32_t)1 << 24;
    }

    *result = dps310_calc_pressure(rtdata, prs);
    return 0;
}

/**
 * Calculates a scaled and compensated pressure value from raw data
 * raw: raw temperature value read from Dps310
 * returns: temperature value in °C
 */
float dps310_calc_temp(struct dps310_rtdata *rtdata, int32_t raw)
{
    double temp = raw;

    /* scale temperature according to scaling table and oversampling */
    temp /= scaling_facts[rtdata->tempOsr];

    /* update last measured temperature
     * it will be used for pressure compensation
     */
    rtdata->lastTempScal = temp;

    /* Calculate compensated temperature */
    temp = rtdata->c0Half + rtdata->c1 * temp;

    return (float)temp;
}

int dps310_get_temp(struct dps310_rtdata *rtdata, float *result)
{
    struct sns_port *sport = &rtdata->port;
    uint8_t buffer[3] = {0};

    int ret = dps310_read_data(sport, DPS310_REG_ADR_TEMP, buffer, DPS310_REG_LEN_TEMP);
    if(ret) {
        return ret;
    }

    /* compose raw temperature value from buffer */
    int32_t temp = (uint32_t)buffer[0] << 16 | (uint32_t)buffer[1] << 8 | (uint32_t)buffer[2];
    /*
     * recognize non-32-bit negative numbers
     * and convert them to 32-bit negative numbers using 2's complement
     */
    if(temp & ((uint32_t)1 << 23)) {
        temp -= (uint32_t)1 << 24;
    }

    *result = dps310_calc_temp(rtdata, temp);
    return 0;
}

int dps310_get_single_result(struct dps310_rtdata *rtdata, float *result)
{
    struct sns_port *sport = &rtdata->port;
    uint8_t status;
    DPS310_MODE oldMode;

    /* read finished bit for current opMode */
    switch(rtdata->opMode) {
        case TEMP_CMD_MODE:
            dps310_read_byte_field(sport, &status, DPS310_REG_INFO_TEMP_RDY);
            break;
        case PRS_CMD_MODE:
             dps310_read_byte_field(sport, &status, DPS310_REG_INFO_PRS_RDY);
             break;
        default:
             return -1;
    }

    switch(status) {
        /* ready flag not set, measurement still in progress */
        case 0:
            return -1;
        /* measurement ready, expected case */
        case 1:
            oldMode = rtdata->opMode;
            /* opcode was automatically reseted by DPS310 */
            rtdata->opMode = IDLE_MODE;
            switch(oldMode)
            {
                case TEMP_CMD_MODE:
                    return dps310_get_temp(rtdata, result);
                case PRS_CMD_MODE:
                    return dps310_get_pressure(rtdata, result);
                default:
                    return -1;
            }
        default:
             return -1;
    }
    return -1;
}

int dps310_set_opmode(struct dps310_rtdata *rtdata, DPS310_MODE opMode)
{
    struct sns_port *sport = &rtdata->port;
    int ret = -1;

    if(opMode == INVAL_CMD_MODE1 || opMode == INVAL_CMD_MODE2) {
        return ret;
    }

    ret = dps310_write_byte_field(sport, (uint8_t)opMode, DPS310_REG_INFO_OPMODE);
    if (ret) {
        printf("DPS310 set option mode err, ret:%d\n", ret);
        return ret;
    }

    rtdata->opMode= opMode;

    return ret;
}

int dps310_config_pressure(struct dps310_rtdata *rtdata, uint8_t prsMr, uint8_t prsOsr)
{
    struct sns_port *sport = &rtdata->port;
    int ret;

    prsMr &= DPS310_REG_MASK_PRS_MR >> DPS310_REG_SHIFT_PRS_MR;
    prsOsr &= DPS310_REG_MASK_PRS_OSR >> DPS310_REG_SHIFT_PRS_OSR;

    /* set config register according to parameters */
    uint8_t toWrite = prsMr << DPS310_REG_SHIFT_PRS_MR;
    toWrite |= prsOsr << DPS310_REG_SHIFT_PRS_OSR;

    ret = dps310_write_reg(sport, DPS310_REG_ADR_PRS_MR, toWrite);
    if(ret < 0) {
        printf("DPS310 write reg:DPS310_REG_ADR_PRS_MR err, ret:%d\n", ret);
        return ret;
    }

    /* set PM SHIFT ENABLE if oversampling rate higher than eight(2^3) */
    ret = dps310_write_byte_field(sport, (prsOsr > DPS310_OSR_SE)?1:0, DPS310_REG_INFO_PRS_SE);

    if(!ret) {
        rtdata->prsMr = prsMr;
        rtdata->prsOsr = prsOsr;
    } else {
        /* try to rollback on fail avoiding endless recursion
         * this is to make sure that shift enable and oversampling rate
         * are always consistent
         */
        if(prsMr != rtdata->prsMr || prsOsr != rtdata->prsOsr) {
            dps310_config_pressure(rtdata, rtdata->prsMr, rtdata->prsOsr);
        }
    }
    return ret;

}

int dps310_config_temp(struct dps310_rtdata *rtdata, uint8_t tempMr, uint8_t tempOsr)
{
    struct sns_port *sport = &rtdata->port;
    int ret;
    uint8_t toWrite;

    tempMr &= DPS310_REG_MASK_TEMP_MR >> DPS310_REG_SHIFT_TEMP_MR;
    tempOsr &= DPS310_REG_MASK_TEMP_OSR >> DPS310_REG_SHIFT_TEMP_OSR;

    /* set config register according to parameters */
    toWrite = tempMr << DPS310_REG_SHIFT_TEMP_MR;
    toWrite |= tempOsr << DPS310_REG_SHIFT_TEMP_OSR;
    /* using recommended temperature sensor */
    toWrite |= DPS310_REG_MASK_TEMP_SENSOR
        & (rtdata->tempSensor << DPS310_REG_SHIFT_TEMP_SENSOR);

    ret = dps310_write_reg(sport, DPS310_REG_ADR_TEMP_MR, toWrite);
    if(ret) {
        printf("DPS310 write reg:DPS310_REG_ADR_TEMP_MR err, ret:%d\n", ret);
        return ret;
    }

    /* set TEMP SHIFT ENABLE if oversampling rate higher than eight(2^3) */
    ret = dps310_write_byte_field(sport,(tempOsr > DPS310_OSR_SE)?1:0, DPS310_REG_INFO_TEMP_SE);

    if(!ret) {
        rtdata->tempMr = tempMr;
        rtdata->tempOsr = tempOsr;
    } else {
        /* try to rollback on fail avoiding endless recursion
         * this is to make sure that shift enable and oversampling rate
         * are always consistent
         */
        if(tempMr != rtdata->tempMr || tempOsr != rtdata->tempOsr) {
            dps310_config_temp(rtdata, rtdata->tempMr, rtdata->tempOsr);
        }
    }
    return ret;
}

static void dps310_set_standby_mode(struct dps310_rtdata *rtdata)
{
    struct sns_port *sport = &rtdata->port;
    int ret;

    /* set device to idling mode */
    ret = dps310_set_opmode(rtdata, IDLE_MODE);
    if(ret) {
        printf("DPS310 set opmode err, ret:%d\n", ret);
    }

    /* flush the FIFO */
    ret = dps310_write_byte_field(sport, DPS310_FIFO_FLUSH, DPS310_REG_INFO_FIFO_FL);
    if (ret) {
        printf("DPS310 flush fifo err, ret:%d\n", ret);
    }

    /* disable FIFO */
    ret = dps310_write_byte_field(sport, DPS310_FIFO_DISABLE, DPS310_REG_INFO_FIFO_EN);
    if (ret) {
        printf("DPS310 disable fifo err, ret:%d\n", ret);
    }
}

int dps310_measure_temp_once(struct dps310_rtdata *rtdata, float *result)
{
    /* Start measurement */
    int ret = dps310_set_opmode(rtdata, TEMP_CMD_MODE);
    if(ret) {
        return ret;
    }

    /* wait until measurement is finished */
    osDelay(osMsecToTick(dps310_calc_busy_time(0, rtdata->tempOsr) / 10));
    osDelay(osMsecToTick(10));

    ret = dps310_get_single_result(rtdata, result);
    if(ret) {
        dps310_set_standby_mode(rtdata);
    }
    return ret;
}

/*
 * Function to fix a hardware problem on some devices
 * You have this problem if you measure a temperature which is too high (e.g. 60°C when temperature is around 20°C)
 * Call correctTemp() directly after begin() to fix this issue
 */
int dps310_correct_temp(struct dps310_rtdata *rtdata)
{
    struct sns_port *sport = &rtdata->port;
    float trash;

    dps310_write_reg(sport, 0x0E, 0xA5);
    dps310_write_reg(sport, 0x0F, 0x96);
    dps310_write_reg(sport, 0x62, 0x02);
    dps310_write_reg(sport, 0x0E, 0x00);
    dps310_write_reg(sport, 0x0F, 0x00);

    /* perform a first temperature measurement (again)
     * the most recent temperature will be saved internally
     * and used for compensation when calculating pressure
     */
    dps310_measure_temp_once(rtdata, &trash);

    return 0;
}

static int dps310_sensor_init(struct dps310_rtdata *rtdata)
{
    struct sns_port *sport = &rtdata->port;
    int ret;
    float trash;
    uint8_t reg = 0;

    ret = dps310_read_byte_field(sport, &reg, DPS310_REG_INFO_PROD_ID);
    if (ret) {
        printf("dps310 read DPS310_REG_ADR_PROD_ID err, ret: %d\n", ret);
        return ret;
    }

    if (reg != DPS310_PRODUCT_ID) {
        printf("check DPS310_REG_ADR_PROD_ID failed\n");
        return -1;
    } else {
        printf("DPS310_REG_ADR_PROD_ID=0x%x\n", reg);
        ret = dps310_read_byte_field(sport, &reg, DPS310_REG_INFO_REV_ID);
        if (ret) {
            printf("DPS310 read DPS310_REG_ADR_REV_ID err, ret: %d\n", ret);
            return ret;
        }
        printf("DPS310_REG_ADR_REV_ID=0x%x\n", reg);
        if (reg != DPS310_REVISION_ID) {
            return -1;
        }
    }

    /* find out which temperature sensor is calibrated with coefficients... */
    ret = dps310_read_byte_field(sport, &reg, DPS310_REG_INFO_TEMP_SENSORREC);
    if (ret) {
        printf("DPS310 read DPS310_REG_ADR_TEMP_SENSORREC err, ret: %d\n", ret);
        return ret;
    }

    rtdata->tempSensor = reg;
    /* ...and use this sensor for temperature measurement */
    ret = dps310_write_byte_field(sport, reg, DPS310_REG_INFO_TEMP_SENSOR);
    if (ret) {
        printf("DPS310 write DPS310_REG_ADR_TEMP_SENSOR err, ret: %d\n", ret);
        return ret;
    }

    /* read coefficients */
    ret = dps310_get_calib_param(rtdata);
    if (ret) {
        printf("DPS310 get calib param err, ret: %d\n", ret);
        return ret;
    }

    /* set to standby for further configuration */
    dps310_set_standby_mode(rtdata);

    /* set measurement precision and rate to standard values; */
    dps310_config_temp(rtdata, DPS310_TEMP_STD_MR, DPS310_TEMP_STD_OSR);
    dps310_config_pressure(rtdata, DPS310_PRS_STD_MR, DPS310_PRS_STD_OSR);

    /* perform a first temperature measurement (again)
     * the most recent temperature will be saved internally
     * and used for compensation when calculating pressure float trash
     */
    dps310_measure_temp_once(rtdata, &trash);

    /* make sure the DPS310 is in standby after initialization */
    dps310_set_standby_mode(rtdata);

    /* Fix IC with a fuse bit problem, which lead to a wrong temperature
     * Should not affect ICs without this problem
     */
    dps310_correct_temp(rtdata);

    printf("dps310_sensor_init success\n");

    return 0;
}

static void dps310_worker(void *data, int64_t ts)
{
    struct dps310_rtdata *rtdata = data;
    struct sns_port *sport = &rtdata->port;
    uint8_t temp_status = 0, prs_status = 0;
    struct sensor_event prox_event;

    prox_event.timestamp = ts;

    dps310_read_byte_field(sport, &temp_status, DPS310_REG_INFO_TEMP_RDY);
    if (temp_status) {
        dps310_get_temp(rtdata, &rtdata->temperature);
    }
    dps310_read_byte_field(sport, &prs_status, DPS310_REG_INFO_PRS_RDY);
    if (prs_status) {
        dps310_get_pressure(rtdata, &rtdata->pressure);
    }

    if (prs_status) {
        prox_event.pressure_raw.raw0 = rtdata->prs_raw0;
        prox_event.pressure_raw.raw1 = rtdata->prs_raw1;
        prox_event.pressure_t.value = rtdata->pressure;
        prox_event.pressure_t.temperature = rtdata->temperature;
        prox_event.type = SENSOR_TYPE_PRESSURE;
        smgr_push_data(&prox_event, 1);
    }
}

static void dps310_timer_cb(void *arg)
{
    smgr_schedule_work(dps310_worker, arg, get_timestamp(), LOW_WORK);
}

static int dps310_activate(const struct sensor_dev *dev, bool enable)
{
    struct dps310_rtdata *rtdata = dev->rtdata;
    int ret = 0;
    bool old_status = false;
    bool new_status = false;

    printf("dps310_activate activate %s %s\n", dev->name, enable ? "true" : "false");

    osMutexAcquire(rtdata->mutex, osWaitForever);
    old_status = rtdata->press_enabled;

    if ((dev->type == SENSOR_TYPE_PRESSURE) && (rtdata->press_enabled != enable)) {
        rtdata->press_enabled = enable;
    } else {
        printf("unhandled case for active dps310\n");
        ret = -EINVAL;
        goto exit_err;
    }

    new_status = rtdata->press_enabled;
    if (old_status != new_status) {
        if (new_status) {
            dps310_set_opmode(rtdata, PRS_TEMP_BG_MODE);
            osTimerStart(rtdata->timer, osUsecToTick(rtdata->udelay));
        } else {
            dps310_set_opmode(rtdata, IDLE_MODE);
            osTimerStop(rtdata->timer);
        }
    }

exit_err:
    osMutexRelease(rtdata->mutex);
    return ret;
    return 0;
}

static void dps310_sync_odr(struct dps310_rtdata *rtdata)
{
    int actual_odr = 1000000/rtdata->udelay;
    int idx = dps310_find_odr(actual_odr);

    printf("dps310 set odr udelay:%d, odr:%d, idx:%d\n",
              rtdata->udelay, dps310_odr_array[idx].odr, idx);

    dps310_config_temp(rtdata, dps310_odr_array[idx].temp_rate, dps310_odr_array[idx].temp_osr);
    dps310_config_pressure(rtdata, dps310_odr_array[idx].pre_rate, dps310_odr_array[idx].pre_osr);

    if (rtdata->press_enabled && rtdata->timer)
        osTimerStart(rtdata->timer, osUsecToTick(1000000 / dps310_odr_array[idx].odr));
}

static int dps310_set_delay(const struct sensor_dev *dev, uint32_t us)
{
    struct dps310_rtdata *rtdata = dev->rtdata;

    printf("dps310 set delay:us:%d\n", us);
    if (rtdata->udelay != us) {
        osMutexAcquire(rtdata->mutex, osWaitForever);
        rtdata->udelay = us;
        dps310_sync_odr(rtdata);
        osMutexRelease(rtdata->mutex);
    }

    return 0;
}

static int dps310_selftest(const struct sensor_dev *dev, void *data)
{
    return 0;
}

static int dps310_set_offset(const struct sensor_dev *dev, void *offset)
{
    return 0;
}

static int dps310_get_offset(const struct sensor_dev *dev, void *offset)
{
    return 0;
}

static char* dps310_get_vendor(const struct sensor_dev *dev)
{
    return "infineon";
}

static char* dps310_get_module(const struct sensor_dev *dev)
{
    return "dps310";
}

static int dps310_get_type(const struct sensor_dev *dev)
{
    return SENSOR_TYPE_PRESSURE;
}

static float dps310_get_max_range(const struct sensor_dev *dev)
{
    return 1200;
}

static float dps310_get_power(const struct sensor_dev *dev)
{
    return 0.05;
}

static int dps310_get_min_delay(const struct sensor_dev *dev)
{
    int idx, odr;

    idx = dps310_find_odr(INT_MAX);
    odr = dps310_odr_array[idx].odr;

    return 1000000 / odr;
}

static int dps310_get_max_delay(const struct sensor_dev *dev)
{
    int idx, odr;

    idx = dps310_find_odr(0);
    odr = dps310_odr_array[idx].odr;

    return 1000000 / odr;
}

static float dps310_get_resolution(const struct sensor_dev *dev)
{
    return 0.06;
}

static struct sensor_ops dps310_ops = {
    .activate = dps310_activate,
    .set_delay = dps310_set_delay,
    .selftest = dps310_selftest,
    .set_offset = dps310_set_offset,
    .get_offset = dps310_get_offset,
    .get_vendor = dps310_get_vendor,
    .get_module = dps310_get_module,
    .get_type = dps310_get_type,
    .get_max_range = dps310_get_max_range,
    .get_power = dps310_get_power,
    .get_min_delay = dps310_get_min_delay,
    .get_max_delay = dps310_get_max_delay,
    .get_resolution = dps310_get_resolution,
};

static int dps310_probe(const struct sensor_platform_data *pdata,
            const struct sensor_matching_data *mdata,
            struct sensor_dev **dev, uint32_t *num_sensors)
{
    struct sensor_dev *sdevs;
    struct dps310_rtdata *rtdata;
    size_t msize;
    int ret;

    msize = sizeof(*sdevs) * 1 + sizeof(struct dps310_rtdata);
    sdevs = calloc(1, msize);
    if (!sdevs) {
        printf("dps310 failed to malloc\n");
        return -ENOMEM;
    }

    rtdata = (struct dps310_rtdata *)&sdevs[1];
    ret = sns_port_init(&pdata->bus_info, &rtdata->port);
    if (ret) {
        printf("failed to init port for %s\n", pdata->name);
        goto port_err;
    }

    rtdata->mutex = osMutexNew(NULL);
    if (!rtdata->mutex) {
        printf("create mutex failed for dps310\n");
        ret = osError;
        goto mutex_err;
    }

    rtdata->udelay = PRESS_DELAY_DEFAULT;
    rtdata->timer = osTimerNew(dps310_timer_cb, osTimerPeriodic, rtdata, NULL);
    if (!rtdata->timer) {
        printf("failed to create timer for dps310 sensor\n");
        ret = -ETIME;
        goto timer_err;
    }

    ret = dps310_sensor_init(rtdata);
    if (ret < 0) {
        printf("dps310_sensor_init failed\n");
        goto sensor_init_fail;
    }

    init_sensor_dev(&sdevs[0], PRESSURE_SENSOR, SENSOR_TYPE_PRESSURE,
            &dps310_ops, pdata, rtdata, mdata);

    *dev = sdevs;
    *num_sensors = 1;

    printf("sensor probed for dps310\n");

    return 0;

sensor_init_fail:
    osTimerDelete(rtdata->timer);
timer_err:
    osMutexDelete(rtdata->mutex);
    rtdata->mutex = NULL;
mutex_err:
    sns_port_deinit(&rtdata->port);
port_err:
    free(sdevs);

    return ret;
}

static const struct sensor_matching_data dps310_mdata = {
    .name = "infineon,dps310",
    .priv = NULL,
};

static const struct sensor_matching_data *drv_mdata[] = {
    &dps310_mdata,
    NULL,
};

static const struct sensor_driver __define_drv__ dps310_drv = {
    .name = "dps310",
    .mdata = &drv_mdata[0],
    .probe = dps310_probe,
};
