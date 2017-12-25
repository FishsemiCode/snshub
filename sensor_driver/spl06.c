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
#include "spl06.h"
#include "utils.h"


#define PRESSURE_SENSOR    "pressure"
#define TEMPERATURE_SENSOR "temperature"


#define SPL06_I2C_ADDRESS_1 0x76
#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define SENSOR_PRESSURE     0
#define SENSOR_TEMPERATURE  1
#define SPL06_PRODUCT_ID    0x10
#define SPL06_REG_CHECK_SUCCESS 0x02
#define SPL06_REG_CFG_COEF_RDY  0x80
#define SPL06_REG_TEMP_INT_RDY  0x2
#define SPL06_REG_PRESSURE_INT_RDY  0x1

#define SPL06_REG_PROD_ID           0x0D

#define SPL06_REG_PSR_B2            0x00
#define SPL06_REG_PSR_B1            0x01
#define SPL06_REG_PSR_B0            0x02
#define SPL06_REG_TMP_B2            0x03
#define SPL06_REG_TMP_B1            0x04
#define SPL06_REG_TMP_B0            0x05

#define SPL06_REG_PRS_CFG           0x06
#define SPL06_REG_TMP_CFG           0x07
#define SPL06_REG_MEAS_CFG          0x08
#define SPL06_REG_CFG_REG           0x09
#define SPL06_REG_INT_STS           0x0A
#define SPL06_REG_SOFT_RST          0X0C


#define SPL06_REG_COEF_C0           (0x10)
#define SPL06_REG_COEF_C0l1m        (0x11)
#define SPL06_REG_COEF_C1l          (0x12)
#define SPL06_REG_COEF_C00h         (0x13)
#define SPL06_REG_COEF_C00m         (0x14)
#define SPL06_REG_COEF_C0010        (0x15)
#define SPL06_REG_COEF_C10m         (0x16)
#define SPL06_REG_COEF_C10l         (0x17)
#define SPL06_REG_COEF_C01h         (0x18)
#define SPL06_REG_COEF_C01l         (0x19)
#define SPL06_REG_COEF_C11h         (0x1A)
#define SPL06_REG_COEF_C11l         (0x1B)
#define SPL06_REG_COEF_C20h         (0x1C)
#define SPL06_REG_COEF_C20l         (0x1D)
#define SPL06_REG_COEF_C21h         (0x1E)
#define SPL06_REG_COEF_C21l         (0x1F)
#define SPL06_REG_COEF_C30h         (0x20)
#define SPL06_REG_COEF_C30l         (0x21)
#define SPL06_REG_CHECK             (0x32)

#define PRESS_DELAY_DEFAULT         (1000000/100) /* 100HZ */

struct spl0601_calib_param {
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
};

struct spl0601_odr_info {
    int32_t odr;
    int32_t measurement_rate;
    int32_t oversampling_rate;
};

/* max percision set according to the time:1000/measurement_rate/2 */
const static struct spl0601_odr_info spl0601_odr_array[] = {
    {5,  8,  32},
    {15, 16, 16},
    {25, 32, 8},
    {50, 64, 2},
    {100, 128, 0},
};

struct spl0601_rtdata {
    struct sns_port port;
    osMutexId_t mutex;
    struct spl0601_calib_param calib_param; /* calibration data */

    bool press_enabled;
    bool temp_enabled;
    bool readable;
    bool t_p_switch; /* 0 for temperature, 1 for pressure */

    int32_t prepare_flag;
    uint32_t udelay;
    int32_t i32rawPressure;
    int32_t i32rawTemperature;
    int32_t i32kP;
    int32_t i32kT;
    float pressure;
    float temperature;
    int32_t fTsc;
    int32_t fPsc;

    osTimerId_t timer;
};


static inline int spl0601_read_data(struct sns_port *port, uint8_t cmd, uint8_t *pdata, size_t num)
{
    return sns_port_read(port, cmd, pdata, num);
}

static inline int spl0601_read_reg(struct sns_port *port, uint8_t cmd, uint8_t *pdata)
{
    return sns_port_read(port, cmd, pdata, 1);
}

static inline int spl0601_write_reg(struct sns_port *port, uint8_t cmd, uint8_t val)
{
    return sns_port_write(port, cmd, &val, 1);
}

static void spl0601_start_temperature(struct spl0601_rtdata *spl)
{
    spl0601_write_reg(&spl->port, SPL06_REG_MEAS_CFG,  0x02);
}

static void spl0601_start_pressure(struct spl0601_rtdata *spl)
{
    spl0601_write_reg(&spl->port, SPL06_REG_MEAS_CFG,  0x01);
}

static void spl0601_get_raw_temp(struct spl0601_rtdata *spl)
{
    uint8_t reg[3]={0};

    spl0601_read_data(&spl->port, SPL06_REG_TMP_B2, reg, 3);
    spl->i32rawTemperature = (int32_t)reg[0] << 16 | (int32_t)reg[1] << 8 | (int32_t)reg[2];
    spl->i32rawTemperature= (spl->i32rawTemperature & 0x800000)?(0xFF000000 | spl->i32rawTemperature):spl->i32rawTemperature;
}

static void spl0601_get_raw_pressure(struct spl0601_rtdata *spl)
{
   uint8_t reg[3] = {0};

   spl0601_read_data(&spl->port, SPL06_REG_PSR_B2, reg, 3);
   spl->i32rawPressure = (int32_t)reg[0] << 16 | (int32_t)reg[1] << 8 | (int32_t)reg[2];
   spl->i32rawPressure = (spl->i32rawPressure & 0x800000)?(0xFF000000 | spl->i32rawPressure):spl->i32rawPressure;
}

static void spl0601_get_temperature(struct spl0601_rtdata *spl)
{
    float fTCompensate;
    float fTsc;

    fTsc = spl->i32rawTemperature / (float)spl->i32kT;
    fTCompensate =  spl->calib_param.c0 * 0.5 + spl->calib_param.c1 * fTsc;
    spl->temperature = fTCompensate;
    spl->fTsc = fTsc;
}

static void spl0601_get_pressure(struct spl0601_rtdata *spl)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = spl->i32rawTemperature / (float)spl->i32kT;
    fPsc = spl->i32rawPressure / (float)spl->i32kP;
    qua2 = spl->calib_param.c10 + fPsc * (spl->calib_param.c20 + fPsc * spl->calib_param.c30);
    qua3 = fTsc * fPsc * (spl->calib_param.c11 + fPsc * spl->calib_param.c21);

    fPCompensate = spl->calib_param.c00 + fPsc * qua2 + fTsc * spl->calib_param.c01 + qua3;
    spl->pressure = fPCompensate;
    spl->fTsc = fTsc;
    spl->fPsc = fPsc;
}

static void spl0601_rateset(struct spl0601_rtdata *rtdata, uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    struct sns_port *sport = &rtdata->port;
    uint8_t reg = 0;
    int32_t i32kPkT = 0;

    switch(u8SmplRate) {
    case 2:
         reg |= (1 << 4);
        break;
    case 4:
        reg |= (2 << 4);
        break;
    case 8:
        reg |= (3 << 4);
        break;
    case 16:
        reg |= (4 << 4);
        break;
    case 32:
        reg |= (5 << 4);
        break;
    case 64:
        reg |= (6 << 4);
        break;
    case 128:
        reg |= (7 << 4);
        break;
    case 1:
    default:
        break;
    }

    switch(u8OverSmpl) {
    case 2:
        reg |= 1;
        i32kPkT = 1572864;
        break;
    case 4:
        reg |= 2;
        i32kPkT = 3670016;
        break;
    case 8:
        reg |= 3;
        i32kPkT = 7864320;
        break;
    case 16:
        i32kPkT = 253952;
        reg |= 4;
        break;
    case 32:
        i32kPkT = 516096; /* 101 - 32 measurements pr. sec. */
        reg |= 5;
        break;
    case 64:
        i32kPkT = 1040384;
        reg |= 6;
        break;
    case 128:
        i32kPkT = 2088960;
        reg |= 7;
        break;
    case 1:
    default:
        i32kPkT = 524288;
        break;
    }

    if (iSensor == SENSOR_PRESSURE) {
        rtdata->i32kP = i32kPkT;
        spl0601_write_reg(sport, SPL06_REG_PRS_CFG, reg);
        if (u8OverSmpl > 8) {
            spl0601_read_reg(sport, SPL06_REG_CFG_REG, &reg);
            printf("SPL06_REG_CFG_REG: 0x%x\n", reg);
            spl0601_write_reg(sport, SPL06_REG_CFG_REG, reg | 0x04);
        }
    }
    if (iSensor == SENSOR_TEMPERATURE) {
        rtdata->i32kT = i32kPkT;
        spl0601_write_reg(sport, SPL06_REG_TMP_CFG, reg | 0x80);/* Using mems temperature */
        if (u8OverSmpl > 8) {
            spl0601_read_reg(sport, SPL06_REG_CFG_REG, &reg);
            printf("SPL06_REG_CFG_REG: 0x%x\n", reg);
            spl0601_write_reg(sport, SPL06_REG_CFG_REG, reg | 0x08);
        }
    }

}

static int spl0601_get_calib_param(struct spl0601_rtdata *rtdata)
{
    struct sns_port *sport = &rtdata->port;
    uint8_t h;
    uint8_t m;
    uint8_t l;

    spl0601_read_reg(sport, SPL06_REG_COEF_C0, &h);
    spl0601_read_reg(sport, SPL06_REG_COEF_C0l1m, &l);
    rtdata->calib_param.c0 = (int16_t) (((h << 4) & 0x0FF0) | (((l >> 4) & 0x0F) & 0x00FF));
    rtdata->calib_param.c0 = (rtdata->calib_param.c0 & 0x0800)?(0xF000 | rtdata->calib_param.c0):rtdata->calib_param.c0;

    spl0601_read_reg(sport, SPL06_REG_COEF_C0l1m, &h);
    spl0601_read_reg(sport, SPL06_REG_COEF_C1l, &l);
    rtdata->calib_param.c1 = (int16_t)((((h & 0x0F) << 8 ) & 0x0F00) | (l & 0x00FF));
    rtdata->calib_param.c1 = (rtdata->calib_param.c1 & 0x0800)?(0xF000 | rtdata->calib_param.c1):rtdata->calib_param.c1;

    spl0601_read_reg(sport, SPL06_REG_COEF_C00h, &h);
    spl0601_read_reg(sport, SPL06_REG_COEF_C00m, &m);
    spl0601_read_reg(sport, SPL06_REG_COEF_C0010, &l);
    rtdata->calib_param.c00 = (int32_t)(((h << 12) & 0x000FF000) | ((m << 4) & 0x00000FF0) | ((l >> 4) & 0x0000000F));
    rtdata->calib_param.c00 = (rtdata->calib_param.c00 & 0x080000)?(0xFFF00000 | rtdata->calib_param.c00):rtdata->calib_param.c00;

    spl0601_read_reg(sport, SPL06_REG_COEF_C0010, &h);
    spl0601_read_reg(sport, SPL06_REG_COEF_C10m, &m);
    spl0601_read_reg(sport, SPL06_REG_COEF_C10l, &l);
    rtdata->calib_param.c10 = (int32_t)((((h & 0x0F) << 16) & 0x000F0000) | (( m << 8) & 0x0000FF00 ) | (l & 0x000000FF));
    rtdata->calib_param.c10 = (rtdata->calib_param.c10 & 0x080000)?(0xFFF00000 | rtdata->calib_param.c10):rtdata->calib_param.c10;

    spl0601_read_reg(sport, SPL06_REG_COEF_C01h, &h);
    spl0601_read_reg(sport, SPL06_REG_COEF_C01l, &l);
    rtdata->calib_param.c01 = (int16_t)(((h << 8) & 0xFF00) | (l & 0x00FF));

    spl0601_read_reg(sport, SPL06_REG_COEF_C11h, &h);
    spl0601_read_reg(sport, SPL06_REG_COEF_C11l, &l);
    rtdata->calib_param.c11 = (int16_t)(((h << 8) & 0xFF00) | (l & 0x00FF));

    spl0601_read_reg(sport, SPL06_REG_COEF_C20h, &h);
    spl0601_read_reg(sport, SPL06_REG_COEF_C20l, &l);
    rtdata->calib_param.c20 = (int16_t)(((h << 8) & 0xFF00) | (l & 0x00FF));

    spl0601_read_reg(sport, SPL06_REG_COEF_C21h, &h);
    spl0601_read_reg(sport, SPL06_REG_COEF_C21l, &l);
    rtdata->calib_param.c21 = (int16_t)(((h << 8) & 0xFF00) | (l & 0x00FF));

    spl0601_read_reg(sport, SPL06_REG_COEF_C30h, &h);
    spl0601_read_reg(sport, SPL06_REG_COEF_C30l, &l);
    rtdata->calib_param.c30 = (int16_t)(((h << 8) & 0xFF00) | (l & 0x00FF));

    return 0;
}

static int spl0601_sensor_data_trig(struct spl0601_rtdata *rtdata)
{
    if (!rtdata->readable) {
        if (rtdata->t_p_switch) {
            /* triger pressure reading */
            spl0601_start_pressure(rtdata);
        } else {
            /* triger temperature reading */
            spl0601_start_temperature(rtdata);
        }
        rtdata->readable = true;
    }

    return 0;
}

static int spl0601_sensor_data_get(struct spl0601_rtdata *rtdata)
{
    if (rtdata->readable) {
        if (rtdata->t_p_switch) { /* read pressure */
            spl0601_get_raw_pressure(rtdata);
            spl0601_get_pressure(rtdata);
            rtdata->readable = false;
            rtdata->t_p_switch = 0;
        } else { /* read temperature */
            spl0601_get_raw_temp(rtdata);
            spl0601_get_temperature(rtdata);
            rtdata->readable = false;
            rtdata->t_p_switch = 1;
        }
    }

    return 0;
}

static void spl0601_timer_cb(void *arg)
{
    struct spl0601_rtdata *rtdata = arg;
    struct sensor_event prox_event;
    struct sns_port *sport = &rtdata->port;
    uint8_t reg;

    if (rtdata->prepare_flag == 0)
        return;

    memset(&prox_event, 0, sizeof(prox_event));
    prox_event.timestamp = get_timestamp();

    if (!rtdata->temp_enabled && !rtdata->press_enabled) {
        printf("both disabled\n");
        return;
    }

    spl0601_read_reg(sport, SPL06_REG_INT_STS, &reg);

    if (!((reg & SPL06_REG_PRESSURE_INT_RDY) || (reg & SPL06_REG_TEMP_INT_RDY)))
        return;

    spl0601_sensor_data_get(rtdata);
    if (rtdata->press_enabled && !rtdata->t_p_switch) {
        prox_event.pressure_raw.raw0 = rtdata->fTsc;
        prox_event.pressure_raw.raw1 = rtdata->fPsc;
        prox_event.pressure_t.value = rtdata->pressure;
        prox_event.pressure_t.temperature = rtdata->temperature;

        prox_event.type = SENSOR_TYPE_PRESSURE;
        smgr_push_data(&prox_event, 1);
    }

    spl0601_sensor_data_trig(rtdata);
}

static int spl0601_sensor_init(struct spl0601_rtdata *rtdata)
{
    struct sns_port *sport = &rtdata->port;
    int ret;
    uint8_t reg = 0;

    ret = spl0601_write_reg(sport, SPL06_REG_SOFT_RST, 0x09);
    if (ret < 0) {
        printf("spl0601 write SPL06_REG_SOFT_RST err, ret: %d\n", ret);
        return ret;
    }

    osDelay(osUsecToTick(40000));

    ret = spl0601_read_reg(sport, SPL06_REG_PROD_ID, &reg);
    if (ret < 0) {
        printf("spl0601 read SPL06_REG_PROD_ID err, ret: %d\n", ret);
        return ret;
    }

    if (reg != SPL06_PRODUCT_ID) {
        printf("check SPL06_REG_PROD_ID failed\n");
        return -1;
    } else {
        printf("SPL06_REG_PROD_ID=0x%x\n", reg);
        ret = spl0601_read_reg(sport, SPL06_REG_CHECK, &reg);
        if (ret < 0) {
            printf("spl0601 read SPL06_REG_CHECK err, ret: %d\n", ret);
            return ret;
        }
        printf("reg SPL06_REG_CHECK=%d\n", reg);
        if (reg != SPL06_REG_CHECK_SUCCESS) {
            return -1;
        }
    }

    ret = spl0601_read_reg(sport, SPL06_REG_MEAS_CFG, &reg);
    if (ret < 0) {
        printf("spl0601 read SPL06_REG_MEAS_CFG err, ret: %d\n", ret);
        return ret;
    }

    while((reg & SPL06_REG_CFG_COEF_RDY) == 0) {
        ret = spl0601_read_reg(sport, SPL06_REG_MEAS_CFG, &reg);
        if (ret < 0) {
            printf("spl0601 read SPL06_REG_MEAS_CFG err, ret: %d\n", ret);
            return ret;
        }
    }

    spl0601_get_calib_param(rtdata);
    spl0601_rateset(rtdata, SENSOR_PRESSURE, 128, 0);
    spl0601_rateset(rtdata, SENSOR_TEMPERATURE, 128, 0);

    /*set temp\pressure data ready interrupt*/
    spl0601_read_reg(sport, SPL06_REG_CFG_REG, &reg);
    spl0601_write_reg(sport, SPL06_REG_CFG_REG, reg | 0x30);

    rtdata->i32rawPressure = 0;
    rtdata->i32rawTemperature = 0;
    rtdata->t_p_switch = 0;
    rtdata->readable = false;
    printf("spl0601_sensor_init success\n");

    return 0;
}

static int spl0601_activate(const struct sensor_dev *dev, bool enable)
{
    struct spl0601_rtdata *rtdata = dev->rtdata;
    int ret = 0;
    bool old_status = false;
    bool new_status = false;

    printf("spl0601_activate activate %s %s\n", dev->name, enable ? "true" : "false");

    osMutexAcquire(rtdata->mutex, osWaitForever);
    old_status = rtdata->press_enabled || rtdata->temp_enabled;

    if ((dev->type == SENSOR_TYPE_PRESSURE) && (rtdata->press_enabled != enable)) {
        rtdata->press_enabled = enable;
    } else if ((dev->type == SENSOR_TYPE_TEMPERATURE) && (rtdata->temp_enabled != enable)) {
        rtdata->temp_enabled = enable;
    } else if ((dev->type != SENSOR_TYPE_PRESSURE) && (dev->type != SENSOR_TYPE_TEMPERATURE)) {
        printf("unhandled case for active spl0601\n");
        ret = -EINVAL;
        goto exit_err;
    }

    new_status = rtdata->press_enabled || rtdata->temp_enabled;
    if (old_status != new_status) {
        if (new_status) {
            spl0601_sensor_data_trig(rtdata);
            osTimerStart(rtdata->timer, osUsecToTick(rtdata->udelay));
        } else {
            osTimerStop(rtdata->timer);
        }
    }

exit_err:
    osMutexRelease(rtdata->mutex);
    return ret;
}

static int spl_find_odr(int expect)
{
    int i;

    for (i = 0; i < __countof(spl0601_odr_array); i++) {
        if (expect <= spl0601_odr_array[i].odr)
            return i;
    }

    return i - 1;
}

static void spl_sync_odr(struct spl0601_rtdata *rtdata)
{
    int actual_odr = 1000000/rtdata->udelay;
    int idx = spl_find_odr(actual_odr);

    printf("spl0601 set odr udelay:%d, odr:%d, idx:%d\n",
              rtdata->udelay, actual_odr, idx);
    spl0601_rateset(rtdata, SENSOR_PRESSURE, spl0601_odr_array[idx].measurement_rate, spl0601_odr_array[idx].oversampling_rate);
    spl0601_rateset(rtdata, SENSOR_TEMPERATURE, spl0601_odr_array[idx].measurement_rate, spl0601_odr_array[idx].oversampling_rate);
}

static int spl0601_set_delay(const struct sensor_dev *dev, uint32_t us)
{
    struct spl0601_rtdata *rtdata = dev->rtdata;

    if (rtdata->udelay != us) {
        osMutexAcquire(rtdata->mutex, osWaitForever);
        rtdata->udelay = us;
        spl_sync_odr(rtdata);
        osMutexRelease(rtdata->mutex);
    }

    return 0;
}

static int spl0601_selftest(const struct sensor_dev *dev, void *data)
{
    return 0;
}

static int spl0601_set_offset(const struct sensor_dev *dev, void *offset)
{
    return 0;
}

static int spl0601_get_offset(const struct sensor_dev *dev, void *offset)
{
    return 0;
}

static char* spl0601_get_vendor(const struct sensor_dev *dev)
{
    return "GoerTek";
}

static char* spl0601_get_module(const struct sensor_dev *dev)
{
    return "spl0601";
}

static int spl0601_get_type(const struct sensor_dev *dev)
{
    return 0;
}

static float spl0601_get_max_range(const struct sensor_dev *dev)
{
    return 0;
}

static float spl0601_get_power(const struct sensor_dev *dev)
{
    return 0.05;
}

static int spl0601_get_min_delay(const struct sensor_dev *dev)
{
    int idx, odr;

    idx = spl_find_odr(INT_MAX);
    odr = spl0601_odr_array[idx].odr;

    return 1000000 / odr;
}

static int spl0601_get_max_delay(const struct sensor_dev *dev)
{
    int idx, odr;

    idx = spl_find_odr(0);
    odr = spl0601_odr_array[idx].odr;

    return 1000000 / odr;
}

static float spl0601_get_resolution(const struct sensor_dev *dev)
{
    return 0;
}

static struct sensor_ops spl0601_ops = {
    .activate = spl0601_activate,
    .set_delay = spl0601_set_delay,
    .selftest = spl0601_selftest,
    .set_offset = spl0601_set_offset,
    .get_offset = spl0601_get_offset,
    .get_vendor = spl0601_get_vendor,
    .get_module = spl0601_get_module,
    .get_type = spl0601_get_type,
    .get_max_range = spl0601_get_max_range,
    .get_power = spl0601_get_power,
    .get_min_delay = spl0601_get_min_delay,
    .get_max_delay = spl0601_get_max_delay,
    .get_resolution = spl0601_get_resolution,
};

static int spl0601_probe(const struct sensor_platform_data *pdata,
            const struct sensor_matching_data *mdata,
            struct sensor_dev **dev, uint32_t *num_sensors)
{
    struct sensor_dev *sdevs;
    struct spl0601_rtdata *rtdata;
    size_t msize;
    int ret;

    msize = sizeof(*sdevs) * 1 + sizeof(struct spl0601_rtdata);
    sdevs = calloc(1, msize);
    if (!sdevs) {
        printf("spl0601 failed to malloc\n");
        return -ENOMEM;
    }

    rtdata = (struct spl0601_rtdata *)&sdevs[1];
    ret = sns_port_init(&pdata->bus_info, &rtdata->port);
    if (ret) {
        printf("failed to init port for %s\n", pdata->name);
        goto port_err;
    }

    rtdata->mutex = osMutexNew(NULL);
    if (!rtdata->mutex) {
        printf("create mutex failed for spl0601\n");
        ret = osError;
        goto mutex_err;
    }

    rtdata->timer = osTimerNew(spl0601_timer_cb, osTimerPeriodic, rtdata, NULL);
    if (!rtdata->timer) {
        printf("failed to create timer for spl0601 sensor\n");
        ret = -ETIME;
        goto timer_err;
    }

    rtdata->udelay = PRESS_DELAY_DEFAULT;
    ret = spl0601_sensor_init(rtdata);
    if (ret < 0) {
        printf("spl0601_sensor_init failed\n");
        goto sensor_init_fail;
    }

    init_sensor_dev(&sdevs[0], PRESSURE_SENSOR, SENSOR_TYPE_PRESSURE,
            &spl0601_ops, pdata, rtdata, mdata);

    *dev = sdevs;
    *num_sensors = 1;

    rtdata->prepare_flag = 1;

    printf("sensor probed for spl0601\n");

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

static const struct sensor_matching_data spl0601_mdata = {
    .name = "goertek,spl0601",
    .priv = NULL,
};

static const struct sensor_matching_data *drv_mdata[] = {
    &spl0601_mdata,
    NULL,
};

static const struct sensor_driver __define_drv__ spl0601_drv = {
    .name = "spl0601",
    .mdata = &drv_mdata[0],
    .probe = spl0601_probe,
};
