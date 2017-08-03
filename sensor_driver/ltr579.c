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
#include "ltr579.h"
#include "utils.h"

#define PROXIMITY_SENSOR    "proximity"
#define LIGHT_SENSOR        "light"

#define FLAG_FIRST_ALS_COMPLETE     0x0001


/*Define Register Code*/
#define MAIN_CTRL           0x00
#define PS_LED              0x01
#define PS_PULSES           0x02
#define PS_MEAS_RATE        0x03
#define ALS_MEAS_RATE       0x04
#define ALS_GAIN            0x05
#define PART_ID             0x06
#define MAIN_STATUS         0x07
#define PS_DATA_0           0x08
#define PS_DATA_1           0x09
#define ALS_DATA_0          0x0D
#define ALS_DATA_1          0x0E
#define ALS_DATA_2          0x0F
#define INT_CFG             0x19
#define INT_PST             0x1A
#define PS_THRES_UP_0       0x1B
#define PS_THRES_UP_1       0x1C
#define PS_THRES_LOW_0      0x1D
#define PS_THRES_LOW_1      0x1E
#define PS_CAN_0            0x1F
#define PS_CAN_1            0x20
#define ALS_THRES_UP_0      0x21
#define ALS_THRES_UP_1      0x22
#define ALS_THRES_UP_2      0x23
#define ALS_THRES_LOW_0     0x24
#define ALS_THRES_LOW_1     0x25
#define ALS_THRES_LOW_2     0x26

#define PRX_ACTIVE          (1 << 0)
#define ALS_ACTIVE          (1 << 1)

#define PRX_LED_2P5MA       (0 << 0)
#define PRX_LED_5MA         (1 << 0)
#define PRX_LED_10MA        (2 << 0)
#define PRX_LED_25MA        (3 << 0)
#define PRX_LED_50MA        (4 << 0)
#define PRX_LED_75MA        (5 << 0)
#define PRX_LED_100MA       (6 << 0)
#define PRX_LED_125MA       (7 << 0)

#define PRX_LED_60KHZ       (3 << 4)
#define PRX_LED_70KHZ       (4 << 4)
#define PRX_LED_80KHZ       (5 << 4)
#define PRX_LED_90KHZ       (6 << 4)
#define PRX_LED_100KHZ      (7 << 4)

#define PRX_INTV_6P25MS     (1 << 0)
#define PRX_INTV_12P5MS     (2 << 0)
#define PRX_INTV_25MS       (3 << 0)
#define PRX_INTV_50MS       (4 << 0)
#define PRX_INTV_100MS      (5 << 0)
#define PRX_INTV_200MS      (6 << 0)
#define PRX_INTV_400MS      (7 << 0)

#define PRX_DATA_WIDTH_8    (0 << 3)
#define PRX_DATA_WIDTH_9    (1 << 3)
#define PRX_DATA_WIDTH_10   (2 << 3)
#define PRX_DATA_WIDTH_11   (3 << 3)

#define ALS_GAIN_1X         (0 << 0)
#define ALS_GAIN_3X         (1 << 0)
#define ALS_GAIN_6X         (2 << 0)
#define ALS_GAIN_9X         (3 << 0)
#define ALS_GAIN_18X        (4 << 0)

#define ALS_INTGR_400MS     (0 << 4)
#define ALS_INTGR_200MS     (1 << 4)
#define ALS_INTGR_100MS     (2 << 4)
#define ALS_INTGR_50MS      (3 << 4)
#define ALS_INTGR_25MS      (4 << 4)

#define ALS_INTV_25MS       (0 << 0)
#define ALS_INTV_50MS       (1 << 0)
#define ALS_INTV_100MS      (2 << 0)
#define ALS_INTV_500MS      (3 << 0)
#define ALS_INTV_1000MS     (5 << 0)
#define ALS_INTV_2000MS     (6 << 0)

#define PRX_NEW_DATA_READY  (1 << 0)
#define PRX_OVERFLOW        (1 << 3)
#define ALS_NEW_DATA_READY  (1 << 3)

#define PRX_CONTR_DEFAULT   (0)
#define PRX_LED_DEFAULT     (PRX_LED_125MA | PRX_LED_60KHZ)
#define PRX_PULSE_DEFAULT   (24)
#define PRX_WIDTH_DEFAULT   PRX_DATA_WIDTH_11
#define PRX_MEAS_DEFAULT    (1 << 6 | PRX_INTV_50MS | PRX_WIDTH_DEFAULT)

#define ALS_GAIN_DEFAULT    (ALS_GAIN_18X)
#define ALS_MEAS_DEFAULT    (ALS_INTGR_100MS | ALS_INTV_100MS)
#define ALS_MEAS_HIGH_RES   (ALS_INTGR_400MS | ALS_INTV_500MS)

#define CHIP_ID             0xb1
#define PROX_DELAY          100
#define ALS_DELAY           120

#define ALS_CHANGE_RES_LOW  20
#define ALS_CHANGE_RES_HIGH 2000

#define ALS_HIGH_RES_FACTOR     (900)
#define ALS_NORMAL_RES_FACTOR   (900 / 4)
#define ALS_LOW_RES_FACTOR      (50 / 8)

#define ALS_HIGH_RES        0
#define ALS_NORMAL_RES      1
#define ALS_LOW_RES         2

struct ltr579_rtdata {
    struct sns_port port;
    osMutexId_t mutex;
    uint8_t main_status;
    osThreadId_t prox_thread;

    osTimerId_t prox_timer;
    bool        prox_enabled;
    bool        prox_continus;
    bool        prox_first_data;
    bool        prox_near;
    uint16_t    prox_offset;
    uint16_t    prox_raw;

    osTimerId_t als_timer;
    bool        als_enabled;
    bool        als_continus;
    int         als_res_idx;
    float       als_res;
    bool        als_res_changed;
    int         als_raw;
};

struct ltr579_als_res_config {
    uint8_t als_control;
    uint8_t als_measure_rate;
    uint32_t als_res_factor;
};

static const struct ltr579_als_res_config  ltr579_als_res[] = {
    [ALS_HIGH_RES] = {ALS_GAIN_18X, ALS_MEAS_HIGH_RES, ALS_HIGH_RES_FACTOR},
    [ALS_NORMAL_RES] = {ALS_GAIN_18X, ALS_MEAS_DEFAULT, ALS_NORMAL_RES_FACTOR},
    [ALS_LOW_RES] = {ALS_GAIN_1X, ALS_MEAS_DEFAULT, ALS_LOW_RES_FACTOR},
};

static inline int ltr579_read_data(struct sns_port *port, uint8_t cmd, uint8_t *pdata, size_t num)
{
    return sns_port_read(port, cmd, pdata, num);
}

static inline int ltr579_read_reg(struct sns_port *port, uint8_t cmd, uint8_t *pdata)
{
    return sns_port_read(port, cmd, pdata, 1);
}

static inline int ltr579_write_reg(struct sns_port *port, uint8_t cmd, uint8_t val)
{
    return sns_port_write(port, cmd, &val, 1);
}

static int update_device(struct ltr579_rtdata *rtdata)
{
    int ret = 0;
    uint8_t cmd_data;
    uint8_t main_ctrl_data;

    ret = ltr579_read_reg(&rtdata->port, MAIN_CTRL, &main_ctrl_data);
    if (ret < 0)
        return ret;

    main_ctrl_data &= ~(PRX_ACTIVE | ALS_ACTIVE);

    cmd_data = PRX_LED_DEFAULT;
    ret = ltr579_write_reg(&rtdata->port, PS_LED, cmd_data);
    if (ret < 0)
        return ret;

    cmd_data = PRX_PULSE_DEFAULT;
    ret = ltr579_write_reg(&rtdata->port, PS_PULSES, cmd_data);
    if (ret < 0)
        return ret;

    cmd_data = PRX_MEAS_DEFAULT;
    ret = ltr579_write_reg(&rtdata->port, PS_MEAS_RATE, cmd_data);
    if (ret < 0)
        return ret;

    if (rtdata->prox_enabled)
        main_ctrl_data |= PRX_ACTIVE;

    /* if the resolution changed then disable the als sensor first,
     * this can help us to save a conversion time to get the new data
     * with the new resolution */
    if (rtdata->als_res_changed) {
        rtdata->als_res_changed = false;
        ret = ltr579_write_reg(&rtdata->port, MAIN_CTRL, main_ctrl_data);
        if (ret < 0)
            return ret;
    }

    cmd_data = ltr579_als_res[rtdata->als_res_idx].als_measure_rate;
    ret = ltr579_write_reg(&rtdata->port, ALS_MEAS_RATE, cmd_data);
    if (ret < 0)
        return ret;

    cmd_data = ltr579_als_res[rtdata->als_res_idx].als_control;
    ret = ltr579_write_reg(&rtdata->port, ALS_GAIN, cmd_data);
    if (ret < 0)
        return ret;

    if (rtdata->als_enabled)
        main_ctrl_data |= ALS_ACTIVE;

    ret = ltr579_write_reg(&rtdata->port, MAIN_CTRL, main_ctrl_data);
    if (ret < 0)
        return ret;

    return 0;
}

static int ltr579_get_type(const struct sensor_dev *dev)
{
    return 0;
}

static int ltr579_activate(const struct sensor_dev *dev, bool enable)
{
    struct ltr579_rtdata *rtdata = dev->rtdata;
    int ret;

    printf("activate %s %s\n", dev->name, enable ? "true" : "false");

    osMutexAcquire(rtdata->mutex, osWaitForever);

    if ((dev->type == SENSOR_TYPE_PROXIMITY) && (rtdata->prox_enabled != enable)) {
        if (rtdata->als_enabled && (rtdata->als_raw == -1)) {
            /* init the thread flags */
            /* if the first conversion of the ALS is not complete, the proximity sensor
             * should not be allowed to access the main_controll register, thus we
             * can send the first ALS event as soon as possible to the upper layer,
             * especially for the automatic brightness contoller */
            osThreadFlagsClear(FLAG_FIRST_ALS_COMPLETE);
            rtdata->prox_thread = osThreadGetId();

            osMutexRelease(rtdata->mutex);
            osThreadFlagsWait(FLAG_FIRST_ALS_COMPLETE, osFlagsWaitAny, osMsecToTick(ALS_DELAY));

            osMutexAcquire(rtdata->mutex, osWaitForever);
            rtdata->prox_thread = NULL;
        }

        if (enable) {
            rtdata->prox_near = false;
            rtdata->prox_first_data = true;
        }
        rtdata->prox_enabled = enable;
    } else if ((dev->type == SENSOR_TYPE_LIGHT) && (rtdata->als_enabled != enable)) {
        if (enable) {
            rtdata->als_raw = -1;
            rtdata->als_res_idx = ALS_NORMAL_RES;
            rtdata->als_res_changed = false;
        }
        rtdata->als_enabled = enable;
    } else {
        printf("unhandled case for active ltr579\n");
        ret = -EINVAL;
        goto exit_err_state;
    }

    ret = update_device(rtdata);
    if (ret) {
        printf("%s %s failed\n", dev->name, enable ? "enable" : "disable");
        if (enable) {
            if (dev->type == SENSOR_TYPE_PROXIMITY)
                rtdata->prox_enabled = false;
            else
                rtdata->als_enabled = false;

            goto exit_err_state;
        }
    }

    if (dev->type == SENSOR_TYPE_PROXIMITY) {
        if (enable)
            osTimerStart(rtdata->prox_timer, osMsecToTick(PROX_DELAY));
        else
            osTimerStop(rtdata->prox_timer);
    } else if (dev->type == SENSOR_TYPE_LIGHT) {
        if (enable)
            osTimerStart(rtdata->als_timer, osMsecToTick(ALS_DELAY));
        else
            osTimerStop(rtdata->als_timer);
    }

exit_err_state:
    osMutexRelease(rtdata->mutex);

    return ret;
}

/* set delay for ltr579 is a null operation */
static int ltr579_set_delay(const struct sensor_dev *dev, uint32_t us)
{
    return 0;
}

static int ltr579_selftest(const struct sensor_dev *dev, void *data)
{
    return 0;
}

static int ltr579_set_offset(const struct sensor_dev *dev, void *offset)
{
    struct ltr579_rtdata *rtdata = dev->rtdata;
    int *prx_offset = offset;
    int ret = -EINVAL;

    if (dev->type == SENSOR_TYPE_PROXIMITY) {
        rtdata->prox_offset = *prx_offset;
        ret = 0;
    }

    return ret;
}

static int ltr579_get_offset(const struct sensor_dev *dev, void *offset)
{
    struct ltr579_rtdata *rtdata = dev->rtdata;
    int *prx_offset = offset;
    int ret = -EINVAL;

    if (dev->type == SENSOR_TYPE_PROXIMITY) {
        *prx_offset = rtdata->prox_offset;
        ret = 0;
    }

    return ret;
}

static char* ltr579_get_vendor(const struct sensor_dev *dev)
{
    return "liteon";
}

static char* ltr579_get_module(const struct sensor_dev *dev)
{

    return "ltr579";
}

static float ltr579_get_max_range(const struct sensor_dev *dev)
{
    if (dev->type == SENSOR_TYPE_PROXIMITY)
        return 5.0;
    else
        return 200000;
}

static float ltr579_get_power(const struct sensor_dev *dev)
{
    return 0.05;
}

static int ltr579_get_min_delay(const struct sensor_dev *dev)
{
    if (dev->type == SENSOR_TYPE_LIGHT)
        return 50000;
    else
        return 0;
}

static int ltr579_get_max_delay(const struct sensor_dev *dev)
{
    if (dev->type == SENSOR_TYPE_LIGHT)
        return 200000;
    else
        return 0;
}

static float ltr579_calc_resolution(const struct sensor_dev *dev)
{
    const struct ltr579_platform_data *spdata = dev->pdata->spdata;
    int factor;
    float scale;

    if (!spdata->als_auto_high_res)
        factor = ALS_NORMAL_RES_FACTOR;
    else
        factor = ALS_HIGH_RES_FACTOR;

    scale = (float)spdata->als_trans_ratio * 10 / factor;

    return scale;
}

static float ltr579_get_resolution(const struct sensor_dev *dev)
{
    struct ltr579_rtdata *rtdata = dev->rtdata;

    if (dev->type == SENSOR_TYPE_LIGHT)
        return rtdata->als_res;
    else
        return 5.0;
}

static int ltr579_check_id(struct sns_port *port)
{
    int ret;
    uint8_t chip_id = 0;

    ret = ltr579_read_reg(port, PART_ID, &chip_id);
    if (ret < 0 || chip_id != CHIP_ID) {
        printf("check chip id: ret=%d, chip_id=0x%02x\n", ret, chip_id);
        return -ENXIO;
    }

    return 0;
}

static struct sensor_ops ltr579_ops = {
    .activate = ltr579_activate,
    .set_delay = ltr579_set_delay,
    .selftest = ltr579_selftest,
    .set_offset = ltr579_set_offset,
    .get_offset = ltr579_get_offset,
    .get_vendor = ltr579_get_vendor,
    .get_module = ltr579_get_module,
    .get_type = ltr579_get_type,
    .get_max_range = ltr579_get_max_range,
    .get_power = ltr579_get_power,
    .get_min_delay = ltr579_get_min_delay,
    .get_max_delay = ltr579_get_max_delay,
    .get_resolution = ltr579_get_resolution,
};

static bool ltr579_test_main_status(struct ltr579_rtdata *rtdata, uint8_t bit)
{
    struct sns_port *port = &rtdata->port;
    uint8_t status;
    int ret;

    ret = ltr579_read_reg(port, MAIN_STATUS, &status);
    if (ret)
        return false;

    rtdata->main_status |= status;

    return !!(rtdata->main_status & bit);
}

static inline void ltr579_clear_main_status(struct ltr579_rtdata *rtdata, uint8_t bit)
{
    rtdata->main_status &= ~bit;
}

static int ltr579_prox_read(struct sensor_dev *dev)
{
    struct ltr579_rtdata *rtdata = dev->rtdata;
    struct sns_port *port = &rtdata->port;
    int prxdata;
    uint8_t prx_regs[2];
    int ret;

    if (!ltr579_test_main_status(rtdata, PRX_NEW_DATA_READY))
        return -EBUSY;

    ltr579_clear_main_status(rtdata, PRX_NEW_DATA_READY);
    ret = ltr579_read_data(port, PS_DATA_0, prx_regs, 2);
    if (ret)
        return -EIO;
    if (prx_regs[1] & PRX_OVERFLOW) {
        switch (PRX_WIDTH_DEFAULT) {
        case PRX_DATA_WIDTH_8:
            return 255;
        case PRX_DATA_WIDTH_9:
            return 511;
        case PRX_DATA_WIDTH_10:
            return 1023;
        case PRX_DATA_WIDTH_11:
        default:
            return 2047;
        }
    }

    prxdata = ((prx_regs[1] & 0x07) << 8) + prx_regs[0];

    return prxdata;
}

static bool ltr579_is_near_far_changed(struct sensor_dev *dev, uint16_t raw)
{
    struct ltr579_rtdata *rtdata = dev->rtdata;
    const struct ltr579_platform_data *spdata = dev->pdata->spdata;
    int16_t adjust_raw = raw - rtdata->prox_offset;
    bool changed = false;

    raw = adjust_raw > 0 ? adjust_raw : 0;

    if (rtdata->prox_near) {
        if (raw < spdata->prox_thres_far) {
            rtdata->prox_near = false;
            changed = true;
        }
    } else {
        if (raw > spdata->prox_thres_near) {
            rtdata->prox_near = true;
            changed = true;
        }
    }

    return changed;
}

static void ltr579_prox_timer_cb(void *arg)
{
    struct sensor_dev *dev = arg;
    struct ltr579_rtdata *rtdata = dev->rtdata;
    struct sensor_event prox_event;
    uint16_t raw;
    bool changed;
    int ret = 0;

    prox_event.timestamp = get_timestamp();

    osMutexAcquire(rtdata->mutex, osWaitForever);

    if (!rtdata->prox_enabled)
        goto prx_state_err;

    raw = ret = ltr579_prox_read(dev);
    if (ret < 0)
        goto prx_read_err;

    changed = ltr579_is_near_far_changed(dev, raw);
    rtdata->prox_raw = raw;
    if (changed || rtdata->prox_continus || rtdata->prox_first_data) {
        printf("prox: distance=%d, raw=%d\n", rtdata->prox_near ? 0 : 5, raw);
        rtdata->prox_first_data = false;

        prox_event.distance_raw.raw0 = rtdata->prox_near ? 0 : 5;
        prox_event.distance_raw.raw1 = raw;
        prox_event.distance.value = rtdata->prox_near ? 0 : 5.0f;
        prox_event.type = SENSOR_TYPE_PROXIMITY;
        smgr_push_data(&prox_event, 1);
    }

prx_read_err:
prx_state_err:

    osMutexRelease(rtdata->mutex);
}

static int ltr579_als_read(struct sensor_dev *dev)
{
    struct ltr579_rtdata *rtdata = dev->rtdata;
    struct sns_port *port = &rtdata->port;
    int luxdata;
    uint8_t lux_regs[3];
    int ret;

    if (!ltr579_test_main_status(rtdata, ALS_NEW_DATA_READY))
        return -EBUSY;

    ltr579_clear_main_status(rtdata, ALS_NEW_DATA_READY);
    ret = ltr579_read_data(port, ALS_DATA_0, lux_regs, 3);
    if (ret)
        return -EIO;

    luxdata = (lux_regs[2] << 16) + (lux_regs[1] << 8) + lux_regs[0];
    return luxdata;
}

static bool ltr579_als_is_hysteresis_changed(struct sensor_dev *dev, int raw)
{
    struct ltr579_rtdata *rtdata = dev->rtdata;
    const struct ltr579_platform_data *spdata = dev->pdata->spdata;
    int high, low;

    if (rtdata->als_raw < 0)
        return true;

    raw *= 100;
    high = (100 + spdata->als_hysteresis_ratio) * rtdata->als_raw;
    low = (100 - spdata->als_hysteresis_ratio) * rtdata->als_raw;

    if (raw < low || raw > high)
        return true;
    else
        return false;
}

static int ltr579_als_adjust_resolution(struct sensor_dev *dev)
{
    struct ltr579_rtdata *rtdata = dev->rtdata;
    const struct ltr579_platform_data *spdata = dev->pdata->spdata;

    if (rtdata->als_res_idx > ALS_HIGH_RES && rtdata->als_raw < ALS_CHANGE_RES_LOW) {
        if (!spdata->als_auto_high_res && rtdata->als_res_idx == ALS_NORMAL_RES)
            return 0;

        rtdata->als_res_idx--;
        rtdata->als_res_changed = true;
    } else if (rtdata->als_res_idx < ALS_LOW_RES && rtdata->als_raw > ALS_CHANGE_RES_HIGH) {
        rtdata->als_res_idx++;
        rtdata->als_res_changed = true;
    }

    if (rtdata->als_res_changed) {
        rtdata->als_raw = -1;
        update_device(rtdata);
    }

    return 0;
}

static void ltr579_als_timer_cb(void *arg)
{
    struct sensor_dev *dev = arg;
    struct ltr579_rtdata *rtdata = dev->rtdata;
    struct sensor_event als_event;
    int ret = 0;
    int raw;
    bool changed;

    als_event.timestamp = get_timestamp();

    osMutexAcquire(rtdata->mutex, osWaitForever);

    if (!rtdata->als_enabled)
        goto als_state_err;

    ret = ltr579_als_read(dev);
    if (ret < 0)
        goto als_read_err;

    if (rtdata->als_raw == -1)
        osThreadFlagsSet(rtdata->prox_thread, FLAG_FIRST_ALS_COMPLETE);

    raw = ret * ALS_NORMAL_RES_FACTOR / ltr579_als_res[rtdata->als_res_idx].als_res_factor;
    changed = ltr579_als_is_hysteresis_changed(dev, raw);
    if (changed || rtdata->als_continus) {
        printf("als: lux count=%d\n", ret);
        rtdata->als_raw = raw;

        als_event.light.value = raw * rtdata->als_res;
        als_event.light_raw.raw0 = raw;
        als_event.type = SENSOR_TYPE_LIGHT;
        smgr_push_data(&als_event, 1);

        ltr579_als_adjust_resolution(dev);
    }

als_read_err:
als_state_err:
    osMutexRelease(rtdata->mutex);
}

static int ltr579_probe(const struct sensor_platform_data *pdata,
            const struct sensor_matching_data *mdata,
            struct sensor_dev **dev, uint32_t *num_sensors)
{
    struct sensor_dev *sdevs;
    struct ltr579_rtdata *rtdata;
    const struct ltr579_platform_data *spdata = pdata->spdata;
    size_t msize;
    int ret;

    msize = sizeof(*sdevs) * 2 + sizeof(struct ltr579_rtdata);
    sdevs = calloc(1, msize);
    if (!sdevs) {
        printf("ltr579 failed to malloc\n");
        return -ENOMEM;
    }

    rtdata = (struct ltr579_rtdata *)&sdevs[2];
    ret = sns_port_init(&pdata->bus_info, &rtdata->port);
    if (ret) {
        printf("failed to init port for %s\n", pdata->name);
        goto port_err;
    }

    ret = ltr579_check_id(&rtdata->port);
    if (ret)
        goto chip_id_err;

    rtdata->mutex = osMutexNew(NULL);
    if (!rtdata->mutex) {
        printf("create mutex failed for ltr579\n");
        ret = osError;
        goto mutex_err;
    }

    rtdata->prox_timer = osTimerNew(ltr579_prox_timer_cb, osTimerPeriodic, &sdevs[0], NULL);
    if (!rtdata->prox_timer) {
        printf("failed to create timer for prox sensor\n");
        ret = -ETIME;
        goto prx_timer_err;
    }

    rtdata->als_timer = osTimerNew(ltr579_als_timer_cb, osTimerPeriodic, &sdevs[1], NULL);
    if (!rtdata->als_timer) {
        printf("failed to create timer for als sensor\n");
        ret = -ETIME;
        goto als_timer_err;
    }

    init_sensor_dev(&sdevs[0], PROXIMITY_SENSOR, SENSOR_TYPE_PROXIMITY,
            &ltr579_ops, pdata, rtdata, mdata);
    init_sensor_dev(&sdevs[1], LIGHT_SENSOR, SENSOR_TYPE_LIGHT,
            &ltr579_ops, pdata, rtdata, mdata);

    rtdata->prox_offset = spdata->prox_default_offset;
    rtdata->als_res = ltr579_calc_resolution(&sdevs[1]);

    *dev = sdevs;
    *num_sensors = 2;

    printf("2 sensors probed for ltr579\n");

    return 0;

als_timer_err:
    osTimerDelete(rtdata->prox_timer);
prx_timer_err:
    osSemaphoreDelete(rtdata->mutex);
    rtdata->mutex = NULL;
mutex_err:
chip_id_err:
    sns_port_deinit(&rtdata->port);
port_err:
    free(sdevs);

    return ret;
}

static const struct sensor_matching_data ltr579_mdata = {
    .name = "liteon,ltr579",
    .priv = NULL,
};

static const struct sensor_matching_data *drv_mdata[] = {
    &ltr579_mdata,
    NULL,
};

static const struct sensor_driver __define_drv__ ltr579_drv = {
    .name = "ltr579",
    .mdata = &drv_mdata[0],
    .probe = ltr579_probe,
};
