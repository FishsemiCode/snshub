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

#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cmsis_os2.h>
#include "sensor.h"
#include "sensor_manager.h"
#include "sensor_driver.h"
#include "sensor_port.h"
#include "icm20602.h"
#include "utils.h"

#define ACCEL_SENSOR            "accelerometer"
#define GYRO_SENSOR             "gyroscope"

#define ICM20602_DEVICE_ID       0x12

#define REG_SELF_TEST_X_GYRO        0x00
#define REG_SELF_TEST_Y_GYRO        0x01
#define REG_SELF_TEST_Z_GYRO        0x02
#define REG_SELF_TEST_X_ACC         0x0D
#define REG_SELF_TEST_Y_ACC         0x0E
#define REG_SELF_TEST_Z_ACC         0x0F

#define REG_XG_OFFS_USRH            0x13
#define REG_XG_OFFS_USRL            0x14
#define REG_YG_OFFS_USRH            0x15
#define REG_YG_OFFS_USRL            0x16
#define REG_ZG_OFFS_USRH            0x17
#define REG_ZG_OFFS_USRL            0x18

#define REG_SMPLRT_DIV              0x19 // 25
#define REG_CONFIGURATION           0x1A // 26
#define REG_GYRO_CONFIGURATION      0x1B // 27
#define REG_ACC_CONFIGURATION       0x1C // 28
#define REG_ACC_CONFIGURATION_2     0x1D // 29
#define REG_LP_ODR_CTRL             0x1E // 30
#define REG_WOM_THR_X               0x20 // 32
#define REG_WOM_THR_Y               0x21 // 33
#define REG_WOM_THR_Z               0x22 // 34
#define REG_FIFO_ENABLE             0x23 // 35
#define REG_ODR_DELAY_EN            0x2E //
#define REG_FSYNC_INT               0x2F //
#define REG_INT_PIN_CFG             0x37 //
#define REG_INTERRUPT_ENABLE        0x38 // 56
#define REG_FIFO_WM_INT_STATUS      0x39 // 57
#define REG_INT_STATUS              0x3A // 58
#define REG_ACC_DATA_XOUTH          0x3B // 59 /* big endian */
#define REG_TEMP_DATA_OUTH          0x41 // 65 /* big endian */
#define REG_GYRO_DATA_XOUTH         0x43 // 67 /* big endian */
#define REG_ODR_DLY_CNT             0x60 // 95 /* 16bits, big endian */
#define REG_FIFO_WM_TH              0x61 // 97 /* 8bits */
#define REG_SIGNAL_PATH_RESET       0x68 // 104
#define REG_ACC_INTEL_CTRL          0x69 // 105
#define REG_USER_CONTROL            0x6A // 106
#define REG_PWR_MGMT_1              0x6B // 107
#define REG_PWR_MGMT_2              0x6C // 108
#define REG_OIS_ENABLE              0x70 // 112
#define REG_FIFO_COUNT_H            0x72 // 114 /* 16bits, big endian */
#define REG_FIFO_R_W                0x74 // 116
#define REG_WHO_AM_I                0x75 // 117
#define REG_BANK_SEL                0x76 // 118
#define REG_INTOSC                  0x1A // 26 in cfg bank

/* field for REG_INT_PIN_CFG */
#define INT_PIN_CONFIG_MASK         (0x0f << 4)
/* high level trigger, push-pull, read status reg to clear */
#define INT_PIN_CONFIG              (2 << 4)

/* field for REG_INTERRUPT_ENABLE */
#define DATA_RDY_INT                (1 << 0)

/* field for REG_FIFO_WM_INT_STATUS */
#define ST_FIFO_WM_INT              (1 << 6)

/* field for REG_INT_STATUS */
#define ST_DATA_RDY_INT             (1 << 0)

/* field for REG_CONFIGURATION */
#define OVERWRITE_MODE_MASK         (1 << 6)
#define GDLPF_CFG_MASK              (7 << 0)
#define GDLPF_CFG_5HZ               (6 << 0)
#define GDLPF_CFG_10HZ              (5 << 0)
#define GDLPF_CFG_20HZ              (4 << 0)
#define GDLPF_CFG_41HZ              (3 << 0)
#define GDLPF_CFG_92HZ              (2 << 0)

/* field for REG_GYRO_CONFIGURATION */
#define GFCHOICE_B_MASK             (0x03 << 0)
#define GFCHOICE_DLPF               0x00
#define GFS_MASK                    (0x03 << 3)
#define GFS_250_DPS                 (0 << 3)
#define GFS_500_DPS                 (1 << 3)
#define GFS_1000_DPS                (2 << 3)
#define GFS_2000_DPS                (3 << 3)

/* field for REG_ACC_CONFIGURATION */
#define AFS_OIS_2G                  (0x00)
#define AFS_OIS_4G                  (1 << 0)
#define AFS_OIS_8G                  (2 << 0)
#define AFS_OIS_1G                  (3 << 0)
#define AFS_MASK                    (0x03 << 3)
#define AFS_2G                      (0 << 3)
#define AFS_4G                      (1 << 3)
#define AFS_8G                      (2 << 3)
#define AFS_16G                     (3 << 3)

/* field for REG_ACC_CONFIGURATION_2 */
#define ADLPF_CFG_MASK              (0x07 << 0)
#define ADLPF_CFG_5HZ               (6 << 0)
#define ADLPF_CFG_10HZ              (5 << 0)
#define ADLPF_CFG_20HZ              (4 << 0)
#define ADLPF_CFG_45HZ              (3 << 0)
#define ADLPF_CFG_100HZ             (2 << 0)
#define AFCHOICE_MASK               (1 << 3)
#define AFCHOICE_DLPF               (0 << 3)

/* field for REG_PWR_MGMT_1 */
#define DEVICE_RESET                (1 << 7)
#define DEVICE_SLEEP                (1 << 6)
#define DEVICE_CYCLE                (1 << 5)
#define CLKSEL_MASK                 (0x07 << 0)
#define CLKSEL_INTERNAL             0
#define CLKSEL_AUTO_BEST            1

/* field for REG_PWR_MGMT_2 */
#define STBY_ACC                    (7 << 3)
#define STBY_GYR                    (7 << 0)

#define DEFAULT_ODR                 5
#define DEFAULT_ACC_RANGE           4
#define DEFAULT_ACC_BW              20
#define DEFAULT_GYRO_RANGE          2000
#define DEFAULT_GYRO_BW             20

#define TEMP_SENSITIVITY            326.8f
#define ROOMTEMP_OFFSET             25.0f

struct icm20602_rtdata {
    struct sns_port port;
    osMutexId_t mutex;
    const struct sensor_platform_data *pdata;

    osTimerId_t timer;

    bool acc_enabled;
    int acc_range_idx;
    int acc_bw_idx;
    int acc_expect_odr;

    bool gyro_enabled;
    int gyro_range_idx;
    int gyro_bw_idx;
    int gyro_expect_odr;

    int actual_odr_idx;
};

struct icm_param_range {
    int range;
    int reg_value;
    float resolution;
};

struct icm_param_odr {
    int odr;
    int reg_value;
};

struct icm_param_bw {
    int bw;
    int reg_value;
};

const static struct icm_param_range icm_acc_ranges[] = {
    {2, AFS_2G, 0.00059855},
    {4, AFS_4G, 0.0011971},
    {8, AFS_8G, 0.0023942},
    {16, AFS_16G, 0.0047884},
};

const static struct icm_param_range icm_gyro_ranges[] = {
    {250, GFS_250_DPS, 0.00762939 * DEGREE2RAD},
    {500, GFS_500_DPS, 0.01525879 * DEGREE2RAD},
    {1000, GFS_1000_DPS, 0.03051758 * DEGREE2RAD},
    {2000, GFS_2000_DPS, 0.06103516 * DEGREE2RAD},
};

const static struct icm_param_odr icm_acc_gyro_odrs[] = {
    {5, 199}, /* NORMAL */
    {20, 49}, /* UI */
    {50, 19}, /* GAME */
    {100, 9}, /* FASTEST */
    {200, 4}, /* MORE FASTER */
    {1000, 0}, /* SUPPER */
};

const static struct icm_param_bw icm_acc_bws[] = {
    {5, ADLPF_CFG_5HZ},
    {10, ADLPF_CFG_10HZ},
    {20, ADLPF_CFG_20HZ},
    {45, ADLPF_CFG_45HZ},
    {100, ADLPF_CFG_100HZ},
};

const static struct icm_param_bw icm_gyro_bws[] = {
    {5, GDLPF_CFG_5HZ},
    {10, GDLPF_CFG_10HZ},
    {20, GDLPF_CFG_20HZ},
    {40, GDLPF_CFG_41HZ},
    {90, GDLPF_CFG_92HZ},
};

static inline int icm_regs_read(struct sns_port *port, uint8_t reg, void *data, int len)
{
    uint8_t modify;

    modify = port->binfo->bus_type == BUS_SPI ? 0x80 : 0x00;
    return sns_port_read(port, reg | modify, data, len);
}

static inline int icm_reg_write(struct sns_port *port, uint8_t reg, uint8_t reg_data)
{
    return sns_port_write(port, reg, &reg_data, 1);
}

static int icm_reg_update(struct sns_port *port, uint8_t reg, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t ori;

    ret = icm_regs_read(port, reg, &ori, 1);
    if (ret)
        return ret;

    ori &= ~mask;
    ori |= value & mask;

    return sns_port_write(port, reg, &ori, 1);
}

static int icm_find_acc_range(int expect)
{
    int i;

    for (i = 0; i < __countof(icm_acc_ranges); i++) {
        if (expect <= icm_acc_ranges[i].range)
            return i;
    }

    return i - 1;
}

static int icm_find_gyro_range(int expect)
{
    int i;

    for (i = 0; i < __countof(icm_gyro_ranges); i++) {
        if (expect <= icm_gyro_ranges[i].range)
            return i;
    }

    return i - 1;
}

static int icm_find_odr(int expect)
{
    int i;

    for (i = 0; i < __countof(icm_acc_gyro_odrs); i++) {
        if (expect <= icm_acc_gyro_odrs[i].odr)
            return i;
    }

    return i - 1;
}

static int icm_find_acc_bw(int expect)
{
    int i;

    for (i = 0; i < __countof(icm_acc_bws); i++) {
        if (expect <= icm_acc_bws[i].bw)
            return i;
    }

    return i - 1;
}

static int icm_find_gyro_bw(int expect)
{
    int i;

    for (i = 0; i < __countof(icm_gyro_bws); i++) {
        if (expect <= icm_gyro_bws[i].bw)
            return i;
    }

    return i - 1;
}

static void icm_sync_odr(struct icm20602_rtdata *rtdata)
{
    struct sns_port *port = &rtdata->port;
    int actual_odr = MAX(rtdata->acc_expect_odr, rtdata->gyro_expect_odr);
    int idx = icm_find_odr(actual_odr);

    if (rtdata->actual_odr_idx != idx) {
        rtdata->actual_odr_idx = idx;
        icm_reg_write(port, REG_SMPLRT_DIV, icm_acc_gyro_odrs[idx].reg_value);

        if (rtdata->timer)
            osTimerStart(rtdata->timer,
                    osUsecToTick(1000000 / icm_acc_gyro_odrs[rtdata->actual_odr_idx].odr));
    }
}

static int icm_acc_enable(struct icm20602_rtdata *rtdata, bool enable)
{
    if (enable) {
        icm_reg_update(&rtdata->port, REG_PWR_MGMT_2, STBY_ACC, 0);
    } else {
        icm_reg_update(&rtdata->port, REG_PWR_MGMT_2, STBY_ACC, STBY_ACC);
        rtdata->acc_expect_odr = INT_MIN;
    }

    return 0;
}

static int icm_gyro_enable(struct icm20602_rtdata *rtdata, bool enable)
{
    if (enable) {
        icm_reg_update(&rtdata->port, REG_PWR_MGMT_2, STBY_GYR, 0);
    } else {
        icm_reg_update(&rtdata->port, REG_PWR_MGMT_2, STBY_GYR, STBY_GYR);
        rtdata->gyro_expect_odr = INT_MIN;
    }

    return 0;
}

static int icm20602_activate(const struct sensor_dev *dev, bool enable)
{
    struct icm20602_rtdata *rtdata = dev->rtdata;
    bool old_status, new_status;

    /* XXX: just non-fifo mode for now, later will create a fifo procedure */
    osMutexAcquire(rtdata->mutex, osWaitForever);

    printf("icm: activate %s to %s\n", dev->name, enable ? "enable" : "disable");
    old_status = rtdata->acc_enabled || rtdata->gyro_enabled;
    if (dev->type == SENSOR_TYPE_ACCELEROMETER && rtdata->acc_enabled != enable) {
        icm_acc_enable(rtdata, enable);
        rtdata->acc_enabled = enable;
    } else if (dev->type == SENSOR_TYPE_GYROSCOPE && rtdata->gyro_enabled != enable) {
        icm_gyro_enable(rtdata, enable);
        rtdata->gyro_enabled = enable;
    }

    if (rtdata->timer) {
        new_status = rtdata->acc_enabled || rtdata->gyro_enabled;
        if (old_status != new_status) {
            if (new_status)
                osTimerStart(rtdata->timer,
                        osUsecToTick(1000000 / icm_acc_gyro_odrs[rtdata->actual_odr_idx].odr));
            else
                osTimerStop(rtdata->timer);
        }
    }

    osMutexRelease(rtdata->mutex);

    return 0;
}

static int icm20602_set_delay(const struct sensor_dev *dev, uint32_t us)
{
    struct icm20602_rtdata *rtdata = dev->rtdata;
    int odr = 1000000 / us;

    osMutexAcquire(rtdata->mutex, osWaitForever);

    if (dev->type == SENSOR_TYPE_ACCELEROMETER) {
        rtdata->acc_expect_odr = odr;
    } else {
        rtdata->gyro_expect_odr = odr;
    }

    icm_sync_odr(rtdata);
    osMutexRelease(rtdata->mutex);

    return 0;
}

static int icm20602_selftest(const struct sensor_dev *dev, void *data)
{
    return 0;
}

static int icm20602_set_offset(const struct sensor_dev *dev, void *offset)
{
    return 0;
}

static int icm20602_get_offset(const struct sensor_dev *dev, void *offset)
{
    return 0;
}

static char* icm20602_get_vendor(const struct sensor_dev *dev)
{
    return "invensense";
}

static char* icm20602_get_module(const struct sensor_dev *dev)
{
    return "icm20602";
}

static float icm20602_get_max_range(const struct sensor_dev *dev)
{
    struct icm20602_rtdata *rtdata = dev->rtdata;

    if (dev->type == SENSOR_TYPE_ACCELEROMETER)
        return icm_acc_ranges[rtdata->acc_range_idx].range * GRAVITY;
    else
        return icm_gyro_ranges[rtdata->gyro_range_idx].range;
}

static float icm20602_get_power(const struct sensor_dev *dev)
{
    if (dev->type == SENSOR_TYPE_ACCELEROMETER)
        return 0.05;
    else
        return 1.0;
}

static int icm20602_get_min_delay(const struct sensor_dev *dev)
{
    int idx, odr;

    idx = icm_find_odr(INT_MAX);
    odr = icm_acc_gyro_odrs[idx].odr;

    return 1000000 / odr;
}

static int icm20602_get_max_delay(const struct sensor_dev *dev)
{
    int idx, odr;

    idx = icm_find_odr(0);
    odr = icm_acc_gyro_odrs[idx].odr;

    return 1000000 / odr;
}

static float icm20602_get_resolution(const struct sensor_dev *dev)
{
    struct icm20602_rtdata *rtdata = dev->rtdata;

    if (dev->type == SENSOR_TYPE_ACCELEROMETER)
        return icm_acc_ranges[rtdata->acc_range_idx].resolution;
    else
        return icm_gyro_ranges[rtdata->gyro_range_idx].resolution;
}

static struct sensor_ops icm_ops = {
    .activate = icm20602_activate,
    .set_delay = icm20602_set_delay,
    .selftest = icm20602_selftest,
    .set_offset = icm20602_set_offset,
    .get_offset = icm20602_get_offset,
    .get_vendor = icm20602_get_vendor,
    .get_module = icm20602_get_module,
    .get_max_range = icm20602_get_max_range,
    .get_power = icm20602_get_power,
    .get_min_delay = icm20602_get_min_delay,
    .get_max_delay = icm20602_get_max_delay,
    .get_resolution = icm20602_get_resolution,
};

static void icm_convert_reg_to_event(int8_t *reg_data, struct sensor_event *event, int place)
{
    int16_t raw_data[3];
    int i;

    for (i = 0; i < 3; i++)
        raw_data[i] = (reg_data[i << 1] << 8) | (uint8_t)reg_data[(i << 1) + 1];

    remap_vector_raw16to32_axis(raw_data, event->data_raw, place);
}

static void icm20602_worker(void *data, int64_t ts)
{
    struct icm20602_rtdata *rtdata = data;
    const struct icm20602_platform_data *spdata = rtdata->pdata->spdata;
    int64_t timestamp = ts;
    struct sensor_event event[2];
    int8_t reg_data[6];
    uint8_t num = 0, int_status = 0;
    uint16_t raw_temp = 0;
    int ret;

    ret = icm_regs_read(&rtdata->port, REG_INT_STATUS, &int_status, 1);
    if (spdata->irq_pin)
        sensor_enable_gpio_irq(spdata->irq_pin, spdata->trigger_type);
    if (ret < 0) {
        printf("icm20602 reg REG_INT_STATUS read faild, err:%d\n", ret);
        return;
    }

    ret = icm_regs_read(&rtdata->port, REG_TEMP_DATA_OUTH, reg_data, 2);
    if (!ret) {
        raw_temp = (reg_data[0]<<8) | reg_data[1];
    }

    if (rtdata->acc_enabled && (int_status & ST_DATA_RDY_INT)) {
        ret = icm_regs_read(&rtdata->port, REG_ACC_DATA_XOUTH, reg_data, 6);
        if (!ret) {
            icm_convert_reg_to_event(reg_data, &event[num], spdata->place);
            event[num].data[0] = event[num].data_raw[0] * icm_acc_ranges[rtdata->acc_range_idx].resolution;
            event[num].data[1] = event[num].data_raw[1] * icm_acc_ranges[rtdata->acc_range_idx].resolution;
            event[num].data[2] = event[num].data_raw[2] * icm_acc_ranges[rtdata->acc_range_idx].resolution;
            event[num].data[3] = raw_temp / TEMP_SENSITIVITY + ROOMTEMP_OFFSET;
            event[num].type = SENSOR_TYPE_ACCELEROMETER;
            event[num].timestamp = timestamp;
            num++;
        }
    }

    if (rtdata->gyro_enabled && (int_status & ST_DATA_RDY_INT)) {
        ret = icm_regs_read(&rtdata->port, REG_GYRO_DATA_XOUTH, reg_data, 6);
        if (!ret) {
            icm_convert_reg_to_event(reg_data, &event[num], spdata->place);
            event[num].data[0] = event[num].data_raw[0] * icm_gyro_ranges[rtdata->gyro_range_idx].resolution;
            event[num].data[1] = event[num].data_raw[1] * icm_gyro_ranges[rtdata->gyro_range_idx].resolution;
            event[num].data[2] = event[num].data_raw[2] * icm_gyro_ranges[rtdata->gyro_range_idx].resolution;
            event[num].data[3] = raw_temp / TEMP_SENSITIVITY + ROOMTEMP_OFFSET;
            event[num].type = SENSOR_TYPE_GYROSCOPE;
            event[num].timestamp = timestamp;
            num++;
        }
    }

    if (num > 0)
        smgr_push_data(event, num);
}

static void icm20602_irq_handler(uint32_t pin, void *data)
{
    struct icm20602_rtdata *rtdata = data;
    const struct icm20602_platform_data *spdata = rtdata->pdata->spdata;
    int64_t timestamp = get_timestamp();

    /* this is an IRQ context, we must use the pended worker
     * since the i2c access will be blocked */
    smgr_schedule_work(icm20602_worker, rtdata, timestamp, HIGH_WORK);
    /* XXX: because it's a level triggered interrupt, the handler would be called
     * frequetly until the pended handler is executed to read fifo data register.
     * so we need to temperarily disable the gpio interrupt and then re-enable it
     * in the pended handler */
    sensor_enable_gpio_irq(spdata->irq_pin, 0);
}

static void icm20602_timer_cb(void *arg)
{
    smgr_schedule_work(icm20602_worker, arg, get_timestamp(), HIGH_WORK);
}

static int icm20602_check_id(struct sns_port *port)
{
    int ret = 0;
    uint8_t device_id;

    ret = icm_regs_read(port, REG_WHO_AM_I, &device_id, 1);
    if (!ret && device_id != ICM20602_DEVICE_ID) {
        printf("icm20602: wrong deivce_id=0x%02x\n", device_id);
        ret = -EINVAL;
    }

    return ret;
}

int icm20602_reset(struct icm20602_rtdata *rtdata)
{
    struct sns_port *port = &rtdata->port;
    uint8_t reg;
    int ret = 0, retry = 0;

    icm_reg_update(port, REG_PWR_MGMT_1, DEVICE_RESET, DEVICE_RESET);

    for (retry = 0; retry < 10; retry++) {
        ret = icm_regs_read(port, REG_PWR_MGMT_1, &reg, 1);
        if(!ret && !(reg & DEVICE_RESET) && (reg & DEVICE_SLEEP))
            break;
        osDelay(osMsecToTick(200));
    }
    return ret;
}

void intr_init(struct icm20602_rtdata *rtdata)
{
    struct sns_port *port = &rtdata->port;

    /* 1.configure interrupt pin */
    icm_reg_write(port, REG_INT_PIN_CFG, 0x30);

    /* 2.configure interrupt */
    icm_reg_write(port, REG_INTERRUPT_ENABLE, 0x01);
    osDelay(osMsecToTick(1));
}

static int icm20602_init_chip(struct icm20602_rtdata *rtdata)
{
    int ret = -1;
    int idx;
    struct sns_port *port = &rtdata->port;
    const struct icm20602_platform_data *spdata = rtdata->pdata->spdata;

    /* 1.reset chip */
    ret = icm20602_reset(rtdata);
    if (ret) {
        printf("reset failed for icm20602\n");
        return ret;
    }

    /* 2.exit sleep mode */
    icm_reg_update(port, REG_PWR_MGMT_1, DEVICE_SLEEP|DEVICE_CYCLE, 0);

    /* 3.check product id */
    ret = icm20602_check_id(port);
    if (ret) {
        printf("check id failed for icm20602\n");
        return ret;
    }

    /* 4.set clock source */
    icm_reg_update(port, REG_PWR_MGMT_1, CLKSEL_MASK, 0);
    osDelay(osMsecToTick(5));

    /* 5.set the default range, bandwidth, odrs*/
    if (!spdata->acc_range)
        idx = icm_find_acc_range(spdata->acc_range);
    else
        idx = icm_find_acc_range(DEFAULT_ACC_RANGE);
    rtdata->acc_range_idx = idx;
    icm_reg_update(port, REG_ACC_CONFIGURATION, AFS_MASK, icm_acc_ranges[idx].reg_value);

    if (!spdata->acc_bw)
        idx = icm_find_acc_bw(spdata->acc_bw);
    else
        idx = icm_find_acc_bw(DEFAULT_ACC_BW);
    rtdata->acc_bw_idx = idx;
    icm_reg_update(port, REG_ACC_CONFIGURATION_2, AFCHOICE_MASK | ADLPF_CFG_MASK, AFCHOICE_DLPF | icm_acc_bws[idx].reg_value);

    if (!spdata->gyro_range)
        idx = icm_find_gyro_range(spdata->gyro_range);
    else
        idx = icm_find_gyro_range(DEFAULT_GYRO_RANGE);
    rtdata->gyro_range_idx = idx;
    icm_reg_update(port, REG_GYRO_CONFIGURATION, GFCHOICE_B_MASK | GFS_MASK, GFCHOICE_DLPF | icm_gyro_ranges[idx].reg_value);

    if (!spdata->gyro_bw)
        idx = icm_find_gyro_bw(spdata->gyro_bw);
    else
        idx = icm_find_gyro_bw(DEFAULT_GYRO_BW);
    rtdata->gyro_bw_idx = idx;
    icm_reg_update(port, REG_CONFIGURATION, GDLPF_CFG_MASK, icm_gyro_bws[idx].reg_value);

    /* 6.set default odr */
    idx = icm_find_odr(DEFAULT_ODR);
    rtdata->actual_odr_idx = idx;
    rtdata->acc_expect_odr = INT_MIN;
    rtdata->gyro_expect_odr = INT_MIN;
    icm_reg_write(port, REG_SMPLRT_DIV, icm_acc_gyro_odrs[idx].reg_value);

    /* 7.configure intr and intr pin */
    intr_init(rtdata);

    /* 8.disable ACC and GYRO by default */
    icm_reg_update(port, REG_PWR_MGMT_2, STBY_ACC | STBY_GYR, STBY_ACC | STBY_GYR);
    osDelay(osMsecToTick(200));

    return ret;
}

static int icm20602_probe(const struct sensor_platform_data *pdata,
            const struct sensor_matching_data *mdata,
            struct sensor_dev **dev, uint32_t *num_sensors)
{
    struct sensor_dev *sdevs;
    struct icm20602_rtdata *rtdata;
    const struct icm20602_platform_data *spdata = pdata->spdata;
    size_t msize;
    int ret;

    msize = sizeof(*sdevs) * 2 + sizeof(struct icm20602_rtdata);
    sdevs = calloc(1, msize);
    if (!sdevs) {
        printf("failed to malloc fot icm20602\n");
        return -ENOMEM;
    }

    rtdata = (struct icm20602_rtdata *)&sdevs[2];
    rtdata->pdata = pdata;

    ret = sns_port_init(&pdata->bus_info, &rtdata->port);
    if (ret) {
        printf("failed to init port for %s\n", pdata->name);
        goto port_err;
    }

    ret = icm20602_init_chip(rtdata);
    if (ret) {
        printf("failed to init for icm20602:%d\n", ret);
        goto init_err;
    }

    rtdata->mutex = osMutexNew(NULL);
    if (!rtdata->mutex) {
        printf("create mutex failed for icm20602\n");
        ret = osError;
        goto mutex_err;
    }

    if (spdata->irq_pin) {
        ret = sensor_register_gpio_irq(spdata->irq_pin, spdata->trigger_type,
                                icm20602_irq_handler, rtdata);
        if (ret) {
            printf("failed to register irq handler for icm20602:%d\n", ret);
            goto gpio_irq_err;
        }
    } else {
        rtdata->timer = osTimerNew(icm20602_timer_cb, osTimerPeriodic, rtdata, NULL);
        if (!rtdata->timer) {
            printf("failed to create timer for icm20602 sensor\n");
            ret = -ETIME;
            goto timer_err;
        }
    }

    init_sensor_dev(&sdevs[0], ACCEL_SENSOR, SENSOR_TYPE_ACCELEROMETER,
        &icm_ops, pdata, rtdata, mdata);
    init_sensor_dev(&sdevs[1], GYRO_SENSOR, SENSOR_TYPE_GYROSCOPE,
        &icm_ops, pdata, rtdata, mdata);

    *dev = sdevs;
    *num_sensors = 2;

    printf("2 sensors probed for icm20602\n");

    return ret;

timer_err:
gpio_irq_err:
    osMutexDelete(rtdata->mutex);
mutex_err:
init_err:
    sns_port_deinit(&rtdata->port);
port_err:
    free(sdevs);

    return ret;
}

const static struct sensor_matching_data icm20602_mdata = {
    .name = "invn,icm20602",
    .priv = NULL,
};

const static struct sensor_matching_data *drv_mdata[] = {
    &icm20602_mdata,
    NULL,
};

const static struct sensor_driver __define_drv__ icm20602_drv = {
    .name = "icm20602",
    .mdata = &drv_mdata[0],
    .probe = icm20602_probe,
};
