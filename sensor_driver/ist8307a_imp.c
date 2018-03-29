/* Copyright Statement: *
 * This software/firmware and related documentation ("Pinecone Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to Pinecone Inc. and/or its licensors. Without
 * the prior written permission of Pinecone inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of Pinecone Software, and
 * information contained herein, in whole or in part, shall be strictly * prohibited.
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
#include "utils.h"
#include "ist8307a_imp.h"

#define MAG_SENSOR                  "magnetometer"
#define MAG_Z_RESOLUTION            (0.3) /* range(2500) * 2 / (2 ^ 14) */
#define MAG_XY_RESOLUTION           (0.2) /* range(1600) * 2 / (2 ^ 14) */
#define MIN_ODR                     (1)
#define MAX_ODR                     (50)
#define DEFAULT_ODR                 (5)

#define IST8307A_REG_WIA            (0x00)  /* Who I am */
#define IST8307A_REG_INFO           (0x01)  /* More Info */
#define IST8307A_REG_DATAX          (0x03)  /* Output Value x */
#define IST8307A_REG_DATAY          (0x05)  /* Output Value y */
#define IST8307A_REG_DATAZ          (0x07)  /* Output Value z */
#define IST8307A_REG_DATA_TEMPL     (0x1C)
#define IST8307A_REG_DATA_TEMPH     (0x1D)
#define IST8307A_REG_STAT1          (0x02)  /* Status register 1 */
#define IST8307A_REG_STAT2          (0x09)  /* Status register 2 */
#define IST8307A_REG_CNTRL1         (0x0A)  /* Control setting register 1 */
#define IST8307A_REG_CNTRL2         (0x0B)  /* Control setting register 2 */
#define IST8307A_REG_CNTRL3         (0x0D)  /* Control setting register 3 */
#define IST8307A_REG_OFFSET_START   (0xDC)  /* Offset */
#define IST8307A_REG_SELECTION      (0x42)  /* Sensor Selection register */
#define IST8307A_REG_TEST           (0x40)  /* Chip Test register */
#define IST8307A_REG_TUNING         (0x47)  /* Bandgap Tuning register */
#define IST8307A_ODR_MODE           (0x01)  /* Force mode */
#define IST8307A_DRDY_MASK          (1 << 0)
#define IST8307A_SRST_MASK          (1 << 0)

struct ist8307a_rtdata {
    struct sns_port port;
    osMutexId_t mutex;
    osTimerId_t poll_timer;
    const struct sensor_platform_data *pdata;

    int actual_odr;
    bool reading;
};

static inline int ist_regs_read(struct sns_port *port, uint8_t reg, void *data, int len)
{
    uint8_t modify;

    modify = port->binfo->bus_type == BUS_SPI ? 0x80 : 0x00;
    return sns_port_read(port, reg | modify, data, len);
}

static inline int ist_reg_write(struct sns_port *port, uint8_t reg, uint8_t reg_data)
{
    return sns_port_write(port, reg, &reg_data, 1);
}

static inline int ist_reg_update(struct sns_port *port, uint8_t reg, uint8_t mask, uint8_t value)
{
    int ret = 0;
    uint8_t ori;

    ret = ist_regs_read(port, reg, &ori, 1);
    if (ret)
        return ret;

    ori &= ~mask;
    ori |= value & mask;

    return sns_port_write(port, reg, &ori, 1);
}

static void ist8307a_worker(void *data, int64_t ts)
{
    struct ist8307a_rtdata *rtdata = data;
    struct sns_port *port = &rtdata->port;
    const struct ist8307a_platform_data *spdata = rtdata->pdata->spdata;
    struct sensor_event event;
    int64_t timestamp = ts;
    uint8_t status_reg_1 = 0;
    int16_t reg_data[3];
    int ret;
    uint16_t raw_temp = 0;

    if (!rtdata->reading) {
        ist_reg_write(port, IST8307A_REG_CNTRL1, IST8307A_ODR_MODE);
        rtdata->reading = true;
    }

    /* read the status, this will clear status_reg_1 */
    ret = ist_regs_read(&rtdata->port, IST8307A_REG_STAT1, &status_reg_1, 1);
    if (ret)
        return;

    if (rtdata->reading == true) {
        ret = ist_regs_read(&rtdata->port, IST8307A_REG_DATA_TEMPL, &raw_temp, 1 * 2);
        if (ret)
            return;

        ret = ist_regs_read(&rtdata->port, IST8307A_REG_DATAX, reg_data, 3 * 2);
        if (!ret) {
            remap_vector_raw16to32_axis(reg_data, event.data_raw, spdata->place);
            event.data[0] = event.data_raw[0] * MAG_XY_RESOLUTION;
            event.data[1] = event.data_raw[1] * MAG_XY_RESOLUTION;
            event.data[2] = event.data_raw[2] * MAG_Z_RESOLUTION;
            event.data[3] = raw_temp;
            event.type = SENSOR_TYPE_MAGNETIC_FIELD;
            event.timestamp = timestamp;
            rtdata->reading = false;
            smgr_push_data(&event, 1);
        }
    }
}

static void ist8307a_timer_cb(void *arg)
{
    smgr_schedule_work(ist8307a_worker, arg, get_timestamp(), LOW_WORK);
}

static int ist8307a_init_chip(struct ist8307a_rtdata *rtdata)
{
    struct sns_port *port = &rtdata->port;
    uint8_t chip_id, reg;
    int ret = 0;

    /* soft reset chip */
    ret = ist_reg_write(port, IST8307A_REG_CNTRL2, 0x01);
    if (ret < 0) {
        printf("ist8307a soft reset faild\n");
        return -EINVAL;
    }
    /* this reg will be set to zero after POR routine */
    while(1) {
        ist_regs_read(port, IST8307A_REG_CNTRL2, &reg, 1);
        if (!(reg & IST8307A_SRST_MASK))
            break;
    }

    /* read chip id */
    ret = ist_regs_read(port, IST8307A_REG_WIA, &chip_id, 1);
    if ((ret < 0) || ((chip_id != 0xFF) && (chip_id != 0x07))) {
        printf("ist8307a the sensor ID error or read reg failed, %d, 0x%x\n", ret, chip_id);
        return -EINVAL;
    } else {
        printf("ist8307a the sensor ID is 0x%x\n", chip_id);
    }

    /* Switch to stand-by mode (0x00: stand-by mode, 0x01: Singel measure mode) */
    ret = ist_reg_write(port, IST8307A_REG_CNTRL1, 0x00);
    if (ret < 0) {
        printf("ist8307a write reg:0x%x failed\n", IST8307A_REG_CNTRL1);
        return -EINVAL;
    }

    /* Data ready enable control */
    ret = ist_reg_write(port, IST8307A_REG_CNTRL2, 0x08);
    if (ret < 0) {
        printf("ist8307a write reg:0x%x failed\n", IST8307A_REG_CNTRL2);
        return -EINVAL;
    }

    rtdata->reading = false;
    rtdata->actual_odr = DEFAULT_ODR;
    return 0;
}

static int ist8307a_activate(const struct sensor_dev *dev, bool enable)
{
    struct ist8307a_rtdata *rtdata = dev->rtdata;
    int ret = 0;

    osMutexAcquire(rtdata->mutex, osWaitForever);

    if (enable) {
        osTimerStart(rtdata->poll_timer, osUsecToTick(1000000 / rtdata->actual_odr));
    } else {
        osTimerStop(rtdata->poll_timer);
    }

    osMutexRelease(rtdata->mutex);

    return ret;
}

static int ist8307a_set_delay(const struct sensor_dev *dev, uint32_t us)
{
    struct ist8307a_rtdata *rtdata = dev->rtdata;
    int odr = 1000000 / us;

    osMutexAcquire(rtdata->mutex, osWaitForever);
    if (odr > MAX_ODR) {
        odr = MAX_ODR;
        printf("ist8307a set delay max is 50hz\n");
    }

    if (odr < MIN_ODR) { /* actual mix odr is 0.25hz */
        odr = MIN_ODR;
        printf("ist8307a set delay min is 1hz\n");
    }

    rtdata->actual_odr = odr;

    osTimerStart(rtdata->poll_timer, osUsecToTick(1000000 / rtdata->actual_odr));
    osMutexRelease(rtdata->mutex);

    return 0;
}

static int ist8307a_selftest(const struct sensor_dev *dev, void *data)
{
    return 0;
}

static int ist8307a_set_offset(const struct sensor_dev *dev, void *offset)
{
    return 0;
}

static int ist8307a_get_offset(const struct sensor_dev *dev, void *offset)
{
    return 0;
}

static char* ist8307a_get_vendor(const struct sensor_dev *dev)
{
    return "isentek";
}

static char* ist8307a_get_module(const struct sensor_dev *dev)
{
    return "ist8307a";
}

static float ist8307a_get_max_range(const struct sensor_dev *dev)
{
    return 2500; /* xy:1600 z:2500 */
}

static float ist8307a_get_power(const struct sensor_dev *dev)
{
    return 0.003; /* max:1200uA*2.8V*1s/1000000 */
}

static int ist8307a_get_min_delay(const struct sensor_dev *dev)
{
    return 1000000 / MAX_ODR;
}

static int ist8307a_get_max_delay(const struct sensor_dev *dev)
{
    return 1000000 / MIN_ODR;
}

static float ist8307a_get_resolution(const struct sensor_dev *dev)
{
    return MAG_XY_RESOLUTION;
}

static struct sensor_ops ist_ops = {
    .activate = ist8307a_activate,
    .set_delay = ist8307a_set_delay,
    .selftest = ist8307a_selftest,
    .set_offset = ist8307a_set_offset,
    .get_offset = ist8307a_get_offset,
    .get_vendor = ist8307a_get_vendor,
    .get_module = ist8307a_get_module,
    .get_max_range = ist8307a_get_max_range,
    .get_power = ist8307a_get_power,
    .get_min_delay = ist8307a_get_min_delay,
    .get_max_delay = ist8307a_get_max_delay,
    .get_resolution = ist8307a_get_resolution,
};

static int ist8307a_probe(const struct sensor_platform_data *pdata,
            const struct sensor_matching_data *mdata,
            struct sensor_dev **dev, uint32_t *num_sensors)
{
    struct sensor_dev *sdevs;
    struct ist8307a_rtdata *rtdata;

    size_t msize;
    int ret;

    msize = sizeof(*sdevs) + sizeof(struct ist8307a_rtdata);
    sdevs = calloc(1, msize);
    if (!sdevs) {
        printf("ist8307a failed to malloc\n");
        return -ENOMEM;
    }

    rtdata = (struct ist8307a_rtdata *)&sdevs[1];
    rtdata->pdata = pdata;

    ret = sns_port_init(&pdata->bus_info, &rtdata->port);
    if (ret) {
        printf("failed to init port for %s\n", pdata->name);
        goto port_err;
    }

    ret = ist8307a_init_chip(rtdata);
    if (ret) {
        printf("ist8307a init failed:%d\n", ret);
        goto init_err;
    }

    rtdata->mutex = osMutexNew(NULL);
    if (!rtdata->mutex) {
        printf("create mutex failed for ist8307a\n");
        ret = osError;
        goto mutex_err;
    }

    rtdata->poll_timer = osTimerNew(ist8307a_timer_cb, osTimerPeriodic, rtdata, NULL);
    if (ret) {
        printf("failed to create poller timer for ist8307a:%d\n", ret);
        goto timer_err;
    }

    init_sensor_dev(&sdevs[0], MAG_SENSOR, SENSOR_TYPE_MAGNETIC_FIELD,
            &ist_ops, pdata, rtdata, mdata);

    *dev = sdevs;
    *num_sensors = 1;

    printf("ist8307a mag-sensor probe success\n");
    return ret;

timer_err:
    osMutexDelete(rtdata->mutex);
mutex_err:
init_err:
    sns_port_deinit(&rtdata->port);
port_err:
    free(sdevs);

    return ret;
}

const static struct sensor_matching_data ist8307a_mdata = {
    .name = "isentek,ist8307a",
    .priv = NULL,
};

const static struct sensor_matching_data *drv_mdata[] = {
    &ist8307a_mdata,
    NULL,
};

const static struct sensor_driver __define_drv__ ist8307a_drv = {
    .name = "ist8307a",
    .mdata = &drv_mdata[0],
    .probe = ist8307a_probe,
};
