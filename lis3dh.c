/****************************************************************************
 * /vendor/services/snshub/lis3dh.c
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
 *   Author: Ming Yang <yangming@fishsemi.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>
#include <nuttx/random.h>
#include <nuttx/fs/fs.h>
#include <nuttx/semaphore.h>

#include "sensor.h"
#include "sensor_manager.h"
#include "sensor_driver.h"
#include "sensor_port.h"
#include "utils.h"
#include "lis3dh.h"

struct lis3dh_rtdata {
  struct sns_port port;
  const struct sensor_platform_data *pdata;

  bool acc_enabled;
  uint32_t a_odr;
  uint32_t accel_fsr;
  pthread_mutex_t mutex;
  timer_t timer;
  struct work_s work;
  int64_t ts;
};
struct lis3dh_odr_info lis3dh_acc_odr[] = {
  /* 1hz 1000ms */
  {LIS3DH_CTRL_REG1_ODR_1Hz, 1, 1000000},
  /* 10hz 100ms */
  {LIS3DH_CTRL_REG1_ODR_10Hz, 10, 100000},
  /* 25hz 40ms */
  {LIS3DH_CTRL_REG1_ODR_25Hz, 25, 40000},
  /* 50hz 20ms */
  {LIS3DH_CTRL_REG1_ODR_50Hz, 50, 20000},
  /* 100hz 10ms */
  {LIS3DH_CTRL_REG1_ODR_100Hz, 100, 10000},
  /* 200hz 5ms */
  {LIS3DH_CTRL_REG1_ODR_200Hz, 200, 5000},
  /* 400hz 2.5ms */
  {LIS3DH_CTRL_REG1_ODR_400Hz, 400, 2500},
  /* 1.6khz 625us */
  {LIS3DH_CTRL_REG1_ODR_LP_1600Hz, 1600, 625},
  /* 1.344khz 744us */
  {LIS3DH_CTRL_REG1_ODR_1344Hz, 1344, 744},
  /* 5.376khz 186us */
  {LIS3DH_CTRL_REG1_ODR_LP_5376Hz, 5376, 186},
};
const static struct lis3dh_fsr_info lis3dh_acc_fsr[] = {
  /* the resolution is 9.8/sensitivity */
  {2,  LIS3DH_CTRL_REG4_FS_2G , 0.001},
  {4,  LIS3DH_CTRL_REG4_FS_4G , 0.002},
  {8,  LIS3DH_CTRL_REG4_FS_8G , 0.004},
  {16, LIS3DH_CTRL_REG4_FS_16G, 0.008},
};
static int lis3dh_set_odr(FAR struct lis3dh_rtdata *rtdata, uint32_t current_odr);

static inline int lis_reg_read(FAR struct sns_port *port, uint8_t reg, FAR void *data, int len)
{
  uint8_t modify;

  modify = port->binfo->bus_type == BUS_SPI ? 0x80 : 0x00;
  return sns_port_read(port, reg | modify, data, len);
}

static inline int lis_reg_write(FAR struct sns_port *port, uint8_t reg, uint8_t reg_data)
{
  return sns_port_write(port, reg, &reg_data, 1);
}

static inline int lis3dh_full_scale_set(FAR struct lis3dh_rtdata *rtdata, int8_t val)
{
  int ret;
  lis3dh_ctrl_reg4 ctrl_reg4;

  ret = lis_reg_read(&rtdata->port, LIS3DH_CTRL_REG4, (uint8_t *)(&ctrl_reg4), 1);
  ctrl_reg4.fs = val;
  ret = lis_reg_write(&rtdata->port, LIS3DH_CTRL_REG4, *(uint8_t *)(&ctrl_reg4));

  return ret;
}

static inline int lis3dh_block_data_update_set(FAR struct lis3dh_rtdata *rtdata, int8_t val)
{
  int ret;
  lis3dh_ctrl_reg4 ctrl_reg4;

  ret = lis_reg_read(&rtdata->port, LIS3DH_CTRL_REG4, (uint8_t *)(&ctrl_reg4), 1);
  ctrl_reg4.bdu = val;
  ret = lis_reg_write(&rtdata->port, LIS3DH_CTRL_REG4, *(uint8_t *)(&ctrl_reg4));

  return ret;
}

static int lis3dh_readdata_set_mode(FAR struct lis3dh_rtdata *rtdata, enum lis3dh_readmode mode)
{
  int ret = OK;
  uint8_t temp;

  switch (mode)
    {
      /*FIFO mode*/
      case fifo_mode:
        ret = lis_reg_read(&rtdata->port, LIS3DH_FIFO_CTRL_REG, &temp, 1);
        temp &= 0x3f;
        temp |= LIS3DH_FIFO_CTRL_REG_MODE_FIFO;
        ret = lis_reg_write(&rtdata->port, LIS3DH_FIFO_CTRL_REG, temp);
        break;

      /*STREAM_FIFO*/
      case stream_mode:
        ret = lis_reg_read(&rtdata->port, LIS3DH_FIFO_CTRL_REG, &temp, 1);
        temp &= 0x3f;
        temp |= LIS3DH_FIFO_CTRL_REG_MODE_STREAM;
        ret = lis_reg_write(&rtdata->port, LIS3DH_FIFO_CTRL_REG, temp);
        break;

      /*STREAM_TO_FIFO*/
      case stream_t_fifo_mode:
        ret = lis_reg_read(&rtdata->port, LIS3DH_FIFO_CTRL_REG, &temp, 1);
        temp &= 0x3f;
        temp |= LIS3DH_FIFO_CTRL_REG_MODE_STREAM2;
        ret = lis_reg_write(&rtdata->port, LIS3DH_FIFO_CTRL_REG, temp);
        break;

      /*bypass*/
      case bypass_mode:
        ret = lis_reg_read(&rtdata->port, LIS3DH_FIFO_CTRL_REG, &temp, 1);
        temp &= 0x3f;
        temp |= LIS3DH_FIFO_CTRL_REG_MODE_BYPASS;
        ret = lis_reg_write(&rtdata->port, LIS3DH_FIFO_CTRL_REG, temp);
        break;

      default:
        ret = -EINVAL;
        break;
    }

  return ret;
}

static void lis3dh_accel_enable(FAR struct lis3dh_rtdata *rtdata, bool enable)
{
  int ret;

  if (enable)
    {
      ret = lis_reg_write(&rtdata->port, LIS3DH_CTRL_REG1,
                          LIS3DH_CTRL_REG1_ZEN | LIS3DH_CTRL_REG1_XEN | LIS3DH_CTRL_REG1_YEN);
      ret = lis3dh_set_odr(rtdata, 1);
      if (ret)
        {
          snshuberr("acc enable failed:%d\n", ret);
        }
    }
  else
    {
      ret = lis_reg_write(&rtdata->port, LIS3DH_CTRL_REG1, false);
      if (ret)
        {
          snshuberr("lis3dh reg failed:%d\n", ret);
        }

    }
}

static int lis3dh_activate(FAR const struct sensor_dev *dev, bool enable, snshub_data_mode mode)
{
  FAR struct lis3dh_rtdata *rtdata = dev->rtdata;
  FAR const struct lis3dh_platform_data *spdata = rtdata->pdata->spdata;
  bool old_status, new_status;

  pthread_mutex_lock(&rtdata->mutex);
  old_status = rtdata->acc_enabled;
  if (dev->type == SENSOR_TYPE_ACCELEROMETER && rtdata->acc_enabled != enable)
    {
      snshubinfo("lis: activate %s to %s\n", dev->name, enable ? "enable" : "disable");
      lis3dh_accel_enable(rtdata, enable);
      rtdata->acc_enabled = enable;
    }

   new_status = rtdata->acc_enabled;
   if (old_status != new_status)
     {
       if (new_status)
         {
           if (rtdata->timer)
             sensor_timer_start(true, &rtdata->timer, 1000 / rtdata->a_odr);
           else
             sensor_enable_gpio_irq(spdata->irq_pin, spdata->trigger_type);
         }
       else
         {
           if (rtdata->timer)
             sensor_timer_start(false, &rtdata->timer, 0);
           else
             sensor_enable_gpio_irq(spdata->irq_pin, IOEXPANDER_VAL_DISABLE);
         }
     }

  pthread_mutex_unlock(&rtdata->mutex);
  return OK;
}

static int lis3dh_find_odr(uint32_t except)
{
  int i;

  for (i = 0; i < __countof(lis3dh_acc_odr); i++)
    {
      if (except <= lis3dh_acc_odr[i].odr)
      return i;
    }

  return __countof(lis3dh_acc_odr) - 1;
}

static int lis3dh_set_odr(FAR struct lis3dh_rtdata *rtdata, uint32_t current_odr)
{
  int ret;
  uint8_t temp;
  int idx = lis3dh_find_odr(current_odr);

  if (rtdata->a_odr == lis3dh_acc_odr[idx].odr)
    {
      return OK;
    }

  ret = lis_reg_read(&rtdata->port, LIS3DH_CTRL_REG1, &temp, 1);
  temp &= 0x0f;
  temp |= lis3dh_acc_odr[idx].regval;
  ret = lis_reg_write(&rtdata->port, LIS3DH_CTRL_REG1, temp);
  rtdata->a_odr = lis3dh_acc_odr[idx].odr;

  return ret;
}

static int lis3dh_set_delay(FAR const struct sensor_dev *dev, uint32_t us)
{
  int ret;
  FAR struct lis3dh_rtdata *rtdata = dev->rtdata;

  pthread_mutex_lock(&rtdata->mutex);
  ret = lis3dh_set_odr(dev->rtdata, 1000000 / us);
  pthread_mutex_unlock(&rtdata->mutex);

  return ret;
}

static int lis3dh_selftest(FAR const struct sensor_dev *dev, void *data)
{
  return 0;
}

static int lis3dh_set_offset(FAR const struct sensor_dev *dev, void *offset)
{
  return 0;
}

static int lis3dh_get_offset(FAR const struct sensor_dev *dev, void *offset)
{
  return 0;
}

static FAR char* lis3dh_get_vendor(FAR const struct sensor_dev *dev)
{
  return "st";
}

static FAR char* lis3dh_get_module(FAR const struct sensor_dev *dev)
{
  return "lis3dh";
}

static float lis3dh_get_max_range(FAR const struct sensor_dev *dev)
{
  return lis3dh_acc_fsr[__countof(lis3dh_acc_fsr) - 1].fsr * GRAVITY;
}

static float lis3dh_get_power(FAR const struct sensor_dev *dev)
{
  return 0.05;
}

static int lis3dh_find_accel_fsr(uint32_t except)
{
  int i;

  for (i = 0; i < __countof(lis3dh_acc_fsr); i++)
    {
      if (except <= lis3dh_acc_fsr[i].fsr)
        return i;
    }

  return __countof(lis3dh_acc_fsr) - 1;
}

static float lis3dh_get_resolution(FAR const struct sensor_dev *dev)
{
  FAR struct lis3dh_rtdata *rtdata = dev->rtdata;
  int idx;

  if (dev->type == SENSOR_TYPE_ACCELEROMETER)
    {
      idx = lis3dh_find_accel_fsr(rtdata->accel_fsr);
    }

  return lis3dh_acc_fsr[idx].resolution;
}

static int lis3dh_get_max_delay(FAR const struct sensor_dev *dev)
{
  return lis3dh_acc_odr[0].times;
}

static int lis3dh_get_min_delay(FAR const struct sensor_dev *dev)
{
  return lis3dh_acc_odr[__countof(lis3dh_acc_odr) - 1].times;
}

static int lis3dh_get_data(FAR struct lis3dh_rtdata *rtdata, FAR struct sensor_event *event, uint64_t stamp)
{
  uint8_t temperature_raw[2];
  uint8_t reg_raw[6];
  int ret;
  lis3dh_status_reg status_reg;

  ret = lis_reg_read(&rtdata->port, LIS3DH_STATUS_REG, (uint8_t *)(&status_reg), 1);
  if (ret)
    {
      snshuberr(" failed:%d,%d\n", ret, __LINE__);
      return ret;
    }

  if (status_reg.xyzda && rtdata->acc_enabled)
    {
      ret = lis_reg_read(&rtdata->port, LIS3DH_OUT_X_L | ADDRESS_AUTO_INCRESE_READ, reg_raw, 6);
      if (ret)
        {
          snshuberr("lis3dh read reg: failed:%d\n", ret);
          return ret;
        }
      ret = lis_reg_read(&rtdata->port, LIS3DH_OUT_ADC3_L | ADDRESS_AUTO_INCRESE_READ, temperature_raw, 2);
      if (ret)
        {
          snshuberr("lis3dh read temperature: failed:%d\n", ret);
          return ret;
        }
      event->data[0] = ((int16_t)((reg_raw[0] | reg_raw[1] << 8)) >> 4) * 0.001;
      event->data[1] = ((int16_t)((reg_raw[2] | reg_raw[3] << 8)) >> 4) * 0.001;
      event->data[2] = ((int16_t)((reg_raw[4] | reg_raw[5] << 8)) >> 4) * 0.001;
      event->data[3] = (int)(temperature_raw[1]) + 25;
      event->type = SENSOR_TYPE_ACCELEROMETER;
      event->timestamp = stamp;
    }
}

int lis3dh_read_data(FAR const struct sensor_dev *dev, FAR struct sensor_event *event)
{
  uint64_t stamp = get_timestamp();
  FAR struct lis3dh_rtdata *rtdata = dev->rtdata;
  int ret;

  ret = lis3dh_get_data(rtdata, event, stamp);
  if (ret)
    {
      snshuberr("lis3dh get data :failed:%d\n", ret);
    }

  return 0;
}

struct sensor_ops lis_ops = {
  .activate = lis3dh_activate,
  .set_delay = lis3dh_set_delay,
  .selftest = lis3dh_selftest,
  .set_offset = lis3dh_set_offset,
  .get_offset = lis3dh_get_offset,
  .get_vendor = lis3dh_get_vendor,
  .get_module = lis3dh_get_module,
  .get_max_range = lis3dh_get_max_range,
  .get_power = lis3dh_get_power,
  .get_resolution = lis3dh_get_resolution,
  .get_max_delay = lis3dh_get_max_delay,
  .get_min_delay = lis3dh_get_min_delay,
  .read_data = lis3dh_read_data,
};

static void lis3dh_worker(FAR void *data)
{
  FAR struct lis3dh_rtdata *rtdata = data;
  FAR const struct lis3dh_platform_data *spdata = rtdata->pdata->spdata;
  struct sensor_event event;
  int ret;

  if (spdata->irq_pin)
    sensor_enable_gpio_irq(spdata->irq_pin, spdata->trigger_type);

  ret = lis3dh_get_data(rtdata, &event, rtdata->ts);
  if (ret)
    {
      snshuberr("lis3dh read data :failed:%d\n", ret);
    }

  smgr_push_data(&event, 1);
}

static void lis3dh_timer_cb(union sigval arg)
{
  struct lis3dh_rtdata *rtdata = arg.sival_ptr;

  rtdata->ts = get_timestamp();
  work_queue(LPWORK, &rtdata->work, lis3dh_worker, rtdata, 0);
}

static int lis3dh_irq_handler(FAR struct ioexpander_dev_s *dev, ioe_pinset_t pinset, FAR void *data)
{
  FAR struct lis3dh_rtdata *rtdata = data;
  FAR const struct lis3dh_platform_data *spdata = rtdata->pdata->spdata;

  rtdata->ts = get_timestamp();
  /* this is an IRQ context, we must use the pended worker
   * since the i2c access will be blocked */
  work_queue(LPWORK, &rtdata->work, lis3dh_worker, rtdata, 0);
  /* XXX: because it's a level triggered interrupt, the handler would be called
   * frequetly until the pended handler is executed to read fifo data register.
   * so we need to temperarily disable the gpio interrupt and then re-enable it
   * in the pended handler */
  sensor_enable_gpio_irq(spdata->irq_pin, IOEXPANDER_VAL_DISABLE);
  return 0;
}

static int lis3dh_power_set_mode(FAR struct lis3dh_rtdata *rtdata, int8_t val)
{
  int ret;
  uint8_t ctrl_reg1_len;
  uint8_t ctrl_reg4_hr;

  ret = lis_reg_read(&rtdata->port, LIS3DH_CTRL_REG1, &ctrl_reg1_len, 1);
  ret = lis_reg_read(&rtdata->port, LIS3DH_CTRL_REG4, &ctrl_reg4_hr, 1);

  switch (val)
    {
      case LIS3DH_POWER_MODE_LOW:
        ctrl_reg1_len |= LIS3DH_CTRL_REG1_LPEN;
        ctrl_reg4_hr  &= ~LIS3DH_CTRL_REG4_HR;
        break;

      case LIS3DH_POWER_MODE_NORMAL:
        ctrl_reg1_len &= ~LIS3DH_CTRL_REG1_LPEN;
        ctrl_reg4_hr  &= ~LIS3DH_CTRL_REG4_HR;
        break;

      case LIS3DH_POWER_MODE_HIGH:
        ctrl_reg1_len &= ~LIS3DH_CTRL_REG1_LPEN;
        ctrl_reg4_hr  |= LIS3DH_CTRL_REG4_HR;
        break;

      default:
        ret = -EINVAL;
        break;
    }

  ret = lis_reg_write(&rtdata->port, LIS3DH_CTRL_REG1, ctrl_reg1_len);
  ret = lis_reg_write(&rtdata->port, LIS3DH_CTRL_REG4, ctrl_reg4_hr);

  return ret;
}

static int lis3dh_aux_adc_set(FAR struct lis3dh_rtdata *rtdata, bool lis3dh_temp_enable)
{
  int ret;
  uint8_t temp_reg;

  ret = lis_reg_read(&rtdata->port, LIS3DH_TEMP_CFG_REG, &temp_reg, 1);
  temp_reg = (lis3dh_temp_enable == true ? temp_reg | ADDRESS_AUTO_INCRESE_READ : temp_reg & 0x7f);
  ret = lis3dh_block_data_update_set(rtdata, true);
  ret = lis_reg_write(&rtdata->port, LIS3DH_TEMP_CFG_REG, temp_reg);

  return ret;
}

static int lis3dh_init_chip(FAR struct lis3dh_rtdata *rtdata)
{
  int ret;
  uint8_t reg_value;

  ret = lis_reg_write(&rtdata->port, LIS3DH_CTRL_REG5, LIS3DH_CTRL_REG5_BOOT);
  usleep(100000);
  ret = lis_reg_read(&rtdata->port, LIS3DH_WHO_AM_I, &reg_value, 1);
  if (ret)
    {
      snshuberr("lis3dh read reg:0x%x failed:%d\n", LIS3DH_VER_WHO_AM_I, ret);
      return ret;
    }
  if (reg_value != LIS3DH_VER_WHO_AM_I)
    {
      snshuberr("lis3dh who am i value:0x%x\n", reg_value);
      return -ENXIO;
    }
  ret = lis3dh_block_data_update_set(rtdata, true);
  if (ret)
    {
      snshuberr("lis3dh block data update set failed %d\n", ret);
    }
  ret = lis3dh_set_odr(rtdata, 1);
  if (ret)
    {
       snshuberr("lis3dh set odr failed %d\n", ret);
    }

  ret = lis3dh_full_scale_set(rtdata, LIS3DH_CTRL_REG4_FS_2G);
  if (ret)
    {
      snshuberr("lis3dh set full scale failed %d\n", ret);
    }
  ret = lis3dh_readdata_set_mode(rtdata, bypass_mode);
  if (ret)
    {
      snshuberr("lis3dh set bypass scale failed %d\n", ret);
    }
  ret = lis3dh_aux_adc_set(rtdata, true);
  if (ret)
    {
      snshuberr("lis3dh temperature init failed\n");
    }
  ret = lis3dh_power_set_mode(rtdata, LIS3DH_POWER_MODE_HIGH);
  if (ret)
    {
      snshuberr("lis3dh set mode failed %d\n", ret);
    }
  ret = lis3dh_set_odr(rtdata, 1);
  if (ret)
    {
       snshuberr("lis3dh set odr failed %d\n", ret);
    }

  return ret;
}

static int lis3dh_probe(FAR const struct sensor_platform_data *pdata,
      FAR const struct sensor_matching_data *mdata,
      FAR struct sensor_dev **dev, FAR uint32_t *num_sensors)
{
  FAR struct sensor_dev *sdevs;
  FAR struct lis3dh_rtdata *rtdata;
  FAR const struct lis3dh_platform_data *spdata = pdata->spdata;
  int ret;
  void *handle;

  sdevs = calloc(1, sizeof(*sdevs) + sizeof(struct lis3dh_rtdata));
  if (!sdevs)
    {
      snshuberr("failed to calloc memory for lis3dh\n");
      return -ENOMEM;
    }

  rtdata = (struct lis3dh_rtdata *)(&sdevs[1]);
  rtdata->pdata = pdata;

  ret = sns_port_init(&pdata->bus_info, &rtdata->port);
  if (ret)
    {
      snshuberr("failed to init port for %s\n", pdata->name);
      goto port_err;
    }
  ret = lis3dh_init_chip(rtdata);
  if (ret)
    {
      snshuberr("failed to init for lis3dh\n");
      goto init_err;
    }
  ret = pthread_mutex_init(&rtdata->mutex, NULL);
  if (ret)
    {
      snshuberr("failed to create pthread mutex, ret:%d\n", ret);
      goto mutex_err;
    }
  if (spdata->irq_pin)
    {
      handle = sensor_register_gpio_irq(spdata->irq_pin, spdata->trigger_type, lis3dh_irq_handler, rtdata);
      if (handle == NULL) {
        snshuberr("failed to register irq handler for lis3dh\n");
        goto gpio_irq_err;
      }
    }
  else
    {
      ret = sensor_timer_init(&rtdata->timer, lis3dh_timer_cb, rtdata);
      if (ret)
        {
          snshuberr("failed to create timer for lis3dh sensor\n");
          ret = -ETIME;
          goto timer_err;
        }
    }
  /*name is ACCEL_SENSO*/
  init_sensor_dev(&sdevs[0], ACCEL_SENSOR, SENSOR_TYPE_ACCELEROMETER, &lis_ops, pdata, rtdata, mdata);
  *dev = sdevs;
  *num_sensors = 1;

  snshubinfo("1 sensor probed for lis3dh\n");

  return ret;

timer_err:
gpio_irq_err:
  pthread_mutex_destroy(&rtdata->mutex);
mutex_err:
init_err:
  sns_port_deinit(&rtdata->port);
port_err:
  free(sdevs);

  return ret;
}

const static struct sensor_matching_data lis3dh_mdata = {
  .name = "st,lis3dh",
  .priv = NULL,
};

const static struct sensor_matching_data *drv_mdata[] = {
  &lis3dh_mdata,
  NULL,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

const struct sensor_driver lis3dh_drv = {
  .name = "lis3dh",
  .mdata = &drv_mdata[0],
  .probe = lis3dh_probe,
};
