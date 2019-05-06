/* Copyright Statement:
 *
 * This software/firmware and related documentation ("Fishsemi Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to Fishsemi Inc. and/or its licensors. Without
 * the prior written permission of Fishsemi inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of Fishsemi Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 *
 * Fishsemi Inc. (C) 2019. All rights reserved.
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
 * The following software/firmware and/or related documentation ("Fishsemi
 * Software") have been modified by Fishsemi Inc. All revisions are subject to
 * any receiver's applicable license agreements with Fishsemi Inc.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <signal.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "sensor.h"
#include "utils.h"

/****************************************************************************
 * Private
 ****************************************************************************/

struct axis_map {
  int8_t src_x;
  int8_t src_y;
  int8_t src_z;

  int8_t sign_x;
  int8_t sign_y;
  int8_t sign_z;
};

static const struct  axis_map remap_tbl[] =
{
  /* src_x src_y src_z  sign_x  sign_y  sign_z */
  {  0,  1,  2,   1,    1,    1 }, /* P0 */
  {  1,  0,  2,   1,   -1,    1 }, /* P1 */
  {  0,  1,  2,  -1,   -1,    1 }, /* P2 */
  {  1,  0,  2,  -1,    1,    1 }, /* P3 */

  {  0,  1,  2,  -1,    1,   -1 }, /* P4 */
  {  1,  0,  2,  -1,   -1,   -1 }, /* P5 */
  {  0,  1,  2,   1,   -1,   -1 }, /* P6 */
  {  1,  0,  2,   1,    1,   -1 }, /* P7 */
};

/****************************************************************************
 * Public
 ****************************************************************************/

void remap_vector_raw16to32_axis(const int16_t *in, int32_t *out, int place)
{
  const struct axis_map *remap;

  DEBUGASSERT(place < __countof(remap_tbl));

  remap = &remap_tbl[place];
  out[0] = in[remap->src_x] * remap->sign_x;
  out[1] = in[remap->src_y] * remap->sign_y;
  out[2] = in[remap->src_z] * remap->sign_z;
}

void remap_vector_raw32_axis(const int32_t *in, int32_t *out, int place)
{
}

FAR void *sensor_register_gpio_irq(int pin, int trigger_type, ioe_callback_t callback, FAR void *data)
{
  struct ioexpander_dev_s *ioe = g_ioe[0];
  ioe_pinset_t pinset = (((ioe_pinset_t)1 << pin) & IOEXPANDER_PINMASK);
  FAR void *cb = NULL;
  int ret, fd;
  char pinctrl_path[] = "/dev/pinctrl0";

  fd = open(pinctrl_path, O_RDWR);
  if (fd < 0)
    {
      printf("open %s failed, ret:%s\n", pinctrl_path, strerror(errno));
      return NULL;
    }

  /* pinctrl control start form gpio4 */
  ret = ioctl(fd, PINCTRLC_SELGPIO, pin - 4);
  if (ret < 0)
    {
      printf("ioctl %s failed, ret:%s\n", pinctrl_path, strerror(errno));
      return NULL;
    }

  close(fd);

  ret = IOEXP_SETDIRECTION(ioe, pin, IOEXPANDER_DIRECTION_IN);
  if (ret)
    return cb;

  cb = IOEP_ATTACH(ioe, pinset, callback, data);
  if (cb == NULL)
    return cb;

  ret = IOEXP_SETOPTION(ioe, pin, IOEXPANDER_OPTION_INTCFG, (void *)trigger_type);
  if (ret)
    {
      IOEP_DETACH(ioe, cb);
    }

  return cb;
}

int sensor_unregister_gpio_irq(int pin, FAR void *handle)
{
  FAR struct ioexpander_dev_s *ioe = g_ioe[0];

  IOEXP_SETOPTION(ioe, pin, IOEXPANDER_OPTION_INTCFG, IOEXPANDER_VAL_DISABLE);

  return IOEP_DETACH(ioe, handle);
}

int sensor_enable_gpio_irq(int pin, int trigger_type)
{
  FAR struct ioexpander_dev_s *ioe = g_ioe[0];

  return IOEXP_SETOPTION(ioe, pin, IOEXPANDER_OPTION_INTCFG, (void *)trigger_type);
}

int sensor_timer_init(timer_t *timerid, sigev_notify_function_t cb, void *arg)
{
  struct sigevent notify;

  notify.sigev_value.sival_ptr = arg;
  /* need enable CONFIG_SIG_EVTHREAD */
  notify.sigev_notify = SIGEV_THREAD;
  notify.sigev_notify_function = cb;

  return timer_create(CLOCK_REALTIME, &notify, timerid);
}

int sensor_timer_start(bool enable, timer_t *timerid, uint32_t millisec)
{
  struct itimerspec spec;

  memset(&spec, 0, sizeof(spec));

  if (enable) {
    spec.it_value.tv_sec = 1;
    spec.it_value.tv_nsec = 0;
    spec.it_interval.tv_sec = millisec / 1000;
    spec.it_interval.tv_nsec = (millisec % 1000) * 1000 * 1000;
  }

  return timer_settime(*timerid, 0, &spec, NULL);
}
