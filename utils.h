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

#ifndef __UTILS_H__
#define __UTILS_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <time.h>
#include <nuttx/ioexpander/ioexpander.h>
#include <nuttx/pinctrl/pinctrl.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_SNSHUB_ERROR
#  define snshuberr(format, ...)  _err(format, ##__VA_ARGS__)
#else
#  define snshuberr(x...)
#endif

#ifdef CONFIG_DEBUG_SNSHUB_WARN
#  define snshubwarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define snshubwarn(x...)
#endif

#ifdef CONFIG_DEBUG_SNSHUB_INFO
#  define snshubinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define snshubinfo(x...)
#endif

#define timespec_to_nsec(ts) \
    ((uint64_t)(ts)->tv_sec * NSEC_PER_SEC + (ts)->tv_nsec)
#define __countof(arr) (sizeof(arr) / sizeof((arr)[0]))
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

typedef enum {
  INIT_BASE  = 5,
  SH_INIT_HW_SNS = INIT_BASE + 5,
  SH_INIT_SMGR = INIT_BASE + 10,
  SH_INIT_CMGR = INIT_BASE + 15,
  SH_INIT_IPC = INIT_BASE + 20,
  SH_INIT_APP = INIT_BASE + 25
} sh_init_order_t;

static inline int64_t get_timestamp(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return timespec_to_nsec(&ts);
}

void remap_vector_raw16to32_axis(FAR const int16_t *in, FAR int32_t *out, int place);
void remap_vector_raw32_axis(FAR const int32_t *in, FAR int32_t *out, int place);

FAR void *sensor_register_gpio_irq(int pin, int trigger_type, ioe_callback_t callback, FAR void *data);
int sensor_unregister_gpio_irq(int pin, FAR void *handle);
/* IOEXPANDER_VAL_DISABLE for trigger_type means to disable the interrupt */
int sensor_enable_gpio_irq(int pin, int trigger_type);

int sensor_timer_init(timer_t *timerid, sigev_notify_function_t cb, void *arg);
int sensor_timer_start(bool enable, timer_t *timerid, uint32_t millisec);

#endif
