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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include "sensor_port_spi.h"
#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>
#include "utils.h"

#define MAX_TRANSFER_LENGTH   256

static int get_max_master_id(void)
{
  int i;

  for (i = 0; g_spi[i]; i++);

  return i - 1;
}

static int _spi_read(FAR struct sns_port *port, uint8_t reg, FAR void *buff, size_t size)
{
  FAR struct spi_dev_s *drv_spi = port->master;
  FAR const struct sensor_spi_info *spi_info = &port->binfo->u.spi_info;
  struct spi_sequence_s seq;
  struct spi_trans_s trans;
  uint8_t rx_buffer[MAX_TRANSFER_LENGTH + 1] = "\0";
  uint8_t tx_buffer[MAX_TRANSFER_LENGTH + 1] = "\0";
  int ret;

  if (size > MAX_TRANSFER_LENGTH)
    return -EINVAL;

  memset(&seq, 0, sizeof(seq));

  seq.mode = spi_info->mode;
  seq.nbits = 8;
  seq.ntrans = 1;
  seq.frequency = spi_info->max_frequency;

  trans.delay = 0;
  trans.nwords = size + 1;
  trans.txbuffer = tx_buffer;
  trans.rxbuffer = rx_buffer;

  seq.trans = &trans;

  tx_buffer[0] = reg;

  ret = spi_transfer(drv_spi, &seq);

  if (!ret)
    memcpy(buff, rx_buffer + 1, size);

  return ret;
}

static int _spi_write(FAR struct sns_port *port, uint8_t reg, FAR void *buff, size_t size)
{
  FAR struct spi_dev_s *drv_spi = port->master;
  FAR const struct sensor_spi_info *spi_info = &port->binfo->u.spi_info;
  struct spi_sequence_s seq;
  struct spi_trans_s trans;
  uint8_t rx_buffer[MAX_TRANSFER_LENGTH + 1] = "\0";
  uint8_t tx_buffer[MAX_TRANSFER_LENGTH + 1] = "\0";

  if (size > MAX_TRANSFER_LENGTH)
    return -EINVAL;

  memset(&seq, 0, sizeof(seq));

  seq.mode = spi_info->mode;
  seq.nbits = 8;
  seq.ntrans = 1;
  seq.frequency = spi_info->max_frequency;

  trans.delay = 0;
  trans.nwords = size + 1;
  trans.txbuffer = tx_buffer;
  trans.rxbuffer = rx_buffer;

  seq.trans = &trans;

  tx_buffer[0] = reg;
  memcpy(tx_buffer + 1, buff, size);

  return spi_transfer(drv_spi, &seq);
}

static struct port_ops spi_ops = {
  .read = _spi_read,
  .write = _spi_write,
};

int port_init_spi(FAR const struct sensor_bus_info *info, FAR struct sns_port *port)
{
  FAR const struct sensor_spi_info *spi_info = &info->u.spi_info;
  int max_id;

  max_id = get_max_master_id();
  if (max_id < 0 || max_id < spi_info->master_id) {
    snshuberr("no spi master found\n");
    return -ENODEV;
  }

  port->master = g_spi[spi_info->master_id];
  port->ops = &spi_ops;
  port->binfo = info;

  return 0;
}
