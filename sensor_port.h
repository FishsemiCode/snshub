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

#ifndef __SENSOR_PORT_H__
#define __SENSOR_PORT_H__

#define SPI_CPOL0_CPHA0   0
#define SPI_CPOL0_CPHA1   1
#define SPI_CPOL1_CPHA0   2
#define SPI_CPOL1_CPHA1   3

#include <nuttx/spi/spi.h>

typedef enum {
  BUS_I2C,
  BUS_SPI
} bus_type_t;

struct sensor_i2c_info {
  uint8_t master_id;
  uint32_t frequency;
  uint16_t slave_addr;
  uint8_t addr_len;
};

struct sensor_spi_info {
  int master_id;
  int mode;
  int max_frequency;
  enum spi_devtype_e type;
};

struct sensor_bus_info {
  bus_type_t bus_type;
  union {
    struct sensor_i2c_info i2c_info;
    struct sensor_spi_info spi_info;
  } u;
};

struct sns_port;
struct port_ops {
  int (*read)(FAR struct sns_port *port, uint8_t reg, FAR void *buff, size_t size);
  int (*write)(FAR struct sns_port *port, uint8_t reg, FAR void *buff, size_t size);
};

struct sns_port {
  FAR void *master;
  FAR struct port_ops *ops;
  FAR const struct sensor_bus_info *binfo;
};

/* XXX:NOTE: these interface should never be called in an ISR context */
int sns_port_init(FAR const struct sensor_bus_info *info, FAR struct sns_port *port);
int sns_port_deinit(FAR struct sns_port *port);
int sns_port_read(FAR struct sns_port *port, uint8_t reg, FAR void *buff, size_t size);
int sns_port_write(FAR struct sns_port *port, uint8_t reg, FAR void *buff, size_t size);
#endif
