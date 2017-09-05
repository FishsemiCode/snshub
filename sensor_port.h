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

#ifndef __SENSOR_PORT_H__
#define __SENSOR_PORT_H__

#define SPI_CPOL0_CPHA0     0
#define SPI_CPOL0_CPHA1     1
#define SPI_CPOL1_CPHA0     2
#define SPI_CPOL1_CPHA1     3

typedef enum {
    BUS_I2C,
    BUS_SPI
} bus_type_t;

struct sensor_i2c_info {
    int master_id;
    int slave_addr;
};

struct sensor_spi_info {
    int master_id;
    int chip_select;
    int mode;
    int max_frequency;
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
    int (*read)(struct sns_port *port, uint8_t reg, void *buff, size_t size);
    int (*write)(struct sns_port *port, uint8_t reg, void *buff, size_t size);
};

struct sns_port {
    void *master;
    struct port_ops *ops;
    const struct sensor_bus_info *binfo;
};

/* XXX:NOTE: these interface should never be called in an ISR context */
int sns_port_init(const struct sensor_bus_info *info, struct sns_port *port);
int sns_port_deinit(struct sns_port *port);
int sns_port_read(struct sns_port *port, uint8_t reg, void *buff, size_t size);
int sns_port_write(struct sns_port *port, uint8_t reg, void *buff, size_t size);
#endif
