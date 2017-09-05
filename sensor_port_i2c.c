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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <cmsis_os2.h>
#include <sys/list.h>
#include <Driver_I2C.h>
#include "sensor_port_i2c.h"

#define I2C_TIMEOUT         (osMsecToTick(1000))

#define I2C_FLAG_COMPLETE   0x01

#define MAX_WRITE_LENGTH    16

struct transfer_st {
    osThreadId_t thread;
    uint32_t event;
};

struct port_i2c_master {
    int id;
    ARM_DRIVER_I2C *drv_i2c;
    osMutexId_t mutex;
    struct transfer_st st;
    struct list_node node;
};

static struct list_node i2c_masters = LIST_INITIAL_VALUE(i2c_masters);

static int get_max_master_id()
{
    int i;

    for (i = 0; Driver_I2C[i]; i++);

    return i - 1;
}

static struct port_i2c_master *get_i2c_master(ARM_DRIVER_I2C *drv_i2c)
{
    struct port_i2c_master *master;

    if (!list_is_empty(&i2c_masters)) {
        list_for_every_entry(&i2c_masters, master, struct port_i2c_master, node) {
            if (master->drv_i2c == drv_i2c) {
                return master;
            }
        }
    }

    return NULL;
}

static void i2c_signal_handler(uint32_t event, void *prv)
{
    struct port_i2c_master *master = prv;
    osThreadId_t thread = master->st.thread;

    if (!thread) {
        printf("error thread state\n");
        return;
    }

    master->st.event = event;
    osThreadFlagsSet(thread, I2C_FLAG_COMPLETE);
}

static int init_i2c_master(struct port_i2c_master *master, ARM_DRIVER_I2C *drv_i2c)
{
    int ret;

    ret = drv_i2c->SetExtHandler(i2c_signal_handler, master);
    if (ret != ARM_DRIVER_OK) {
        printf("init i2c master %d failed\n", master->id);
        return ret;
    }

    master->mutex = osMutexNew(NULL);
    if (!master->mutex) {
        printf("create mutex failed for i2c master %d\n", master->id);
        ret = -EPIPE;
        goto create_mutex_err;
    }

    master->drv_i2c = drv_i2c;
    list_add_tail(&i2c_masters, &master->node);
    return 0;

create_mutex_err:
    drv_i2c->Uninitialize();

    return ret;
}

static int i2c_xfer(struct port_i2c_master *master, ARM_I2C_MSG *msg, size_t msg_num)
{
    osThreadId_t thread;
    ARM_I2C_STATUS status;
    int ret;

    osMutexAcquire(master->mutex, osWaitForever);

    osDisableStandby();
    thread = osThreadGetId();
    master->st.thread = thread;

    master->drv_i2c->PowerControl(ARM_POWER_FULL);
    ret = master->drv_i2c->MasterTransfer(msg, msg_num);
    if (ret != ARM_DRIVER_OK) {
        printf("i2c transfer failed:%d\n", ret);
        goto transfer_err;
    }

    ret = osThreadFlagsWait(I2C_FLAG_COMPLETE, osFlagsWaitAny, I2C_TIMEOUT);
    if (ret < 0) {
        printf("i2c transfer wait err:%d\n", ret);
        goto wait_err;
    }

    status = master->drv_i2c->GetStatus();
    /*TODO: add some error handlering here, like recovery/print */

    if (status.bus_error)
        ret = -EIO;
    else
        ret = 0;

wait_err:
transfer_err:
    master->st.thread = NULL;
    master->drv_i2c->PowerControl(ARM_POWER_OFF);
    osEnableStandby();
    osMutexRelease(master->mutex);

    return ret;
}

static int i2c_read(struct sns_port *port, uint8_t reg, void *buff, size_t size)
{
    struct port_i2c_master *master = port->master;
    const struct sensor_i2c_info *i2c_info = &port->binfo->u.i2c_info;
    ARM_I2C_MSG msg[2];

    msg[0].addr = i2c_info->slave_addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &reg;
    msg[1].addr = i2c_info->slave_addr;
    msg[1].flags = ARM_I2C_M_RD;
    msg[1].len = size;
    msg[1].buf = buff;

    return i2c_xfer(master, msg, 2);
}

static int i2c_write(struct sns_port *port, uint8_t reg, void *buff, size_t size)
{
    struct port_i2c_master *master = port->master;
    const struct sensor_i2c_info *i2c_info = &port->binfo->u.i2c_info;
    ARM_I2C_MSG msg;
    uint8_t data[MAX_WRITE_LENGTH + 1];

    if (size > MAX_WRITE_LENGTH) {
        printf("too much write data\n");
        return -EINVAL;
    }

    data[0] = reg;
    memcpy(&data[1], buff, size);

    msg.addr = i2c_info->slave_addr;
    msg.flags = 0;
    msg.len = size + 1;
    msg.buf = data;

    return i2c_xfer(master, &msg, 1);
}

static struct port_ops i2c_ops = {
    .read = i2c_read,
    .write = i2c_write,
};

int port_init_i2c(const struct sensor_bus_info *info, struct sns_port *port)
{
    struct port_i2c_master *master;
    const struct sensor_i2c_info *i2c_info = &info->u.i2c_info;
    ARM_DRIVER_I2C *drv_i2c;
    int max_id;
    int ret = 0;

    max_id = get_max_master_id();
    if (max_id < 0 || max_id < i2c_info->master_id) {
        printf("no i2c master found\n");
        return -ENODEV;
    }

    drv_i2c = Driver_I2C[i2c_info->master_id];
    /* has this master been already initialized ? */
    if (!(master = get_i2c_master(drv_i2c))) {
        master = calloc(1, sizeof(*master));
        if (!master) {
            printf("failed to malloc for i2c master %d\n", i2c_info->master_id);
            return -ENOMEM;
        }

        master->id = i2c_info->master_id;
        ret = init_i2c_master(master, drv_i2c);
        if (ret)
            free(master);
    }

    if (!ret) {
        port->master = master;
        port->ops = &i2c_ops;
        port->binfo = info;
    }

    return ret;
}
