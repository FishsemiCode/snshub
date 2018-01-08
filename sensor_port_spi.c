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
#include <Driver_SPI.h>
#include "sensor_port_spi.h"

#define SPI_TIMEOUT             (osMsecToTick(1000))
#define SPI_FLAG_COMPLETE       0x01
#define MAX_TRANSFER_LENGTH     256

struct transfer_st {
    osThreadId_t thread;
    uint32_t event;
};

struct port_spi_master {
    int id;
    ARM_DRIVER_SPI *drv_spi;
    osMutexId_t mutex;
    struct transfer_st st;
    union {
        int32_t tx_pool[MAX_TRANSFER_LENGTH / sizeof(int32_t) + 1];
        uint8_t tx_buffer[MAX_TRANSFER_LENGTH + 1];
    };
    union {
        int32_t rx_pool[MAX_TRANSFER_LENGTH / sizeof(int32_t) + 1];
        uint8_t rx_buffer[MAX_TRANSFER_LENGTH + 1];
    };
    struct list_node node;
};

struct spi_msg {
    const void *tx;
    void *rx;
    size_t length;
};

static struct list_node spi_masters = LIST_INITIAL_VALUE(spi_masters);
static const uint32_t spi_cpol_cpha[] = {
    [SPI_CPOL0_CPHA0] = ARM_SPI_CPOL0_CPHA0,
    [SPI_CPOL0_CPHA1] = ARM_SPI_CPOL0_CPHA1,
    [SPI_CPOL1_CPHA0] = ARM_SPI_CPOL1_CPHA0,
    [SPI_CPOL1_CPHA1] = ARM_SPI_CPOL1_CPHA1,
};

static int get_max_master_id()
{
    int i;

    for (i = 0; Driver_SPI[i]; i++);

    return i - 1;
}

static struct port_spi_master *get_spi_master(ARM_DRIVER_SPI *drv_spi)
{
    struct port_spi_master *master;

    if (!list_is_empty(&spi_masters)) {
        list_for_every_entry(&spi_masters, master, struct port_spi_master, node) {
            if (master->drv_spi == drv_spi) {
                return master;
            }
        }
    }

    return NULL;
}

static void spi_signal_handler(uint32_t event, void *prv)
{
    struct port_spi_master *master = prv;
    osThreadId_t thread = master->st.thread;

    if (!thread) {
        printf("error thread state\n");
        return;
    }

    master->st.event = event;
    osThreadFlagsSet(thread, SPI_FLAG_COMPLETE);
}

static int init_spi_master(struct port_spi_master *master, ARM_DRIVER_SPI *drv_spi)
{
    int ret;

    ret = drv_spi->SetExtHandler(spi_signal_handler, master);
    if (ret != ARM_DRIVER_OK) {
        printf("init spi master %d failed\n", master->id);
        return ret;
    }

    master->mutex = osMutexNew(NULL);
    if (!master->mutex) {
        printf("create mutex failed for spi master %d\n", master->id);
        ret = -EPIPE;
        goto create_mutex_err;
    }

    master->drv_spi = drv_spi;
    list_add_tail(&spi_masters, &master->node);
    return 0;

create_mutex_err:
    drv_spi->Uninitialize();

    return ret;

}

static int spi_xfer(struct sns_port *port, struct spi_msg *msg)
{
    struct port_spi_master *master = port->master;
    const struct sensor_spi_info *info = &port->binfo->u.spi_info;
    ARM_SPI_STATUS status;
    uint32_t control_code;
    int ret;

    osDisableStandby();
    master->st.thread = osThreadGetId();

    control_code = ARM_SPI_MODE_MASTER;
    master->drv_spi->Control(control_code, info->max_frequency);

    control_code = ARM_SPI_CONTROL_SS | spi_cpol_cpha[info->mode];
    master->drv_spi->Control(control_code, ARM_SPI_SS_ACTIVE);

    master->drv_spi->PowerControl(ARM_POWER_FULL);

    ret = master->drv_spi->Transfer(msg->tx, msg->rx, msg->length);
    if (ret != ARM_DRIVER_OK) {
        printf("spi %d transfer failed:%d\n", master->id, ret);
        goto transfer_err;
    }

    ret = osThreadFlagsWait(SPI_FLAG_COMPLETE, osFlagsWaitAny, SPI_TIMEOUT);
    if (ret < 0) {
        printf("spi %d transfer wait err:%d\n", master->id, ret);
        ret = -ETIMEDOUT;
        goto transfer_err;
    }

    ret = master->drv_spi->GetDataCount();

    status = master->drv_spi->GetStatus();
    /*TODO: add some error handlering here, like retry/print */
    if (status.busy)
        ret = -EBUSY;
    else if (status.data_lost)
        ret = -EIO;
    else
        ret = 0;

transfer_err:
    master->st.thread = NULL;
    master->drv_spi->PowerControl(ARM_POWER_OFF);
    master->drv_spi->Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
    osEnableStandby();

    return ret;
}

static int spi_read(struct sns_port *port, uint8_t reg, void *buff, size_t size)
{
    struct port_spi_master *master = port->master;
    struct spi_msg msg = {
        .tx = master->tx_buffer,
        .rx = master->rx_buffer,
        .length = size + 1,
    };
    int ret;

    if (size > MAX_TRANSFER_LENGTH)
        return -EINVAL;

    osMutexAcquire(master->mutex, osWaitForever);

    master->tx_buffer[0] = reg;
    ret = spi_xfer(port, &msg);
    if (!ret)
        b2ccpy(buff, master->rx_buffer, size, true);

    osMutexRelease(master->mutex);

    return ret;
}

static int spi_write(struct sns_port *port, uint8_t reg, void *buff, size_t size)
{
    struct port_spi_master *master = port->master;
    struct spi_msg msg = {
        .tx = master->tx_buffer,
        .rx = master->rx_buffer,
        .length = size + 1,
    };
    int ret;

    if (size > MAX_TRANSFER_LENGTH)
        return -EINVAL;

    osMutexAcquire(master->mutex, osWaitForever);

    master->tx_buffer[0] = reg;
    c2bcpy(master->tx_buffer, buff, size, true);
    ret = spi_xfer(port, &msg);

    osMutexRelease(master->mutex);

    return ret;
}

static struct port_ops spi_ops = {
    .read = spi_read,
    .write = spi_write,
};

int port_init_spi(const struct sensor_bus_info *info, struct sns_port *port)
{
    struct port_spi_master *master;
    const struct sensor_spi_info *spi_info = &info->u.spi_info;
    ARM_DRIVER_SPI *drv_spi;
    int max_id;
    int ret = 0;

    max_id = get_max_master_id();
    if (max_id < 0 || max_id < spi_info->master_id) {
        printf("no spi master found\n");
        return -ENODEV;
    }

    drv_spi = Driver_SPI[spi_info->master_id];
    /* has this master been already initialized ? */
    if (!(master = get_spi_master(drv_spi))) {
        master = calloc(1, sizeof(*master));
        if (!master) {
            printf("failed to malloc for spi master %d\n", spi_info->master_id);
            return -ENOMEM;
        }

        master->id = spi_info->master_id;
        ret = init_spi_master(master, drv_spi);
        if (ret)
            free(master);
    }

    if (!ret) {
        port->master = master;
        port->ops = &spi_ops;
        port->binfo = info;
    }

    return 0;
}
