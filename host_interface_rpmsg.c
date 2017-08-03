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

#include <cmsis_os2.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <openamp/open_amp.h>
#include "host_interface.h"

#define SNSHUB_CHANNEL_NAME     "rpmsg-snshub"
static struct rpmsg_channel *hi_rpmsg_channel;
static hi_received_cb hi_rpmsg_rec_cb;
static osSemaphoreId_t channel_sem;

static void hi_rpmsg_device_created(struct remote_device *rdev, void *priv)
{
    rpmsg_create_channel(rdev, SNSHUB_CHANNEL_NAME);
}

static void hi_rpmsg_channel_created(struct rpmsg_channel *channel)
{
    printf("%s channel created\n", SNSHUB_CHANNEL_NAME);
    hi_rpmsg_channel = channel;
    if (channel_sem)
        osSemaphoreRelease(channel_sem);
}

static void hi_rpmsg_channel_destroyed(struct rpmsg_channel *channel)
{
    hi_rpmsg_rec_cb = NULL;
}

static void hi_rpmsg_channel_received(struct rpmsg_channel *channel, void *data,
                       int len, void *priv, unsigned long src)
{
    if (hi_rpmsg_rec_cb)
        hi_rpmsg_rec_cb(data, len);
}

static int hi_rpmsg_register_received(hi_received_cb cb)
{
    int ret = 0;
    osSemaphoreAttr_t sem_attr = {};

    if (!cb || hi_rpmsg_rec_cb)
        return -EINVAL;

    if (osKernelGetState() != osKernelRunning)
        return -EBUSY;

    sem_attr.name = "sns_rpmsg_sem";
    channel_sem = osSemaphoreNew(1, 0, &sem_attr);
    if (!channel_sem) {
        printf("create event semaphore failed for rpmsg channel\n");
        return -ENOENT;
    }

    ret = rpmsg_register_callback(  SNSHUB_CHANNEL_NAME,
                                    NULL,
                                    hi_rpmsg_device_created,
                                    NULL,
                                    hi_rpmsg_channel_created,
                                    hi_rpmsg_channel_destroyed,
                                    hi_rpmsg_channel_received);
    if (ret)
        goto register_err;

    osSemaphoreAcquire(channel_sem, osWaitForever);

    hi_rpmsg_rec_cb = cb;

register_err:
    osSemaphoreDelete(channel_sem);
    return ret;
}

int hi_rpmsg_send(void *data, size_t len)
{
    return rpmsg_send(hi_rpmsg_channel, data, len);
}

int hi_rpmsg_get_max_send_size()
{
    if (!hi_rpmsg_channel)
        return -EINVAL;

    return rpmsg_get_buffer_size(hi_rpmsg_channel);
}

const struct host_interface interface_rpmsg = {
    .register_received = hi_rpmsg_register_received,
    .send = hi_rpmsg_send,
    .get_max_send_size = hi_rpmsg_get_max_send_size,
};
