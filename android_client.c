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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "utils.h"
#include "circ_buffer.h"
#include "client_manager.h"
#include "host_interface.h"

#define ANDRD_CLNT_INTERFACE    HOST_INTERFACE_RPMSG

#define ANDRD_CLNT_NAME     "android client"

#define SNS_FLAG_CMD_RECV       0x0001
#define SNS_FLAG_NEW_EVENT      0x0002

typedef enum {
    SNS_CMD_GET_VERSION = 1,
    SNS_CMD_GET_NUM,
    SNS_CMD_GET_LIST,
    SNS_CMD_GET_INFO,
    SNS_CMD_ENABLE,
    SNS_CMD_SET_DELAY,
    SNS_CMD_SYNC_TIME,

    /* send from sensor hub to AP */
    SNS_CMD_SNS_EVENT = 0x20,
}sns_commands;

struct sns_hdr {
    uint32_t command;
    int32_t result;
    uint64_t cookie;
};

struct sns_get_version {
    struct sns_hdr hdr;
    uint32_t ver;
};

struct sns_get_number {
    struct sns_hdr hdr;
    uint32_t num;
};

struct sns_get_list {
    struct sns_hdr hdr;
    int seq;
    uint32_t num;
    int handles[];
};

#define STRING_INFO_LEN     20
struct sensor_info {
    int type;
    int handle;
    int min_delay;
    int max_delay;
    int resolution_mod;
    uint32_t flags;
    int fifo_max_event_count;
    int fifo_reserved_event_count;

    char vendor[STRING_INFO_LEN];
    char module[STRING_INFO_LEN];

    /* stringlised floating data */
    char max_range[STRING_INFO_LEN];
    char power[STRING_INFO_LEN];
    char resolution[STRING_INFO_LEN];
};

struct sns_get_info {
    struct sns_hdr hdr;
    int handle;
    int reserved;
    struct sensor_info sns_info;
};

struct sns_enable {
    struct sns_hdr hdr;
    int handle;
    int enable;
};

struct sns_set_delay {
    struct sns_hdr hdr;
    int handle;
    uint32_t delay;
};

struct sns_sync_time {
    struct sns_hdr hdr;
    uint32_t time;
};

/* TODO:
struct sns_set_batch {
};

struct sns_flush {
};
 * */

union sns_cmd_package {
    struct sns_get_version get_version;
    struct sns_get_list get_list;
    struct sns_get_info get_info;
    struct sns_enable enable;
    struct sns_set_delay set_delay;
    struct sns_sync_time sync_time;
};

/* mind the layout of this struct, it must be partly the same with sensor_event */
struct aclient_sns_event {
    int64_t timestamp;
    int8_t status;
    uint8_t reserved1[3];
    int32_t data[3];
    int32_t handle;
    uint8_t reserved2[4];
};

struct sns_send_pkg {
    struct sns_hdr hdr;
    uint32_t num;
    uint8_t reversed[4];
    struct aclient_sns_event events[];
};

static struct sensor_client *aclient;
static osThreadId_t aclient_thread;
static struct circ_buffer *aclient_ebuffer;
static struct circ_buffer *aclient_cbuffer;

static int aclient_get_version_handler(void *data, size_t len)
{
    struct sns_get_version *cmd = data;

    if (!cmd->hdr.cookie)
        return -EBADMSG;

    cmd->ver = cmgr_get_version();
    cmd->hdr.result = 0;

    return hi_send(ANDRD_CLNT_INTERFACE, cmd, sizeof(*cmd));
}

static int aclient_get_number_handler(void *data, size_t len)
{
    struct sns_get_number *cmd = data;
    int num;

    if (!cmd->hdr.cookie)
        return -EBADMSG;

    num = cmgr_get_num_of_sensors();
    cmd->hdr.result = num <= 0 ? -ENOENT : 0;
    cmd->num = num;

    return hi_send(ANDRD_CLNT_INTERFACE, cmd, sizeof(*cmd));
}

/* XXX: need to optimize for the current assumption:
 * the caller never know the interface detail, so it should request the send buffer
 * from the interface, and free it after send */
static int aclient_get_list_handler(void *data, size_t len)
{
    struct sns_get_list *cmd = data;
    struct sns_get_list *rsp;
    int num, rsp_size;
    int ret;

    if (!cmd->hdr.cookie)
        return -EBADMSG;

    num = cmgr_get_num_of_sensors();
    if (num <= 0) {
        cmd->hdr.result = -ENOENT;
        return hi_send(ANDRD_CLNT_INTERFACE, cmd, sizeof(*cmd));
    }

    rsp_size = sizeof(*rsp) + sizeof(int) * num;
    rsp = malloc(rsp_size);
    memcpy(rsp, cmd, sizeof(*rsp));

    rsp->hdr.result = 0;
    rsp->num = num;
    cmgr_get_all_handles(rsp->handles);

    ret = hi_send(ANDRD_CLNT_INTERFACE, rsp, rsp_size);
    free(rsp);

    return ret;
}

static int aclient_fill_sensor_info(int handle, struct sensor_info *info)
{
    struct sensor_t *sensor;
    int ret;

    sensor = cmgr_get_sensor_by_handle(handle);
    if (!sensor)
        return -EINVAL;

    info->type = sensor->type;
    info->handle = handle;
    info->min_delay = sensor->min_delay;
    info->max_delay = sensor->max_delay;
    info->resolution_mod = (int)(sensor->resolution * 1000000);
    info->flags = sensor->flags;
    info->fifo_max_event_count = sensor->fifo_max_event_count;
    info->fifo_reserved_event_count = sensor->fifo_reserved_event_count;

    strncpy(info->vendor, sensor->vendor, STRING_INFO_LEN);
    strncpy(info->module, sensor->module, STRING_INFO_LEN);

    ret = snprintf(info->max_range, STRING_INFO_LEN, "%f", sensor->max_range);
    if(ret >= STRING_INFO_LEN)
        printf("warn: snprintf for %s max_range truncated\n", sensor->name);

    ret = snprintf(info->power, STRING_INFO_LEN, "%f", sensor->power);
    if(ret >= STRING_INFO_LEN)
        printf("warn: snprintf for %s power truncated\n", sensor->name);

    ret = snprintf(info->resolution, STRING_INFO_LEN, "%f", sensor->resolution);
    if(ret >= STRING_INFO_LEN)
        printf("warn: snprintf for %s resolution truncated\n", sensor->name);

    return 0;
}

static int aclient_get_info_handler(void *data, size_t len)
{
    struct sns_get_info *cmd = data;
    int ret;

    if (!cmd->hdr.cookie)
        return -EBADMSG;

    ret = aclient_fill_sensor_info(cmd->handle, &cmd->sns_info);
    cmd->hdr.result = ret;
    return hi_send(ANDRD_CLNT_INTERFACE, cmd, sizeof(*cmd));
}

static int aclient_enable_handler(void *data, size_t len)
{
    struct sns_enable *cmd = data;
    struct sensor_t *sensor;
    int ret;

    sensor = cmgr_get_sensor_by_handle(cmd->handle);
    if (!sensor) {
        cmd->hdr.result = -ENOENT;
        goto sensor_err;
    }

    ret = cmgr_activate_sensor(aclient, sensor, !!cmd->enable);
    cmd->hdr.result = ret;

sensor_err:
    ret = 0;
    if (cmd->hdr.cookie)
        ret = hi_send(ANDRD_CLNT_INTERFACE, cmd, sizeof(*cmd));

    return ret;
}

static int aclient_set_delay_handler(void *data, size_t len)
{
    struct sns_set_delay *cmd = data;
    struct sensor_t *sensor;
    int ret;

    sensor = cmgr_get_sensor_by_handle(cmd->handle);
    if (!sensor) {
        cmd->hdr.result = -ENOENT;
        goto sensor_err;
    }

    ret = cmgr_set_delay(aclient, sensor, cmd->delay);
    cmd->hdr.result = ret;

sensor_err:
    ret = 0;
    if (cmd->hdr.cookie)
        ret = hi_send(ANDRD_CLNT_INTERFACE, cmd, sizeof(*cmd));

    return ret;
}

static int aclient_sync_time_handler(void *data, size_t len)
{
    return 0;
}

static const hi_received_cb hi_handlers[] = {
    [SNS_CMD_GET_VERSION] = aclient_get_version_handler,
    [SNS_CMD_GET_NUM] = aclient_get_number_handler,
    [SNS_CMD_GET_LIST] = aclient_get_list_handler,
    [SNS_CMD_GET_INFO] = aclient_get_info_handler,
    [SNS_CMD_ENABLE] = aclient_enable_handler,
    [SNS_CMD_SET_DELAY] = aclient_set_delay_handler,
    [SNS_CMD_SYNC_TIME] = aclient_sync_time_handler,
};

/* the callback to handle the command from the host side, run in
 * the host-interface(such as rpmsg-remote_proc) thread context
 * we need to transfer the actual handling from the context of
 * the host-interface to the one of the client thread
 * */
static int aclient_hi_received_cb(void *data, size_t len)
{
    int ret;

    /* TODO: need to use a cmd fifo to buffer the commands? */
    if (len > sizeof(union sns_cmd_package)) {
        printf("fatal error: invalid length=%d\n", len);
        return -EINVAL;
    }

    ret = circ_buffer_push(aclient_cbuffer, data, 1);
    if (ret) {
        printf("failed to buffer new command:%d\n", ret);
        return ret;
    }

    osThreadFlagsSet(aclient_thread, SNS_FLAG_CMD_RECV);

    return 0;
}

static int aclient_cmd_handler(void *data)
{
    struct sns_hdr *cmd = data;
    int ret = -EINVAL;

    if (cmd->command < __countof(hi_handlers) && hi_handlers[cmd->command])
        ret = hi_handlers[cmd->command](cmd, sizeof(union sns_cmd_package));

    return ret;
}

static inline int aclient_handle_cmds()
{
    return circ_buffer_for_each(aclient_cbuffer, aclient_cmd_handler);
}

/* the callback to handle the sensor events, in client_manager thread context */
static void aclient_sevent_cb(struct sensor_event *event, size_t num)
{
    int err;

    err = circ_buffer_push(aclient_ebuffer, event, num);
    if (err) {
        printf("failed to push event to aclient buffer\n");
    } else {
        osThreadFlagsSet(aclient_thread, SNS_FLAG_NEW_EVENT);
    }
}

/* the callback to handle the sensor accuracy change */
static void aclient_saccuracy_cb(struct sensor_t *sensor, int accuracy)
{

}

static struct client_callback aclient_sns_cb = {
    .event_update = aclient_sevent_cb,
    .accuracy_changed = aclient_saccuracy_cb,
};

static int aclient_handle_event(struct sns_send_pkg *event_pkg, int max_num)
{
    int real_num, i;
    struct sensor_event events[max_num];

    do {
        /* TODO: optimize: add a new "each_event(.., max_num)" to create the final event directly */
        real_num = circ_buffer_pop(aclient_ebuffer, events, max_num);
        if (real_num > 0) {
            event_pkg->num = real_num;
            for (i = 0; i < real_num; i++) {
                memcpy(&event_pkg->events[i], &events[i].timestamp, 24);
                event_pkg->events[i].handle = events[i].sensor->handle;
                memset(event_pkg->events[i].reserved2, 0, 4);
            }

            hi_send(ANDRD_CLNT_INTERFACE, event_pkg,
                    sizeof(*event_pkg) + sizeof(struct aclient_sns_event) * real_num);
        }
    } while (real_num == max_num);

    return 0;
}

static void aclient_thread_func(void *arg)
{
    int max_size;
    struct sns_send_pkg *event_pkg;
    int max_num;
    uint32_t sns_flags = 0;

    /* need to wait the client manager and sensor manager being ready */
    while(!cmgr_system_ready());

    /* XXX: this should be called later after the smgr & cmgr ready */
    if (hi_register_received(ANDRD_CLNT_INTERFACE, aclient_hi_received_cb)) {
        printf("failed to register received for android client\n");
        return;
    }
    printf("android client interface registered\n");

    max_size = hi_get_max_send_size(ANDRD_CLNT_INTERFACE);
    if (max_size <= 0) {
        printf("fatal error of max send size, %d\n", max_size);
        return;
    }

    event_pkg = malloc(max_size);

    max_num = max_size - sizeof(struct sns_send_pkg);
    max_num /= sizeof(struct aclient_sns_event);
    event_pkg->hdr.command = SNS_CMD_SNS_EVENT;
    event_pkg->hdr.cookie = 0;

    osThreadFlagsClear(SNS_FLAG_CMD_RECV | SNS_FLAG_NEW_EVENT);

    while (1) {
        sns_flags = osThreadFlagsWait(SNS_FLAG_CMD_RECV | SNS_FLAG_NEW_EVENT, osFlagsWaitAny, osWaitForever);

        if (sns_flags & SNS_FLAG_NEW_EVENT) {
            aclient_handle_event(event_pkg, max_num);
        }

        if (sns_flags & SNS_FLAG_CMD_RECV) {
            aclient_handle_cmds();
        }
    }
}

static osStatus_t aclient_init()
{
    int ret = osOK;
    osThreadAttr_t thread_attr = {};

    aclient = cmgr_client_request(ANDRD_CLNT_NAME, &aclient_sns_cb);
    if (!aclient) {
        printf("failed to request sensor client\n");
        return osErrorResource;
    }

    /* initialize the circular event buffer */
    ret = circ_buffer_init(&aclient_ebuffer, "aclient event", sizeof(struct sensor_event), 32);
    if (ret) {
        printf("aclient event buffer init failed:%d\n", ret);
        ret = osError;
        goto ebuffer_err;
    }

    /* initialize the circular command buffer */
    ret = circ_buffer_init(&aclient_cbuffer, "aclient command", sizeof(union sns_cmd_package), 8);
    if (ret) {
        printf("aclient command buffer init failed:%d\n", ret);
        ret = osError;
        goto cbuffer_err;
    }

    thread_attr.name = "aclient_thread";
    /* the priority of the clients must be lower than the client manager */
    thread_attr.priority = osPriorityNormal;
    aclient_thread = osThreadNew(aclient_thread_func, NULL, &thread_attr);
    if (!aclient_thread) {
        ret = osErrorNoMemory;
        goto thread_err;
    }
    return osOK;

thread_err:
    circ_buffer_deinit(aclient_cbuffer);
cbuffer_err:
    circ_buffer_deinit(aclient_ebuffer);
ebuffer_err:
    cmgr_client_release(aclient);

    return ret;
}

osInitDef(aclient_init, SH_INIT_APP);
