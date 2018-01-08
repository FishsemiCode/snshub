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
#include "circ_buffer.h"
#include "sensor_manager.h"
#include "client_manager.h"
#include "utils.h"

/* version 0.1 : 0x00001000 */
/* version 1.1 : 0x00011000 */
/* version 1.21 : 0x00012100 */
#define SNSHUB_VERSION      0x00001000

/* flag for cmgr ready state */
#define CMGR_READY_FLAG     0x00000001

/* flags for cmgr thread */
#define CMGR_EVENT_FLAG     0x00000001

struct obj_node {
    void *obj;
    struct list_node node;
};

struct sensor_client {
    const char *name;
    struct client_callback *cb;
    struct list_node sensors_list;
    struct list_node delay_list;
    struct list_node active_node;
};

struct delay_info {
    struct sensor_t *sensor;
    int delay;
    struct sensor_client *client;
};

struct sensor_record {
    struct sensor_t *sensor;
    struct delay_info *actual_delay;
    struct list_node clients_list;
    struct list_node delay_list;
    struct list_node active_node;
};

static osMutexId_t cmgr_ops_mutex;
static osMutexId_t cmgr_event_mutex;
static osEventFlagsId_t cmgr_flags;
static osThreadId_t cmgr_thread;
static struct circ_buffer *cmgr_buffer;

/* all clients which has at least one active sensor */
static struct list_node active_clients = LIST_INITIAL_VALUE(active_clients);
/* all active sensors */
static struct list_node active_sensors = LIST_INITIAL_VALUE(active_sensors);

static bool cmgr_is_client_active(struct sensor_client *client)
{
    struct sensor_client *tmp;

    list_for_every_entry(&active_clients, tmp, struct sensor_client, active_node) {
        if (tmp == client)
            return true;
    }

    return false;
}

static bool cmgr_add_active_client(struct sensor_client *client)
{
    bool ret;

    ret = cmgr_is_client_active(client);
    if (!ret)
        list_add_tail(&active_clients, &client->active_node);

    return !ret;
}

static struct sensor_record *cmgr_get_active_sensor(struct sensor_t *sensor)
{
    struct sensor_record *rec;

    list_for_every_entry(&active_sensors, rec, struct sensor_record, active_node) {
        if (rec->sensor == sensor)
            return rec;
    }

    return NULL;
}

static inline void cmgr_add_active_sensor(struct sensor_record *rec)
{
    list_add_tail(&active_sensors, &rec->active_node);
}

static inline void cmgr_delete_active_sensor(struct sensor_record *rec)
{
    list_delete(&rec->active_node);
}

static void cmgr_clean_sensor(struct sensor_record *rec)
{
    /*XXX: TODO need to clean all the clients in list, and remove the sensor in all the clients' list;
     * and also need to remove all the delay info*/
    list_delete(&rec->active_node);
    free(rec);
}

static bool sensor_has_client(struct sensor_record *rec, struct sensor_client *client)
{
    struct obj_node *obj;
    bool ret = false;

    list_for_every_entry(&rec->clients_list, obj, struct obj_node, node) {
        if (obj->obj == client) {
            ret = true;
            break;
        }
    }

    return ret;
}

static int sensor_list_delete_obj(struct list_node *list, void *obj)
{
    struct obj_node *obj_node, *tmp;

    list_for_every_entry_safe(list, obj_node, tmp, struct obj_node, node) {
        if (obj_node->obj == obj) {
            list_delete(&obj_node->node);
            free(obj_node);

            return -ENOENT;
        }
    }

    return 0;
}

static bool sensor_add_client(struct sensor_record *rec, struct sensor_client *client)
{
    bool ret;
    struct obj_node *obj;

    ret = sensor_has_client(rec, client);
    if (!ret) {
        obj = malloc(sizeof(*obj));
        if (!obj) {
            printf("failed to malloc for client obj\n");
            return false;
        }

        obj->obj = client;
        list_add_tail(&rec->clients_list, &obj->node);
    }

    return !ret;
}

static int sensor_delete_client(struct sensor_record *rec, struct sensor_client *client)
{
    struct delay_info *info;
    struct obj_node *obj, *tmp;

    if (!sensor_has_client(rec, client)) {
        printf("sensor %s has no client %s\n", rec->sensor->name, client->name);
        return -ENOENT;
    }

    sensor_list_delete_obj(&rec->clients_list, client);

    /* when delete a client of a sensor, we should also delete the delay info for this
     * client, and free this delay info struct */
    list_for_every_entry_safe(&rec->delay_list, obj, tmp, struct obj_node, node) {
        info = obj->obj;
        if (info->client == client) {
            if (rec->actual_delay == info)
                rec->actual_delay = NULL;

            list_delete(&obj->node);
            free(obj);
        }
    }

    return 0;
}

static inline bool sensor_has_no_client(struct sensor_record *rec) {
    return list_is_empty(&rec->clients_list);
}

static struct delay_info *sensor_get_delay(struct sensor_record *rec, struct sensor_client *client)
{
    struct obj_node *obj;
    struct delay_info *info;

    list_for_every_entry(&rec->delay_list, obj, struct obj_node, node) {
        info = obj->obj;
        if (info->client == client)
            return info;
    }

    return NULL;
}

static int sensor_add_delay(struct sensor_record *rec, struct delay_info *info)
{
    struct obj_node *obj;
    int ret = 0;

    if (!sensor_get_delay(rec, info->client)) {
        obj = malloc(sizeof(*obj));
        if (!obj) {
            printf("failed to malloc obj_node for delay\n");
            ret = -ENOMEM;
        }

        obj->obj = info;
        list_add_tail(&rec->delay_list, &obj->node);
    } else {
        /* the info of this sensor for this client is already in the list,
         * just modify the delay value, need to reset the actual delay_info to force a new selection */
        rec->actual_delay = NULL;
    }

    return ret;
}

static bool sensor_select_delay(struct sensor_record *rec)
{
    struct delay_info *info;
    struct obj_node *obj;
    struct delay_info *actual = rec->actual_delay;
    bool change = false;

    if (list_is_empty(&rec->delay_list))
        return false;

    list_for_every_entry(&rec->delay_list, obj, struct obj_node, node) {
        info = obj->obj;
        if (!actual || (info->delay < actual->delay))
            actual = info;
    }

    if (!rec->actual_delay || actual->delay != rec->actual_delay->delay)
        change = true;

    rec->actual_delay = actual;

    return change;
}

static struct delay_info *client_get_delay(struct sensor_client *client, struct sensor_t *sensor)
{
    struct obj_node *obj;
    struct delay_info *delay;

    list_for_every_entry(&client->delay_list, obj, struct obj_node, node) {
        delay = obj->obj;
        if (delay->sensor == sensor)
            return delay;
    }

    return NULL;
}

/* return pointer to the delay_info on successs, otherwise return a negative error no
 * An assumption: the memory address is lower than 0x80000000 */
static struct delay_info *client_set_delay(struct sensor_client *client, struct sensor_t *sensor, uint32_t delay)
{
    struct obj_node *obj;
    struct delay_info *info;
    int ret = -ENOMEM;

    info = client_get_delay(client, sensor);
    if (!info) {
        info = calloc(1, sizeof(struct delay_info));
        if (!info) {
            printf("failed to malloc for delay_info\n");
            goto delay_info_err;
        }

        info->sensor = sensor;
        info->client = client;

        obj = malloc(sizeof(struct obj_node));
        if (!obj) {
            printf("failed to malloc for obj_node\n");
            goto obj_node_err;
        }

        obj->obj = info;
        list_add_tail(&client->delay_list, &obj->node);
    }

    info->delay = MIN(sensor->max_delay, MAX(delay, sensor->min_delay));
    return info;

obj_node_err:
    free(info);
delay_info_err:
    return (struct delay_info *)ret;
}

static int client_delete_delay(struct sensor_client *client, struct sensor_t *sensor)
{
    struct delay_info *info;

    info = client_get_delay(client, sensor);
    if (!info)
        return -EINVAL;

    sensor_list_delete_obj(&client->delay_list, info);
    free(info);

    return 0;
}

static bool client_has_sensor(struct sensor_client *client, struct sensor_record *rec)
{
    struct obj_node *obj;
    bool ret = false;

    list_for_every_entry(&client->sensors_list, obj, struct obj_node, node) {
        if (obj->obj == rec) {
            ret = true;
            break;
        }
    }

    return ret;
}

static bool client_add_sensor(struct sensor_client *client, struct sensor_record *rec)
{
    bool ret;
    struct obj_node *obj;

    ret = client_has_sensor(client, rec);
    if (!ret) {
        obj = malloc(sizeof(*obj));
        if (!obj) {
            printf("failed to malloc for record obj\n");
            return false;
        }

        obj->obj = rec;
        list_add_tail(&client->sensors_list, &obj->node);
    }

    return !ret;
}

static int client_delete_sensor(struct sensor_client *client, struct sensor_record *rec)
{
    if (!client_has_sensor(client, rec)) {
        printf("client has no sensor\n");
        return -ENOENT;
    }

    sensor_list_delete_obj(&client->sensors_list, rec);
    client_delete_delay(client, rec->sensor);

    return 0;
}

static void cmgr_init_sensor_record(struct sensor_record *rec, struct sensor_t *sensor, struct sensor_client *client)
{
    rec->sensor = sensor;
    rec->actual_delay = NULL;
    list_initialize(&rec->clients_list);
    list_initialize(&rec->delay_list);

    sensor_add_client(rec, client);
}

int cmgr_set_delay(struct sensor_client *client, struct sensor_t *sensor, uint32_t delay)
{
    struct sensor_record *rec;
    struct delay_info *info = NULL;
    int ret = 0;

    osMutexAcquire(cmgr_ops_mutex, osWaitForever);

    info = client_set_delay(client, sensor, delay);
    if ((int)info < 0) {
        /* failed to set delay inside the client world */
        ret = (int)info;
        goto client_delay_err;
    }

    /* the sensor has not been activated, so we can not do the hardware operations on this sensor even for ODR changes */
    if (!(rec = cmgr_get_active_sensor(sensor))) {
        goto do_nothing;
    }

    sensor_add_delay(rec, info);

    if (client_add_sensor(client, rec))
        cmgr_add_active_client(client);

    if (sensor_select_delay(rec))
        smgr_set_delay(sensor->handle, rec->actual_delay->delay);

do_nothing:
client_delay_err:
    osMutexRelease(cmgr_ops_mutex);
    return ret;
}

static int cmgr_enable_sensor(struct sensor_client *client, struct sensor_t *sensor)
{
    struct sensor_record *rec;
    struct delay_info *info;
    bool hardware_op = false;
    bool sync_delay = false;
    int ret = 0;

    osMutexAcquire(cmgr_ops_mutex, osWaitForever);

    if (!(rec = cmgr_get_active_sensor(sensor))) {
        rec = calloc(1, sizeof(*rec));
        if (!rec) {
            osMutexRelease(cmgr_ops_mutex);
            return -ENOMEM;
        }

        cmgr_init_sensor_record(rec, sensor, client);
        cmgr_add_active_sensor(rec);
        hardware_op = true;
    } else {
        if (sensor_add_client(rec, client)) {
            if (smgr_is_sensor_onchange(sensor)) {
                /*TODO:
                 * 1. get the last event
                 * 2. send the last the event to the new client
                 * */
            }
        } else {
            printf("the client %s already in sensor %s\n", client->name, rec->sensor->name);
        }
    }

    if (client_add_sensor(client, rec)) {
        /* this client has not enabled this sensor before */
        cmgr_add_active_client(client);

        /* if the set_delay is called before activate */
        info = client_get_delay(client, sensor);
        if (info) {
            sensor_add_delay(rec, info);
            sync_delay = true;
        }
    }

    if (hardware_op) {
        ret = smgr_activate(sensor->handle, true);
        if (ret) {
            cmgr_clean_sensor(rec);
        }
    }

    if (sync_delay && sensor_select_delay(rec))
        smgr_set_delay(sensor->handle, rec->actual_delay->delay);

    osMutexRelease(cmgr_ops_mutex);

    return ret;
}

static int cmgr_disable_sensor(struct sensor_client *client, struct sensor_t *sensor)
{
    struct sensor_record *rec;
    int ret = 0;

    osMutexAcquire(cmgr_ops_mutex, osWaitForever);

    rec = cmgr_get_active_sensor(sensor);
    if (!rec) {
        printf("failed to disable sensor: no such active sensor\n");
        osMutexRelease(cmgr_ops_mutex);
        return -EINVAL;
    }

    sensor_delete_client(rec, client);
    client_delete_sensor(client, rec);

    if (sensor_has_no_client(rec)) {
        ret = smgr_activate(rec->sensor->handle, false);
        cmgr_delete_active_sensor(rec);
        free(rec);
    } else if (sensor_select_delay(rec))
        ret = smgr_set_delay(rec->sensor->handle, rec->actual_delay->delay);

    osMutexRelease(cmgr_ops_mutex);

    return ret;
}

struct sensor_client *cmgr_client_request(const char *name, struct client_callback *cb)
{
    struct sensor_client *client;

    client = calloc(1, sizeof(*client));
    if (!client) {
        printf("no memory for new client\n");
        return NULL;
    }

    client->name = name;
    client->cb = cb;
    list_initialize(&client->sensors_list);
    list_initialize(&client->delay_list);
    list_initialize(&client->active_node);

    return client;
}

int cmgr_client_release(struct sensor_client *client)
{
    /*XXX: TODO:
     * if there is/are sensors not disabled in this client,
     * do we need to disable all these sensors automatically ??
     * */

    free(client);

    return 0;
}

int cmgr_get_num_of_sensors()
{
    struct sensor_t *sensor_list;

    return smgr_get_sensor_list(&sensor_list);
}

int cmgr_get_all_handles(int *handles)
{
    struct sensor_t *sensor_list;
    int num, i;

    num = smgr_get_sensor_list(&sensor_list);
    for (i = 0; i < num; i++) {
        handles[i] = sensor_list[i].handle;
    }

    return 0;
}

/* return the fisrt sensor matching the type */
struct sensor_t *cmgr_get_sensor_by_type(int type)
{
    struct sensor_t *sensor_list;
    int num, i;

    num = smgr_get_sensor_list(&sensor_list);
    for (i = 0; i < num; i++) {
        if (sensor_list[i].type == type)
            return &sensor_list[i];
    }

    return NULL;
}

struct sensor_t *cmgr_get_sensor_by_handle(int handle)
{
    struct sensor_t *sensor_list;
    int num, i;

    num = smgr_get_sensor_list(&sensor_list);
    for (i = 0; i < num; i++) {
        if (sensor_list[i].handle == handle)
            return &sensor_list[i];
    }

    return NULL;
}

int cmgr_activate_sensor(struct sensor_client *client, struct sensor_t *sensor, bool enable)
{
    int ret;

    if (enable)
        ret = cmgr_enable_sensor(client, sensor);
    else
        ret = cmgr_disable_sensor(client, sensor);

    return ret;
}

int cmgr_activate_sensor_one(struct sensor_client *client, struct sensor_t *sensor, uint32_t delay, bool enable)
{
    int ret;

    if (enable && !sensor)
        return -EINVAL;

    if (enable) {
        ret = cmgr_set_delay(client, sensor, delay);
        if (ret) {
            goto set_delay_err;
        }
        ret = cmgr_enable_sensor(client, sensor);
    } else {
        ret = cmgr_disable_sensor(client, sensor);
    }

set_delay_err:

    return ret;
}

uint32_t cmgr_get_version()
{
    return SNSHUB_VERSION;
}

bool cmgr_system_ready()
{
    if (osKernelGetState() != osKernelRunning)
        return false;

    osEventFlagsWait(cmgr_flags, CMGR_READY_FLAG, osFlagsNoClear, osWaitForever);

    return true;
}

static int cmgr_push_event_buffer(struct sensor_event *data, int num)
{
    int ret = 0;

    /*XXX: need to protect with mutex between possible multi drivers ?*/
    osMutexAcquire(cmgr_event_mutex, osWaitForever);

    ret = circ_buffer_push(cmgr_buffer, data, num);
    if (!ret)
        osThreadFlagsSet(cmgr_thread, CMGR_EVENT_FLAG);

    osMutexRelease(cmgr_event_mutex);

    return ret;
}

static int cmgr_send_event_to_client(struct sensor_event *events, size_t cnt)
{
    struct sensor_record *rec;
    struct sensor_client *client;
    struct obj_node *obj;
    size_t i;

    osMutexAcquire(cmgr_ops_mutex, osWaitForever);
    for (i = 0; i < cnt; i++) {
        rec = cmgr_get_active_sensor(events[i].sensor);
        if (!rec) {
            printf("send event: failed to find sensor record for %s\n",
                    events[i].sensor->name);
            continue;
        }

        /*TODO: need to optimize for continus events from the same sensor */
        list_for_every_entry(&rec->clients_list, obj, struct obj_node, node) {
            client = obj->obj;
            if (client->cb && client->cb->event_update) {
                client->cb->event_update(&events[i], 1);
            }
        }
    }
    osMutexRelease(cmgr_ops_mutex);

    return 0;
}

static int cmgr_handle_event_buffer()
{
    return circ_buffer_for_each(cmgr_buffer, (buffer_handler)smgr_handle_event);
}

static void cmgr_thread_func(void *arg)
{
    int ret;
    uint32_t flags;

    ret = smgr_init(cmgr_push_event_buffer, cmgr_send_event_to_client);
    if (ret) {
        printf("fatal error: sensor manager init failed:ret=%d\n", ret);
        /* retrun from a thread body will assert an error and cause panic */
        return;
    }

    osEventFlagsSet(cmgr_flags, CMGR_READY_FLAG);

    /* the main loop of the client manager:
     * 1. wait for the event from the hardware sensor driver
     * 2. call the new event generator in smgr for derived sensors
     * 3. call the callbacks of all the active clients
     * */
    while (1) {
        flags = osThreadFlagsWait(CMGR_EVENT_FLAG, osFlagsWaitAny, osWaitForever);
        if (flags & CMGR_EVENT_FLAG)
            cmgr_handle_event_buffer();
    }
}

static osStatus_t cmgr_init()
{
    int ret = 0;
    osThreadAttr_t thread_attr = {};

    /* initialize the circular event buffer */
    ret = circ_buffer_init(&cmgr_buffer, "cmgr event", sizeof(struct sensor_event), 128);
    if (ret) {
        printf("event buffer init failed:%d\n", ret);
        return osError;
    }

    cmgr_ops_mutex = osMutexNew(NULL);
    if (!cmgr_ops_mutex) {
        printf("create mutex failed for client manager\n");
        ret = osError;
        goto mutex_err;
    }

    cmgr_event_mutex = osMutexNew(NULL);
    if (!cmgr_ops_mutex) {
        printf("create event mutex failed for client manager\n");
        ret = osError;
        goto event_mutex_err;
    }

    cmgr_flags = osEventFlagsNew(NULL);
    if (!cmgr_flags) {
        ret = osError;
        goto flag_err;
    }

    thread_attr.name = "cmgr_thread";
    thread_attr.priority = osPriorityHigh;
    cmgr_thread = osThreadNew(cmgr_thread_func, NULL, &thread_attr);
    if (!cmgr_thread) {
        ret = osErrorNoMemory;
        goto thread_err;
    }

    return ret;

thread_err:
    osEventFlagsDelete(cmgr_flags);
    cmgr_flags = NULL;
flag_err:
    osMutexDelete(cmgr_event_mutex);
    cmgr_event_mutex = NULL;
event_mutex_err:
    osMutexDelete(cmgr_ops_mutex);
    cmgr_ops_mutex = NULL;
mutex_err:
    circ_buffer_deinit(cmgr_buffer);

    return ret;
}

int cmgr_deinit()
{
    osMutexDelete(cmgr_ops_mutex);
    osMutexDelete(cmgr_event_mutex);
    osThreadTerminate(cmgr_thread);
    circ_buffer_deinit(cmgr_buffer);

    return 0;
}

osInitDef(cmgr_init, SH_INIT_CMGR);
