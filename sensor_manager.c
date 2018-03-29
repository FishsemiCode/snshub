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
#include <assert.h>
#include "sensor.h"
#include "sensor_driver.h"
#include "hw_sensors.h"
#include "sensor_manager.h"
#include "algo_manager.h"
#include "utils.h"

#define SMGR_MAX_SENSORS    20
#define VENDOR_PINECONE     "Pinecone"
#define SMGR_MAX_DEPS       6

static cmgr_push_event push_to_cmgr;
static cmgr_handle_dispatch dispatch_cb;
static struct sensor_t sensor_list[SMGR_MAX_SENSORS];
static struct sensor_ctx sensor_context[SMGR_MAX_SENSORS];
static int smgr_sensor_num;

static osMessageQueueId_t smgr_workqueue[PRI_NUM];

struct smgr_work {
    smgr_work_func func;
    void *data;
    int64_t ts;
};

struct virtual_sensor_desc {
    char *name;
    char *vendor;
    int type;
    /* the sensor types that this virtual sensor depends, must end with -1 */
    int deps[SMGR_MAX_DEPS];
};

/* if sensor_A depends on sensor_B, try to place sensor_A after sensor_B */
static const struct virtual_sensor_desc possible_vsensors[] = {
    /* calibrated magnetic sensor
     * depends:
     *      uncalibrated magnetic sensor
     * */
    {
        "magnetic sensor",
        VENDOR_PINECONE,
        SENSOR_TYPE_MAGNETIC_FIELD,
        {SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED, -1},
    },

    /* calibrated gyro sensor
     * depends:
     *      uncalibrated gyro sensor
     * */
    {
        "gyroscope sensor",
        VENDOR_PINECONE,
        SENSOR_TYPE_GYROSCOPE,
        {SENSOR_TYPE_GYROSCOPE_UNCALIBRATED, -1},
    },

    /* orientation sensor
     * depends:
     *      accelerometer sensor
     *      calibrated magnetic sensor
     * */
    {
        "orientation sensor",
        VENDOR_PINECONE,
        SENSOR_TYPE_GYROSCOPE,
        {SENSOR_TYPE_ACCELEROMETER, SENSOR_TYPE_MAGNETIC_FIELD, -1},
        /*XXX: select from AMG and AM, but both is from the fusion algo */
    },
};

static struct sensor_ctx *get_ctx_by_handle(int handle)
{
    int i;

    for (i = 0; i < smgr_sensor_num; i++) {
        if (sensor_context[i].sensor->handle == handle)
            return &sensor_context[i];
    }

    return NULL;
}

static struct sensor_ctx *get_ctx_by_type(int type)
{
    int i;

    for (i = 0; i < smgr_sensor_num; i++) {
        if (sensor_context[i].sensor->type == type)
            return &sensor_context[i];
    }

    return NULL;
}

static bool ctx_in_list(struct list_node *list, struct sensor_ctx *ctx)
{
    struct ctx_list_node *ctx_node;

    list_for_every_entry(list, ctx_node, struct ctx_list_node, node) {
        if (ctx_node->ctx == ctx)
            return true;
    }

    return false;
}

static struct ctx_list_node *get_ctx_node(struct list_node *list,
                                            struct sensor_ctx *ctx)
{
    struct ctx_list_node *ctx_node;

    list_for_every_entry(list, ctx_node, struct ctx_list_node, node) {
        if (ctx_node->ctx == ctx)
            return ctx_node;
    }

    return NULL;
}

static int add_dependency_sensor(struct sensor_ctx *ctx, struct sensor_ctx *dep)
{
    struct ctx_list_node *node;

    if (ctx_in_list(&ctx->dependency, dep))
        return 0;

    node = malloc(sizeof(*node));
    if (!node) {
        printf("%s->%s alloc memory failed to add dependency\n",
                ctx->sensor->name, dep->sensor->name);
        return -ENOMEM;
    }

    node->ctx = dep;
    list_add_head(&ctx->dependency, &node->node);

    return 0;
}

static void clear_all_dependency(struct sensor_ctx *ctx)
{
    struct ctx_list_node *node, *tmp_node;

    list_for_every_entry_safe(&ctx->dependency, node, tmp_node, struct ctx_list_node, node) {
        list_delete(&node->node);
        free(node);
    }
}

static int register_subscriber(struct sensor_ctx *ctx, struct sensor_ctx *subscriber)
{
    struct ctx_list_node *node;

    if (ctx_in_list(&ctx->subscriber, subscriber))
        return 0;

    node = malloc(sizeof(*node));
    if (!node) {
        printf("%s<--%s alloc memory failed to register subscriber\n",
                ctx->sensor->name, subscriber->sensor->name);
        return -ENOMEM;
    }

    node->ctx = subscriber;
    list_add_head(&ctx->subscriber, &node->node);

    return 0;
}

static int unregister_subscriber(struct sensor_ctx *ctx, struct sensor_ctx *subscriber)
{
    struct ctx_list_node *node;

    if (!(node = get_ctx_node(&ctx->subscriber, subscriber))) {
        printf("no such subscriber\n");
        return -ENOENT;
    }

    list_delete(&node->node);
    free(node);

    return 0;
}

static int init_sensor_context(struct sensor_ctx *ctx,
        const struct sensor_dev *sdev, const struct virtual_sensor_desc *vinfo,
        struct sensor_algo *algo, int handle)
{
    struct sensor_t *sensor = ctx->sensor;
    int ret;

    printf("smgr: init context for %s with handle=%d\n", sdev->name, handle);
    ctx->sdev = sdev;
    ctx->enabled = false;
    ctx->delay = 0;
    list_initialize(&ctx->dependency);
    list_initialize(&ctx->subscriber);

    sensor->handle = handle;
    if (sdev) {
        if (!sdev->ops)
            return -EINVAL;

        sensor->name = sdev->name;
        sensor->vendor = sdev->ops->get_vendor(sdev);
        sensor->module = sdev->ops->get_module(sdev);
        /* XXX, TODO: get rid of get_type method? */
        sensor->type = sdev->type;
        sensor->module = sdev->ops->get_module(sdev);
        sensor->max_range = sdev->ops->get_max_range(sdev);
        sensor->power = sdev->ops->get_power(sdev);
        sensor->min_delay = sdev->ops->get_min_delay(sdev);
        sensor->max_delay = sdev->ops->get_max_delay(sdev);
        sensor->resolution = sdev->ops->get_resolution(sdev);

        /* a hardware sensor also need to depend to itself */
        ret = add_dependency_sensor(ctx, ctx);
        if (ret)
            return ret;
    } else if (vinfo->deps[0] > 0) {
        /*for virtual sensors*/
        struct sensor_ctx *dep_ctx;
        int i;

        /*add all dependencies*/
        for (i = 0; vinfo->deps[i] > 0; i++) {
            dep_ctx = get_ctx_by_type(vinfo->deps[i]);
            if (!dep_ctx) {
                clear_all_dependency(ctx);
                return -ENOENT;
            }

            if ((ret = add_dependency_sensor(ctx, dep_ctx))) {
                clear_all_dependency(ctx);
                return ret;
            }
        }
        sensor->name = vinfo->name;
        sensor->vendor = vinfo->vendor;
        sensor->type = vinfo->type;
        /*TODO: fill the rest attributes such as delay/range/power/resolution*/
    } else {
        return -EINVAL;
    }

    /*TODO: bind proper algo with this context if need */
    ctx->algo = algo;

    return 0;
}

/*XXX: return error code or the number of successfull hardware sensors */
static int probe_hardware_sensors()
{
    int num;
    int i, j;
    int ret;
    const struct sensor_dev *sdev;
    const struct sensor_dev **sdev_list;

    ret = hw_sensors_init();
    if (ret) {
        printf("failed to init hardware sensors:ret=%d\n", ret);
        return ret;
    }

    num = hw_sensors_get_list(&sdev_list);

    printf("smgr: start to init sensor context\n");
    i = j = 0;
    while (num-- > 0) {
        sdev = *(sdev_list + j++);
        ret = init_sensor_context(&sensor_context[i], sdev,
                                NULL, NULL, i);
        if (ret) {
            printf("init sensor dev %s failed, ret=%d\n", sdev->name, ret);
            continue;
        }
        i++;
    }
    printf("smgr: finish to init sensor context\n");

    smgr_sensor_num = i;

    printf("smgr: %d hw sensors probed:\n", smgr_sensor_num);
    for (i = 0; i < smgr_sensor_num; i++) {
        printf("\tname=%s, type=%d, handle=%d\n", sensor_context[i].sensor->name,
                sensor_context[i].sensor->type,
                sensor_context[i].sensor->handle);
    }

    return smgr_sensor_num ? 0 : -ENOENT;
}

static bool is_dependency_present(const int *dep_types)
{
    int i;

    for (i = 0; dep_types[i] > 0 && i < SMGR_MAX_DEPS; i++)
        if (!get_ctx_by_type(dep_types[i]))
            return false;

    return true;
}

/*XXX: return error code or the number of successfull virtual sensors? */
static int probe_virtual_sensors()
{
    int ret = 0;
    int i, j;
    struct sensor_algo *algo;

    j = smgr_sensor_num;
    for (i = 0; i < __countof(possible_vsensors); i++) {
        if (is_dependency_present(possible_vsensors[i].deps) &&
            (algo = amgr_find_algo(possible_vsensors[i].type))) {
            ret = init_sensor_context(&sensor_context[j], NULL,
                                    &possible_vsensors[i], algo, j);
            if (!ret)
                j++;
        }
    }

    printf("smgr: %d virtual sensors probed:\n", j - smgr_sensor_num);
    for (i = smgr_sensor_num; i < j; i++) {
        printf("\tname=%s, type=%d, handle=%d\n", sensor_context[i].sensor->name,
                sensor_context[i].sensor->type,
                sensor_context[i].sensor->handle);
    }

    smgr_sensor_num = j;

    return 0;
}

static int probe_sensors()
{
    int ret;

    ret = probe_hardware_sensors();
    if (ret < 0) {
        printf("Fatal error: non hardware sensor\n");
        return ret;
    }

    ret = probe_virtual_sensors();
    if (ret < 0)
        printf("probe virtual sensors: %d\n", ret);

    return 0;
}

static void smgr_work_thread(void *argument)
{
    osMessageQueueId_t queue = argument;
    struct smgr_work work;
    osStatus_t status;

    while (1) {
        status = osMessageQueueGet(queue, &work, NULL, osWaitForever);
        if (status != osOK) {
            printf("%s fail to get message %d\n", osThreadGetName(osThreadGetId()), status);
            continue;
        }

        if (work.func)
            work.func(work.data, work.ts);
    }
}

int smgr_schedule_work(smgr_work_func func, void *data, int64_t ts, uint32_t priority)
{
    struct smgr_work work;
    osStatus_t status;

    if (priority >= PRI_NUM)
        return -EINVAL;

    work.func = func;
    work.data = data;
    work.ts = ts;

    status = osMessageQueuePut(smgr_workqueue[priority], &work, 0, 0);

    return status == osOK ? 0 : -EBUSY;
}

static int smgr_create_workqueue()
{
    osMessageQueueAttr_t queue_attr = {};
    osThreadAttr_t thread_attr = {};
    osThreadId_t thread_hi, thread_low;

    queue_attr.name = "smgr_hi_queue";
    smgr_workqueue[HIGH_WORK] = osMessageQueueNew(64, sizeof(struct smgr_work), &queue_attr);
    if (smgr_workqueue[HIGH_WORK] == NULL)
        return osErrorNoMemory;

    queue_attr.name = "smgr_low_queue";
    smgr_workqueue[LOW_WORK] = osMessageQueueNew(32, sizeof(struct smgr_work), &queue_attr);
    if (smgr_workqueue[LOW_WORK] == NULL)
        goto low_queue_err;

    thread_attr.name = "smgr_hi";
    thread_attr.priority = osPriorityRealtime5;
    thread_hi = osThreadNew(smgr_work_thread, smgr_workqueue[HIGH_WORK], &thread_attr);
    if (thread_hi == NULL)
        goto hi_thread_err;

    thread_attr.name = "smgr_low";
    thread_attr.priority = osPriorityRealtime3;
    thread_low = osThreadNew(smgr_work_thread, smgr_workqueue[LOW_WORK], &thread_attr);
    if (thread_low == NULL)
        goto low_thread_err;

    return 0;

low_thread_err:
    osThreadTerminate(thread_hi);
hi_thread_err:
    osMessageQueueDelete(smgr_workqueue[LOW_WORK]);
low_queue_err:
    osMessageQueueDelete(smgr_workqueue[HIGH_WORK]);

    return osErrorNoMemory;
}

/* probe all hardware sensors via hw_sensors interface, and then
 * generate the virtual sensors with the provided algorithm.
 * this one must be inited by osInitDef after the hardware's
 * */
int smgr_init(cmgr_push_event push_event, cmgr_handle_dispatch dispatch)
{
    int i;
    int ret;

    if (!push_event || !dispatch)
        return -EINVAL;

    push_to_cmgr = push_event;
    dispatch_cb = dispatch;

    smgr_sensor_num = 0;
    for (i = 0; i < SMGR_MAX_SENSORS; i++)
        sensor_context[i].sensor = &sensor_list[i];

    ret = probe_sensors();
    if (ret) {
        printf("smgr_init: result=%d\n", ret);
        return ret;
    }

    return smgr_create_workqueue();
}

/* get all of sensors, both hardware and virtual sensors */
int smgr_get_sensor_list(struct sensor_t **list)
{
    *list = sensor_list;

    return smgr_sensor_num;
}

/* modify the ODR of the depended sensor according to the all subscribers' request */
static int sync_delay(struct sensor_ctx *ctx)
{
    struct ctx_list_node *ctx_node;
    int min_delay = INT32_MAX;

    if (list_is_empty(&ctx->subscriber))
        return 0;

    list_for_every_entry(&ctx->subscriber, ctx_node, struct ctx_list_node, node) {
        if (ctx_node->ctx->delay == 0)
            continue;

        if (min_delay > ctx_node->ctx->delay)
            min_delay = ctx_node->ctx->delay;
    }

    return smgr_set_delay(ctx->sensor->handle, min_delay);
}

int smgr_activate(int handle, int enable)
{
    struct ctx_list_node *ctx_node;
    struct sensor_ctx *ctx = get_ctx_by_handle(handle);
    bool dep_activate = false;
    int ret;

    printf("smgr_activate: handle=%d(%s), enable=%d\n", handle, ctx->sensor->name, enable);
    if (!ctx) {
        printf("failed to get context for handle=%d\n", handle);
        return -EINVAL;
    }

    if (ctx->enabled == enable) {
        printf("smgr_activate: handle=%d, enable=%d, state not changed\n", handle, enable);
        return 0;
    }

    list_for_every_entry(&ctx->dependency, ctx_node, struct ctx_list_node, node) {
        if (enable) {
            ret = register_subscriber(ctx_node->ctx, ctx);
            if (ret)
                goto register_err;

            /* for the first subscriber, the depended sensor should be enabled */
            if (list_length(&ctx_node->ctx->subscriber) == 1)
                dep_activate = true;
        } else {
            ret = unregister_subscriber(ctx_node->ctx, ctx);
            if (ret)
                goto unregister_err;

            if (list_is_empty(&ctx_node->ctx->subscriber)) {
                dep_activate = true;
            } else {
                /* recalculate the best delay among the all subscribers */
                sync_delay(ctx_node->ctx);
            }
        }

        /*XXX: need to recursivly find all the depended hardware sensor an enable them if not yet */
        if (dep_activate) {
            ret = ctx_node->ctx->sdev->ops->activate(ctx_node->ctx->sdev, enable);
            if (ret)
                goto activate_dep_err;
        }
    }

    if (ctx->algo && ctx->algo->command)
        ret = ctx->algo->command(ctx->sensor, CMD_ENABLE, &enable);

    if (ret)
        goto algo_err;

    ctx->enabled = enable;
    return 0;

/* when error occured, we just unregister the subcriber to the depended sensors, and
 * set the sensor as DISABLED state */
register_err:
unregister_err:
activate_dep_err:
algo_err:
    list_for_every_entry(&ctx->dependency, ctx_node, struct ctx_list_node, node) {
            unregister_subscriber(ctx_node->ctx, ctx);

            if (list_is_empty(&ctx_node->ctx->subscriber))
                ctx_node->ctx->sdev->ops->activate(ctx_node->ctx->sdev, 0);
    }

    return ret;
}

int smgr_set_delay(int handle, uint32_t us)
{
    struct sensor_ctx *ctx = get_ctx_by_handle(handle);
    struct ctx_list_node *ctx_node;
    int ret = 0;

    printf("smgr_set_delay: handle=%d(%s), delay=%d\n", handle, ctx->sensor->name, us);
    if (!ctx) {
        printf("failed to get context for handle=%d\n", handle);
        return -EINVAL;
    }

    if (us < ctx->sensor->min_delay)
        us = ctx->sensor->min_delay;

    ctx->delay = us;

    if (!list_is_empty(&ctx->dependency)) {
        list_for_every_entry(&ctx->dependency, ctx_node, struct ctx_list_node, node) {
            if (ctx_node->ctx != ctx)
                sync_delay(ctx_node->ctx);
        }
    }

    if (ctx->sdev) {
        ret = ctx->sdev->ops->set_delay(ctx->sdev, us);
        printf("smgr_set_delay:(%s) for %d\n", ctx->sensor->name, us);
    }

    return ret;
}

bool smgr_is_sensor_onchange(struct sensor_t *sensor)
{
    return false;
}

/* this routine will be called by all of the hardware drivers when they have new sensor events */
int smgr_push_data(struct sensor_event *data, int num)
{
    struct sensor_ctx *ctx;
    int i;

    if (!num)
        return 0;

    if (!push_to_cmgr)
        return -EPIPE;

    for (i = 0; i < num; i++) {
        ctx = get_ctx_by_type(data[i].type);
        assert(ctx);

        data[i].sensor = ctx->sensor;
    }

    return push_to_cmgr(data, num);
}

static void smgr_update_virtual_event(struct sensor_ctx *ctx, struct sensor_event *event)
{
    struct ctx_list_node *ctx_node;
    struct sensor_event aevent;

    /* recusively generate the virtual event for all the subsribers */
    list_for_every_entry(&ctx->subscriber, ctx_node, struct ctx_list_node, node) {
        struct sensor_ctx *ctx = ctx_node->ctx;
        if (ctx->algo && ctx->algo->update &&
                !ctx->algo->update(ctx->sensor, event, &aevent)) {
            if (ctx->enabled)
                dispatch_cb(&aevent, 1);

            smgr_update_virtual_event(ctx, &aevent);
        }
    }
}

/* a hardware sensor event maybe used to generate the virtual events, so we need
 * to call the handler provided by sensor manager. the new generated events will
 * be also sent to the clients
 * */
int smgr_handle_event(struct sensor_event *event)
{
    struct sensor_ctx *ctx;
    struct sensor_event aevent;
    struct sensor_event *pevent = event;

    if (!dispatch_cb)
        return -ENOSYS;

    ctx = get_ctx_by_type(event->sensor->type);
    if (!ctx)
        return -ENODEV;

    /* a hardware sensor must have a sdev member */
    if (!ctx->sdev)
        return -EINVAL;

    if (ctx->enabled) {
        if (ctx->algo && ctx->algo->update &&
                !ctx->algo->update(ctx->sensor, event, &aevent))
            pevent = &aevent;

        dispatch_cb(pevent, 1);
    }

    smgr_update_virtual_event(ctx, pevent);

    return 0;
}
