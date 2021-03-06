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
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("FISHSEMI SOFTWARE")
 * RECEIVED FROM FISHSEMI AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. FISHSEMI EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES FISHSEMI PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE FISHSEMI SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN FISHSEMI
 * SOFTWARE. FISHSEMI SHALL ALSO NOT BE RESPONSIBLE FOR ANY FISHSEMI SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND FISHSEMI'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE FISHSEMI SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT FISHSEMI'S OPTION, TO REVISE OR REPLACE THE
 * FISHSEMI SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO FISHSEMI FOR SUCH FISHSEMI SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("Fishsemi
 * Software") have been modified by Fishsemi Inc. All revisions are subject to
 * any receiver's applicable license agreements with Fishsemi Inc.
 */

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
#include <fcntl.h>
#include <sys/stat.h>

#define SMGR_MAX_SENSORS    20
#define VENDOR_FISHSEMI     "Fishsemi"
#define SMGR_MAX_DEPS       6

#define SMGR_HI_PRIORITY (SCHED_PRIORITY_DEFAULT + 50)
#define SMGR_LOW_PRIORITY (SCHED_PRIORITY_DEFAULT + 10)

static cmgr_push_event push_to_cmgr;
static cmgr_handle_dispatch dispatch_cb;
static struct snshub_sensor_t sensor_list[SMGR_MAX_SENSORS];
static struct sensor_ctx sensor_context[SMGR_MAX_SENSORS];
static int smgr_sensor_num;

struct smgr_work {
    smgr_work_func func;
    FAR void *data;
    int64_t ts;
};

struct virtual_sensor_desc {
    FAR char *name;
    FAR char *vendor;
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
        VENDOR_FISHSEMI,
        SENSOR_TYPE_MAGNETIC_FIELD,
        {SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED, -1},
    },

    /* calibrated gyro sensor
     * depends:
     *      uncalibrated gyro sensor
     * */
    {
        "gyroscope sensor",
        VENDOR_FISHSEMI,
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
        VENDOR_FISHSEMI,
        SENSOR_TYPE_GYROSCOPE,
        {SENSOR_TYPE_ACCELEROMETER, SENSOR_TYPE_MAGNETIC_FIELD, -1},
        /*XXX: select from AMG and AM, but both is from the fusion algo */
    },
};

static FAR struct sensor_ctx *get_ctx_by_handle(int handle)
{
    int i;

    for (i = 0; i < smgr_sensor_num; i++) {
        if (sensor_context[i].sensor->handle == handle)
            return &sensor_context[i];
    }

    return NULL;
}

static FAR struct sensor_ctx *get_ctx_by_type(int type)
{
    int i;

    for (i = 0; i < smgr_sensor_num; i++) {
        if (sensor_context[i].sensor->type == type)
            return &sensor_context[i];
    }

    return NULL;
}

static bool ctx_in_list(FAR struct list_node *list, FAR struct sensor_ctx *ctx)
{
    FAR struct ctx_list_node *ctx_node;

    list_for_every_entry(list, ctx_node, struct ctx_list_node, node) {
        if (ctx_node->ctx == ctx)
            return true;
    }

    return false;
}

static FAR struct ctx_list_node *get_ctx_node(FAR struct list_node *list,
                                            FAR struct sensor_ctx *ctx)
{
    FAR struct ctx_list_node *ctx_node;

    list_for_every_entry(list, ctx_node, struct ctx_list_node, node) {
        if (ctx_node->ctx == ctx)
            return ctx_node;
    }

    return NULL;
}

static int add_dependency_sensor(FAR struct sensor_ctx *ctx, FAR struct sensor_ctx *dep)
{
    FAR struct ctx_list_node *node;

    if (ctx_in_list(&ctx->dependency, dep))
        return 0;

    node = malloc(sizeof(*node));
    if (!node) {
        snshuberr("%s->%s alloc memory failed to add dependency\n",
                ctx->sensor->name, dep->sensor->name);
        return -ENOMEM;
    }

    node->ctx = dep;
    list_add_head(&ctx->dependency, &node->node);

    return 0;
}

static void clear_all_dependency(FAR struct sensor_ctx *ctx)
{
    FAR struct ctx_list_node *node, *tmp_node;

    list_for_every_entry_safe(&ctx->dependency, node, tmp_node, struct ctx_list_node, node) {
        list_delete(&node->node);
        free(node);
    }
}

static int register_subscriber(FAR struct sensor_ctx *ctx, FAR struct sensor_ctx *subscriber)
{
    FAR struct ctx_list_node *node;

    if (ctx_in_list(&ctx->subscriber, subscriber))
        return 0;

    node = malloc(sizeof(*node));
    if (!node) {
        snshuberr("%s<--%s alloc memory failed to register subscriber\n",
                ctx->sensor->name, subscriber->sensor->name);
        return -ENOMEM;
    }

    node->ctx = subscriber;
    list_add_head(&ctx->subscriber, &node->node);

    return 0;
}

static int unregister_subscriber(FAR struct sensor_ctx *ctx, FAR struct sensor_ctx *subscriber)
{
    FAR struct ctx_list_node *node;

    if (!(node = get_ctx_node(&ctx->subscriber, subscriber))) {
        snshuberr("no such subscriber\n");
        return -ENOENT;
    }

    list_delete(&node->node);
    free(node);

    return 0;
}

static int init_sensor_context(FAR struct sensor_ctx *ctx,
        FAR const struct sensor_dev *sdev, FAR const struct virtual_sensor_desc *vinfo,
        FAR struct sensor_algo *algo, int handle)
{
    FAR struct snshub_sensor_t *sensor = ctx->sensor;
    int ret;

    snshubinfo("smgr: init context for %s with handle=%d\n", sdev->name, handle);
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
static int probe_hardware_sensors(void)
{
    int num;
    int i, j;
    int ret;
    FAR const struct sensor_dev *sdev;
    FAR const struct sensor_dev **sdev_list;

    ret = hw_sensors_init();
    if (ret) {
        snshuberr("failed to init hardware sensors:ret=%d\n", ret);
        return ret;
    }

    num = hw_sensors_get_list(&sdev_list);

    snshubinfo("smgr: start to init sensor context\n");
    i = j = 0;
    while (num-- > 0) {
        sdev = *(sdev_list + j++);
        ret = init_sensor_context(&sensor_context[i], sdev,
                                NULL, NULL, i);
        if (ret) {
            snshuberr("init sensor dev %s failed, ret=%d\n", sdev->name, ret);
            continue;
        }
        i++;
    }
    snshubinfo("smgr: finish to init sensor context\n");

    smgr_sensor_num = i;

    snshubinfo("smgr: %d hw sensors probed:\n", smgr_sensor_num);
    for (i = 0; i < smgr_sensor_num; i++) {
        snshubinfo("\tname=%s, type=%d, handle=%d\n", sensor_context[i].sensor->name,
                sensor_context[i].sensor->type,
                sensor_context[i].sensor->handle);
    }

    return smgr_sensor_num ? 0 : -ENOENT;
}

static bool is_dependency_present(FAR const int *dep_types)
{
    int i;

    for (i = 0; dep_types[i] > 0 && i < SMGR_MAX_DEPS; i++)
        if (!get_ctx_by_type(dep_types[i]))
            return false;

    return true;
}

/*XXX: return error code or the number of successfull virtual sensors? */
static int probe_virtual_sensors(void)
{
    int ret = 0;
    int i, j;
    FAR struct sensor_algo *algo;

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

    snshubinfo("smgr: %d virtual sensors probed:\n", j - smgr_sensor_num);
    for (i = smgr_sensor_num; i < j; i++) {
        snshubinfo("\tname=%s, type=%d, handle=%d\n", sensor_context[i].sensor->name,
                sensor_context[i].sensor->type,
                sensor_context[i].sensor->handle);
    }

    smgr_sensor_num = j;

    return 0;
}

static int probe_sensors(void)
{
    int ret;

    ret = probe_hardware_sensors();
    if (ret < 0) {
        snshuberr("Fatal error: non hardware sensor\n");
        return ret;
    }

    ret = probe_virtual_sensors();
    if (ret < 0)
        snshuberr("probe virtual sensors: %d\n", ret);

    return 0;
}

/* probe all hardware sensors via hw_sensors interface, and then
 * generate the virtual sensors with the provided algorithm.
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
        snshuberr("smgr_init: result=%d\n", ret);
        return ret;
    }

    return ret;
}

/* get all of sensors, both hardware and virtual sensors */
int smgr_get_sensor_list(FAR struct snshub_sensor_t **list)
{
    *list = sensor_list;

    return smgr_sensor_num;
}

/* modify the ODR of the depended sensor according to the all subscribers' request */
static int sync_delay(FAR struct sensor_ctx *ctx)
{
    FAR struct ctx_list_node *ctx_node;
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

int smgr_activate(int handle, int enable, snshub_data_mode mode)
{
    FAR struct ctx_list_node *ctx_node;
    FAR struct sensor_ctx *ctx = get_ctx_by_handle(handle);
    bool dep_activate = false;
    int ret;

    snshubinfo("smgr_activate: handle=%d(%s), enable=%d\n", handle, ctx->sensor->name, enable);
    if (!ctx) {
        snshuberr("failed to get context for handle=%d\n", handle);
        return -EINVAL;
    }

    if (ctx->enabled == enable) {
        snshuberr("smgr_activate: handle=%d, enable=%d, state not changed\n", handle, enable);
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
            ret = ctx_node->ctx->sdev->ops->activate(ctx_node->ctx->sdev, enable, mode);
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
                ctx_node->ctx->sdev->ops->activate(ctx_node->ctx->sdev, 0, mode);
    }

    return ret;
}

int smgr_set_delay(int handle, uint32_t us)
{
    FAR struct sensor_ctx *ctx = get_ctx_by_handle(handle);
    FAR struct ctx_list_node *ctx_node;
    int ret = 0;

    snshubinfo("smgr_set_delay: handle=%d(%s), delay=%d\n", handle, ctx->sensor->name, us);
    if (!ctx) {
        snshuberr("failed to get context for handle=%d\n", handle);
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
        snshubinfo("smgr_set_delay:(%s) for %d\n", ctx->sensor->name, us);
    }

    return ret;
}

int smgr_read_data(int handle, struct sensor_event *event)
{
    FAR struct sensor_ctx *ctx = get_ctx_by_handle(handle);
    int ret = 0;

    if (!ctx) {
        snshuberr("failed to get context for handle=%d\n", handle);
        return -EINVAL;
    }

    if (ctx->sdev) {
        ret = ctx->sdev->ops->read_data(ctx->sdev, event);
    }

    return ret;
}

bool smgr_is_sensor_onchange(FAR struct snshub_sensor_t *sensor)
{
    return false;
}

/* this routine will be called by all of the hardware drivers when they have new sensor events */
int smgr_push_data(FAR struct sensor_event *data, int num)
{
    struct sensor_ctx *ctx;
    int i;

    if (!num)
        return 0;

    if (!push_to_cmgr)
        return -EPIPE;

    for (i = 0; i < num; i++) {
        ctx = get_ctx_by_type(data[i].type);
        DEBUGASSERT(ctx);

        data[i].sensor = ctx->sensor;
    }

    return push_to_cmgr(data, num);
}

static void smgr_update_virtual_event(FAR struct sensor_ctx *ctx, FAR struct sensor_event *event)
{
    FAR struct ctx_list_node *ctx_node;
    struct sensor_event aevent;

    /* recusively generate the virtual event for all the subsribers */
    list_for_every_entry(&ctx->subscriber, ctx_node, struct ctx_list_node, node) {
        ctx = ctx_node->ctx;
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
int smgr_handle_event(FAR struct sensor_event *event)
{
    FAR struct sensor_ctx *ctx;
    struct sensor_event aevent;
    FAR struct sensor_event *pevent = event;

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
