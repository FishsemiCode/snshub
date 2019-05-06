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

#ifndef __SENSOR_MANAGER_H__
#define __SENSOR_MANAGER_H__

#include <nuttx/list.h>
#include "sensor.h"

enum work_priorty {
    HIGH_WORK = 0,
    LOW_WORK,
    PRI_NUM
};

struct sensor_ctx {
    FAR struct snshub_sensor_t *sensor;
    FAR const struct sensor_dev *sdev;
    FAR struct sensor_algo *algo;
    bool enabled;
    int delay;
    struct list_node dependency;
    struct list_node subscriber;
};

struct ctx_list_node {
    FAR struct sensor_ctx *ctx;
    struct list_node node;
};

typedef void (*smgr_work_func) (FAR void *data, int64_t ts);

/*XXX: this typedef should be in client_mmanager header file, and should be
 * implemented in client manager */
typedef int (*cmgr_push_event)(FAR struct sensor_event *data, int num);
typedef int (*cmgr_handle_dispatch)(FAR struct sensor_event *data, size_t num);

int smgr_init(cmgr_push_event push_event, cmgr_handle_dispatch dispatch);
int smgr_get_sensor_list(FAR struct snshub_sensor_t **list);
int smgr_activate(int handle, int enable, snshub_data_mode mode);
int smgr_set_delay(int handle, uint32_t us);
int smgr_read_data(int handle, struct sensor_event *event);
int smgr_push_data(FAR struct sensor_event *data, int num);
int smgr_handle_event(FAR struct sensor_event *event);
bool smgr_is_sensor_onchange(FAR struct snshub_sensor_t *sensor);
int smgr_schedule_work(smgr_work_func func, FAR void *data, int64_t ts, uint32_t priority);

#endif
