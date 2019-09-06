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

#ifndef __SENSOR_H__
#define __SENSOR_H__

#include <stdint.h>
#include <stdbool.h>

/* sensor type definition, follow the android sensor sub-system */
#define SENSOR_TYPE_ACCELEROMETER                   1
#define SENSOR_TYPE_MAGNETIC_FIELD                  2
#define SENSOR_TYPE_ORIENTATION                     3
#define SENSOR_TYPE_GYROSCOPE                       4
#define SENSOR_TYPE_LIGHT                           5
#define SENSOR_TYPE_PRESSURE                        6
#define SENSOR_TYPE_TEMPERATURE                     7
#define SENSOR_TYPE_PROXIMITY                       8
#define SENSOR_TYPE_GRAVITY                         9
#define SENSOR_TYPE_LINEAR_ACCELERATION             10
#define SENSOR_TYPE_ROTATION_VECTOR                 11
#define SENSOR_TYPE_RELATIVE_HUMIDITY               12
#define SENSOR_TYPE_AMBIENT_TEMPERATURE             13
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED     14
#define SENSOR_TYPE_GAME_ROTATION_VECTOR            15
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED          16
#define SENSOR_TYPE_SIGNIFICANT_MOTION              17
#define SENSOR_TYPE_STEP_DETECTOR                   18
#define SENSOR_TYPE_STEP_COUNTER                    19
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR     20
#define SENSOR_TYPE_HEART_RATE                      21
#define SENSOR_TYPE_TILT_DETECTOR                   22
#define SENSOR_TYPE_WAKE_GESTURE                    23
#define SENSOR_TYPE_GLANCE_GESTURE                  24
#define SENSOR_TYPE_PICK_UP_GESTURE                 25
#define SENSOR_TYPE_WRIST_TITL_GESTURE              26
#define SENSOR_TYPE_DEVICE_ORIENTATION              27
#define SENSOR_TYPE_POSE_6DOF                       28
#define SENSOR_TYPE_STATIONARY_GESTURE              29
#define SENSOR_TYPE_MOTION_DETECT                   30
#define SENSOR_TYPE_HEART_BEAT                      31
#define SENSOR_TYPE_DYNAMIC_SENSOR_META             32
#define SENSOR_TYPE_ADDITIONAL_INFO                 33
/*---non-android----*/
#define SENSOR_TYPE_PEDOMETER                       35
#define SENSOR_TYPE_INPOCKET                        36
#define SENSOR_TYPE_ACTIVITY                        37
#define SENSOR_TYPE_PDR                             38
#define SENSOR_TYPE_FREEFALL                        39
#define SENSOR_TYPE_ACCELEROMETER_UNCALIBRATED      40

#define GRAVITY     (9.80665f)
#define DEGREE2RAD  (3.14159265358979323f / 180)

/* user can active reading sensor data or also can wait the sensor event reported */
typedef enum {
  SNSHUB_POLLING,
  SNSHUB_INTERRUPT,
} snshub_data_mode;

struct snshub_sensor_t {
  FAR char *name;
  FAR char *vendor;
  FAR char *module;
  int type;
  int handle;
  float max_range;
  float power;
  int min_delay;
  int max_delay;
  float resolution;
  uint32_t flags;
  /*XXX: remove here? these 2 member should be in the client that work for the host interface */
  int fifo_max_event_count;
  int fifo_reserved_event_count;
  snshub_data_mode mode;
};

struct sensor_vector {
  float x;
  float y;
  float z;
};

struct sensor_vector_temp {
  float x;
  float y;
  float z;
  float temperature;
};

struct sensor_vector_raw {
  int32_t x_raw;
  int32_t y_raw;
  int32_t z_raw;
};

struct sensor_scalar {
  float value;
};

struct sensor_scalar_temp {
  float value;
  float temperature;
};

struct sensor_scalar_raw {
  int32_t raw0;
  int32_t raw1;
};

struct sensor_event {
  FAR struct snshub_sensor_t *sensor;
  int type;
  int64_t timestamp;
  int8_t status;
  uint8_t reserved[3];
  union {
    int32_t data_raw[3];
    struct sensor_vector_raw accel_raw;
    struct sensor_vector_raw gyro_raw;
    struct sensor_vector_raw magnetic_raw;
    struct sensor_vector_raw orientation_raw;
    struct sensor_scalar_raw light_raw;
    struct sensor_scalar_raw distance_raw;
    struct sensor_scalar_raw pressure_raw;
    struct sensor_scalar_raw temperature_raw;
    struct sensor_scalar_raw humidity_raw;
  };
  union {
    float data[4];
    struct sensor_vector accel;
    struct sensor_vector_temp accel_t;
    struct sensor_vector gyro;
    struct sensor_vector_temp gyro_t;
    struct sensor_vector magnetic;
    struct sensor_vector_temp magnetic_t;
    struct sensor_vector orientation;
    struct sensor_vector_temp orientation_t;
    struct sensor_scalar light;
    struct sensor_scalar_temp light_t;
    struct sensor_scalar distance;
    struct sensor_scalar_temp distance_t;
    struct sensor_scalar pressure;
    struct sensor_scalar_temp pressure_t;
    struct sensor_scalar temperature;
    struct sensor_scalar_temp temperature_t;
    struct sensor_scalar humidity;
    struct sensor_scalar_temp humidity_t;
  };
};

#endif
