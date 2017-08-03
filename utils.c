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
#include <errno.h>
#include <assert.h>
#include <cmsis_os2.h>
#include <Driver_GPIO.h>
#include "sensor.h"
#include "utils.h"

struct axis_map {
    int8_t src_x;
    int8_t src_y;
    int8_t src_z;

    int8_t sign_x;
    int8_t sign_y;
    int8_t sign_z;
};

static const struct  axis_map remap_tbl[] = {
    /* src_x src_y src_z  sign_x  sign_y  sign_z */
    {  0,    1,    2,     1,      1,      1 }, /* P0 */
    {  1,    0,    2,     1,     -1,      1 }, /* P1 */
    {  0,    1,    2,    -1,     -1,      1 }, /* P2 */
    {  1,    0,    2,    -1,      1,      1 }, /* P3 */

    {  0,    1,    2,    -1,      1,     -1 }, /* P4 */
    {  1,    0,    2,    -1,     -1,     -1 }, /* P5 */
    {  0,    1,    2,     1,     -1,     -1 }, /* P6 */
    {  1,    0,    2,     1,      1,     -1 }, /* P7 */
};

void remap_vector_raw16to32_axis(const int16_t *in, int32_t *out, int place)
{
    const struct axis_map *remap;

    assert(place < __countof(remap_tbl));

    remap = &remap_tbl[place];
    out[0] = in[remap->src_x] * remap->sign_x;
    out[1] = in[remap->src_y] * remap->sign_y;
    out[2] = in[remap->src_z] * remap->sign_z;
}

void remap_vector_raw32_axis(const int32_t *in, int32_t *out, int place)
{
}

int sensor_register_gpio_irq(int pin, int trigger_type, ARM_GPIO_Handler_t handler, void *data)
{
    ARM_DRIVER_GPIO *gpio_ctl = Driver_GPIO[0];
    int ret;

    ret = gpio_ctl->SetDirection(pin, ARM_GPIO_DIR_INPUT);
    if (ret)
        return ret;

    ret = gpio_ctl->SetHandler(pin, handler, data);
    if (ret)
        return ret;

    ret = gpio_ctl->SetTrigger(pin, trigger_type);
    if (ret) {
        /* XXX:optimise the gpio driver interface to support unregister */
    }

    return ret;
}

int sensor_enable_gpio_irq(int pin, int trigger_type)
{
    ARM_DRIVER_GPIO *gpio_ctl = Driver_GPIO[0];

    return gpio_ctl->SetTrigger(pin, trigger_type);
}
