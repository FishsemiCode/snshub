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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "circ_buffer.h"

static inline size_t buffer_count(struct circ_buffer *buffer)
{
    return (buffer->head - buffer->tail) & (buffer->size - 1);
}

static inline size_t buffer_count_to_end(struct circ_buffer *buffer)
{
    size_t end = buffer->size - buffer->tail;
    size_t n = (buffer->head + end) & (buffer->size - 1);

    return n < end ? n : end;
}

static inline size_t buffer_space(struct circ_buffer *buffer)
{
    return (buffer->tail - buffer->head - 1) & (buffer->size - 1);
}

static inline size_t buffer_space_to_end(struct circ_buffer *buffer)
{
    size_t end = buffer->size - 1 - buffer->head;
    size_t n = (end + buffer->tail) & (buffer->size - 1);

    return n <= end ? n : (end + 1);
}

int circ_buffer_init(struct circ_buffer **buffer, const char *name, size_t element_size, size_t size)
{
    struct circ_buffer *cb;

    if (size & (size - 1)) {
        printf("circ buffer size is invalid of %s\n", name);
        return -EINVAL;
    }

    cb = calloc(1, sizeof(*cb) + element_size * size);
    if (!cb) {
        printf("failed to malloc memory for circ buffer %s\n", name);
        return -ENOMEM;
    }

    cb->name = name;
    cb->element_size = element_size;
    cb->size = size;
    cb->data = cb + 1;

    *buffer = cb;

    return 0;
}

int circ_buffer_push(struct circ_buffer *buffer, void *data, int num)
{
    size_t space, num_to_write;
    size_t esize;

    space = buffer_space(buffer);
    if (num > space) {
        printf("circ buffer: no enough space of %s\n", buffer->name);
        return -ENOMEM;
    }

    esize = buffer->element_size;
    space = buffer_space_to_end(buffer);
    num_to_write = (space >= num ? num : space);
    memcpy(buffer->data + buffer->head * esize, data, num_to_write * esize);

    num -= num_to_write;
    if (num > 0)
        memcpy(buffer->data, data + num_to_write * esize, num * esize);

    buffer->head = (buffer->head + num + num_to_write) & (buffer->size - 1);

    return 0;
}

/* return the actual element num got, otherwise a negative error */
int circ_buffer_pop(struct circ_buffer *buffer, void *data, int num)
{
    size_t cnt, num_to_read;
    size_t esize;

    esize = buffer->element_size;

    cnt = buffer_count(buffer);
    num = cnt > num ? num : cnt;

    cnt = buffer_count_to_end(buffer);
    num_to_read = cnt >= num ? num : cnt;
    memcpy(data, buffer->data + buffer->tail * esize, num_to_read * esize);

    num -= num_to_read;
    if (num > 0)
        memcpy(data + num_to_read * esize, buffer->data, num * esize);

    buffer->tail = (buffer->tail + num + num_to_read) & (buffer->size - 1);

    return num + num_to_read;
}

/* the return value is the count of handled elements, or negetive error N.O. */
int circ_buffer_for_each(struct circ_buffer *buffer, buffer_handler handler)
{
    size_t cnt, i;
    void *element;
    size_t esize;

    if (!handler)
        return -EINVAL;

    esize = buffer->element_size;

    cnt = buffer_count(buffer);
    for (i = 0; i < cnt; i++) {
        element = buffer->data + ((buffer->tail + i) & (buffer->size - 1)) * esize;
        handler(element);
    }

    buffer->tail = (buffer->tail + cnt) & (buffer->size - 1);

    return cnt;
}

void circ_buffer_deinit(struct circ_buffer *buffer)
{
    free(buffer);
}
