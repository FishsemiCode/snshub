#include <cmsis_os2.h>
#include <Driver_USART.h>
#include <errno.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sensor.h"
#include "utils.h"
#include "circ_buffer.h"
#include "client_manager.h"

#define SNS_FLAG_NEW_EVENT      0x0001
#define SNS_FLAG_UART_RCV       0x0002

#define ACCEL_SENSOR_RATE       100
#define GYRO_SENSOR_RATE        100
#define MAG_SENSOR_RATE         50
#define PRESSURE_SENSOR_RATE    100
#define PRX_SENSOR_RATE         10
#define ALS_SENSOR_RATE         10

#define uart    (Driver_USART[3])

struct sensor_config {
    int32_t type;
    uint32_t delay;
};

static const struct sensor_config config[] = {
    { SENSOR_TYPE_ACCELEROMETER, 1000000 / ACCEL_SENSOR_RATE },
    { SENSOR_TYPE_GYROSCOPE, 1000000 / GYRO_SENSOR_RATE },
    { SENSOR_TYPE_MAGNETIC_FIELD, 1000000 / MAG_SENSOR_RATE },
    { SENSOR_TYPE_PRESSURE, 1000000 / PRESSURE_SENSOR_RATE },
    { SENSOR_TYPE_PROXIMITY, 1000000 / PRX_SENSOR_RATE },
    { SENSOR_TYPE_LIGHT, 1000000 / ALS_SENSOR_RATE },
};


static struct sensor_client *tclient;
static osThreadId_t tclient_thread;
static struct circ_buffer *tclient_ebuffer;

static int tclient_sensor_config(struct sensor_client *client, bool enable)
{
    struct sensor_t *sensor;
    int i;

    for (i = 0; i < sizeof(config) / sizeof(struct sensor_config); i++) {
        sensor = cmgr_get_sensor_by_type(config[i].type);
        if (!sensor) {
            printf("failed to get sensor type %d\n", config[i].type);
            continue;
        }

        if (cmgr_activate_sensor_one(tclient, sensor, config[i].delay, enable)) {
            printf("failed to %s sensor type %d\n", enable ? "enalbe" : "disable", config[i].type);
            return -EIO;
        }
    }

    return 0;
}

/* the callback to handle the sensor events, in client_manager thread context */
static void tclient_sevent_cb(struct sensor_event *event, size_t num)
{
    int err;

    err = circ_buffer_push(tclient_ebuffer, event, num);
    if (err) {
        printf("failed to push event to tclient buffer\n");
    } else {
        osThreadFlagsSet(tclient_thread, SNS_FLAG_NEW_EVENT);
    }
}

/* the callback to handle the sensor accuracy change */
static void tclient_saccuracy_cb(struct sensor_t *sensor, int accuracy)
{

}

static struct client_callback tclient_sns_cb = {
    .event_update = tclient_sevent_cb,
    .accuracy_changed = tclient_saccuracy_cb,
};

static int tclient_event_handler(void *data)
{
    struct sensor_event *event = data;
    static uint32_t cnt_a = 0;
    static uint32_t cnt_g = 0;
    static uint32_t cnt_m = 0;
    static uint32_t cnt_p = 0;
    char info[128];
    int ret;

    switch (event->sensor->type) {
    case SENSOR_TYPE_ACCELEROMETER:
        ret = snprintf(info, sizeof(info), "accel:%f, %f, %f\n",
                event->accel_t.x, event->accel_t.y,event->accel_t.z);
        uart->Send(info, ret);

        if (!(cnt_a++ % ACCEL_SENSOR_RATE))
            printf("%s", info);
        break;
    case SENSOR_TYPE_GYROSCOPE:
        if (!(cnt_g++ % GYRO_SENSOR_RATE))
            printf("gyro:%f, %f, %f\n",
                    event->gyro_t.x, event->gyro_t.y, event->gyro_t.z);
        break;
    case SENSOR_TYPE_MAGNETIC_FIELD:
        if (!(cnt_m++ % MAG_SENSOR_RATE))
            printf("mag:%f, %f, %f\n",
                    event->magnetic_t.x, event->magnetic_t.y, event->magnetic_t.z);
        break;
    case SENSOR_TYPE_PRESSURE:
        if (!(cnt_p++ % PRESSURE_SENSOR_RATE))
            printf("pressure:%f\n", event->pressure_t.value);
        break;
    case SENSOR_TYPE_PROXIMITY:
        printf("proximity:%d, %d\n", event->distance_raw.raw0, event->distance_raw.raw1);
        break;
    case SENSOR_TYPE_LIGHT:
        printf("light:%d, %d\n", event->light_raw.raw0, event->light_raw.raw1);
        ret = snprintf(info, 32, "tclient light %d\n", event->light_raw.raw0);
        uart->Send(info, ret);
        break;
    default:
        break;
    }

    return 0;
}

void tclient_uart_handler(uint32_t event, void *edata)
{
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
        osThreadFlagsSet(tclient_thread, SNS_FLAG_UART_RCV);
}

static void tclient_thread_func(void *arg)
{
    uint32_t sns_flags = 0;
    char rcv[128];
    int ret;
    bool state = false;

    /* need to wait the client manager and sensor manager being ready */
    while(!cmgr_system_ready());

    osThreadFlagsClear(SNS_FLAG_NEW_EVENT | SNS_FLAG_UART_RCV);
    uart->SetExtHandler(tclient_uart_handler, NULL);

    uart->Receive(rcv, 32);
    while (1) {
        sns_flags = osThreadFlagsWait(SNS_FLAG_NEW_EVENT | SNS_FLAG_UART_RCV, osFlagsWaitAny, osWaitForever);
        if (sns_flags & SNS_FLAG_NEW_EVENT) {
            circ_buffer_for_each(tclient_ebuffer, tclient_event_handler);
        }

        if (sns_flags & SNS_FLAG_UART_RCV) {
            rcv[uart->GetRxCount() - 1 ] = '\0';
            if (state && !strncmp("stop_sns", rcv, strlen("stop_sns"))) {
                printf("now stop all the sensors for tclient\n");
                tclient_sensor_config(tclient, false);
                state = false;
            } else if (!state && !strncmp("start_sns", rcv, strlen("start_sns"))) {
                ret = tclient_sensor_config(tclient, true);
                assert(!ret);
                state = true;
            }

            uart->Receive(rcv, 32);
        }
    }
}

static osStatus_t tclient_init()
{
    int ret = osOK;
    osThreadAttr_t thread_attr = {};

    tclient = cmgr_client_request("test_client", &tclient_sns_cb);
    if (!tclient) {
        printf("failed to request sensor client\n");
        return osErrorResource;
    }

    /* initialize the circular event buffer */
    ret = circ_buffer_init(&tclient_ebuffer, "tclient event", sizeof(struct sensor_event), 32);
    if (ret) {
        printf("tclient event buffer init failed:%d\n", ret);
        ret = osError;
        goto ebuffer_err;
    }

    thread_attr.name = "tclient_thread";
    /* the priority of the clients must be lower than the client manager */
    thread_attr.priority = osPriorityNormal;
    tclient_thread = osThreadNew(tclient_thread_func, NULL, &thread_attr);
    if (!tclient_thread) {
        ret = osErrorNoMemory;
        goto thread_err;
    }
    return osOK;

thread_err:
    circ_buffer_deinit(tclient_ebuffer);
ebuffer_err:
    cmgr_client_release(tclient);

    return ret;
}

osInitDef(tclient_init, SH_INIT_APP);
