#ifndef __ICM42605_H__
#define __ICM42605_H__

#include "sensor.h"

struct icm42605_odr_info {
  uint8_t regval;
  /* the unit is hz */
  uint32_t odr;
  /* the unit is us */
  uint32_t times;
};

struct icm42605_fsr_info {
  uint32_t fsr;
  uint8_t regval;
  float resolution;
};

struct icm42605_platform_data {
  int irq_pin;
  int trigger_type;
  int place;
};

int icm42605_initialize(FAR struct sensor_dev **dev, FAR int *num);
void icm42605_uninitialize(void);

#endif
