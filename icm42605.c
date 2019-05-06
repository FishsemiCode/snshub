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

#include <errno.h>
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "sensor.h"
#include "sensor_manager.h"
#include "sensor_driver.h"
#include "sensor_port.h"
#include "icm42605.h"
#include "utils.h"

#define ACCEL_SENSOR                "accelerometer"
#define GYRO_SENSOR                 "gyroscope"

/* Bank 0 */
#define REG_CHIP_CONFIG              0x11
#define REG_DRIVE_CONFIG             0x13
#define REG_INT_CONFIG               0x14
#define REG_FIFO_CONFIG              0x16
#define REG_TEMP_DATA0_UI            0x1D
#define REG_ACCEL_DATA_X0_UI         0x1F
#define REG_GYRO_DATA_X0_UI          0x25
#define REG_TMST_FSYNC1              0x2B
#define REG_INT_STATUS               0x2D
#define REG_FIFO_BYTE_COUNT1         0x2E
#define REG_FIFO_BYTE_COUNT2         0x2F
#define REG_FIFO_DATA                0x30
#define REG_APEX_DATA0               0x31
#define REG_APEX_DATA1               0x32
#define REG_APEX_DATA2               0x33
#define REG_APEX_DATA3               0x34
#define REG_APEX_DATA4               0x35
#define REG_APEX_DATA5               0x36
#define REG_INT_STATUS2              0x37
#define REG_INT_STATUS3              0x38
#define REG_SIGNAL_PATH_RESET        0x4B
#define REG_INTF_CONFIG0             0x4C
#define REG_INTF_CONFIG1             0x4D
#define REG_PWR_MGMT_0               0x4E
#define REG_GYRO_CONFIG0             0x4F
#define REG_ACCEL_CONFIG0            0x50
#define REG_GYRO_CONFIG1             0x51
#define REG_ACCEL_GYRO_CONFIG0       0x52
#define REG_ACCEL_CONFIG1            0x53
#define REG_TMST_CONFIG              0x54
#define REG_APEX_CONFIG0             0x56
#define REG_SMD_CONFIG               0x57
#define REG_INT_RAW                  0x58
#define REG_FIFO_CONFIG1             0x5F
#define REG_FIFO_CONFIG2             0x60
#define REG_FSYNC_CONFIG             0x62
#define REG_INT_CONFIG0              0x63
#define REG_INT_CONFIG1              0x64
#define REG_INT_SOURCE0              0x65
#define REG_INT_SOURCE1              0x66
#define REG_INT_SOURCE2              0x67
#define REG_INT_SOURCE3              0x68
#define REG_INT_SOURCE4              0x69
#define REG_INT_SOURCE5              0x6A
#define REG_SENSOR_SELFTEST          0x6B
#define REG_FIFO_LOST_PKT0           0x6C
#define REG_SELF_TEST_CONFIG         0x70
#define REG_SCAN0                    0x71
#define REG_MEM_BANK_SEL             0x72
#define REG_MEM_START_ADDR           0x73
#define REG_MEM_R_W                  0x74
#define REG_WHO_AM_I                 0x75
#define REG_REG_BANK_SEL             0x76

/* Bank 1 */
#define REG_GYRO_CONFIG_STATIC2_B1   0x0B
#define REG_GYRO_CONFIG_STATIC3_B1   0x0C
#define REG_GYRO_CONFIG_STATIC4_B1   0x0D
#define REG_GYRO_CONFIG_STATIC5_B1   0x0E
#define REG_XG_ST_DATA_B1            0x5F
#define REG_YG_ST_DATA_B1            0x60
#define REG_ZG_ST_DATA_B1            0x61
#define REG_TMST_VAL0_B1             0x62
#define REG_OTP_SEC_STATUS_B1        0x70
#define REG_HTR_CONFIG_B1            0x77
#define REG_INTF_CONFIG4_B1          0x7A
#define REG_INTF_CONFIG5_B1          0x7B
#define REG_INTF_CONFIG6_B1          0x7C

/* Bank 2 */
#define REG_ACCEL_CONFIG_STATIC2_B2  0x03
#define REG_ACCEL_CONFIG_STATIC3_B2  0x04
#define REG_ACCEL_CONFIG_STATIC4_B2  0x05
#define REG_ACCEL_CONFIG_STATIC0_B2  0x39
#define REG_XA_ST_DATA_B2            0x3B
#define REG_YA_ST_DATA_B2            0x3C
#define REG_ZA_ST_DATA_B2            0x3D
/* Only accessible from AUX1 */
#define REG_OIS1_CONFIG1_B2          0x44
#define REG_OIS1_CONFIG2_B2          0x45
#define REG_OIS1_CONFIG3_B2          0x46
#define REG_ACCEL_DATA_X0_OIS1_B2    0x49
#define REG_GYRO_DATA_X0_OIS1_B2     0x4F
#define REG_INT_STATUS_OIS1_B2       0x57
/* End of Only accessible from AUX1 */
/* Only accessible from AUX2 */
#define REG_OIS2_CONFIG1_B2          0x59
#define REG_OIS2_CONFIG2_B2          0x5A
#define REG_OIS2_CONFIG3_B2          0x5B
#define REG_ACCEL_DATA_X0_OIS2_B2    0x5E
#define REG_GYRO_DATA_X0_OIS2_B2     0x64
#define REG_INT_STATUS_OIS2_B2       0x6C
/* End of Only accessible from AUX2 */

/* Bank 4 */
#define REG_APEX_CONFIG1_B4          0x40
#define REG_APEX_CONFIG2_B4          0x41
#define REG_APEX_CONFIG3_B4          0x42
#define REG_APEX_CONFIG4_B4          0x43
#define REG_APEX_CONFIG5_B4          0x44
#define REG_APEX_CONFIG6_B4          0x45
#define REG_APEX_CONFIG7_B4          0x46
#define REG_APEX_CONFIG8_B4          0x47
#define REG_APEX_CONFIG9_B4          0x48
#define REG_ACCEL_WOM_X_THR_B4       0x4A
#define REG_ACCEL_WOM_Y_THR_B4       0x4B
#define REG_ACCEL_WOM_Z_THR_B4       0x4C
#define REG_INT_SOURCE6_B4           0x4D
#define REG_INT_SOURCE7_B4           0x4E
#define REG_INT_SOURCE8_B4           0x4F
#define REG_INT_SOURCE9_B4           0x50
#define REG_INT_SOURCE10_B4          0x51
#define REG_INTF_CONFIG10_B4         0x52
#define REG_OFFSET_USER_0_B4         0x77
#define REG_OFFSET_USER_1_B4         0x78
#define REG_OFFSET_USER_2_B4         0x79
#define REG_OFFSET_USER_3_B4         0x7A
#define REG_OFFSET_USER_4_B4         0x7B
#define REG_OFFSET_USER_5_B4         0x7C
#define REG_OFFSET_USER_6_B4         0x7D
#define REG_OFFSET_USER_7_B4         0x7E
#define REG_OFFSET_USER_8_B4         0x7F

/* REG REG_INT STATUS */
#define BIT_INT_STATUS_UI_FSYNC      0x40
#define BIT_INT_STATUS_PLL_RDY       0x20
#define BIT_INT_STATUS_RESET_DONE    0x10
#define BIT_INT_STATUS_DRDY          0x08
#define BIT_INT_STATUS_FIFO_THS      0x04
#define BIT_INT_STATUS_FIFO_FULL     0x02
#define BIT_INT_STATUS_AGC_RDY       0x01

/* FIFO CONFIG MODE */
#define REG_FIFO_CONFIG_MODE_SNAPSHOT                (0x2 << 6)
#define REG_FIFO_CONFIG_MODE_STREAM                  (0x1 << 6)
#define REG_FIFO_CONFIG_MODE_BYPASS                  (0x0 << 6)
#define REG_FIFO_CONFIG_MODE_MASK                    (0x3 << 6)

/* FSYNC CONFIG UI SEL*/
#define REG_FSYNC_CONFIG_U1_SEL_NO                   (0x0 << 4)
#define REG_FSYNC_CONFIG_U1_SEL_MASK                 (0x7 << 4)

/* TMST CONFIG */
#define REG_TMST_CONFIG_RESOL_16us                   (0x1 << 3)
#define REG_TMST_CONFIG_RESOL_1us                    (0x0 << 3)
#define REG_TMST_CONFIG_RESOL_MASK                   (0x1 << 3)

/* GYRO UI FILT ORD */
#define REG_GYRO_CONFIG_GYRO_UI_FILT_ORD_1ST_ORDER   (0x0 << 2)
#define REG_GYRO_CONFIG_GYRO_UI_FILT_ORD_2ND_ORDER   (0x1 << 2)
#define REG_GYRO_CONFIG_GYRO_UI_FILT_ORD_3RD_ORDER   (0x2 << 2)
#define REG_GYRO_CONFIG_GYRO_UI_FILT_ORD_MASK        (0x3 << 2)

/* ACCEL UI FILT ORD */
#define REG_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_1ST_ORDER (0x0 << 3)
#define REG_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_2ND_ORDER (0x1 << 3)
#define REG_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_3RD_ORDER (0x2 << 3)
#define REG_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_MASK      (0x3 << 3)

/* WOM INT MODE */
#define REG_SMD_CONFIG_WOM_INT_MODE_ANDED            (0x1 << 3)
#define REG_SMD_CONFIG_WOM_INT_MODE_ORED             (0x0 << 3)
#define REG_SMD_CONFIG_WOM_INT_MODE_MASK             (0x1 << 3)

/* WOM MODE */
#define REG_SMD_CONFIG_WOM_MODE_CMP_PREV             (0x1 << 2)
#define REG_SMD_CONFIG_WOM_MODE_CMP_INIT             (0x0 << 2)
#define REG_SMD_CONFIG_WOM_MODE_CMP_MASK             (0x1 << 2)

/* PWR MGMT ACCEL MODE */
#define REG_PWR_MGMT_0_ACCEL_MODE_LN                 (0x3 << 0)
#define REG_PWR_MGMT_0_ACCEL_MODE_LP                 (0x2 << 0)
#define REG_PWR_MGMT_0_ACCEL_MODE_OFF                (0x0 << 0)
#define REG_PWR_MGMT_0_ACCEL_MODE_MASK               (0x3 << 0)

/* PWR MGMT GYRO MODE */
#define REG_PWR_MGMT_0_GYRO_MODE_LN                  (0x3 << 2)
#define REG_PWR_MGMT_0_GYRO_MODE_STANDBY             (0x1 << 2)
#define REG_PWR_MGMT_0_GYRO_MODE_OFF                 (0x0 << 2)
#define REG_PWR_MGMT_0_GYRO_MODE_MASK                (0x3 << 2)

/* ACCEL LP CLK SEL */
#define REG_INTF_CONFIG1_ACCEL_LP_WUOC               (0x0 << 3)
#define REG_INTF_CONFIG1_ACCEL_LP_RC                 (0x1 << 3)
#define REG_INTF_CONFIG1_ACCEL_LP_MASK               (0x1 << 3)
#define REG_INTF_CONFIG1_CLKSEL_ALWAYS_RC            (0x0 << 0)
#define REG_INTF_CONFIG1_CLKSEL_PLL_RC               (0x1 << 0)
#define REG_INTF_CONFIG1_CLKSEL_DISABLE              (0x3 << 0)

/* INT_CONFIG_INT1_POLARITY */
#define REG_INT_CONFIG_INT1_POLARITY_HIGH            (0x1 << 0)
#define REG_INT_CONFIG_INT1_POLARITY_LOW             (0x0 << 0)
#define REG_INT_CONFIG_INT1_POLARITY_MASK            (0x1 << 0)

/* INT_CONFIG_INT1_DRIVE_CIRCUIT */
#define REG_INT_CONFIG_INT1_DRIVE_CIRCUIT_OPENDRAIN  (0x0 << 1)
#define REG_INT_CONFIG_INT1_DRIVE_CIRCUIT_PUSHPULL   (0x1 << 1)
#define REG_INT_CONFIG_INT1_DRIVE_CIRCUIT_MASK       (0x1 << 1)

/* INT_CONFIG_INT1_INTERRUPT_MODE */
#define REG_INT_CONFIG_INT1_INTERRUPT_MODE_PULSE     (0x0 << 2)
#define REG_INT_CONFIG_INT1_INTERRUPT_MODE_LATCHED   (0x1 << 2)
#define REG_INT_CONFIG_INT1_INTERRUPT_MODE_MASK      (0x1 << 2)

#define REG_INT_SOURCE0_UI_DRDY_INT1_EN              (0x1 << 3)
#define REG_INT_SOURCE0_UI_DRDY_INT1_EN_MASK         (0x1 << 3)

/* GYRO ACCEL CONFIG0 BandWidth for Accel LPF */
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_BW_40      (0x7 << 4)
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_BW_20      (0x6 << 4)
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_BW_16      (0x5 << 4)
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_BW_10      (0x4 << 4)
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_BW_8       (0x3 << 4)
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_BW_5       (0x2 << 4)
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_BW_4       (0x1 << 4)
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_BW_2       (0x0 << 4)
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_AVG_16     (0x6 << 4)
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_AVG_1      (0x1 << 4)
#define REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_MASK       (0xf << 4)

/* GYRO ACCEL CONFIG0 BandWidth for GYRO LPF */
#define REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_BW_40       (0x7 << 0)
#define REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_BW_20       (0x6 << 0)
#define REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_BW_16       (0x5 << 0)
#define REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_BW_10       (0x4 << 0)
#define REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_BW_8        (0x3 << 0)
#define REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_BW_5        (0x2 << 0)
#define REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_BW_4        (0x1 << 0)
#define REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_BW_2        (0x0 << 0)
#define REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_MASK        (0xf << 0)

#define ICM42605_WHOAMI                (0x42)
#define REG_FSR_MASK                   (0x7 << 5)
#define REG_ODR_MASK                   (0xf << 0)
#define REG_DISABLE_I3C_MASK           (0x3 << 0)
#define REG_RESET_EN_MASK              (0x1 << 0)
#define REG_NO_FSYNC_INT2_MASK         (0x3 << 1)

#define ICM_DEFAULT_WOM_THS_MG         (52 >> 4)
#define ICM_TEMP_NO_FIFO_RES           (132.48f)
#define ICM_TEMP_FIFO_RES              (2.07f)
#define ACCEL_DEFAULT_FSR              (4)
#define GYRO_DEFAULT_FSR               (2000)
#define ACCEL_GYRO_DEFAULT_ODR         1000
#define ACCEL_LP_ODR_TH                (200)

struct icm42605_rtdata {
  struct sns_port port;
  const struct sensor_platform_data *pdata;

  bool gyro_enabled;
  bool acc_enabled;
  uint32_t a_g_odr;
  uint32_t accel_fsr;
  uint32_t gyro_fsr;

  pthread_mutex_t mutex;
  timer_t timer;
  struct work_s work;
  int64_t ts;
};

/* accel and gyro must use same odr, because if enabled with different ords,
 * the sensor data register maybe invalid, so set_odr will operate gyro's and
 * accel's odr register
 */
struct icm42605_odr_info icm42605_accel_gyro_odr[] = {
  /* 12.5hz 80ms */
  {0xB, 12, 80000},
  /* 25hz 40ms */
  {0xA, 25, 40000},
  /* 50hz 20ms */
  {0x9, 50, 20000},
  /* 100hz 10ms */
  {0x8, 100, 10000},
  /* 200hz 5ms */
  {0x7, 200, 5000},
  /* 500hz 2ms */
  {0xF, 500, 2000},
  /* 1khz 1ms */
  {0x6, 1000, 1000},
  /* 2khz 500us */
  {0x5, 2000, 500},
  /* 4khz 250us */
  {0x4, 4000, 250},
  /* 8khz 125us */
  {0x3, 8000, 125},
};

const static struct icm42605_fsr_info icm42605_accel_fsr[] = {
  /* the resolution is 9.8/sensitivity */
  {2,  (0x3 << 5), 0.000598145},
  {4,  (0x2 << 5), 0.001196289},
  {8,  (0x1 << 5), 0.002392578},
  {16, (0x0 << 5), 0.004785156}
};

/* the resolution is 1/sensitivity * DEGREE2RAD */
const static struct icm42605_fsr_info icm42605_gyro_fsr[] = {
  {16,   (0x7 << 5), 0.000476826 * DEGREE2RAD},
  {31,   (0x6 << 5), 0.000953652 * DEGREE2RAD},
  {62,   (0x5 << 5), 0.001907305 * DEGREE2RAD},
  {125,  (0x4 << 5), 0.003816793 * DEGREE2RAD},
  {250,  (0x3 << 5), 0.007633588 * DEGREE2RAD},
  {500,  (0x2 << 5), 0.015267175 * DEGREE2RAD},
  {1000, (0x1 << 5), 0.030487804 * DEGREE2RAD},
  {2000, (0x0 << 5), 0.060975601 * DEGREE2RAD}
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int icm_regs_read(FAR struct sns_port *port, uint8_t reg, FAR void *data, int len)
{
  uint8_t modify;

  modify = port->binfo->bus_type == BUS_SPI ? 0x80 : 0x00;
  return sns_port_read(port, reg | modify, data, len);
}

static inline int icm_reg_write(FAR struct sns_port *port, uint8_t reg, uint8_t reg_data)
{
  return sns_port_write(port, reg, &reg_data, 1);
}

static int icm_reg_update(FAR struct sns_port *port, uint8_t reg, uint8_t mask, uint8_t value)
{
  int ret = 0;
  uint8_t ori;

  ret = icm_regs_read(port, reg, &ori, 1);
  if (ret)
    return ret;

  ori &= ~mask;
  ori |= value & mask;

  return sns_port_write(port, reg, &ori, 1);
}

static int icm42605_find_accel_fsr(uint32_t except)
{
  int i;
  for (i = 0; i < __countof(icm42605_accel_fsr); i++)
    {
      if (except <= icm42605_accel_fsr[i].fsr)
        return i;
    }
  return __countof(icm42605_accel_fsr) -1;
}

static int icm42605_find_gyro_fsr(uint32_t except)
{
  int i;
  for (i = 0; i < __countof(icm42605_gyro_fsr); i++)
    {
      if (except <= icm42605_gyro_fsr[i].fsr)
        return i;
    }
  return __countof(icm42605_gyro_fsr) -1;
}

static int icm42605_set_fsr(FAR struct icm42605_rtdata *rtdata, int type, uint32_t except)
{
  int idx, ret;

  switch (type)
    {
    case SENSOR_TYPE_ACCELEROMETER:
      idx = icm42605_find_accel_fsr(except);
      if (rtdata->accel_fsr == icm42605_accel_fsr[idx].fsr)
        return 0;
      ret = icm_reg_update(&rtdata->port, REG_ACCEL_CONFIG0,
      icm42605_accel_fsr[idx].regval, REG_FSR_MASK);
      if (ret)
        {
          snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_ACCEL_CONFIG0, ret);
          return ret;
        }
      rtdata->accel_fsr = icm42605_accel_fsr[idx].fsr;
      return ret;
    case SENSOR_TYPE_GYROSCOPE:
      idx = icm42605_find_gyro_fsr(except);
      if (rtdata->gyro_fsr == icm42605_gyro_fsr[idx].fsr)
        return 0;
      ret = icm_reg_update(&rtdata->port, REG_GYRO_CONFIG0,
      icm42605_gyro_fsr[idx].regval, REG_FSR_MASK);
      if (ret)
        {
          snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_GYRO_CONFIG0, ret);
          return ret;
        }
      rtdata->gyro_fsr = icm42605_gyro_fsr[idx].fsr;
      return ret;
    case SENSOR_TYPE_TEMPERATURE:
      return 0;
    default:
      return -EINVAL;
    }
}

static int icm42605_find_odr(uint32_t except)
{
  int i;
  for (i = 0; i < __countof(icm42605_accel_gyro_odr); i++)
    {
      if (except <= icm42605_accel_gyro_odr[i].odr)
      return i;
    }
  return __countof(icm42605_accel_gyro_odr) -1;
}

/* keep gyro and accel same odr */
static int icm42605_set_odr(FAR struct icm42605_rtdata *rtdata, uint32_t except)
{
  int ret, idx = icm42605_find_odr(except);

  if (rtdata->a_g_odr == icm42605_accel_gyro_odr[idx].odr)
    return 0;

  ret = icm_reg_update(&rtdata->port, REG_ACCEL_CONFIG0,
    icm42605_accel_gyro_odr[idx].regval, REG_ODR_MASK);
  if (ret)
    {
      snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_ACCEL_CONFIG0, ret);
      return ret;
    }

  ret = icm_reg_update(&rtdata->port, REG_GYRO_CONFIG0,
    icm42605_accel_gyro_odr[idx].regval, REG_ODR_MASK);
  if (ret)
    {
      snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_GYRO_CONFIG0, ret);
      return ret;
    }

  rtdata->a_g_odr = icm42605_accel_gyro_odr[idx].odr;

  return 0;
}

static int icm42605_reset(FAR struct icm42605_rtdata *rtdata)
{
  uint8_t intf_cfg4_reg, intf_cfg6_reg, data;
  int ret;

  /* save registers necessary to perform soft reset while still
   * keeping communication link alive
   */
  ret = icm_reg_write(&rtdata->port, REG_REG_BANK_SEL, 1);
  if (ret)
    {
      snshuberr("icm42605 set reg bank failed:%d\n", ret);
      return ret;
    }

  /* SPI config */
  ret = icm_regs_read(&rtdata->port, REG_INTF_CONFIG4_B1, &intf_cfg4_reg, 1);
  if (ret)
    {
      snshuberr("icm42605 read reg:0x%x failed:%d\n", REG_INTF_CONFIG4_B1, ret);
      return ret;
    }

  /* I3C ennable config */
  ret = icm_regs_read(&rtdata->port, REG_INTF_CONFIG6_B1, &intf_cfg6_reg, 1);
  if (ret)
    {
      snshuberr("icm42605 read reg:0x%x failed:%d\n", REG_INTF_CONFIG6_B1, ret);
      return ret;
    }

  ret = icm_reg_write(&rtdata->port, REG_REG_BANK_SEL, 0);
  if (ret)
    {
      snshuberr("icm42605 set reg bank failed:%d\n", ret);
      return ret;
    }

  /* reset the internal registers and restores the default settings
   * the bit automatically cliear to 0 once the reset is done
   */
  ret = icm_reg_write(&rtdata->port, REG_CHIP_CONFIG, REG_RESET_EN_MASK);
  if (ret)
    {
      snshuberr("icm42605 write reg:0x%x failed:%d\n", REG_CHIP_CONFIG, ret);
      return ret;
    }

  /* wait 1ms for soft reset to be effective before attempting
   * any other register access
   */
  usleep(1000);

  /* clear the int reset Done bit */
  ret = icm_regs_read(&rtdata->port, REG_INT_STATUS, &data, 1);
  if (ret)
    {
      snshuberr("icm42605 read reg:0x%x failed:%d\n", REG_INT_STATUS, ret);
      return ret;
    }

  if (0 == (data & BIT_INT_STATUS_RESET_DONE))
    {
      snshuberr("icm42605 reset chip failed\n");
    }

  ret = icm_reg_write(&rtdata->port, REG_REG_BANK_SEL, 1);
  if (ret)
    {
      snshuberr("icm42605 set reg bank failed:%d\n", ret);
      return ret;
    }

  ret = icm_reg_write(&rtdata->port, REG_INTF_CONFIG4_B1, intf_cfg4_reg);
  if (ret)
    {
      snshuberr("icm42605 write reg:0x%x failed:%d\n", REG_INTF_CONFIG4_B1, ret);
      return ret;
    }

  ret = icm_reg_write(&rtdata->port, REG_INTF_CONFIG6_B1, intf_cfg6_reg);
  if (ret)
    {
      snshuberr("icm42605 write reg:0x%x failed:%d\n", REG_INTF_CONFIG6_B1, ret);
      return ret;
    }

  /* FSYNC and INT2 don't connect to pin9 */
  icm_reg_write(&rtdata->port, REG_INTF_CONFIG5_B1, REG_NO_FSYNC_INT2_MASK);
  if (ret)
    {
      snshuberr("icm42605 write reg:0x%x failed:%d\n", REG_INTF_CONFIG5_B1, ret);
      return ret;
    }

  ret = icm_reg_write(&rtdata->port, REG_REG_BANK_SEL, 0);
  if (ret)
    {
      snshuberr("icm42605 set reg bank failed:%d\n", ret);
      return ret;
    }

  return 0;
}

static int icm42605_configure_smd_wom(FAR struct icm42605_rtdata *rtdata, const uint8_t x_th, const uint8_t y_th, const uint8_t z_th, uint8_t smd_cfg)
{
  int ret;

  ret = icm_reg_write(&rtdata->port, REG_REG_BANK_SEL, 4);
  if (ret)
    {
      snshuberr("icm42605 set reg bank failed:%d\n", ret);
      return ret;
    }

  ret = icm_reg_write(&rtdata->port, REG_ACCEL_WOM_X_THR_B4, x_th);
  if (ret)
    {
      snshuberr("icm42605 write reg:0x%x failed:%d\n", REG_ACCEL_WOM_X_THR_B4, ret);
      return ret;
    }

  ret = icm_reg_write(&rtdata->port, REG_ACCEL_WOM_Y_THR_B4, y_th);
  if (ret)
    {
      snshuberr("icm42605 write reg:0x%x failed:%d\n", REG_ACCEL_WOM_Y_THR_B4, ret);
      return ret;
    }

  ret = icm_reg_write(&rtdata->port, REG_ACCEL_WOM_Z_THR_B4, z_th);
  if (ret)
    {
      snshuberr("icm42605 write reg:0x%x failed:%d\n", REG_ACCEL_WOM_Z_THR_B4, ret);
      return ret;
    }

  ret = icm_reg_write(&rtdata->port, REG_REG_BANK_SEL, 0);
  if (ret)
    {
      snshuberr("icm42605 set reg bank failed:%d\n", ret);
      return ret;
    }

  icm_reg_write(&rtdata->port, REG_SMD_CONFIG, smd_cfg);
  if (ret)
    {
      snshuberr("icm42605 write reg:0x%x failed:%d\n", REG_SMD_CONFIG, ret);
    }
  return ret;
}

static void icm42605_enable_accel_low_power_mode(FAR struct icm42605_rtdata *rtdata)
{
  int ret;

  /* restore filter averaging settings */
  ret = icm_reg_update(&rtdata->port, REG_ACCEL_GYRO_CONFIG0, REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_AVG_16, REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_MASK);
  if (ret)
    snshuberr("icm42605 update reg:0x%x failed\n", REG_ACCEL_GYRO_CONFIG0, ret);

  /* enable/switch the accelermeter in/to low power mode */
  ret = icm_reg_update(&rtdata->port, REG_PWR_MGMT_0, REG_PWR_MGMT_0_ACCEL_MODE_LP, REG_PWR_MGMT_0_ACCEL_MODE_MASK);
  if (ret)
    snshuberr("icm42605 update reg:0x%x failed\n", REG_PWR_MGMT_0, ret);

  usleep(200);
}

static void icm42605_enable_accel_low_noise_mode(FAR struct icm42605_rtdata *rtdata)
{
  int ret;

  /* restore filter bw settings */
  ret = icm_reg_update(&rtdata->port, REG_ACCEL_GYRO_CONFIG0, REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_BW_4, REG_ACCEL_GYRO_CONFIG0_ACCEL_FILT_MASK);
  if (ret)
    snshuberr("icm42605 update reg:0x%x failed\n", REG_ACCEL_GYRO_CONFIG0, ret);

  /* enable/switch the accelermeter in/to low noise mode */
  ret = icm_reg_update(&rtdata->port, REG_PWR_MGMT_0, REG_PWR_MGMT_0_ACCEL_MODE_LN, REG_PWR_MGMT_0_ACCEL_MODE_MASK);
  if (ret)
    snshuberr("icm42605 update reg:0x%x failed\n", REG_PWR_MGMT_0, ret);

  usleep(200);
}

static void icm42605_enable_gyro_low_noise_mode(FAR struct icm42605_rtdata *rtdata)
{
  int ret;

  /* restore filter bw settings */
  ret = icm_reg_update(&rtdata->port, REG_ACCEL_GYRO_CONFIG0, REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_BW_4, REG_ACCEL_GYRO_CONFIG0_GYRO_FILT_MASK);
  if (ret)
    snshuberr("icm42605 update reg:0x%x failed\n", REG_ACCEL_GYRO_CONFIG0, ret);

  /* enable/switch the gyro in/to low noise mode */
  ret = icm_reg_update(&rtdata->port, REG_PWR_MGMT_0, REG_PWR_MGMT_0_GYRO_MODE_LN, REG_PWR_MGMT_0_GYRO_MODE_MASK);
  if (ret)
    snshuberr("icm42605 update reg:0x%x failed\n", REG_PWR_MGMT_0, ret);

  usleep(200);
}

static void icm42605_gyro_enable(FAR struct icm42605_rtdata *rtdata, bool enable)
{
  int ret;

  if (enable)
    {
      icm42605_enable_gyro_low_noise_mode(rtdata);
    }
  else
    {
      /* turns accelermeter off */
      ret = icm_reg_update(&rtdata->port, REG_PWR_MGMT_0, REG_PWR_MGMT_0_GYRO_MODE_OFF, REG_PWR_MGMT_0_GYRO_MODE_MASK);
      if (ret)
        {
          snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_PWR_MGMT_0, ret);
        }
    }
}

static void icm42605_accel_enable(FAR struct icm42605_rtdata * rtdata, bool enable)
{
  int ret;

  if (enable)
    {
      if (rtdata->a_g_odr >= ACCEL_LP_ODR_TH)
        icm42605_enable_accel_low_noise_mode(rtdata);
      else
        icm42605_enable_accel_low_power_mode(rtdata);
    }
  else
    {
      /* turns accelermeter off */
      ret = icm_reg_update(&rtdata->port, REG_PWR_MGMT_0, REG_PWR_MGMT_0_ACCEL_MODE_OFF, REG_PWR_MGMT_0_ACCEL_MODE_MASK);
      if (ret)
        {
          snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_PWR_MGMT_0, ret);
        }

      /* set accelermeter lp mode use wake up oscillator clock */
      ret = icm_reg_update(&rtdata->port, REG_INTF_CONFIG1, REG_INTF_CONFIG1_ACCEL_LP_WUOC, REG_INTF_CONFIG1_ACCEL_LP_MASK);
      if (ret)
        {
          snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_INTF_CONFIG1, ret);
        }
    }
}

static void icm42605_convert_reg_to_event(FAR struct icm42605_rtdata *rtdata,
        FAR const uint8_t *reg_data, FAR struct sensor_event *event, int place)
{
  int16_t raw_data[3];
  int i;

  /* the sensor data is reported in big endian format default */
  for (i = 0; i < 3; i++)
  raw_data[i] = (reg_data[i << 1] << 8) | (uint8_t)reg_data[(i << 1) + 1];

  remap_vector_raw16to32_axis(raw_data, event->data_raw, place);
}

static int icm42605_init_chip(FAR struct icm42605_rtdata *rtdata)
{
  uint8_t who_am_i;
  int ret = 0;

  /* wait some time for icm to be properly supplied power-on reset setup time */
  usleep(3000);

  /* configure serial interface (i2c, i3c, spi), we select i2c */
  ret = icm_reg_write(&rtdata->port, REG_REG_BANK_SEL, 1);
  if (ret)
    {
      snshuberr("icm42605 set reg bank failed:%d\n", ret);
      return ret;
    }

  ret = icm_reg_update(&rtdata->port, REG_INTF_CONFIG6_B1, 0, REG_DISABLE_I3C_MASK);
  if (ret)
    {
      snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_INTF_CONFIG6_B1, ret);
      return ret;
    }

  ret = icm_reg_write(&rtdata->port, REG_REG_BANK_SEL, 0);
  if (ret)
    {
      snshuberr("icm42605 set reg bank failed:%d\n", ret);
      return ret;
    }

  /* reset chip */
  ret = icm42605_reset(rtdata);
  if (ret)
    {
      snshuberr("icm42605 reset failed:%d\n", ret);
      return ret;
    }

  /* check chip id */
  ret = icm_regs_read(&rtdata->port, REG_WHO_AM_I, &who_am_i, 1);
  if (ret)
    {
      snshuberr("icm42605 read reg:0x%x failed:%d\n", REG_WHO_AM_I, ret);
      return ret;
    }

  if (who_am_i != ICM42605_WHOAMI)
    {
      snshuberr("icm42605 bad whoami value:0x%x\n", who_am_i);
      return -ENXIO;
    }

  /* make sure FIFO is disabled */
  ret = icm_reg_write(&rtdata->port, REG_FIFO_CONFIG, REG_FIFO_CONFIG_MODE_BYPASS);
  if (ret)
    {
      snshuberr("icm42605 write reg:0x%x failed:%d\n", REG_FIFO_CONFIG, ret);
      return ret;
    }

  /* deactivate FSYNC by default */
  ret = icm_reg_update(&rtdata->port, REG_FSYNC_CONFIG, REG_FSYNC_CONFIG_U1_SEL_NO, REG_FSYNC_CONFIG_U1_SEL_MASK);
  if (ret)
    {
      snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_FSYNC_CONFIG, ret);
      return ret;
    }

  /* configure timestamp resolution */
  ret = icm_reg_update(&rtdata->port, REG_TMST_CONFIG, REG_TMST_CONFIG_RESOL_16us, REG_TMST_CONFIG_RESOL_MASK);
  if (ret)
    {
      snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_TMST_CONFIG, ret);
      return ret;
    }

  /* enable fifo */

  /* config interrupt pin */
  ret = icm_reg_update(&rtdata->port, REG_INT_CONFIG, REG_INT_CONFIG_INT1_POLARITY_HIGH
     | REG_INT_CONFIG_INT1_DRIVE_CIRCUIT_PUSHPULL,
     REG_INT_CONFIG_INT1_POLARITY_MASK | REG_INT_CONFIG_INT1_DRIVE_CIRCUIT_MASK);
  if (ret)
    {
      snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_INT_CONFIG, ret);
      return ret;
    }

  ret = icm_reg_update(&rtdata->port, REG_INT_SOURCE0, REG_INT_SOURCE0_UI_DRDY_INT1_EN, REG_INT_SOURCE0_UI_DRDY_INT1_EN_MASK);
  if (ret)
    {
      snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_INT_SOURCE0, ret);
      return ret;
    }

  /* set the UI filter order to 2 for both gyro and accel */
  ret = icm_reg_update(&rtdata->port, REG_GYRO_CONFIG1, REG_GYRO_CONFIG_GYRO_UI_FILT_ORD_2ND_ORDER,
     REG_GYRO_CONFIG_GYRO_UI_FILT_ORD_MASK);
  if (ret)
    {
      snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_GYRO_CONFIG1, ret);
      return ret;
    }

  ret = icm_reg_update(&rtdata->port, REG_ACCEL_CONFIG1, REG_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_2ND_ORDER,
     REG_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_MASK);
  if (ret)
    {
      snshuberr("icm42605 update reg:0x%x failed:%d\n", REG_ACCEL_CONFIG1, ret);
      return ret;
    }

  /* FIFO packets are 16bit format by default (high res is disabled) */

  /* configure WOM to compare current sample with the previous sample and
   * to produce signal when all axis exceed 52mg
   * but the smd is not enabled
   */
  ret = icm42605_configure_smd_wom(rtdata,
  ICM_DEFAULT_WOM_THS_MG,
  ICM_DEFAULT_WOM_THS_MG,
  ICM_DEFAULT_WOM_THS_MG,
  REG_SMD_CONFIG_WOM_INT_MODE_ANDED |
  REG_SMD_CONFIG_WOM_MODE_CMP_PREV);
  if (ret)
    {
      snshuberr("icm42605 config smd wom failed:%d\n", ret);
      return ret;
    }

  ret = icm42605_set_fsr(rtdata, SENSOR_TYPE_ACCELEROMETER, ACCEL_DEFAULT_FSR);
  if (ret)
    {
      snshuberr("icm42605 set acccel fsr failed:%d\n", ret);
      return ret;
    }

  ret = icm42605_set_fsr(rtdata, SENSOR_TYPE_GYROSCOPE, GYRO_DEFAULT_FSR);
  if (ret)
    {
      snshuberr("icm42605 set gyro fsr failed:%d\n", ret);
      return ret;
    }

  icm42605_set_odr(rtdata, ACCEL_GYRO_DEFAULT_ODR);
  if (ret)
    {
      snshuberr("icm42605 set accel and gyro odr failed:%d\n", ret);
    }

  return ret;
}

static void icm42605_worker(FAR void *data)
{
  FAR struct icm42605_rtdata *rtdata = data;
  FAR const struct icm42605_platform_data *spdata = rtdata->pdata->spdata;
  int64_t timestamp = rtdata->ts;
  struct sensor_event event[2];
  uint8_t reg_data[6];
  uint8_t num = 0, int_status = 0;
  uint16_t raw_temp = 0;
  int ret, idx;

  /* ensure data ready status bit is set */
  ret = icm_regs_read(&rtdata->port, REG_INT_STATUS, &int_status, 1);

  if (spdata->irq_pin)
    sensor_enable_gpio_irq(spdata->irq_pin, spdata->trigger_type);

  if (ret < 0)
    {
      snshuberr("icm42605 read reg:0x%x failed:%d\n", REG_INT_STATUS, ret);
      return;
    }

  /* read temperature raw data */
  ret = icm_regs_read(&rtdata->port, REG_TEMP_DATA0_UI, reg_data, 2);
  if (!ret)
    {
      raw_temp = (reg_data[0] << 8) | reg_data[1];
    }

  if (rtdata->acc_enabled && int_status & BIT_INT_STATUS_DRDY)
    {
      ret = icm_regs_read(&rtdata->port, REG_ACCEL_DATA_X0_UI, reg_data, 6);
      if (!ret)
        {
          icm42605_convert_reg_to_event(rtdata, reg_data, &event[0], spdata->place);
          idx = icm42605_find_accel_fsr(rtdata->accel_fsr);
          event[num].data[0] = event[num].data_raw[0] * icm42605_accel_fsr[idx].resolution;
          event[num].data[1] = event[num].data_raw[1] * icm42605_accel_fsr[idx].resolution;
          event[num].data[2] = event[num].data_raw[2] * icm42605_accel_fsr[idx].resolution;
          event[num].data[3] = raw_temp / ICM_TEMP_NO_FIFO_RES + 25.f;
          event[num].type = SENSOR_TYPE_ACCELEROMETER;
          event[num].timestamp = timestamp;
          num++;
        }
    }

  if (rtdata->gyro_enabled && int_status & BIT_INT_STATUS_DRDY)
    {
      ret = icm_regs_read(&rtdata->port, REG_GYRO_DATA_X0_UI, reg_data, 6);
      if (!ret)
        {
          icm42605_convert_reg_to_event(rtdata, reg_data, &event[1], spdata->place);
          idx = icm42605_find_gyro_fsr(rtdata->gyro_fsr);
          event[num].data[0] = event[num].data_raw[0] * icm42605_gyro_fsr[idx].resolution;
          event[num].data[1] = event[num].data_raw[1] * icm42605_gyro_fsr[idx].resolution;
          event[num].data[2] = event[num].data_raw[2] * icm42605_gyro_fsr[idx].resolution;
          event[num].data[3] = raw_temp / ICM_TEMP_NO_FIFO_RES + 25.f;
          event[num].type = SENSOR_TYPE_GYROSCOPE;
          event[num].timestamp = timestamp;
          num++;
        }
    }

  if (num > 0)
  smgr_push_data(event, num);
}

int icm42605_activate(FAR const struct sensor_dev *dev, bool enable, snshub_data_mode mode)
{
  FAR struct icm42605_rtdata *rtdata = dev->rtdata;
  FAR const struct icm42605_platform_data *spdata = rtdata->pdata->spdata;
  bool old_status, new_status;

  /* XXX: just non-fifo mode for now, later will create a fifo procedure */
  pthread_mutex_lock(&rtdata->mutex);

  snshubinfo("icm: activate %s to %s\n", dev->name, enable ? "enable" : "disable");

  old_status = rtdata->acc_enabled || rtdata->gyro_enabled;
  if (dev->type == SENSOR_TYPE_ACCELEROMETER && rtdata->acc_enabled != enable)
    {
      icm42605_accel_enable(rtdata, enable);
      rtdata->acc_enabled = enable;
    }
  else if (dev->type == SENSOR_TYPE_GYROSCOPE && rtdata->gyro_enabled != enable)
    {
      icm42605_gyro_enable(rtdata, enable);
      rtdata->gyro_enabled = enable;
    }

  if (mode != ACTIVE_READING)
    {
      new_status = rtdata->acc_enabled || rtdata->gyro_enabled;
      if (old_status != new_status)
        {
          if (new_status)
            {
              if (rtdata->timer)
                sensor_timer_start(true, &rtdata->timer, 1000 / rtdata->a_g_odr);
              else
                sensor_enable_gpio_irq(spdata->irq_pin, spdata->trigger_type);
            }
          else
            {
              if (rtdata->timer)
                sensor_timer_start(false, &rtdata->timer, 0);
              else
                sensor_enable_gpio_irq(spdata->irq_pin, IOEXPANDER_VAL_DISABLE);
            }
        }
    }
  else
    {
      if (rtdata->timer)
        sensor_timer_start(false, &rtdata->timer, 0);
      else
        sensor_enable_gpio_irq(spdata->irq_pin, IOEXPANDER_VAL_DISABLE);
    }

  pthread_mutex_unlock(&rtdata->mutex);

  return 0;
}

int icm42605_read_data(FAR const struct sensor_dev *dev, FAR struct sensor_event *event)
{
  uint8_t int_status, idx;
  uint8_t temperature_raw[2];
  uint8_t reg_raw[6];
  uint16_t temp_raw;
  uint64_t stamp = get_timestamp();
  FAR struct icm42605_rtdata *rtdata = dev->rtdata;
  FAR const struct icm42605_platform_data *spdata = rtdata->pdata->spdata;
  int ret;

  /* ensure data ready status bit is set */
  ret = icm_regs_read(&rtdata->port, REG_INT_STATUS, &int_status, 1);
  if (ret)
    {
      snshuberr("icm42605 read reg:0x%x failed:%d,%d\n", REG_INT_STATUS, ret, __LINE__);
      return ret;
    }

  /* read temperature raw data */
  ret = icm_regs_read(&rtdata->port, REG_TEMP_DATA0_UI, temperature_raw, 2);
  if (ret)
    {
      snshuberr("icm42605 read reg:0x%x failed:%d\n", REG_TEMP_DATA0_UI, ret);
      return ret;
    }

  temp_raw = (temperature_raw[0] << 8) | temperature_raw[1];

  if (int_status & BIT_INT_STATUS_DRDY)
    {
      if (rtdata->acc_enabled && dev->type == SENSOR_TYPE_ACCELEROMETER)
        {
          ret = icm_regs_read(&rtdata->port, REG_ACCEL_DATA_X0_UI, reg_raw, 6);
          if (ret)
            {
              snshuberr("icm42605 read reg:0x%x failed:%d\n", REG_ACCEL_DATA_X0_UI, ret);
              return ret;
            }
          icm42605_convert_reg_to_event(rtdata, reg_raw, event, spdata->place);
          idx = icm42605_find_accel_fsr(rtdata->accel_fsr);
          event->data[0] = event->data_raw[0] * icm42605_accel_fsr[idx].resolution;
          event->data[1] = event->data_raw[1] * icm42605_accel_fsr[idx].resolution;
          event->data[2] = event->data_raw[2] * icm42605_accel_fsr[idx].resolution;
          event->data[3] = temp_raw / ICM_TEMP_NO_FIFO_RES + 25.f;
          event->type = SENSOR_TYPE_ACCELEROMETER;
          event->timestamp = stamp;
        }
      if (rtdata->gyro_enabled && dev->type == SENSOR_TYPE_GYROSCOPE)
        {
          memcpy(reg_raw, 0, sizeof(reg_raw));
          icm_regs_read(&rtdata->port, REG_GYRO_DATA_X0_UI, reg_raw, 6);
          if (ret)
            {
              snshuberr("icm42605 read reg:0x%x failed:%d\n", REG_GYRO_DATA_X0_UI, ret);
              return ret;
            }
          icm42605_convert_reg_to_event(rtdata, reg_raw, event, spdata->place);
          idx = icm42605_find_gyro_fsr(rtdata->gyro_fsr);
          event->data[0] = event->data_raw[0] * icm42605_gyro_fsr[idx].resolution;
          event->data[1] = event->data_raw[1] * icm42605_gyro_fsr[idx].resolution;
          event->data[2] = event->data_raw[2] * icm42605_gyro_fsr[idx].resolution;
          event->data[3] = temp_raw / ICM_TEMP_NO_FIFO_RES + 25.f;
          event->type = SENSOR_TYPE_GYROSCOPE;
          event->timestamp = stamp;
        }
    }
  return 0;
}

static int icm42605_set_delay(FAR const struct sensor_dev *dev, uint32_t us)
{
  int ret;
  FAR struct icm42605_rtdata *rtdata = dev->rtdata;

  pthread_mutex_lock(&rtdata->mutex);

  ret = icm42605_set_odr(dev->rtdata, 1000000 / us);

  pthread_mutex_unlock(&rtdata->mutex);

  return ret;
}

static int icm42605_get_min_delay(FAR const struct sensor_dev *dev)
{
  return icm42605_accel_gyro_odr[__countof(icm42605_accel_gyro_odr) - 1].times;
}

static int icm42605_get_max_delay(FAR const struct sensor_dev *dev)
{
  return icm42605_accel_gyro_odr[0].times;
}

static float icm42605_get_power(FAR const struct sensor_dev *dev)
{
  if (dev->type == SENSOR_TYPE_ACCELEROMETER)
    return 0.05;
  else
    return 1.0;
}

static int icm42605_selftest(FAR const struct sensor_dev *dev, void *data)
{
  return 0;
}

static int icm42605_set_offset(FAR const struct sensor_dev *dev, void *offset)
{
  return 0;
}

static int icm42605_get_offset(FAR const struct sensor_dev *dev, void *offset)
{
  return 0;
}

static FAR char* icm42605_get_vendor(FAR const struct sensor_dev *dev)
{
  return "invensense";
}

static FAR char* icm42605_get_module(FAR const struct sensor_dev *dev)
{
  return "icm42605";
}

static float icm42605_get_max_range(FAR const struct sensor_dev *dev)
{
  if (dev->type == SENSOR_TYPE_ACCELEROMETER)
    return icm42605_accel_fsr[__countof(icm42605_accel_fsr) - 1].fsr* GRAVITY;
  else
    return icm42605_gyro_fsr[__countof(icm42605_gyro_fsr) - 1].fsr;
}

static float icm42605_get_resolution(FAR const struct sensor_dev *dev)
{
  FAR struct icm42605_rtdata *rtdata = dev->rtdata;
  int idx;

  if (dev->type == SENSOR_TYPE_ACCELEROMETER)
    {
      idx = icm42605_find_accel_fsr(rtdata->accel_fsr);
      return icm42605_accel_fsr[idx].resolution;
    }
  else
    {
      idx = icm42605_find_gyro_fsr(rtdata->gyro_fsr);
      return icm42605_gyro_fsr[idx].resolution;
    }
}

static int icm42605_irq_handler(FAR struct ioexpander_dev_s *dev,
                ioe_pinset_t pinset, FAR void *data)
{
  FAR struct icm42605_rtdata *rtdata = data;
  FAR const struct icm42605_platform_data *spdata = rtdata->pdata->spdata;

  rtdata->ts = get_timestamp();
  /* this is an IRQ context, we must use the pended worker
   * since the i2c access will be blocked */
  work_queue(LPWORK, &rtdata->work, icm42605_worker, rtdata, 0);
  /* XXX: because it's a level triggered interrupt, the handler would be called
   * frequetly until the pended handler is executed to read fifo data register.
   * so we need to temperarily disable the gpio interrupt and then re-enable it
   * in the pended handler */
  sensor_enable_gpio_irq(spdata->irq_pin, IOEXPANDER_VAL_DISABLE);
  return 0;
}

static void icm42605_timer_cb(union sigval arg)
{
  struct icm42605_rtdata *rtdata = arg.sival_ptr;

  rtdata->ts = get_timestamp();

  work_queue(LPWORK, &rtdata->work, icm42605_worker, rtdata, 0);
}

struct sensor_ops icm_ops = {
  .activate = icm42605_activate,
  .set_delay = icm42605_set_delay,
  .selftest = icm42605_selftest,
  .set_offset = icm42605_set_offset,
  .get_offset = icm42605_get_offset,
  .get_vendor = icm42605_get_vendor,
  .get_module = icm42605_get_module,
  .get_max_range = icm42605_get_max_range,
  .get_power = icm42605_get_power,
  .get_resolution = icm42605_get_resolution,
  .get_max_delay = icm42605_get_max_delay,
  .get_min_delay = icm42605_get_min_delay,
  .read_data = icm42605_read_data,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int icm42605_probe(FAR const struct sensor_platform_data *pdata,
      FAR const struct sensor_matching_data *mdata,
      FAR struct sensor_dev **dev, FAR uint32_t *num_sensors)
{
  FAR struct sensor_dev *sdevs;
  FAR struct icm42605_rtdata *rtdata;
  FAR const struct icm42605_platform_data *spdata = pdata->spdata;
  size_t msize;
  int ret;
  FAR void *handle;

  msize = sizeof(*sdevs) * 2 + sizeof(struct icm42605_rtdata);
  sdevs = calloc(1, msize);
  if (!sdevs)
    {
      snshuberr("failed to calloc memory for icm42605\n");
      return -ENOMEM;
    }

  rtdata = (struct icm42605_rtdata *)&sdevs[2];
  rtdata->pdata = pdata;

  ret = sns_port_init(&pdata->bus_info, &rtdata->port);
  if (ret)
    {
      snshuberr("failed to init port for %s\n", pdata->name);
      goto port_err;
    }

  ret = icm42605_init_chip(rtdata);
  if (ret)
    {
      snshuberr("failed to init for icm42605\n");
      goto init_err;
    }

  ret = pthread_mutex_init(&rtdata->mutex, NULL);
  if (ret)
    {
      snshuberr("failed to create pthread mutex, ret:%d\n", ret);
      goto mutex_err;
    }

  if (spdata->irq_pin)
    {
      handle = sensor_register_gpio_irq(spdata->irq_pin, spdata->trigger_type,
                  icm42605_irq_handler, rtdata);
      if (handle == NULL) {
        snshuberr("failed to register irq handler for icm42605\n");
        goto gpio_irq_err;
      }
    }
  else
    {
      ret = sensor_timer_init(&rtdata->timer, icm42605_timer_cb, rtdata);
      if (ret)
        {
          snshuberr("failed to create timer for icm42605 sensor\n");
          ret = -ETIME;
          goto timer_err;
        }
    }

  init_sensor_dev(&sdevs[0], ACCEL_SENSOR, SENSOR_TYPE_ACCELEROMETER,
    &icm_ops, pdata, rtdata, mdata);
  init_sensor_dev(&sdevs[1], GYRO_SENSOR, SENSOR_TYPE_GYROSCOPE,
    &icm_ops, pdata, rtdata, mdata);

  *dev = sdevs;
  *num_sensors = 2;

  snshubinfo("2 sensors probed for icm42605\n");

  return ret;

timer_err:
gpio_irq_err:
  pthread_mutex_destroy(&rtdata->mutex);
mutex_err:
init_err:
  sns_port_deinit(&rtdata->port);
port_err:
  free(sdevs);

  return ret;
}

const static struct sensor_matching_data icm42605_mdata = {
  .name = "invn,icm42605",
  .priv = NULL,
};

const static struct sensor_matching_data *drv_mdata[] = {
  &icm42605_mdata,
  NULL,
};

/****************************************************************************
 * Public
 ****************************************************************************/

const struct sensor_driver icm42605_drv = {
  .name = "icm42605",
  .mdata = &drv_mdata[0],
  .probe = icm42605_probe,
};
