#ifndef __LIS3DH_SPI_H
#define __LIS3DH_SPI_H

/*****************************************************************************
  * Pre-processor Definitions
  ****************************************************************************/
#define LIS3DH_VER_WHO_AM_I   (0x33)

#define LIS3DH_STATUS_REG_AUX (0x07)
#define LIS3DH_OUT_ADC1_L     (0x08)
#define LIS3DH_OUT_ADC1_H     (0x09)
#define LIS3DH_OUT_ADC2_L     (0x0A)
#define LIS3DH_OUT_ADC2_H     (0x0B)
#define LIS3DH_OUT_ADC3_L     (0x0C)
#define LIS3DH_OUT_ADC3_H     (0x0D)
#define LIS3DH_WHO_AM_I       (0x0F)
#define LIS3DH_CTRL_REG0      (0x1E)
#define LIS3DH_TEMP_CFG_REG   (0x1F)
#define LIS3DH_CTRL_REG1      (0x20)
#define LIS3DH_CTRL_REG2      (0x21)
#define LIS3DH_CTRL_REG3      (0x22)
#define LIS3DH_CTRL_REG4      (0x23)
#define LIS3DH_CTRL_REG5      (0x24)
#define LIS3DH_CTRL_REG6      (0x25)
#define LIS3DH_REFERENCE      (0x26)
#define LIS3DH_STATUS_REG     (0x27)
#define LIS3DH_OUT_X_L        (0x28)
#define LIS3DH_OUT_X_H        (0x29)
#define LIS3DH_OUT_Y_L        (0x2A)
#define LIS3DH_OUT_Y_H        (0x2B)
#define LIS3DH_OUT_Z_L        (0x2C)
#define LIS3DH_OUT_Z_H        (0x2D)
#define LIS3DH_FIFO_CTRL_REG  (0x2E)
#define LIS3DH_FIFO_SRC_REG   (0x2F)
#define LIS3DH_INT1_CFG       (0x30)
#define LIS3DH_INT1_SRC       (0x31)
#define LIS3DH_INT1_THS       (0x32)
#define LIS3DH_INT1_DURATION  (0x33)
#define LIS3DH_INT2_CFG       (0x34)
#define LIS3DH_INT2_SRC       (0x35)
#define LIS3DH_INT2_THS       (0x36)
#define LIS3DH_INT2_DURATION  (0x37)
#define LIS3DH_CLICK_THS      (0x3A)
#define LIS3DH_TIME_LIMIT     (0x3B)
#define LIS3DH_TIME_LATENCY   (0x3C)
#define LIS3DH_TIME_WINDOW    (0x3D)
#define LIS3DH_ACT_THS        (0x3E)
#define LIS3DH_ACT_DUR        (0x3F)

#define LIS3DH_STATUS_REG_AUX_321OR       (1 << 7)
#define LIS3DH_STATUS_REG_AUX_3OR         (1 << 6)
#define LIS3DH_STATUS_REG_AUX_2OR         (1 << 5)
#define LIS3DH_STATUS_REG_AUX_1OR         (1 << 4)
#define LIS3DH_STATUS_REG_AUX_321DA       (1 << 3)
#define LIS3DH_STATUS_REG_AUX_3DA         (1 << 2)
#define LIS3DH_STATUS_REG_AUX_2DA         (1 << 1)
#define LIS3DH_STATUS_REG_AUX_1DA         (1 << 0)

#define LIS3DH_CTRL_REG0_SDO_PU_DISC      (1 << 7)  /* Disconnect SDO/SA0 pull-up */
#define LIS3DH_TEMP_CFG_REG_ADC_EN        (1 << 7)  /* ADC enable */
#define LIS3DH_TEMP_CFG_REG_TEMP_EN       (1 << 6)  /* Temperator sensor enable */

#define LIS3DH_CTRL_REG1_ODR_SHIFT        (4)
#define LIS3DH_CTRL_REG1_ODR_MASK         (0xf << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR(n)           ((n) << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_POWER_DOWN   (0)
#define LIS3DH_CTRL_REG1_ODR_1Hz          (0x1 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_10Hz         (0x2 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_25Hz         (0x3 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_50Hz         (0x4 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_100Hz        (0x5 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_200Hz        (0x6 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_400Hz        (0x7 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_LP_1600Hz    (0x8 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_1344Hz       (0x9 << LIS3DH_CTRL_REG1_ODR_SHIFT)
#define LIS3DH_CTRL_REG1_ODR_LP_5376Hz    (0x9 << LIS3DH_CTRL_REG1_ODR_SHIFT)

#define LIS3DH_CTRL_REG1_LPEN             (1 << 3)
#define LIS3DH_CTRL_REG1_ZEN              (1 << 2)
#define LIS3DH_CTRL_REG1_YEN              (1 << 1)
#define LIS3DH_CTRL_REG1_XEN              (1 << 0)

#define LIS3DH_CTRL_REG3_I1_CLICK         (1 << 6)
#define LIS3DH_CTRL_REG3_I1_IA1           (1 << 6)
#define LIS3DH_CTRL_REG3_I1_IA2           (1 << 5)
#define LIS3DH_CTRL_REG3_I1_ZYXDA         (1 << 4)
#define LIS3DH_CTRL_REG3_I1_321DA         (1 << 3)
#define LIS3DH_CTRL_REG3_I1_WTM           (1 << 2)
#define LIS3DH_CTRL_REG3_I1_OVERRUN       (1 << 1)

#define LIS3DH_CTRL_REG4_BDU              (1 << 7)
#define LIS3DH_CTRL_REG4_BLE              (1 << 6)
#define LIS3DH_CTRL_REG4_FS_16G           (3 << 4)
#define LIS3DH_CTRL_REG4_FS_8G            (2 << 4)
#define LIS3DH_CTRL_REG4_FS_4G            (1 << 4)
#define LIS3DH_CTRL_REG4_FS_2G            (0 << 4)
#define LIS3DH_CTRL_REG4_HR               (1 << 3)
#define LIS3DH_CTRL_REG4_ST1              (2 << 1)
#define LIS3DH_CTRL_REG4_ST0              (1 << 1)
#define LIS3DH_CTRL_REG4_SIM              (1 << 0)

#define LIS3DH_CTRL_REG5_BOOT             (1 << 7)
#define LIS3DH_CTRL_REG5_FIFO_EN          (1 << 6)
#define LIS3DH_CTRL_REG5_LIR_INT1         (1 << 3)
#define LIS3DH_CTRL_REG5_D4D_INT1         (1 << 2)
#define LIS3DH_CTRL_REG5_LIR_INT2         (1 << 1)
#define LIS3DH_CTRL_REG5_D4D_INT2         (1 << 0)

#define LIS3DH_CTRL_REG6_I2_CLICK         (1 << 6)
#define LIS3DH_CTRL_REG6_I2_IA1           (1 << 6)
#define LIS3DH_CTRL_REG6_I2_IA2           (1 << 5)
#define LIS3DH_CTRL_REG6_I2_BOOT          (1 << 4)
#define LIS3DH_CTRL_REG6_I2_ACT           (1 << 3)
#define LIS3DH_CTRL_REG6_INT_POLARITY     (1 << 1)

#define LIS3DH_STATUS_ZYXOR               (1 << 7)
#define LIS3DH_STATUS_ZOR                 (1 << 6)
#define LIS3DH_STATUS_YOR                 (1 << 5)
#define LIS3DH_STATUS_XOR                 (1 << 4)
#define LIS3DH_STATUS_REG_ZYXDA           (1 << 3)
#define LIS3DH_STATUS_REG_ZDA             (1 << 2)
#define LIS3DH_STATUS_REG_YDA             (1 << 1)
#define LIS3DH_STATUS_REG_XDA             (1 << 0)
/*stream2 means stream to fifo*/
#define LIS3DH_FIFO_CTRL_REG_MODE_STREAM2 (3 << 6)
#define LIS3DH_FIFO_CTRL_REG_MODE_STREAM  (2 << 6)
#define LIS3DH_FIFO_CTRL_REG_MODE_FIFO    (1 << 6)
#define LIS3DH_FIFO_CTRL_REG_MODE_BYPASS  (0 << 6)
#define LIS3DH_FIFO_CTRL_REG_TR           (1 << 5)

#define LIS3DH_FIFO_SRC_REG_WTM           (1 << 7)
#define LIS3DH_FIFO_SRC_REG_OVRN_FIFO     (1 << 6)
#define LIS3DH_FIFO_SRC_REG_EMPTY         (1 << 5)

#define SPIDEV_MODE0                      ( 0 )
#define SPIDEV_MODE1                      ( 1 )
#define SPIDEV_MODE2                      ( 2 )
#define SPIDEV_MODE3                      ( 3 )

#define LIS3DH_SPI_FREQUENCY              ( 1000000 )
#define LIS3DH_SPI_MODE                   ( SPIDEV_MODE3 )

#define LIS3DH_POWER_MODE_LOW             ( 0x0 )
#define LIS3DH_POWER_MODE_NORMAL          ( 0x1 )
#define LIS3DH_POWER_MODE_HIGH            ( 0x2 )

#define LIS3DH_ODR_POWER_DOWN             ( 0x0 )
#define LIS3DH_ODR_1HZ                    ( 0x1 )
#define LIS3DH_ODR_10HZ                   ( 0x2 )
#define LIS3DH_ODR_25HZ                   ( 0x3 )
#define LIS3DH_ODR_50HZ                   ( 0x4 )
#define LIS3DH_ODR_100HZ                  ( 0x5 )
#define LIS3DH_ODR_200HZ                  ( 0x6 )
#define LIS3DH_ODR_400HZ                  ( 0x7 )
#define LIS3DH_ODR_LP_1600HZ              ( 0x8 )
#define LIS3DH_ODR_1344HZ                 ( 0x9 )
#define LIS3DH_ODR_LP_5376HZ              ( 0x9 )
#define GRAVITY                           ( 9.80665f )
#define ACCEL_SENSOR                      "accelerometer"
#define ADDRESS_AUTO_INCREASE_READ        ( 0xC0 )

typedef struct
{
  uint8_t xda:1;
  uint8_t yda:1;
  uint8_t zda:1;
  uint8_t xyzda:1;
  uint8_t _xor:1;
  uint8_t yor:1;
  uint8_t zor:1;
  uint8_t xyzor:1;
} lis3dh_status_reg;

typedef struct
{
  uint8_t sim:1;
  uint8_t st:2;
  uint8_t hr:1;
  uint8_t fs:2;
  uint8_t ble:1;
  uint8_t bdu:1;
} lis3dh_ctrl_reg4;

typedef struct
{
  uint8_t fth:5;
  uint8_t tr:1;
  uint8_t fm:2;
}lis3dh_fifo_ctrl_reg;

typedef struct {
  uint8_t hp:3; /* HPCLICK + HP_IA2 + HP_IA1 -> HP */
  uint8_t fds:1;
  uint8_t hpcf:2;
  uint8_t hpm:2;
} lis3dh_ctrl_reg2_t;

typedef enum {
  LIS3DH_DISC_FROM_INT_GENERATOR = 0,
  LIS3DH_ON_INT1_GEN = 1,
  LIS3DH_ON_INT2_GEN = 2,
  LIS3DH_ON_TAP_GEN = 4,
  LIS3DH_ON_INT1_INT2_GEN = 3,
  LIS3DH_ON_INT1_TAP_GEN = 5,
  LIS3DH_ON_INT2_TAP_GEN = 6,
  LIS3DH_ON_INT1_INT2_TAP_GEN = 7,
} lis3dh_hp_t;

typedef struct
{
  uint8_t ths:7;
  uint8_t unused:1;
}lis3dh_init1_ths;

typedef struct
{
  uint8_t xlie:1;
  uint8_t xhie:1;
  uint8_t ylie:1;
  uint8_t yhie:1;
  uint8_t zlie:1;
  uint8_t zhie:1;
  uint8_t _6d:1;
  uint8_t aoi:1;
}lis3dh_int1_config;

typedef union
{
  uint8_t all;
  lis3dh_int1_config config_bit;
}lis3dh_init_config_union;

enum lis3dh_readmode
{
  fifo_mode = 3,
  stream_mode = 2,
  stream_t_fifo_mode = 1,
  bypass_mode = 0,
};

typedef enum
{
  lis3dh_aux_disable = 0,
  lis3dh_aux_temperature = 1,
  lis3dh_aux_on_pads = 2,
}lis3dh_temp_en;

struct lis3dh_odr_info {
  uint8_t regval;
  /* the unit is hz */
  uint32_t odr;
  /* the unit is us */
  uint32_t times;
};

struct lis3dh_fsr_info {
  uint32_t fsr;
  uint8_t regval;
  float resolution;
};

struct lis3dh_platform_data {
  int irq_pin;
  int trigger_type;
  int place;
};
#endif/* __LIS3DH_SPI_H */
