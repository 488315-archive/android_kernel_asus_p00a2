/*!
 * @section LICENSE
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename bma2x2.c
 * @date    2015/01/20 10:32
 * @id       "08b44b4"
 * @version  2.1.2
 *
 * @brief
 * This file contains all function implementations for the BMA2X2 in linux
*/

#ifdef CONFIG_SIG_MOTION
#undef CONFIG_HAS_EARLYSUSPEND
#endif
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <asm/irq.h>

#include <accel.h>
#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include <linux/io.h>

#include <linux/gpio.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <asm/uaccess.h>
#include <asm/setup.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/of_irq.h>
#include <linux/fs.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif

#include "bstclass.h"
#include "bs_log.h"

#define ACC_NAME  "ACC"
//#define CONFIG_BMA_ENABLE_NEWDATA_INT 1

//#define LDBG(s,args...)	{printk(KERN_ERR "[BMA2x2] : func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#define SENSOR_NAME                 "bma2x2"
#define ABSMIN                      -512
#define ABSMAX                      512
#define SLOPE_THRESHOLD_VALUE       32
#define SLOPE_DURATION_VALUE        1
#define INTERRUPT_LATCH_MODE        13
#define INTERRUPT_ENABLE            1
#define INTERRUPT_DISABLE           0
#define MAP_SLOPE_INTERRUPT         2
#define SLOPE_X_INDEX               5
#define SLOPE_Y_INDEX               6
#define SLOPE_Z_INDEX               7
#define BMA2X2_MAX_DELAY            200

#define I2C_RETRY_DELAY()           usleep_range(1000, 2000)
/* wait 2ms for calibration ready */
#define WAIT_CAL_READY()            usleep_range(2000, 2500)
/* >3ms wait device ready */
#define WAIT_DEVICE_READY()         usleep_range(3000, 5000)
/* >5ms for device reset */
#define RESET_DELAY()               usleep_range(5000, 10000)
/* wait 10ms for self test  done */
#define SELF_TEST_DELAY()           usleep_range(10000, 15000)

#define LOW_G_INTERRUPT             REL_Z
#define HIGH_G_INTERRUPT            REL_HWHEEL
#define SLOP_INTERRUPT              REL_DIAL
#define DOUBLE_TAP_INTERRUPT        REL_WHEEL
#define SINGLE_TAP_INTERRUPT        REL_MISC
#define UNKNOW_TAP_INTERRUPT        REL_MAX
#define ORIENT_INTERRUPT            ABS_PRESSURE
#define FLAT_INTERRUPT              ABS_DISTANCE
#define SLOW_NO_MOTION_INTERRUPT    REL_Y
/* Add by Tom for Zen Motion*/
#define MOTION_FLICK_INTERRUPT      ABS_GAS
#define MOTION_FLICK_HV_INTERRUPT      ABS_WHEEL
#define MOTION_TERMINAL_INTERRUPT   ABS_PRESSURE

#define HIGH_G_INTERRUPT_X_HAPPENED                 1
#define HIGH_G_INTERRUPT_Y_HAPPENED                 2
#define HIGH_G_INTERRUPT_Z_HAPPENED                 3
#define HIGH_G_INTERRUPT_X_NEGATIVE_HAPPENED        4
#define HIGH_G_INTERRUPT_Y_NEGATIVE_HAPPENED        5
#define HIGH_G_INTERRUPT_Z_NEGATIVE_HAPPENED        6
#define SLOPE_INTERRUPT_X_HAPPENED                  7
#define SLOPE_INTERRUPT_Y_HAPPENED                  8
#define SLOPE_INTERRUPT_Z_HAPPENED                  9
#define SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED         10
#define SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED         11
#define SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED         12
#define DOUBLE_TAP_INTERRUPT_HAPPENED               13
#define SINGLE_TAP_INTERRUPT_HAPPENED               14
#define UPWARD_PORTRAIT_UP_INTERRUPT_HAPPENED       15
#define UPWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED     16
#define UPWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED    17
#define UPWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED   18
#define DOWNWARD_PORTRAIT_UP_INTERRUPT_HAPPENED     19
#define DOWNWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED   20
#define DOWNWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED  21
#define DOWNWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED 22
#define FLAT_INTERRUPT_TURE_HAPPENED                23
#define FLAT_INTERRUPT_FALSE_HAPPENED               24
#define LOW_G_INTERRUPT_HAPPENED                    25
#define SLOW_NO_MOTION_INTERRUPT_HAPPENED           26

/* Add by Tom for Skip Same Terminal Event */
#define TERMINAL_DOWNWARD     0
#define TERMINAL_UPWARD       1
#define TERMINAL_UNKNOWN      2

#define PAD_LOWG                    0
#define PAD_HIGHG                   1
#define PAD_SLOP                    2
#define PAD_DOUBLE_TAP              3
#define PAD_SINGLE_TAP              4
#define PAD_ORIENT                  5
#define PAD_FLAT                    6
#define PAD_SLOW_NO_MOTION          7

#define BMA2X2_EEP_OFFSET                       0x16
#define BMA2X2_IMAGE_BASE                       0x38
#define BMA2X2_IMAGE_LEN                        22

#define BMA2X2_CHIP_ID_REG                      0x00
#define BMA2X2_VERSION_REG                      0x01
#define BMA2X2_X_AXIS_LSB_REG                   0x02
#define BMA2X2_X_AXIS_MSB_REG                   0x03
#define BMA2X2_Y_AXIS_LSB_REG                   0x04
#define BMA2X2_Y_AXIS_MSB_REG                   0x05
#define BMA2X2_Z_AXIS_LSB_REG                   0x06
#define BMA2X2_Z_AXIS_MSB_REG                   0x07
#define BMA2X2_TEMPERATURE_REG                  0x08
#define BMA2X2_STATUS1_REG                      0x09
#define BMA2X2_STATUS2_REG                      0x0A
#define BMA2X2_STATUS_TAP_SLOPE_REG             0x0B
#define BMA2X2_STATUS_ORIENT_HIGH_REG           0x0C
#define BMA2X2_STATUS_FIFO_REG                  0x0E
#define BMA2X2_RANGE_SEL_REG                    0x0F
#define BMA2X2_BW_SEL_REG                       0x10
#define BMA2X2_MODE_CTRL_REG                    0x11
#define BMA2X2_LOW_NOISE_CTRL_REG               0x12
#define BMA2X2_DATA_CTRL_REG                    0x13
#define BMA2X2_RESET_REG                        0x14
#define BMA2X2_INT_ENABLE1_REG                  0x16
#define BMA2X2_INT_ENABLE2_REG                  0x17
#define BMA2X2_INT_SLO_NO_MOT_REG               0x18
#define BMA2X2_INT1_PAD_SEL_REG                 0x19
#define BMA2X2_INT_DATA_SEL_REG                 0x1A
#define BMA2X2_INT2_PAD_SEL_REG                 0x1B
#define BMA2X2_INT_SRC_REG                      0x1E
#define BMA2X2_INT_SET_REG                      0x20
#define BMA2X2_INT_CTRL_REG                     0x21
#define BMA2X2_LOW_DURN_REG                     0x22
#define BMA2X2_LOW_THRES_REG                    0x23
#define BMA2X2_LOW_HIGH_HYST_REG                0x24
#define BMA2X2_HIGH_DURN_REG                    0x25
#define BMA2X2_HIGH_THRES_REG                   0x26
#define BMA2X2_SLOPE_DURN_REG                   0x27
#define BMA2X2_SLOPE_THRES_REG                  0x28
#define BMA2X2_SLO_NO_MOT_THRES_REG             0x29
#define BMA2X2_TAP_PARAM_REG                    0x2A
#define BMA2X2_TAP_THRES_REG                    0x2B
#define BMA2X2_ORIENT_PARAM_REG                 0x2C
#define BMA2X2_THETA_BLOCK_REG                  0x2D
#define BMA2X2_THETA_FLAT_REG                   0x2E
#define BMA2X2_FLAT_HOLD_TIME_REG               0x2F
#define BMA2X2_FIFO_WML_TRIG                    0x30
#define BMA2X2_SELF_TEST_REG                    0x32
#define BMA2X2_EEPROM_CTRL_REG                  0x33
#define BMA2X2_SERIAL_CTRL_REG                  0x34
#define BMA2X2_EXTMODE_CTRL_REG                 0x35
#define BMA2X2_OFFSET_CTRL_REG                  0x36
#define BMA2X2_OFFSET_PARAMS_REG                0x37
#define BMA2X2_OFFSET_X_AXIS_REG                0x38
#define BMA2X2_OFFSET_Y_AXIS_REG                0x39
#define BMA2X2_OFFSET_Z_AXIS_REG                0x3A
#define BMA2X2_GP0_REG                          0x3B
#define BMA2X2_GP1_REG                          0x3C
#define BMA2X2_FIFO_MODE_REG                    0x3E
#define BMA2X2_FIFO_DATA_OUTPUT_REG             0x3F

#define BMA2X2_CHIP_ID__POS             0
#define BMA2X2_CHIP_ID__MSK             0xFF
#define BMA2X2_CHIP_ID__LEN             8
#define BMA2X2_CHIP_ID__REG             BMA2X2_CHIP_ID_REG

#define BMA2X2_VERSION__POS          0
#define BMA2X2_VERSION__LEN          8
#define BMA2X2_VERSION__MSK          0xFF
#define BMA2X2_VERSION__REG          BMA2X2_VERSION_REG

#define BMA2x2_SLO_NO_MOT_DUR__POS   2
#define BMA2x2_SLO_NO_MOT_DUR__LEN   6
#define BMA2x2_SLO_NO_MOT_DUR__MSK   0xFC
#define BMA2x2_SLO_NO_MOT_DUR__REG   BMA2X2_SLOPE_DURN_REG

#define BMA2X2_NEW_DATA_X__POS          0
#define BMA2X2_NEW_DATA_X__LEN          1
#define BMA2X2_NEW_DATA_X__MSK          0x01
#define BMA2X2_NEW_DATA_X__REG          BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X14_LSB__POS           2
#define BMA2X2_ACC_X14_LSB__LEN           6
#define BMA2X2_ACC_X14_LSB__MSK           0xFC
#define BMA2X2_ACC_X14_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X12_LSB__POS           4
#define BMA2X2_ACC_X12_LSB__LEN           4
#define BMA2X2_ACC_X12_LSB__MSK           0xF0
#define BMA2X2_ACC_X12_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X10_LSB__POS           6
#define BMA2X2_ACC_X10_LSB__LEN           2
#define BMA2X2_ACC_X10_LSB__MSK           0xC0
#define BMA2X2_ACC_X10_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X8_LSB__POS           0
#define BMA2X2_ACC_X8_LSB__LEN           0
#define BMA2X2_ACC_X8_LSB__MSK           0x00
#define BMA2X2_ACC_X8_LSB__REG           BMA2X2_X_AXIS_LSB_REG

#define BMA2X2_ACC_X_MSB__POS           0
#define BMA2X2_ACC_X_MSB__LEN           8
#define BMA2X2_ACC_X_MSB__MSK           0xFF
#define BMA2X2_ACC_X_MSB__REG           BMA2X2_X_AXIS_MSB_REG

#define BMA2X2_NEW_DATA_Y__POS          0
#define BMA2X2_NEW_DATA_Y__LEN          1
#define BMA2X2_NEW_DATA_Y__MSK          0x01
#define BMA2X2_NEW_DATA_Y__REG          BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y14_LSB__POS           2
#define BMA2X2_ACC_Y14_LSB__LEN           6
#define BMA2X2_ACC_Y14_LSB__MSK           0xFC
#define BMA2X2_ACC_Y14_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y12_LSB__POS           4
#define BMA2X2_ACC_Y12_LSB__LEN           4
#define BMA2X2_ACC_Y12_LSB__MSK           0xF0
#define BMA2X2_ACC_Y12_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y10_LSB__POS           6
#define BMA2X2_ACC_Y10_LSB__LEN           2
#define BMA2X2_ACC_Y10_LSB__MSK           0xC0
#define BMA2X2_ACC_Y10_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y8_LSB__POS           0
#define BMA2X2_ACC_Y8_LSB__LEN           0
#define BMA2X2_ACC_Y8_LSB__MSK           0x00
#define BMA2X2_ACC_Y8_LSB__REG           BMA2X2_Y_AXIS_LSB_REG

#define BMA2X2_ACC_Y_MSB__POS           0
#define BMA2X2_ACC_Y_MSB__LEN           8
#define BMA2X2_ACC_Y_MSB__MSK           0xFF
#define BMA2X2_ACC_Y_MSB__REG           BMA2X2_Y_AXIS_MSB_REG

#define BMA2X2_NEW_DATA_Z__POS          0
#define BMA2X2_NEW_DATA_Z__LEN          1
#define BMA2X2_NEW_DATA_Z__MSK          0x01
#define BMA2X2_NEW_DATA_Z__REG          BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z14_LSB__POS           2
#define BMA2X2_ACC_Z14_LSB__LEN           6
#define BMA2X2_ACC_Z14_LSB__MSK           0xFC
#define BMA2X2_ACC_Z14_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z12_LSB__POS           4
#define BMA2X2_ACC_Z12_LSB__LEN           4
#define BMA2X2_ACC_Z12_LSB__MSK           0xF0
#define BMA2X2_ACC_Z12_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z10_LSB__POS           6
#define BMA2X2_ACC_Z10_LSB__LEN           2
#define BMA2X2_ACC_Z10_LSB__MSK           0xC0
#define BMA2X2_ACC_Z10_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z8_LSB__POS           0
#define BMA2X2_ACC_Z8_LSB__LEN           0
#define BMA2X2_ACC_Z8_LSB__MSK           0x00
#define BMA2X2_ACC_Z8_LSB__REG           BMA2X2_Z_AXIS_LSB_REG

#define BMA2X2_ACC_Z_MSB__POS           0
#define BMA2X2_ACC_Z_MSB__LEN           8
#define BMA2X2_ACC_Z_MSB__MSK           0xFF
#define BMA2X2_ACC_Z_MSB__REG           BMA2X2_Z_AXIS_MSB_REG

#define BMA2X2_TEMPERATURE__POS         0
#define BMA2X2_TEMPERATURE__LEN         8
#define BMA2X2_TEMPERATURE__MSK         0xFF
#define BMA2X2_TEMPERATURE__REG         BMA2X2_TEMP_RD_REG

#define BMA2X2_LOWG_INT_S__POS          0
#define BMA2X2_LOWG_INT_S__LEN          1
#define BMA2X2_LOWG_INT_S__MSK          0x01
#define BMA2X2_LOWG_INT_S__REG          BMA2X2_STATUS1_REG

#define BMA2X2_HIGHG_INT_S__POS          1
#define BMA2X2_HIGHG_INT_S__LEN          1
#define BMA2X2_HIGHG_INT_S__MSK          0x02
#define BMA2X2_HIGHG_INT_S__REG          BMA2X2_STATUS1_REG

#define BMA2X2_SLOPE_INT_S__POS          2
#define BMA2X2_SLOPE_INT_S__LEN          1
#define BMA2X2_SLOPE_INT_S__MSK          0x04
#define BMA2X2_SLOPE_INT_S__REG          BMA2X2_STATUS1_REG


#define BMA2X2_SLO_NO_MOT_INT_S__POS          3
#define BMA2X2_SLO_NO_MOT_INT_S__LEN          1
#define BMA2X2_SLO_NO_MOT_INT_S__MSK          0x08
#define BMA2X2_SLO_NO_MOT_INT_S__REG          BMA2X2_STATUS1_REG

#define BMA2X2_DOUBLE_TAP_INT_S__POS     4
#define BMA2X2_DOUBLE_TAP_INT_S__LEN     1
#define BMA2X2_DOUBLE_TAP_INT_S__MSK     0x10
#define BMA2X2_DOUBLE_TAP_INT_S__REG     BMA2X2_STATUS1_REG

#define BMA2X2_SINGLE_TAP_INT_S__POS     5
#define BMA2X2_SINGLE_TAP_INT_S__LEN     1
#define BMA2X2_SINGLE_TAP_INT_S__MSK     0x20
#define BMA2X2_SINGLE_TAP_INT_S__REG     BMA2X2_STATUS1_REG

#define BMA2X2_ORIENT_INT_S__POS         6
#define BMA2X2_ORIENT_INT_S__LEN         1
#define BMA2X2_ORIENT_INT_S__MSK         0x40
#define BMA2X2_ORIENT_INT_S__REG         BMA2X2_STATUS1_REG

#define BMA2X2_FLAT_INT_S__POS           7
#define BMA2X2_FLAT_INT_S__LEN           1
#define BMA2X2_FLAT_INT_S__MSK           0x80
#define BMA2X2_FLAT_INT_S__REG           BMA2X2_STATUS1_REG

#define BMA2X2_FIFO_FULL_INT_S__POS           5
#define BMA2X2_FIFO_FULL_INT_S__LEN           1
#define BMA2X2_FIFO_FULL_INT_S__MSK           0x20
#define BMA2X2_FIFO_FULL_INT_S__REG           BMA2X2_STATUS2_REG

#define BMA2X2_FIFO_WM_INT_S__POS           6
#define BMA2X2_FIFO_WM_INT_S__LEN           1
#define BMA2X2_FIFO_WM_INT_S__MSK           0x40
#define BMA2X2_FIFO_WM_INT_S__REG           BMA2X2_STATUS2_REG

#define BMA2X2_DATA_INT_S__POS           7
#define BMA2X2_DATA_INT_S__LEN           1
#define BMA2X2_DATA_INT_S__MSK           0x80
#define BMA2X2_DATA_INT_S__REG           BMA2X2_STATUS2_REG

#define BMA2X2_SLOPE_FIRST_X__POS        0
#define BMA2X2_SLOPE_FIRST_X__LEN        1
#define BMA2X2_SLOPE_FIRST_X__MSK        0x01
#define BMA2X2_SLOPE_FIRST_X__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_SLOPE_FIRST_Y__POS        1
#define BMA2X2_SLOPE_FIRST_Y__LEN        1
#define BMA2X2_SLOPE_FIRST_Y__MSK        0x02
#define BMA2X2_SLOPE_FIRST_Y__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_SLOPE_FIRST_Z__POS        2
#define BMA2X2_SLOPE_FIRST_Z__LEN        1
#define BMA2X2_SLOPE_FIRST_Z__MSK        0x04
#define BMA2X2_SLOPE_FIRST_Z__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_SLOPE_SIGN_S__POS         3
#define BMA2X2_SLOPE_SIGN_S__LEN         1
#define BMA2X2_SLOPE_SIGN_S__MSK         0x08
#define BMA2X2_SLOPE_SIGN_S__REG         BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_FIRST_X__POS        4
#define BMA2X2_TAP_FIRST_X__LEN        1
#define BMA2X2_TAP_FIRST_X__MSK        0x10
#define BMA2X2_TAP_FIRST_X__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_FIRST_Y__POS        5
#define BMA2X2_TAP_FIRST_Y__LEN        1
#define BMA2X2_TAP_FIRST_Y__MSK        0x20
#define BMA2X2_TAP_FIRST_Y__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_FIRST_Z__POS        6
#define BMA2X2_TAP_FIRST_Z__LEN        1
#define BMA2X2_TAP_FIRST_Z__MSK        0x40
#define BMA2X2_TAP_FIRST_Z__REG        BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_TAP_SIGN_S__POS         7
#define BMA2X2_TAP_SIGN_S__LEN         1
#define BMA2X2_TAP_SIGN_S__MSK         0x80
#define BMA2X2_TAP_SIGN_S__REG         BMA2X2_STATUS_TAP_SLOPE_REG

#define BMA2X2_HIGHG_FIRST_X__POS        0
#define BMA2X2_HIGHG_FIRST_X__LEN        1
#define BMA2X2_HIGHG_FIRST_X__MSK        0x01
#define BMA2X2_HIGHG_FIRST_X__REG        BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_HIGHG_FIRST_Y__POS        1
#define BMA2X2_HIGHG_FIRST_Y__LEN        1
#define BMA2X2_HIGHG_FIRST_Y__MSK        0x02
#define BMA2X2_HIGHG_FIRST_Y__REG        BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_HIGHG_FIRST_Z__POS        2
#define BMA2X2_HIGHG_FIRST_Z__LEN        1
#define BMA2X2_HIGHG_FIRST_Z__MSK        0x04
#define BMA2X2_HIGHG_FIRST_Z__REG        BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_HIGHG_SIGN_S__POS         3
#define BMA2X2_HIGHG_SIGN_S__LEN         1
#define BMA2X2_HIGHG_SIGN_S__MSK         0x08
#define BMA2X2_HIGHG_SIGN_S__REG         BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_ORIENT_S__POS             4
#define BMA2X2_ORIENT_S__LEN             3
#define BMA2X2_ORIENT_S__MSK             0x70
#define BMA2X2_ORIENT_S__REG             BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_FLAT_S__POS               7
#define BMA2X2_FLAT_S__LEN               1
#define BMA2X2_FLAT_S__MSK               0x80
#define BMA2X2_FLAT_S__REG               BMA2X2_STATUS_ORIENT_HIGH_REG

#define BMA2X2_FIFO_FRAME_COUNTER_S__POS             0
#define BMA2X2_FIFO_FRAME_COUNTER_S__LEN             7
#define BMA2X2_FIFO_FRAME_COUNTER_S__MSK             0x7F
#define BMA2X2_FIFO_FRAME_COUNTER_S__REG             BMA2X2_STATUS_FIFO_REG

#define BMA2X2_FIFO_OVERRUN_S__POS             7
#define BMA2X2_FIFO_OVERRUN_S__LEN             1
#define BMA2X2_FIFO_OVERRUN_S__MSK             0x80
#define BMA2X2_FIFO_OVERRUN_S__REG             BMA2X2_STATUS_FIFO_REG

#define BMA2X2_RANGE_SEL__POS             0
#define BMA2X2_RANGE_SEL__LEN             4
#define BMA2X2_RANGE_SEL__MSK             0x0F
#define BMA2X2_RANGE_SEL__REG             BMA2X2_RANGE_SEL_REG

#define BMA2X2_BANDWIDTH__POS             0
#define BMA2X2_BANDWIDTH__LEN             5
#define BMA2X2_BANDWIDTH__MSK             0x1F
#define BMA2X2_BANDWIDTH__REG             BMA2X2_BW_SEL_REG

#define BMA2X2_SLEEP_DUR__POS             1
#define BMA2X2_SLEEP_DUR__LEN             4
#define BMA2X2_SLEEP_DUR__MSK             0x1E
#define BMA2X2_SLEEP_DUR__REG             BMA2X2_MODE_CTRL_REG

#define BMA2X2_MODE_CTRL__POS             5
#define BMA2X2_MODE_CTRL__LEN             3
#define BMA2X2_MODE_CTRL__MSK             0xE0
#define BMA2X2_MODE_CTRL__REG             BMA2X2_MODE_CTRL_REG

#define BMA2X2_DEEP_SUSPEND__POS          5
#define BMA2X2_DEEP_SUSPEND__LEN          1
#define BMA2X2_DEEP_SUSPEND__MSK          0x20
#define BMA2X2_DEEP_SUSPEND__REG          BMA2X2_MODE_CTRL_REG

#define BMA2X2_EN_LOW_POWER__POS          6
#define BMA2X2_EN_LOW_POWER__LEN          1
#define BMA2X2_EN_LOW_POWER__MSK          0x40
#define BMA2X2_EN_LOW_POWER__REG          BMA2X2_MODE_CTRL_REG

#define BMA2X2_EN_SUSPEND__POS            7
#define BMA2X2_EN_SUSPEND__LEN            1
#define BMA2X2_EN_SUSPEND__MSK            0x80
#define BMA2X2_EN_SUSPEND__REG            BMA2X2_MODE_CTRL_REG

#define BMA2X2_SLEEP_TIMER__POS          5
#define BMA2X2_SLEEP_TIMER__LEN          1
#define BMA2X2_SLEEP_TIMER__MSK          0x20
#define BMA2X2_SLEEP_TIMER__REG          BMA2X2_LOW_NOISE_CTRL_REG

#define BMA2X2_LOW_POWER_MODE__POS          6
#define BMA2X2_LOW_POWER_MODE__LEN          1
#define BMA2X2_LOW_POWER_MODE__MSK          0x40
#define BMA2X2_LOW_POWER_MODE__REG          BMA2X2_LOW_NOISE_CTRL_REG

#define BMA2X2_EN_LOW_NOISE__POS          7
#define BMA2X2_EN_LOW_NOISE__LEN          1
#define BMA2X2_EN_LOW_NOISE__MSK          0x80
#define BMA2X2_EN_LOW_NOISE__REG          BMA2X2_LOW_NOISE_CTRL_REG

#define BMA2X2_DIS_SHADOW_PROC__POS       6
#define BMA2X2_DIS_SHADOW_PROC__LEN       1
#define BMA2X2_DIS_SHADOW_PROC__MSK       0x40
#define BMA2X2_DIS_SHADOW_PROC__REG       BMA2X2_DATA_CTRL_REG

#define BMA2X2_EN_DATA_HIGH_BW__POS         7
#define BMA2X2_EN_DATA_HIGH_BW__LEN         1
#define BMA2X2_EN_DATA_HIGH_BW__MSK         0x80
#define BMA2X2_EN_DATA_HIGH_BW__REG         BMA2X2_DATA_CTRL_REG

#define BMA2X2_EN_SOFT_RESET__POS         0
#define BMA2X2_EN_SOFT_RESET__LEN         8
#define BMA2X2_EN_SOFT_RESET__MSK         0xFF
#define BMA2X2_EN_SOFT_RESET__REG         BMA2X2_RESET_REG

#define BMA2X2_EN_SOFT_RESET_VALUE        0xB6

#define BMA2X2_EN_SLOPE_X_INT__POS         0
#define BMA2X2_EN_SLOPE_X_INT__LEN         1
#define BMA2X2_EN_SLOPE_X_INT__MSK         0x01
#define BMA2X2_EN_SLOPE_X_INT__REG         BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_SLOPE_Y_INT__POS         1
#define BMA2X2_EN_SLOPE_Y_INT__LEN         1
#define BMA2X2_EN_SLOPE_Y_INT__MSK         0x02
#define BMA2X2_EN_SLOPE_Y_INT__REG         BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_SLOPE_Z_INT__POS         2
#define BMA2X2_EN_SLOPE_Z_INT__LEN         1
#define BMA2X2_EN_SLOPE_Z_INT__MSK         0x04
#define BMA2X2_EN_SLOPE_Z_INT__REG         BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_DOUBLE_TAP_INT__POS      4
#define BMA2X2_EN_DOUBLE_TAP_INT__LEN      1
#define BMA2X2_EN_DOUBLE_TAP_INT__MSK      0x10
#define BMA2X2_EN_DOUBLE_TAP_INT__REG      BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_SINGLE_TAP_INT__POS      5
#define BMA2X2_EN_SINGLE_TAP_INT__LEN      1
#define BMA2X2_EN_SINGLE_TAP_INT__MSK      0x20
#define BMA2X2_EN_SINGLE_TAP_INT__REG      BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_ORIENT_INT__POS          6
#define BMA2X2_EN_ORIENT_INT__LEN          1
#define BMA2X2_EN_ORIENT_INT__MSK          0x40
#define BMA2X2_EN_ORIENT_INT__REG          BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_FLAT_INT__POS            7
#define BMA2X2_EN_FLAT_INT__LEN            1
#define BMA2X2_EN_FLAT_INT__MSK            0x80
#define BMA2X2_EN_FLAT_INT__REG            BMA2X2_INT_ENABLE1_REG

#define BMA2X2_EN_HIGHG_X_INT__POS         0
#define BMA2X2_EN_HIGHG_X_INT__LEN         1
#define BMA2X2_EN_HIGHG_X_INT__MSK         0x01
#define BMA2X2_EN_HIGHG_X_INT__REG         BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_HIGHG_Y_INT__POS         1
#define BMA2X2_EN_HIGHG_Y_INT__LEN         1
#define BMA2X2_EN_HIGHG_Y_INT__MSK         0x02
#define BMA2X2_EN_HIGHG_Y_INT__REG         BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_HIGHG_Z_INT__POS         2
#define BMA2X2_EN_HIGHG_Z_INT__LEN         1
#define BMA2X2_EN_HIGHG_Z_INT__MSK         0x04
#define BMA2X2_EN_HIGHG_Z_INT__REG         BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_LOWG_INT__POS            3
#define BMA2X2_EN_LOWG_INT__LEN            1
#define BMA2X2_EN_LOWG_INT__MSK            0x08
#define BMA2X2_EN_LOWG_INT__REG            BMA2X2_INT_ENABLE2_REG

#define BMA2X2_EN_NEW_DATA_INT__POS        4
#define BMA2X2_EN_NEW_DATA_INT__LEN        1
#define BMA2X2_EN_NEW_DATA_INT__MSK        0x10
#define BMA2X2_EN_NEW_DATA_INT__REG        BMA2X2_INT_ENABLE2_REG

#define BMA2X2_INT_FFULL_EN_INT__POS        5
#define BMA2X2_INT_FFULL_EN_INT__LEN        1
#define BMA2X2_INT_FFULL_EN_INT__MSK        0x20
#define BMA2X2_INT_FFULL_EN_INT__REG        BMA2X2_INT_ENABLE2_REG

#define BMA2X2_INT_FWM_EN_INT__POS        6
#define BMA2X2_INT_FWM_EN_INT__LEN        1
#define BMA2X2_INT_FWM_EN_INT__MSK        0x40
#define BMA2X2_INT_FWM_EN_INT__REG        BMA2X2_INT_ENABLE2_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__POS        0
#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__MSK        0x01
#define BMA2X2_INT_SLO_NO_MOT_EN_X_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__POS        1
#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__MSK        0x02
#define BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__POS        2
#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__MSK        0x04
#define BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__POS        3
#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__LEN        1
#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__MSK        0x08
#define BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__REG        BMA2X2_INT_SLO_NO_MOT_REG

#define BMA2X2_EN_INT1_PAD_LOWG__POS        0
#define BMA2X2_EN_INT1_PAD_LOWG__LEN        1
#define BMA2X2_EN_INT1_PAD_LOWG__MSK        0x01
#define BMA2X2_EN_INT1_PAD_LOWG__REG        BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_HIGHG__POS       1
#define BMA2X2_EN_INT1_PAD_HIGHG__LEN       1
#define BMA2X2_EN_INT1_PAD_HIGHG__MSK       0x02
#define BMA2X2_EN_INT1_PAD_HIGHG__REG       BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_SLOPE__POS       2
#define BMA2X2_EN_INT1_PAD_SLOPE__LEN       1
#define BMA2X2_EN_INT1_PAD_SLOPE__MSK       0x04
#define BMA2X2_EN_INT1_PAD_SLOPE__REG       BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__POS        3
#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__LEN        1
#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__MSK        0x08
#define BMA2X2_EN_INT1_PAD_SLO_NO_MOT__REG        BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_DB_TAP__POS      4
#define BMA2X2_EN_INT1_PAD_DB_TAP__LEN      1
#define BMA2X2_EN_INT1_PAD_DB_TAP__MSK      0x10
#define BMA2X2_EN_INT1_PAD_DB_TAP__REG      BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_SNG_TAP__POS     5
#define BMA2X2_EN_INT1_PAD_SNG_TAP__LEN     1
#define BMA2X2_EN_INT1_PAD_SNG_TAP__MSK     0x20
#define BMA2X2_EN_INT1_PAD_SNG_TAP__REG     BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_ORIENT__POS      6
#define BMA2X2_EN_INT1_PAD_ORIENT__LEN      1
#define BMA2X2_EN_INT1_PAD_ORIENT__MSK      0x40
#define BMA2X2_EN_INT1_PAD_ORIENT__REG      BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_FLAT__POS        7
#define BMA2X2_EN_INT1_PAD_FLAT__LEN        1
#define BMA2X2_EN_INT1_PAD_FLAT__MSK        0x80
#define BMA2X2_EN_INT1_PAD_FLAT__REG        BMA2X2_INT1_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_LOWG__POS        0
#define BMA2X2_EN_INT2_PAD_LOWG__LEN        1
#define BMA2X2_EN_INT2_PAD_LOWG__MSK        0x01
#define BMA2X2_EN_INT2_PAD_LOWG__REG        BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_HIGHG__POS       1
#define BMA2X2_EN_INT2_PAD_HIGHG__LEN       1
#define BMA2X2_EN_INT2_PAD_HIGHG__MSK       0x02
#define BMA2X2_EN_INT2_PAD_HIGHG__REG       BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_SLOPE__POS       2
#define BMA2X2_EN_INT2_PAD_SLOPE__LEN       1
#define BMA2X2_EN_INT2_PAD_SLOPE__MSK       0x04
#define BMA2X2_EN_INT2_PAD_SLOPE__REG       BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__POS        3
#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__LEN        1
#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__MSK        0x08
#define BMA2X2_EN_INT2_PAD_SLO_NO_MOT__REG        BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_DB_TAP__POS      4
#define BMA2X2_EN_INT2_PAD_DB_TAP__LEN      1
#define BMA2X2_EN_INT2_PAD_DB_TAP__MSK      0x10
#define BMA2X2_EN_INT2_PAD_DB_TAP__REG      BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_SNG_TAP__POS     5
#define BMA2X2_EN_INT2_PAD_SNG_TAP__LEN     1
#define BMA2X2_EN_INT2_PAD_SNG_TAP__MSK     0x20
#define BMA2X2_EN_INT2_PAD_SNG_TAP__REG     BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_ORIENT__POS      6
#define BMA2X2_EN_INT2_PAD_ORIENT__LEN      1
#define BMA2X2_EN_INT2_PAD_ORIENT__MSK      0x40
#define BMA2X2_EN_INT2_PAD_ORIENT__REG      BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT2_PAD_FLAT__POS        7
#define BMA2X2_EN_INT2_PAD_FLAT__LEN        1
#define BMA2X2_EN_INT2_PAD_FLAT__MSK        0x80
#define BMA2X2_EN_INT2_PAD_FLAT__REG        BMA2X2_INT2_PAD_SEL_REG

#define BMA2X2_EN_INT1_PAD_NEWDATA__POS     0
#define BMA2X2_EN_INT1_PAD_NEWDATA__LEN     1
#define BMA2X2_EN_INT1_PAD_NEWDATA__MSK     0x01
#define BMA2X2_EN_INT1_PAD_NEWDATA__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT1_PAD_FWM__POS     1
#define BMA2X2_EN_INT1_PAD_FWM__LEN     1
#define BMA2X2_EN_INT1_PAD_FWM__MSK     0x02
#define BMA2X2_EN_INT1_PAD_FWM__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT1_PAD_FFULL__POS     2
#define BMA2X2_EN_INT1_PAD_FFULL__LEN     1
#define BMA2X2_EN_INT1_PAD_FFULL__MSK     0x04
#define BMA2X2_EN_INT1_PAD_FFULL__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT2_PAD_FFULL__POS     5
#define BMA2X2_EN_INT2_PAD_FFULL__LEN     1
#define BMA2X2_EN_INT2_PAD_FFULL__MSK     0x20
#define BMA2X2_EN_INT2_PAD_FFULL__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT2_PAD_FWM__POS     6
#define BMA2X2_EN_INT2_PAD_FWM__LEN     1
#define BMA2X2_EN_INT2_PAD_FWM__MSK     0x40
#define BMA2X2_EN_INT2_PAD_FWM__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_EN_INT2_PAD_NEWDATA__POS     7
#define BMA2X2_EN_INT2_PAD_NEWDATA__LEN     1
#define BMA2X2_EN_INT2_PAD_NEWDATA__MSK     0x80
#define BMA2X2_EN_INT2_PAD_NEWDATA__REG     BMA2X2_INT_DATA_SEL_REG

#define BMA2X2_UNFILT_INT_SRC_LOWG__POS        0
#define BMA2X2_UNFILT_INT_SRC_LOWG__LEN        1
#define BMA2X2_UNFILT_INT_SRC_LOWG__MSK        0x01
#define BMA2X2_UNFILT_INT_SRC_LOWG__REG        BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_HIGHG__POS       1
#define BMA2X2_UNFILT_INT_SRC_HIGHG__LEN       1
#define BMA2X2_UNFILT_INT_SRC_HIGHG__MSK       0x02
#define BMA2X2_UNFILT_INT_SRC_HIGHG__REG       BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_SLOPE__POS       2
#define BMA2X2_UNFILT_INT_SRC_SLOPE__LEN       1
#define BMA2X2_UNFILT_INT_SRC_SLOPE__MSK       0x04
#define BMA2X2_UNFILT_INT_SRC_SLOPE__REG       BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__POS        3
#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__LEN        1
#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__MSK        0x08
#define BMA2X2_UNFILT_INT_SRC_SLO_NO_MOT__REG        BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_TAP__POS         4
#define BMA2X2_UNFILT_INT_SRC_TAP__LEN         1
#define BMA2X2_UNFILT_INT_SRC_TAP__MSK         0x10
#define BMA2X2_UNFILT_INT_SRC_TAP__REG         BMA2X2_INT_SRC_REG

#define BMA2X2_UNFILT_INT_SRC_DATA__POS        5
#define BMA2X2_UNFILT_INT_SRC_DATA__LEN        1
#define BMA2X2_UNFILT_INT_SRC_DATA__MSK        0x20
#define BMA2X2_UNFILT_INT_SRC_DATA__REG        BMA2X2_INT_SRC_REG

#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__POS       0
#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__LEN       1
#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__MSK       0x01
#define BMA2X2_INT1_PAD_ACTIVE_LEVEL__REG       BMA2X2_INT_SET_REG

#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__POS       2
#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__LEN       1
#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__MSK       0x04
#define BMA2X2_INT2_PAD_ACTIVE_LEVEL__REG       BMA2X2_INT_SET_REG

#define BMA2X2_INT1_PAD_OUTPUT_TYPE__POS        1
#define BMA2X2_INT1_PAD_OUTPUT_TYPE__LEN        1
#define BMA2X2_INT1_PAD_OUTPUT_TYPE__MSK        0x02
#define BMA2X2_INT1_PAD_OUTPUT_TYPE__REG        BMA2X2_INT_SET_REG

#define BMA2X2_INT2_PAD_OUTPUT_TYPE__POS        3
#define BMA2X2_INT2_PAD_OUTPUT_TYPE__LEN        1
#define BMA2X2_INT2_PAD_OUTPUT_TYPE__MSK        0x08
#define BMA2X2_INT2_PAD_OUTPUT_TYPE__REG        BMA2X2_INT_SET_REG

#define BMA2X2_INT_MODE_SEL__POS                0
#define BMA2X2_INT_MODE_SEL__LEN                4
#define BMA2X2_INT_MODE_SEL__MSK                0x0F
#define BMA2X2_INT_MODE_SEL__REG                BMA2X2_INT_CTRL_REG

#define BMA2X2_RESET_INT__POS           7
#define BMA2X2_RESET_INT__LEN           1
#define BMA2X2_RESET_INT__MSK           0x80
#define BMA2X2_RESET_INT__REG           BMA2X2_INT_CTRL_REG

#define BMA2X2_LOWG_DUR__POS                    0
#define BMA2X2_LOWG_DUR__LEN                    8
#define BMA2X2_LOWG_DUR__MSK                    0xFF
#define BMA2X2_LOWG_DUR__REG                    BMA2X2_LOW_DURN_REG

#define BMA2X2_LOWG_THRES__POS                  0
#define BMA2X2_LOWG_THRES__LEN                  8
#define BMA2X2_LOWG_THRES__MSK                  0xFF
#define BMA2X2_LOWG_THRES__REG                  BMA2X2_LOW_THRES_REG

#define BMA2X2_LOWG_HYST__POS                   0
#define BMA2X2_LOWG_HYST__LEN                   2
#define BMA2X2_LOWG_HYST__MSK                   0x03
#define BMA2X2_LOWG_HYST__REG                   BMA2X2_LOW_HIGH_HYST_REG

#define BMA2X2_LOWG_INT_MODE__POS               2
#define BMA2X2_LOWG_INT_MODE__LEN               1
#define BMA2X2_LOWG_INT_MODE__MSK               0x04
#define BMA2X2_LOWG_INT_MODE__REG               BMA2X2_LOW_HIGH_HYST_REG

#define BMA2X2_HIGHG_DUR__POS                    0
#define BMA2X2_HIGHG_DUR__LEN                    8
#define BMA2X2_HIGHG_DUR__MSK                    0xFF
#define BMA2X2_HIGHG_DUR__REG                    BMA2X2_HIGH_DURN_REG

#define BMA2X2_HIGHG_THRES__POS                  0
#define BMA2X2_HIGHG_THRES__LEN                  8
#define BMA2X2_HIGHG_THRES__MSK                  0xFF
#define BMA2X2_HIGHG_THRES__REG                  BMA2X2_HIGH_THRES_REG

#define BMA2X2_HIGHG_HYST__POS                  6
#define BMA2X2_HIGHG_HYST__LEN                  2
#define BMA2X2_HIGHG_HYST__MSK                  0xC0
#define BMA2X2_HIGHG_HYST__REG                  BMA2X2_LOW_HIGH_HYST_REG

#define BMA2X2_SLOPE_DUR__POS                    0
#define BMA2X2_SLOPE_DUR__LEN                    2
#define BMA2X2_SLOPE_DUR__MSK                    0x03
#define BMA2X2_SLOPE_DUR__REG                    BMA2X2_SLOPE_DURN_REG

#define BMA2X2_SLO_NO_MOT_DUR__POS                    2
#define BMA2X2_SLO_NO_MOT_DUR__LEN                    6
#define BMA2X2_SLO_NO_MOT_DUR__MSK                    0xFC
#define BMA2X2_SLO_NO_MOT_DUR__REG                    BMA2X2_SLOPE_DURN_REG

#define BMA2X2_SLOPE_THRES__POS                  0
#define BMA2X2_SLOPE_THRES__LEN                  8
#define BMA2X2_SLOPE_THRES__MSK                  0xFF
#define BMA2X2_SLOPE_THRES__REG                  BMA2X2_SLOPE_THRES_REG

#define BMA2X2_SLO_NO_MOT_THRES__POS                  0
#define BMA2X2_SLO_NO_MOT_THRES__LEN                  8
#define BMA2X2_SLO_NO_MOT_THRES__MSK                  0xFF
#define BMA2X2_SLO_NO_MOT_THRES__REG           BMA2X2_SLO_NO_MOT_THRES_REG

#define BMA2X2_TAP_DUR__POS                    0
#define BMA2X2_TAP_DUR__LEN                    3
#define BMA2X2_TAP_DUR__MSK                    0x07
#define BMA2X2_TAP_DUR__REG                    BMA2X2_TAP_PARAM_REG

#define BMA2X2_TAP_SHOCK_DURN__POS             6
#define BMA2X2_TAP_SHOCK_DURN__LEN             1
#define BMA2X2_TAP_SHOCK_DURN__MSK             0x40
#define BMA2X2_TAP_SHOCK_DURN__REG             BMA2X2_TAP_PARAM_REG

#define BMA2X2_ADV_TAP_INT__POS                5
#define BMA2X2_ADV_TAP_INT__LEN                1
#define BMA2X2_ADV_TAP_INT__MSK                0x20
#define BMA2X2_ADV_TAP_INT__REG                BMA2X2_TAP_PARAM_REG

#define BMA2X2_TAP_QUIET_DURN__POS             7
#define BMA2X2_TAP_QUIET_DURN__LEN             1
#define BMA2X2_TAP_QUIET_DURN__MSK             0x80
#define BMA2X2_TAP_QUIET_DURN__REG             BMA2X2_TAP_PARAM_REG

#define BMA2X2_TAP_THRES__POS                  0
#define BMA2X2_TAP_THRES__LEN                  5
#define BMA2X2_TAP_THRES__MSK                  0x1F
#define BMA2X2_TAP_THRES__REG                  BMA2X2_TAP_THRES_REG

#define BMA2X2_TAP_SAMPLES__POS                6
#define BMA2X2_TAP_SAMPLES__LEN                2
#define BMA2X2_TAP_SAMPLES__MSK                0xC0
#define BMA2X2_TAP_SAMPLES__REG                BMA2X2_TAP_THRES_REG

#define BMA2X2_ORIENT_MODE__POS                  0
#define BMA2X2_ORIENT_MODE__LEN                  2
#define BMA2X2_ORIENT_MODE__MSK                  0x03
#define BMA2X2_ORIENT_MODE__REG                  BMA2X2_ORIENT_PARAM_REG

#define BMA2X2_ORIENT_BLOCK__POS                 2
#define BMA2X2_ORIENT_BLOCK__LEN                 2
#define BMA2X2_ORIENT_BLOCK__MSK                 0x0C
#define BMA2X2_ORIENT_BLOCK__REG                 BMA2X2_ORIENT_PARAM_REG

#define BMA2X2_ORIENT_HYST__POS                  4
#define BMA2X2_ORIENT_HYST__LEN                  3
#define BMA2X2_ORIENT_HYST__MSK                  0x70
#define BMA2X2_ORIENT_HYST__REG                  BMA2X2_ORIENT_PARAM_REG

#define BMA2X2_ORIENT_AXIS__POS                  7
#define BMA2X2_ORIENT_AXIS__LEN                  1
#define BMA2X2_ORIENT_AXIS__MSK                  0x80
#define BMA2X2_ORIENT_AXIS__REG                  BMA2X2_THETA_BLOCK_REG

#define BMA2X2_ORIENT_UD_EN__POS                  6
#define BMA2X2_ORIENT_UD_EN__LEN                  1
#define BMA2X2_ORIENT_UD_EN__MSK                  0x40
#define BMA2X2_ORIENT_UD_EN__REG                  BMA2X2_THETA_BLOCK_REG

#define BMA2X2_THETA_BLOCK__POS                  0
#define BMA2X2_THETA_BLOCK__LEN                  6
#define BMA2X2_THETA_BLOCK__MSK                  0x3F
#define BMA2X2_THETA_BLOCK__REG                  BMA2X2_THETA_BLOCK_REG

#define BMA2X2_THETA_FLAT__POS                  0
#define BMA2X2_THETA_FLAT__LEN                  6
#define BMA2X2_THETA_FLAT__MSK                  0x3F
#define BMA2X2_THETA_FLAT__REG                  BMA2X2_THETA_FLAT_REG

#define BMA2X2_FLAT_HOLD_TIME__POS              4
#define BMA2X2_FLAT_HOLD_TIME__LEN              2
#define BMA2X2_FLAT_HOLD_TIME__MSK              0x30
#define BMA2X2_FLAT_HOLD_TIME__REG              BMA2X2_FLAT_HOLD_TIME_REG

#define BMA2X2_FLAT_HYS__POS                   0
#define BMA2X2_FLAT_HYS__LEN                   3
#define BMA2X2_FLAT_HYS__MSK                   0x07
#define BMA2X2_FLAT_HYS__REG                   BMA2X2_FLAT_HOLD_TIME_REG

#define BMA2X2_FIFO_WML_TRIG_RETAIN__POS                   0
#define BMA2X2_FIFO_WML_TRIG_RETAIN__LEN                   6
#define BMA2X2_FIFO_WML_TRIG_RETAIN__MSK                   0x3F
#define BMA2X2_FIFO_WML_TRIG_RETAIN__REG                   BMA2X2_FIFO_WML_TRIG

#define BMA2X2_EN_SELF_TEST__POS                0
#define BMA2X2_EN_SELF_TEST__LEN                2
#define BMA2X2_EN_SELF_TEST__MSK                0x03
#define BMA2X2_EN_SELF_TEST__REG                BMA2X2_SELF_TEST_REG

#define BMA2X2_NEG_SELF_TEST__POS               2
#define BMA2X2_NEG_SELF_TEST__LEN               1
#define BMA2X2_NEG_SELF_TEST__MSK               0x04
#define BMA2X2_NEG_SELF_TEST__REG               BMA2X2_SELF_TEST_REG

#define BMA2X2_SELF_TEST_AMP__POS               4
#define BMA2X2_SELF_TEST_AMP__LEN               1
#define BMA2X2_SELF_TEST_AMP__MSK               0x10
#define BMA2X2_SELF_TEST_AMP__REG               BMA2X2_SELF_TEST_REG


#define BMA2X2_UNLOCK_EE_PROG_MODE__POS     0
#define BMA2X2_UNLOCK_EE_PROG_MODE__LEN     1
#define BMA2X2_UNLOCK_EE_PROG_MODE__MSK     0x01
#define BMA2X2_UNLOCK_EE_PROG_MODE__REG     BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_START_EE_PROG_TRIG__POS      1
#define BMA2X2_START_EE_PROG_TRIG__LEN      1
#define BMA2X2_START_EE_PROG_TRIG__MSK      0x02
#define BMA2X2_START_EE_PROG_TRIG__REG      BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_EE_PROG_READY__POS          2
#define BMA2X2_EE_PROG_READY__LEN          1
#define BMA2X2_EE_PROG_READY__MSK          0x04
#define BMA2X2_EE_PROG_READY__REG          BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_UPDATE_IMAGE__POS                3
#define BMA2X2_UPDATE_IMAGE__LEN                1
#define BMA2X2_UPDATE_IMAGE__MSK                0x08
#define BMA2X2_UPDATE_IMAGE__REG                BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_EE_REMAIN__POS                4
#define BMA2X2_EE_REMAIN__LEN                4
#define BMA2X2_EE_REMAIN__MSK                0xF0
#define BMA2X2_EE_REMAIN__REG                BMA2X2_EEPROM_CTRL_REG

#define BMA2X2_EN_SPI_MODE_3__POS              0
#define BMA2X2_EN_SPI_MODE_3__LEN              1
#define BMA2X2_EN_SPI_MODE_3__MSK              0x01
#define BMA2X2_EN_SPI_MODE_3__REG              BMA2X2_SERIAL_CTRL_REG

#define BMA2X2_I2C_WATCHDOG_PERIOD__POS        1
#define BMA2X2_I2C_WATCHDOG_PERIOD__LEN        1
#define BMA2X2_I2C_WATCHDOG_PERIOD__MSK        0x02
#define BMA2X2_I2C_WATCHDOG_PERIOD__REG        BMA2X2_SERIAL_CTRL_REG

#define BMA2X2_EN_I2C_WATCHDOG__POS            2
#define BMA2X2_EN_I2C_WATCHDOG__LEN            1
#define BMA2X2_EN_I2C_WATCHDOG__MSK            0x04
#define BMA2X2_EN_I2C_WATCHDOG__REG            BMA2X2_SERIAL_CTRL_REG

#define BMA2X2_EXT_MODE__POS              7
#define BMA2X2_EXT_MODE__LEN              1
#define BMA2X2_EXT_MODE__MSK              0x80
#define BMA2X2_EXT_MODE__REG              BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_ALLOW_UPPER__POS        6
#define BMA2X2_ALLOW_UPPER__LEN        1
#define BMA2X2_ALLOW_UPPER__MSK        0x40
#define BMA2X2_ALLOW_UPPER__REG        BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_MAP_2_LOWER__POS            5
#define BMA2X2_MAP_2_LOWER__LEN            1
#define BMA2X2_MAP_2_LOWER__MSK            0x20
#define BMA2X2_MAP_2_LOWER__REG            BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_MAGIC_NUMBER__POS            0
#define BMA2X2_MAGIC_NUMBER__LEN            5
#define BMA2X2_MAGIC_NUMBER__MSK            0x1F
#define BMA2X2_MAGIC_NUMBER__REG            BMA2X2_EXTMODE_CTRL_REG

#define BMA2X2_UNLOCK_EE_WRITE_TRIM__POS        4
#define BMA2X2_UNLOCK_EE_WRITE_TRIM__LEN        4
#define BMA2X2_UNLOCK_EE_WRITE_TRIM__MSK        0xF0
#define BMA2X2_UNLOCK_EE_WRITE_TRIM__REG        BMA2X2_CTRL_UNLOCK_REG

#define BMA2X2_EN_SLOW_COMP_X__POS              0
#define BMA2X2_EN_SLOW_COMP_X__LEN              1
#define BMA2X2_EN_SLOW_COMP_X__MSK              0x01
#define BMA2X2_EN_SLOW_COMP_X__REG              BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_EN_SLOW_COMP_Y__POS              1
#define BMA2X2_EN_SLOW_COMP_Y__LEN              1
#define BMA2X2_EN_SLOW_COMP_Y__MSK              0x02
#define BMA2X2_EN_SLOW_COMP_Y__REG              BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_EN_SLOW_COMP_Z__POS              2
#define BMA2X2_EN_SLOW_COMP_Z__LEN              1
#define BMA2X2_EN_SLOW_COMP_Z__MSK              0x04
#define BMA2X2_EN_SLOW_COMP_Z__REG              BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_FAST_CAL_RDY_S__POS             4
#define BMA2X2_FAST_CAL_RDY_S__LEN             1
#define BMA2X2_FAST_CAL_RDY_S__MSK             0x10
#define BMA2X2_FAST_CAL_RDY_S__REG             BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_CAL_TRIGGER__POS                5
#define BMA2X2_CAL_TRIGGER__LEN                2
#define BMA2X2_CAL_TRIGGER__MSK                0x60
#define BMA2X2_CAL_TRIGGER__REG                BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_RESET_OFFSET_REGS__POS           7
#define BMA2X2_RESET_OFFSET_REGS__LEN           1
#define BMA2X2_RESET_OFFSET_REGS__MSK           0x80
#define BMA2X2_RESET_OFFSET_REGS__REG           BMA2X2_OFFSET_CTRL_REG

#define BMA2X2_COMP_CUTOFF__POS                 0
#define BMA2X2_COMP_CUTOFF__LEN                 1
#define BMA2X2_COMP_CUTOFF__MSK                 0x01
#define BMA2X2_COMP_CUTOFF__REG                 BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_X__POS        1
#define BMA2X2_COMP_TARGET_OFFSET_X__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_X__MSK        0x06
#define BMA2X2_COMP_TARGET_OFFSET_X__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_Y__POS        3
#define BMA2X2_COMP_TARGET_OFFSET_Y__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_Y__MSK        0x18
#define BMA2X2_COMP_TARGET_OFFSET_Y__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_COMP_TARGET_OFFSET_Z__POS        5
#define BMA2X2_COMP_TARGET_OFFSET_Z__LEN        2
#define BMA2X2_COMP_TARGET_OFFSET_Z__MSK        0x60
#define BMA2X2_COMP_TARGET_OFFSET_Z__REG        BMA2X2_OFFSET_PARAMS_REG

#define BMA2X2_FIFO_DATA_SELECT__POS                 0
#define BMA2X2_FIFO_DATA_SELECT__LEN                 2
#define BMA2X2_FIFO_DATA_SELECT__MSK                 0x03
#define BMA2X2_FIFO_DATA_SELECT__REG                 BMA2X2_FIFO_MODE_REG

#define BMA2X2_FIFO_TRIGGER_SOURCE__POS                 2
#define BMA2X2_FIFO_TRIGGER_SOURCE__LEN                 2
#define BMA2X2_FIFO_TRIGGER_SOURCE__MSK                 0x0C
#define BMA2X2_FIFO_TRIGGER_SOURCE__REG                 BMA2X2_FIFO_MODE_REG

#define BMA2X2_FIFO_TRIGGER_ACTION__POS                 4
#define BMA2X2_FIFO_TRIGGER_ACTION__LEN                 2
#define BMA2X2_FIFO_TRIGGER_ACTION__MSK                 0x30
#define BMA2X2_FIFO_TRIGGER_ACTION__REG                 BMA2X2_FIFO_MODE_REG

#define BMA2X2_FIFO_MODE__POS                 6
#define BMA2X2_FIFO_MODE__LEN                 2
#define BMA2X2_FIFO_MODE__MSK                 0xC0
#define BMA2X2_FIFO_MODE__REG                 BMA2X2_FIFO_MODE_REG


#define BMA2X2_STATUS1                             0
#define BMA2X2_STATUS2                             1
#define BMA2X2_STATUS3                             2
#define BMA2X2_STATUS4                             3
#define BMA2X2_STATUS5                             4


#define BMA2X2_RANGE_2G                 3
#define BMA2X2_RANGE_4G                 5
#define BMA2X2_RANGE_8G                 8
#define BMA2X2_RANGE_16G               12

#define BMA2X2_MAX_DELAY		200
#define BMA2X2_RANGE_SET		BMA2X2_RANGE_4G   // +/- 4G
#define BMA2X2_BW_SET			10  // 31_25 Hz  

#define BMA2X2_BW_7_81HZ        0x08
#define BMA2X2_BW_15_63HZ       0x09
#define BMA2X2_BW_31_25HZ       0x0A
#define BMA2X2_BW_62_50HZ       0x0B
#define BMA2X2_BW_125HZ         0x0C
#define BMA2X2_BW_250HZ         0x0D
#define BMA2X2_BW_500HZ         0x0E
#define BMA2X2_BW_1000HZ        0x0F

#define BMA2X2_SLEEP_DUR_0_5MS        0x05
#define BMA2X2_SLEEP_DUR_1MS          0x06
#define BMA2X2_SLEEP_DUR_2MS          0x07
#define BMA2X2_SLEEP_DUR_4MS          0x08
#define BMA2X2_SLEEP_DUR_6MS          0x09
#define BMA2X2_SLEEP_DUR_10MS         0x0A
#define BMA2X2_SLEEP_DUR_25MS         0x0B
#define BMA2X2_SLEEP_DUR_50MS         0x0C
#define BMA2X2_SLEEP_DUR_100MS        0x0D
#define BMA2X2_SLEEP_DUR_500MS        0x0E
#define BMA2X2_SLEEP_DUR_1S           0x0F

#define BMA2X2_LATCH_DUR_NON_LATCH    0x00
#define BMA2X2_LATCH_DUR_250MS        0x01
#define BMA2X2_LATCH_DUR_500MS        0x02
#define BMA2X2_LATCH_DUR_1S           0x03
#define BMA2X2_LATCH_DUR_2S           0x04
#define BMA2X2_LATCH_DUR_4S           0x05
#define BMA2X2_LATCH_DUR_8S           0x06
#define BMA2X2_LATCH_DUR_LATCH        0x07
#define BMA2X2_LATCH_DUR_NON_LATCH1   0x08
#define BMA2X2_LATCH_DUR_250US        0x09
#define BMA2X2_LATCH_DUR_500US        0x0A
#define BMA2X2_LATCH_DUR_1MS          0x0B
#define BMA2X2_LATCH_DUR_12_5MS       0x0C
#define BMA2X2_LATCH_DUR_25MS         0x0D
#define BMA2X2_LATCH_DUR_50MS         0x0E
#define BMA2X2_LATCH_DUR_LATCH1       0x0F

#define BMA2X2_MODE_NORMAL             0
#define BMA2X2_MODE_LOWPOWER1          1
#define BMA2X2_MODE_SUSPEND            2
#define BMA2X2_MODE_DEEP_SUSPEND       3
#define BMA2X2_MODE_LOWPOWER2          4
#define BMA2X2_MODE_STANDBY            5

#define BMA2X2_X_AXIS           0
#define BMA2X2_Y_AXIS           1
#define BMA2X2_Z_AXIS           2

#define BMA2X2_Low_G_Interrupt       0
#define BMA2X2_High_G_X_Interrupt    1
#define BMA2X2_High_G_Y_Interrupt    2
#define BMA2X2_High_G_Z_Interrupt    3
#define BMA2X2_DATA_EN               4
#define BMA2X2_Slope_X_Interrupt     5
#define BMA2X2_Slope_Y_Interrupt     6
#define BMA2X2_Slope_Z_Interrupt     7
#define BMA2X2_Single_Tap_Interrupt  8
#define BMA2X2_Double_Tap_Interrupt  9
#define BMA2X2_Orient_Interrupt      10
#define BMA2X2_Flat_Interrupt        11
#define BMA2X2_FFULL_INTERRUPT       12
#define BMA2X2_FWM_INTERRUPT         13

#define BMA2X2_INT1_LOWG         0
#define BMA2X2_INT2_LOWG         1
#define BMA2X2_INT1_HIGHG        0
#define BMA2X2_INT2_HIGHG        1
#define BMA2X2_INT1_SLOPE        0
#define BMA2X2_INT2_SLOPE        1
#define BMA2X2_INT1_SLO_NO_MOT   0
#define BMA2X2_INT2_SLO_NO_MOT   1
#define BMA2X2_INT1_DTAP         0
#define BMA2X2_INT2_DTAP         1
#define BMA2X2_INT1_STAP         0
#define BMA2X2_INT2_STAP         1
#define BMA2X2_INT1_ORIENT       0
#define BMA2X2_INT2_ORIENT       1
#define BMA2X2_INT1_FLAT         0
#define BMA2X2_INT2_FLAT         1
#define BMA2X2_INT1_NDATA        0
#define BMA2X2_INT2_NDATA        1
#define BMA2X2_INT1_FWM          0
#define BMA2X2_INT2_FWM          1
#define BMA2X2_INT1_FFULL        0
#define BMA2X2_INT2_FFULL        1

#define BMA2X2_SRC_LOWG         0
#define BMA2X2_SRC_HIGHG        1
#define BMA2X2_SRC_SLOPE        2
#define BMA2X2_SRC_SLO_NO_MOT   3
#define BMA2X2_SRC_TAP          4
#define BMA2X2_SRC_DATA         5

#define BMA2X2_INT1_OUTPUT      0
#define BMA2X2_INT2_OUTPUT      1
#define BMA2X2_INT1_LEVEL       0
#define BMA2X2_INT2_LEVEL       1

#define BMA2X2_LOW_DURATION            0
#define BMA2X2_HIGH_DURATION           1
#define BMA2X2_SLOPE_DURATION          2
#define BMA2X2_SLO_NO_MOT_DURATION     3

#define BMA2X2_LOW_THRESHOLD            0
#define BMA2X2_HIGH_THRESHOLD           1
#define BMA2X2_SLOPE_THRESHOLD          2
#define BMA2X2_SLO_NO_MOT_THRESHOLD     3


#define BMA2X2_LOWG_HYST                0
#define BMA2X2_HIGHG_HYST               1

#define BMA2X2_ORIENT_THETA             0
#define BMA2X2_FLAT_THETA               1

#define BMA2X2_I2C_SELECT               0
#define BMA2X2_I2C_EN                   1

#define BMA2X2_SLOW_COMP_X              0
#define BMA2X2_SLOW_COMP_Y              1
#define BMA2X2_SLOW_COMP_Z              2

#define BMA2X2_CUT_OFF                  0
#define BMA2X2_OFFSET_TRIGGER_X         1
#define BMA2X2_OFFSET_TRIGGER_Y         2
#define BMA2X2_OFFSET_TRIGGER_Z         3

#define BMA2X2_GP0                      0
#define BMA2X2_GP1                      1

#define BMA2X2_SLO_NO_MOT_EN_X          0
#define BMA2X2_SLO_NO_MOT_EN_Y          1
#define BMA2X2_SLO_NO_MOT_EN_Z          2
#define BMA2X2_SLO_NO_MOT_EN_SEL        3

#define BMA2X2_WAKE_UP_DUR_20MS         0
#define BMA2X2_WAKE_UP_DUR_80MS         1
#define BMA2X2_WAKE_UP_DUR_320MS                2
#define BMA2X2_WAKE_UP_DUR_2560MS               3

#define BMA2X2_SELF_TEST0_ON            1
#define BMA2X2_SELF_TEST1_ON            2

#define BMA2X2_EE_W_OFF                 0
#define BMA2X2_EE_W_ON                  1

#define BMA2X2_LOW_TH_IN_G(gthres, range)           ((256 * gthres) / range)


#define BMA2X2_HIGH_TH_IN_G(gthres, range)          ((256 * gthres) / range)


#define BMA2X2_LOW_HY_IN_G(ghyst, range)            ((32 * ghyst) / range)


#define BMA2X2_HIGH_HY_IN_G(ghyst, range)           ((32 * ghyst) / range)


#define BMA2X2_SLOPE_TH_IN_G(gthres, range)    ((128 * gthres) / range)


#define BMA2X2_GET_BITSLICE(regvar, bitname)\
	((regvar & bitname##__MSK) >> bitname##__POS)


#define BMA2X2_SET_BITSLICE(regvar, bitname, val)\
	((regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK))

#define CHECK_CHIP_ID_TIME_MAX 5
#define BMA255_CHIP_ID 0XFA
#define BMA250E_CHIP_ID 0XF9
#define BMA222E_CHIP_ID 0XF8
#define BMA280_CHIP_ID 0XFB
#define BMA355_CHIP_ID 0XEA

#define BMA255_TYPE 0
#define BMA250E_TYPE 1
#define BMA222E_TYPE 2
#define BMA280_TYPE 3

#define MAX_FIFO_F_LEVEL 32
#define MAX_FIFO_F_BYTES 6
#define BMA_MAX_RETRY_I2C_XFER (100)

#ifdef CONFIG_DOUBLE_TAP
#define DEFAULT_TAP_JUDGE_PERIOD 350    /* default judge in 350 ms */
#endif

#ifdef CONFIG_FLICK
#define DEFAULT_FLICK_JUDGE_PERIOD 800    	/* default judge in 800 ms */
#define FLICK_DEBOUNCE_TIME 30    		/* debounce time 300 ms */
#define FLICK_DOUBLE_DEBOUNCE_TIME 30    	/* debounce time 300 ms */
#endif

/*! Bosch sensor unknown place*/
#define BOSCH_SENSOR_PLACE_UNKNOWN (-1)
/*! Bosch sensor remapping table size P0~P7*/
#define MAX_AXIS_REMAP_TAB_SZ 8
#define Z380M_BMA2X2_DEFAULT_PLACE 4	// P4
#define Z300M_BMA2X2_DEFAULT_PLACE 1	// P1

/* How was BMA enabled(set to operation mode) */
#define BMA_ENABLED_ALL 0
#define BMA_ENABLED_SGM 1
#define BMA_ENABLED_DTAP 2
#define BMA_ENABLED_INPUT 3
#define BMA_ENABLED_BSX 4
#define BMA_ENABLED_FLICK 5
#define BMA_ENABLED_TERMINAL 6


/*!
 * @brief:BMI058 feature
 *  macro definition
*/

#define BMA2X2_FIFO_DAT_SEL_X		1
#define BMA2X2_FIFO_DAT_SEL_Y		2
#define BMA2X2_FIFO_DAT_SEL_Z		3
// for calibration file added by alp
#define BMA2X2_NVRAM_LENGTH		14

#ifdef CONFIG_SENSORS_BMI058
#define C_BMI058_One_U8X                                 1
#define C_BMI058_Two_U8X                                 2
#define BMI058_OFFSET_TRIGGER_X                BMA2X2_OFFSET_TRIGGER_Y
#define BMI058_OFFSET_TRIGGER_Y                BMA2X2_OFFSET_TRIGGER_X
/*! BMI058 X AXIS OFFSET REG definition*/
#define BMI058_OFFSET_X_AXIS_REG              BMA2X2_OFFSET_Y_AXIS_REG
/*! BMI058 Y AXIS OFFSET REG definition*/
#define BMI058_OFFSET_Y_AXIS_REG              BMA2X2_OFFSET_X_AXIS_REG
#define BMI058_FIFO_DAT_SEL_X                       BMA2X2_FIFO_DAT_SEL_Y
#define BMI058_FIFO_DAT_SEL_Y                       BMA2X2_FIFO_DAT_SEL_X
/*! BMA2x2 common slow no motion X interrupt type definition*/
#define BMA2X2_SLOW_NO_MOT_X_INT          12
/*! BMA2x2 common slow no motion Y interrupt type definition*/
#define BMA2X2_SLOW_NO_MOT_Y_INT          13
/*! BMA2x2 common High G X interrupt type definition*/
#define BMA2X2_HIGHG_X_INT          1
/*! BMA2x2 common High G Y interrupt type definition*/
#define BMA2X2_HIGHG_Y_INT          2
/*! BMA2x2 common slope X interrupt type definition*/
#define BMA2X2_SLOPE_X_INT          5
/*! BMA2x2 common slope Y interrupt type definition*/
#define BMA2X2_SLOPE_Y_INT          6

/*! this structure holds some interrupt types difference
**between BMA2x2 and BMI058.
*/
struct interrupt_map_t {
	int x;
	int y;
};
/*!*Need to use BMA2x2 Common interrupt type definition to
* instead of Some of BMI058 reversed Interrupt type
* because of HW Register.
* The reversed Interrupt types contain:
* slow_no_mot_x_int && slow_not_mot_y_int
* highg_x_int && highg_y_int
* slope_x_int && slope_y_int
**/
static const struct interrupt_map_t int_map[] = {
	{BMA2X2_SLOW_NO_MOT_X_INT, BMA2X2_SLOW_NO_MOT_Y_INT},
	{BMA2X2_HIGHG_X_INT, BMA2X2_HIGHG_Y_INT},
	{BMA2X2_SLOPE_X_INT, BMA2X2_SLOPE_Y_INT}
};

/*! high g or slope interrupt type definition for BMI058*/
/*! High G interrupt of x, y, z axis happened */
#define HIGH_G_INTERRUPT_X            HIGH_G_INTERRUPT_Y_HAPPENED
#define HIGH_G_INTERRUPT_Y            HIGH_G_INTERRUPT_X_HAPPENED
#define HIGH_G_INTERRUPT_Z            HIGH_G_INTERRUPT_Z_HAPPENED
/*! High G interrupt of x, y, z negative axis happened */
#define HIGH_G_INTERRUPT_X_N          HIGH_G_INTERRUPT_Y_NEGATIVE_HAPPENED
#define HIGH_G_INTERRUPT_Y_N          HIGH_G_INTERRUPT_X_NEGATIVE_HAPPENED
#define HIGH_G_INTERRUPT_Z_N          HIGH_G_INTERRUPT_Z_NEGATIVE_HAPPENED
/*! Slope interrupt of x, y, z axis happened */
#define SLOPE_INTERRUPT_X             SLOPE_INTERRUPT_Y_HAPPENED
#define SLOPE_INTERRUPT_Y             SLOPE_INTERRUPT_X_HAPPENED
#define SLOPE_INTERRUPT_Z             SLOPE_INTERRUPT_Z_HAPPENED
/*! Slope interrupt of x, y, z negative axis happened */
#define SLOPE_INTERRUPT_X_N           SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED
#define SLOPE_INTERRUPT_Y_N           SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED
#define SLOPE_INTERRUPT_Z_N           SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED


#else

/*! high g or slope interrupt type definition*/
/*! High G interrupt of x, y, z axis happened */
#define HIGH_G_INTERRUPT_X            HIGH_G_INTERRUPT_X_HAPPENED
#define HIGH_G_INTERRUPT_Y            HIGH_G_INTERRUPT_Y_HAPPENED
#define HIGH_G_INTERRUPT_Z            HIGH_G_INTERRUPT_Z_HAPPENED
/*! High G interrupt of x, y, z negative axis happened */
#define HIGH_G_INTERRUPT_X_N          HIGH_G_INTERRUPT_X_NEGATIVE_HAPPENED
#define HIGH_G_INTERRUPT_Y_N          HIGH_G_INTERRUPT_Y_NEGATIVE_HAPPENED
#define HIGH_G_INTERRUPT_Z_N          HIGH_G_INTERRUPT_Z_NEGATIVE_HAPPENED
/*! Slope interrupt of x, y, z axis happened */
#define SLOPE_INTERRUPT_X             SLOPE_INTERRUPT_X_HAPPENED
#define SLOPE_INTERRUPT_Y             SLOPE_INTERRUPT_Y_HAPPENED
#define SLOPE_INTERRUPT_Z             SLOPE_INTERRUPT_Z_HAPPENED
/*! Slope interrupt of x, y, z negative axis happened */
#define SLOPE_INTERRUPT_X_N           SLOPE_INTERRUPT_X_NEGATIVE_HAPPENED
#define SLOPE_INTERRUPT_Y_N           SLOPE_INTERRUPT_Y_NEGATIVE_HAPPENED
#define SLOPE_INTERRUPT_Z_N           SLOPE_INTERRUPT_Z_NEGATIVE_HAPPENED


#endif/*End of CONFIG_SENSORS_BMI058*/

/* Add by Tom for Calibration */
#define REPORT_SINGLE_TAP_WHEN_DOUBLE_TAP_SENSOR_ENABLED
#define GS_CALI_PATH "/persist/gs_cali.ini"
//#define GS_CALI_PATH "/data/misc/sensor/gs_cali.ini"	// for test
static void bma2x2_read_from_califile(void); 
static int bma2x2_write_to_califile(int raw_x,int raw_y,int raw_z);
static struct bma2x2_data *g_accel_data ;
mm_segment_t old_gs_fs;

static int DEBUG = 0;

static struct kobject *android_gsensor_kobj = NULL;	// Sys kobject variable

#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
static struct callback_data* callback_struct;
void bma2x2_screen_chenged_listaner(const int state);
#endif

/*! A workaroud mask definition with complete resolution exists
* aim at writing operation FIFO_CONFIG_1, 0x3E register */
#define FIFO_WORKAROUNDS_MSK         BMA2X2_FIFO_TRIGGER_SOURCE__MSK

struct bma2x2_type_map_t {

	/*! bma2x2 sensor chip id */
	uint16_t chip_id;

	/*! bma2x2 sensor type */
	uint16_t sensor_type;

	/*! bma2x2 sensor name */
	const char *sensor_name;
};

static const struct bma2x2_type_map_t sensor_type_map[] = {
	{BMA255_CHIP_ID, BMA255_TYPE, "BMA255/254"},
	{BMA355_CHIP_ID, BMA255_TYPE, "BMA355"},
	{BMA250E_CHIP_ID, BMA250E_TYPE, "BMA250E"},
	{BMA222E_CHIP_ID, BMA222E_TYPE, "BMA222E"},
	{BMA280_CHIP_ID, BMA280_TYPE, "BMA280"},
};

//-------------------------------flag----------------------------------------------------
#define APS_TAG					"alp.D :"
#define APS_FUN(f)			printk(KERN_INFO APS_TAG"%s\n", __func__)
#define APS_ERR(fmt, args...)	printk(KERN_ERR APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)	printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)	printk(KERN_INFO APS_TAG fmt, ##args)
//---------------------------------------------------------------------------------------

#define MTK_ACC_GPIO_PIN	51
unsigned int gsensor_int_gpio_number = 0;
static unsigned int gsensor_irq;

/*!
* Bst sensor common definition,
* please give parameters in BSP file.
*/
struct bosch_sensor_specific {
	char *name;
	/* 0 to 7 */
	int place;
	int irq;
	int (*irq_gpio_cfg)(void);
};


/*!
 * we use a typedef to hide the detail,
 * because this type might be changed
 */
struct bosch_sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis */
	/* if an target OS axis is remapped from (-)x,
	 * src is 0, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)y,
	 * src is 1, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)z,
	 * src is 2, sign_* is (-)1 */
	int src_x:3;
	int src_y:3;
	int src_z:3;

	int sign_x:2;
	int sign_y:2;
	int sign_z:2;
};

struct bosch_sensor_data {
	union {
		int16_t v[3];
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
	};
};

struct bma2x2acc {
	s16 x;
	s16 y;
	s16 z;
};

/* Add by Tom for Zen Motion*/
#define NO_FLICK			0
#define SINGLE_FLICK_RIGHT		1
#define DOUBLE_FLICK_RIGHT		2
#define SINGLE_FLICK_LEFT		3
#define DOUBLE_FLICK_LEFT		4
#define SINGLE_FLICK_UP			5
#define DOUBLE_FLICK_UP			6
#define SINGLE_FLICK_DOWN		7
#define DOUBLE_FLICK_DOWN		8
#define SINGLE_FLICK_FORWARD		9
#define DOUBLE_FLICK_FORWARD		10
#define SINGLE_FLICK_BACKWARD		11
#define DOUBLE_FLICK_BACKWARD		12
#define UNKNOWN_FLICK			13
#define SINGLE_FLICK			14
#define DOUBLE_FLICK			15

struct bma2x2_data {
	struct i2c_client *bma2x2_client;
	atomic_t delay;
	atomic_t enable;
	atomic_t en_bst;
	atomic_t selftest_result;
	/* Add by Tom for Zen Motion*/
	atomic_t en_terminal;

	unsigned int chip_id;
	unsigned int fifo_count;
	unsigned char fifo_datasel;
	unsigned char mode;
	signed char sensor_type;
	struct input_dev *input;

	struct bst_dev *bst_acc;

	struct bma2x2acc value;
	struct mutex value_mutex;
	struct mutex enable_mutex;
	struct mutex mode_mutex;
	struct delayed_work work;
	struct work_struct irq_work;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int IRQ;
	int gpio;
	int state;	// for gsensor_status
	struct bosch_sensor_specific *bst_pd;

	int bma_mode_enabled;
	struct input_dev *dev_interrupt;
	/* Add by Tom for Zen Motion*/
	struct input_dev *dev_terminal;

#ifdef CONFIG_SIG_MOTION
	struct class *g_sensor_class;
	struct device *g_sensor_dev;

	/*struct bma250_platform_data *pdata;*/
	atomic_t en_sig_motion;
#endif

#ifdef CONFIG_DOUBLE_TAP
	struct class *g_sensor_class_doubletap;
	struct device *g_sensor_dev_doubletap;
	atomic_t en_double_tap;
	unsigned char tap_times;
	spinlock_t tap_lock;
	struct timer_list	tap_timer;
	int tap_time_period;
#endif
#ifdef CONFIG_FLICK
	atomic_t en_flick;
	struct input_dev *dev_flick;
	unsigned char flick_times;
	spinlock_t flick_lock;
	struct timer_list flick_timer;
	int flick_time_period;
	u64 flick_debounce_time;
	int is_no_motion;
#endif
#ifdef CONFIG_FLICK_HV
	atomic_t en_flick_hv;
	struct input_dev *dev_flick_hv;
#endif
	/* add by Tom for Calibration*/ 
	int is_cali;
	int cali_retry;
	struct bma2x2acc cali;
	/* add by Tom for Load Calibration*/ 
	struct delayed_work cali_work; 
	struct workqueue_struct *cali_wq; 
	/* add by Tom for Skip Same Terminal Event */
	int pre_ter_event;
	int ter_event;
	/* Add by Tom for CTS Workaround */ 
	int en_sensor_type;
//	struct delayed_work CTS_dw; 
//	struct workqueue_struct *CTS_wq; 
};

static struct bma2x2_data *g_accel_data ;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma2x2_early_suspend(struct early_suspend *h);
static void bma2x2_late_resume(struct early_suspend *h);
#endif

static int bma2x2_set_mode(struct i2c_client *client, u8 mode, u8 enabled_mode);
static int bma2x2_get_mode(struct i2c_client *client, u8 *mode);
static int bma2x2_get_fifo_mode(struct i2c_client *client, u8 *fifo_mode);
static int bma2x2_set_fifo_mode(struct i2c_client *client, u8 fifo_mode);
static int bma2x2_normal_to_suspend(struct bma2x2_data *bma2x2,
				unsigned char data1, unsigned char data2);

static int bma2x2_local_init(void);
static int bma2x2_remove(void);
static int bma2x2_init_flag = -1; /* 0<==>OK -1 <==> fail */

static int of_get_bma2x2_platform_data(struct device *dev);

/*Remapping for BMA2X2*/

static const struct bosch_sensor_axis_remap
bst_axis_remap_tab_dft[MAX_AXIS_REMAP_TAB_SZ] = {
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


static void bst_remap_sensor_data(struct bosch_sensor_data *data,
		const struct bosch_sensor_axis_remap *remap)
{
	struct bosch_sensor_data tmp;

	tmp.x = data->v[remap->src_x] * remap->sign_x;
	tmp.y = data->v[remap->src_y] * remap->sign_y;
	tmp.z = data->v[remap->src_z] * remap->sign_z;

	memcpy(data, &tmp, sizeof(*data));
}


static void bst_remap_sensor_data_dft_tab(struct bosch_sensor_data *data,
		int place)
{
	/* sensor with place 0 needs not to be remapped */
	if ((place <= 0) || (place >= MAX_AXIS_REMAP_TAB_SZ))
		return;

	bst_remap_sensor_data(data, &bst_axis_remap_tab_dft[place]);
}

static void bma2x2_remap_sensor_data(struct bma2x2acc *val,
		struct bma2x2_data *client_data)
{
	struct bosch_sensor_data bsd;
	int place;

//	if ((NULL == client_data->bst_pd) || (BOSCH_SENSOR_PLACE_UNKNOWN == client_data->bst_pd->place))
//		place = Z380M_BMA2X2_DEFAULT_PLACE;
//	else
//		place = client_data->bst_pd->place;
#ifdef CONFIG_Z380M
	place = Z380M_BMA2X2_DEFAULT_PLACE;
#endif

#ifdef CONFIG_Z300M
	place = Z300M_BMA2X2_DEFAULT_PLACE;
#endif

#ifdef CONFIG_SENSORS_BMI058
/*x,y need to be invesed becase of HW Register for BMI058*/
	bsd.y = val->x;
	bsd.x = val->y;
	bsd.z = val->z;
#else
	bsd.x = val->x;
	bsd.y = val->y;
	bsd.z = val->z;
#endif

	bst_remap_sensor_data_dft_tab(&bsd, place);

	val->x = bsd.x;
	val->y = bsd.y;
	val->z = bsd.z;

}


static int bma2x2_smbus_read_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;
	dummy = i2c_smbus_read_byte_data(client, reg_addr);
	if (dummy < 0)
		return -1;
	*data = dummy & 0x000000ff;

	return 0;
}

static int bma2x2_smbus_write_byte(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data)
{
	s32 dummy;

	dummy = i2c_smbus_write_byte_data(client, reg_addr, *data);
	if (dummy < 0)
		return -1;
	udelay(2);
	return 0;
}

static int bma2x2_smbus_read_byte_block(struct i2c_client *client,
		unsigned char reg_addr, unsigned char *data, unsigned char len)
{
	s32 dummy;
	dummy = i2c_smbus_read_i2c_block_data(client, reg_addr, len, data);
	if (dummy < 0)
		return -1;
	return 0;
}

static int bma_i2c_burst_read(struct i2c_client *client, u8 reg_addr,
		u8 *data, u16 len)
{
	int retry;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		},

		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};

	for (retry = 0; retry < BMA_MAX_RETRY_I2C_XFER; retry++) {
		if (i2c_transfer(client->adapter, msg, ARRAY_SIZE(msg)) > 0)
			break;
		else
			I2C_RETRY_DELAY();
	}

	if (BMA_MAX_RETRY_I2C_XFER <= retry) {
		APS_LOG( "I2C xfer error");
		return -EIO;
	}

	return 0;
}

static int bma2x2_check_chip_id(struct i2c_client *client,
					struct bma2x2_data *data)
{
	int i = 0;
	int err = 0;
	unsigned char chip_id = 0;
	unsigned char read_count = 0;
	unsigned char bma2x2_sensor_type_count = 0;

	bma2x2_sensor_type_count = sizeof(sensor_type_map) / sizeof(struct bma2x2_type_map_t);

	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		if (bma2x2_smbus_read_byte(client, BMA2X2_CHIP_ID_REG,&chip_id) < 0) {
			PERR("Bosch Sensortec Device not found" "i2c bus read error, read chip_id:%d\n", chip_id);
			APS_ERR("bma2x2_check_chip_id err = %d\n", chip_id);
			continue;
		} else {
			APS_ERR("bma2x2_check_chip_id chip_id = %d\n", chip_id);
			for (i = 0; i < bma2x2_sensor_type_count; i++) {
				if (sensor_type_map[i].chip_id == chip_id) {
					data->sensor_type = sensor_type_map[i].sensor_type;
					data->chip_id = chip_id;
//					PINFO("Bosch Sensortec Device detected," " HW IC name: %s\n", sensor_type_map[i].sensor_name);
					APS_LOG("bma2x2_check_chip_id(%d) Device name = %s\n",data->chip_id , sensor_type_map[i].sensor_name);
					return err;
				}
			}
			if (i < bma2x2_sensor_type_count)
				return err;
			else {
				if (read_count == CHECK_CHIP_ID_TIME_MAX) {
					PERR("Failed! Bosch Sensortec Device" " not found, mismatch chip_id:%d\n", chip_id);
					err = -ENODEV;
					return err;
				}
			}
		I2C_RETRY_DELAY();
		}
	}
	return err;
}

#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
static int bma2x2_set_newdata(struct i2c_client *client,
			unsigned char channel, unsigned char int_newdata)
{

	unsigned char data = 0;
	int comres = 0;

	switch (channel) {
	case BMA2X2_INT1_NDATA:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT1_PAD_NEWDATA__REG, &data);
		data = BMA2X2_SET_BITSLICE(data,
				BMA2X2_EN_INT1_PAD_NEWDATA, int_newdata);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT1_PAD_NEWDATA__REG, &data);
		break;
	case BMA2X2_INT2_NDATA:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT2_PAD_NEWDATA__REG, &data);
		data = BMA2X2_SET_BITSLICE(data,
				BMA2X2_EN_INT2_PAD_NEWDATA, int_newdata);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT2_PAD_NEWDATA__REG, &data);
		break;
	default:
		comres = -1;
		break;
	}

	return comres;

}
#endif /* CONFIG_BMA_ENABLE_NEWDATA_INT */

#ifdef BMA2X2_ENABLE_INT1
static int bma2x2_set_int1_pad_sel(struct i2c_client *client, unsigned char
		int1sel)
{
	int comres = 0;
	unsigned char data = 0;
	unsigned char state;
	state = 0x01;


	switch (int1sel) {
	case 0:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT1_PAD_LOWG__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_LOWG,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT1_PAD_LOWG__REG, &data);
		break;
	case 1:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT1_PAD_HIGHG__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_HIGHG,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT1_PAD_HIGHG__REG, &data);
		break;
	case 2:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT1_PAD_SLOPE__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_SLOPE,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT1_PAD_SLOPE__REG, &data);
		break;
	case 3:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT1_PAD_DB_TAP__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_DB_TAP,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT1_PAD_DB_TAP__REG, &data);
		break;
	case 4:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT1_PAD_SNG_TAP__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_SNG_TAP,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT1_PAD_SNG_TAP__REG, &data);
		break;
	case 5:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT1_PAD_ORIENT__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_ORIENT,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT1_PAD_ORIENT__REG, &data);
		break;
	case 6:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT1_PAD_FLAT__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_FLAT,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT1_PAD_FLAT__REG, &data);
		break;
	case 7:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT1_PAD_SLO_NO_MOT__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT1_PAD_SLO_NO_MOT,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT1_PAD_SLO_NO_MOT__REG, &data);
		break;

	default:
		break;
	}

	return comres;
}
#endif /* BMA2X2_ENABLE_INT1 */

#ifdef BMA2X2_ENABLE_INT2
static int bma2x2_set_int2_pad_sel(struct i2c_client *client, unsigned char
		int2sel)
{
	int comres = 0;
	unsigned char data = 0;
	unsigned char state;
	state = 0x01;


	switch (int2sel) {
	case 0:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT2_PAD_LOWG__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT2_PAD_LOWG,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT2_PAD_LOWG__REG, &data);
		break;
	case 1:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT2_PAD_HIGHG__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT2_PAD_HIGHG,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT2_PAD_HIGHG__REG, &data);
		break;
	case 2:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT2_PAD_SLOPE__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT2_PAD_SLOPE,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT2_PAD_SLOPE__REG, &data);
		break;
	case 3:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT2_PAD_DB_TAP__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT2_PAD_DB_TAP,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT2_PAD_DB_TAP__REG, &data);
		break;
	case 4:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT2_PAD_SNG_TAP__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT2_PAD_SNG_TAP,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT2_PAD_SNG_TAP__REG, &data);
		break;
	case 5:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT2_PAD_ORIENT__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT2_PAD_ORIENT,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT2_PAD_ORIENT__REG, &data);
		break;
	case 6:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT2_PAD_FLAT__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT2_PAD_FLAT,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT2_PAD_FLAT__REG, &data);
		break;
	case 7:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_EN_INT2_PAD_SLO_NO_MOT__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_INT2_PAD_SLO_NO_MOT,
				state);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_EN_INT2_PAD_SLO_NO_MOT__REG, &data);
		break;
	default:
		break;
	}

	return comres;
}
#endif /* BMA2X2_ENABLE_INT2 */

static int bma2x2_set_Int_Enable(struct i2c_client *client, unsigned char
		InterruptType , unsigned char value)
{
	int comres = 0;
	unsigned char data1 = 0;
	unsigned char data2 = 0;

	if ((11 < InterruptType) && (InterruptType < 16)) {
		switch (InterruptType) {
		case 12:
			/* slow/no motion X Interrupt  */
			comres = bma2x2_smbus_read_byte(client,
				BMA2X2_INT_SLO_NO_MOT_EN_X_INT__REG, &data1);
			data1 = BMA2X2_SET_BITSLICE(data1,
				BMA2X2_INT_SLO_NO_MOT_EN_X_INT, value);
			comres = bma2x2_smbus_write_byte(client,
				BMA2X2_INT_SLO_NO_MOT_EN_X_INT__REG, &data1);
			break;
		case 13:
			/* slow/no motion Y Interrupt  */
			comres = bma2x2_smbus_read_byte(client,
				BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__REG, &data1);
			data1 = BMA2X2_SET_BITSLICE(data1,
				BMA2X2_INT_SLO_NO_MOT_EN_Y_INT, value);
			comres = bma2x2_smbus_write_byte(client,
				BMA2X2_INT_SLO_NO_MOT_EN_Y_INT__REG, &data1);
			break;
		case 14:
			/* slow/no motion Z Interrupt  */
			comres = bma2x2_smbus_read_byte(client,
				BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__REG, &data1);
			data1 = BMA2X2_SET_BITSLICE(data1,
				BMA2X2_INT_SLO_NO_MOT_EN_Z_INT, value);
			comres = bma2x2_smbus_write_byte(client,
				BMA2X2_INT_SLO_NO_MOT_EN_Z_INT__REG, &data1);
			break;
		case 15:
			/* slow / no motion Interrupt select */
			comres = bma2x2_smbus_read_byte(client,
				BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__REG, &data1);
			data1 = BMA2X2_SET_BITSLICE(data1,
				BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT, value);
			comres = bma2x2_smbus_write_byte(client,
				BMA2X2_INT_SLO_NO_MOT_EN_SEL_INT__REG, &data1);
		}

	return comres;
	}

	comres = bma2x2_smbus_read_byte(client, BMA2X2_INT_ENABLE1_REG, &data1);
	comres = bma2x2_smbus_read_byte(client, BMA2X2_INT_ENABLE2_REG, &data2);

	value = value & 1;
	switch (InterruptType) {
	case 0:
		/* Low G Interrupt  */
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_EN_LOWG_INT, value);
		break;

	case 1:
		/* High G X Interrupt */
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_EN_HIGHG_X_INT,
				value);
		break;

	case 2:
		/* High G Y Interrupt */
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_EN_HIGHG_Y_INT,
				value);
		break;

	case 3:
		/* High G Z Interrupt */
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_EN_HIGHG_Z_INT,
				value);
		break;

	case 4:
		/* New Data Interrupt  */
		data2 = BMA2X2_SET_BITSLICE(data2, BMA2X2_EN_NEW_DATA_INT,
				value);
		break;

	case 5:
		/* Slope X Interrupt */
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_EN_SLOPE_X_INT,
				value);
		break;

	case 6:
		/* Slope Y Interrupt */
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_EN_SLOPE_Y_INT,
				value);
		break;

	case 7:
		/* Slope Z Interrupt */
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_EN_SLOPE_Z_INT,
				value);
		break;

	case 8:
		/* Single Tap Interrupt */
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_EN_SINGLE_TAP_INT,
				value);
		break;

	case 9:
		/* Double Tap Interrupt */
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_EN_DOUBLE_TAP_INT,
				value);
		break;

	case 10:
		/* Orient Interrupt  */
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_EN_ORIENT_INT, value);
		break;

	case 11:
		/* Flat Interrupt */
		data1 = BMA2X2_SET_BITSLICE(data1, BMA2X2_EN_FLAT_INT, value);
		break;

	default:
		break;
	}
	comres = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE1_REG,
			&data1);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_INT_ENABLE2_REG,
			&data2);

	return comres;
}


#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)
static int bma2x2_get_interruptstatus1(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_STATUS1_REG, &data);
	*intstatus = data;

	return comres;
}

#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
static int bma2x2_get_interruptstatus2(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_STATUS2_REG, &data);
	*intstatus = data;

	return comres;
}
#endif

#ifdef CONFIG_FLICK
	#ifdef CONFIG_FLICK_DEBUG
static int bma2x2_get_HIGH_first(struct i2c_client *client, unsigned char
						param, unsigned char *intstatus)
{
	int comres = 0;
	unsigned char data = 0;

	switch (param) {
	case 0:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_STATUS_ORIENT_HIGH_REG, &data);
		data = BMA2X2_GET_BITSLICE(data, BMA2X2_HIGHG_FIRST_X);
		*intstatus = data;
		break;
	case 1:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_STATUS_ORIENT_HIGH_REG, &data);
		data = BMA2X2_GET_BITSLICE(data, BMA2X2_HIGHG_FIRST_Y);
		*intstatus = data;
		break;
	case 2:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_STATUS_ORIENT_HIGH_REG, &data);
		data = BMA2X2_GET_BITSLICE(data, BMA2X2_HIGHG_FIRST_Z);
		*intstatus = data;
		break;
	default:
		break;
	}

	return comres;
}

static int bma2x2_get_HIGH_sign(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_STATUS_ORIENT_HIGH_REG,
			&data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_HIGHG_SIGN_S);
	*intstatus = data;

	return comres;
}
	#endif
#endif


#ifndef CONFIG_SIG_MOTION
static int bma2x2_get_slope_first(struct i2c_client *client, unsigned char
	param, unsigned char *intstatus)
{
	int comres = 0;
	unsigned char data = 0;

	switch (param) {
	case 0:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_STATUS_TAP_SLOPE_REG, &data);
		data = BMA2X2_GET_BITSLICE(data, BMA2X2_SLOPE_FIRST_X);
		*intstatus = data;
		break;
	case 1:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_STATUS_TAP_SLOPE_REG, &data);
		data = BMA2X2_GET_BITSLICE(data, BMA2X2_SLOPE_FIRST_Y);
		*intstatus = data;
		break;
	case 2:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_STATUS_TAP_SLOPE_REG, &data);
		data = BMA2X2_GET_BITSLICE(data, BMA2X2_SLOPE_FIRST_Z);
		*intstatus = data;
		break;
	default:
		break;
	}

	return comres;
}

static int bma2x2_get_slope_sign(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_STATUS_TAP_SLOPE_REG,
			&data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_SLOPE_SIGN_S);
	*intstatus = data;

	return comres;
}
#endif

static int bma2x2_get_orient_status(struct i2c_client *client, unsigned char
		*intstatus)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_STATUS_ORIENT_HIGH_REG,
			&data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_ORIENT_S);
	*intstatus = data;

	return comres;
}

static int bma2x2_get_orient_flat_status(struct i2c_client *client, unsigned
		char *intstatus)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_STATUS_ORIENT_HIGH_REG,
			&data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_FLAT_S);
	*intstatus = data;

	return comres;
}
#endif /* defined(BMA2X2_ENABLE_INT1)||defined(BMA2X2_ENABLE_INT2) */

static int bma2x2_set_Int_Mode(struct i2c_client *client, unsigned char Mode)
{
	int comres = 0;
	unsigned char data = 0;


	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_INT_MODE_SEL__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_INT_MODE_SEL, Mode);
	comres = bma2x2_smbus_write_byte(client,
			BMA2X2_INT_MODE_SEL__REG, &data);


	return comres;
}

static int bma2x2_get_Int_Mode(struct i2c_client *client, unsigned char *Mode)
{
	int comres = 0;
	unsigned char data = 0;


	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_INT_MODE_SEL__REG, &data);
	data  = BMA2X2_GET_BITSLICE(data, BMA2X2_INT_MODE_SEL);
	*Mode = data;


	return comres;
}
static int bma2x2_set_slope_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data = 0;


	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_SLOPE_DUR__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_SLOPE_DUR, duration);
	comres = bma2x2_smbus_write_byte(client,
			BMA2X2_SLOPE_DUR__REG, &data);

	return comres;
}

static int bma2x2_get_slope_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;


	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_SLOPE_DURN_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_SLOPE_DUR);
	*status = data;


	return comres;
}

static int bma2x2_set_slope_no_mot_duration(struct i2c_client *client,
			unsigned char duration)
{
	int comres = 0;
	unsigned char data = 0;


	comres = bma2x2_smbus_read_byte(client,
			BMA2x2_SLO_NO_MOT_DUR__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2x2_SLO_NO_MOT_DUR, duration);
	comres = bma2x2_smbus_write_byte(client,
			BMA2x2_SLO_NO_MOT_DUR__REG, &data);


	return comres;
}

static int bma2x2_get_slope_no_mot_duration(struct i2c_client *client,
			unsigned char *status)
{
	int comres = 0;
	unsigned char data = 0;


	comres = bma2x2_smbus_read_byte(client,
			BMA2x2_SLO_NO_MOT_DUR__REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2x2_SLO_NO_MOT_DUR);
	*status = data;


	return comres;
}

static int bma2x2_set_slope_threshold(struct i2c_client *client,
		unsigned char threshold)
{
	int comres = 0;
	unsigned char data = 0;

	data = threshold;
	comres = bma2x2_smbus_write_byte(client,
			BMA2X2_SLOPE_THRES__REG, &data);

	return comres;
}

static int bma2x2_get_slope_threshold(struct i2c_client *client,
		unsigned char *status)
{
	int comres = 0;
	unsigned char data = 0;


	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_SLOPE_THRES_REG, &data);
	*status = data;

	return comres;
}

static int bma2x2_set_slope_no_mot_threshold(struct i2c_client *client,
		unsigned char threshold)
{
	int comres = 0;
	unsigned char data = 0;

	data = threshold;
	comres = bma2x2_smbus_write_byte(client,
			BMA2X2_SLO_NO_MOT_THRES_REG, &data);

	return comres;
}

static int bma2x2_get_slope_no_mot_threshold(struct i2c_client *client,
		unsigned char *status)
{
	int comres = 0;
	unsigned char data = 0;


	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_SLO_NO_MOT_THRES_REG, &data);
	*status = data;

	return comres;
}


static int bma2x2_set_low_g_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_LOWG_DUR__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_LOWG_DUR, duration);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_LOWG_DUR__REG, &data);

	return comres;
}

static int bma2x2_get_low_g_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_LOW_DURN_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_LOWG_DUR);
	*status = data;

	return comres;
}

static int bma2x2_set_low_g_threshold(struct i2c_client *client, unsigned char
		threshold)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_LOWG_THRES__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_LOWG_THRES, threshold);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_LOWG_THRES__REG, &data);

	return comres;
}

static int bma2x2_get_low_g_threshold(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_LOW_THRES_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_LOWG_THRES);
	*status = data;

	return comres;
}

static int bma2x2_set_high_g_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_HIGHG_DUR__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_HIGHG_DUR, duration);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_HIGHG_DUR__REG, &data);

	return comres;
}

static int bma2x2_get_high_g_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_HIGH_DURN_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_HIGHG_DUR);
	*status = data;

	return comres;
}

static int bma2x2_set_high_g_threshold(struct i2c_client *client, unsigned char
		threshold)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_HIGHG_THRES__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_HIGHG_THRES, threshold);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_HIGHG_THRES__REG,
			&data);

	return comres;
}

static int bma2x2_get_high_g_threshold(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_HIGH_THRES_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_HIGHG_THRES);
	*status = data;

	return comres;
}


static int bma2x2_set_tap_duration(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_DUR__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_TAP_DUR, duration);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_DUR__REG, &data);

	return comres;
}

static int bma2x2_get_tap_duration(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_PARAM_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_TAP_DUR);
	*status = data;

	return comres;
}

static int bma2x2_set_tap_shock(struct i2c_client *client, unsigned char setval)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_SHOCK_DURN__REG,
			&data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_TAP_SHOCK_DURN, setval);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_SHOCK_DURN__REG,
			&data);

	return comres;
}

static int bma2x2_get_tap_shock(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_PARAM_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_TAP_SHOCK_DURN);
	*status = data;

	return comres;
}

static int bma2x2_set_tap_quiet(struct i2c_client *client, unsigned char
		duration)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_QUIET_DURN__REG,
			&data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_TAP_QUIET_DURN, duration);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_QUIET_DURN__REG,
			&data);

	return comres;
}

static int bma2x2_get_tap_quiet(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_PARAM_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_TAP_QUIET_DURN);
	*status = data;

	return comres;
}

static int bma2x2_set_tap_threshold(struct i2c_client *client, unsigned char
		threshold)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_THRES__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_TAP_THRES, threshold);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_THRES__REG, &data);

	return comres;
}

static int bma2x2_get_tap_threshold(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_THRES_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_TAP_THRES);
	*status = data;

	return comres;
}

static int bma2x2_set_tap_samp(struct i2c_client *client, unsigned char samp)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_SAMPLES__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_TAP_SAMPLES, samp);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_SAMPLES__REG,
			&data);

	return comres;
}

static int bma2x2_get_tap_samp(struct i2c_client *client, unsigned char *status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_THRES_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_TAP_SAMPLES);
	*status = data;

	return comres;
}

static int bma2x2_set_orient_mode(struct i2c_client *client, unsigned char mode)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_ORIENT_MODE__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_ORIENT_MODE, mode);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_ORIENT_MODE__REG,
			&data);

	return comres;
}

static int bma2x2_get_orient_mode(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_ORIENT_PARAM_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_ORIENT_MODE);
	*status = data;

	return comres;
}

static int bma2x2_set_orient_blocking(struct i2c_client *client, unsigned char
		samp)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_ORIENT_BLOCK__REG,
			&data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_ORIENT_BLOCK, samp);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_ORIENT_BLOCK__REG,
			&data);

	return comres;
}

static int bma2x2_get_orient_blocking(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_ORIENT_PARAM_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_ORIENT_BLOCK);
	*status = data;

	return comres;
}

static int bma2x2_set_orient_hyst(struct i2c_client *client, unsigned char
		orienthyst)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_ORIENT_HYST__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_ORIENT_HYST, orienthyst);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_ORIENT_HYST__REG,
			&data);

	return comres;
}

static int bma2x2_get_orient_hyst(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_ORIENT_PARAM_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_ORIENT_HYST);
	*status = data;

	return comres;
}
static int bma2x2_set_theta_blocking(struct i2c_client *client, unsigned char
		thetablk)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_THETA_BLOCK__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_THETA_BLOCK, thetablk);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_THETA_BLOCK__REG,
			&data);

	return comres;
}

static int bma2x2_get_theta_blocking(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_THETA_BLOCK_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_THETA_BLOCK);
	*status = data;

	return comres;
}

static int bma2x2_set_theta_flat(struct i2c_client *client, unsigned char
		thetaflat)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_THETA_FLAT__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_THETA_FLAT, thetaflat);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_THETA_FLAT__REG, &data);

	return comres;
}

static int bma2x2_get_theta_flat(struct i2c_client *client, unsigned char
		*status)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_THETA_FLAT_REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_THETA_FLAT);
	*status = data;

	return comres;
}

static int bma2x2_set_flat_hold_time(struct i2c_client *client, unsigned char
		holdtime)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_FLAT_HOLD_TIME__REG,
			&data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_FLAT_HOLD_TIME, holdtime);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_FLAT_HOLD_TIME__REG,
			&data);

	return comres;
}

static int bma2x2_get_flat_hold_time(struct i2c_client *client, unsigned char
		*holdtime)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_FLAT_HOLD_TIME_REG,
			&data);
	data  = BMA2X2_GET_BITSLICE(data, BMA2X2_FLAT_HOLD_TIME);
	*holdtime = data;

	return comres;
}

//static void CTS_report_dummy_event(struct work_struct *work) 
//{
//	struct bma2x2_data *bma2x2 = g_accel_data;
//	msleep(50);
//	switch (bma2x2->en_sensor_type) {
//		case 1:
//			/* Flick */
//			APS_LOG("Report NO_FLICK for Flick CTS\n");
//			input_report_abs(g_accel_data->dev_flick, MOTION_FLICK_INTERRUPT, -1);
//			input_report_abs(g_accel_data->dev_flick, MOTION_FLICK_INTERRUPT, NO_FLICK);
//			input_sync(g_accel_data->dev_flick);
//			break;
//		case 2:
//			/* Flick HV */
//			APS_LOG("Report NO_FLICK for Flick HV CTS\n");
//			input_report_abs(g_accel_data->dev_flick_hv, MOTION_FLICK_INTERRUPT, -1);
//			input_report_abs(g_accel_data->dev_flick_hv, MOTION_FLICK_INTERRUPT, NO_FLICK);
//			input_sync(g_accel_data->dev_flick_hv);
//			break;
//		case 3:
//			/* Terminal */
//			APS_LOG("Report dummy event for Terminal CTS\n");
//			input_report_abs(g_accel_data->dev_terminal, MOTION_TERMINAL_INTERRUPT, 0);
//			input_report_abs(g_accel_data->dev_terminal, MOTION_TERMINAL_INTERRUPT, -1);
//			input_sync(g_accel_data->dev_terminal);
//			break;
//		case 4:
//			/* Double Tap */
//			APS_LOG("Report UNKNOW for Double Tap CTS\n");
//			input_report_rel(g_accel_data->dev_interrupt,
//					UNKNOW_TAP_INTERRUPT,
//					DOUBLE_TAP_INTERRUPT_HAPPENED);
//			input_sync(g_accel_data->dev_interrupt);
//			break;
//	}
//}

static void bma2x2_workqueue_read_from_califile(struct work_struct *work) 
{
	if(g_accel_data->cali_retry > 5) {
		APS_LOG("re-try out of times = %d\n", g_accel_data->cali_retry);
		cancel_delayed_work(&g_accel_data->cali_work);
		return;

	}else 
		g_accel_data->cali_retry++;

	APS_LOG("start to do calibration work queue \n");
	bma2x2_read_from_califile();
}

static void bma2x2_read_from_califile(void) 
{
	int ilen = 0;
	char tmp_data[BMA2X2_NVRAM_LENGTH] = {0};
	unsigned int long_raw_x=0, long_raw_y=0, long_raw_z=0;
	int raw_x = 0, raw_y = 0, raw_z = 0;
	struct file *fp = NULL;
	
	if(1 == g_accel_data->is_cali) {
		APS_LOG("Calibration already loaded !\n");
		if(!DEBUG)	
			return;
	}
		
	old_gs_fs = get_fs();
	set_fs(KERNEL_DS);
	fp=filp_open(GS_CALI_PATH,O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);
//	fp=filp_open(GS_CALI_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR(fp)){
		APS_LOG("filp_open fail [%s] \n", GS_CALI_PATH);
		return;
	}

	ilen = fp->f_op->read(fp,tmp_data,BMA2X2_NVRAM_LENGTH,&fp->f_pos);
	if(ilen == BMA2X2_NVRAM_LENGTH) {
		long_raw_x = tmp_data[0] |(tmp_data[1]<<8) | (tmp_data[2]<<16) |(tmp_data[3]<<24) ;   
		raw_x = (int)long_raw_x;
		long_raw_y = tmp_data[4] |(tmp_data[5]<<8) | (tmp_data[6]<<16) |(tmp_data[7]<<24) ;   
		raw_y = (int)long_raw_y;
		long_raw_z = tmp_data[8] |(tmp_data[9]<<8) | (tmp_data[10]<<16) |(tmp_data[11]<<24) ;   
		raw_z = (int)long_raw_z;
		g_accel_data->cali.x = raw_x/128;
		g_accel_data->cali.y = raw_y/128;
		g_accel_data->cali.z = raw_z/128;

		if(DEBUG) APS_LOG("raw_x [%d], raw_y[%d], raw_z[%d] ;  long_raw_x[%d], long_raw_z[%d], long_raw_z[%d] \n", raw_x, raw_y, raw_z, long_raw_x, long_raw_y, long_raw_z);
		if(DEBUG) APS_LOG("Load Calibration Data X[%d] , Y[%d] , Z[%d] \n", g_accel_data->cali.x, g_accel_data->cali.y, g_accel_data->cali.z);
		g_accel_data->is_cali = 1;
	} else {
		g_accel_data->cali.x = 0 ;
		g_accel_data->cali.y = 0 ;
		g_accel_data->cali.z = 0 ;
		if(DEBUG) APS_LOG("ilen[%d] != 14 \n", ilen);	
		if(DEBUG) APS_LOG("Use Defalut Calibration Data X[%d] , Y[%d] , Z[%d] \n", g_accel_data->cali.x, g_accel_data->cali.y, g_accel_data->cali.z);
		g_accel_data->is_cali = 0;
	}
	set_fs(old_gs_fs);
	filp_close(fp,NULL);

	if( (g_accel_data->cali.x==0) && (g_accel_data->cali.y==0) && (g_accel_data->cali.z==0) )
		g_accel_data->is_cali = 0;
	
}

static int bma2x2_write_to_califile(int raw_x,int raw_y,int raw_z)
{
	int retval = 0, ilen=0, iloop=0;
	int tmp_data[BMA2X2_NVRAM_LENGTH]={0};
	char nvram_data[BMA2X2_NVRAM_LENGTH]={0};
	unsigned int long_x=0,long_y=0,long_z=0;
	int ck=0, flag=0;	//for checknumber
	struct file *fp=NULL;
	
	old_gs_fs = get_fs();
	set_fs(KERNEL_DS);
	fp=filp_open(GS_CALI_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR(fp)){
		if(DEBUG) APS_LOG("GS filp_open fail\n");
		return retval;
	}

	// long_data = raw_data * (1/512) *9.80065 * 65536 / 9.80065 ==> long_data = raw_data*128
	// *1/512 -> Normaliate into ? g (for range 4G) ; *1/1024 -> Normaliate into ? g (for range 2G)
	// *9.80065 -> 1 g = 9.80065
	// *65536/9.80065 Normaliate into NVRam 
	long_x = (unsigned int) (raw_x * 128);
	long_y = (unsigned int) (raw_y * 128);
	long_z = (unsigned int) (raw_z * 128);
	//for X into NVRam
	for(iloop=0;iloop<4;iloop++){
		tmp_data[iloop]=long_x%256;
		long_x = long_x/256;
	}
	//for Y into NVRam
	for(iloop=0;iloop<4;iloop++){
		tmp_data[iloop+4]=long_y%256;
		long_y = long_y/256;
	}
	//for Z into NVRam
	for(iloop=0;iloop<4;iloop++){
		tmp_data[iloop+8]=long_z%256;
		long_z = long_z/256;
	}
	tmp_data[12]=0xAA;

	//for checknumber
	for(iloop=0;iloop<12;iloop++){
		if(flag){
			ck ^= tmp_data[iloop];
			flag = 0;
		}else{
			ck += tmp_data[iloop];
			flag = 1;
		}
	}
	tmp_data[13]=ck;
	if(DEBUG) APS_LOG("check number = %d\n",ck);
	if(DEBUG) APS_LOG("%2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x %3x %3x ", tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3], tmp_data[4], tmp_data[5], tmp_data[6], tmp_data[7], tmp_data[8], tmp_data[9], tmp_data[10], tmp_data[11], tmp_data[12], tmp_data[13]);
	
	//transfor into nvram char
	for(iloop=0;iloop<BMA2X2_NVRAM_LENGTH;iloop++){
			nvram_data[iloop] = tmp_data[iloop];
	}
//	sprintf(nvram_data, "%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x%2x", tmp_data[0], tmp_data[1], tmp_data[2], tmp_data[3], tmp_data[4], tmp_data[5], tmp_data[6], tmp_data[7], tmp_data[8], tmp_data[9], tmp_data[10], tmp_data[11], tmp_data[12], tmp_data[13]);
	ilen = fp->f_op->write(fp,nvram_data,BMA2X2_NVRAM_LENGTH,&fp->f_pos);
	if(DEBUG) APS_LOG("GS write file len = %d\n",ilen);
	set_fs(old_gs_fs);
	filp_close(fp,NULL);
	
	return retval;
}

/*!
 * brief: bma2x2 switch from normal to suspend mode
 * @param[i] bma2x2
 * @param[i] data1, write to PMU_LPW
 * @param[i] data2, write to PMU_LOW_NOSIE
 *
 * @return zero success, none-zero failed
 */
static int bma2x2_normal_to_suspend(struct bma2x2_data *bma2x2,
				unsigned char data1, unsigned char data2)
{
	unsigned char current_fifo_mode;
	unsigned char current_op_mode;
	if (bma2x2 == NULL)
		return -1;
	/* get current op mode from mode register */
	if (bma2x2_get_mode(bma2x2->bma2x2_client, &current_op_mode) < 0)
		return -1;
	/* only aimed at operatiom mode chang from normal/lpw1 mode
	 * to suspend state.
	*/
	if (current_op_mode == BMA2X2_MODE_NORMAL ||
			current_op_mode == BMA2X2_MODE_LOWPOWER1) {
		/* get current fifo mode from fifo config register */
		if (bma2x2_get_fifo_mode(bma2x2->bma2x2_client,
							&current_fifo_mode) < 0)
			return -1;
		else {
			bma2x2_smbus_write_byte(bma2x2->bma2x2_client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
			bma2x2_smbus_write_byte(bma2x2->bma2x2_client,
					BMA2X2_MODE_CTRL_REG, &data1);
			/*! Aim at fifo workarounds with FIFO_CONFIG_1 */
			current_fifo_mode |= FIFO_WORKAROUNDS_MSK;
			bma2x2_smbus_write_byte(bma2x2->bma2x2_client,
				BMA2X2_FIFO_MODE__REG, &current_fifo_mode);
			WAIT_DEVICE_READY();
			return 0;
		}
	} else {
		bma2x2_smbus_write_byte(bma2x2->bma2x2_client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
		bma2x2_smbus_write_byte(bma2x2->bma2x2_client,
					BMA2X2_MODE_CTRL_REG, &data1);
		WAIT_DEVICE_READY();
		return 0;
	}

}

static int bma2x2_set_mode(struct i2c_client *client, unsigned char mode,
						unsigned char enabled_mode)
{
	int comres = 0;
	unsigned char data1 = 0;
	unsigned char data2 = 0;
	int ret = 0;
//	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);
	APS_FUN();

	APS_LOG("Set Sensor Status [%s] , is_cali [%d] , bma_mode_enabled [%d]", mode == 0 ? "Nomal" : "Suspend" , g_accel_data->is_cali, g_accel_data->bma_mode_enabled);
	mutex_lock(&g_accel_data->mode_mutex);
	if (BMA2X2_MODE_SUSPEND == mode) {
		if (enabled_mode != BMA_ENABLED_ALL) {
			if ((g_accel_data->bma_mode_enabled &
						(1<<enabled_mode)) == 0) {
				/* sensor is already closed in this mode */
				printk("->[%d]\n", g_accel_data->bma_mode_enabled);
				mutex_unlock(&g_accel_data->mode_mutex);
				return 0;
			} else {
				g_accel_data->bma_mode_enabled &= ~(1<<enabled_mode);
			}
		} else {
			/* shut down, close all and force do it*/
			g_accel_data->bma_mode_enabled = 0;
		}
	} else if (BMA2X2_MODE_NORMAL == mode) {
		if ((g_accel_data->bma_mode_enabled & (1<<enabled_mode)) != 0) {
			/* sensor is already enabled in this mode */
			printk("->[%d]\n", g_accel_data->bma_mode_enabled);
			mutex_unlock(&g_accel_data->mode_mutex);
			return 0;
		} else {
			g_accel_data->bma_mode_enabled |= (1<<enabled_mode);
		}
	} else {
		/* other mode, close all and force do it*/
		g_accel_data->bma_mode_enabled = 0;
	}
	printk("->[%d]\n", g_accel_data->bma_mode_enabled);
	mutex_unlock(&g_accel_data->mode_mutex);

	if (mode < 6) {
		comres = bma2x2_smbus_read_byte(client, BMA2X2_MODE_CTRL_REG,
				&data1);
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_LOW_NOISE_CTRL_REG,
				&data2);
		switch (mode) {
		case BMA2X2_MODE_NORMAL:
				/* If Probe Read Fail When Sensor Enable Read again */
				if(0 == g_accel_data->is_cali)
					queue_delayed_work(g_accel_data->cali_wq, &g_accel_data->cali_work, 0);
				
				data1  = BMA2X2_SET_BITSLICE(data1,
						BMA2X2_MODE_CTRL, 0);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 0);
				bma2x2_smbus_write_byte(client,
						BMA2X2_MODE_CTRL_REG, &data1);
				WAIT_DEVICE_READY();
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
				break;
		case BMA2X2_MODE_LOWPOWER1:
				data1  = BMA2X2_SET_BITSLICE(data1,
						BMA2X2_MODE_CTRL, 2);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 0);
				bma2x2_smbus_write_byte(client,
						BMA2X2_MODE_CTRL_REG, &data1);
				WAIT_DEVICE_READY();
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
				break;
		case BMA2X2_MODE_SUSPEND:
				if (g_accel_data->bma_mode_enabled != 0) {
					APS_LOG("bma still working\n");
					return 0;
				}
				data1  = BMA2X2_SET_BITSLICE(data1,
						BMA2X2_MODE_CTRL, 4);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 0);
			/*aimed at anomaly resolution when switch to suspend*/
			ret = bma2x2_normal_to_suspend(g_accel_data, data1, data2);
			if (ret < 0)
				APS_LOG("Error switching to suspend\n");
				break;
		case BMA2X2_MODE_DEEP_SUSPEND:
				if (g_accel_data->bma_mode_enabled != 0) {
					APS_LOG("bma still working\n");
					return 0;
				} 
				data1  = BMA2X2_SET_BITSLICE(data1,
							BMA2X2_MODE_CTRL, 1);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 1);
				bma2x2_smbus_write_byte(client,
						BMA2X2_MODE_CTRL_REG, &data1);
			WAIT_DEVICE_READY();
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
				break;
		case BMA2X2_MODE_LOWPOWER2:
				data1  = BMA2X2_SET_BITSLICE(data1,
						BMA2X2_MODE_CTRL, 2);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 1);
				bma2x2_smbus_write_byte(client,
						BMA2X2_MODE_CTRL_REG, &data1);
			WAIT_DEVICE_READY();
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
				break;
		case BMA2X2_MODE_STANDBY:
				data1  = BMA2X2_SET_BITSLICE(data1,
						BMA2X2_MODE_CTRL, 4);
				data2  = BMA2X2_SET_BITSLICE(data2,
						BMA2X2_LOW_POWER_MODE, 1);
				bma2x2_smbus_write_byte(client,
					BMA2X2_LOW_NOISE_CTRL_REG, &data2);
			WAIT_DEVICE_READY();
				bma2x2_smbus_write_byte(client,
						BMA2X2_MODE_CTRL_REG, &data1);
				break;
		}
	} else {
		comres = -1;
	}

	return comres;
}


static int bma2x2_get_mode(struct i2c_client *client, unsigned char *mode)
{
	int comres = 0;
	unsigned char data1 = 0;
	unsigned char data2 = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_MODE_CTRL_REG, &data1);
	comres = bma2x2_smbus_read_byte(client, BMA2X2_LOW_NOISE_CTRL_REG,
			&data2);

	data1  = (data1 & 0xE0) >> 5;
	data2  = (data2 & 0x40) >> 6;


	if ((data1 == 0x00) && (data2 == 0x00)) {
		*mode  = BMA2X2_MODE_NORMAL;
	} else {
		if ((data1 == 0x02) && (data2 == 0x00)) {
			*mode  = BMA2X2_MODE_LOWPOWER1;
		} else {
			if ((data1 == 0x04 || data1 == 0x06) &&
						(data2 == 0x00)) {
				*mode  = BMA2X2_MODE_SUSPEND;
			} else {
				if (((data1 & 0x01) == 0x01)) {
					*mode  = BMA2X2_MODE_DEEP_SUSPEND;
				} else {
					if ((data1 == 0x02) &&
							(data2 == 0x01)) {
						*mode  = BMA2X2_MODE_LOWPOWER2;
					} else {
						if ((data1 == 0x04) && (data2 ==
									0x01)) {
							*mode  =
							BMA2X2_MODE_STANDBY;
						} else {
							*mode =
						BMA2X2_MODE_DEEP_SUSPEND;
						}
					}
				}
			}
		}
	}

	return comres;
}

static int bma2x2_set_range(struct i2c_client *client, unsigned char Range)
{
	int comres = 0;
	unsigned char data1 = 0;

	if ((Range == 3) || (Range == 5) || (Range == 8) || (Range == 12)) {
		comres = bma2x2_smbus_read_byte(client, BMA2X2_RANGE_SEL_REG,
				&data1);
		switch (Range) {
		case BMA2X2_RANGE_2G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 3);
			break;
		case BMA2X2_RANGE_4G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 5);
			break;
		case BMA2X2_RANGE_8G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 8);
			break;
		case BMA2X2_RANGE_16G:
			data1  = BMA2X2_SET_BITSLICE(data1,
					BMA2X2_RANGE_SEL, 12);
			break;
		default:
			break;
		}
		comres += bma2x2_smbus_write_byte(client, BMA2X2_RANGE_SEL_REG,
				&data1);
	} else {
		comres = -1;
	}

	return comres;
}

static int bma2x2_get_range(struct i2c_client *client, unsigned char *Range)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_RANGE_SEL__REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_RANGE_SEL);
	*Range = data;

	return comres;
}


static int bma2x2_set_bandwidth(struct i2c_client *client, unsigned char BW)
{
	int comres = 0;
	unsigned char data = 0;
	int Bandwidth = 0;

#ifdef CONFIG_DOUBLE_TAP
	if (atomic_read(&g_accel_data->en_double_tap) == 1) {
		APS_LOG("Skip set bandwith [%d] when Double Tap is turned on\n", BW);
		return comres;
	}
#endif

	if (BW > 7 && BW < 16) {
		switch (BW) {
		case BMA2X2_BW_7_81HZ:
			Bandwidth = BMA2X2_BW_7_81HZ;

			/*  7.81 Hz      64000 uS   */
			break;
		case BMA2X2_BW_15_63HZ:
			Bandwidth = BMA2X2_BW_15_63HZ;

			/*  15.63 Hz     32000 uS   */
			break;
		case BMA2X2_BW_31_25HZ:
			Bandwidth = BMA2X2_BW_31_25HZ;

			/*  31.25 Hz     16000 uS   */
			break;
		case BMA2X2_BW_62_50HZ:
			Bandwidth = BMA2X2_BW_62_50HZ;

			/*  62.50 Hz     8000 uS   */
			break;
		case BMA2X2_BW_125HZ:
			Bandwidth = BMA2X2_BW_125HZ;

			/*  125 Hz       4000 uS   */
			break;
		case BMA2X2_BW_250HZ:
			Bandwidth = BMA2X2_BW_250HZ;

			/*  250 Hz       2000 uS   */
			break;
		case BMA2X2_BW_500HZ:
			Bandwidth = BMA2X2_BW_500HZ;

			/*  500 Hz       1000 uS   */
			break;
		case BMA2X2_BW_1000HZ:
			Bandwidth = BMA2X2_BW_1000HZ;

			/*  1000 Hz      500 uS   */
			break;
		default:
			break;
		}
		comres = bma2x2_smbus_read_byte(client, BMA2X2_BANDWIDTH__REG,
				&data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_BANDWIDTH, Bandwidth);
		comres += bma2x2_smbus_write_byte(client, BMA2X2_BANDWIDTH__REG,
				&data);
	} else {
		comres = -1;
	}

	return comres;
}

static int bma2x2_get_bandwidth(struct i2c_client *client, unsigned char *BW)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_BANDWIDTH__REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_BANDWIDTH);
	*BW = data;

	return comres;
}

int bma2x2_get_sleep_duration(struct i2c_client *client, unsigned char
		*sleep_dur)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_SLEEP_DUR__REG, &data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_SLEEP_DUR);
	*sleep_dur = data;

	return comres;
}

int bma2x2_set_sleep_duration(struct i2c_client *client, unsigned char
		sleep_dur)
{
	int comres = 0;
	unsigned char data = 0;
	int sleep_duration = 0;

	if (sleep_dur > 4 && sleep_dur < 16) {
		switch (sleep_dur) {
		case BMA2X2_SLEEP_DUR_0_5MS:
			sleep_duration = BMA2X2_SLEEP_DUR_0_5MS;

			/*  0.5 MS   */
			break;
		case BMA2X2_SLEEP_DUR_1MS:
			sleep_duration = BMA2X2_SLEEP_DUR_1MS;

			/*  1 MS  */
			break;
		case BMA2X2_SLEEP_DUR_2MS:
			sleep_duration = BMA2X2_SLEEP_DUR_2MS;

			/*  2 MS  */
			break;
		case BMA2X2_SLEEP_DUR_4MS:
			sleep_duration = BMA2X2_SLEEP_DUR_4MS;

			/*  4 MS   */
			break;
		case BMA2X2_SLEEP_DUR_6MS:
			sleep_duration = BMA2X2_SLEEP_DUR_6MS;

			/*  6 MS  */
			break;
		case BMA2X2_SLEEP_DUR_10MS:
			sleep_duration = BMA2X2_SLEEP_DUR_10MS;

			/*  10 MS  */
			break;
		case BMA2X2_SLEEP_DUR_25MS:
			sleep_duration = BMA2X2_SLEEP_DUR_25MS;

			/*  25 MS  */
			break;
		case BMA2X2_SLEEP_DUR_50MS:
			sleep_duration = BMA2X2_SLEEP_DUR_50MS;

			/*  50 MS   */
			break;
		case BMA2X2_SLEEP_DUR_100MS:
			sleep_duration = BMA2X2_SLEEP_DUR_100MS;

			/*  100 MS  */
			break;
		case BMA2X2_SLEEP_DUR_500MS:
			sleep_duration = BMA2X2_SLEEP_DUR_500MS;

			/*  500 MS   */
			break;
		case BMA2X2_SLEEP_DUR_1S:
			sleep_duration = BMA2X2_SLEEP_DUR_1S;

			/*  1 SECS   */
			break;
		default:
			break;
		}
		comres = bma2x2_smbus_read_byte(client, BMA2X2_SLEEP_DUR__REG,
				&data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_SLEEP_DUR,
				sleep_duration);
		comres = bma2x2_smbus_write_byte(client, BMA2X2_SLEEP_DUR__REG,
				&data);
	} else {
		comres = -1;
	}


	return comres;
}

static int bma2x2_get_fifo_mode(struct i2c_client *client, unsigned char
		*fifo_mode)
{
	int comres;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_FIFO_MODE__REG, &data);
	*fifo_mode = BMA2X2_GET_BITSLICE(data, BMA2X2_FIFO_MODE);

	return comres;
}

static int bma2x2_set_fifo_mode(struct i2c_client *client, unsigned char
		fifo_mode)
{
	unsigned char data = 0;
	int comres = 0;

	if (fifo_mode < 4) {
		comres = bma2x2_smbus_read_byte(client, BMA2X2_FIFO_MODE__REG,
				&data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_FIFO_MODE, fifo_mode);
		/*! Aim at fifo workarounds with FIFO_CONFIG_1 */
		data |= FIFO_WORKAROUNDS_MSK;
		comres = bma2x2_smbus_write_byte(client, BMA2X2_FIFO_MODE__REG,
				&data);
	} else {
		comres = -1;
	}

	return comres;
}

static int bma2x2_get_fifo_trig(struct i2c_client *client, unsigned char
		*fifo_trig)
{
	int comres;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_FIFO_TRIGGER_ACTION__REG, &data);
	*fifo_trig = BMA2X2_GET_BITSLICE(data, BMA2X2_FIFO_TRIGGER_ACTION);

	return comres;
}

static int bma2x2_set_fifo_trig(struct i2c_client *client, unsigned char
		fifo_trig)
{
	unsigned char data = 0;
	int comres = 0;

	if (fifo_trig < 4) {
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_FIFO_TRIGGER_ACTION__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_FIFO_TRIGGER_ACTION,
				fifo_trig);
		/*! Aim at fifo workarounds with FIFO_CONFIG_1 */
		data |= FIFO_WORKAROUNDS_MSK;
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_FIFO_TRIGGER_ACTION__REG, &data);
	} else {
		comres = -1;
	}

	return comres;
}

static int bma2x2_get_fifo_trig_src(struct i2c_client *client, unsigned char
		*trig_src)
{
	int comres;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_FIFO_TRIGGER_SOURCE__REG, &data);
	*trig_src = BMA2X2_GET_BITSLICE(data, BMA2X2_FIFO_TRIGGER_SOURCE);

	return comres;
}

static int bma2x2_set_fifo_trig_src(struct i2c_client *client, unsigned char
		trig_src)
{
	unsigned char data = 0;
	int comres = 0;

	if (trig_src < 4) {
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_FIFO_TRIGGER_SOURCE__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_FIFO_TRIGGER_SOURCE,
				trig_src);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_FIFO_TRIGGER_SOURCE__REG, &data);
	} else {
		comres = -1;
	}

	return comres;
}

static int bma2x2_get_fifo_framecount(struct i2c_client *client, unsigned char
			 *framecount)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_FIFO_FRAME_COUNTER_S__REG, &data);
	*framecount = BMA2X2_GET_BITSLICE(data, BMA2X2_FIFO_FRAME_COUNTER_S);

	return comres;
}

static int bma2x2_get_fifo_data_sel(struct i2c_client *client, unsigned char
		*data_sel)
{
	int comres;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client,
			BMA2X2_FIFO_DATA_SELECT__REG, &data);
	*data_sel = BMA2X2_GET_BITSLICE(data, BMA2X2_FIFO_DATA_SELECT);

	return comres;
}

static int bma2x2_set_fifo_data_sel(struct i2c_client *client, unsigned char
		data_sel)
{
	unsigned char data = 0;
	int comres = 0;

	if (data_sel < 4) {
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_FIFO_DATA_SELECT__REG,
				&data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_FIFO_DATA_SELECT,
				data_sel);
		/*! Aim at fifo workarounds with FIFO_CONFIG_1 */
		data |= FIFO_WORKAROUNDS_MSK;
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_FIFO_DATA_SELECT__REG,
				&data);
	} else {
		comres = -1;
	}

	return comres;
}


static int bma2x2_get_offset_target(struct i2c_client *client, unsigned char
		channel, unsigned char *offset)
{
	unsigned char data = 0;
	int comres = 0;

	switch (channel) {
	case BMA2X2_CUT_OFF:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_COMP_CUTOFF__REG, &data);
		*offset = BMA2X2_GET_BITSLICE(data, BMA2X2_COMP_CUTOFF);
		break;
	case BMA2X2_OFFSET_TRIGGER_X:
		comres = bma2x2_smbus_read_byte(client,
			BMA2X2_COMP_TARGET_OFFSET_X__REG, &data);
		*offset = BMA2X2_GET_BITSLICE(data,
				BMA2X2_COMP_TARGET_OFFSET_X);
		break;
	case BMA2X2_OFFSET_TRIGGER_Y:
		comres = bma2x2_smbus_read_byte(client,
			BMA2X2_COMP_TARGET_OFFSET_Y__REG, &data);
		*offset = BMA2X2_GET_BITSLICE(data,
				BMA2X2_COMP_TARGET_OFFSET_Y);
		break;
	case BMA2X2_OFFSET_TRIGGER_Z:
		comres = bma2x2_smbus_read_byte(client,
			BMA2X2_COMP_TARGET_OFFSET_Z__REG, &data);
		*offset = BMA2X2_GET_BITSLICE(data,
				BMA2X2_COMP_TARGET_OFFSET_Z);
		break;
	default:
		comres = -1;
		break;
	}

	return comres;
}

static int bma2x2_set_offset_target(struct i2c_client *client, unsigned char
		channel, unsigned char offset)
{
	unsigned char data = 0;
	int comres = 0;

	switch (channel) {
	case BMA2X2_CUT_OFF:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_COMP_CUTOFF__REG, &data);
		data = BMA2X2_SET_BITSLICE(data, BMA2X2_COMP_CUTOFF,
				offset);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_COMP_CUTOFF__REG, &data);
		break;
	case BMA2X2_OFFSET_TRIGGER_X:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_COMP_TARGET_OFFSET_X__REG,
				&data);
		data = BMA2X2_SET_BITSLICE(data,
				BMA2X2_COMP_TARGET_OFFSET_X,
				offset);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_COMP_TARGET_OFFSET_X__REG,
				&data);
		break;
	case BMA2X2_OFFSET_TRIGGER_Y:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_COMP_TARGET_OFFSET_Y__REG,
				&data);
		data = BMA2X2_SET_BITSLICE(data,
				BMA2X2_COMP_TARGET_OFFSET_Y,
				offset);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_COMP_TARGET_OFFSET_Y__REG,
				&data);
		break;
	case BMA2X2_OFFSET_TRIGGER_Z:
		comres = bma2x2_smbus_read_byte(client,
				BMA2X2_COMP_TARGET_OFFSET_Z__REG,
				&data);
		data = BMA2X2_SET_BITSLICE(data,
				BMA2X2_COMP_TARGET_OFFSET_Z,
				offset);
		comres = bma2x2_smbus_write_byte(client,
				BMA2X2_COMP_TARGET_OFFSET_Z__REG,
				&data);
		break;
	default:
		comres = -1;
		break;
	}

	return comres;
}

static int bma2x2_get_cal_ready(struct i2c_client *client,
					unsigned char *calrdy)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_FAST_CAL_RDY_S__REG,
			&data);
	data = BMA2X2_GET_BITSLICE(data, BMA2X2_FAST_CAL_RDY_S);
	*calrdy = data;

	return comres;
}

static int bma2x2_set_cal_trigger(struct i2c_client *client, unsigned char
		caltrigger)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_CAL_TRIGGER__REG, &data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_CAL_TRIGGER, caltrigger);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_CAL_TRIGGER__REG,
			&data);

	return comres;
}

static int bma2x2_write_reg(struct i2c_client *client, unsigned char addr,
		unsigned char *data)
{
	int comres = 0;
	comres = bma2x2_smbus_write_byte(client, addr, data);

	return comres;
}


static int bma2x2_set_offset_x(struct i2c_client *client, unsigned char
		offsetfilt)
{
	int comres = 0;
	unsigned char data = 0;

	data =  offsetfilt;

#ifdef CONFIG_SENSORS_BMI058
	comres = bma2x2_smbus_write_byte(client, BMI058_OFFSET_X_AXIS_REG,
							&data);
#else
	comres = bma2x2_smbus_write_byte(client, BMA2X2_OFFSET_X_AXIS_REG,
						&data);
#endif

	return comres;
}


static int bma2x2_get_offset_x(struct i2c_client *client, unsigned char
						*offsetfilt)
{
	int comres = 0;
	unsigned char data = 0;

#ifdef CONFIG_SENSORS_BMI058
	comres = bma2x2_smbus_read_byte(client, BMI058_OFFSET_X_AXIS_REG,
							&data);
#else
	comres = bma2x2_smbus_read_byte(client, BMA2X2_OFFSET_X_AXIS_REG,
							&data);
#endif
	*offsetfilt = data;

	return comres;
}

static int bma2x2_set_offset_y(struct i2c_client *client, unsigned char
						offsetfilt)
{
	int comres = 0;
	unsigned char data = 0;

	data =  offsetfilt;

#ifdef CONFIG_SENSORS_BMI058
	comres = bma2x2_smbus_write_byte(client, BMI058_OFFSET_Y_AXIS_REG,
							&data);
#else
	comres = bma2x2_smbus_write_byte(client, BMA2X2_OFFSET_Y_AXIS_REG,
							&data);
#endif
	return comres;
}

static int bma2x2_get_offset_y(struct i2c_client *client, unsigned char
						*offsetfilt)
{
	int comres = 0;
	unsigned char data = 0;

#ifdef CONFIG_SENSORS_BMI058
	comres = bma2x2_smbus_read_byte(client, BMI058_OFFSET_Y_AXIS_REG,
							&data);
#else
	comres = bma2x2_smbus_read_byte(client, BMA2X2_OFFSET_Y_AXIS_REG,
							&data);
#endif
	*offsetfilt = data;

	return comres;
}

static int bma2x2_set_offset_z(struct i2c_client *client, unsigned char
						offsetfilt)
{
	int comres = 0;
	unsigned char data = 0;

	data =  offsetfilt;
	comres = bma2x2_smbus_write_byte(client, BMA2X2_OFFSET_Z_AXIS_REG,
						&data);

	return comres;
}

static int bma2x2_get_offset_z(struct i2c_client *client, unsigned char
						*offsetfilt)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_OFFSET_Z_AXIS_REG,
						&data);
	*offsetfilt = data;

	return comres;
}


static int bma2x2_set_selftest_st(struct i2c_client *client, unsigned char
		selftest)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_EN_SELF_TEST__REG,
			&data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_EN_SELF_TEST, selftest);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_EN_SELF_TEST__REG,
			&data);

	return comres;
}

static int bma2x2_set_selftest_stn(struct i2c_client *client, unsigned char stn)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_NEG_SELF_TEST__REG,
			&data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_NEG_SELF_TEST, stn);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_NEG_SELF_TEST__REG,
			&data);

	return comres;
}

static int bma2x2_set_selftest_amp(struct i2c_client *client, unsigned char amp)
{
	int comres = 0;
	unsigned char data = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_SELF_TEST_AMP__REG,
			&data);
	data = BMA2X2_SET_BITSLICE(data, BMA2X2_SELF_TEST_AMP, amp);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_SELF_TEST_AMP__REG,
			&data);

	return comres;
}

static int bma2x2_read_accel_x(struct i2c_client *client,
				signed char sensor_type, short *a_x)
{
	int comres = 0;
	unsigned char data[2];

	switch (sensor_type) {
	case 0:
		comres = bma2x2_smbus_read_byte_block(client,
					BMA2X2_ACC_X12_LSB__REG, data, 2);
		*a_x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X12_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X12_LSB__LEN));
		*a_x = *a_x << (sizeof(short)*8-(BMA2X2_ACC_X12_LSB__LEN
					+ BMA2X2_ACC_X_MSB__LEN));
		*a_x = *a_x >> (sizeof(short)*8-(BMA2X2_ACC_X12_LSB__LEN
					+ BMA2X2_ACC_X_MSB__LEN));
		break;
	case 1:
		comres = bma2x2_smbus_read_byte_block(client,
					BMA2X2_ACC_X10_LSB__REG, data, 2);
		*a_x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X10_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X10_LSB__LEN));
		*a_x = *a_x << (sizeof(short)*8-(BMA2X2_ACC_X10_LSB__LEN
					+ BMA2X2_ACC_X_MSB__LEN));
		*a_x = *a_x >> (sizeof(short)*8-(BMA2X2_ACC_X10_LSB__LEN
					+ BMA2X2_ACC_X_MSB__LEN));
		break;
	case 2:
		comres = bma2x2_smbus_read_byte_block(client,
					BMA2X2_ACC_X8_LSB__REG, data, 2);
		*a_x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X8_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X8_LSB__LEN));
		*a_x = *a_x << (sizeof(short)*8-(BMA2X2_ACC_X8_LSB__LEN
					+ BMA2X2_ACC_X_MSB__LEN));
		*a_x = *a_x >> (sizeof(short)*8-(BMA2X2_ACC_X8_LSB__LEN
					+ BMA2X2_ACC_X_MSB__LEN));
		break;
	case 3:
		comres = bma2x2_smbus_read_byte_block(client,
					BMA2X2_ACC_X14_LSB__REG, data, 2);
		*a_x = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_X14_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_X_MSB)<<(BMA2X2_ACC_X14_LSB__LEN));
		*a_x = *a_x << (sizeof(short)*8-(BMA2X2_ACC_X14_LSB__LEN
					+ BMA2X2_ACC_X_MSB__LEN));
		*a_x = *a_x >> (sizeof(short)*8-(BMA2X2_ACC_X14_LSB__LEN
					+ BMA2X2_ACC_X_MSB__LEN));
		break;
	default:
		break;
	}

	return comres;
}

static int bma2x2_soft_reset(struct i2c_client *client)
{
	int comres = 0;
	unsigned char data = BMA2X2_EN_SOFT_RESET_VALUE;

	comres = bma2x2_smbus_write_byte(client, BMA2X2_EN_SOFT_RESET__REG,
					&data);

	return comres;
}

static int bma2x2_read_accel_y(struct i2c_client *client,
				signed char sensor_type, short *a_y)
{
	int comres = 0;
	unsigned char data[2];

	switch (sensor_type) {
	case 0:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_Y12_LSB__REG, data, 2);
		*a_y = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_Y12_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y12_LSB__LEN));
		*a_y = *a_y << (sizeof(short)*8-(BMA2X2_ACC_Y12_LSB__LEN
						+ BMA2X2_ACC_Y_MSB__LEN));
		*a_y = *a_y >> (sizeof(short)*8-(BMA2X2_ACC_Y12_LSB__LEN
						+ BMA2X2_ACC_Y_MSB__LEN));
		break;
	case 1:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_Y10_LSB__REG, data, 2);
		*a_y = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_Y10_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y10_LSB__LEN));
		*a_y = *a_y << (sizeof(short)*8-(BMA2X2_ACC_Y10_LSB__LEN
						+ BMA2X2_ACC_Y_MSB__LEN));
		*a_y = *a_y >> (sizeof(short)*8-(BMA2X2_ACC_Y10_LSB__LEN
						+ BMA2X2_ACC_Y_MSB__LEN));
		break;
	case 2:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_Y8_LSB__REG, data, 2);
		*a_y = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_Y8_LSB)|
				(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y8_LSB__LEN));
		*a_y = *a_y << (sizeof(short)*8-(BMA2X2_ACC_Y8_LSB__LEN
						+ BMA2X2_ACC_Y_MSB__LEN));
		*a_y = *a_y >> (sizeof(short)*8-(BMA2X2_ACC_Y8_LSB__LEN
						+ BMA2X2_ACC_Y_MSB__LEN));
		break;
	case 3:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_Y14_LSB__REG, data, 2);
		*a_y = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_Y14_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_Y_MSB)<<(BMA2X2_ACC_Y14_LSB__LEN));
		*a_y = *a_y << (sizeof(short)*8-(BMA2X2_ACC_Y14_LSB__LEN
						+ BMA2X2_ACC_Y_MSB__LEN));
		*a_y = *a_y >> (sizeof(short)*8-(BMA2X2_ACC_Y14_LSB__LEN
						+ BMA2X2_ACC_Y_MSB__LEN));
		break;
	default:
		break;
	}

	return comres;
}

static int bma2x2_read_accel_z(struct i2c_client *client,
				signed char sensor_type, short *a_z)
{
	int comres = 0;
	unsigned char data[2];

	switch (sensor_type) {
	case 0:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_Z12_LSB__REG, data, 2);
		*a_z = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_Z12_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z12_LSB__LEN));
		*a_z = *a_z << (sizeof(short)*8-(BMA2X2_ACC_Z12_LSB__LEN
						+ BMA2X2_ACC_Z_MSB__LEN));
		*a_z = *a_z >> (sizeof(short)*8-(BMA2X2_ACC_Z12_LSB__LEN
						+ BMA2X2_ACC_Z_MSB__LEN));
		break;
	case 1:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_Z10_LSB__REG, data, 2);
		*a_z = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_Z10_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z10_LSB__LEN));
		*a_z = *a_z << (sizeof(short)*8-(BMA2X2_ACC_Z10_LSB__LEN
						+ BMA2X2_ACC_Z_MSB__LEN));
		*a_z = *a_z >> (sizeof(short)*8-(BMA2X2_ACC_Z10_LSB__LEN
						+ BMA2X2_ACC_Z_MSB__LEN));
		break;
	case 2:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_Z8_LSB__REG, data, 2);
		*a_z = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_Z8_LSB)|
			(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z8_LSB__LEN));
		*a_z = *a_z << (sizeof(short)*8-(BMA2X2_ACC_Z8_LSB__LEN
						+ BMA2X2_ACC_Z_MSB__LEN));
		*a_z = *a_z >> (sizeof(short)*8-(BMA2X2_ACC_Z8_LSB__LEN
						+ BMA2X2_ACC_Z_MSB__LEN));
		break;
	case 3:
		comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_Z14_LSB__REG, data, 2);
		*a_z = BMA2X2_GET_BITSLICE(data[0], BMA2X2_ACC_Z14_LSB)|
				(BMA2X2_GET_BITSLICE(data[1],
				BMA2X2_ACC_Z_MSB)<<(BMA2X2_ACC_Z14_LSB__LEN));
		*a_z = *a_z << (sizeof(short)*8-(BMA2X2_ACC_Z14_LSB__LEN
						+ BMA2X2_ACC_Z_MSB__LEN));
		*a_z = *a_z >> (sizeof(short)*8-(BMA2X2_ACC_Z14_LSB__LEN
						+ BMA2X2_ACC_Z_MSB__LEN));
		break;
	default:
		break;
	}

	return comres;
}


static int bma2x2_read_temperature(struct i2c_client *client,
					signed char *temperature)
{
	unsigned char data = 0;
	int comres = 0;

	comres = bma2x2_smbus_read_byte(client, BMA2X2_TEMPERATURE_REG, &data);
	*temperature = (signed char)data;

	return comres;
}

static ssize_t bma2x2_enable_int_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int type, value;
#ifdef CONFIG_SENSORS_BMI058
	int i;
#endif

	sscanf(buf, "%3d %3d", &type, &value);

#ifdef CONFIG_SENSORS_BMI058
	for (i = 0; i < sizeof(int_map) / sizeof(struct interrupt_map_t); i++) {
		if (int_map[i].x == type) {
			type = int_map[i].y;
			break;
		}
		if (int_map[i].y == type) {
			type = int_map[i].x;
			break;
		}
	}
#endif

	if (bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, type, value) < 0)
		return -EINVAL;

	return count;
}


static ssize_t bma2x2_int_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	if (bma2x2_get_Int_Mode(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_int_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_Int_Mode(g_accel_data->bma2x2_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma2x2_slope_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	if (bma2x2_get_slope_duration(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_slope_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_slope_duration(g_accel_data->bma2x2_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_slope_no_mot_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	if (bma2x2_get_slope_no_mot_duration(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_slope_no_mot_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_slope_no_mot_duration(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}


static ssize_t bma2x2_slope_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_slope_threshold(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_slope_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_slope_threshold(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_slope_no_mot_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_slope_no_mot_threshold(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_slope_no_mot_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_slope_no_mot_threshold(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_high_g_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_high_g_duration(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_high_g_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_high_g_duration(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_high_g_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_high_g_threshold(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_high_g_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_high_g_threshold(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_low_g_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_low_g_duration(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_low_g_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_low_g_duration(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_low_g_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_low_g_threshold(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_low_g_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_low_g_threshold(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma2x2_tap_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_tap_threshold(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_tap_threshold_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_tap_threshold(g_accel_data->bma2x2_client, (unsigned char)data)
			< 0)
		return -EINVAL;

	return count;
}
static ssize_t bma2x2_tap_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_tap_duration(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_tap_duration_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_tap_duration(g_accel_data->bma2x2_client, (unsigned char)data)
			< 0)
		return -EINVAL;

	return count;
}
static ssize_t bma2x2_tap_quiet_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_tap_quiet(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_tap_quiet_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_tap_quiet(g_accel_data->bma2x2_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_tap_shock_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_tap_shock(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_tap_shock_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_tap_shock(g_accel_data->bma2x2_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_tap_samp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_tap_samp(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_tap_samp_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_tap_samp(g_accel_data->bma2x2_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_orient_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_orient_mode(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_orient_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_orient_mode(g_accel_data->bma2x2_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_orient_blocking_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_orient_blocking(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_orient_blocking_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_orient_blocking(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma2x2_orient_hyst_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_orient_hyst(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_orient_hyst_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_orient_hyst(g_accel_data->bma2x2_client, (unsigned char)data) <
			0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_orient_theta_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	if (bma2x2_get_theta_blocking(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_orient_theta_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_theta_blocking(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_flat_theta_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	if (bma2x2_get_theta_flat(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_flat_theta_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_theta_flat(g_accel_data->bma2x2_client, (unsigned char)data) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma2x2_flat_hold_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_flat_hold_time(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}
static ssize_t bma2x2_selftest_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = to_i2c_client(dev);
//	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", atomic_read(&g_accel_data->selftest_result));

}

static ssize_t bma2x2_softreset_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	if (bma2x2_soft_reset(g_accel_data->bma2x2_client) < 0)
		return -EINVAL;

	return count;
}
static ssize_t bma2x2_selftest_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{

	unsigned long data;
	unsigned char clear_value = 0;
	int error;
	short value1 = 0;
	short value2 = 0;
	short diff = 0;
	unsigned long result = 0;
	unsigned char test_result_branch = 0;
//	struct i2c_client *client = to_i2c_client(dev);
//	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	bma2x2_soft_reset(g_accel_data->bma2x2_client);
	mdelay(5);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (data != 1)
		return -EINVAL;

	bma2x2_write_reg(g_accel_data->bma2x2_client, 0x32, &clear_value);

	if ((g_accel_data->sensor_type == BMA280_TYPE) ||
		(g_accel_data->sensor_type == BMA255_TYPE)) {
#ifdef CONFIG_SENSORS_BMI058
		/*set self test amp */
		if (bma2x2_set_selftest_amp(g_accel_data->bma2x2_client, 1) < 0)
			return -EINVAL;
		/* set to 8 G range */
		if (bma2x2_set_range(g_accel_data->bma2x2_client,
							BMA2X2_RANGE_8G) < 0)
			return -EINVAL;
#else
		/* set to 4 G range */
		if (bma2x2_set_range(g_accel_data->bma2x2_client,
							BMA2X2_RANGE_4G) < 0)
			return -EINVAL;
#endif
	}

	if ((g_accel_data->sensor_type == BMA250E_TYPE) ||
			(g_accel_data->sensor_type == BMA222E_TYPE)) {
		/* set to 8 G range */
		if (bma2x2_set_range(g_accel_data->bma2x2_client, 8) < 0)
			return -EINVAL;
		if (bma2x2_set_selftest_amp(g_accel_data->bma2x2_client, 1) < 0)
			return -EINVAL;
	}

	/* 1 for x-axis(but BMI058 is 1 for y-axis )*/
	bma2x2_set_selftest_st(g_accel_data->bma2x2_client, 1);
	bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 0);
	mdelay(10);
	bma2x2_read_accel_x(g_accel_data->bma2x2_client,
					g_accel_data->sensor_type, &value1);
	bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 1);
	mdelay(10);
	bma2x2_read_accel_x(g_accel_data->bma2x2_client,
					g_accel_data->sensor_type, &value2);
	diff = value1-value2;

#ifdef CONFIG_SENSORS_BMI058
	PINFO("diff y is %d,value1 is %d, value2 is %d\n", diff,
				value1, value2);
	test_result_branch = 2;
#else
	PINFO("diff x is %d,value1 is %d, value2 is %d\n", diff,
				value1, value2);
	test_result_branch = 1;
#endif

	if (g_accel_data->sensor_type == BMA280_TYPE) {
#ifdef CONFIG_SENSORS_BMI058
		if (abs(diff) < 819)
			result |= test_result_branch;
#else
		if (abs(diff) < 1638)
			result |= test_result_branch;
#endif
	}
	if (g_accel_data->sensor_type == BMA255_TYPE) {
		if (abs(diff) < 409)
			result |= 1;
	}
	if (g_accel_data->sensor_type == BMA250E_TYPE) {
		if (abs(diff) < 51)
			result |= 1;
	}
	if (g_accel_data->sensor_type == BMA222E_TYPE) {
		if (abs(diff) < 12)
			result |= 1;
	}

	/* 2 for y-axis but BMI058 is 1*/
	bma2x2_set_selftest_st(g_accel_data->bma2x2_client, 2);
	bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 0);
	mdelay(10);
	bma2x2_read_accel_y(g_accel_data->bma2x2_client,
					g_accel_data->sensor_type, &value1);
	bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 1);
	mdelay(10);
	bma2x2_read_accel_y(g_accel_data->bma2x2_client,
					g_accel_data->sensor_type, &value2);
	diff = value1-value2;

#ifdef CONFIG_SENSORS_BMI058
	PINFO("diff x is %d,value1 is %d, value2 is %d\n", diff,
				value1, value2);
	test_result_branch = 1;
#else
	PINFO("diff y is %d,value1 is %d, value2 is %d\n", diff,
				value1, value2);
	test_result_branch = 2;
#endif

	if (g_accel_data->sensor_type == BMA280_TYPE) {
#ifdef CONFIG_SENSORS_BMI058
		if (abs(diff) < 819)
			result |= test_result_branch;
#else
		if (abs(diff) < 1638)
			result |= test_result_branch;
#endif
	}
	if (g_accel_data->sensor_type == BMA255_TYPE) {
		if (abs(diff) < 409)
			result |= test_result_branch;
	}
	if (g_accel_data->sensor_type == BMA250E_TYPE) {
		if (abs(diff) < 51)
			result |= test_result_branch;
	}
	if (g_accel_data->sensor_type == BMA222E_TYPE) {
		if (abs(diff) < 12)
			result |= test_result_branch;
	}


	bma2x2_set_selftest_st(g_accel_data->bma2x2_client, 3); /* 3 for z-axis*/
	bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 0);
	mdelay(10);
	bma2x2_read_accel_z(g_accel_data->bma2x2_client,
					g_accel_data->sensor_type, &value1);
	bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 1);
	mdelay(10);
	bma2x2_read_accel_z(g_accel_data->bma2x2_client,
					g_accel_data->sensor_type, &value2);
	diff = value1-value2;

	PINFO("diff z is %d,value1 is %d, value2 is %d\n", diff,
			value1, value2);

	if (g_accel_data->sensor_type == BMA280_TYPE) {
#ifdef CONFIG_SENSORS_BMI058
			if (abs(diff) < 409)
				result |= 4;
#else
			if (abs(diff) < 819)
				result |= 4;
#endif
	}
	if (g_accel_data->sensor_type == BMA255_TYPE) {
		if (abs(diff) < 204)
			result |= 4;
	}
	if (g_accel_data->sensor_type == BMA250E_TYPE) {
		if (abs(diff) < 25)
			result |= 4;
	}
	if (g_accel_data->sensor_type == BMA222E_TYPE) {
		if (abs(diff) < 6)
			result |= 4;
	}

	/* self test for bma254 */
	if ((g_accel_data->sensor_type == BMA255_TYPE) && (result > 0)) {
		result = 0;
		bma2x2_soft_reset(g_accel_data->bma2x2_client);
		mdelay(5);
		bma2x2_write_reg(g_accel_data->bma2x2_client, 0x32, &clear_value);
		/* set to 8 G range */
		if (bma2x2_set_range(g_accel_data->bma2x2_client, 8) < 0)
			return -EINVAL;
		if (bma2x2_set_selftest_amp(g_accel_data->bma2x2_client, 1) < 0)
			return -EINVAL;

		bma2x2_set_selftest_st(g_accel_data->bma2x2_client, 1); /* 1
								for x-axis*/
		bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 0); /*
							positive direction*/
		mdelay(10);
		bma2x2_read_accel_x(g_accel_data->bma2x2_client,
						g_accel_data->sensor_type, &value1);
		bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 1); /*
							negative direction*/
		mdelay(10);
		bma2x2_read_accel_x(g_accel_data->bma2x2_client,
						g_accel_data->sensor_type, &value2);
		diff = value1-value2;

		PINFO("diff x is %d,value1 is %d, value2 is %d\n",
						diff, value1, value2);
		if (abs(diff) < 204)
			result |= 1;

		bma2x2_set_selftest_st(g_accel_data->bma2x2_client, 2); /* 2
								for y-axis*/
		bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 0); /*
							positive direction*/
		mdelay(10);
		bma2x2_read_accel_y(g_accel_data->bma2x2_client,
						g_accel_data->sensor_type, &value1);
		bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 1); /*
							negative direction*/
		mdelay(10);
		bma2x2_read_accel_y(g_accel_data->bma2x2_client,
						g_accel_data->sensor_type, &value2);
		diff = value1-value2;
		PINFO("diff y is %d,value1 is %d, value2 is %d\n",
						diff, value1, value2);

		if (abs(diff) < 204)
			result |= 2;

		bma2x2_set_selftest_st(g_accel_data->bma2x2_client, 3); /* 3
								for z-axis*/
		bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 0); /*
							positive direction*/
		mdelay(10);
		bma2x2_read_accel_z(g_accel_data->bma2x2_client,
						g_accel_data->sensor_type, &value1);
		bma2x2_set_selftest_stn(g_accel_data->bma2x2_client, 1); /*
							negative direction*/
		mdelay(10);
		bma2x2_read_accel_z(g_accel_data->bma2x2_client,
						g_accel_data->sensor_type, &value2);
		diff = value1-value2;

		PINFO("diff z is %d,value1 is %d, value2 is %d\n",
						diff, value1, value2);
		if (abs(diff) < 102)
			result |= 4;
	}

	atomic_set(&g_accel_data->selftest_result, (unsigned int)result);

	bma2x2_soft_reset(g_accel_data->bma2x2_client);
	mdelay(5);
	PINFO("self test finished\n");

	return count;
}



static ssize_t bma2x2_flat_hold_time_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_flat_hold_time(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

const int bma2x2_sensor_bitwidth[] = {
	12,  10,  8, 14
};

static int bma2x2_read_accel_xyz(struct i2c_client *client,
		signed char sensor_type, struct bma2x2acc *acc)
{
	int comres = 0;
	unsigned char data[6];
//	struct bma2x2_data *client_data = i2c_get_clientdata(client);
#ifndef BMA2X2_SENSOR_IDENTIFICATION_ENABLE
	int bitwidth;
#endif
	comres = bma2x2_smbus_read_byte_block(client,
				BMA2X2_ACC_X12_LSB__REG, data, 6);
	if (sensor_type >= 4)
		return -EINVAL;

	acc->x = (data[1]<<8)|data[0];
	acc->y = (data[3]<<8)|data[2];
	acc->z = (data[5]<<8)|data[4];

#ifndef BMA2X2_SENSOR_IDENTIFICATION_ENABLE
	bitwidth = bma2x2_sensor_bitwidth[sensor_type];

	acc->x = (acc->x >> (16 - bitwidth));
	acc->y = (acc->y >> (16 - bitwidth));
	acc->z = (acc->z >> (16 - bitwidth));
#endif

	bma2x2_remap_sensor_data(acc, g_accel_data);
	return comres;
}

#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
static void bma2x2_work_func(struct work_struct *work)
{
//	struct bma2x2_data *bma2x2 = container_of((struct delayed_work *)work, struct bma2x2_data, work);
	static struct bma2x2acc acc;
	unsigned long delay = msecs_to_jiffies(atomic_read(&g_accel_data->delay));

	bma2x2_read_accel_xyz(g_accel_data->bma2x2_client, g_accel_data->sensor_type, &acc);
	
	// Add by Tom for Calibration
	if(g_accel_data->is_cali == 1) {
		acc.x += g_accel_data->cali.x;
		acc.y += g_accel_data->cali.y;
		acc.z += g_accel_data->cali.z;
	}

	input_report_abs(g_accel_data->input, ABS_X, acc.x);
	input_report_abs(g_accel_data->input, ABS_Y, acc.y);
	input_report_abs(g_accel_data->input, ABS_Z, acc.z);
	input_sync(g_accel_data->input);
	mutex_lock(&g_accel_data->value_mutex);
	g_accel_data->value = acc;
	mutex_unlock(&g_accel_data->value_mutex);
	schedule_delayed_work(&g_accel_data->work, delay);
	// APS_LOG("x[%d], y[%d], z[%d], delay[%ld]\n",acc.x ,acc.y ,acc.z, delay);
}
#endif

static ssize_t bma2x2_register_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int address, value;

	sscanf(buf, "%3d %3d", &address, &value);
	if (bma2x2_write_reg(g_accel_data->bma2x2_client, (unsigned char)address,
				(unsigned char *)&value) < 0)
		return -EINVAL;
	return count;
}
static ssize_t bma2x2_register_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	u8 reg[0x40];
	int i;

	for (i = 0; i < 0x40; i++) {
		bma2x2_smbus_read_byte(g_accel_data->bma2x2_client, i, reg+i);

		count += sprintf(&buf[count], "0x%x: %d\n", i, reg[i]);
	}
	return count;


}

static ssize_t bma2x2_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
//	struct i2c_client *client = to_i2c_client(dev);
//	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	if (bma2x2_get_range(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_range_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_range(g_accel_data->bma2x2_client, (unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_bandwidth_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_bandwidth(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_bandwidth_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (g_accel_data->sensor_type == BMA280_TYPE)
		if ((unsigned char) data > 14)
			return -EINVAL;

	APS_LOG("set bandwith [%lu] from AP\n", data);
	if (bma2x2_set_bandwidth(g_accel_data->bma2x2_client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_mode(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d %d\n", data, g_accel_data->bma_mode_enabled);
}

static ssize_t bma2x2_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_mode(g_accel_data->bma2x2_client,
		(unsigned char) data, BMA_ENABLED_BSX) < 0)
			return -EINVAL;
	else
		atomic_set(&g_accel_data->en_bst, (unsigned int) data);

	return count;
}

static ssize_t bma2x2_value_cache_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bma2x2acc acc_value;

	mutex_lock(&g_accel_data->value_mutex);
	acc_value = g_accel_data->value;
	mutex_unlock(&g_accel_data->value_mutex);

	return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t bma2x2_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bma2x2acc acc_value;

	bma2x2_read_accel_xyz(g_accel_data->bma2x2_client, g_accel_data->sensor_type,
								&acc_value);

	/* Add by Tom for Calibration */
	if(g_accel_data->is_cali == 1) {
		acc_value.x += g_accel_data->cali.x;
		acc_value.y += g_accel_data->cali.y;
		acc_value.z += g_accel_data->cali.z;
	}

	return sprintf(buf, "%d %d %d\n", acc_value.x, acc_value.y,
			acc_value.z);
}

static ssize_t bma2x2_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&g_accel_data->delay));
}

static ssize_t bma2x2_chip_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", g_accel_data->chip_id);
}

static ssize_t bma2x2_place_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
#ifdef CONFIG_Z380M
	int place = Z380M_BMA2X2_DEFAULT_PLACE;
#endif

#ifdef CONFIG_Z300M
	int place = Z300M_BMA2X2_DEFAULT_PLACE;
#endif

	if (NULL != g_accel_data->bst_pd)
		place = g_accel_data->bst_pd->place;

	return sprintf(buf, "%d\n", place);
}


static ssize_t bma2x2_delay_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (data > BMA2X2_MAX_DELAY)
		data = BMA2X2_MAX_DELAY;
	atomic_set(&g_accel_data->delay, (unsigned int) data);

	return count;
}


static ssize_t bma2x2_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	DEBUG = 1;
	return sprintf(buf, "%d\n", atomic_read(&g_accel_data->enable));

}

static void bma2x2_set_enable(struct device *dev, int enable)
{
	int pre_enable = atomic_read(&g_accel_data->enable);
	static struct bma2x2acc acc;

	APS_FUN();
	APS_LOG("Set enable [%d] pre_enable [%d]\n", enable, pre_enable);
	mutex_lock(&g_accel_data->enable_mutex);
	if (enable) {
		if (pre_enable == 0) {
			bma2x2_set_mode(g_accel_data->bma2x2_client,
					BMA2X2_MODE_NORMAL, BMA_ENABLED_INPUT);
#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
			schedule_delayed_work(&g_accel_data->work, msecs_to_jiffies(atomic_read(&g_accel_data->delay)));
#endif
			atomic_set(&g_accel_data->enable, 1);
		}

	} else {
		if (pre_enable == 1) {
			bma2x2_set_mode(g_accel_data->bma2x2_client,
					BMA2X2_MODE_SUSPEND, BMA_ENABLED_INPUT);

#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
			cancel_delayed_work_sync(&g_accel_data->work);
#endif
			atomic_set(&g_accel_data->enable, 0);

			// For CTS
			bma2x2_read_accel_xyz(g_accel_data->bma2x2_client, g_accel_data->sensor_type, &acc);
			if(g_accel_data->is_cali == 1) {
				acc.x += g_accel_data->cali.x;
				acc.y += g_accel_data->cali.y;
				acc.z += g_accel_data->cali.z;
			}
			input_report_abs(g_accel_data->input, ABS_X, acc.x);
			input_report_abs(g_accel_data->input, ABS_Y, acc.y);
			input_report_abs(g_accel_data->input, ABS_Z, acc.z);
			input_sync(g_accel_data->input);

		}
	}
	mutex_unlock(&g_accel_data->enable_mutex);

}

static ssize_t bma2x2_enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	/* If Probe Read Fail When Sensor Enable Read again */
	if(0 == g_accel_data->is_cali){
		g_accel_data->cali_retry = 0;
		queue_delayed_work(g_accel_data->cali_wq, &g_accel_data->cali_work, 0);
	}

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	APS_LOG("Set Sensor [%ld], is_cali [%d] from AP\n", data, g_accel_data->is_cali);
	
	bma2x2_set_range(g_accel_data->bma2x2_client, BMA2X2_RANGE_SET);
		
	if ((data == 0) || (data == 1))
		bma2x2_set_enable(dev, data);
	
	return count;
}
static ssize_t bma2x2_fast_calibration_x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

#ifdef CONFIG_SENSORS_BMI058
	if (bma2x2_get_offset_target(g_accel_data->bma2x2_client,
				BMI058_OFFSET_TRIGGER_X, &data) < 0)
		return -EINVAL;
#else
	if (bma2x2_get_offset_target(g_accel_data->bma2x2_client,
				BMA2X2_OFFSET_TRIGGER_X, &data) < 0)
		return -EINVAL;
#endif

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_fast_calibration_x_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

#ifdef CONFIG_SENSORS_BMI058
	if (bma2x2_set_offset_target(g_accel_data->bma2x2_client,
			BMI058_OFFSET_TRIGGER_X, (unsigned char)data) < 0)
		return -EINVAL;
#else
	if (bma2x2_set_offset_target(g_accel_data->bma2x2_client,
			BMA2X2_OFFSET_TRIGGER_X, (unsigned char)data) < 0)
		return -EINVAL;
#endif

	if (bma2x2_set_cal_trigger(g_accel_data->bma2x2_client, 1) < 0)
		return -EINVAL;

	do {
		WAIT_CAL_READY();
		bma2x2_get_cal_ready(g_accel_data->bma2x2_client, &tmp);

		/*PINFO("wait 2ms cal ready flag is %d\n", tmp); */
		timeout++;
		if (timeout == 50) {
			APS_LOG("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	APS_LOG("x axis fast calibration finished\n");
	return count;
}

static ssize_t bma2x2_fast_calibration_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

#ifdef CONFIG_SENSORS_BMI058
	if (bma2x2_get_offset_target(g_accel_data->bma2x2_client,
					BMI058_OFFSET_TRIGGER_Y, &data) < 0)
		return -EINVAL;
#else
	if (bma2x2_get_offset_target(g_accel_data->bma2x2_client,
					BMA2X2_OFFSET_TRIGGER_Y, &data) < 0)
		return -EINVAL;
#endif

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_fast_calibration_y_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

#ifdef CONFIG_SENSORS_BMI058
	if (bma2x2_set_offset_target(g_accel_data->bma2x2_client,
			BMI058_OFFSET_TRIGGER_Y, (unsigned char)data) < 0)
		return -EINVAL;
#else
	if (bma2x2_set_offset_target(g_accel_data->bma2x2_client,
			BMA2X2_OFFSET_TRIGGER_Y, (unsigned char)data) < 0)
		return -EINVAL;
#endif

	if (bma2x2_set_cal_trigger(g_accel_data->bma2x2_client, 2) < 0)
		return -EINVAL;

	do {
		WAIT_CAL_READY();
		bma2x2_get_cal_ready(g_accel_data->bma2x2_client, &tmp);

		/*PINFO("wait 2ms cal ready flag is %d\n", tmp);*/
		timeout++;
		if (timeout == 50) {
			APS_LOG("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	APS_LOG("y axis fast calibration finished\n");
	return count;
}

static ssize_t bma2x2_fast_calibration_z_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_offset_target(g_accel_data->bma2x2_client, 3, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_fast_calibration_z_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	signed char tmp;
	unsigned char timeout = 0;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_offset_target(g_accel_data->bma2x2_client, 3, (unsigned
					char)data) < 0)
		return -EINVAL;

	if (bma2x2_set_cal_trigger(g_accel_data->bma2x2_client, 3) < 0)
		return -EINVAL;

	do {
		WAIT_CAL_READY();
		bma2x2_get_cal_ready(g_accel_data->bma2x2_client, &tmp);

/*APS_LOG("wait 2ms cal ready flag is %d\n", tmp);*/
		timeout++;
		if (timeout == 50) {
			APS_LOG("get fast calibration ready error\n");
			return -EINVAL;
		};

	} while (tmp == 0);

	APS_LOG("z axis fast calibration finished\n");
	return count;
}


static ssize_t bma2x2_SleepDur_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_sleep_duration(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_SleepDur_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_sleep_duration(g_accel_data->bma2x2_client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_fifo_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_fifo_mode(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_fifo_mode_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_fifo_mode(g_accel_data->bma2x2_client,
				(unsigned char) data) < 0)
		return -EINVAL;
	return count;
}



static ssize_t bma2x2_fifo_trig_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_fifo_trig(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_fifo_trig_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_fifo_trig(g_accel_data->bma2x2_client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}



static ssize_t bma2x2_fifo_trig_src_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_fifo_trig_src(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_fifo_trig_src_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (bma2x2_set_fifo_trig_src(g_accel_data->bma2x2_client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}


/*!
 * @brief show fifo_data_sel axis definition(Android definition, not sensor HW reg).
 * 0--> x, y, z axis fifo data for every frame
 * 1--> only x axis fifo data for every frame
 * 2--> only y axis fifo data for every frame
 * 3--> only z axis fifo data for every frame
 */
static ssize_t bma2x2_fifo_data_sel_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
#ifdef CONFIG_Z380M
	signed char place = Z380M_BMA2X2_DEFAULT_PLACE;
#endif

#ifdef CONFIG_Z300M
	signed char place = Z300M_BMA2X2_DEFAULT_PLACE;
#endif
	if (bma2x2_get_fifo_data_sel(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

#ifdef CONFIG_SENSORS_BMI058
/*Update BMI058 fifo_data_sel to the BMA2x2 common definition*/
	if (BMI058_FIFO_DAT_SEL_X == data)
		data = BMA2X2_FIFO_DAT_SEL_X;
	else if (BMI058_FIFO_DAT_SEL_Y == data)
		data = BMA2X2_FIFO_DAT_SEL_Y;
#endif

	/*remaping fifo_dat_sel if define virtual place in BSP files*/
	if ((NULL != g_accel_data->bst_pd) &&
		(BOSCH_SENSOR_PLACE_UNKNOWN != g_accel_data->bst_pd->place)) {
		place = g_accel_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place > 0) && (place < MAX_AXIS_REMAP_TAB_SZ)) {
			/* BMA2X2_FIFO_DAT_SEL_X: 1, Y:2, Z:3;
			* but bst_axis_remap_tab_dft[i].src_x:0, y:1, z:2
			* so we need to +1*/
			if (BMA2X2_FIFO_DAT_SEL_X == data)
				data = bst_axis_remap_tab_dft[place].src_x + 1;
			else if (BMA2X2_FIFO_DAT_SEL_Y == data)
				data = bst_axis_remap_tab_dft[place].src_y + 1;
		}
	}

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_fifo_framecount_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	unsigned char mode;
	if (bma2x2_get_fifo_framecount(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	if (data > MAX_FIFO_F_LEVEL) {
		APS_LOG("bma2x2 fifo_count abnormal, %d\n", data);

		if (bma2x2_get_mode(g_accel_data->bma2x2_client, &mode) < 0)
			return -EINVAL;

		if (BMA2X2_MODE_NORMAL == mode) {
			PERR("bma2x2 fifo_count: %d abnormal, op_mode: %d", data, mode);
			data = MAX_FIFO_F_LEVEL;
		} else {
			/*chip already suspend or shutdown*/
			data = 0;
		}
	}

	return sprintf(buf, "%d\n", data);
}

static ssize_t bma2x2_fifo_framecount_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	g_accel_data->fifo_count = (unsigned int) data;

	return count;
}

static ssize_t bma2x2_temperature_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	if (bma2x2_read_temperature(g_accel_data->bma2x2_client, &data) < 0)
		return -EINVAL;

	return sprintf(buf, "%d\n", data);

}

/*!
 * @brief store fifo_data_sel axis definition(Android definition, not sensor HW reg).
 * 0--> x, y, z axis fifo data for every frame
 * 1--> only x axis fifo data for every frame
 * 2--> only y axis fifo data for every frame
 * 3--> only z axis fifo data for every frame
 */
static ssize_t bma2x2_fifo_data_sel_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	signed char place;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	/*save fifo_data_sel(android definition)*/
	g_accel_data->fifo_datasel = (unsigned char) data;

	/*remaping fifo_dat_sel if define virtual place*/
	if ((NULL != g_accel_data->bst_pd) &&
		(BOSCH_SENSOR_PLACE_UNKNOWN != g_accel_data->bst_pd->place)) {
		place = g_accel_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place > 0) && (place < MAX_AXIS_REMAP_TAB_SZ)) {
			/*Need X Y axis revesal sensor place: P1, P3, P5, P7 */
			/* BMA2X2_FIFO_DAT_SEL_X: 1, Y:2, Z:3;
			  * but bst_axis_remap_tab_dft[i].src_x:0, y:1, z:2
			  * so we need to +1*/
			if (BMA2X2_FIFO_DAT_SEL_X == data)
				data =  bst_axis_remap_tab_dft[place].src_x + 1;
			else if (BMA2X2_FIFO_DAT_SEL_Y == data)
				data =  bst_axis_remap_tab_dft[place].src_y + 1;
		}
	}
#ifdef CONFIG_SENSORS_BMI058
	/*Update BMI058 fifo_data_sel to the BMA2x2 common definition*/
		if (BMA2X2_FIFO_DAT_SEL_X == data)
			data = BMI058_FIFO_DAT_SEL_X;
		else if (BMA2X2_FIFO_DAT_SEL_Y == data)
			data = BMI058_FIFO_DAT_SEL_Y;

#endif
	if (bma2x2_set_fifo_data_sel(g_accel_data->bma2x2_client,
				(unsigned char) data) < 0)
		return -EINVAL;

	return count;
}


/*!
 * brief: bma2x2 single axis data remaping
 * @param[i] fifo_datasel   fifo axis data select setting
 * @param[i/o] remap_dir   remapping direction
 * @param[i] client_data   to transfer sensor place
 *
 * @return none
 */
static void bma2x2_single_axis_remaping(unsigned char fifo_datasel,
		unsigned char *remap_dir, struct bma2x2_data *client_data)
{
	if ((NULL == client_data->bst_pd) ||
			(BOSCH_SENSOR_PLACE_UNKNOWN
			 == client_data->bst_pd->place))
		return;
	else {
		signed char place = client_data->bst_pd->place;
		/* sensor with place 0 needs not to be remapped */
		if ((place <= 0)  || (place >= MAX_AXIS_REMAP_TAB_SZ))
			return;

		if (fifo_datasel < 1 || fifo_datasel > 3)
			return;
		else {
			switch (fifo_datasel) {
			/*P2, P3, P4, P5 X axis(andorid) need to reverse*/
			case BMA2X2_FIFO_DAT_SEL_X:
				if (-1 == bst_axis_remap_tab_dft[place].sign_x)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			/*P1, P2, P5, P6 Y axis(andorid) need to reverse*/
			case BMA2X2_FIFO_DAT_SEL_Y:
				if (-1 == bst_axis_remap_tab_dft[place].sign_y)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			case BMA2X2_FIFO_DAT_SEL_Z:
			/*P4, P5, P6, P7 Z axis(andorid) need to reverse*/
				if (-1 == bst_axis_remap_tab_dft[place].sign_z)
					*remap_dir = 1;
				else
					*remap_dir = 0;
				break;
			default:
				break;
			}
		}
	}

	return;
}

static ssize_t bma2x2_fifo_data_out_frame_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err, i, len;
	signed char fifo_data_out[MAX_FIFO_F_LEVEL * MAX_FIFO_F_BYTES] = {0};
	unsigned char f_len = 0;
	s16 value;
	struct bma2x2acc acc_lsb;
	unsigned char axis_dir_remap = 0;
	if (g_accel_data->fifo_datasel) {
		/*Select one axis data output for every fifo frame*/
		f_len = 2;
	} else	{
		/*Select X Y Z axis data output for every fifo frame*/
		f_len = 6;
	}

	if (g_accel_data->fifo_count == 0)
		return -EINVAL;

	if (g_accel_data->bma_mode_enabled == 0) {
		APS_LOG("bma2x2_fifo_data, bma_mode_enabled 0\n");
		/*return -EINVAL;*/
	}
	if (bma_i2c_burst_read(g_accel_data->bma2x2_client,
			BMA2X2_FIFO_DATA_OUTPUT_REG, fifo_data_out,
						g_accel_data->fifo_count * f_len) < 0)
		return -EINVAL;

	err = 0;

/* please give attation for the fifo output data format*/
	if (f_len == 6) {
		/* Select X Y Z axis data output for every frame */
		for (i = 0; i < g_accel_data->fifo_count; i++) {
			acc_lsb.x =
			((unsigned char)fifo_data_out[i * f_len + 1] << 8 |
				(unsigned char)fifo_data_out[i * f_len + 0]);
			acc_lsb.y =
			((unsigned char)fifo_data_out[i * f_len + 3] << 8 |
				(unsigned char)fifo_data_out[i * f_len + 2]);
			acc_lsb.z =
			((unsigned char)fifo_data_out[i * f_len + 5] << 8 |
				(unsigned char)fifo_data_out[i * f_len + 4]);
#ifndef BMA2X2_SENSOR_IDENTIFICATION_ENABLE
			acc_lsb.x >>=
			(16 - bma2x2_sensor_bitwidth[g_accel_data->sensor_type]);
			acc_lsb.y >>=
			(16 - bma2x2_sensor_bitwidth[g_accel_data->sensor_type]);
			acc_lsb.z >>=
			(16 - bma2x2_sensor_bitwidth[g_accel_data->sensor_type]);
#endif
			bma2x2_remap_sensor_data(&acc_lsb, g_accel_data);
			
			/* Add by Tom for Calibration */
			if(g_accel_data->is_cali == 1) {
				acc_lsb.x += g_accel_data->cali.x;
				acc_lsb.y += g_accel_data->cali.y;
				acc_lsb.z += g_accel_data->cali.z;
			}
			len = sprintf(buf, "%d %d %d ",
				acc_lsb.x, acc_lsb.y, acc_lsb.z);
			buf += len;
			err += len;
		}
	} else {
		/* single axis data output for every frame */
		bma2x2_single_axis_remaping(g_accel_data->fifo_datasel,
					&axis_dir_remap, g_accel_data);
		for (i = 0; i < g_accel_data->fifo_count * f_len / 2; i++)	{
			value = ((unsigned char)fifo_data_out[2 * i + 1] << 8 |
					(unsigned char)fifo_data_out[2 * i]);
#ifndef BMA2X2_SENSOR_IDENTIFICATION_ENABLE
			value >>=
			(16 - bma2x2_sensor_bitwidth[g_accel_data->sensor_type]);
#endif
			if (axis_dir_remap)
				value = 0 - value;
			len = sprintf(buf, "%d ", value);
			buf += len;
			err += len;
		}
	}

	return err;
}

static ssize_t bma2x2_offset_x_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	if (bma2x2_get_offset_x(g_accel_data->bma2x2_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_offset_x_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_offset_x(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_offset_y_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;

	if (bma2x2_get_offset_y(g_accel_data->bma2x2_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_offset_y_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_offset_y(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_offset_z_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	if (bma2x2_get_offset_z(g_accel_data->bma2x2_client, &data) < 0)
		return sprintf(buf, "Read error\n");

	return sprintf(buf, "%d\n", data);

}

static ssize_t bma2x2_offset_z_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if (bma2x2_set_offset_z(g_accel_data->bma2x2_client, (unsigned
					char)data) < 0)
		return -EINVAL;

	return count;
}

static ssize_t bma2x2_camera_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bma2x2acc acc_value;
	int is_enable = 0;
	is_enable = atomic_read(&g_accel_data->enable);	
	
	if(is_enable == 0) {
		bma2x2_set_enable(dev,1);
		mdelay(10);
	}
	bma2x2_read_accel_xyz(g_accel_data->bma2x2_client, g_accel_data->sensor_type,
								&acc_value);
	
	if(is_enable == 0) bma2x2_set_enable(dev,0);
	
	if(g_accel_data->is_cali == 1) {
		acc_value.x += g_accel_data->cali.x;
		acc_value.y += g_accel_data->cali.y;
		acc_value.z += g_accel_data->cali.z;
	}

	return snprintf(buf, PAGE_SIZE, "%d.%02d %d.%02d %d.%02d\n", 
				(acc_value.x * 191 / 10000) ,(((acc_value.x < 0) ? (acc_value.x * -1) : acc_value.x) * 191 % 10000), 
				(acc_value.y * 191 / 10000) ,(((acc_value.y < 0) ? (acc_value.y * -1) : acc_value.y) * 191 % 10000), 
				(acc_value.z * 191 / 10000) ,(((acc_value.z < 0) ? (acc_value.z * -1) : acc_value.z) * 191 % 10000));
}

#ifdef CONFIG_SIG_MOTION
static int bma2x2_set_en_slope_int(struct bma2x2_data *bma2x2,
		int en)
{
	int err;
	struct i2c_client *client = bma2x2->bma2x2_client;

	if (en) {
		/* Set the related parameters which needs to be fine tuned by
		* interfaces: slope_threshold and slope_duration
		*/
		/*dur: 192 samples ~= 3s*/
		err = bma2x2_set_slope_duration(client, 0x0);
		err += bma2x2_set_slope_threshold(client, 0x16);

		/*Enable the interrupts*/
		err += bma2x2_set_Int_Enable(client, 5, 1);/*Slope X*/
		err += bma2x2_set_Int_Enable(client, 6, 1);/*Slope Y*/
		err += bma2x2_set_Int_Enable(client, 7, 1);/*Slope Z*/
	#ifdef BMA2X2_ENABLE_INT1
		/* TODO: SLOPE can now only be routed to INT1 pin*/
		err += bma2x2_set_int1_pad_sel(client, PAD_SLOP);
	#else
		/* err += bma2x2_set_int2_pad_sel(client, PAD_SLOP); */
	#endif
	} else {
		err = bma2x2_set_Int_Enable(client, 5, 0);/*Slope X*/
		err += bma2x2_set_Int_Enable(client, 6, 0);/*Slope Y*/
		err += bma2x2_set_Int_Enable(client, 7, 0);/*Slope Z*/
	}
	return err;
}

static ssize_t bma2x2_en_sig_motion_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d\n", atomic_read(&g_accel_data->en_sig_motion));
}

static int bma2x2_set_en_sig_motion(struct bma2x2_data *bma2x2,
		int en)
{
	int err = 0;

	en = (en >= 1) ? 1 : 0;  /* set sig motion sensor status */

	if (atomic_read(&g_accel_data->en_sig_motion) != en) {
		if (en) {
			err = bma2x2_set_mode(g_accel_data->bma2x2_client,
					BMA2X2_MODE_NORMAL, BMA_ENABLED_SGM);
			err = bma2x2_set_en_slope_int(g_accel_data, en);
			enable_irq_wake(gsensor_irq);
		} else {
			disable_irq_wake(gsensor_irq);
			err = bma2x2_set_en_slope_int(g_accel_data, en);
			err = bma2x2_set_mode(g_accel_data->bma2x2_client,
					BMA2X2_MODE_SUSPEND, BMA_ENABLED_SGM);
		}
		atomic_set(&g_accel_data->en_sig_motion, en);
	}
	return err;
}

static ssize_t bma2x2_en_sig_motion_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	if ((data == 0) || (data == 1))
		bma2x2_set_en_sig_motion(g_accel_data, data);

	return count;
}
#endif

#ifdef CONFIG_DOUBLE_TAP
static int bma2x2_set_en_single_tap_int(struct bma2x2_data *bma2x2, int en)
{
	int err;
	struct i2c_client *client = g_accel_data->bma2x2_client;

	if (en) {
		/* set tap interruption parameter here if needed.
		bma2x2_set_tap_duration(client, 0x01);
		bma2x2_set_tap_threshold(client, 0x02);
		*/

		/*Enable the single tap interrupts*/
		err = bma2x2_set_Int_Enable(client, 8, 1);
	#ifdef BMA2X2_ENABLE_INT1
		err += bma2x2_set_int1_pad_sel(client, PAD_SINGLE_TAP);
	#else
		err += bma2x2_set_int2_pad_sel(client, PAD_SINGLE_TAP);
	#endif
	} else {
		err = bma2x2_set_Int_Enable(client, 8, 0);
	}
	return err;
}

static ssize_t bma2x2_tap_time_period_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", g_accel_data->tap_time_period);
}

static ssize_t bma2x2_tap_time_period_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	g_accel_data->tap_time_period = data;

	return count;
}

static ssize_t bma2x2_en_double_tap_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&g_accel_data->en_double_tap));
}

static int bma2x2_set_en_double_tap(struct bma2x2_data *bma2x2,
		int en)
{
	int err = 0;
	unsigned char data = 0;
	en = (en >= 1) ? 1 : 0;

	mutex_lock(&g_accel_data->enable_mutex);
	if (atomic_read(&g_accel_data->en_double_tap) != en) {
		if (en) {
			// Change Bandwidth to 125 Hz
			data = BMA2X2_SET_BITSLICE(data, BMA2X2_BANDWIDTH, BMA2X2_BW_125HZ);
			err = bma2x2_smbus_write_byte(g_accel_data->bma2x2_client, BMA2X2_BANDWIDTH__REG, &data);
			
			err = bma2x2_set_mode(g_accel_data->bma2x2_client,
					BMA2X2_MODE_NORMAL, BMA_ENABLED_DTAP);
			err = bma2x2_set_en_single_tap_int(g_accel_data, en);
			bma2x2->en_sensor_type = 4;
//			queue_delayed_work(g_accel_data->CTS_wq, &g_accel_data->CTS_dw, 0);
		} else {
			err = bma2x2_set_en_single_tap_int(g_accel_data, en);
			err = bma2x2_set_mode(g_accel_data->bma2x2_client,
					BMA2X2_MODE_SUSPEND, BMA_ENABLED_DTAP);
		}
		atomic_set(&g_accel_data->en_double_tap, en);
	}
	mutex_unlock(&g_accel_data->enable_mutex);
	return err;
}

static ssize_t bma2x2_en_double_tap_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	APS_LOG("Set Double Tap Status [%ld] from AP\n", data);
	if ((data == 0) || (data == 1))
		bma2x2_set_en_double_tap(g_accel_data, data);

	return count;
}

static void bma2x2_tap_timeout_handle(unsigned long data)
{
	unsigned long flags;
	
	APS_LOG("tap interrupt handle, timeout\n");
        spin_lock_irqsave(&g_accel_data->tap_lock, flags);
	g_accel_data->tap_times = 0;
	spin_unlock_irqrestore(&g_accel_data->tap_lock, flags);

	/* if a single tap need to report, open the define */
#ifdef REPORT_SINGLE_TAP_WHEN_DOUBLE_TAP_SENSOR_ENABLED
	input_report_rel(g_accel_data->dev_interrupt,
		SINGLE_TAP_INTERRUPT,
		SINGLE_TAP_INTERRUPT_HAPPENED);
	input_sync(g_accel_data->dev_interrupt);
#endif

}
#endif

#ifdef CONFIG_FLICK
static ssize_t bma2x2_flick_time_period_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", g_accel_data->flick_time_period);
}

static ssize_t bma2x2_flick_time_period_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;

	g_accel_data->flick_time_period = data;

	return count;
}

static void bma2x2_flick_timeout_handle(unsigned long data)
{
	unsigned long flags;
		APS_LOG("flick report [%d] ... \n", g_accel_data->flick_times);
	if(g_accel_data->flick_times == 1) {
		/* Report Single event*/
		if(atomic_read(&g_accel_data->en_flick) == 1) {
			input_report_abs(g_accel_data->dev_flick, MOTION_FLICK_INTERRUPT, SINGLE_FLICK);
			input_report_abs(g_accel_data->dev_flick, MOTION_FLICK_INTERRUPT, UNKNOWN_FLICK);
			input_sync(g_accel_data->dev_flick);
		}

#ifdef CONFIG_FLICK_HV
		if(atomic_read(&g_accel_data->en_flick_hv) == 1) {
			input_report_abs(g_accel_data->dev_flick_hv, MOTION_FLICK_INTERRUPT, SINGLE_FLICK);
			input_report_abs(g_accel_data->dev_flick_hv, MOTION_FLICK_INTERRUPT, UNKNOWN_FLICK);
			input_sync(g_accel_data->dev_flick_hv);
		}
#endif
	} else {
		/* Report Double event */
		if(atomic_read(&g_accel_data->en_flick) == 1) {
			input_report_abs(g_accel_data->dev_flick, MOTION_FLICK_INTERRUPT, DOUBLE_FLICK);
			input_report_abs(g_accel_data->dev_flick, MOTION_FLICK_INTERRUPT, UNKNOWN_FLICK);
			input_sync(g_accel_data->dev_flick);
		}
#ifdef CONFIG_FLICK_HV
		if(atomic_read(&g_accel_data->en_flick_hv) == 1) {
			input_report_abs(g_accel_data->dev_flick_hv, MOTION_FLICK_INTERRUPT, DOUBLE_FLICK);
			input_report_abs(g_accel_data->dev_flick_hv, MOTION_FLICK_INTERRUPT, UNKNOWN_FLICK);
			input_sync(g_accel_data->dev_flick_hv);
		}
#endif
	}
        spin_lock_irqsave(&g_accel_data->flick_lock, flags);
	g_accel_data->flick_times = 0;
	spin_unlock_irqrestore(&g_accel_data->flick_lock, flags);
}
static ssize_t bma2x2_enable_flick_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	APS_LOG("Report NO_FLICK to HAL\n");
	input_report_abs(g_accel_data->dev_flick, MOTION_FLICK_INTERRUPT, NO_FLICK);
	input_sync(g_accel_data->dev_flick);

	return sprintf(buf, "%d\n", atomic_read(&g_accel_data->en_flick));
}
static ssize_t bma2x2_enable_flick_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int err = 0;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;
	
	// Set Flick HV Status
	if (data == 1)
		atomic_set(&g_accel_data->en_flick, 1);
	else if (data == 0)
		atomic_set(&g_accel_data->en_flick, 0);

	mutex_lock(&g_accel_data->enable_mutex);
	if (data == 1 && atomic_read(&g_accel_data->en_flick_hv) == 0) {
		/* Change Bandwidth to 31_25 Hz */
		bma2x2_set_bandwidth(g_accel_data->bma2x2_client, BMA2X2_BW_SET);

		// Only enable Slope X & Y for keep high sensitivity
		err += bma2x2_set_mode(g_accel_data->bma2x2_client,
				BMA2X2_MODE_NORMAL, BMA_ENABLED_FLICK);			// Enable
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 1, 1);		// Slope X
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 2, 1);		// Slope Y

		if(err == 0) {
			// atomic_set(&g_accel_data->en_flick, 1);
			APS_LOG("Enable Flick function from AP\n");
			g_accel_data->en_sensor_type = 1;
//			queue_delayed_work(g_accel_data->CTS_wq, &g_accel_data->CTS_dw, 0);
			// input_report_abs(g_accel_data->dev_flick, MOTION_FLICK_INTERRUPT, NO_FLICK);
			// input_sync(g_accel_data->dev_flick);
		}
		else{
			atomic_set(&g_accel_data->en_flick, 0);
			APS_LOG("bma2x2_enable_flick_store enable fail!!\n");
		}
	} else if (data == 0 && atomic_read(&g_accel_data->en_flick_hv) == 0) {
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 1, 0);		//Slope X
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 2, 0);		//Slope Y
		err += bma2x2_set_mode(g_accel_data->bma2x2_client,
				BMA2X2_MODE_SUSPEND, BMA_ENABLED_FLICK);		// Disable
		if(err == 0) {
			APS_LOG("Disable Flick function from AP\n");
			input_report_abs(g_accel_data->dev_flick, MOTION_FLICK_INTERRUPT, NO_FLICK);
			input_sync(g_accel_data->dev_flick);
		} else {
			APS_LOG("bma2x2_enable_flick_store disable fail!!\n");
		}
		// atomic_set(&g_accel_data->en_flick, 0);
	}else{
		APS_LOG( "bma2x2_enable_flick_store error input !!\n");
	}
	mutex_unlock(&g_accel_data->enable_mutex);

	return count;
}
#endif

#ifdef CONFIG_FLICK_HV
static ssize_t bma2x2_enable_flick_hv_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	APS_LOG("Report NO_FLICK to HAL\n");
	input_report_abs(g_accel_data->dev_flick_hv, MOTION_FLICK_INTERRUPT, NO_FLICK);
	input_sync(g_accel_data->dev_flick_hv);

	return sprintf(buf, "%d\n", atomic_read(&g_accel_data->en_flick_hv));
}
static ssize_t bma2x2_enable_flick_hv_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int err = 0;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	mutex_lock(&g_accel_data->enable_mutex);
	// Set Flick HV Status
	if (data == 1)
		atomic_set(&g_accel_data->en_flick_hv, 1);
	else if (data == 0)
		atomic_set(&g_accel_data->en_flick_hv, 0);

	if (data == 1 && atomic_read(&g_accel_data->en_flick) == 0) {
		/* Change Bandwidth to 31_25 Hz */
		bma2x2_set_bandwidth(g_accel_data->bma2x2_client, BMA2X2_BW_SET);

		// Only enable Slope X & Y for keep high sensitivity
		err += bma2x2_set_mode(g_accel_data->bma2x2_client,
				BMA2X2_MODE_NORMAL, BMA_ENABLED_FLICK);			// Enable
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 1, 1);		// Slope X
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 2, 1);		// Slope Y

		if(err == 0) {
			// atomic_set(&g_accel_data->en_flick_hv, 1);
			APS_LOG("Enable Flick HV function from AP\n");
			g_accel_data->en_sensor_type = 2;
//			queue_delayed_work(g_accel_data->CTS_wq, &g_accel_data->CTS_dw, 0);
			// input_report_abs(g_accel_data->dev_flick_hv, MOTION_FLICK_INTERRUPT, NO_FLICK);
			// input_sync(g_accel_data->dev_flick_hv);
		}
		else{
			atomic_set(&g_accel_data->en_flick_hv, 0);
			APS_LOG("bma2x2_enable_flick_store enable fail!!\n");
		}
	} else if (data == 0 && atomic_read(&g_accel_data->en_flick) == 0) {
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 1, 0);		//Slope X
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 2, 0);		//Slope Y
		err += bma2x2_set_mode(g_accel_data->bma2x2_client,
				BMA2X2_MODE_SUSPEND, BMA_ENABLED_FLICK);		// Disable
		if(err == 0) {
			APS_LOG("Disable Flick function from AP\n");
			input_report_abs(g_accel_data->dev_flick_hv, MOTION_FLICK_INTERRUPT, NO_FLICK);
			input_sync(g_accel_data->dev_flick_hv);
		} else {
			APS_LOG("bma2x2_enable_flick_store disable fail!!\n");
		}
		// atomic_set(&g_accel_data->en_flick_hv, 0);
	}else{
		APS_LOG( "bma2x2_enable_flick_store error input !!\n");
	}
	mutex_unlock(&g_accel_data->enable_mutex);
		
	return count;
}
#endif

static ssize_t bma2x2_gsensor_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	int ret = -1 , result = -1;
//	ret = bma2x2_selftest(g_accel_data);
//	result = atomic_read(&g_accel_data->selftest_result);
//	APS_LOG("result [%d] , ret [%d]\n", result, ret);
//	if((ret == 0) && (result == 0))	
//		return snprintf(buf, PAGE_SIZE, "1\n");
//	else
//		return snprintf(buf, PAGE_SIZE, "0\n");
	return sprintf(buf, "%d\n", g_accel_data->state);
}

static ssize_t bma2x2_asus_calibrarion_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int Cali_Result = 1;
	bma2x2_read_from_califile();
	APS_LOG("calibration[%d] value X[%d],Y[%d],Z[%d] !!\n", g_accel_data->is_cali ,g_accel_data->cali.x,g_accel_data->cali.y,g_accel_data->cali.z);
	return sprintf(buf, "%d\n",Cali_Result);
}
static ssize_t bma2x2_asus_calibrarion_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct bma2x2acc acc_value;
	int tmp_x=0,tmp_y=0,tmp_z=0,tmp_count=0;
	unsigned long data;
	int iloop=0,error=0, Cali_Result=0;

	g_accel_data->is_cali = 0;
	error = kstrtoul(buf, 10, &data);
	if (error) {
		APS_LOG("input error (%d) !!\n",error);		
		return error;
	}
	switch(data) {
		case 1 :
			if(DEBUG) APS_LOG("Calibration 1 axis\n");
			for(iloop=0 ; iloop<8 ; iloop++) {
				if(iloop>1) {	
					bma2x2_read_accel_xyz(g_accel_data->bma2x2_client, g_accel_data->sensor_type, &acc_value);			
					tmp_x = tmp_x + acc_value.x ;
					tmp_y = tmp_y + acc_value.y ;
					tmp_z = tmp_z + acc_value.z ;
					tmp_count++;
					if(DEBUG) APS_LOG( "[%d] times : X[%d],Y[%d],Z[%d] !!\n", iloop, acc_value.x, acc_value.y, acc_value.z);
				}
				mdelay(50);
			}
			g_accel_data->cali.x = (0-tmp_x/tmp_count) ;
			g_accel_data->cali.y = (0-tmp_y/tmp_count) ;
			if(BMA2X2_RANGE_SET == BMA2X2_RANGE_8G)
				g_accel_data->cali.z = (256 - tmp_z/tmp_count);
			else{
				g_accel_data->cali.z = (512 - tmp_z/tmp_count);
			}
			if(DEBUG) APS_LOG( "Calibration value X[%d],Y[%d],Z[%d] !!\n", g_accel_data->cali.x,g_accel_data->cali.y,g_accel_data->cali.z);
			bma2x2_write_to_califile(g_accel_data->cali.x,g_accel_data->cali.y,g_accel_data->cali.z);
			mdelay(10);
			g_accel_data->is_cali = 1;
			Cali_Result = 1;
			break;

		case 0 :
			if(DEBUG) APS_LOG("disable calibration \n");
			g_accel_data->cali.x = 0;
			g_accel_data->cali.y = 0;
			g_accel_data->cali.z = 0;
			bma2x2_write_to_califile(g_accel_data->cali.x,g_accel_data->cali.y,g_accel_data->cali.z);
			g_accel_data->is_cali = 0;
			Cali_Result = 1;

		default :
			Cali_Result = 0;
			break;
	}
	printk("%d\n", Cali_Result);
	return count ;
}

/* Add by Tom for Zen Motion*/
static ssize_t bma2x2_enable_terminal_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	input_report_abs(g_accel_data->dev_terminal, MOTION_TERMINAL_INTERRUPT, -1);
	input_sync(g_accel_data->dev_terminal);
	APS_LOG( "bma2x2_enable_terminal_show !!\n");

	return sprintf(buf, "%d\n", atomic_read(&g_accel_data->en_terminal));
}
static ssize_t bma2x2_enable_terminal_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data;
	int err = 0;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	mutex_lock(&g_accel_data->enable_mutex);
	if (data == 1) {
		/* Change Bandwidth to 31_25 Hz */
		bma2x2_set_bandwidth(g_accel_data->bma2x2_client, BMA2X2_BW_SET);
		err += bma2x2_set_mode(g_accel_data->bma2x2_client,
				BMA2X2_MODE_NORMAL, BMA_ENABLED_TERMINAL);  // Enable
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 10, 1); // Orientation 
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 11, 1); // Flat
		
		if(err == 0) {
			atomic_set(&g_accel_data->en_terminal, 1);
			APS_LOG("Enable Terminal function from AP\n");
		} else {
			atomic_set(&g_accel_data->en_terminal, 0);
			APS_LOG( "bma2x2_enable_terminal_store enable fail!!\n");
		}

		g_accel_data->en_sensor_type = 3;
//		queue_delayed_work(g_accel_data->CTS_wq, &g_accel_data->CTS_dw, 0);
		// input_report_abs(g_accel_data->dev_terminal, MOTION_TERMINAL_INTERRUPT, -1);
		// input_sync(g_accel_data->dev_terminal);

	} else if(data == 0) {
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 10, 0); // Orientation
		err += bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 11, 0); // Flat
		err += bma2x2_set_mode(g_accel_data->bma2x2_client,
				BMA2X2_MODE_SUSPEND, BMA_ENABLED_TERMINAL); // Disable

		if(err == 0) {
			APS_LOG("Disable Terminal function from AP\n");
		} else {
			APS_LOG( "bma2x2_enable_terminal_store disable fail!!\n");
		}

		atomic_set(&g_accel_data->en_terminal, 0);
		input_report_abs(g_accel_data->dev_terminal, MOTION_TERMINAL_INTERRUPT, -1);
		input_sync(g_accel_data->dev_terminal);
	} else {
		APS_LOG( "bma2x2_enable_terminal_store error input !!\n");
	}
	/* Add by Tom for Skip Same Terminal Event */
	g_accel_data->pre_ter_event = TERMINAL_UNKNOWN;
	mutex_unlock(&g_accel_data->enable_mutex);
	return count;
}

static ssize_t bma2x2_chip_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct i2c_client *client = to_i2c_client(dev);
//	struct bma2x2_data *bma2x2 = i2c_get_clientdata(client);

	return sprintf(buf, "%u\n", g_accel_data->chip_id);

}


static DEVICE_ATTR(range, 0660, bma2x2_range_show, bma2x2_range_store);
static DEVICE_ATTR(bandwidth, 0660, bma2x2_bandwidth_show, bma2x2_bandwidth_store);
static DEVICE_ATTR(op_mode, 0660, bma2x2_mode_show, bma2x2_mode_store);
static DEVICE_ATTR(value, 0660, bma2x2_value_show, NULL);
static DEVICE_ATTR(value_cache, 0660, bma2x2_value_cache_show, NULL);
static DEVICE_ATTR(delay, 0660, bma2x2_delay_show, bma2x2_delay_store);
static DEVICE_ATTR(enable, 0660, bma2x2_enable_show, bma2x2_enable_store);
static DEVICE_ATTR(SleepDur, 0660, bma2x2_SleepDur_show, bma2x2_SleepDur_store);
static DEVICE_ATTR(fast_calibration_x, 0660, bma2x2_fast_calibration_x_show, bma2x2_fast_calibration_x_store);
static DEVICE_ATTR(fast_calibration_y, 0660, bma2x2_fast_calibration_y_show, bma2x2_fast_calibration_y_store);
static DEVICE_ATTR(fast_calibration_z, 0660, bma2x2_fast_calibration_z_show, bma2x2_fast_calibration_z_store);
static DEVICE_ATTR(fifo_mode, 0660,  bma2x2_fifo_mode_show, bma2x2_fifo_mode_store);
static DEVICE_ATTR(fifo_framecount, 0660, bma2x2_fifo_framecount_show, bma2x2_fifo_framecount_store);
static DEVICE_ATTR(fifo_trig, 0660, bma2x2_fifo_trig_show, bma2x2_fifo_trig_store);
static DEVICE_ATTR(fifo_trig_src, 0660, bma2x2_fifo_trig_src_show, bma2x2_fifo_trig_src_store);
static DEVICE_ATTR(fifo_data_sel, 0660, bma2x2_fifo_data_sel_show, bma2x2_fifo_data_sel_store);
static DEVICE_ATTR(fifo_data_frame, 0660, bma2x2_fifo_data_out_frame_show, NULL);
static DEVICE_ATTR(reg, 0660, bma2x2_register_show, bma2x2_register_store);
static DEVICE_ATTR(chip_id, 0660, bma2x2_chip_id_show, NULL);
static DEVICE_ATTR(offset_x, 0660, bma2x2_offset_x_show, bma2x2_offset_x_store);
static DEVICE_ATTR(offset_y, 0660, bma2x2_offset_y_show, bma2x2_offset_y_store);
static DEVICE_ATTR(offset_z, 0660, bma2x2_offset_z_show, bma2x2_offset_z_store);
static DEVICE_ATTR(enable_int, 0660, 
		NULL, bma2x2_enable_int_store);
static DEVICE_ATTR(int_mode, 0660, 
		bma2x2_int_mode_show, bma2x2_int_mode_store);
static DEVICE_ATTR(slope_duration, 0660, 
		bma2x2_slope_duration_show, bma2x2_slope_duration_store);
static DEVICE_ATTR(slope_threshold, 0660, 
		bma2x2_slope_threshold_show, bma2x2_slope_threshold_store);
static DEVICE_ATTR(slope_no_mot_duration, 0660, 
		bma2x2_slope_no_mot_duration_show,
			bma2x2_slope_no_mot_duration_store);
static DEVICE_ATTR(slope_no_mot_threshold, 0660, 
		bma2x2_slope_no_mot_threshold_show,
			bma2x2_slope_no_mot_threshold_store);
static DEVICE_ATTR(high_g_duration, 0660, 
		bma2x2_high_g_duration_show, bma2x2_high_g_duration_store);
static DEVICE_ATTR(high_g_threshold, 0660, 
		bma2x2_high_g_threshold_show, bma2x2_high_g_threshold_store);
static DEVICE_ATTR(low_g_duration, 0660, 
		bma2x2_low_g_duration_show, bma2x2_low_g_duration_store);
static DEVICE_ATTR(low_g_threshold, 0660, 
		bma2x2_low_g_threshold_show, bma2x2_low_g_threshold_store);
static DEVICE_ATTR(tap_duration, 0660, 
		bma2x2_tap_duration_show, bma2x2_tap_duration_store);
static DEVICE_ATTR(tap_threshold, 0660, 
		bma2x2_tap_threshold_show, bma2x2_tap_threshold_store);
static DEVICE_ATTR(tap_quiet, 0660, 
		bma2x2_tap_quiet_show, bma2x2_tap_quiet_store);
static DEVICE_ATTR(tap_shock, 0660, 
		bma2x2_tap_shock_show, bma2x2_tap_shock_store);
static DEVICE_ATTR(tap_samp, 0660, 
		bma2x2_tap_samp_show, bma2x2_tap_samp_store);
static DEVICE_ATTR(orient_mode, 0660, 
		bma2x2_orient_mode_show, bma2x2_orient_mode_store);
static DEVICE_ATTR(orient_blocking, 0660, 
		bma2x2_orient_blocking_show, bma2x2_orient_blocking_store);
static DEVICE_ATTR(orient_hyst, 0660, 
		bma2x2_orient_hyst_show, bma2x2_orient_hyst_store);
static DEVICE_ATTR(orient_theta, 0660, 
		bma2x2_orient_theta_show, bma2x2_orient_theta_store);
static DEVICE_ATTR(flat_theta, 0660, 
		bma2x2_flat_theta_show, bma2x2_flat_theta_store);
static DEVICE_ATTR(flat_hold_time, 0660, 
		bma2x2_flat_hold_time_show, bma2x2_flat_hold_time_store);
static DEVICE_ATTR(selftest, 0660, 
		bma2x2_selftest_show, bma2x2_selftest_store);
static DEVICE_ATTR(softreset, 0660, 
		NULL, bma2x2_softreset_store);
static DEVICE_ATTR(temperature, 0660, 
		bma2x2_temperature_show, NULL);
static DEVICE_ATTR(place, 0660, 
		bma2x2_place_show, NULL);
static DEVICE_ATTR(gsensor_status, 0660, bma2x2_gsensor_status_show, NULL);
static DEVICE_ATTR(gs_cali, 0660, bma2x2_asus_calibrarion_show, bma2x2_asus_calibrarion_store);
static DEVICE_ATTR(gsensor_value, 0664, bma2x2_camera_value_show, NULL);

//-------------------- Add by for Zen Motion --------------------
#ifdef CONFIG_FLICK
static DEVICE_ATTR(flick_time_period, S_IRUGO|S_IWUSR|S_IWGRP|S_IROTH, bma2x2_flick_time_period_show, bma2x2_flick_time_period_store);
static DEVICE_ATTR(en_flick, S_IRUGO|S_IWUSR|S_IWGRP|S_IROTH, bma2x2_enable_flick_show, bma2x2_enable_flick_store);
#endif
static DEVICE_ATTR(en_terminal, S_IRUGO|S_IWUSR|S_IWGRP|S_IROTH, bma2x2_enable_terminal_show, bma2x2_enable_terminal_store);
//------------------------------------------------------------

#ifdef CONFIG_DOUBLE_TAP
static DEVICE_ATTR(tap_time_period, S_IRUGO|S_IWUSR|S_IWGRP|S_IROTH, bma2x2_tap_time_period_show, bma2x2_tap_time_period_store);
static DEVICE_ATTR(en_double_tap, S_IRUGO|S_IWUSR|S_IWGRP|S_IROTH, bma2x2_en_double_tap_show, bma2x2_en_double_tap_store);
#endif

#ifdef CONFIG_SIG_MOTION
static DEVICE_ATTR(en_sig_motion, S_IRUGO|S_IWUSR|S_IWGRP|S_IROTH, bma2x2_en_sig_motion_show, bma2x2_en_sig_motion_store);
#endif

#ifdef CONFIG_FLICK_HV
static DEVICE_ATTR(en_flick_hv, S_IRUGO|S_IWUSR|S_IWGRP|S_IROTH, bma2x2_enable_flick_hv_show, bma2x2_enable_flick_hv_store);
#endif

static DEVICE_ATTR(status, S_IRUGO, bma2x2_chip_status_show, NULL);

static struct attribute *bma2x2_attributes[] = {
	&dev_attr_range.attr,
	&dev_attr_bandwidth.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_value.attr,
	&dev_attr_value_cache.attr,
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_SleepDur.attr,
	&dev_attr_reg.attr,
	&dev_attr_fast_calibration_x.attr,
	&dev_attr_fast_calibration_y.attr,
	&dev_attr_fast_calibration_z.attr,
	&dev_attr_fifo_mode.attr,
	&dev_attr_fifo_framecount.attr,
	&dev_attr_fifo_trig.attr,
	&dev_attr_fifo_trig_src.attr,
	&dev_attr_fifo_data_sel.attr,
	&dev_attr_fifo_data_frame.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_offset_x.attr,
	&dev_attr_offset_y.attr,
	&dev_attr_offset_z.attr,
	&dev_attr_enable_int.attr,
	&dev_attr_int_mode.attr,
	&dev_attr_slope_duration.attr,
	&dev_attr_slope_threshold.attr,
	&dev_attr_slope_no_mot_duration.attr,
	&dev_attr_slope_no_mot_threshold.attr,
	&dev_attr_high_g_duration.attr,
	&dev_attr_high_g_threshold.attr,
	&dev_attr_low_g_duration.attr,
	&dev_attr_low_g_threshold.attr,
	&dev_attr_tap_threshold.attr,
	&dev_attr_tap_duration.attr,
	&dev_attr_tap_quiet.attr,
	&dev_attr_tap_shock.attr,
	&dev_attr_tap_samp.attr,
	&dev_attr_orient_mode.attr,
	&dev_attr_orient_blocking.attr,
	&dev_attr_orient_hyst.attr,
	&dev_attr_orient_theta.attr,
	&dev_attr_flat_theta.attr,
	&dev_attr_flat_hold_time.attr,
	&dev_attr_selftest.attr,
	&dev_attr_softreset.attr,
	&dev_attr_temperature.attr,
	&dev_attr_place.attr,
	&dev_attr_gsensor_status.attr,
	&dev_attr_gs_cali.attr,
	&dev_attr_gsensor_value.attr,
#ifdef CONFIG_FLICK
	&dev_attr_flick_time_period.attr,
	&dev_attr_en_flick.attr,
#endif
	&dev_attr_en_terminal.attr,
#ifdef CONFIG_DOUBLE_TAP
	&dev_attr_en_double_tap.attr,
#endif
#ifdef CONFIG_SIG_MOTION
	&dev_attr_en_sig_motion.attr,
#endif
#ifdef CONFIG_FLICK_HV
	&dev_attr_en_flick_hv.attr,
#endif
	&dev_attr_status.attr,
	NULL
};

static struct attribute_group bma2x2_attribute_group = {
	.attrs = bma2x2_attributes
};

#ifdef CONFIG_SIG_MOTION
static struct attribute *bma2x2_sig_motion_attributes[] = {
	&dev_attr_slope_duration.attr,
	&dev_attr_slope_threshold.attr,
	&dev_attr_en_sig_motion.attr,
	NULL
};
static struct attribute_group bma2x2_sig_motion_attribute_group = {
	.attrs = bma2x2_sig_motion_attributes
};
#endif

#ifdef CONFIG_DOUBLE_TAP
static struct attribute *bma2x2_double_tap_attributes[] = {
	&dev_attr_tap_threshold.attr,
	&dev_attr_tap_duration.attr,
	&dev_attr_tap_quiet.attr,
	&dev_attr_tap_shock.attr,
	&dev_attr_tap_samp.attr,
	&dev_attr_tap_time_period.attr,
	&dev_attr_en_double_tap.attr,
	NULL
};
static struct attribute_group bma2x2_double_tap_attribute_group = {
	.attrs = bma2x2_double_tap_attributes
};
#endif

#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)
unsigned char *orient[] = {
	"downward looking portrait upright",
	"downward looking portrait upside-down",
	"downward looking landscape left",
	"downward looking landscape right",
	"upward looking portrait upright",
	"upward looking portrait upside-down",
	"upward looking landscape left",
	"upward looking landscape right"
	};

#ifdef CONFIG_FLICK
	#ifdef CONFIG_FLICK_DEBUG
static int bma2x2_get_high_g_event(struct bma2x2_data *bma2x2)
{
	unsigned char first_value = 0;
	unsigned char sign_value = 0;
	int axiz, state = 0;
	for (axiz = 0; axiz < 3; axiz++) {
		bma2x2_get_HIGH_first(g_accel_data->bma2x2_client, axiz, &first_value);
		if (first_value == 1) {
			bma2x2_get_HIGH_sign(g_accel_data->bma2x2_client, &sign_value);
			if (sign_value == 1) {
				if (axiz == 0){
					state =  SINGLE_FLICK_BACKWARD;
				}
				if (axiz == 1){
					state =  SINGLE_FLICK_LEFT;
				}
				if (axiz == 2){
					state = SINGLE_FLICK_UP;
				}
			} else {
				if (axiz == 0){
					state = SINGLE_FLICK_FORWARD;
				}
				if (axiz == 1){
					state = SINGLE_FLICK_RIGHT;
				}					
				if (axiz == 2){
					state = SINGLE_FLICK_DOWN;
				}
			}
		}
	}
	return state;
}
*/
	#endif
#endif

#ifndef CONFIG_SIG_MOTION
static void bma2x2_slope_interrupt_handle(struct bma2x2_data *bma2x2)
{
	unsigned char first_value = 0;
	unsigned char sign_value = 0;
	int i;
	for (i = 0; i < 3; i++) {
		bma2x2_get_slope_first(g_accel_data->bma2x2_client, i, &first_value);
		if (first_value == 1) {
			bma2x2_get_slope_sign(g_accel_data->bma2x2_client,
								&sign_value);
			if (sign_value == 1) {
				if (i == 0)
					input_report_rel(g_accel_data->dev_interrupt,
							SLOP_INTERRUPT,
							SLOPE_INTERRUPT_X_N);
				if (i == 1)
					input_report_rel(g_accel_data->dev_interrupt,
							SLOP_INTERRUPT,
							SLOPE_INTERRUPT_Y_N);
				if (i == 2)
					input_report_rel(bma2x2->dev_interrupt,
							SLOP_INTERRUPT,
							SLOPE_INTERRUPT_Z_N);
			} else {
				if (i == 0)
					input_report_rel(g_accel_data->dev_interrupt,
							SLOP_INTERRUPT,
							SLOPE_INTERRUPT_X);
				if (i == 1)
					input_report_rel(g_accel_data->dev_interrupt,
							SLOP_INTERRUPT,
							SLOPE_INTERRUPT_Y);
				if (i == 2)
					input_report_rel(g_accel_data->dev_interrupt,
							SLOP_INTERRUPT,
							SLOPE_INTERRUPT_Z);

			}
		}

		PINFO("Slop interrupt happened,exis is %d,"
					"first is %d,sign is %d\n", i,
						first_value, sign_value);
	}
}
#endif

static void bma2x2_irq_work_func(struct work_struct *work)
{
//	struct bma2x2_data *bma2x2 = container_of((struct work_struct *)work, struct bma2x2_data, irq_work);
#ifdef CONFIG_DOUBLE_TAP
	unsigned long flags;
	struct i2c_client *client = g_accel_data->bma2x2_client;
#endif
#ifdef CONFIG_FLICK
	//int state = 0;
	unsigned long flick_flags;
	u64 now_jiffies = 0;
	short flick_z_axis = 0;
#endif
	unsigned char status = 0;
	unsigned char first_value = 0;
	unsigned char sign_value = 0;
	short z_value = 0;
#ifdef CONFIG_DOUBLE_TAP
	unsigned char bandwith_data = 0;
	static unsigned long pre_jiffies = 0;
#endif

#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
	static struct bma2x2acc acc;
	APS_LOG("alp test irq func !!\n");

	bma2x2_get_interruptstatus2(g_accel_data->bma2x2_client, &status);

	if ((status&0x80) == 0x80) {
		/* PINFO("New data interrupt happened\n");*/
		bma2x2_read_accel_xyz(g_accel_data->bma2x2_client,
					g_accel_data->sensor_type, &acc);
		input_report_abs(g_accel_data->input, ABS_X, acc.x);
		input_report_abs(g_accel_data->input, ABS_Y, acc.y);
		input_report_abs(g_accel_data->input, ABS_Z, acc.z);
		input_sync(g_accel_data->input);
		mutex_lock(&g_accel_data->value_mutex);
		g_accel_data->value = acc;
		mutex_unlock(&g_accel_data->value_mutex);
		return;
	}
#endif

	bma2x2_get_interruptstatus1(g_accel_data->bma2x2_client, &status);
	APS_LOG("bma2x2_irq_work_func, status = 0x%x\n", status);

#ifdef CONFIG_SIG_MOTION
	if (status & 0x04)	{
		if (atomic_read(&g_accel_data->en_sig_motion) == 1) {
			APS_LOG("Significant motion interrupt happened\n");
			/* close sig sensor,
			it will be open again if APP wants */
			bma2x2_set_en_sig_motion(g_accel_data, 0);

			input_report_rel(g_accel_data->dev_interrupt,
				SLOP_INTERRUPT, 1);
			input_sync(g_accel_data->dev_interrupt);
		}
	}
#endif

#ifdef CONFIG_DOUBLE_TAP
	if (status & 0x20) {
		if (atomic_read(&g_accel_data->en_double_tap) == 1) {
			PINFO("single tap interrupt happened\n");
			bma2x2_set_Int_Enable(client, 8, 0);
			if (g_accel_data->tap_times == 0)	{
				mod_timer(&g_accel_data->tap_timer, jiffies +
				msecs_to_jiffies(g_accel_data->tap_time_period));
				pre_jiffies = jiffies;
				g_accel_data->tap_times = 1;
			} else {
				/* only double tap is judged */
				APS_LOG("double tap , spend [%d] times\n", jiffies_to_msecs(jiffies-pre_jiffies));
				spin_lock_irqsave(&g_accel_data->tap_lock, flags);
				g_accel_data->tap_times = 0;
				del_timer(&g_accel_data->tap_timer);
				spin_unlock_irqrestore(&g_accel_data->tap_lock, flags);
				input_report_rel(g_accel_data->dev_interrupt,
					DOUBLE_TAP_INTERRUPT,
					DOUBLE_TAP_INTERRUPT_HAPPENED);
				input_sync(g_accel_data->dev_interrupt);
			}
			bma2x2_set_Int_Enable(client, 8, 1);
		}
	}
#endif

	switch (status) {

	case 0x01:
		PINFO("Low G interrupt happened\n");
		input_report_rel(g_accel_data->dev_interrupt, LOW_G_INTERRUPT,
				LOW_G_INTERRUPT_HAPPENED);
		break;

#ifdef CONFIG_FLICK
	case 0x02:

		// Debounce 400 ms
		now_jiffies = get_jiffies_64();
		if (g_accel_data->flick_debounce_time == 0) {
			g_accel_data->flick_debounce_time = now_jiffies;
		} else if ((now_jiffies < g_accel_data->flick_debounce_time + FLICK_DEBOUNCE_TIME) && 
				(g_accel_data->flick_times == 1)) {
			// APS_LOG("Skip flick interrupt , [%d] ms\n", jiffies_to_msecs(now_jiffies - g_accel_data->flick_debounce_time));
			break;
		} else if ((now_jiffies < g_accel_data->flick_debounce_time + FLICK_DOUBLE_DEBOUNCE_TIME) && 
				(g_accel_data->flick_times == 2)) {
			// APS_LOG("Skip flick interrupt , [%d] ms\n", jiffies_to_msecs(now_jiffies - g_accel_data->flick_debounce_time));
			break;
		}

		/* If Z-axis > 0 then skip the flick event */
		bma2x2_read_accel_z(g_accel_data->bma2x2_client, g_accel_data->sensor_type, &flick_z_axis);
		if (flick_z_axis > 0) {
			APS_LOG("Skip flick interrupt , Z-axis [%d]\n", flick_z_axis);
			spin_lock_irqsave(&g_accel_data->flick_lock, flick_flags);
			// setting no motion flag 
			g_accel_data->is_no_motion = 0;
			// delete single tap timer 
			if (g_accel_data->flick_times != 0) del_timer(&g_accel_data->flick_timer); 
			g_accel_data->flick_times = 0;
			spin_unlock_irqrestore(&g_accel_data->flick_lock, flick_flags);
			/* Disable Flick Interruts */
			bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 1, 0);
			bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 2, 0);
			// Enable No Motion Interrupt 
			bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 15, 1);
			bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 12, 1);
			bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 13, 1);
			break;
		}

		if (atomic_read(&g_accel_data->en_flick) == 1 || atomic_read(&g_accel_data->en_flick_hv) == 1) {
			/// Get Event for debug 
			#ifdef CONFIG_FLICK_DEBUG
			state = bma2x2_get_high_g_event(bma2x2);
			APS_LOG("[Debug] flick interrupt happened ... Event [%d]\n", state);
			#endif

			/* Disable Flick Interruts */
			bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 1, 0);
			bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 2, 0);

			g_accel_data->flick_debounce_time = now_jiffies; 

			if (g_accel_data->flick_times == 0)	{
				/* single tap is judged */
				APS_LOG("[Single] flick interrupt happened , Z-axis [%d]\n", flick_z_axis);
				spin_lock_irqsave(&g_accel_data->flick_lock, flick_flags);
				mod_timer(&g_accel_data->flick_timer, jiffies +
						msecs_to_jiffies(g_accel_data->flick_time_period));
				pre_jiffies = jiffies;
				g_accel_data->flick_times = 1;
				spin_unlock_irqrestore(&g_accel_data->flick_lock, flick_flags);
			} else if (g_accel_data->flick_times == 1){
				/* double tap is judged */
				APS_LOG("[Double] flick interrupt happened, interval [%d] times, Z-axis [%d]\n", jiffies_to_msecs(jiffies-pre_jiffies), flick_z_axis);
				spin_lock_irqsave(&g_accel_data->flick_lock, flick_flags);
				pre_jiffies = jiffies;
				g_accel_data->flick_times = 2;
				// delete single tap timer 
				del_timer(&g_accel_data->flick_timer); 
				// create double tap debounce timer
				mod_timer(&g_accel_data->flick_timer, jiffies +
						msecs_to_jiffies(450));
				spin_unlock_irqrestore(&g_accel_data->flick_lock, flick_flags);
			} else {
				/* unknown tap is judged */
				APS_LOG("[Unknown] flick interrupt happened, interval [%d] times, Z-axis [%d]\n", jiffies_to_msecs(jiffies-pre_jiffies), flick_z_axis);
				spin_lock_irqsave(&g_accel_data->flick_lock, flick_flags);
				g_accel_data->flick_times = 0;
				// setting no motion flag 
				g_accel_data->is_no_motion = 0;
				// delete single tap timer 
				del_timer(&g_accel_data->flick_timer); 
				spin_unlock_irqrestore(&g_accel_data->flick_lock, flick_flags);
				// Enable No Motion Interrupt 
				bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 15, 1);
				bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 12, 1);
				bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 13, 1);
				break;
			}

			/* Enable Flick Interruts */
			bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 1, 1);
			bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 2, 1);
		}
		break;
#endif

#ifndef CONFIG_SIG_MOTION
	case 0x04:
		bma2x2_slope_interrupt_handle(g_accel_data);
		break;
#endif

#ifdef CONFIG_FLICK
	case 0x08:
		APS_LOG("slow/ no motion interrupt happened\n");
		spin_lock_irqsave(&g_accel_data->flick_lock, flick_flags);
		/* Setting No Motion Flag */
		g_accel_data->is_no_motion = 1;
		spin_unlock_irqrestore(&g_accel_data->flick_lock, flick_flags);
		/* Disable No Motion Interrupt */
		bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 12, 0);
		bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 13, 0);
		/* Enable Flick Interruts */
		bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 1, 1);
		bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 2, 1);
		break;
#endif

#ifndef CONFIG_DOUBLE_TAP
	case 0x10:
		PINFO("double tap interrupt happened\n");
		input_report_rel(g_accel_data->dev_interrupt,
			DOUBLE_TAP_INTERRUPT,
			DOUBLE_TAP_INTERRUPT_HAPPENED);
		break;
	case 0x20:
		PINFO("single tap interrupt happened\n");
		input_report_rel(g_accel_data->dev_interrupt,
			SINGLE_TAP_INTERRUPT,
			SINGLE_TAP_INTERRUPT_HAPPENED);
		break;
#endif

	case 0x40:
		bma2x2_get_orient_status(g_accel_data->bma2x2_client,
				    &first_value);

#ifdef CONFIG_DOUBLE_TAP
		/* Add by Tom to check bandwith is 31.25 Hz */
		if (bma2x2_get_bandwidth(g_accel_data->bma2x2_client, &bandwith_data) >= 0) {
			if((atomic_read(&g_accel_data->en_double_tap) == 0) && (bandwith_data != 10)) {
				APS_LOG("bandwith [%d] -> [%d]\n", bandwith_data, BMA2X2_BW_SET);
				/* Change Bandwidth to 31.25 Hz */
				bma2x2_set_bandwidth(g_accel_data->bma2x2_client, BMA2X2_BW_SET);
			}
		}
#endif
		
		/* Add by Tom for Skip Same Terminal Event */
		if ((g_accel_data->pre_ter_event == TERMINAL_DOWNWARD) && 
			(first_value == 0 || first_value == 1 || first_value == 2 || first_value == 3)) {
			if(DEBUG) APS_LOG("Skip Same Event [%s], orient interrupt happened, [%s]\n", 
				(g_accel_data->pre_ter_event == TERMINAL_DOWNWARD) ? "DOWN": "UP", orient[first_value]);
			break;
		} else if ((g_accel_data->pre_ter_event == TERMINAL_UPWARD) && 
                        (first_value == 4 || first_value == 5 || first_value == 6 || first_value == 7)) {
			if(DEBUG) APS_LOG("Skip Same Event [%s], orient interrupt happened, [%s]\n", 
				(g_accel_data->pre_ter_event == TERMINAL_DOWNWARD) ? "DOWN": "UP", orient[first_value]);
			break;
		}
		if(DEBUG) {
			bma2x2_get_orient_flat_status(g_accel_data->bma2x2_client,&sign_value);
			APS_LOG("orient interrupt happened, [%s] , flat status is [%d]\n", orient[first_value], sign_value);
		}
		g_accel_data->ter_event = first_value;

		/* Workaround for face up, but not flat */
		if ((g_accel_data->pre_ter_event == TERMINAL_DOWNWARD) && g_accel_data->ter_event >= 4) {
			APS_LOG("terminal interrupt happened, [%s]\n", orient[g_accel_data->ter_event]);
			g_accel_data->pre_ter_event = TERMINAL_UPWARD;
			if (g_accel_data->ter_event == 4) {
				input_report_abs(g_accel_data->dev_terminal, 
						ORIENT_INTERRUPT, UPWARD_PORTRAIT_UP_INTERRUPT_HAPPENED);
			} else if (g_accel_data->ter_event == 5) {
				input_report_abs(g_accel_data->dev_terminal, 
						ORIENT_INTERRUPT, UPWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED);
			} else if (g_accel_data->ter_event == 6) {
				input_report_abs(g_accel_data->dev_terminal, 
						ORIENT_INTERRUPT, UPWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED);
			} else if (g_accel_data->ter_event == 7) {
				input_report_abs(g_accel_data->dev_terminal,
						ORIENT_INTERRUPT, UPWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED);
			} else {
				input_report_abs(g_accel_data->dev_terminal,
						ORIENT_INTERRUPT, UPWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED);
			}
			input_sync(g_accel_data->dev_terminal);
		}

		break;

	case 0x80:
		bma2x2_get_orient_flat_status(g_accel_data->bma2x2_client,&sign_value);
		if(DEBUG) APS_LOG("flat interrupt happened,flat status is %d\n",sign_value);
		/* Flat */
		if (sign_value == 1) { 
			/* Check Z-axis to decide face down or face up */
			bma2x2_read_accel_z(g_accel_data->bma2x2_client, g_accel_data->sensor_type, &z_value);
			if(DEBUG) APS_LOG("flat interrupt happened, z-axis [%d] , pre_terminal [%s]\n",
				z_value, (g_accel_data->pre_ter_event == TERMINAL_DOWNWARD) ? "DOWN": "UP");
			/* Face Down */
			if((z_value > 350) && (g_accel_data->pre_ter_event != TERMINAL_DOWNWARD)) {
				APS_LOG("terminal interrupt happened, [%s]\n", orient[g_accel_data->ter_event]);
				g_accel_data->pre_ter_event = TERMINAL_DOWNWARD;
				if (g_accel_data->ter_event == 0) {
					input_report_abs(g_accel_data->dev_terminal, 
							ORIENT_INTERRUPT, DOWNWARD_PORTRAIT_UP_INTERRUPT_HAPPENED);
				} else if (g_accel_data->ter_event == 1) {
					input_report_abs(g_accel_data->dev_terminal, 
							ORIENT_INTERRUPT, DOWNWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED);
				} else if (g_accel_data->ter_event == 2) {
					input_report_abs(g_accel_data->dev_terminal, 
							ORIENT_INTERRUPT, DOWNWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED);
				} else if (g_accel_data->ter_event == 3) {
					input_report_abs(g_accel_data->dev_terminal, 
							ORIENT_INTERRUPT, DOWNWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED);
				} else {
					APS_LOG("terminal interrupt (Orient != Z-axis), default [DOWN]\n");
					input_report_abs(g_accel_data->dev_terminal, 
							ORIENT_INTERRUPT, DOWNWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED);
				}
				input_sync(g_accel_data->dev_terminal);
			/* Face UP */
			} else if ((z_value < -350) && (g_accel_data->pre_ter_event != TERMINAL_UPWARD)) {
				APS_LOG("terminal interrupt happened, [%s]\n", orient[g_accel_data->ter_event]);
				g_accel_data->pre_ter_event = TERMINAL_UPWARD;
				if (g_accel_data->ter_event == 4) {
					input_report_abs(g_accel_data->dev_terminal, 
							ORIENT_INTERRUPT, UPWARD_PORTRAIT_UP_INTERRUPT_HAPPENED);
				} else if (g_accel_data->ter_event == 5) {
					input_report_abs(g_accel_data->dev_terminal, 
							ORIENT_INTERRUPT, UPWARD_PORTRAIT_DOWN_INTERRUPT_HAPPENED);
				} else if (g_accel_data->ter_event == 6) {
					input_report_abs(g_accel_data->dev_terminal, 
							ORIENT_INTERRUPT, UPWARD_LANDSCAPE_LEFT_INTERRUPT_HAPPENED);
				} else if (g_accel_data->ter_event == 7) {
					input_report_abs(g_accel_data->dev_terminal,
							ORIENT_INTERRUPT, UPWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED);
				} else {
					APS_LOG("terminal interrupt (Orient != Z-axis), default [UP]\n");
					input_report_abs(g_accel_data->dev_terminal,
							ORIENT_INTERRUPT, UPWARD_LANDSCAPE_RIGHT_INTERRUPT_HAPPENED);
				}
				input_sync(g_accel_data->dev_terminal);
			} else {
				if(DEBUG) APS_LOG("skip flat interrupt , z-axis [%d] , pre_terminal [%s]\n", 
					z_value, (g_accel_data->pre_ter_event == TERMINAL_DOWNWARD) ? "DOWN": "UP");
			}
		}
		break;

	default:
		break;
	}
}

static irqreturn_t bma2x2_irq_handler(int irq, void *handle)
{
	struct bma2x2_data *data = handle;
	APS_FUN();
	if (data == NULL)
		return IRQ_HANDLED;
	if (data->bma2x2_client == NULL)
		return IRQ_HANDLED;

	schedule_work(&data->irq_work);

	return IRQ_HANDLED;
}
#endif /* defined(BMA2X2_ENABLE_INT1)||defined(BMA2X2_ENABLE_INT2) */

static int of_get_bma2x2_platform_data(struct device *dev)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek,bma2x2");
	if (node) {
		gsensor_int_gpio_number = of_get_named_gpio(node, "int-gpio", 0);
		gsensor_irq = irq_of_parse_and_map(node, 0);
		if (gsensor_irq < 0) {
			APS_ERR("gsensor request_irq IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		APS_LOG("gsensor_int_gpio_number %d; alsps_irq : %d\n", gsensor_int_gpio_number, gsensor_irq);
	}
	return 0;
}

static int bma2x2_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	struct input_dev *dev;
	struct bst_dev  *dev_acc;
#ifdef CONFIG_DOUBLE_TAP
	int comres = 0;
	unsigned char dt_thd = 0;
#endif

#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)
	struct bosch_sensor_specific *pdata;
#endif
	struct input_dev *dev_interrupt;

	APS_FUN();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("i2c_check_functionality error\n");
		err = -EIO;
		goto exit;
	}
	g_accel_data = kzalloc(sizeof(struct bma2x2_data), GFP_KERNEL);
	if (!g_accel_data) {
		err = -ENOMEM;
		goto exit;
	}

	/* do soft reset */
	APS_LOG(" do soft reset +++\n");
	mdelay(5);
	if (bma2x2_soft_reset(client) < 0) {
		PERR("i2c bus write error, pls check HW connection\n");
		err = -EINVAL;
		goto kfree_exit;
	}
	RESET_DELAY();
	/* read and check chip id */
	APS_LOG(" bma2x2_check_chip_id +++\n");
	if (bma2x2_check_chip_id(client, g_accel_data) < 0) {
		err = -EINVAL;
		goto kfree_exit;
	}
	g_accel_data->state=1;	//	selftest success , gsensor status = 1 ;

	i2c_set_clientdata(client, g_accel_data);
	g_accel_data->bma2x2_client = client;
	mutex_init(&g_accel_data->value_mutex);
	mutex_init(&g_accel_data->mode_mutex);
	mutex_init(&g_accel_data->enable_mutex);
	bma2x2_set_bandwidth(client, BMA2X2_BW_SET);
	bma2x2_set_range(client, BMA2X2_RANGE_SET);

#if defined(BMA2X2_ENABLE_INT1) || defined(BMA2X2_ENABLE_INT2)
	pdata = client->dev.platform_data;
	if (pdata) {
		if (pdata->irq_gpio_cfg && (pdata->irq_gpio_cfg() < 0)) {
			PERR("IRQ GPIO conf. error %d\n", client->irq);
		}
	}

	#ifdef BMA2X2_ENABLE_INT1
	/* maps interrupt to INT1 pin */
	bma2x2_set_int1_pad_sel(client, PAD_LOWG);
	bma2x2_set_int1_pad_sel(client, PAD_HIGHG);
	bma2x2_set_int1_pad_sel(client, PAD_SLOP);
	bma2x2_set_int1_pad_sel(client, PAD_DOUBLE_TAP);
	bma2x2_set_int1_pad_sel(client, PAD_SINGLE_TAP);
	bma2x2_set_int1_pad_sel(client, PAD_ORIENT);
	bma2x2_set_int1_pad_sel(client, PAD_FLAT);
	bma2x2_set_int1_pad_sel(client, PAD_SLOW_NO_MOTION);
		#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
		bma2x2_set_newdata(client, BMA2X2_INT1_NDATA, 1);
		bma2x2_set_newdata(client, BMA2X2_INT2_NDATA, 0);
		#endif
	#endif

	#ifdef BMA2X2_ENABLE_INT2
	/* maps interrupt to INT2 pin */
	bma2x2_set_int2_pad_sel(client, PAD_LOWG);
	bma2x2_set_int2_pad_sel(client, PAD_HIGHG);
	bma2x2_set_int2_pad_sel(client, PAD_SLOP);
	bma2x2_set_int2_pad_sel(client, PAD_DOUBLE_TAP);
	bma2x2_set_int2_pad_sel(client, PAD_SINGLE_TAP);
	bma2x2_set_int2_pad_sel(client, PAD_ORIENT);
	bma2x2_set_int2_pad_sel(client, PAD_FLAT);
	bma2x2_set_int2_pad_sel(client, PAD_SLOW_NO_MOTION);
		#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
		bma2x2_set_newdata(client, BMA2X2_INT1_NDATA, 0);
		bma2x2_set_newdata(client, BMA2X2_INT2_NDATA, 1);
		#endif
	#endif

	bma2x2_set_Int_Mode(client, 14);/*latch interrupt 50ms*/

	/* do not open any interrupt here  */
	/*10,orient
	11,flat*/
	/* bma2x2_set_Int_Enable(client, 10, 1);	*/
	/* bma2x2_set_Int_Enable(client, 11, 1); */

	#ifdef CONFIG_BMA_ENABLE_NEWDATA_INT
	/* enable new data interrupt */
	bma2x2_set_Int_Enable(client, 4, 1);
	#endif

	of_get_bma2x2_platform_data(&client->dev);
	APS_LOG("gsensor_init_gpio_number = %d\n", gsensor_int_gpio_number);
	err += gpio_request_one(gsensor_int_gpio_number, GPIOF_IN, "bma2x2_int");
	err += gpio_direction_input(gsensor_int_gpio_number);
	if (err) {
		PERR("can not request gpio to irq number\n");
		gsensor_int_gpio_number = 0;
	}
	err = request_irq(gsensor_irq, bma2x2_irq_handler, IRQF_TRIGGER_RISING, "gsensor-irq", g_accel_data);
	if (err)
		PERR("could not request irq\n");

	#ifdef CONFIG_SIG_MOTION
		enable_irq_wake(gsensor_irq);
	#endif

	INIT_WORK(&g_accel_data->irq_work, bma2x2_irq_work_func);
#endif

#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
	INIT_DELAYED_WORK(&g_accel_data->work, bma2x2_work_func);
#endif
	atomic_set(&g_accel_data->delay, BMA2X2_MAX_DELAY);
	atomic_set(&g_accel_data->enable, 0);

	APS_ERR(" bma2x2 init input subsystem +++\n");
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev_interrupt = input_allocate_device();
	if (!dev_interrupt) {
		kfree(g_accel_data);
		input_free_device(dev); /*free the successful dev and return*/
		return -ENOMEM;
	}

	/* only value events reported */
	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_I2C;
	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, ABSMIN, ABSMAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, ABSMIN, ABSMAX, 0, 0);

APS_ERR(" bma2x2 init input test 1 !!!\n");
	input_set_drvdata(dev, g_accel_data);
APS_ERR(" bma2x2 init input test 2 !!!\n");
	err = input_register_device(dev);
	if (err < 0) {
		APS_LOG("Cannot register input device\n");
		goto err_register_input_device;
	}
APS_ERR(" bma2x2 init input test 3 !!!\n");

#ifdef CONFIG_FLICK
	g_accel_data->dev_flick = input_allocate_device();
	if (!g_accel_data->dev_flick) {
		APS_LOG("Cannot allocate input device\n");
		return -ENOMEM;
	}
	g_accel_data->dev_flick->name = "bma2x2_flick";
	g_accel_data->dev_flick->id.bustype = BUS_I2C;
	input_set_capability(g_accel_data->dev_flick, EV_ABS, MOTION_FLICK_INTERRUPT);
	input_set_abs_params(g_accel_data->dev_flick, ABS_GAS, ABSMIN, ABSMAX, 0, 0);
	input_set_drvdata(g_accel_data->dev_flick, g_accel_data);
	err = input_register_device(g_accel_data->dev_flick);
	if (err < 0)
		goto err_register_input_device;
#endif

#ifdef CONFIG_FLICK_HV
	g_accel_data->dev_flick_hv = input_allocate_device();
	if (!g_accel_data->dev_flick_hv) {
		APS_LOG("Cannot allocate input device\n");
		return -ENOMEM;
	}
	g_accel_data->dev_flick_hv->name = "bma2x2_flick_hv";
	g_accel_data->dev_flick_hv->id.bustype = BUS_I2C;
	input_set_capability(g_accel_data->dev_flick_hv, EV_ABS, MOTION_FLICK_INTERRUPT);
	input_set_abs_params(g_accel_data->dev_flick_hv, ABS_WHEEL, ABSMIN, ABSMAX, 0, 0);
	input_set_drvdata(g_accel_data->dev_flick_hv, g_accel_data);
	err = input_register_device(g_accel_data->dev_flick_hv);
	if (err < 0)
		goto err_register_input_device;
#endif
	g_accel_data->dev_terminal = input_allocate_device();
	if (!dev) {
		APS_LOG("Cannot allocate input device\n");
		return -ENOMEM;
	}
	g_accel_data->dev_terminal->name = "bma2x2_terminal";
	g_accel_data->dev_terminal->id.bustype = BUS_I2C;
	input_set_capability(g_accel_data->dev_terminal, EV_ABS, MOTION_TERMINAL_INTERRUPT);
	input_set_abs_params(g_accel_data->dev_terminal, ABS_PRESSURE, ABSMIN, ABSMAX, 0, 0);
	input_set_drvdata(g_accel_data->dev_terminal, g_accel_data);
	err = input_register_device(g_accel_data->dev_terminal);
	if (err < 0)
		goto err_register_input_device;

	/* all interrupt generated events are moved to interrupt input devices*/
	dev_interrupt->name = "bma_interrupt";
	dev_interrupt->id.bustype = BUS_I2C;
	input_set_capability(dev_interrupt, EV_REL,
		SLOW_NO_MOTION_INTERRUPT);
	input_set_capability(dev_interrupt, EV_REL,
		LOW_G_INTERRUPT);
	input_set_capability(dev_interrupt, EV_REL,
		HIGH_G_INTERRUPT);
	input_set_capability(dev_interrupt, EV_REL,
		SLOP_INTERRUPT);
	input_set_capability(dev_interrupt, EV_REL,
		DOUBLE_TAP_INTERRUPT);
	input_set_capability(dev_interrupt, EV_REL,
		SINGLE_TAP_INTERRUPT);
	input_set_capability(dev_interrupt, EV_REL,
		UNKNOW_TAP_INTERRUPT);
	input_set_capability(dev_interrupt, EV_ABS,
		ORIENT_INTERRUPT);
	input_set_capability(dev_interrupt, EV_ABS,
		FLAT_INTERRUPT);
	input_set_drvdata(dev_interrupt, g_accel_data);

	err = input_register_device(dev_interrupt);
	if (err < 0)
		goto err_register_input_device_interrupt;

	g_accel_data->dev_interrupt = dev_interrupt;
	g_accel_data->input = dev;

#ifdef CONFIG_SIG_MOTION
	g_accel_data->g_sensor_class = class_create(THIS_MODULE, "sig_sensor");
	if (IS_ERR(g_accel_data->g_sensor_class)) {
		err = PTR_ERR(g_accel_data->g_sensor_class);
		g_accel_data->g_sensor_class = NULL;
		PERR("could not allocate g_sensor_class\n");
		goto err_create_class;
	}

	g_accel_data->g_sensor_dev = device_create(g_accel_data->g_sensor_class,NULL, 0, "%s", "g_sensor");
	if (unlikely(IS_ERR(g_accel_data->g_sensor_dev))) {
		err = PTR_ERR(g_accel_data->g_sensor_dev);
		g_accel_data->g_sensor_dev = NULL;

		PERR("could not allocate g_sensor_dev\n");
		goto err_create_g_sensor_device;
	}

	dev_set_drvdata(g_accel_data->g_sensor_dev, g_accel_data);

	err = sysfs_create_group(&g_accel_data->g_sensor_dev->kobj,
			&bma2x2_sig_motion_attribute_group);
	if (err < 0)
		goto error_sysfs;
#endif

#ifdef CONFIG_DOUBLE_TAP
	g_accel_data->g_sensor_class_doubletap =
		class_create(THIS_MODULE, "dtap_sensor");
	if (IS_ERR(g_accel_data->g_sensor_class_doubletap)) {
		err = PTR_ERR(g_accel_data->g_sensor_class_doubletap);
		g_accel_data->g_sensor_class_doubletap = NULL;
		PERR("could not allocate g_sensor_class_doubletap\n");
		goto err_create_class;
	}

	g_accel_data->g_sensor_dev_doubletap = device_create(
				g_accel_data->g_sensor_class_doubletap,
				NULL, 0, "%s", "g_sensor");
	if (unlikely(IS_ERR(g_accel_data->g_sensor_dev_doubletap))) {
		err = PTR_ERR(g_accel_data->g_sensor_dev_doubletap);
		g_accel_data->g_sensor_dev_doubletap = NULL;

		PERR("could not allocate g_sensor_dev_doubletap\n");
		goto err_create_g_sensor_device_double_tap;
	}

	dev_set_drvdata(g_accel_data->g_sensor_dev_doubletap, g_accel_data);

	err = sysfs_create_group(&g_accel_data->g_sensor_dev_doubletap->kobj,
			&bma2x2_double_tap_attribute_group);
	if (err < 0)
		goto error_sysfs;

	// Add by Tom for reset double tap threshold
	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_THRES__REG, &dt_thd);
	APS_LOG("Double Tap default threshod [%u]\n", dt_thd);
	dt_thd = BMA2X2_SET_BITSLICE(dt_thd, BMA2X2_TAP_THRES, 1);
	comres = bma2x2_smbus_write_byte(client, BMA2X2_TAP_THRES__REG, &dt_thd);
	comres = bma2x2_smbus_read_byte(client, BMA2X2_TAP_THRES__REG, &dt_thd);
	APS_LOG("Reset Double Tap threshod [%u]\n", dt_thd);
#endif

	err = sysfs_create_group(&g_accel_data->input->dev.kobj,
			&bma2x2_attribute_group);
	if (err < 0)
		goto error_sysfs;

	dev_acc = bst_allocate_device();
	if (!dev_acc) {
		err = -ENOMEM;
		goto error_sysfs;
	}
	dev_acc->name = ACC_NAME;

	bst_set_drvdata(dev_acc, g_accel_data);

	err = bst_register_device(dev_acc);
	if (err < 0)
		goto bst_free_acc_exit;

	g_accel_data->bst_acc = dev_acc;
	err = sysfs_create_group(&g_accel_data->bst_acc->dev.kobj,
			&bma2x2_attribute_group);

	if (err < 0)
		goto bst_free_exit;

	if (NULL != client->dev.platform_data) {
		g_accel_data->bst_pd = kzalloc(sizeof(*g_accel_data->bst_pd),
				GFP_KERNEL);

		if (NULL != g_accel_data->bst_pd) {
			memcpy(g_accel_data->bst_pd, client->dev.platform_data,
					sizeof(*g_accel_data->bst_pd));
			PINFO("%s sensor driver set place: p%d",
				g_accel_data->bst_pd->name, g_accel_data->bst_pd->place);
		}
	}
	// sys init ++
	err = 0;
	android_gsensor_kobj = kobject_create_and_add("android_gsensor", NULL);	
	err = sysfs_create_file(android_gsensor_kobj, &dev_attr_enable.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_delay.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_value.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_op_mode.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_gsensor_status.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_gs_cali.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_gsensor_value.attr);
#ifdef CONFIG_FLICK
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_flick_time_period.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_en_flick.attr);
#endif
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_en_terminal.attr);
#ifdef CONFIG_SIG_MOTION
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_en_sig_motion.attr);
#endif
#ifdef CONFIG_FLICK_HV
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_en_flick_hv.attr);
#endif
#ifdef CONFIG_DOUBLE_TAP
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_tap_threshold.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_tap_duration.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_tap_quiet.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_tap_shock.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_tap_samp.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_tap_time_period.attr);
	err += sysfs_create_file(android_gsensor_kobj, &dev_attr_en_double_tap.attr);
#endif
	if (err) {
		printk(KERN_ERR "alp.D : attr gsensor dev_attr_*  fail !!\n");
	}
	// sys init -


#ifdef CONFIG_HAS_EARLYSUSPEND
	g_accel_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	g_accel_data->early_suspend.suspend = bma2x2_early_suspend;
	g_accel_data->early_suspend.resume = bma2x2_late_resume;
	register_early_suspend(&g_accel_data->early_suspend);
#endif

	g_accel_data->bma_mode_enabled = 0;
	g_accel_data->fifo_datasel = 0;
	g_accel_data->fifo_count = 0;
#ifdef CONFIG_SIG_MOTION
	atomic_set(&g_accel_data->en_sig_motion, 0);
#endif
#ifdef CONFIG_DOUBLE_TAP
	atomic_set(&g_accel_data->en_double_tap, 0);
	g_accel_data->tap_times = 0;
	g_accel_data->tap_time_period = DEFAULT_TAP_JUDGE_PERIOD;
	spin_lock_init(&g_accel_data->tap_lock);
	setup_timer(&g_accel_data->tap_timer, bma2x2_tap_timeout_handle,
			(unsigned long)g_accel_data);
#endif
	if (bma2x2_set_mode(client, BMA2X2_MODE_SUSPEND, BMA_ENABLED_ALL) < 0)
		return -EINVAL;
//	PINFO("BMA2x2 driver probe successfully");
	APS_ERR(" bma2x2 probe success !!!!\n");
	
	bma2x2_init_flag = 0;
	// reg callback
#ifdef CONFIG_PM_SCREEN_STATE_NOTIFIER
	callback_struct = register_screen_state_notifier(&bma2x2_screen_chenged_listaner);
#endif
	/* Create loading calibration workqueue & Init delay work */
	g_accel_data->cali_wq = create_singlethread_workqueue("g-sensor_cali_wq");
	if(!g_accel_data->cali_wq)
	{       
		APS_LOG("create_singlethread_workqueue fail\n");
	} else {
		g_accel_data->cali_retry = 0;
		INIT_DELAYED_WORK(&g_accel_data->cali_work, bma2x2_workqueue_read_from_califile);
		queue_delayed_work(g_accel_data->cali_wq, &g_accel_data->cali_work, 40*HZ);
	}
	g_accel_data->is_cali = 0;

	/* Add by Tom for CTS Workaround */ 
	g_accel_data->en_sensor_type = -1;
//	g_accel_data->CTS_wq = create_singlethread_workqueue("BMA_CTS_wq");
//	INIT_DELAYED_WORK(&g_accel_data->CTS_dw, CTS_report_dummy_event);

	/* Add by Tom Init Terminal Parameters */
	g_accel_data->ter_event = 0;
	g_accel_data->pre_ter_event = TERMINAL_UNKNOWN;
	bma2x2_set_theta_flat(client, 10); // 10 angle
	bma2x2_set_flat_hold_time(client, 0); // 0 hold time

#ifdef CONFIG_FLICK
	atomic_set(&g_accel_data->en_flick, 0);
	g_accel_data->is_no_motion = 1;
	g_accel_data->flick_times = 0;
	g_accel_data->flick_debounce_time = 0; 
	g_accel_data->flick_time_period = DEFAULT_FLICK_JUDGE_PERIOD;
	spin_lock_init(&g_accel_data->flick_lock);
	setup_timer(&g_accel_data->flick_timer, bma2x2_flick_timeout_handle,
			(unsigned long)g_accel_data);
	bma2x2_set_high_g_threshold(g_accel_data->bma2x2_client, 120);
	bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 15, 1);
#endif

	APS_LOG("BMA2x2 driver probe successfully\n");

	return 0;

bst_free_exit:
	bst_unregister_device(dev_acc);

bst_free_acc_exit:
	bst_free_device(dev_acc);

error_sysfs:
	input_unregister_device(g_accel_data->input);

#ifdef CONFIG_DOUBLE_TAP
err_create_g_sensor_device_double_tap:
	class_destroy(g_accel_data->g_sensor_class_doubletap);
#endif

#ifdef CONFIG_SIG_MOTION
err_create_g_sensor_device:
	class_destroy(g_accel_data->g_sensor_class);
#endif

#if defined(CONFIG_SIG_MOTION) || defined(CONFIG_DOUBLE_TAP)
err_create_class:
	input_unregister_device(g_accel_data->dev_interrupt);
#endif

err_register_input_device_interrupt:

#ifdef CONFIG_FLICK
	input_free_device(g_accel_data->dev_flick);
#endif
#ifdef CONFIG_FLICK_HV
	input_free_device(g_accel_data->dev_flick_hv);
#endif
	input_free_device(g_accel_data->dev_terminal); 
	input_free_device(dev_interrupt);
	input_unregister_device(g_accel_data->input);

err_register_input_device:
	input_free_device(dev);

kfree_exit:
	if ((NULL != g_accel_data) && (NULL != g_accel_data->bst_pd)) {
		kfree(g_accel_data->bst_pd);
		g_accel_data->bst_pd = NULL;
	}
	kfree(g_accel_data);
exit:
	return err;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void bma2x2_early_suspend(struct early_suspend *h)
{
//	struct bma2x2_data *data =  container_of(h, struct bma2x2_data, early_suspend);
	APS_FUN();
	mutex_lock(&g_accel_data->enable_mutex);
	if (atomic_read(&g_accel_data->enable) == 1) {
		bma2x2_set_mode(g_accel_data->bma2x2_client,
			BMA2X2_MODE_SUSPEND, BMA_ENABLED_INPUT);
#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
		cancel_delayed_work_sync(&g_accel_data->work);
#endif
	}
	mutex_unlock(&data->enable_mutex);
}

static void bma2x2_late_resume(struct early_suspend *h)
{
//	struct bma2x2_data *data = container_of(h, struct bma2x2_data, early_suspend);
	if (NULL == g_accel_data)
		return;
	APS_FUN();
	mutex_lock(&g_accel_data->enable_mutex);
	if (atomic_read(&g_accel_data->enable) == 1) {
		bma2x2_set_mode(g_accel_data->bma2x2_client,
			BMA2X2_MODE_NORMAL, BMA_ENABLED_INPUT);
#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
		schedule_delayed_work(&g_accel_data->work,
				msecs_to_jiffies(atomic_read(&g_accel_data->delay)));
#endif
	}
	mutex_unlock(&data->enable_mutex);
}
#endif

static int bma2x2_i2c_remove(struct i2c_client *client)
{
//	struct bma2x2_data *data = i2c_get_clientdata(client);
	APS_FUN();
	if (NULL == g_accel_data)
		return 0;

	bma2x2_set_enable(&client->dev, 0);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&g_accel_data->early_suspend);
#endif
	sysfs_remove_group(&g_accel_data->input->dev.kobj, &bma2x2_attribute_group);
	input_unregister_device(g_accel_data->input);

	if (NULL != g_accel_data->bst_pd) {
		kfree(g_accel_data->bst_pd);
		g_accel_data->bst_pd = NULL;
	}

	kfree(g_accel_data);
	return 0;
}

void bma2x2_i2c_shutdown(struct i2c_client *client)
{
//	struct bma2x2_data *data = i2c_get_clientdata(client);
	APS_FUN();
	mutex_lock(&g_accel_data->enable_mutex);
	bma2x2_set_mode(g_accel_data->bma2x2_client,
		BMA2X2_MODE_DEEP_SUSPEND, BMA_ENABLED_ALL);
	mutex_unlock(&g_accel_data->enable_mutex);
}

#ifdef CONFIG_PM
static int bma2x2_suspend(struct i2c_client *client, pm_message_t mesg)
{
//	struct bma2x2_data *data = i2c_get_clientdata(client);
	APS_FUN();
	mutex_lock(&g_accel_data->enable_mutex);
	if (atomic_read(&g_accel_data->enable) == 1) {
		bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 10, 0); // Orientation
		bma2x2_set_Int_Enable(g_accel_data->bma2x2_client, 11, 0); // Flat
		bma2x2_set_mode(g_accel_data->bma2x2_client,
			BMA2X2_MODE_SUSPEND, BMA_ENABLED_INPUT);
#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
		cancel_delayed_work_sync(&g_accel_data->work);
#endif
	}
	mutex_unlock(&g_accel_data->enable_mutex);

	return 0;
}

static int bma2x2_resume(struct i2c_client *client)
{
//	struct bma2x2_data *data = i2c_get_clientdata(client);
	APS_FUN();
	mutex_lock(&g_accel_data->enable_mutex);
	if (atomic_read(&g_accel_data->enable) == 1) {
		bma2x2_set_mode(g_accel_data->bma2x2_client,
			BMA2X2_MODE_NORMAL, BMA_ENABLED_INPUT);
#ifndef CONFIG_BMA_ENABLE_NEWDATA_INT
		schedule_delayed_work(&g_accel_data->work,
				msecs_to_jiffies(atomic_read(&g_accel_data->delay)));
#endif
	}
	mutex_unlock(&g_accel_data->enable_mutex);

	return 0;
}

#else

#define bma2x2_suspend      NULL
#define bma2x2_resume       NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id bma2x2_id[] = {
	{ SENSOR_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, bma2x2_id);

#ifdef CONFIG_OF
static const struct of_device_id acc_of_match[] = {
	{.compatible = "mediatek,bma2x2"},
	{},
};
#endif

static struct i2c_driver bma2x2_i2c_driver = {
	.driver = {
//		.owner  = THIS_MODULE,
		.name   = SENSOR_NAME,
#ifdef CONFIG_OF
		.of_match_table = acc_of_match,
#endif
	},
	.suspend    = bma2x2_suspend,
	.resume     = bma2x2_resume,
	.id_table   = bma2x2_id,
	.probe      = bma2x2_i2c_probe,
	.remove     = bma2x2_i2c_remove,
	.shutdown   = bma2x2_i2c_shutdown,
};

static struct acc_init_info bma2x2_init_info = {
	.name = SENSOR_NAME,
	.init = bma2x2_local_init,
	.uninit = bma2x2_remove,
};

//----------------------------------added  for  MTK--------------------------------------
static int bma2x2_local_init(void)
{
	APS_FUN();
	if (i2c_add_driver(&bma2x2_i2c_driver)) {
		APS_ERR("add driver bma2x2 error !!\n");
		return -1;
	}
	if (-1 == bma2x2_init_flag) {
		APS_ERR("add driver--bma2x2_init_flag check error\n");
		return -1;
	}

	return 0;
}
static int bma2x2_remove(void)
{
	APS_FUN();
	i2c_del_driver(&bma2x2_i2c_driver);
	bma2x2_init_flag = -1;

	return 0;
}
//--------------------------------- ------ End ------------------------------------------

static int __init BMA2X2_init(void)
{
	APS_FUN();
	return  acc_driver_add(&bma2x2_init_info);	
//	return i2c_add_driver(&bma2x2_i2c_driver);
}

static void __exit BMA2X2_exit(void)
{
	APS_FUN();
//	i2c_del_driver(&bma2x2_i2c_driver);
}

MODULE_AUTHOR("contact@bosch-sensortec.com");
MODULE_DESCRIPTION("BMA2X2 ACCELEROMETER SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

module_init(BMA2X2_init);
module_exit(BMA2X2_exit);

