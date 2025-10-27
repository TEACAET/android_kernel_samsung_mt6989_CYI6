/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2025 MediaTek Inc.
 */

/*****************************************************************************
 *
 * Filename:
 * ---------
 *     gc05a2mipiraw_Sensor.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _GC05A2MIPI_SENSOR_H
#define _GC05A2MIPI_SENSOR_H

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define_v4l2.h"
#include "kd_imgsensor_errcode.h"

#include "gc05a2_ana_gain_table.h"
#include "gc05a2_Sensor_setting.h"

#include "adaptor-subdrv-ctrl.h"
#include "adaptor-i2c.h"
#include "adaptor.h"
#include "adaptor-ctrls.h"

#define GC05A2_OTP_CHECK_BANK       0x2000

#define GC05A2_OTP_BANK1_MARK       0x01
#define GC05A2_OTP_BANK2_MARK       0x03
#define GC05A2_OTP_BANK3_MARK       0x07

#define GC05A2_OTP_BANK1_START_ADDR   (0x2020)
#define GC05A2_OTP_BANK2_START_ADDR   (0x5520)
#define GC05A2_OTP_BANK3_START_ADDR   (0x8A20)
#define GC05A2_OTP_BANK4_START_ADDR   (0x1904)

#endif