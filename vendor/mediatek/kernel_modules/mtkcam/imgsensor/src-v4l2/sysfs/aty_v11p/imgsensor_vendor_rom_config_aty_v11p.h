// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Inc.
 */

#ifndef IMGESENSOR_VENDOR_ROM_CONFIG_ATY_V11P_H
#define IMGESENSOR_VENDOR_ROM_CONFIG_ATY_V11P_H

#include "imgsensor_vendor_specific.h"
#include "imgsensor_eeprom_rear_sc800cs.h"
#include "imgsensor_otp_front_gc05a2.h"
#include "imgsensor_eeprom_rear_gc08a3.h"
#include "kd_imgsensor.h"

#define REAR_CAMERA
#define FRONT_CAMERA

#define REAR_FLASH_SYSFS

#if IS_ENABLED(CONFIG_IMGSENSOR_SYSFS)
#define IMGSENSOR_HW_PARAM
#endif

extern const struct imgsensor_vendor_rom_info vendor_rom_info[SENSOR_POSITION_MAX];

#endif /*IMGESENSOR_VENDOR_ROM_CONFIG_ATY_V11P_H*/
