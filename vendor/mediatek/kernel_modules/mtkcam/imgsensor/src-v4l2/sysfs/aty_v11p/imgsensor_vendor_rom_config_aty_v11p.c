// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Inc.
 */

#include "imgsensor_vendor_rom_config_aty_v11p.h"

const struct imgsensor_vendor_rom_info vendor_rom_info[] = {
		{SENSOR_POSITION_REAR, SC800CS_SENSOR_ID, &rear_sc800cs_cal_addr},
		{SENSOR_POSITION_FRONT, GC05A2_SENSOR_ID, &front_gc05a2_cal_addr},
		{SENSOR_POSITION_REAR, GC08A3_SENSOR_ID, &rear_gc08a3_cal_addr},
};
