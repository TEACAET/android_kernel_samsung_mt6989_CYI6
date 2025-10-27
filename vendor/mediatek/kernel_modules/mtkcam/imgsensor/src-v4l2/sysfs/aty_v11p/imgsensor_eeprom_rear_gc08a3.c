// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Samsung Electronics Inc.
 */

#include "imgsensor_eeprom_rear_gc08a3.h"

#define REAR_GC08A3_MAX_CAL_SIZE               (0x0809 + 0x1)

#define REAR_GC08A3_HEADER_CHECKSUM_LEN        (0x00CB - 0x0000 + 0x1)
#define REAR_GC08A3_OEM_CHECKSUM_LEN           (0x01AB - 0x0160 + 0x1)
#define REAR_GC08A3_AWB_CHECKSUM_LEN           (0x01CB - 0x01B0 + 0x1)
#define REAR_GC08A3_SHADING_CHECKSUM_LEN       (0x05CB - 0x01D0 + 0x1)

#define REAR_GC08A3_DUAL_CHECKSUM_LEN          -1
#define REAR_GC08A3_PDAF_CHECKSUM_LEN          -1
#define REAR_GC08A3_OIS_CHECKSUM_LEN           -1

#define REAR_GC08A3_CONVERTED_MAX_CAL_SIZE     (0x0B59 + 0x1)
#define REAR_GC08A3_CONVERTED_AWB_CHECKSUM_LEN (0x01BB - 0x01B0 + 0x1)
#define REAR_GC08A3_CONVERTED_LSC_CHECKSUM_LEN (0x091B - 0x01C0 + 0x1)


const struct rom_converted_cal_addr rear_gc08a3_converted_cal_addr = {
	.rom_awb_cal_data_start_addr            = 0x01B0,
	.rom_awb_checksum_addr                  = 0x01BC,
	.rom_awb_checksum_len                   = (REAR_GC08A3_CONVERTED_AWB_CHECKSUM_LEN),
	.rom_shading_cal_data_start_addr        = 0x01C0,
	.rom_shading_checksum_addr              = 0x091C,
	.rom_shading_checksum_len               = (REAR_GC08A3_CONVERTED_LSC_CHECKSUM_LEN),
};

const struct imgsensor_vendor_rom_addr rear_gc08a3_cal_addr = {
	/* Set '-1' if not used */
	.camera_module_es_version               = 'A',
	.cal_map_es_version                     = '1',
	.rom_max_cal_size                       = REAR_GC08A3_MAX_CAL_SIZE,
	.rom_header_cal_data_start_addr         = 0x00,
	.rom_header_main_module_info_start_addr = 0x3E,
	.rom_header_cal_map_ver_start_addr      = 0x60,
	.rom_header_project_name_start_addr     = 0x68,
	.rom_header_module_id_addr              = 0x86,
	.rom_header_main_sensor_id_addr         = 0x90,

	.rom_header_sub_module_info_start_addr  = -1,
	.rom_header_sub_sensor_id_addr          = -1,

	.rom_header_main_header_start_addr      = 0x00,
	.rom_header_main_header_end_addr        = 0x04,
	.rom_header_main_oem_start_addr         = 0x18,
	.rom_header_main_oem_end_addr           = 0x1C,
	.rom_header_main_awb_start_addr         = 0x20,
	.rom_header_main_awb_end_addr           = 0x24,
	.rom_header_main_shading_start_addr     = 0x28,
	.rom_header_main_shading_end_addr       = 0x2C,
	.rom_header_main_sensor_cal_start_addr  = -1,
	.rom_header_main_sensor_cal_end_addr    = -1,
	.rom_header_dual_cal_start_addr         = -1,
	.rom_header_dual_cal_end_addr           = -1,
	.rom_header_pdaf_cal_start_addr         = -1,
	.rom_header_pdaf_cal_end_addr           = -1,
	.rom_header_ois_cal_start_addr          = -1,
	.rom_header_ois_cal_end_addr            = -1,

	.rom_header_sub_oem_start_addr          = -1,
	.rom_header_sub_oem_end_addr            = -1,
	.rom_header_sub_awb_start_addr          = -1,
	.rom_header_sub_awb_end_addr            = -1,
	.rom_header_sub_shading_start_addr      = -1,
	.rom_header_sub_shading_end_addr        = -1,

	.rom_header_main_mtf_data_addr          = -1, // It must be in af area, not in factory area in cal data
	.rom_header_sub_mtf_data_addr           = -1, // It must be in af area, not in factory area in cal data

	.rom_header_checksum_addr               = 0x00CC,
	.rom_header_checksum_len                = REAR_GC08A3_HEADER_CHECKSUM_LEN,

	.rom_oem_af_inf_position_addr           = 0x016C,
	.rom_oem_af_macro_position_addr         = 0x0178,
	.rom_oem_module_info_start_addr         = -1,
	.rom_oem_checksum_addr                  = 0x01AC,
	.rom_oem_checksum_len                   = REAR_GC08A3_OEM_CHECKSUM_LEN,

	.rom_module_cal_data_start_addr         = -1,
	.rom_module_module_info_start_addr      = -1,
	.rom_module_checksum_addr               = -1,
	.rom_module_checksum_len                = -1,

	.rom_awb_cal_data_start_addr            = 0x01B0,
	.rom_awb_module_info_start_addr         = -1,
	.rom_awb_checksum_addr                  = 0x01CC,
	.rom_awb_checksum_len                   = REAR_GC08A3_AWB_CHECKSUM_LEN,

	.rom_shading_cal_data_start_addr        = 0x01D0,
	.rom_shading_module_info_start_addr     = -1,
	.rom_shading_checksum_addr              = 0x05CC,
	.rom_shading_checksum_len               = REAR_GC08A3_SHADING_CHECKSUM_LEN,

	.rom_sensor_cal_module_info_start_addr  = -1,
	.rom_sensor_cal_checksum_addr           = -1,
	.rom_sensor_cal_checksum_len            = -1,

	.rom_dual_module_info_start_addr        = -1,
	.rom_dual_checksum_addr                 = -1,
	.rom_dual_checksum_len                  = -1,

	.rom_pdaf_module_info_start_addr        = -1,
	.rom_pdaf_checksum_addr                 = -1,
	.rom_pdaf_checksum_len                  = -1,
	.rom_ois_checksum_addr                  = -1,
	.rom_ois_checksum_len                   = -1,

	.rom_sub_oem_af_inf_position_addr       = -1,
	.rom_sub_oem_af_macro_position_addr     = -1,
	.rom_sub_oem_module_info_start_addr     = -1,
	.rom_sub_oem_checksum_addr              = -1,
	.rom_sub_oem_checksum_len               = -1,

	.rom_sub_awb_module_info_start_addr     = -1,
	.rom_sub_awb_checksum_addr              = -1,
	.rom_sub_awb_checksum_len               = -1,

	.rom_sub_shading_module_info_start_addr = -1,
	.rom_sub_shading_checksum_addr          = -1,
	.rom_sub_shading_checksum_len           = -1,

	.rom_dual_cal_data2_start_addr          = -1,
	.rom_dual_cal_data2_size                = -1,
	.rom_dual_tilt_x_addr                   = -1,
	.rom_dual_tilt_y_addr                   = -1,
	.rom_dual_tilt_z_addr                   = -1,
	.rom_dual_tilt_sx_addr                  = -1,
	.rom_dual_tilt_sy_addr                  = -1,
	.rom_dual_tilt_range_addr               = -1,
	.rom_dual_tilt_max_err_addr             = -1,
	.rom_dual_tilt_avg_err_addr             = -1,
	.rom_dual_tilt_dll_version_addr         = -1,
	.rom_dual_tilt_dll_modelID_addr         = -1,
	.rom_dual_tilt_dll_modelID_size         = -1,
	.rom_dual_shift_x_addr                  = -1,
	.rom_dual_shift_y_addr                  = -1,

	.extend_cal_addr                        = NULL,

	.converted_cal_addr                     = &rear_gc08a3_converted_cal_addr,
	.rom_converted_max_cal_size             = REAR_GC08A3_CONVERTED_MAX_CAL_SIZE,

	.sensor_maker                           = "GALAXYCORE",  // SENSOR_MAKER NAME 
	.sensor_name                            = "GC08A3",
	.sub_sensor_maker                       = NULL,
	.sub_sensor_name                        = NULL,

	.bayerformat                            = -1,
};
