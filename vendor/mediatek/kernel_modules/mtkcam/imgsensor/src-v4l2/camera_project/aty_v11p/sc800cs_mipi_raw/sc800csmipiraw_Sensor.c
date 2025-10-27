// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 MediaTek Inc.
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 sc800csmipiraw_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#define pr_fmt(fmt) "[D/D]" fmt
#include "sc800csmipiraw_Sensor.h"
#ifdef CONFIG_CAMERA_ADAPTIVE_MIPI
#include "sc800cs_Sensor_adaptive_mipi.h"
#endif

static u16 get_gain2reg(u32 gain);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static int sc800cs_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int sc800cs_set_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void sc800cs_set_streaming_control(struct subdrv_ctx *ctx, bool enable);
static int sc800cs_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int sc800cs_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int sc800cs_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len);

#ifdef CONFIG_CAMERA_ADAPTIVE_MIPI
static int set_mipi_mode(enum MSDK_SCENARIO_ID_ENUM scenario_id, struct subdrv_ctx *ctx);
static int sc800cs_get_mipi_pixel_rate(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int update_mipi_info(enum SENSOR_SCENARIO_ID_ENUM scenario_id);
static void sc800cs_set_mipi_pixel_rate_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id, u32 *mipi_pixel_rate);
static u32 sc800cs_get_custom_mipi_pixel_rate(enum SENSOR_SCENARIO_ID_ENUM scenario_id);

static int adaptive_mipi_index = -1;
#endif

/* STRUCT */
static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_STREAMING_RESUME, sc800cs_set_streaming_resume},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, sc800cs_set_streaming_suspend},
	{SENSOR_FEATURE_SET_ESHUTTER, sc800cs_set_shutter},
	{SENSOR_FEATURE_SET_SHUTTER_FRAME_TIME, sc800cs_set_shutter_frame_length},
	{SENSOR_FEATURE_SET_GAIN, sc800cs_set_gain},
#ifdef CONFIG_CAMERA_ADAPTIVE_MIPI
	{SENSOR_FEATURE_GET_MIPI_PIXEL_RATE, sc800cs_get_mipi_pixel_rate},
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CC0, /* 3264 */
			.vsize = 0x0990, /* 2448 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CC0, /* 3264 */
			.vsize = 0x0990, /* 2448 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CC0, /* 3264 */
			.vsize = 0x072C, /* 1836 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CC0, /* 3264 */
			.vsize = 0x0990, /* 2448 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CC0, /* 3264 */
			.vsize = 0x0990, /* 2448 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0660, /* 1632 */
			.vsize = 0x04C8, /* 1224 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CC0, /* 3264 */
			.vsize = 0x0990, /* 2448 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CC0, /* 3264 */
			.vsize = 0x0990, /* 2448 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CC0, /* 3264 */
			.vsize = 0x0990, /* 2448 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0CC0, /* 3264 */
			.vsize = 0x0990, /* 2448 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	// frame_desc_prev
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = sc800cs_preview_setting,
		.mode_setting_len = ARRAY_SIZE(sc800cs_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000, /* 1440 Mbps 2Lane */
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_cap
	{
		.frame_desc = frame_desc_cap,
		.num_entries = ARRAY_SIZE(frame_desc_cap),
		.mode_setting_table = sc800cs_preview_setting,
		.mode_setting_len = ARRAY_SIZE(sc800cs_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000, /* 1440 Mbps 2Lane */
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_vid
	{
		.frame_desc = frame_desc_vid,
		.num_entries = ARRAY_SIZE(frame_desc_vid),
		.mode_setting_table = sc800cs_video_setting,
		.mode_setting_len = ARRAY_SIZE(sc800cs_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000, /* 1440 Mbps 2Lane*/
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 314,
			.w0_size = 3264,
			.h0_size = 1836,
			.scale_w = 3264,
			.scale_h = 1836,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 1836,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 1836,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_hs_vid - not used mode
	{
		.frame_desc = frame_desc_hs_vid,
		.num_entries = ARRAY_SIZE(frame_desc_hs_vid),
		.mode_setting_table = sc800cs_preview_setting,
		.mode_setting_len = ARRAY_SIZE(sc800cs_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000, /* 1440 Mbps 2Lane */
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_slim_vid - not used mode
	{
		.frame_desc = frame_desc_slim_vid,
		.num_entries = ARRAY_SIZE(frame_desc_slim_vid),
		.mode_setting_table = sc800cs_preview_setting,
		.mode_setting_len = ARRAY_SIZE(sc800cs_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000, /* 1440 Mbps 2Lane*/ 
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_cus1
	{
		.frame_desc = frame_desc_cus1,
		.num_entries = ARRAY_SIZE(frame_desc_cus1),
		.mode_setting_table = sc800cs_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(sc800cs_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 132000000,
		.linelength = 1720,
		.framelength = 1279,
		.max_framerate = 600,
		.mipi_pixel_rate = 288000000, /* 1440 Mbps 2Lane */
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 1632,
			.scale_h = 1224,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1632,
			.h1_size = 1224,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1632,
			.h2_tg_size = 1224,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000, // by IQ request, actual 2x2 binning
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 78, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_cus2 - not used mode
	{
		.frame_desc = frame_desc_cus2,
		.num_entries = ARRAY_SIZE(frame_desc_cus2),
		.mode_setting_table = sc800cs_preview_setting,
		.mode_setting_len = ARRAY_SIZE(sc800cs_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000, /* 1440 Mbps 2Lane*/ 
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_cus3 - not used mode
	{
		.frame_desc = frame_desc_cus3,
		.num_entries = ARRAY_SIZE(frame_desc_cus3),
		.mode_setting_table = sc800cs_preview_setting,
		.mode_setting_len = ARRAY_SIZE(sc800cs_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000, /* 1440 Mbps 2Lane*/ 
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_cus4 - not used mode
	{
		.frame_desc = frame_desc_cus4,
		.num_entries = ARRAY_SIZE(frame_desc_cus4),
		.mode_setting_table = sc800cs_preview_setting,
		.mode_setting_len = ARRAY_SIZE(sc800cs_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000, /* 1440 Mbps 2Lane*/ 
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
	// frame_desc_cus5 - not used mode
	{
		.frame_desc = frame_desc_cus5,
		.num_entries = ARRAY_SIZE(frame_desc_cus5),
		.mode_setting_table = sc800cs_preview_setting,
		.mode_setting_len = ARRAY_SIZE(sc800cs_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 132000000,
		.linelength = 1760,
		.framelength = 2500,
		.max_framerate = 300,
		.mipi_pixel_rate = 288000000, /* 1440 Mbps 2Lane*/ 
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 3280,
			.full_h = 2464,
			.x0_offset = 8,
			.y0_offset = 8,
			.w0_size = 3264,
			.h0_size = 2448,
			.scale_w = 3264,
			.scale_h = 2448,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 3264,
			.h1_size = 2448,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 3264,
			.h2_tg_size = 2448,
		},
		.pdaf_cap = FALSE,
		.imgsensor_pd_info = PARAM_UNDEFINED,
		.ae_binning_ratio = 1000,
		.fine_integ_line = PARAM_UNDEFINED,
		.delay_frame = 2,
		.csi_param = {
			.dphy_trail = 84, // Fill correct Ths-trail (in ns unit) value
		},
		.dpc_enabled = true,
	},
};

static struct subdrv_static_ctx static_ctx = {
	.sensor_id = SC800CS_SENSOR_ID,
	.reg_addr_sensor_id = {0x3107, 0x3108},
	.i2c_addr_table = {0x6c, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = PARAM_UNDEFINED,
	.eeprom_num = 0,
	.resolution = {3264, 2448},
	.mirror = IMAGE_NORMAL,
	.mclk = 19,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_CSI2,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_B,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 32,
	.ana_gain_type = 3,  // type not included in MTK doc
	.ana_gain_step = 1, // modify maybe step wrt non - continous 
	.ana_gain_table = sc800cs_ana_gain_table,
	.ana_gain_table_size = sizeof(sc800cs_ana_gain_table),
	.exposure_def = 0x3D0,
	.exposure_min = 3,
	.exposure_max = 0xFFFFE,
	.exposure_step = 1,
	.exposure_margin = 10,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = 16382,
	.dig_gain_step = 4,

	.frame_length_max = 0x80004,
	.ae_effective_frame = 2,
	.frame_time_delay_frame = 2,
	.start_exposure_offset = 5500000,

	.pdaf_type = PDAF_SUPPORT_NA,
	.hdr_type = HDR_SUPPORT_NA,
	.seamless_switch_support = FALSE,
	.temperature_support = FALSE,

	.g_temp = PARAM_UNDEFINED,
	.g_gain2reg = get_gain2reg,
	.s_gph = PARAM_UNDEFINED,
	.s_cali = PARAM_UNDEFINED,
	.reg_addr_stream = 0x0100,
	.reg_addr_mirror_flip = PARAM_UNDEFINED,
	.reg_addr_exposure = {{0x3e00, 0x3e01, 0x3e02},},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x3e09},},
	.reg_addr_dig_gain = {{PARAM_UNDEFINED, PARAM_UNDEFINED},},
	.reg_addr_frame_length ={0x326d, 0x320e, 0x320f},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.reg_addr_fast_mode = PARAM_UNDEFINED,
	.chk_s_off_end = 0,
	.init_setting_table = sc800cs_init_setting,
	.init_setting_len = ARRAY_SIZE(sc800cs_init_setting),
	.mode = mode_struct,
	.sensor_mode_num = ARRAY_SIZE(mode_struct),
	.list = feature_control_list,
	.list_len = ARRAY_SIZE(feature_control_list),
	.checksum_value = 0xd086e5a5,
};

static struct subdrv_ops ops = {
	.get_id = common_get_imgsensor_id,
	.init_ctx = init_ctx,
	.open = common_open,
	.get_info = common_get_info,
	.get_resolution = common_get_resolution,
	.control = common_control,
	.feature_control = common_feature_control,
	.close = common_close,
	.get_frame_desc = common_get_frame_desc,
	.get_temp = common_get_temp,
	.get_csi_param = common_get_csi_param,
	.vsync_notify = vsync_notify,
	.update_sof_cnt = common_update_sof_cnt,
};

static struct subdrv_pw_seq_entry pw_seq[] = {
	{HW_ID_RST, 0, 1},
	{HW_ID_AFVDD, 2800000, 1},
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD, 1, 1},
	{HW_ID_AVDD, 1, 0},
	{HW_ID_MCLK, 19, 1},
	{HW_ID_RST, 1, 1},
	{HW_ID_RST, 0, 1},
	{HW_ID_RST, 1, 4},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 4},
};

const struct subdrv_entry sc800cs_mipi_raw_entry = {
	.name = "sc800cs_mipi_raw",
	.id = SC800CS_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */
static u16 get_gain2reg(u32 gain)
{
	return gain * 16 / BASEGAIN - 1 * 16;
}

static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id)
{
	memcpy(&(ctx->s_ctx), &static_ctx, sizeof(struct subdrv_static_ctx));
	subdrv_ctx_init(ctx);
	ctx->i2c_client = i2c_client;
	ctx->i2c_write_id = i2c_write_id;
	return 0;
}

static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt)
{
	DRV_LOG(ctx, "sof_cnt(%u) ctx->ref_sof_cnt(%u) ctx->fast_mode_on(%d)",
		sof_cnt, ctx->ref_sof_cnt, ctx->fast_mode_on);
	return 0;
}

void sc800cs_sensor_wait_stream_off(struct subdrv_ctx *ctx)
{
	// To be done
	DRV_LOG(ctx, "needs to be implemented\n");
}

static void sc800cs_set_streaming_control(struct subdrv_ctx *ctx, bool enable)
{
	u64 stream_ctrl_delay_timing = 0;

	DRV_LOG_MUST(ctx, "E: stream[%s]\n", enable? "ON": "OFF");
	check_current_scenario_id_bound(ctx);
	if (ctx->s_ctx.aov_sensor_support && ctx->s_ctx.streaming_ctrl_imp) {
		if (ctx->s_ctx.s_streaming_control != NULL)
			ctx->s_ctx.s_streaming_control((void *) ctx, enable);
		else
			DRV_LOG_MUST(ctx,
				"please implement drive own streaming control!(sid:%u)\n",
				ctx->current_scenario_id);
		ctx->is_streaming = enable;
		DRV_LOG_MUST(ctx, "enable:%u\n", enable);
		return;
	}
	if (ctx->s_ctx.aov_sensor_support && ctx->s_ctx.mode[ctx->current_scenario_id].aov_mode) {
		DRV_LOG_MUST(ctx,
			"stream ctrl implement on scp side!(sid:%u)\n",
			ctx->current_scenario_id);
		ctx->is_streaming = enable;
		DRV_LOG_MUST(ctx, "enable:%u\n", enable);
		return;
	}

	if (enable) {
		set_dummy(ctx);
#ifdef CONFIG_CAMERA_ADAPTIVE_MIPI
		set_mipi_mode(ctx->current_scenario_id, ctx);
#endif
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x01);
		mDELAY(10);
		subdrv_i2c_wr_u8(ctx, 0x302d, 0x00);
		subdrv_i2c_wr_u8(ctx, 0x3691, 0x66);
		ctx->stream_ctrl_start_time = ktime_get_boottime_ns();
		subdrv_i2c_wr_u8(ctx, 0x3221, 0x66); //flip and mirror
	} else {
		ctx->stream_ctrl_end_time = ktime_get_boottime_ns();
		if (ctx->s_ctx.custom_stream_ctrl_delay &&
			ctx->stream_ctrl_start_time && ctx->stream_ctrl_end_time) {
			stream_ctrl_delay_timing =
				(ctx->stream_ctrl_end_time - ctx->stream_ctrl_start_time) / 1000000;
			DRV_LOG_MUST(ctx,
				"custom_stream_ctrl_delay/stream_ctrl_delay_timing:%llu/%llu\n",
				ctx->s_ctx.custom_stream_ctrl_delay,
				stream_ctrl_delay_timing);
			if (stream_ctrl_delay_timing < ctx->s_ctx.custom_stream_ctrl_delay)
				mDELAY(ctx->s_ctx.custom_stream_ctrl_delay - stream_ctrl_delay_timing);
		}
		subdrv_i2c_wr_u8(ctx, ctx->s_ctx.reg_addr_stream, 0x00);
		if (ctx->s_ctx.reg_addr_fast_mode && ctx->fast_mode_on) {
			ctx->fast_mode_on = FALSE;
			ctx->ref_sof_cnt = 0;
			DRV_LOG(ctx, "seamless_switch disabled.");
			set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_fast_mode, 0x00);
			commit_i2c_buffer(ctx);
		}
		memset(ctx->exposure, 0, sizeof(ctx->exposure));
		memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
		ctx->autoflicker_en = FALSE;
		ctx->extend_frame_length_en = 0;
		ctx->is_seamless = 0;
		if (ctx->s_ctx.chk_s_off_end)
			sc800cs_sensor_wait_stream_off(ctx); /*Wait streamoff sensor specific logic*/
		ctx->stream_ctrl_start_time = 0;
		ctx->stream_ctrl_end_time = 0;
	}
	ctx->sof_no = 0;
	ctx->is_streaming = enable;
	DRV_LOG_MUST(ctx, "X: stream[%s]\n", enable? "ON": "OFF");
}

static int sc800cs_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	
	if (*feature_data) {
		if (ctx->s_ctx.aov_sensor_support &&
			ctx->s_ctx.mode[ctx->current_scenario_id].aov_mode &&
			(ctx->s_ctx.mode[ctx->current_scenario_id].ae_ctrl_support !=
				IMGSENSOR_AE_CONTROL_SUPPORT_VIEWING_MODE))
			DRV_LOG_MUST(ctx,
				"AOV mode not support ae shutter control!\n");
		else {
			*(feature_data + 1) = 0;
			sc800cs_set_shutter_frame_length(ctx, para, len);
		}
	}
	sc800cs_set_streaming_control(ctx, TRUE);
	return 0;
}

static int sc800cs_set_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	sc800cs_set_streaming_control(ctx, FALSE);
	return 0;
}

static int sc800cs_set_shutter(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;

	if (ctx->s_ctx.aov_sensor_support &&
		ctx->s_ctx.mode[ctx->current_scenario_id].aov_mode &&
		(ctx->s_ctx.mode[ctx->current_scenario_id].ae_ctrl_support !=
			IMGSENSOR_AE_CONTROL_SUPPORT_VIEWING_MODE))
		DRV_LOG_MUST(ctx,
			"AOV mode not support ae shutter control!\n");
	else {
		*(feature_data + 1) = 0;
		return sc800cs_set_shutter_frame_length(ctx, para, len);
	}

	return ERROR_NONE;
}

static int sc800cs_set_shutter_frame_length(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *) para;
	u64 shutter = *feature_data;
	u32 frame_length = *(feature_data + 1);

	int fine_integ_line = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	ctx->frame_length = frame_length ? frame_length : ctx->min_frame_length;

	check_current_scenario_id_bound(ctx);
	/* check boundary of shutter */
	fine_integ_line = ctx->s_ctx.mode[ctx->current_scenario_id].fine_integ_line;

	shutter = FINE_INTEG_CONVERT(shutter, fine_integ_line);
	shutter = max_t(u64, shutter,
		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_shutter_range[0].min);
	shutter = min_t(u64, shutter,
		(u64)ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_shutter_range[0].max);
	
	shutter = (shutter > (2 * ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin)) ?
		(2 * ctx->s_ctx.frame_length_max - ctx->s_ctx.exposure_margin) : shutter;

	/* check boundary of framelength */
	ctx->frame_length = max(
		(u32)shutter + ctx->s_ctx.exposure_margin, ctx->min_frame_length);
	ctx->frame_length = min(ctx->frame_length, ctx->s_ctx.frame_length_max);
	ctx->exposure[0] = (u32)shutter;

	/* group hold start */
	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 1);

	/* enable auto extend */
	if (ctx->s_ctx.reg_addr_auto_extend)
		set_i2c_buffer(ctx, ctx->s_ctx.reg_addr_auto_extend, 0x01);
	// write framelength
	if (set_auto_flicker(ctx, 0) || frame_length || !ctx->s_ctx.reg_addr_auto_extend)
		write_frame_length(ctx, ctx->frame_length);
	else if (ctx->s_ctx.reg_addr_auto_extend)
		write_frame_length(ctx, ctx->min_frame_length);

	subdrv_i2c_wr_u8(ctx, 0x3E00, (shutter >>12) & 0xFF); // high
	subdrv_i2c_wr_u8(ctx, 0x3E01, (shutter >> 4) & 0xFF); // middle
	subdrv_i2c_wr_u8(ctx, 0x3E02, (shutter << 4) & 0xF0); //low
	
	if (!ctx->ae_ctrl_gph_en) {
		if (gph)
			ctx->s_ctx.s_gph((void *)ctx, 0);
		commit_i2c_buffer(ctx);
	}
	/* group hold end */

	DRV_LOG(ctx, "exp[0x%x], fll(input/output):%u/%u, flick_en:%d\n",
		ctx->exposure[0], frame_length, ctx->frame_length, ctx->autoflicker_en);
	return ERROR_NONE;
}

static int sc800cs_set_gain(struct subdrv_ctx *ctx, u8 *para, u32 *len) 
{
	u64 *feature_data = (u64 *) para;
	u64 gain = *feature_data;
	u8 ana_gain = 0;
	bool gph = !ctx->is_seamless && (ctx->s_ctx.s_gph != NULL);

	DRV_LOG(ctx, "gain[%llu]\n", gain);

	/* check boundary of gain */
	gain = max(gain,
		ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_ana_gain_range[0].min);
	gain = min(gain,
		ctx->s_ctx.mode[ctx->current_scenario_id].multi_exposure_ana_gain_range[0].max);

	/* restore gain */
	memset(ctx->ana_gain, 0, sizeof(ctx->ana_gain));
	ctx->ana_gain[0] = gain;

/* write gain */
	if ((gain >= BASEGAIN) && (gain < 2 * BASEGAIN)) {
		ana_gain = 0x00;
	} else if ((gain >= 2 * BASEGAIN) && (gain < 4 * BASEGAIN)) {
		ana_gain = 0x01;
	} else if ((gain >= 4 * BASEGAIN) && (gain < 8 * BASEGAIN)) {
		ana_gain = 0x03;
	} else if ((gain >= 8 * BASEGAIN) && (gain < 16 * BASEGAIN)) {
		ana_gain = 0x07;
	} else if ((gain >= 16 * BASEGAIN) && (gain < 32 * BASEGAIN)) {
		ana_gain = 0x0f;
	} else if (gain >= 32 * BASEGAIN) {
		ana_gain = 0x1f;
	}

	/* group hold start */
	if (gph && !ctx->ae_ctrl_gph_en)
		ctx->s_ctx.s_gph((void *)ctx, 1);

	DRV_LOG(ctx, "ana_gain[0x%x]\n", ana_gain);

	subdrv_i2c_wr_u8(ctx, 0x3e09, ana_gain);

	if (gph)
		ctx->s_ctx.s_gph((void *)ctx, 0);
	commit_i2c_buffer(ctx);

	return ERROR_NONE;
}

#ifdef CONFIG_CAMERA_ADAPTIVE_MIPI
static int sc800cs_get_mipi_pixel_rate(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)*feature_data;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	sc800cs_set_mipi_pixel_rate_by_scenario(ctx, scenario_id, (u32 *)(uintptr_t)(*(feature_data + 1)));

	return ERROR_NONE;
}

static u32 sc800cs_get_custom_mipi_pixel_rate(enum SENSOR_SCENARIO_ID_ENUM scenario_id)
{
	u32 mipi_pixel_rate = 0;

	mipi_pixel_rate = update_mipi_info(scenario_id);

	return mipi_pixel_rate;
}

static void sc800cs_set_mipi_pixel_rate_by_scenario(struct subdrv_ctx *ctx,
	enum SENSOR_SCENARIO_ID_ENUM scenario_id, u32 *mipi_pixel_rate)
{
	switch (scenario_id) {
	case SENSOR_SCENARIO_ID_NORMAL_PREVIEW:
	case SENSOR_SCENARIO_ID_NORMAL_CAPTURE:
	case SENSOR_SCENARIO_ID_NORMAL_VIDEO:
	case SENSOR_SCENARIO_ID_HIGHSPEED_VIDEO:
	case SENSOR_SCENARIO_ID_SLIM_VIDEO:
	case SENSOR_SCENARIO_ID_CUSTOM1:
	case SENSOR_SCENARIO_ID_CUSTOM2:
	case SENSOR_SCENARIO_ID_CUSTOM3:
	case SENSOR_SCENARIO_ID_CUSTOM4:
	case SENSOR_SCENARIO_ID_CUSTOM5:
		*mipi_pixel_rate = sc800cs_get_custom_mipi_pixel_rate(scenario_id);
		break;
	default:
		break;
	}

	if (*mipi_pixel_rate == 0)
		*mipi_pixel_rate = ctx->s_ctx.mode[scenario_id].mipi_pixel_rate;

	DRV_LOG_MUST(ctx, "mipi_pixel_rate changed %u->%u\n",
		ctx->s_ctx.mode[scenario_id].mipi_pixel_rate, *mipi_pixel_rate);
}

static int update_mipi_info(enum SENSOR_SCENARIO_ID_ENUM scenario_id)
{
	const struct cam_mipi_sensor_mode *cur_mipi_sensor_mode;
	int sensor_mode = scenario_id;

	pr_info("[%s], scenario=%d\n", __func__, sensor_mode);

	if (sensor_mode == SENSOR_SCENARIO_ID_CUSTOM1) {
		pr_info("skip adaptive mipi , mode invalid:%d\n", sensor_mode);
		return 0;
	}

	cur_mipi_sensor_mode = &sc800cs_adaptive_mipi_sensor_mode[sensor_mode];

	if (cur_mipi_sensor_mode->mipi_cell_ratings_size == 0 ||
		cur_mipi_sensor_mode->mipi_cell_ratings == NULL) {
		pr_info("skip select mipi channel\n");
		return 0;
	}

	adaptive_mipi_index = imgsensor_select_mipi_by_rf_cell_infos(cur_mipi_sensor_mode);
	pr_info("adaptive_mipi_index : %d\n", adaptive_mipi_index);
	if (adaptive_mipi_index != -1) {
		if (adaptive_mipi_index < cur_mipi_sensor_mode->sensor_setting_size) {
			pr_info("mipi_rate:%d\n", cur_mipi_sensor_mode->sensor_setting[adaptive_mipi_index].mipi_rate);
			return cur_mipi_sensor_mode->sensor_setting[adaptive_mipi_index].mipi_rate;
		}
	}
	pr_info("adaptive_mipi_index invalid: %d\n", adaptive_mipi_index);

	return 0;
};

static int set_mipi_mode(enum MSDK_SCENARIO_ID_ENUM scenario_id, struct subdrv_ctx *ctx)
{
	int ret = 0;
	const struct cam_mipi_sensor_mode *cur_mipi_sensor_mode;

	DRV_LOG(ctx, "[%s] scenario_id=%d\n", __func__, scenario_id);

	if (scenario_id == SENSOR_SCENARIO_ID_CUSTOM1) {
		DRV_LOG_MUST(ctx, "NOT supported sensor mode : %d", scenario_id);
		return -1;
	}

	cur_mipi_sensor_mode = &sc800cs_adaptive_mipi_sensor_mode[scenario_id];

	if (cur_mipi_sensor_mode->sensor_setting == NULL) {
		DRV_LOG_MUST(ctx, "no mipi setting for current sensor mode\n");
		return -1;
	}
	if (adaptive_mipi_index < CAM_SC800CS_SET_A_All_720_MHZ || adaptive_mipi_index >= CAM_SC800CS_SET_MAX_NUM) {
		DRV_LOG_MUST(ctx, "adaptive Mipi is set to the default values of 720 MHz(1440Mbps)");
		subdrv_i2c_wr_regs_u8(ctx,
			cur_mipi_sensor_mode->sensor_setting[0].setting,
			cur_mipi_sensor_mode->sensor_setting[0].setting_size);
	} else {
		DRV_LOG(ctx, "adaptive mipi settings: %d: mipi clock [%d]\n",
			 scenario_id, cur_mipi_sensor_mode->sensor_setting[adaptive_mipi_index].mipi_rate);

		subdrv_i2c_wr_regs_u8(ctx,
			cur_mipi_sensor_mode->sensor_setting[adaptive_mipi_index].setting,
			cur_mipi_sensor_mode->sensor_setting[adaptive_mipi_index].setting_size);
	}
	DRV_LOG(ctx, "[%s]-X\n", __func__);
	return ret;
}
#endif
