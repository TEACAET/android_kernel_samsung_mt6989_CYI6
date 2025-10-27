// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2025 MediaTek Inc.
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 gc05a2mipiraw_Sensor.c
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
#include "gc05a2mipiraw_Sensor.h"
#ifdef CONFIG_CAMERA_ADAPTIVE_MIPI
#include "gc05a2_Sensor_adaptive_mipi.h"
#endif

static u16 get_gain2reg(u32 gain);
static int init_ctx(struct subdrv_ctx *ctx,	struct i2c_client *i2c_client, u8 i2c_write_id);
static int vsync_notify(struct subdrv_ctx *ctx,	unsigned int sof_cnt);
static int gc05a2_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int gc05a2_set_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static void gc05a2_set_streaming_control(struct subdrv_ctx *ctx, bool enable);

#ifdef CONFIG_CAMERA_ADAPTIVE_MIPI
static int set_mipi_mode(enum MSDK_SCENARIO_ID_ENUM scenario_id, struct subdrv_ctx *ctx);
static int gc05a2_get_mipi_pixel_rate(struct subdrv_ctx *ctx, u8 *para, u32 *len);
static int update_mipi_info(enum SENSOR_SCENARIO_ID_ENUM scenario_id);
static void gc05a2_set_mipi_pixel_rate_by_scenario(struct subdrv_ctx *ctx,
		enum SENSOR_SCENARIO_ID_ENUM scenario_id, u32 *mipi_pixel_rate);
static u32 gc05a2_get_custom_mipi_pixel_rate(enum SENSOR_SCENARIO_ID_ENUM scenario_id);

static int adaptive_mipi_index = -1;
#endif

/* STRUCT */
static struct subdrv_feature_control feature_control_list[] = {
	{SENSOR_FEATURE_SET_STREAMING_RESUME, gc05a2_set_streaming_resume},
	{SENSOR_FEATURE_SET_STREAMING_SUSPEND, gc05a2_set_streaming_suspend},
#ifdef CONFIG_CAMERA_ADAPTIVE_MIPI
	{SENSOR_FEATURE_GET_MIPI_PIXEL_RATE, gc05a2_get_mipi_pixel_rate},
#endif
};

static struct mtk_mbus_frame_desc_entry frame_desc_prev[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0A10, /* 2576 */
			.vsize = 0x078C, /* 1932 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cap[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0A10, /* 2576 */
			.vsize = 0x078C, /* 1932 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0A00, /* 2560 */
			.vsize = 0x05A0, /* 1440 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_hs_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0A10, /* 2576 */
			.vsize = 0x078C, /* 1932 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_slim_vid[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0A10, /* 2576 */
			.vsize = 0x078C, /* 1932 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus1[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0510, /* 1296 */
			.vsize = 0x03CC, /* 972 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus2[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0A10, /* 2576 */
			.vsize = 0x078C, /* 1932 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus3[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0A10, /* 2576 */
			.vsize = 0x078C, /* 1932 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus4[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0A10, /* 2576 */
			.vsize = 0x078C, /* 1932 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};
static struct mtk_mbus_frame_desc_entry frame_desc_cus5[] = {
	{
		.bus.csi2 = {
			.channel = 0,
			.data_type = 0x2B,
			.hsize = 0x0A10, /* 2576 */
			.vsize = 0x078C, /* 1932 */
			.user_data_desc = VC_STAGGER_NE,
		},
	},
};

static struct subdrv_mode_struct mode_struct[] = {
	// frame_desc_prev
	{
		.frame_desc = frame_desc_prev,
		.num_entries = ARRAY_SIZE(frame_desc_prev),
		.mode_setting_table = gc05a2_preview_setting,
		.mode_setting_len = ARRAY_SIZE(gc05a2_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 227200000,
		.linelength = 3664,
		.framelength = 2032,
		.max_framerate = 300,
		.mipi_pixel_rate = 181760000, /* 908.8 Mbps 2Lane*/
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 2592,
			.full_h = 1944,
			.x0_offset = 8,
			.y0_offset = 6,
			.w0_size = 2576,
			.h0_size = 1932,
			.scale_w = 2576,
			.scale_h = 1932,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2576,
			.h1_size = 1932,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2576,
			.h2_tg_size = 1932,
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
		.mode_setting_table = gc05a2_preview_setting,
		.mode_setting_len = ARRAY_SIZE(gc05a2_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 227200000,
		.linelength = 3664,
		.framelength = 2032,
		.max_framerate = 300,
		.mipi_pixel_rate = 181760000, /* 908.8 Mbps 2Lane*/
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 2592,
			.full_h = 1944,
			.x0_offset = 8,
			.y0_offset = 6,
			.w0_size = 2576,
			.h0_size = 1932,
			.scale_w = 2576,
			.scale_h = 1932,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2576,
			.h1_size = 1932,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2576,
			.h2_tg_size = 1932,
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
		.mode_setting_table = gc05a2_video_setting,
		.mode_setting_len = ARRAY_SIZE(gc05a2_video_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 227200000,
		.linelength = 3664,
		.framelength = 2032,
		.max_framerate = 300,
		.mipi_pixel_rate = 181760000, /* 908.8 Mbps 2Lane*/	
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 2592,
			.full_h = 1944,
			.x0_offset = 16,
			.y0_offset = 252,
			.w0_size = 2560,
			.h0_size = 1440,
			.scale_w = 2560,
			.scale_h = 1440,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2560,
			.h1_size = 1440,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2560,
			.h2_tg_size = 1440,
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
		.mode_setting_table = gc05a2_preview_setting,
		.mode_setting_len = ARRAY_SIZE(gc05a2_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 227200000,
		.linelength = 3664,
		.framelength = 2032,
		.max_framerate = 300,
		.mipi_pixel_rate = 181760000, /* 908.8 Mbps 2Lane*/
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 2592,
			.full_h = 1944,
			.x0_offset = 8,
			.y0_offset = 6,
			.w0_size = 2576,
			.h0_size = 1932,
			.scale_w = 2576,
			.scale_h = 1932,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2576,
			.h1_size = 1932,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2576,
			.h2_tg_size = 1932,
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
		.mode_setting_table = gc05a2_preview_setting,
		.mode_setting_len = ARRAY_SIZE(gc05a2_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 227200000,
		.linelength = 3664,
		.framelength = 2032,
		.max_framerate = 300,
		.mipi_pixel_rate = 181760000, /* 908.8 Mbps 2Lane*/
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 2592,
			.full_h = 1944,
			.x0_offset = 8,
			.y0_offset = 6,
			.w0_size = 2576,
			.h0_size = 1932,
			.scale_w = 2576,
			.scale_h = 1932,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2576,
			.h1_size = 1932,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2576,
			.h2_tg_size = 1932,
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
		.mode_setting_table = gc05a2_custom1_setting,
		.mode_setting_len = ARRAY_SIZE(gc05a2_custom1_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 224000000,
		.linelength = 3616,
		.framelength = 1032,
		.max_framerate = 600,
		.mipi_pixel_rate = 89600000,
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 2592,
			.full_h = 1944,
			.x0_offset = 0,
			.y0_offset = 0,
			.w0_size = 2592,
			.h0_size = 1944,
			.scale_w = 1296,
			.scale_h = 972,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 1296,
			.h1_size = 972,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 1296,
			.h2_tg_size = 972,
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
		.mode_setting_table = gc05a2_preview_setting,
		.mode_setting_len = ARRAY_SIZE(gc05a2_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 227200000,
		.linelength = 3664,
		.framelength = 2032,
		.max_framerate = 300,
		.mipi_pixel_rate = 181760000, /* 908.8 Mbps 2Lane*/
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 2592,
			.full_h = 1944,
			.x0_offset = 8,
			.y0_offset = 6,
			.w0_size = 2576,
			.h0_size = 1932,
			.scale_w = 2576,
			.scale_h = 1932,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2576,
			.h1_size = 1932,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2576,
			.h2_tg_size = 1932,
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
		.mode_setting_table = gc05a2_preview_setting,
		.mode_setting_len = ARRAY_SIZE(gc05a2_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 227200000,
		.linelength = 3664,
		.framelength = 2032,
		.max_framerate = 300,
		.mipi_pixel_rate = 181760000, /* 908.8 Mbps 2Lane*/
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 2592,
			.full_h = 1944,
			.x0_offset = 8,
			.y0_offset = 6,
			.w0_size = 2576,
			.h0_size = 1932,
			.scale_w = 2576,
			.scale_h = 1932,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2576,
			.h1_size = 1932,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2576,
			.h2_tg_size = 1932,
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
		.mode_setting_table = gc05a2_preview_setting,
		.mode_setting_len = ARRAY_SIZE(gc05a2_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 227200000,
		.linelength = 3664,
		.framelength = 2032,
		.max_framerate = 300,
		.mipi_pixel_rate = 181760000, /* 908.8 Mbps 2Lane*/
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 2592,
			.full_h = 1944,
			.x0_offset = 8,
			.y0_offset = 6,
			.w0_size = 2576,
			.h0_size = 1932,
			.scale_w = 2576,
			.scale_h = 1932,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2576,
			.h1_size = 1932,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2576,
			.h2_tg_size = 1932,
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
		.mode_setting_table = gc05a2_preview_setting,
		.mode_setting_len = ARRAY_SIZE(gc05a2_preview_setting),
		.seamless_switch_group = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_table = PARAM_UNDEFINED,
		.seamless_switch_mode_setting_len = PARAM_UNDEFINED,
		.hdr_mode = HDR_NONE,
		.raw_cnt = 1,
		.exp_cnt = 1,
		.pclk = 227200000,
		.linelength = 3664,
		.framelength = 2032,
		.max_framerate = 300,
		.mipi_pixel_rate = 181760000, /* 908.8 Mbps 2Lane*/
		.readout_length = 0,
		.read_margin = 0,
		.framelength_step = 0,
		.coarse_integ_step = 0,
		.imgsensor_winsize_info = {
			.full_w = 2592,
			.full_h = 1944,
			.x0_offset = 8,
			.y0_offset = 6,
			.w0_size = 2576,
			.h0_size = 1932,
			.scale_w = 2576,
			.scale_h = 1932,
			.x1_offset = 0,
			.y1_offset = 0,
			.w1_size = 2576,
			.h1_size = 1932,
			.x2_tg_offset = 0,
			.y2_tg_offset = 0,
			.w2_tg_size = 2576,
			.h2_tg_size = 1932,
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
	.sensor_id = GC05A2_SENSOR_ID,
	.reg_addr_sensor_id = {0x03f0, 0x03f1},
	.i2c_addr_table = {0x6e, 0xFF},
	.i2c_burst_write_support = TRUE,
	.i2c_transfer_data_type = I2C_DT_ADDR_16_DATA_8,
	.eeprom_info = PARAM_UNDEFINED,
	.eeprom_num = 0,
	.resolution = {2576, 1932},
	.mirror = IMAGE_NORMAL,
	.mclk = 19,
	.isp_driving_current = ISP_DRIVING_4MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
	.mipi_sensor_type = MIPI_OPHY_CSI2,
	.mipi_lane_num = SENSOR_MIPI_2_LANE,
	.ob_pedestal = 0x40,

	.sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gr,
	.ana_gain_def = BASEGAIN * 4,
	.ana_gain_min = BASEGAIN * 1,
	.ana_gain_max = BASEGAIN * 16,
	.ana_gain_type = 3,  // type not included in MTK doc
	.ana_gain_step = 1,
	.ana_gain_table = gc05a2_ana_gain_table,
	.ana_gain_table_size = sizeof(gc05a2_ana_gain_table),
	.exposure_def = 0x3D0,
	.exposure_min = 4,
	.exposure_max = (0x7FFF - 16),
	.exposure_step = 1,
	.exposure_margin = 16,
	.dig_gain_min = BASE_DGAIN * 1,
	.dig_gain_max = 16382,
	.dig_gain_step = 4,

	.frame_length_max = 0xFFFF,
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
	.reg_addr_mirror_flip = PARAM_UNDEFINED, /* not used */
	.reg_addr_exposure = {{0x0202, 0x0203},},
	.long_exposure_support = FALSE,
	.reg_addr_exposure_lshift = PARAM_UNDEFINED,
	.reg_addr_ana_gain = {{0x0204, 0x0205},},
	.reg_addr_dig_gain = {{PARAM_UNDEFINED, PARAM_UNDEFINED},},
	.reg_addr_frame_length ={0x0340, 0x0341},
	.reg_addr_temp_en = PARAM_UNDEFINED,
	.reg_addr_temp_read = PARAM_UNDEFINED,
	.reg_addr_auto_extend = PARAM_UNDEFINED,
	.reg_addr_frame_count = PARAM_UNDEFINED,
	.reg_addr_fast_mode = PARAM_UNDEFINED,
	.chk_s_off_end = 0,
	.init_setting_table = gc05a2_init_setting,
	.init_setting_len = ARRAY_SIZE(gc05a2_init_setting),
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
	{HW_ID_DOVDD, 1800000, 1},
	{HW_ID_DVDD, 1200000, 1},
	{HW_ID_AVDD, 1, 1},
	{HW_ID_RST, 1, 1},
	{HW_ID_MCLK, 19, 1},
	{HW_ID_MCLK_DRIVING_CURRENT, 4, 4},
};

const struct subdrv_entry gc05a2_mipi_raw_entry = {
	.name = "gc05a2_mipi_raw",
	.id = GC05A2_SENSOR_ID,
	.pw_seq = pw_seq,
	.pw_seq_cnt = ARRAY_SIZE(pw_seq),
	.ops = &ops,
};

/* FUNCTION */
static u16 get_gain2reg(u32 gain)
{
	return (u16)gain;
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

void gc05a2_sensor_wait_stream_off(struct subdrv_ctx *ctx)
{
	// TODO
	DRV_LOG(ctx, "To be implemented \n");
}

static void gc05a2_set_streaming_control(struct subdrv_ctx *ctx, bool enable)
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
		ctx->stream_ctrl_start_time = ktime_get_boottime_ns();
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
			gc05a2_sensor_wait_stream_off(ctx); /*Wait streamoff sensor specific logic*/
		ctx->stream_ctrl_start_time = 0;
		ctx->stream_ctrl_end_time = 0;
	}
	ctx->sof_no = 0;
	ctx->is_streaming = enable;
	DRV_LOG_MUST(ctx, "X: stream[%s]\n", enable? "ON": "OFF");
}

static int gc05a2_set_streaming_resume(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	gc05a2_set_streaming_control(ctx, TRUE);
	return 0;
}

static int gc05a2_set_streaming_suspend(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	gc05a2_set_streaming_control(ctx, FALSE);
	return 0;
}

static u8 gc05a2_otp_read_byte(struct i2c_client *client, u16 addr)
{
	u8 read_val = 0;
	adaptor_i2c_wr_u8(client, client->addr, 0x0A69, (addr >> 8) & 0xFF);
	adaptor_i2c_wr_u8(client, client->addr, 0x0A6A, addr & 0xFF);
	adaptor_i2c_wr_u8(client, client->addr, 0x0A66, 0x20);
	adaptor_i2c_rd_u8(client, client->addr, 0x0A6C, &read_val);
	return read_val;
}

static int gc05a2_otp_init(struct i2c_client *client)
{
	return adaptor_i2c_wr_regs_u8(client, client->addr,
		gc05a2_otp_init_setting, ARRAY_SIZE(gc05a2_otp_init_setting));
}

static int gc05a2_acc_read_init(struct i2c_client *client)
{
	return adaptor_i2c_wr_regs_u8(client, client->addr,
		gc05a2_acc_read_init_setting, ARRAY_SIZE(gc05a2_acc_read_init_setting));
}

int gc05a2_read_otp_cal(struct i2c_client *client, unsigned char *data, unsigned int size)
{
	u32 i = 0;
	u8 otp_bank = 0;
	u32 read_addr = 0;

	pr_info("%s-E\n", __func__);

	if (data == NULL) {
		pr_err("%s, fail, data is NULL\n", __func__);
		return -1;
	}

	pr_info("%s, check i2c_write_addr: %#04x\n", __func__, client->addr);

	// OTP initial setting
	adaptor_i2c_wr_u8(client, client->addr, 0x031C, 0x60);
	adaptor_i2c_wr_u8(client, client->addr, 0x0315, 0x80);
	adaptor_i2c_wr_u8(client, client->addr, 0x0AF4, 0x01);

	gc05a2_otp_init(client);

	mDELAY(10);

	otp_bank = gc05a2_otp_read_byte(client, GC05A2_OTP_CHECK_BANK);
	pr_info("%s, check OTP bank:%#x, size: %d\n", __func__, otp_bank, size);

	switch (otp_bank) {
	case GC05A2_OTP_BANK1_MARK:
		read_addr = GC05A2_OTP_BANK1_START_ADDR;
		break;
	case GC05A2_OTP_BANK2_MARK:
		read_addr = GC05A2_OTP_BANK2_START_ADDR;
		break;
	case GC05A2_OTP_BANK3_MARK:
		read_addr = GC05A2_OTP_BANK3_START_ADDR;
		break;
	default:
		pr_info("%s, Select default bank1\n", __func__);
		read_addr = GC05A2_OTP_BANK1_START_ADDR;
		break;
	}

	/* Continuous byte reading (ACC read) */
	gc05a2_acc_read_init(client);

	mDELAY(10);

	adaptor_i2c_wr_u8(client, client->addr, 0x0A69, (read_addr >> 8) & 0xFF);
	adaptor_i2c_wr_u8(client, client->addr, 0x0A6A, read_addr & 0xFF);
	adaptor_i2c_wr_u8(client, client->addr, 0x0A66, 0x20);
	adaptor_i2c_wr_u8(client, client->addr, 0x0A66, 0x12);

	for (i = 0; i < size; i++) {
		adaptor_i2c_rd_u8(client, client->addr, 0x0A6C, &data[i]);
		pr_debug("%s, read data read_addr: %#x, data: %x\n", __func__, i, data[i]);
	}

	pr_info("%s-X\n", __func__);

	return (int)size;
}

#ifdef CONFIG_CAMERA_ADAPTIVE_MIPI
static int gc05a2_get_mipi_pixel_rate(struct subdrv_ctx *ctx, u8 *para, u32 *len)
{
	u64 *feature_data = (u64 *)para;
	enum SENSOR_SCENARIO_ID_ENUM scenario_id = (enum SENSOR_SCENARIO_ID_ENUM)*feature_data;

	if (scenario_id >= ctx->s_ctx.sensor_mode_num) {
		DRV_LOG(ctx, "invalid sid:%u, mode_num:%u\n",
			scenario_id, ctx->s_ctx.sensor_mode_num);
		scenario_id = SENSOR_SCENARIO_ID_NORMAL_PREVIEW;
	}

	gc05a2_set_mipi_pixel_rate_by_scenario(ctx, scenario_id, (u32 *)(uintptr_t)(*(feature_data + 1)));

	return ERROR_NONE;
}

static u32 gc05a2_get_custom_mipi_pixel_rate(enum SENSOR_SCENARIO_ID_ENUM scenario_id)
{
	u32 mipi_pixel_rate = 0;

	mipi_pixel_rate = update_mipi_info(scenario_id);

	return mipi_pixel_rate;
}

static void gc05a2_set_mipi_pixel_rate_by_scenario(struct subdrv_ctx *ctx,
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
		*mipi_pixel_rate = gc05a2_get_custom_mipi_pixel_rate(scenario_id);
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

	cur_mipi_sensor_mode = &gc05a2_adaptive_mipi_sensor_mode[sensor_mode];

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

	cur_mipi_sensor_mode = &gc05a2_adaptive_mipi_sensor_mode[scenario_id];

	if (cur_mipi_sensor_mode->sensor_setting == NULL) {
		DRV_LOG_MUST(ctx, "no mipi setting for current sensor mode\n");
		return -1;
	}
	if (adaptive_mipi_index < CAM_GC05A2_SET_A_All_454p4_MHZ || adaptive_mipi_index >= CAM_GC05A2_SET_MAX_NUM) {
		DRV_LOG_MUST(ctx, "adaptive Mipi is set to the default values of 454p4 MHz(908.8Mbps)");
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