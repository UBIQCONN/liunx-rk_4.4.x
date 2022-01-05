/*
 * Copyright 2017-2019 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>

#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>

#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_platform.h>

#include <video/mipi_display.h>
#include <video/videomode.h>
#include <linux/regulator/consumer.h>


#include <drm/drm_atomic_helper.h>
//#include <drm/drm_probe_helper.h>


#define ENABLE_MULTI_LANE_DESKEW

#define DP_I2C_ADDR 					(0xB8 >> 1)   // 0x5c
#define MIPI_I2C_ADDR 					(0xD8 >> 1)   // 0x6c

// MIPI Packed Pixel Stream
#define  PKT_RGB_24b					(0x3E)
#define  PKT_RGB_30b					(0x0D)
#define  PKT_RGB_36b					(0x1D)
#define  PKT_RGB_18b_P					(0x1E)
#define  PKT_RGB_18b_L					(0x2E)
#define  PKT_YCbCr_16b					(0x2C)
#define  PKT_YCbCr_20b					(0x0C)
#define  PKT_YCbCr_24b					(0x1C)

// DPTX reg62[3:0]
#define B_DPTXIN_6Bpp					(0)
#define B_DPTXIN_8Bpp					(1)
#define B_DPTXIN_10Bpp					(2)
#define B_DPTXIN_12Bpp					(3)

#define B_LBR    						(1)
#define B_HBR    						(0)

#define DP_4_LANE 						(3)
#define DP_2_LANE 						(1)
#define DP_1_LANE 						(0)

#define B_SSC_ENABLE   					(1)
#define B_SSC_DISABLE					(0)

#define TRAINING_BITRATE				(B_HBR)
#define DPTX_SSC_SETTING				(B_SSC_ENABLE)	//(B_SSC_DISABLE)
#define HIGH_PCLK						(1)
#define MP_MCLK_INV						(1)
#define MP_CONTINUOUS_CLK				(1)
#define MP_LANE_DESKEW					(1)
#define MP_LANE_SWAP					(0)
#define MP_PN_SWAP						(0)

#define DP_PN_SWAP						(0)
#define DP_AUX_PN_SWAP					(0)
#define DP_LANE_SWAP					(0)	//(0) our convert board need to LANE SWAP for data lane
#define FRAME_RESYNC					(1)
#define VESA_MAP						(1) // '0' for JEIDA , '1' for VESA MAP

#define INT_MASK						(3)
#define MIPI_RECOVER					(1)

#define En_UFO							(1)
#define H_ReSync						(1)  
#define V_ReSync						(1)
#define TIMER_CNT						(0x1A)

// DPTX/MIPIRX 
#define  REG_VEND_ID_LOW				0x00
#define  REG_VEND_ID_HIGH				0x01
#define  REG_DEV_ID_LOW					0x02
#define  REG_DEV_ID_HIGH				0x03
#define  REG_REV_ID						0x04
#define  DEV_ID_HIGH					0x61
#define  DEV_ID_LOW51					0x51

// DPTX Register Bank0
#define  DP_REG_SW_RESET				0x05
#define  DP_SW_RESET_VEDIO_CLK_MASK		(0x1 << 0)
#define  DP_SW_RESET_REF_CLK_MASK		(0x1 << 2)
#define  DP_SW_RESET_AUX_CLK_MASK		(0x1 << 3)
#define  DP_SW_RESET_SDM_CLK_MASK		(0x1 << 5)

#define  DP_REGn_INTR_STATUS_CLEAR(n)	(0x06 + (n))	// 0x6/0x7/0x8
#define  DP_REGn_INTR_MASK(n)			(0x09 + (n))	// 0x9/0xa/0xb

#define  DP_REG_SYS_CONFIG				0x0C			
#define  DP_REGn_SYS_STATUS(n)			(0x0D + (n))	// 0xd/0xe

#define  DP_REG_SYS_DEBUG				0x0F

#define  DP_REGn_CLKBUF_CTRL(n)			(0x10 + (n))    // 0x11-0x14

#define  DP_REGn_LINK_TRAINING(n)		(0x16 + (n))	// 0x16-0x1f

#define  DP_REGn_AUX_CH(n)				(0x20 + (n))    // 0x20-0x2f

#define  DP_REG_HPD_VBG					0x3A

#define	 DP_REGn_PHY(n)					(0x58 + (n))    // 0x58-0x5f

#define  DP_REGn_VIDEO(n)				(0x60 + (n))    // 0x60/0x62

#define  DP_REGn_SYNC_DE_GEN(n)			(0x79 + (n))	// 0x79-0x94

#define  DP_REGn_PATTERN_GEN(n)			(0x95 + (n))    // 0x95-0x9f

#define  DP_REGn_INPUT_VID_TIMING(n)	(0xA0 + (n))    // 0xa0-0xb7

#define	 DP_REGn_PSR_CTRL(n)			(0xC4 + (n))	// 0xc4-0xc7

#define	 DP_REGn_MISC_CTRL(n)			(0xC8 + (n))    // 0xc8-0xcf

#define  DP_REGn_DATA_LINK(n)			(0xD0 + (n))    // 0xd0-0xe7

#define  DP_REG_PACKET					0xE8

#define  DP_REGn_AVI_INF_FRAME_PKT(n)	(0xE9 + (n))	// 0xe9-0xfc

#define  DP_REG_MIPI_PORT				0xFD
#define  DP_MIPI_MIPIRX_EN				(0x1 << 0)

// DPTX Register Bank1

// MIPIRX Register
#define  MIPI_REG_SW_RESET				0x05
#define  MIPI_SW_RESET_OCLK_MASK		(0x1 << 0)				
#define  MIPI_SW_RESET_MCLK_MASK		(0x1 << 1)			
#define  MIPI_SW_RESET_BCLK_MASK		(0x1 << 2)				
#define  MIPI_SW_RESET_NCLK_MASK		(0x1 << 3)
#define  MIPI_SW_RESET_MP_PCLK_MASK		(0x1 << 4)
#define  MIPI_SW_RESET_DP_PCLK_MASK		(0x1 << 5)

#define  MIPI_REGn_INTR_STATUS_CLEAR(n)	(0x06 + (n))	// 0x6/0x7/0x8
#define  MIPI_REGn_INTR_MASK(n)			(0x09 + (n))	// 0x9/0xa/0xb

#define  MIPI_REG_SYS_CONFIG			0x0C   

#define  MIPI_REG_SYS_STATUS			0x0D

#define  MIPI_REGn_CLKBUF_CTRL(n)		(0x10 + (n))	// 0x10-0x17
#define  MIPI_REG_CLKBUF_CTRL_GATE		MIPI_REGn_CLKBUF_CTRL(0)
#define  MIPI_REG_CLKBUF_CTRL_MCLK_SET	MIPI_REGn_CLKBUF_CTRL(1)
#define  MIPI_REG_CLKBUF_CTRL_PSR_SET	MIPI_REGn_CLKBUF_CTRL(2)

#define  MIPI_REGn_PHY_PROTOCOL(n)		(0x18 + (n))	// 0x18/0x19

#define  MIPI_REGn_LANE_PKT_DECODER(n)	(0x20 + (n))	// 0x20/0x27
#define  MIPI_REGn_PKT_DEC_SET			0x20
#define  MIPI_REG_PKT_DTYPE_VCNUM		0x27

#define  MIPI_REGn_UFO(n)				(0x28 + (n))	// 0x28-0x2f
#define  MIPI_VLC_EN_MASK				(0x1)
#define  MIPI_UFO_BLK_NUM_HIGH_MASK		(0x300)
#define  MIPI_UFO_BLK_NUM_LOW_MASK		(0x0FF)
#define  MIPI_REG_HDE_DELAY				0x2E

#define  MIPI_TIMING_USER_DEF			(0x1 << 7)

/* HFP */
#define  MIPI_REG_MIPI_HFP_LOW			0x30
#define  MIPI_HFP_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_HFP_HIGH			0x31
#define  MIPI_HFP_HIGH_MASK				0x0f00

/* HBP */
#define  MIPI_REG_MIPI_HBP_LOW			0x32
#define  MIPI_HBP_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_HBP_HIGH			0x33
#define  MIPI_HBP_HIGH_MASK				0x0f00

/* HDES */
#define  MIPI_REG_MIPI_HDES_LOW			0x34
#define  MIPI_HDES_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_HDES_HIGH		0x35
#define  MIPI_HDES_HIGH_MASK			0x0f00

/* HDEE */
#define  MIPI_REG_MIPI_HDEE_LOW			0x36
#define  MIPI_HDEE_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_HDEE_HIGH		0x37
#define  MIPI_HDEE_HIGH_MASK			0x0f00

/* HTotal*/
#define  MIPI_REG_MIPI_HTOTAL_LOW		0x38
#define  MIPI_HTOTAL_LOW_MASK			0xFF
#define  MIPI_REG_MIPI_HTOTAL_HIGH		0x39
#define  MIPI_HTOTAL_HIGH_MASK			0x0f00

/* VFP */
#define  MIPI_REG_MIPI_VFP_LOW			0x3A
#define  MIPI_VFP_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_VFP_HIGH			0x3B
#define  MIPI_VFP_HIGH_MASK				0x0f00

/* VBP */
#define  MIPI_REG_MIPI_VBP_LOW			0x3C
#define  MIPI_VBP_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_VBP_HIGH			0x3D
#define  MIPI_VBP_HIGH_MASK				0x0f00

/* VDES */
#define  MIPI_REG_MIPI_VDES_LOW			0x3E
#define  MIPI_VDES_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_VDES_HIGH		0x3F
#define  MIPI_VDES_HIGH_MASK			0x0f00

/* VDEE */
#define  MIPI_REG_MIPI_VDEE_LOW			0x40
#define  MIPI_VDEE_LOW_MASK				0xFF
#define  MIPI_REG_MIPI_VDEE_HIGH		0x41
#define  MIPI_VDEE_HIGH_MASK			0x0f00

/* VTotal */
#define  MIPI_REG_MIPI_VTOTAL_LOW		0x42
#define  MIPI_VTOTAL_LOW_MASK			0xFF
#define  MIPI_REG_MIPI_VTOTAL_HIGH		0x43
#define  MIPI_VTOTAL_HIGH_MASK			0x0f00

#define  MIPI_REG_HSVSPOL_HVRESYNC		0x4E
#define  HSYNC_POL_MASK					(0x1 << 0)
#define  VSYNC_POL_MASK					(0x1 << 1)
#define  HRESYNC_EN_MASK				(0x1 << 2)
#define  VRESYNC_EN_MASK				(0x1 << 3)
//#define  FIFO_RESET_EN_MASK			(0x1 << 4)
#define  FORCE_MCLK_ON					(0x1 << 4)
#define  FORCE_PPS_STABLE				(0x1 << 7)

#define  MIPI_REG_FIFOAUTO_RESET_DEBUG_PPS	0x4F

/* 0x50-0x63 readonly registers*/

#define  MIPI_REG_MCLK_HSHIFT_HAVG_PCLK_HSHIFT	0x70

#define  MIPI_REGn_AFE(n)				(0x80 + (n))   // 0x80-0x8c

#define	 MIPI_REGn_MISC(n)				(0x90 + (n))   // 0x90-0x9b
#define	 MIPI_REG_TM_STAMP_EN			MIPI_REGn_MISC(0x0)
#define  MIPI_REG_10US_TIME_INTPART		MIPI_REGn_MISC(0x1)


static const char * const tc358770_supply_names[] = {
	"vcc1v8",		
	"vcc1v2",	
};

#define IT6151_NUM_SUPPLIES ARRAY_SIZE(tc358770_supply_names)

enum it6151_type {
	ITE_6151_A1 = 0,
	ITE_6151_XX,
};

struct it6151 {
	struct i2c_client *dp_i2c;
	struct regmap *dp_regmap;

	struct i2c_client *mipi_i2c;
	struct regmap *mipi_regmap;

	struct drm_display_mode *curr_mode;

	struct platform_device  panel_ofdev;	/* for no mipi panle */
	struct drm_panel		*panel;			/* next point to a panel */
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct gpio_desc *reset_gpio;
	
	/* it6151 MIPI DSI RX related params */
	struct device_node *host_node;
	struct mipi_dsi_device *dsi;
	u8 num_dsi_lanes;
	u8 channel_id;						/* dsi channel 0/1 */

	// u8 phy_channel;

	// it6151 setting from dtb
	u8 n_hs_settle;
	u8 n_skip_stg;

	// dp tx related params
	u8 num_dp_lanes;

	enum it6151_type ite_type;
	struct regulator_bulk_data supplies[IT6151_NUM_SUPPLIES];
};


static inline struct it6151 *bridge_to_it6151(struct drm_bridge *bridge)
{
	return container_of(bridge, struct it6151, bridge);
}

static inline struct it6151 *connector_to_it6151(struct drm_connector *con)
{
	return container_of(con, struct it6151, connector);
}

static inline void it6151_update_bits(struct it6151 *it6151, unsigned int reg,
				    unsigned int mask, unsigned int val)
{
	regmap_update_bits(it6151->dp_regmap, reg, mask, val);
}


static void it6151_hw_reset(struct it6151 *it6151)
{
	if (!it6151->reset_gpio)
		return;

	gpiod_set_value_cansleep(it6151->reset_gpio, 0);

	msleep(30);  /*usleep_range(20000, 30000);*/

	gpiod_set_value_cansleep(it6151->reset_gpio, 1);
}


static void it6151_dp_sw_reset(struct it6151 *it6151,  u32 reset_bits)
{
	int ret;
	ret = regmap_write(it6151->dp_regmap, DP_REG_SW_RESET, reset_bits);
	WARN_ON(ret != 0);	
}

static void it6151_mipi_sw_reset(struct it6151 *it6151,  u32 reset_bits)
{
	int ret;
	ret = regmap_write(it6151->mipi_regmap, MIPI_REG_SW_RESET, reset_bits);
	WARN_ON(ret != 0);
}

static enum drm_connector_status
it6151_connector_detect(struct drm_connector *connector, bool force)
{
//	struct it6151 *it6151 = connector_to_it6151(connector);
	// todo check connect 
	/* connector_status_disconnected */
	return connector_status_connected;		
}

static const struct drm_connector_funcs it6151_connector_funcs = {
	.detect = it6151_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int it6151_configure_regulators(struct it6151 *it6151)
{
	unsigned int i;
	struct device *dev = &it6151->dp_i2c->dev;

	for (i = 0; i < IT6151_NUM_SUPPLIES; i++)
		it6151->supplies[i].supply = tc358770_supply_names[i];

	return devm_regulator_bulk_get(dev,
		IT6151_NUM_SUPPLIES,
		it6151->supplies);
}

static int it6151_get_modes(struct drm_connector *connector)
{
	struct it6151 *it6151 = connector_to_it6151(connector);

	if (WARN_ON(!it6151->panel))
		return -ENODEV;

	return drm_panel_get_modes(it6151->panel);
}

static enum drm_mode_status it6151_mode_valid(struct drm_connector *connector,
					      struct drm_display_mode *mode)
{
	// const struct it6151_minimode *m;
//	int i, vrefresh = drm_mode_vrefresh(mode);
	int vrefresh = drm_mode_vrefresh(mode);
	printk("it6151_mode_valid clk: %d mode->hdisplay: %d\n", mode->clock, mode->hdisplay);
	if (mode->clock > /*150000*//*148500*/165000)
		return MODE_CLOCK_HIGH;

	if ((mode->hdisplay > 1920) ||
		    (mode->vdisplay > /*1080*/1200) ||
		    (vrefresh > 60)) {
			return MODE_BAD;
	}
	// todo check color space

	return MODE_OK;
}

static const struct drm_connector_helper_funcs it6151_connector_helper_funcs = {
	.get_modes = it6151_get_modes,
	.mode_valid = it6151_mode_valid,
};

static void it6151_bridge_disable(struct drm_bridge *bridge)
{
	struct it6151 *it6151 = bridge_to_it6151(bridge);
//	struct regmap *regmap = it6151->dp_regmap;
	
	struct device *dev = &it6151->dp_i2c->dev;

	/* disable panel first */
	if (!it6151->panel) {
	/*	dev_err(dev, "panel %x\n", it6151->panel); */
		dev_err(dev, "panel disable\n");
		return;
	}
	
	if (drm_panel_disable(it6151->panel)) {
		dev_err(dev, "failed to disable panel\n");
		return;
	}

	if (!(IS_ERR(it6151->reset_gpio))) {
		gpiod_set_value_cansleep(it6151->reset_gpio, 0);
	}
	/* disable it6151 */
	// todo 
}

static int get_it6151_pixel_format(int mipi_dsi_pfmt)
{
	switch (mipi_dsi_pfmt)
	{
	case MIPI_DSI_FMT_RGB888:
		return PKT_RGB_24b;

	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return PKT_RGB_18b_P;
	case MIPI_DSI_FMT_RGB565:
	default:
		return -1;
	}
}

/* config mipi dsi dpi at enable */
static void it6151_config_dsi(struct it6151 *it6151)
{
	struct device *dev = &it6151->dp_i2c->dev;
	struct mipi_dsi_device *dsi_device = it6151->dsi;
	struct videomode vm;
	struct regmap *regmap = it6151->mipi_regmap;
	int ret = 0;
	u32 val;

#ifdef	ENABLE_MULTI_LANE_DESKEW
	/* default is Force continuous clock mode */
	regmap_write(regmap, MIPI_REGn_PHY_PROTOCOL(1) /*0x19*/,(MP_CONTINUOUS_CLK << 1) | MP_LANE_DESKEW);
#else
	regmap_write(regmap, MIPI_REGn_PHY_PROTOCOL(1) /*0x19*/,(MP_CONTINUOUS_CLK << 1));
#endif

	/* Color Formats, VESA + RGB888 */
	if ((ret = get_it6151_pixel_format(dsi_device->format)) < 0)
		val = PKT_RGB_24b;
	else
		val = ret;

	dev_info(dev, "DSI format is: %d\n", dsi_device->format);
	ret = regmap_write(regmap, MIPI_REG_PKT_DTYPE_VCNUM/*0x27*/, val);
	
	// timing
	drm_display_mode_to_videomode(it6151->curr_mode, &vm);

	dev_info(dev, "DSI Display Size: %d x %d %d %d %d\n", vm.hactive, vm.vactive, vm.hfront_porch, vm.hback_porch, vm.hsync_len);
#ifndef	ENABLE_MULTI_LANE_DESKEW  // user define timing
	/* hfp low */
	regmap_write(regmap, MIPI_REG_MIPI_HFP_LOW, vm.hfront_porch & MIPI_HFP_LOW_MASK);
	/* hfp high */
	regmap_write(regmap, MIPI_REG_MIPI_HFP_HIGH, (vm.hfront_porch & MIPI_HFP_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);
	
	/* hbp low */
	regmap_write(regmap, MIPI_REG_MIPI_HBP_LOW, vm.vback_porch & MIPI_HBP_LOW_MASK);
	/* hbp high */
	regmap_write(regmap, MIPI_REG_MIPI_HBP_HIGH, (vm.vback_porch & MIPI_HBP_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);

	/* htotal low */
	regmap_write(regmap, MIPI_REG_MIPI_HTOTAL_LOW, vm.hactive & MIPI_HTOTAL_LOW_MASK);
	/* htotal high */
	regmap_write(regmap, MIPI_REG_MIPI_HTOTAL_HIGH, (vm.hactive & MIPI_HTOTAL_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);

	/* vfp low */
	regmap_write(regmap, MIPI_REG_MIPI_VFP_LOW, vm.vfront_porch & MIPI_VFP_LOW_MASK);
	/* vfp high */
	regmap_write(regmap, MIPI_REG_MIPI_VFP_HIGH, (vm.vfront_porch & MIPI_VFP_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);

	/* vbp low */
	regmap_write(regmap, MIPI_REG_MIPI_VBP_LOW, vm.vback_porch & MIPI_VBP_LOW_MASK);
	/* vbp high */
	regmap_write(regmap, MIPI_REG_MIPI_VBP_HIGH, (vm.vback_porch & MIPI_VBP_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);

	/* vtotal low */
	regmap_write(regmap, MIPI_REG_MIPI_VTOTAL_LOW, vm.vactive & MIPI_VTOTAL_LOW_MASK);
	/* vtotal high */
	regmap_write(regmap, MIPI_REG_MIPI_VTOTAL_HIGH, (vm.vactive & MIPI_VTOTAL_HIGH_MASK) >> 8 | 
			MIPI_TIMING_USER_DEF);

	// todo check events mode
	/* hsw low/high vm.hsync_len */
	//regmap_write(regmap, 0x33, MIPI_TIMING_USER_DEF | vm.hsync_len >> 8);
	//regmap_write(regmap, 0x32, vm.hsync_len & 0xff);

	/* vsw low/high  vm.vsync_len */
	//regmap_write(regmap, MIPI_TIMING_USER_DEF | vm.vsync_len >> 8);
	//regmap_write(regmap, 0x3C, vm.vsync_len & 0xff);
#endif
	regmap_write(regmap, MIPI_REGn_UFO(0)/*0x28*/, ((vm.hactive / 4 - 1) >> 2) & 0xC0);
	// regmap_write(regmap, MIPI_REGn_UFO(0)/*0x28*/, ((vm.hactive / 4 - 1)  & MIPI_UFO_BLK_NUM_HIGH_MASK) >> 2) ;
	regmap_write(regmap, MIPI_REGn_UFO(1)/*0x29*/, (vm.hactive / 4 - 1) & MIPI_UFO_BLK_NUM_LOW_MASK);


	regmap_write(regmap, MIPI_REG_HDE_DELAY/*0x2e*/, 0x34);
	regmap_write(regmap, MIPI_REGn_UFO(7)/*0x2f*/, 0x01);   // Enable UFO re-sync

	val = 0x0;
	
	if (it6151->curr_mode->flags & DRM_MODE_FLAG_PHSYNC) {
		val |= HSYNC_POL_MASK;
	}
	if (it6151->curr_mode->flags & DRM_MODE_FLAG_PVSYNC) {
		val |= VSYNC_POL_MASK;
	}

	val |= H_ReSync << 2;
	val |= V_ReSync << 3;
	regmap_write(regmap, MIPI_REG_HSVSPOL_HVRESYNC /* 0x4e */, val);


	/* pclk/hsclk not multi 2, but div 2 */
	regmap_write(regmap, MIPI_REGn_AFE(0)/*0x80*/, (0 << 5) | 0x2); 
//	regmap_write(regmap, MIPI_REGn_AFE(4)/*0x84*/, 0xcf);   //UTI Add 
	/* Enable PPSM video stable change intr */
	regmap_write(regmap, MIPI_REGn_INTR_MASK(0)/*0x09*/, 0x1);
	/* Timer interrupt count number */
	regmap_write(regmap, MIPI_REGn_MISC(2) /*0x92 */, TIMER_CNT); //UTI Sub

	/* todo , eotp set */
}

/* config mipi rx */
static void it6151_config_rx(struct it6151 *it6151)
{
//	struct device *dev = &it6151->dp_i2c->dev;
	struct mipi_dsi_device *dsi = it6151->dsi;
	struct regmap *regmap = it6151->mipi_regmap;
//	int ret;
//	u32 val;

	if (dsi->lanes < 1 || dsi->lanes > 4)
		return;

	// mipirx reset
	it6151_mipi_sw_reset(it6151, 0x00);

	/* rx lanes */
	// regmap_write(regmap, MIPI_REG_SYS_CONFIG, dsi->lanes - 1 /* no swap*/);
	regmap_write(regmap, MIPI_REG_SYS_CONFIG, (MP_LANE_SWAP << 7) | (MP_PN_SWAP << 6)
			| ((dsi->lanes - 1) << 4) /*| UFO = disable */);

	regmap_write(regmap, MIPI_REG_CLKBUF_CTRL_MCLK_SET/*0x11*/, MP_MCLK_INV);

#if 0
	val = 0x0;
	if (it6151->curr_mode->flags & DRM_MODE_FLAG_PHSYNC) {
		val |= HSYNC_POL_MASK;
	}
	if (it6151->curr_mode->flags & DRM_MODE_FLAG_PVSYNC) {
		val |= VSYNC_POL_MASK;
	}
	val |= H_ReSync << 2;
	val |= V_ReSync << 3;
	regmap_write(regmap, MIPI_REG_HSVSPOL_HVRESYNC /* 0x4e */, val);
#endif

#if 0
	regmap_write(regmap, MIPI_REGn_AFE(0)/*0x80*/, (0 << 5)/* ucufo*/ | 0x2 /* mpclkdiv*/);
	// regmap_write(regmap, MIPI_REGn_AFE(4)/*0x84*/, 0x8f);

	// clean interrupt
	// regmap_write(regmap, MIPI_REGn_INTR_STATUS_CLEAR(0)/*0x6*/, 0xFF);
	// disable PPMS interrupt event
	regmap_write(regmap, MIPI_REGn_INTR_MASK(0)/*0x09*/, 0);
#endif
}

/* config dptx */
static void it6151_config_dp(struct it6151 *it6151)
{
//	struct device *dev = &it6151->dp_i2c->dev;
//	struct mipi_dsi_device *dsi = it6151->dsi;
	struct regmap *regmap = it6151->dp_regmap;
//	u8 val = 0;

	// dptx reset
	it6151_dp_sw_reset(it6151, 0x29);
	it6151_dp_sw_reset(it6151, 0x00);
	
	// Enable HPD_IRQ,HPD_CHG,VIDSTABLE
	regmap_write(regmap,  DP_REGn_INTR_MASK(0)/*0x09*/,INT_MASK);
	regmap_write(regmap, DP_REGn_INTR_MASK(1)/*0x0A*/,0x00);
	regmap_write(regmap, DP_REGn_INTR_MASK(2)/*0x0B*/,0x00);

	regmap_write(regmap, DP_REGn_PSR_CTRL(1)/*0xC5*/, 0xC1);
	regmap_write(regmap, DP_REGn_INPUT_VID_TIMING(21)/*0xB5*/, 0x00);
	regmap_write(regmap, DP_REGn_INPUT_VID_TIMING(23)/*0xB7*/, 0x80);
	
	// Panel Self Refresh
	regmap_write(regmap, DP_REGn_PSR_CTRL(0)/*0xC4*/, 0xF0);

	// Clear all interrupt
	regmap_write(regmap, DP_REGn_INTR_STATUS_CLEAR(0)/*0x06*/,0xFF);
	regmap_write(regmap, DP_REGn_INTR_STATUS_CLEAR(1)/*0x07*/,0xFF);
	regmap_write(regmap, DP_REGn_INTR_STATUS_CLEAR(2)/*0x08*/,0xFF);

	it6151_dp_sw_reset(it6151, 0x00);

	// i2c driving strength
	regmap_write(regmap, DP_REG_SYS_CONFIG/*0x0c*/, 0x08);
	// LS2V 
	regmap_write(regmap, DP_REGn_AUX_CH(1)/*0x21*/, 0x05);
	// HW HPD IRQ response enable/video blank gating disable
	regmap_write(regmap, DP_REG_HPD_VBG/*0x3a*/, 0x04);
	// CR value in EQ phase
	regmap_write(regmap, DP_REGn_PHY(7)/*0x5f*/, 0x06);
	// HDP IRQ Event timer
	regmap_write(regmap, DP_REGn_MISC_CTRL(1)/*0xc9*/, 0xf5);
	// AUX debug sel
	regmap_write(regmap, DP_REGn_MISC_CTRL(2)/*0xca*/, 0x4c);
	// I2C read 
	regmap_write(regmap, DP_REGn_MISC_CTRL(3)/*0xcb*/, 0x37);
	// 
	regmap_write(regmap, DP_REGn_MISC_CTRL(6)/*0xce*/, 0x80);
	// Enhanced Framing Mode / video fifo reset enable
	regmap_write(regmap, DP_REGn_DATA_LINK(3)/*0xd3*/, 0x03);
	// 
	regmap_write(regmap, DP_REGn_DATA_LINK(4)/*0xd4*/, 0x60);

	// AVI info frame enable / General info frame enable
	regmap_write(regmap, DP_REG_PACKET/*0xe8*/, 0x11);

	// non-Zero value for CEA setting, check the given input format.
	regmap_write(regmap, DP_REGn_AVI_INF_FRAME_PKT(3)/*0xec*/, 16);
	msleep(5);
	
	// #define  DP_REGn_AUX_CH(n)				(0x20 + (n))    // 0x20-0x2f
	// PC master / EDID no Segment write 
	regmap_write(regmap, DP_REGn_AUX_CH(3)/*0x23*/, 0x42);
	// PC request cmd offset
	regmap_write(regmap, DP_REGn_AUX_CH(4)/*0x24*/, 0x07);
	regmap_write(regmap, DP_REGn_AUX_CH(5)/*0x25*/, 0x01);
	regmap_write(regmap, DP_REGn_AUX_CH(6)/*0x26*/, 0x00);
	// PC request command write data byte1
	regmap_write(regmap, DP_REGn_AUX_CH(7)/*0x27*/, 0x10);
	// Native Aux Write
	regmap_write(regmap, DP_REGn_AUX_CH(11)/*0x2B*/, 0x05);
	// FIFO clear to Normal operation
	regmap_write(regmap, DP_REGn_AUX_CH(3)/*0x23*/,0x40);
	// no dp lane swap/no aux pn swap / Auto AXRCLK average / main link off to gate AXCLK
	regmap_write(regmap, DP_REGn_AUX_CH(2)/*0x22*/,(DP_AUX_PN_SWAP << 3)|(DP_PN_SWAP << 2) | 0x03);
	// DP lanes
	regmap_write(regmap, DP_REGn_LINK_TRAINING(0)/*0x16*/, (DPTX_SSC_SETTING << 4) | 
		 (DP_LANE_SWAP << 3) | ((it6151->num_dp_lanes -1) << 1) | TRAINING_BITRATE);
	
	// select bank1 
	regmap_write(regmap, DP_REG_SYS_DEBUG/*0x0f*/, 0x01);

	regmap_write(regmap, 0x76,0xa7);
	regmap_write(regmap, 0x77,0xaf);
	regmap_write(regmap, 0x7e,0x8f);
	regmap_write(regmap, 0x7f,0x07);

	regmap_write(regmap, 0x80,0xef);
	regmap_write(regmap, 0x81,0x5f);
	regmap_write(regmap, 0x82,0xef);
	regmap_write(regmap, 0x83,0x07);
	regmap_write(regmap, 0x88,0x38);
	regmap_write(regmap, 0x89,0x1f);
	regmap_write(regmap, 0x8a,0x48);

	regmap_write(regmap, 0x0f,0x00);
	regmap_write(regmap, 0x5c,0xf3);
	regmap_write(regmap, 0x17,0x04);
	regmap_write(regmap, 0x17,0x01);

	msleep(50);
}

/* start it6151 */
static void it6151_start_display(struct it6151 *it6151)
{
//	struct device *dev = &it6151->dp_i2c->dev;
//	struct mipi_dsi_device *dsi = it6151->dsi;
	
	// internal register, todo move to start
}

static int it6151_init(struct it6151 *it6151);
static void it6151_bridge_enable(struct drm_bridge *bridge)
{
	struct it6151 *it6151 = bridge_to_it6151(bridge);
	struct device *dev = &it6151->dp_i2c->dev;
	
//	unsigned long timeout;
//	unsigned int status;
//	bool is_stable = false;
//	int i;

	it6151_config_rx(it6151);

	it6151_config_dsi(it6151);

	it6151_config_dp(it6151);

	if (it6151->panel && drm_panel_prepare(it6151->panel)) {
		dev_err(dev, "Failed to setup panel\n");
		// goto prepare_err;
		return;
	}

	it6151_start_display(it6151);

	if (it6151->panel && drm_panel_enable(it6151->panel)) {
		dev_err(dev, "Failed to enable panel\n");
		drm_panel_unprepare(it6151->panel);
		return;
		// goto enable_err;
	}
}

static void tc_bridge_mode_set(struct drm_bridge *bridge,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adj)
{
	struct it6151 *it6151 = bridge_to_it6151(bridge);

	// check adj->clock for 6151

	it6151->curr_mode = drm_mode_duplicate(bridge->dev, adj);
}

int it6151_parse_dt(struct device_node *np, struct it6151 *it6151)
{
	struct device *dev = &it6151->dp_i2c->dev;
	u32 num_lanes = 0, channel_id = 0;

	of_property_read_u32(np, "ite,dsi-channel", &channel_id);
	of_property_read_u32(np, "ite,dsi-lanes", &num_lanes);

	if (num_lanes < 1 || num_lanes > 4) {
		dev_err(dev, "Invalid dsi-lanes: %d\n", num_lanes);
		return -EINVAL;
	}

	if (channel_id > 3) {
		dev_err(dev, "Invalid dsi-channel: %d\n", channel_id);
		return -EINVAL;
	}

	it6151->num_dsi_lanes = num_lanes;
	it6151->channel_id = channel_id;

	// dp lanes
	of_property_read_u32(np, "ite,dp-lanes", &num_lanes);
	if (num_lanes < 1 || num_lanes > 4) {
		dev_err(dev, "Invalid dsi-lanes: %d\n", num_lanes);
		return -EINVAL;
	}
	it6151->num_dp_lanes = num_lanes;
	printk("it6151->num_dp_lanes %d\n", it6151->num_dp_lanes);

	it6151->host_node = of_graph_get_remote_node(np, 0, 0);
	if (!it6151->host_node) {
		printk("parent np %s %s\n", np->name, np->full_name);
		return -ENODEV;
	}

	of_node_put(it6151->host_node);

	of_property_read_u32(np, "ite,skip-stage", (u32 *)&it6151->n_skip_stg);
	of_property_read_u32(np, "ite,hs-settle;", (u32 *)&it6151->n_hs_settle);

	/* TODO: add another ite parameters */
	
	// lvds panel driver have been added panel_simple_init => platform_driver_register
	// add device to do probe 
	// this is not mipi dsi panel, it will not be find at host attach 

#if 0
	for_each_available_child_of_node(host->dev->of_node, node) {
		/* skip nodes without reg property */
		if (!of_find_property(node, "reg", NULL))
			continue;
		of_mipi_dsi_device_add(host, node);
	}
#endif

	// this is not mipi dsi panel, it will not be found at host attach
	// it6151->panel = of_drm_find_panel(/*dev.of_node*/np);

	return 0;
}

#if 0
/* attach ite612x as a mipi dsi device to mipi dsi host */
static int it6151_attach_dsi(struct it6151 *it6151)
{
	struct device *dev = &it6151->dp_i2c->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret = 0;
	const struct mipi_dsi_device_info info = { .type = "ite6151",
						   .channel = it6151->channel_id,
						   .node = NULL,
						 };

	host = of_find_mipi_dsi_host_by_node(it6151->host_node);
	if (!host) {
		dev_err(dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		dev_err(dev, "failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_device;
	}

	it6151->dsi = dsi;

	dsi->lanes = it6151->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE | 
			  /* MIPI_DSI_MODE_EOT_PACKET |*/ MIPI_DSI_MODE_VIDEO_HSE;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(dev, "failed to attach dsi to host\n");
		goto err_dsi_attach;
	}

	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_device:
	return ret;
}
#endif


void it6151_detach_dsi(struct it6151 *it6151)
{
	mipi_dsi_detach(it6151->dsi);
	mipi_dsi_device_unregister(it6151->dsi);
}

static int it6151_bridge_attach(struct drm_bridge *bridge)
{
	struct it6151 *it6151 = bridge_to_it6151(bridge);
	struct drm_device *drm = bridge->dev;
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	const struct mipi_dsi_device_info info = { .type = "ite6151",
						   .channel = it6151->channel_id,
						   .node = NULL,
						 };
	int ret;

	ret = drm_connector_init(drm, &it6151->connector,
				 &it6151_connector_funcs,
				 DRM_MODE_CONNECTOR_eDP);

	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&it6151->connector,
				 &it6151_connector_helper_funcs);
/*	drm_connector_attach_encoder(&it6151->connector, bridge->encoder); */
	drm_mode_connector_attach_encoder(&it6151->connector,
							bridge->encoder);


	host = of_find_mipi_dsi_host_by_node(it6151->host_node);
	if (!host) {
		DRM_ERROR("failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_ERROR("failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_device;
	}

	it6151->dsi = dsi;

	dsi->lanes = it6151->num_dsi_lanes;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE | 
			  /* MIPI_DSI_MODE_EOT_PACKET |*/ MIPI_DSI_MODE_VIDEO_HSE;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_ERROR("failed to attach dsi to host\n");
		goto err_dsi_attach;
	}

	ret = drm_panel_attach(it6151->panel, &it6151->connector); 
	if (ret) {
		dev_err(&it6151->dp_i2c->dev, "Failed to attach panel: %d\n", ret);
		drm_connector_cleanup(&it6151->connector);
		return ret;
	}

	return 0;
err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_device:
	drm_connector_cleanup(&it6151->connector);
	return ret;
}

static void it6151_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct it6151 *it6151 = bridge_to_it6151(bridge);

        if (it6151->reset_gpio){
		gpiod_set_value_cansleep(it6151->reset_gpio, 1);
		usleep_range(10000, 20000);
	}	

	drm_panel_prepare(it6151->panel);
        return ;
}

static void it6151_bridge_post_disable(struct drm_bridge *bridge)
{
//	struct it6151 *it6151 = bridge->driver_private;

//	drm_panel_unprepare(it6151->panel);
/*
	if (!(IS_ERR(it6151->reset_gpio))) {
		gpiod_set_value_cansleep(it6151->reset_gpio, 0);
	}
*/	
}

static bool it6151_bridge_mode_fixup(struct drm_bridge *bridge,
				 const struct drm_display_mode *mode,
				 struct drm_display_mode *adj)
{
	/* Fixup sync polarities, both hsync and vsync are active low */
/*	
	adj->flags = mode->flags;
	adj->flags |= (DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
	adj->flags &= ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);
*/
	return true;
}

static const struct drm_bridge_funcs it6151_bridge_funcs = {
	.attach = it6151_bridge_attach,
	.pre_enable = it6151_bridge_pre_enable,
	.mode_set = tc_bridge_mode_set,
	.post_disable = it6151_bridge_post_disable,
	.disable = it6151_bridge_disable,
	.enable = it6151_bridge_enable,
	.mode_fixup = it6151_bridge_mode_fixup,
};

static int it6151_init(struct it6151 *it6151)
{
//	struct mipi_dsi_device *dsi = it6151->dsi;

	// u8 test =0;

	/* 0x4 */
	it6151_dp_sw_reset(it6151, 0x04);
	regmap_write(it6151->dp_regmap, DP_REG_MIPI_PORT, (MIPI_I2C_ADDR << 1) | DP_MIPI_MIPIRX_EN);
	// it6151_mipi_sw_reset(it6151, 0x0);

	//regmap_read(it6151->dp_regmap, DP_REG_MIPI_PORT, &test);
	
	// set hs /stg
	/* 0x18
	regmap_write(it6151->regmap, MIPI_REGn_PHY_PROTOCOL(0),
			it6151->n_skip_stg << 4 | it6151->n_hs_settle);
	*/

	return 0;
}

/* check 6121/22 */
static int it6151_check_chipid(struct it6151 *it6151)
{
	struct device *dev = &it6151->dp_i2c->dev;
	struct regmap *regmap;
	u32 devid_low, devid_high;
	u32 venid_low, venid_high;
	u32 rev_id;
	int ret = 0;
	
	regmap = it6151->dp_regmap;	

	ret = regmap_read(regmap, REG_DEV_ID_LOW, &devid_low);
	if (ret) {
		dev_err(dev, "regmap_read REG_DEV_ID_LOW failed %d\n", ret);
		return ret;
	}

	ret = regmap_read(regmap, REG_DEV_ID_HIGH, &devid_high);
	if (ret) {
		dev_err(dev, "regmap_read REG_DEV_ID_HIGH failed %d\n", ret);
		return ret;
	}

	if (devid_high != DEV_ID_HIGH &&
			devid_low != DEV_ID_LOW51) {
			dev_err(dev,
				"Invalid ite6151 device id 0x%2x%2x (expect 0x6151)\n",
				devid_high, devid_low);
			return -EINVAL;
	}

	// 0x04 = 0xA1
	it6151->ite_type = ITE_6151_A1;
	ret = regmap_read(regmap, REG_VEND_ID_LOW, &venid_low);
	ret = regmap_read(regmap, REG_VEND_ID_HIGH, &venid_high);
	ret = regmap_read(regmap, REG_REV_ID, &rev_id);

	dev_info(dev, "ite6151 device id 0x%2x%2x\n", devid_high, devid_low);
	dev_info(dev, "ite6151 vendor id 0x%2x%2x\n", venid_high, venid_low);
	dev_info(dev, "ite6151 REV id 0x%2x\n", rev_id);
	return ret;
}

#if 0
static const struct regmap_range it6151_volatile_ranges[] = {
	{ .range_min = 0, .range_max = 0xff },
};

static const struct regmap_access_table it6151_volatile_table = {
	.yes_ranges = it6151_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(it6151_volatile_ranges),
};
#else
static bool it6151_readable_register(struct device *dev, unsigned int reg)
{
	if (reg <= 0xff) {
		return true;
	}

	return false;
}

static bool it6151_writeable_register(struct device *dev, unsigned int reg)
{
	if (reg <= 0xff) {
		return true;
	}

	return false;
}
#endif

static const struct regmap_config it6151_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	// .volatile_table = &it6151_volatile_table,
	.max_register = 0xff,
	.writeable_reg = it6151_writeable_register,
	.readable_reg =  it6151_readable_register,

	.cache_type = REGCACHE_NONE,
};

static int it6151_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct it6151 *it6151;
	int ret;

	it6151 = devm_kzalloc(dev, sizeof(*it6151), GFP_KERNEL);
	if (!it6151)
		return -ENOMEM;

	// it6151->split_mode = of_property_read_bool(np, "split-mode");

	it6151->dp_i2c = client;

	it6151->mipi_i2c = i2c_new_dummy(client->adapter,
						MIPI_I2C_ADDR);
	if (!it6151->dp_i2c) {
		ret = -ENODEV;
		goto unregister_i2c;
	}

	it6151->dp_regmap = devm_regmap_init_i2c(client,
						&it6151_regmap_config);
	if (IS_ERR(it6151->dp_regmap)) {
		ret = PTR_ERR(it6151->dp_regmap);
		goto unregister_i2c;
	}

	it6151->mipi_regmap = devm_regmap_init_i2c(it6151->mipi_i2c,
						&it6151_regmap_config);
	if (IS_ERR(it6151->mipi_regmap)) {
		ret = PTR_ERR(it6151->mipi_regmap);
		goto unregister_i2c;
	}

	/* port@1 is the output port */
	ret = drm_of_find_panel_or_bridge(dev->of_node, 1, 0, &it6151->panel, NULL);
	if (ret && ret != -ENODEV)
		return ret;

	dev_set_drvdata(&client->dev, it6151);

	it6151_configure_regulators(it6151);

	ret = regulator_bulk_enable(IT6151_NUM_SUPPLIES, it6151->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
	}

	it6151->reset_gpio = devm_gpiod_get_optional(dev, "reset",
							GPIOD_ASIS);
	if (IS_ERR(it6151->reset_gpio)) {
		ret = PTR_ERR(it6151->reset_gpio);

		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get reset gpio: %d\n", ret);

		goto unregister_i2c;
	}
	it6151_hw_reset(it6151);

	ret = it6151_parse_dt(dev->of_node, it6151);
	if (ret) {
		goto unregister_i2c;
	}

	ret = it6151_check_chipid(it6151);		// check ite6151 dev id
	if (ret)
		goto unregister_i2c;

	it6151_init(it6151);
	//msleep(100);

	it6151->bridge.driver_private = it6151;
	it6151->bridge.funcs = &it6151_bridge_funcs;
	it6151->bridge.of_node = np;
	drm_bridge_add(&it6151->bridge);

	i2c_set_clientdata(client, it6151);

	return ret;

unregister_i2c:
	if (ret == -EPROBE_DEFER)
		return ret;

	return ret;
}

static int it6151_remove(struct i2c_client *client)
{
	struct it6151 *it6151 = i2c_get_clientdata(client);

	if (!it6151)
		return -EINVAL;

	of_node_put(it6151->host_node);

	it6151_detach_dsi(it6151);

	drm_bridge_remove(&it6151->bridge);

	return 0;
}

static const struct of_device_id it6151_dt_ids[] = {
	{ .compatible = "ite,it6151" },
	{ }
};
MODULE_DEVICE_TABLE(of, it6151_dt_ids);

static const struct i2c_device_id it6151_i2c_ids[] = {
	{ "it6151", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, it6151_i2c_ids);

static struct i2c_driver it6151_driver = {
	.probe = it6151_probe,
	.remove = it6151_remove,
	.driver = {
		.name = "it6151",
		.of_match_table = it6151_dt_ids,
	},
	.id_table = it6151_i2c_ids,
};
module_i2c_driver(it6151_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("ITE Inc. DSI to eDP bridge");
MODULE_LICENSE("GPL");

