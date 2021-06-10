// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2018, The Linux Foundation. All rights reserved.
 */

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

/* Registers */
#define DSI0_PPI_STARTPPI 		0x0104
#define DSI0_PPI_LPTXTIMECNT		0x0114
#define DSI0_PPI_LANEENABLE		0x0134
#define DSI0_PPI_TX_RX_TA		0x013C
#define DSI0_PPI_D0S_CLRSIPOCOUNT	0x0164
#define DSI0_PPI_D1S_CLRSIPOCOUNT	0x0168
#define DSI0_PPI_D2S_CLRSIPOCOUNT	0x016c
#define DSI0_PPI_D3S_CLRSIPOCOUNT	0x0170
#define DSI0_DSI_STARTDSI		0x0204
#define DSI0_DSI_LANEENABLE		0x0210
#define DSI1_PPI_STARTPPI 		0x1104
#define DSI1_PPI_LPTXTIMECNT		0x1114
#define DSI1_PPI_LANEENABLE		0x1134
#define DSI1_PPI_TX_RX_TA		0x113C
#define DSI1_PPI_D0S_CLRSIPOCOUNT	0x1164
#define DSI1_PPI_D1S_CLRSIPOCOUNT	0x1168
#define DSI1_PPI_D2S_CLRSIPOCOUNT	0x116c
#define DSI1_PPI_D3S_CLRSIPOCOUNT	0x1170
#define DSI1_DSI_STARTDSI		0x1204
#define DSI1_DSI_LANEENABLE		0x1210

/* Display Parallel Interface */
#define DPIPXLFMT		0x0440
#define VS_POL_ACTIVE_LOW		(1 << 10)
#define HS_POL_ACTIVE_LOW		(1 << 9)
#define DE_POL_ACTIVE_HIGH		(0 << 8)
#define SUB_CFG_TYPE_CONFIG1		(0 << 2) /* LSB aligned */
#define SUB_CFG_TYPE_CONFIG2		(1 << 2) /* Loosely Packed */
#define SUB_CFG_TYPE_CONFIG3		(2 << 2) /* LSB aligned 8-bit */
#define DPI_BPP_RGB888			(0 << 0)
#define DPI_BPP_RGB666			(1 << 0)
#define DPI_BPP_RGB565			(2 << 0)

/* Video Path */
#define VPCTRL0			0x0450
#define VSDELAY			GENMASK(31, 20)
#define OPXLFMT_RGB666			(0 << 8)
#define OPXLFMT_RGB888			(1 << 8)
#define FRMSYNC_DISABLED		(0 << 4) /* Video Timing Gen Disabled */
#define FRMSYNC_ENABLED			(1 << 4) /* Video Timing Gen Enabled */
#define MSF_DISABLED			(0 << 0) /* Magic Square FRC disabled */
#define MSF_ENABLED			(1 << 0) /* Magic Square FRC enabled */
#define HTIM01			0x0454
#define HPW			GENMASK(8, 0)
#define HBPR			GENMASK(24, 16)
#define HTIM02			0x0458
#define HDISPR			GENMASK(10, 0)
#define HFPR			GENMASK(24, 16)
#define VTIM01			0x045c
#define VSPR			GENMASK(7, 0)
#define VBPR			GENMASK(23, 16)
#define VTIM02			0x0460
#define VFPR			GENMASK(23, 16)
#define VDISPR			GENMASK(10, 0)
#define VFUEN0			0x0464
#define VFUEN				BIT(0)   /* Video Frame Timing Upload */


/* Combiner Layer Registers */
#define CMBCTRL			0x0480
#define LRSIZE			0x0484
#define RMPXL			0x048C


/* System */
#define TC_IDREG		0x0500
#define SYSSTAT			0x0508
#define SYSCTRL			0x0510
#define DP0_AUDSRC_NO_INPUT		(0 << 3)
#define DP0_AUDSRC_I2S_RX		(1 << 3)
#define DP0_VIDSRC_NO_INPUT		(0 << 0)
#define DP0_VIDSRC_DSI_RX		(1 << 0)
#define DP0_VIDSRC_DPI_RX		(2 << 0)
#define DP0_VIDSRC_COLOR_BAR		(3 << 0)
#define SYSRSTENB		0x050c
#define ENBI2C				(1 << 0)
#define ENBLCD0				(1 << 2)
#define ENBBM				(1 << 3)
#define ENBDSIRX			(1 << 4)
#define ENBREG				(1 << 5)
#define ENBHDCP				(1 << 8)
#define GPIOM			0x0540
#define GPIOC			0x0544
#define GPIOO			0x0548
#define GPIOI			0x054c
#define INTCTL_G		0x0560
#define INTSTS_G		0x0564

#define INT_SYSERR		BIT(16)
#define INT_GPIO_H(x)		(1 << (x == 0 ? 2 : 10))
#define INT_GPIO_LC(x)		(1 << (x == 0 ? 3 : 11))

#define INT_GP0_LCNT		0x0584
#define INT_GP1_LCNT		0x0588

/* Control */
#define DP0CTL			0x0600
#define VID_MN_GEN			BIT(6)   /* Auto-generate M/N values */
#define EF_EN				BIT(5)   /* Enable Enhanced Framing */
#define VID_EN				BIT(1)   /* Video transmission enable */
#define DP_EN				BIT(0)   /* Enable DPTX function */

/* Clocks */
#define DP0_VIDMNGEN0		0x0610
#define DP0_VIDMNGEN1		0x0614
#define DP0_VMNGENSTATUS	0x0618

/* Main Channel */
#define DP0_SECSAMPLE		0x0640
#define DP0_VIDSYNCDELAY	0x0644
#define VID_SYNC_DLY		GENMASK(15, 0)
#define THRESH_DLY		GENMASK(31, 16)

#define DP0_TOTALVAL		0x0648
#define H_TOTAL			GENMASK(15, 0)
#define V_TOTAL			GENMASK(31, 16)
#define DP0_STARTVAL		0x064c
#define H_START			GENMASK(15, 0)
#define V_START			GENMASK(31, 16)
#define DP0_ACTIVEVAL		0x0650
#define H_ACT			GENMASK(15, 0)
#define V_ACT			GENMASK(31, 16)

#define DP0_SYNCVAL		0x0654
#define VS_WIDTH		GENMASK(30, 16)
#define HS_WIDTH		GENMASK(14, 0)
#define SYNCVAL_HS_POL_ACTIVE_LOW	(1 << 15)
#define SYNCVAL_VS_POL_ACTIVE_LOW	(1 << 31)
#define DP0_MISC		0x0658
#define TU_SIZE_RECOMMENDED		(63) /* LSCLK cycles per TU */
#define MAX_TU_SYMBOL		GENMASK(28, 23)
#define TU_SIZE			GENMASK(21, 16)
#define BPC_6				(0 << 5)
#define BPC_8				(1 << 5)

/* AUX channel */
#define DP0_AUXCFG0		0x0660
#define DP0_AUXCFG0_BSIZE	GENMASK(11, 8)
#define DP0_AUXCFG0_ADDR_ONLY	BIT(4)
#define DP0_AUXCFG1		0x0664
#define AUX_RX_FILTER_EN		BIT(16)

#define DP0_AUXADDR		0x0668
#define DP0_AUXWDATA(i)		(0x066c + (i) * 4)
#define DP0_AUXRDATA(i)		(0x067c + (i) * 4)
#define DP0_AUXSTATUS		0x068c
#define AUX_BYTES		GENMASK(15, 8)
#define AUX_STATUS		GENMASK(7, 4)
#define AUX_TIMEOUT		BIT(1)
#define AUX_BUSY		BIT(0)
#define DP0_AUXI2CADR		0x0698

/* Link Training */
#define DP0_SRCCTRL		0x06a0
#define DP0_SRCCTRL_SCRMBLDIS		BIT(13)
#define DP0_SRCCTRL_EN810B		BIT(12)
#define DP0_SRCCTRL_NOTP		(0 << 8)
#define DP0_SRCCTRL_TP1			(1 << 8)
#define DP0_SRCCTRL_TP2			(2 << 8)
#define DP0_SRCCTRL_LANESKEW		BIT(7)
#define DP0_SRCCTRL_SSCG		BIT(3)
#define DP0_SRCCTRL_LANES_1		(0 << 2)
#define DP0_SRCCTRL_LANES_2		(1 << 2)
#define DP0_SRCCTRL_BW27		(1 << 1)
#define DP0_SRCCTRL_BW162		(0 << 1)
#define DP0_SRCCTRL_AUTOCORRECT		BIT(0)
#define DP0_LTSTAT		0x06d0
#define LT_LOOPDONE			BIT(22)
#define LT_STATUS_MASK			(0x1f << 8)
#define LT_CHANNEL1_EQ_BITS		(DP_CHANNEL_EQ_BITS << 4)
#define LT_INTERLANE_ALIGN_DONE		BIT(3)
#define LT_CHANNEL0_EQ_BITS		(DP_CHANNEL_EQ_BITS)
#define DP0_SNKLTCHGREQ		0x06d4
#define DP0_LTLOOPCTRL		0x06d8
#define DP0_SNKLTCTRL		0x06e4

#define DP1_SRCCTRL		0x07a0

/* PHY */
#define DP_PHY_CTRL		0x0800
#define PHY_RDY				BIT(16)  /* PHY Main Channels Ready */
#define PHY_M0_RST			BIT(4)   /* Reset PHY0 Main Channel */
#define BGREN				BIT(3)   /* AUX PHY BGR Enable */
#define PHY_2LANE			BIT(2)   /* PHY Enable 2 lanes */
#define PHY_A0_EN			BIT(1)   /* PHY Aux Channel0 Enable */
#define PHY_M0_EN			BIT(0)   /* PHY Main Channel0 Enable */

/* PLL */
#define DP0_PLLCTRL		0x0900
#define DP1_PLLCTRL		0x0904	/* not defined in DS */
#define PXL_PLLCTRL		0x0908
#define PLLUPDATE			BIT(2)
#define PLLBYP				BIT(1)
#define PLLEN				BIT(0)
#define PXL_PLLPARAM		0x0914
#define IN_SEL_REFCLK			(2 << 14)
#define SYS_PLLPARAM		0x0918
#define REF_FREQ_38M4			(0 << 8) /* 38.4 MHz */
#define REF_FREQ_19M2			(1 << 8) /* 19.2 MHz */
#define REF_FREQ_26M			(2 << 8) /* 26 MHz */
#define REF_FREQ_13M			(3 << 8) /* 13 MHz */
#define SYSCLK_SEL_LSCLK		(0 << 4)
#define LSCLK_DIV_1			(0 << 0)
#define LSCLK_DIV_2			(1 << 0)

/* Test & Debug */
#define TSTCTL			0x0a00
#define COLOR_R			GENMASK(31, 24)
#define COLOR_G			GENMASK(23, 16)
#define COLOR_B			GENMASK(15, 8)
#define ENI2CFILTER		BIT(4)
#define COLOR_BAR_MODE		GENMASK(1, 0)
#define COLOR_BAR_MODE_BARS	2
#define PLL_DBG			0x0a04

static bool tc_test_pattern;
module_param_named(test, tc_test_pattern, bool, 0644);

struct tc_edp_link {
	struct drm_dp_link	base;
	u8			assr;
	bool			scrambler_dis;
	bool			spread;
};

struct tc_data {
	struct device			*dev;
	struct regmap			*regmap;
	struct drm_dp_aux		aux;
	struct drm_bridge		bridge;
	struct drm_connector		connector;
	struct device_node		*host_node;
	struct mipi_dsi_device		*dsi;
	struct clk			*refclk;
	struct drm_panel		*panel;
	/* link settings */
	struct tc_edp_link		link;
	/* current mode */
	struct drm_display_mode		mode;
	struct gpio_desc		*enable_gpio;
	u8				assr;
	u32				rev;

	        /* display edid */
        struct edid	                *edid;

};

static inline struct tc_data *aux_to_tc(struct drm_dp_aux *a)
{
	return container_of(a, struct tc_data, aux);
}

static inline struct tc_data *bridge_to_tc(struct drm_bridge *b)
{
	return container_of(b, struct tc_data, bridge);
}

static inline struct tc_data *connector_to_tc(struct drm_connector *c)
{
	return container_of(c, struct tc_data, connector);
}

static int tc358770_write(struct regmap *map, unsigned int reg, unsigned int val)
{
	
	return regmap_write(map, reg, val);
}	

static int tc358770_raw_write(struct regmap *map, unsigned int reg,
                     const void *val, size_t val_len)
{	
	unsigned char tmp[16];

	memcpy(tmp, val, val_len);	
	return regmap_raw_write(map, reg, val, val_len);
}


static inline int tc_poll_timeout(struct tc_data *tc, unsigned int addr,
				  unsigned int cond_mask,
				  unsigned int cond_value,
				  unsigned long sleep_us, u64 timeout_us)
{
	unsigned int val;

	return regmap_read_poll_timeout(tc->regmap, addr, val,
					(val & cond_mask) == cond_value,
					sleep_us, timeout_us);
}

static int tc_aux_wait_busy(struct tc_data *tc)
{
	return tc_poll_timeout(tc, DP0_AUXSTATUS, AUX_BUSY, 0, 100, 100000);
}

static int tc_aux_write_data(struct tc_data *tc, const void *data,
			     size_t size)
{
	u32 auxwdata[DP_AUX_MAX_PAYLOAD_BYTES / sizeof(u32)] = { 0 };
	int ret, count = ALIGN(size, sizeof(u32));

	memcpy(auxwdata, data, size);

	ret = tc358770_raw_write(tc->regmap, DP0_AUXWDATA(0), auxwdata, count);
	if (ret)
		return ret;

	return size;
}

static int tc_aux_read_data(struct tc_data *tc, void *data, size_t size)
{
	u32 auxrdata[DP_AUX_MAX_PAYLOAD_BYTES / sizeof(u32)];
	int ret, count = ALIGN(size, sizeof(u32));

	ret = regmap_raw_read(tc->regmap, DP0_AUXRDATA(0), auxrdata, count);
	if (ret)
		return ret;

	memcpy(data, auxrdata, size);

	return size;
}

static u32 tc_auxcfg0(struct drm_dp_aux_msg *msg, size_t size)
{
	u32 auxcfg0 = msg->request;

	if (size)
		auxcfg0 |= FIELD_PREP(DP0_AUXCFG0_BSIZE, size - 1);
	else
		auxcfg0 |= DP0_AUXCFG0_ADDR_ONLY;

	return auxcfg0;
}

static ssize_t tc_aux_transfer(struct drm_dp_aux *aux,
			       struct drm_dp_aux_msg *msg)
{
	struct tc_data *tc = aux_to_tc(aux);
	size_t size = min_t(size_t, DP_AUX_MAX_PAYLOAD_BYTES - 1, msg->size);
	u8 request = msg->request & ~DP_AUX_I2C_MOT;
	u32 auxstatus;
	int ret;

	ret = regmap_read(tc->regmap, DP0_AUXSTATUS, &auxstatus);
	ret = tc_aux_wait_busy(tc);
	if (ret)
		return ret;

	switch (request) {
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		break;
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
		if (size) {
			ret = tc_aux_write_data(tc, msg->buffer, size);
			if (ret < 0)
				return ret;
		}
		break;
	default:
		return -EINVAL;
	}

	/* Store address */
	ret = tc358770_write(tc->regmap, DP0_AUXADDR, msg->address);
	if (ret)
		return ret;
	/* Start transfer */
	ret = tc358770_write(tc->regmap, DP0_AUXCFG0, tc_auxcfg0(msg, size));
	if (ret)
		return ret;

	ret = tc_aux_wait_busy(tc);
	if (ret)
		return ret;

	ret = regmap_read(tc->regmap, DP0_AUXSTATUS, &auxstatus);
	if (ret)
		return ret;

	if (auxstatus & AUX_TIMEOUT)
		return -ETIMEDOUT;
	/*
	 * For some reason address-only DP_AUX_I2C_WRITE (MOT), still
	 * reports 1 byte transferred in its status. To deal we that
	 * we ignore aux_bytes field if we know that this was an
	 * address-only transfer
	 */
	if (size)
		size = FIELD_GET(AUX_BYTES, auxstatus);
	msg->reply = FIELD_GET(AUX_STATUS, auxstatus);

	switch (request) {
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		if (size)
			return tc_aux_read_data(tc, msg->buffer, size);
		break;
	}

	return size;
}

static const char * const training_pattern1_errors[] = {
	"No errors",
	"Aux write error",
	"Aux read error",
	"Max voltage reached error",
	"Loop counter expired error",
	"res", "res", "res"
};

static const char * const training_pattern2_errors[] = {
	"No errors",
	"Aux write error",
	"Aux read error",
	"Clock recovery failed error",
	"Loop counter expired error",
	"res", "res", "res"
};

#if 0
static u32 tc_srcctrl(struct tc_data *tc)
{
	/*
	 * No training pattern, skew lane 1 data by two LSCLK cycles with
	 * respect to lane 0 data, AutoCorrect Mode = 0
	 */
	u32 reg = DP0_SRCCTRL_NOTP | DP0_SRCCTRL_LANESKEW | DP0_SRCCTRL_EN810B;

	if (tc->link.scrambler_dis)
		reg |= DP0_SRCCTRL_SCRMBLDIS;	/* Scrambler Disabled */
	if (tc->link.spread)
		reg |= DP0_SRCCTRL_SSCG;	/* Spread Spectrum Enable */
	if (tc->link.base.num_lanes == 2)
		reg |= DP0_SRCCTRL_LANES_2;	/* Two Main Channel Lanes */
	if (tc->link.base.rate != 162000)
		reg |= DP0_SRCCTRL_BW27;	/* 2.7 Gbps link */
	return reg;
}
#endif

static int tc_pllupdate(struct tc_data *tc, unsigned int pllctrl)
{
	int ret;

	ret = tc358770_write(tc->regmap, pllctrl, PLLUPDATE | PLLEN);
	if (ret)
		return ret;

	/* Wait for PLL to lock: up to 2.09 ms, depending on refclk */
	usleep_range(3000, 6000);

	return 0;
}

static int tc_pxl_pll_en(struct tc_data *tc, u32 refclk, u32 pixelclock)
{
	int ret;
	int i_pre, best_pre = 1;
	int i_post, best_post = 1;
	int div, best_div = 1;
	int mul, best_mul = 1;
	int delta, best_delta;
	int post_div[] = {1, 2, 3, 5, 7};
	int pre_div[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
	int best_pixelclock = 0;
	int vco_hi = 0;
	u32 pxl_pllparam;

	dev_dbg(tc->dev, "PLL: requested %d pixelclock, ref %d\n", pixelclock,
		refclk);
	best_delta = pixelclock;
	/* Loop over all possible ext_divs, skipping invalid configurations */
	for (i_pre = 0; i_pre < ARRAY_SIZE(pre_div); i_pre++) {
		/*
		 * refclk / ext_pre_div should be in the 1 to 200 MHz range.
		 * We don't allow any refclk > 200 MHz, only check lower bounds.
		 */
		if (refclk / pre_div[i_pre] < 1000000)
			continue;
		for (i_post = 0; i_post < ARRAY_SIZE(post_div); i_post++) {
			for (div = 1; div <= 16; div++) {
				u32 clk;
				u64 tmp;

				tmp = pixelclock * pre_div[i_pre] *
				      post_div[i_post] * div;
				do_div(tmp, refclk);
				mul = tmp;

				/* Check limits */
				if ((mul < 1) || (mul > 128))
					continue;

				clk = (refclk / pre_div[i_pre] / div) * mul;
				/*
				 * refclk * mul / (ext_pre_div * pre_div)
				 * should be in the 150 to 650 MHz range
				 */
				if ((clk > 650000000) || (clk < 150000000))
					continue;

				clk = clk / post_div[i_post];
				delta = clk - pixelclock;

				if (abs(delta) < abs(best_delta)) {
					best_pre = i_pre;
					best_post = i_post;
					best_div = div;
					best_mul = mul;
					best_delta = delta;
					best_pixelclock = clk;
				}
			}
		}
	}
	if (best_pixelclock == 0) {
		dev_err(tc->dev, "Failed to calc clock for %d pixelclock\n",
			pixelclock);
		return -EINVAL;
	}

	dev_dbg(tc->dev, "PLL: got %d, delta %d\n", best_pixelclock,
		best_delta);
	dev_dbg(tc->dev, "PLL: %d / %d / %d * %d / %d\n", refclk,
		pre_div[best_pre], best_div, best_mul, post_div[best_post]);

	/* if VCO >= 300 MHz */
	if (refclk / pre_div[best_pre] / best_div * best_mul >= 300000000)
		vco_hi = 1;
	/* see DS */
	if (best_div == 16)
		best_div = 0;
	if (best_mul == 128)
		best_mul = 0;

	/* Power up PLL and switch to bypass */
	ret = tc358770_write(tc->regmap, PXL_PLLCTRL, PLLBYP | PLLEN);
	if (ret)
		return ret;

	pxl_pllparam  = vco_hi << 24; /* For PLL VCO >= 300 MHz = 1 */
	pxl_pllparam |= pre_div[best_pre] << 20; /* External Pre-divider */
	pxl_pllparam |= post_div[best_post] << 16; /* External Post-divider */
	pxl_pllparam |= IN_SEL_REFCLK; /* Use RefClk as PLL input */
	pxl_pllparam |= best_div << 8; /* Divider for PLL RefClk */
	pxl_pllparam |= best_mul; /* Multiplier for PLL */

	dev_info(tc->dev, "PLL: %d clk  %x\n", refclk, pxl_pllparam);
	ret = tc358770_write(tc->regmap, PXL_PLLPARAM, pxl_pllparam);
	if (ret)
		return ret;

	/* Force PLL parameter update and disable bypass */
	return tc_pllupdate(tc, PXL_PLLCTRL);
}

static int tc_pxl_pll_dis(struct tc_data *tc)
{
	/* Enable PLL bypass, power down PLL */
	return tc358770_write(tc->regmap, PXL_PLLCTRL, PLLBYP);
}

static int tc_stream_clock_calc(struct tc_data *tc)
{
	/*
	 * If the Stream clock and Link Symbol clock are
	 * asynchronous with each other, the value of M changes over
	 * time. This way of generating link clock and stream
	 * clock is called Asynchronous Clock mode. The value M
	 * must change while the value N stays constant. The
	 * value of N in this Asynchronous Clock mode must be set
	 * to 2^15 or 32,768.
	 *
	 * LSCLK = 1/10 of high speed link clock
	 *
	 * f_STRMCLK = M/N * f_LSCLK
	 * M/N = f_STRMCLK / f_LSCLK
	 *
	 */
	return tc358770_write(tc->regmap, DP0_VIDMNGEN1, 0xA8C0);
}

static int tc_set_dsi_configuration(struct tc_data *tc)
{

	 tc358770_write(tc->regmap, DSI0_PPI_TX_RX_TA, 0x0008000B);
	 tc358770_write(tc->regmap, DSI0_PPI_LPTXTIMECNT, 0x00000007);
	 tc358770_write(tc->regmap, DSI0_PPI_D0S_CLRSIPOCOUNT, 0x0000000A);
	 tc358770_write(tc->regmap, DSI0_PPI_D1S_CLRSIPOCOUNT, 0x0000000A);
	 tc358770_write(tc->regmap, DSI0_PPI_D2S_CLRSIPOCOUNT, 0x0000000A);
	 tc358770_write(tc->regmap, DSI0_PPI_D3S_CLRSIPOCOUNT, 0x0000000A);
	 tc358770_write(tc->regmap, DSI0_PPI_LANEENABLE, 0x0000001F);
	 tc358770_write(tc->regmap, DSI0_DSI_LANEENABLE, 0x0000001F);
	 tc358770_write(tc->regmap, DSI0_PPI_STARTPPI, 0x00000001);
	 tc358770_write(tc->regmap, DSI0_DSI_STARTDSI, 0x00000001);
	 
	 tc358770_write(tc->regmap, DSI1_PPI_TX_RX_TA, 0x00070009);
	 tc358770_write(tc->regmap, DSI1_PPI_LPTXTIMECNT, 0x00000006);
	 tc358770_write(tc->regmap, DSI1_PPI_D0S_CLRSIPOCOUNT, 0x00000007);
	 tc358770_write(tc->regmap, DSI1_PPI_D1S_CLRSIPOCOUNT, 0x00000007);
	 tc358770_write(tc->regmap, DSI1_PPI_D2S_CLRSIPOCOUNT, 0x00000007);
	 tc358770_write(tc->regmap, DSI1_PPI_D3S_CLRSIPOCOUNT, 0x00000007);
	 tc358770_write(tc->regmap, DSI1_PPI_LANEENABLE, 0x0000001F);
	 tc358770_write(tc->regmap, DSI1_DSI_LANEENABLE, 0x0000001F);
	 tc358770_write(tc->regmap, DSI1_PPI_STARTPPI, 0x00000001);
	 tc358770_write(tc->regmap, DSI1_DSI_STARTDSI, 0x00000001);

	 return 1;
}

/*
static int tc_set_syspllparam(struct tc_data *tc)
{
	unsigned long rate;
	u32 pllparam = SYSCLK_SEL_LSCLK | LSCLK_DIV_2;

	rate = clk_get_rate(tc->refclk);
	switch (rate) {
	case 38400000:
		pllparam |= REF_FREQ_38M4;
		break;
	case 26000000:
		pllparam |= REF_FREQ_26M;
		break;
	case 19200000:
		pllparam |= REF_FREQ_19M2;
		break;
	case 13000000:
		pllparam |= REF_FREQ_13M;
		break;
	default:
		dev_err(tc->dev, "Invalid refclk rate: %lu Hz\n", rate);
		return -EINVAL;
	}

	return tc358770_write(tc->regmap, SYS_PLLPARAM, pllparam);
}
*/

static int tc_setup_main_link(struct tc_data *tc)
{
	dev_dbg(tc->dev,  "%s\n", __func__);
	tc358770_write(tc->regmap, DP0_SRCCTRL, 0x0000C095);
	tc358770_write(tc->regmap, SYS_PLLPARAM, 0x00000106);

	return 1;
}

static int tc_dp_phy_plls(struct tc_data *tc)
{
	int ret;

	dev_dbg(tc->dev,  "%s\n", __func__);
	tc358770_write(tc->regmap, DP_PHY_CTRL, 0x0000000F);
	/* PLL setup */
	ret = tc_pllupdate(tc, DP0_PLLCTRL);
	if (ret)
		return ret;
#if 0
/*	tc358770_write(tc->regmap, PXL_PLLPARAM, 0x0022823a);  */
	tc358770_write(tc->regmap, PXL_PLLPARAM, 0x00228236);  

	/* Force PLL parameter update and disable bypass */
	return tc_pllupdate(tc, PXL_PLLCTRL);
#endif 		

	ret = tc_pxl_pll_en(tc, clk_get_rate(tc->refclk),
			    1000 * tc->mode.clock);
	return ret;

}	

static int tc_reset_enable_main_link(struct tc_data *tc)
{

	u32 dp_phy_ctrl;
	int ret;

	dev_dbg(tc->dev,  "%s\n", __func__);
	/* Reset/Enable Main Links */
	dp_phy_ctrl = PHY_M0_EN | PHY_A0_EN | PHY_2LANE | BGREN | PHY_M0_RST;
	ret = tc358770_write(tc->regmap, DP_PHY_CTRL, dp_phy_ctrl);
	usleep_range(100, 200);
	dp_phy_ctrl = PHY_M0_EN | PHY_A0_EN | PHY_2LANE | BGREN ;
	ret = tc358770_write(tc->regmap, DP_PHY_CTRL, dp_phy_ctrl);

	ret = tc_poll_timeout(tc, DP_PHY_CTRL, PHY_RDY, PHY_RDY, 500, 100000);
	if (ret) {
		dev_err(tc->dev, "timeout waiting for phy become ready");
		return ret;
	}
	return 0;
}	

static int tc_set_display_props(struct tc_data *tc)
{	

	int ret;

	dev_dbg(tc->dev,  "%s\n", __func__);
	ret = tc358770_write(tc->regmap, DP0_AUXCFG1, 0x0001063F);
	
	/* Read DP Rx Link Capability */
	ret = drm_dp_link_probe(&tc->aux, &tc->link.base);
	if (ret < 0)
		goto err_dpcd_read;
	if (tc->link.base.rate != 162000 && tc->link.base.rate != 270000) {
		dev_dbg(tc->dev, "Falling to 2.7 Gbps rate\n");
		tc->link.base.rate = 270000;
	}

	if (tc->link.base.num_lanes > 2) {
		dev_dbg(tc->dev, "Falling to 2 lanes\n");
		tc->link.base.num_lanes = 2;
	}

	dev_dbg(tc->dev, "DPCD rev: %d.%d, rate: %s, lanes: %d, framing: %s\n",
		tc->link.base.revision >> 4, tc->link.base.revision & 0x0f,
		(tc->link.base.rate == 162000) ? "1.62Gbps" : "2.7Gbps",
		tc->link.base.num_lanes,
		(tc->link.base.capabilities & DP_LINK_CAP_ENHANCED_FRAMING) ?
		"enhanced" : "non-enhanced");
	dev_dbg(tc->dev, "Downspread: %s, scrambler: %s\n",
		tc->link.spread ? "0.5%" : "0.0%",
		tc->link.scrambler_dis ? "disabled" : "enabled");
	dev_dbg(tc->dev, "Display ASSR: %d, TC358770 ASSR: %d\n",
		tc->link.assr, tc->assr);

	/* Setup Link & DPRx Config for Training */
	ret = drm_dp_link_configure(&tc->aux, &tc->link.base);
	if (ret < 0)
		goto err_dpcd_write;

        ret = drm_dp_dpcd_writeb(&tc->aux, DP_MAIN_LINK_CHANNEL_CODING_SET,
                                 DP_SET_ANSI_8B10B);
        if (ret > 0)
                return ret;

err_dpcd_write:
err_dpcd_read:
	dev_err(tc->dev, "failed to read or write DPCD: %d\n", ret);
	return ret;

}

static int tc_wait_link_training(struct tc_data *tc)
{
	u32 value;
	int ret;

	ret = tc_poll_timeout(tc, DP0_LTSTAT, LT_LOOPDONE,
			      LT_LOOPDONE, 5000, 100000);
	if (ret) {
		dev_err(tc->dev, "Link training timeout waiting for LT_LOOPDONE!\n");
		return ret;
	}

	ret = regmap_read(tc->regmap, DP0_LTSTAT, &value);
	if (ret)
		return ret;

	return (value >> 8) & 0x7;
}

static int tc_training_pattern(struct tc_data *tc)
{

	int ret;
	u32 value;
	u8 tmp[DP_LINK_STATUS_SIZE];

	/* Reset voltage-swing & pre-emphasis */
	tmp[0] = tmp[1] = DP_TRAIN_VOLTAGE_SWING_LEVEL_0 |
			  DP_TRAIN_PRE_EMPH_LEVEL_0;
	ret = drm_dp_dpcd_write(&tc->aux, DP_TRAINING_LANE0_SET, tmp, 2);
	if (ret < 0)
		goto err_dpcd_write;

	/* Set DPCD 0x102 for Training Pattern 1 */
	ret = tc358770_write(tc->regmap, DP0_SNKLTCTRL,  0x00000021);
	ret = tc358770_write(tc->regmap, DP0_LTLOOPCTRL, 0x7600000D);
	ret = tc358770_write(tc->regmap, DP0_SRCCTRL, 0x0000C195);


	ret = tc358770_write(tc->regmap, DP0CTL, VID_MN_GEN | DP_EN);

	/* wait */
	ret = tc_wait_link_training(tc);
	if (ret < 0)
		return ret;

	if (ret) {
		dev_err(tc->dev, "Link training phase 1 failed: %s\n",
			training_pattern1_errors[ret]);
		return -ENODEV;
	}

	/* Channel Equalization */

	ret = tc358770_write(tc->regmap, DP0_SNKLTCTRL,  0x00000022);
	ret = tc358770_write(tc->regmap, DP0_SRCCTRL, 0x0000C295);


	/* wait */
	ret = tc_wait_link_training(tc);
	if (ret < 0)
		return ret;

	if (ret) {
		dev_err(tc->dev, "Link training phase 2 failed: %s\n",
			training_pattern2_errors[ret]);
		return -ENODEV;
	}

        /* Clear DPCD 0x102 */
        /* Note: Can Not use DP0_SNKLTCTRL (0x06E4) short cut */
        tmp[0] = tc->link.scrambler_dis ? DP_LINK_SCRAMBLING_DISABLE : 0x00;
        ret = drm_dp_dpcd_writeb(&tc->aux, DP_TRAINING_PATTERN_SET, tmp[0]);
        if (ret < 0)
                goto err_dpcd_write;

	ret = tc358770_write(tc->regmap, DP0_SRCCTRL, 0x00004095);
        /* Check link status */
        ret = drm_dp_dpcd_read_link_status(&tc->aux, tmp);
        if (ret < 0)
                goto err_dpcd_read;

        ret = 0;

        value = tmp[0] & DP_CHANNEL_EQ_BITS;

        if (value != DP_CHANNEL_EQ_BITS) {
                dev_err(tc->dev, "Lane 0 failed: %x\n", value);
                ret = -ENODEV;
        }

        if (tc->link.base.num_lanes == 2) {
                value = (tmp[0] >> 4) & DP_CHANNEL_EQ_BITS;

                if (value != DP_CHANNEL_EQ_BITS) {
                        dev_err(tc->dev, "Lane 1 failed: %x\n", value);
                        ret = -ENODEV;
                }

                if (!(tmp[2] & DP_INTERLANE_ALIGN_DONE)) {
                        dev_err(tc->dev, "Interlane align failed\n");
                        ret = -ENODEV;
                }
        }

        if (ret) {
                dev_err(tc->dev, "0x0202 LANE0_1_STATUS:            0x%02x\n", tmp[0]);
                dev_err(tc->dev, "0x0203 LANE2_3_STATUS             0x%02x\n", tmp[1]);
                dev_err(tc->dev, "0x0204 LANE_ALIGN_STATUS_UPDATED: 0x%02x\n", tmp[2]);
                dev_err(tc->dev, "0x0205 SINK_STATUS:               0x%02x\n", tmp[3]);
                dev_err(tc->dev, "0x0206 ADJUST_REQUEST_LANE0_1:    0x%02x\n", tmp[4]);
                dev_err(tc->dev, "0x0207 ADJUST_REQUEST_LANE2_3:    0x%02x\n", tmp[5]);
                return ret;
        }
        
	tmp[0] = tc->assr;
        ret = drm_dp_dpcd_writeb(&tc->aux, DP_EDP_CONFIGURATION_SET, tmp[0]);
        if (ret < 0)
            goto err_dpcd_read;

	/* DOWNSPREAD_CTRL */
	tmp[0] = tc->link.spread ? DP_SPREAD_AMP_0_5 : 0x00;
	ret = drm_dp_dpcd_write(&tc->aux, DP_DOWNSPREAD_CTRL, tmp, 1);
	if (ret < 0)
		goto err_dpcd_write;

        return 0;

err_dpcd_write:
err_dpcd_read:
        dev_err(tc->dev, "Failed to read/write DPCD: %d\n", ret);
        return ret;
}	


static int tc_set_video_mode(struct tc_data *tc,
			     const struct drm_display_mode *mode)
{
	int ret;
	int vid_sync_dly;
	int max_tu_symbol;

	int left_margin = mode->htotal - mode->hsync_end;
	int right_margin = mode->hsync_start - mode->hdisplay;
	int hsync_len = mode->hsync_end - mode->hsync_start;
	int upper_margin = mode->vtotal - mode->vsync_end;
	int lower_margin = mode->vsync_start - mode->vdisplay;
	int vsync_len = mode->vsync_end - mode->vsync_start;
	u32 dp0_syncval;
	u32 bits_per_pixel = 24;
	u32 in_bw, out_bw;

	/*
	 * Recommended maximum number of symbols transferred in a transfer unit:
	 * DIV_ROUND_UP((input active video bandwidth in bytes) * tu_size,
	 *              (output active video bandwidth in bytes))
	 * Must be less than tu_size.
	 */

	in_bw = mode->clock * bits_per_pixel / 8;
	out_bw = tc->link.base.num_lanes * tc->link.base.rate;
//	max_tu_symbol = DIV_ROUND_UP(in_bw * TU_SIZE_RECOMMENDED, out_bw);
	max_tu_symbol = 0x1e;

	dev_dbg(tc->dev, "set mode %dx%d\n",
		mode->hdisplay, mode->vdisplay);
	dev_dbg(tc->dev, "H margin %d,%d sync %d\n",
		left_margin, right_margin, hsync_len);
	dev_dbg(tc->dev, "V margin %d,%d sync %d\n",
		upper_margin, lower_margin, vsync_len);
	dev_dbg(tc->dev, "total: %dx%d\n", mode->htotal, mode->vtotal);

	/* Combiner Layer */
	tc358770_write(tc->regmap, CMBCTRL, 0);
	tc358770_write(tc->regmap, LRSIZE, (mode->hdisplay)<<16);
	tc358770_write(tc->regmap, RMPXL, 0);

	/*
	 * LCD Ctl Frame Size
	 * datasheet is not clear of vsdelay in case of DPI
	 * assume we do not need any delay when DPI is a source of
	 * sync signals
	 */
	ret = tc358770_write(tc->regmap, VPCTRL0,
			   FIELD_PREP(VSDELAY, 0x200) |
			   OPXLFMT_RGB666 | FRMSYNC_DISABLED | MSF_DISABLED);
	if (ret)
		return ret;

	ret = tc358770_write(tc->regmap, HTIM01,
			   FIELD_PREP(HBPR, ALIGN(left_margin, 2)) |
			   FIELD_PREP(HPW, ALIGN(hsync_len, 2)));
	if (ret)
		return ret;

	ret = tc358770_write(tc->regmap, HTIM02,
			   FIELD_PREP(HDISPR, ALIGN(mode->hdisplay, 2)) |
			   FIELD_PREP(HFPR, ALIGN(right_margin, 2)));
	if (ret)
		return ret;

	ret = tc358770_write(tc->regmap, VTIM01,
			   FIELD_PREP(VBPR, upper_margin) |
			   FIELD_PREP(VSPR, vsync_len));
	if (ret)
		return ret;

	ret = tc358770_write(tc->regmap, VTIM02,
			   FIELD_PREP(VFPR, lower_margin) |
			   FIELD_PREP(VDISPR, mode->vdisplay));
	if (ret)
		return ret;

	ret = tc358770_write(tc->regmap, VFUEN0, VFUEN); /* update settings */
	if (ret)
		return ret;

	/* Test pattern settings */
	ret = tc358770_write(tc->regmap, TSTCTL,
			   FIELD_PREP(COLOR_R, 120) |
			   FIELD_PREP(COLOR_G, 20) |
			   FIELD_PREP(COLOR_B, 99) |
			   ENI2CFILTER |
			   FIELD_PREP(COLOR_BAR_MODE, COLOR_BAR_MODE_BARS));
	if (ret)
		return ret;

	/* DP Main Stream Attributes */
	vid_sync_dly = hsync_len + left_margin + mode->hdisplay;
	ret = tc358770_write(tc->regmap, DP0_VIDSYNCDELAY,
		 FIELD_PREP(THRESH_DLY, max_tu_symbol) |
		 FIELD_PREP(VID_SYNC_DLY, vid_sync_dly));
	ret = tc358770_write(tc->regmap, DP0_VIDSYNCDELAY, 0x1f0804);

	ret = tc358770_write(tc->regmap, DP0_TOTALVAL,
			   FIELD_PREP(H_TOTAL, mode->htotal) |
			   FIELD_PREP(V_TOTAL, mode->vtotal));
	if (ret)
		return ret;

	ret = tc358770_write(tc->regmap, DP0_STARTVAL,
			   FIELD_PREP(H_START, left_margin + hsync_len) |
			   FIELD_PREP(V_START, upper_margin + vsync_len));
	if (ret)
		return ret;

	ret = tc358770_write(tc->regmap, DP0_ACTIVEVAL,
			   FIELD_PREP(V_ACT, mode->vdisplay) |
			   FIELD_PREP(H_ACT, mode->hdisplay));
	if (ret)
		return ret;

	dp0_syncval = FIELD_PREP(VS_WIDTH, vsync_len) |
		      FIELD_PREP(HS_WIDTH, hsync_len);

//	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		dp0_syncval |= SYNCVAL_VS_POL_ACTIVE_LOW;

//	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		dp0_syncval |= SYNCVAL_HS_POL_ACTIVE_LOW;

	ret = tc358770_write(tc->regmap, DP0_SYNCVAL, dp0_syncval);
	if (ret)
		return ret;

	ret = tc358770_write(tc->regmap, DPIPXLFMT,
			   VS_POL_ACTIVE_LOW | HS_POL_ACTIVE_LOW |
			   DE_POL_ACTIVE_HIGH | SUB_CFG_TYPE_CONFIG1 |
			   DPI_BPP_RGB888);
	if (ret)
		return ret;

	ret = tc358770_write(tc->regmap, DP0_MISC,0x1EBF0000);
/*	ret = tc358770_write(tc->regmap, DP0_MISC,
			   FIELD_PREP(MAX_TU_SYMBOL, max_tu_symbol) |
			   FIELD_PREP(TU_SIZE, TU_SIZE_RECOMMENDED) |
			   BPC_6);*/
	if (ret)
		return ret;

	return 0;
}

static int tc_main_link_disable(struct tc_data *tc)
{
	int ret;

	dev_dbg(tc->dev, "link disable\n");

	ret = tc358770_write(tc->regmap, DP0_SRCCTRL, 0);
	if (ret)
		return ret;

	return tc358770_write(tc->regmap, DP0CTL, 0);
}

static int tc_stream_enable(struct tc_data *tc)
{
	int ret;
	u32 value;

	dev_dbg(tc->dev, "enable video stream\n");

	/* PXL PLL setup */
	if (tc_test_pattern) {
		ret = tc_pxl_pll_en(tc, clk_get_rate(tc->refclk),
				    1000 * tc->mode.clock);
		if (ret)
			return ret;
	}

	ret = tc_set_video_mode(tc, &tc->mode);
	if (ret)
		return ret;

	/* Set M/N */
	ret = tc_stream_clock_calc(tc);
	if (ret)
		return ret;

	value = VID_MN_GEN | DP_EN;
	if (tc->link.base.capabilities & DP_LINK_CAP_ENHANCED_FRAMING)
		value |= EF_EN;
	ret = tc358770_write(tc->regmap, DP0CTL, value);
	if (ret)
		return ret;
	/*
	 * VID_EN assertion should be delayed by at least N * LSCLK
	 * cycles from the time VID_MN_GEN is enabled in order to
	 * generate stable values for VID_M. LSCLK is 270 MHz or
	 * 162 MHz, VID_N is set to 32768 in  tc_stream_clock_calc(),
	 * so a delay of at least 203 us should suffice.
	 */
	usleep_range(500, 1000);
	value |= VID_EN;
	ret = tc358770_write(tc->regmap, DP0CTL, value);
	if (ret)
		return ret;
	/* Set input interface */
	value = DP0_AUDSRC_NO_INPUT;
	if (tc_test_pattern)
		value |= DP0_VIDSRC_COLOR_BAR;
	else
		value |= DP0_VIDSRC_DSI_RX;
	ret = tc358770_write(tc->regmap, SYSCTRL, value);
	if (ret)
		return ret;

	return 0;
}

static int tc_stream_disable(struct tc_data *tc)
{
	int ret;

	dev_dbg(tc->dev, "disable video stream\n");

	ret = regmap_update_bits(tc->regmap, DP0CTL, VID_EN, 0);
	if (ret)
		return ret;

	tc_pxl_pll_dis(tc);

	return 0;
}

static void tc_bridge_mode_set(struct drm_bridge *bridge,
			       struct drm_display_mode *mode,
			       struct drm_display_mode *adj)
{
	struct tc_data *tc = bridge_to_tc(bridge);

	dev_dbg(tc->dev,  "%s start\n", __func__);
	tc->mode = *mode;
}

static int tc_connector_get_modes(struct drm_connector *connector)
{
	struct tc_data *tc = connector_to_tc(connector);
	return drm_panel_get_modes(tc->panel);

	dev_dbg(tc->dev,  "%s start\n", __func__);
/*	
	struct edid *edid;
	int count;
	int ret;

	ret = tc_get_display_props(tc);
	if (ret < 0) {
		dev_err(tc->dev, "failed to read display props: %d\n", ret);
		return 0;
	}

	count = drm_panel_get_modes(tc->panel);
	if (count > 0)
		return count;

	edid = drm_get_edid(connector, &tc->aux.ddc);

	kfree(tc->edid);
	tc->edid = edid;
	if (!edid)
		return 0;

	drm_connector_update_edid_property(connector, edid);
	count = drm_add_edid_modes(connector, edid);

	return count;
*/	
}

static struct drm_connector_helper_funcs tc_bridge_connector_helper_funcs = {
	.get_modes = tc_connector_get_modes,
};

static enum drm_connector_status
tc_bridge_connector_detect(struct drm_connector *connector, bool force)
{
	/**
	 * TODO: Currently if drm_panel is present, then always
	 * return the status as connected. Need to add support to detect
	 * device state for hot pluggable scenarios.
	 */
	return connector_status_connected;
}

static const struct drm_connector_funcs tc_bridge_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = tc_bridge_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int tc_bridge_attach(struct drm_bridge *bridge)
{
	int ret;
	struct tc_data *tc = bridge_to_tc(bridge);
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	const struct mipi_dsi_device_info info = { .type = "tc_bridge",
						   .channel = 0,
						   .node = NULL,
						 };

	dev_dbg(tc->dev,  "%s start\n", __func__);
	ret = drm_connector_init(bridge->dev, &tc->connector,
				 &tc_bridge_connector_funcs,
				 DRM_MODE_CONNECTOR_eDP);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&tc->connector,
				 &tc_bridge_connector_helper_funcs);
//	drm_connector_attach_encoder(&tc->connector, bridge->encoder);
	drm_mode_connector_attach_encoder(&tc->connector,
							bridge->encoder);

	/*
	 * TODO: ideally finding host resource and dsi dev registration needs
	 * to be done in bridge probe. But some existing DSI host drivers will
	 * wait for any of the drm_bridge/drm_panel to get added to the global
	 * bridge/panel list, before completing their probe. So if we do the
	 * dsi dev registration part in bridge probe, before populating in
	 * the global bridge list, then it will cause deadlock as dsi host probe
	 * will never complete, neither our bridge probe. So keeping it here
	 * will satisfy most of the existing host drivers. Once the host driver
	 * is fixed we can move the below code to bridge probe safely.
	 */
	host = of_find_mipi_dsi_host_by_node(tc->host_node);
	if (!host) {
		DRM_ERROR("failed to find dsi host\n");
		ret = -ENODEV;
		goto err_dsi_host;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		DRM_ERROR("failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		goto err_dsi_host;
	}

	/* TODO: setting to 4 lanes always for now */
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST
                | MIPI_DSI_MODE_VIDEO_AUTO_VERT | MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		DRM_ERROR("failed to attach dsi to host\n");
		goto err_dsi_attach;
	}
	tc->dsi = dsi;

	/* attach panel to bridge */
	drm_panel_attach(tc->panel, &tc->connector);

	return 0;

err_dsi_attach:
	mipi_dsi_device_unregister(dsi);
err_dsi_host:
	drm_connector_cleanup(&tc->connector);
	return ret;
}

static void tc_bridge_enable(struct drm_bridge *bridge)
{
	struct tc_data *tc = bridge_to_tc(bridge);
	int ret;

	dev_dbg(tc->dev,  "%s start\n", __func__);
	ret = tc_setup_main_link(tc);
	if (ret < 0) {
		dev_err(tc->dev, "failed to setup main link: %d\n", ret);
		return;
	}

	ret = tc_dp_phy_plls(tc);
	if (ret < 0) {
		dev_err(tc->dev, "failed to DP Phy and PLLs: %d\n", ret);
		return;
	}

	ret = tc_reset_enable_main_link(tc);
	if (ret < 0) {
		dev_err(tc->dev, "failed to reset and enable main link: %d\n", ret);
		return;
	}

	ret = tc_set_display_props(tc);
	if (ret < 0) {
		dev_err(tc->dev, "failed to set display property: %d\n", ret);
		return;
	}

	ret = tc_training_pattern(tc);
	if (ret < 0) {
		dev_err(tc->dev, "failed to training pattern: %d\n", ret);
		return;
	}

	ret = tc_stream_enable(tc);
	if (ret < 0) {
		dev_err(tc->dev, "main link stream start error: %d\n", ret);
		tc_main_link_disable(tc);
		return;
	}

	drm_panel_enable(tc->panel);
}

static void tc_bridge_disable(struct drm_bridge *bridge)
{
	struct tc_data *tc = bridge_to_tc(bridge);
	int ret;

	dev_dbg(tc->dev,  "%s start\n", __func__);
	drm_panel_disable(tc->panel);

	ret = tc_stream_disable(tc);
	if (ret < 0)
		dev_err(tc->dev, "main link stream stop error: %d\n", ret);

	ret = tc_main_link_disable(tc);
	if (ret < 0)
		dev_err(tc->dev, "main link disable error: %d\n", ret);
}

static void tc_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct tc_data *tc = bridge_to_tc(bridge);

	dev_dbg(tc->dev,  "%s start\n", __func__);

        if (tc->enable_gpio){
		gpiod_direction_output(tc->enable_gpio, 1);
	}	
	
	tc_set_dsi_configuration(tc);
	drm_panel_prepare(tc->panel);
}

static void tc_bridge_post_disable(struct drm_bridge *bridge)
{
	struct tc_data *tc = bridge_to_tc(bridge);

	dev_dbg(tc->dev,  "%s start\n", __func__);
//	if (tc->refclk)
//		clk_disable_unprepare(tc->refclk);
}

static bool tc_bridge_mode_fixup(struct drm_bridge *bridge,
				 const struct drm_display_mode *mode,
				 struct drm_display_mode *adj)
{
	struct tc_data *tc = bridge_to_tc(bridge);

	dev_dbg(tc->dev,  "%s start\n", __func__);
	/* Fixup sync polarities, both hsync and vsync are active low */
	adj->flags = mode->flags;
	adj->flags |= (DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC);
	adj->flags &= ~(DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC);

	return true;
}

static const struct drm_bridge_funcs tc_bridge_funcs = {
	.attach = tc_bridge_attach,
	.mode_set = tc_bridge_mode_set,
	.pre_enable = tc_bridge_pre_enable,
	.enable = tc_bridge_enable,
	.disable = tc_bridge_disable,
	.post_disable = tc_bridge_post_disable,
	.mode_fixup = tc_bridge_mode_fixup,
};

static bool tc_readable_reg(struct device *dev, unsigned int reg)
{
	return reg != SYSCTRL;
}

static const struct regmap_range tc_volatile_ranges[] = {
	regmap_reg_range(DP0_AUXWDATA(0), DP0_AUXSTATUS),
	regmap_reg_range(DP0_LTSTAT, DP0_SNKLTCHGREQ),
	regmap_reg_range(DP_PHY_CTRL, DP_PHY_CTRL),
	regmap_reg_range(DP0_PLLCTRL, PXL_PLLCTRL),
	regmap_reg_range(VFUEN0, VFUEN0),
	regmap_reg_range(INTSTS_G, INTSTS_G),
	regmap_reg_range(GPIOI, GPIOI),
};

static const struct regmap_access_table tc_volatile_table = {
	.yes_ranges = tc_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(tc_volatile_ranges),
};

static bool tc_writeable_reg(struct device *dev, unsigned int reg)
{
	return (reg != TC_IDREG) &&
	       (reg != DP0_LTSTAT) &&
	       (reg != DP0_SNKLTCHGREQ);
}

static const struct regmap_config tc_regmap_config = {
	.name = "tc3658770",
	.reg_bits = 16,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = PLL_DBG,
	.cache_type = REGCACHE_RBTREE,
	.readable_reg = tc_readable_reg,
	.volatile_table = &tc_volatile_table,
	.writeable_reg = tc_writeable_reg,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int tc_bridge_parse_dsi_host(struct tc_data *tc)
{
	struct device_node *np = tc->dev->of_node;

	tc->host_node = of_graph_get_remote_node(np, 0, 0);

	if (!tc->host_node) {
		DRM_ERROR("remote dsi host node not found\n");
		return -ENODEV;
	}
	return 0;
}

static int tc_bridge_probe(struct i2c_client *client,
			      const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tc_data *tc;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		DRM_ERROR("device doesn't support I2C\n");
		return -ENODEV;
	}

	tc = devm_kzalloc(dev, sizeof(struct tc_data), GFP_KERNEL);
	if (!tc)
		return -ENOMEM;

	tc->regmap = devm_regmap_init_i2c(client, &tc_regmap_config);
	if (IS_ERR(tc->regmap)) {
		ret = PTR_ERR(tc->regmap);
		dev_err(dev, "Failed to initialize regmap: %d\n", ret);
		return ret;
	}

	tc->dev = &client->dev;

	/* port@1 is the output port */
	ret = drm_of_find_panel_or_bridge(dev->of_node, 1, 0, &tc->panel, NULL);
	if (ret && ret != -ENODEV)
		return ret;

	dev_set_drvdata(&client->dev, tc);

	tc->enable_gpio = devm_gpiod_get_optional(tc->dev, "enable",
                                                  GPIOD_ASIS);
	if (IS_ERR(tc->enable_gpio)) {
		DRM_ERROR("failed to get enable gpio from DT\n");
		ret = PTR_ERR(tc->enable_gpio);
	}

        if (tc->enable_gpio){
		gpiod_direction_output(tc->enable_gpio, 1);
	}

	tc->refclk = devm_clk_get(tc->dev, "refclk");
	if (IS_ERR(tc->refclk)) {
		ret = PTR_ERR(tc->refclk);
		if (ret == -EPROBE_DEFER)
			return ret;
		DRM_DEBUG_KMS("refclk not found\n");
		tc->refclk = NULL;
	}

       ret = regmap_read(tc->regmap, TC_IDREG, &tc->rev);
        if (ret) {
                dev_err(tc->dev, "can not read device ID: %d\n", ret);
                return ret;
        }


        dev_info(tc->dev, "Device ID: 0x%08x\n", tc->rev);

        if ((tc->rev != 0x7001) && (tc->rev != 0x7003)) {
                dev_err(tc->dev, "invalid device ID: 0x%08x\n", tc->rev);
                return -EINVAL;
        }

        tc->assr = (tc->rev == 0x7001); /* Enable ASSR for eDP panels */

	//DSI port initial 
//	tc_set_dsi_configuration(tc);

	ret = tc_bridge_parse_dsi_host(tc);
	if (ret)
		return ret;

	/* Register DP AUX channel */
	tc->aux.name = "TC358770 AUX i2c adapter";
	tc->aux.dev = tc->dev;
	tc->aux.transfer = tc_aux_transfer;
	ret = drm_dp_aux_register(&tc->aux);
	if (ret)
		return ret;

	tc->bridge.funcs = &tc_bridge_funcs;
	tc->bridge.of_node = dev->of_node;
	drm_bridge_add(&tc->bridge);

	i2c_set_clientdata(client, tc);

	return 0;
}

static int tc_bridge_remove(struct i2c_client *client)
{
	struct tc_data *tc = i2c_get_clientdata(client);

	if (!tc)
		return -EINVAL;

	of_node_put(tc->host_node);

	if (tc->dsi) {
		mipi_dsi_detach(tc->dsi);
		mipi_dsi_device_unregister(tc->dsi);
	}

	drm_bridge_remove(&tc->bridge);
	drm_dp_aux_unregister(&tc->aux);

	return 0;
}

static const struct i2c_device_id tc358770_i2c_ids[] = {
        { "tc358770", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, tc358770_i2c_ids);

static const struct of_device_id tc358770_of_ids[] = {
        { .compatible = "toshiba,tc358770", },
        { }
};
MODULE_DEVICE_TABLE(of, tc358770_of_ids);

static struct i2c_driver tc_bridge_driver = {
	.driver = {
		.name = "toshiba_tc358770",
		.of_match_table = tc358770_of_ids,
	},
	.probe = tc_bridge_probe,
	.remove = tc_bridge_remove,
	.id_table = tc358770_i2c_ids,
};
module_i2c_driver(tc_bridge_driver);

MODULE_AUTHOR("Shih Yuan Huang <timmy@ubiqconn.com>");
MODULE_DESCRIPTION("tc358770 DSI to eDP bridge driver");
MODULE_LICENSE("GPL v2");

