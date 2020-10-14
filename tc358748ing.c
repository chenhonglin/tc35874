// SPDX-License-Identifier: GPL-2.0-only
/*
 * tc358748 - Parallel to CSI-2 bridge
 *
 * Copyright 2018 Marco Felsch <kernel@pengutronix.de>
 *
 * References:
 * REF_01:
 * - TC358746AXBG/TC358748XBG/TC358748IXBG Functional Specification Rev 1.2
 * REF_02:
 * - TC358746(A)748XBG_Parallel-CSI2_Tv23p.xlsx, Rev Tv23
 */


#define DEBUG
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/hdmi.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-of.h>
#include <media/camera_common.h>

#include <dt-bindings/gpio/tegra-gpio.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>

#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/i2c-dev.h>
#include <linux/fs.h>

// #include <media/v4l2-chip-ident.h>
#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>
#include <media/soc_camera.h>

#include "tc358748_regs.h"

/* RGB ouput selection */
// #define tc358748_VOUT_RGB

static int debug = 3;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

MODULE_DESCRIPTION("Toshiba tc358748 HDMI to CSI-2 bridge driver");
MODULE_AUTHOR("Ramakrishnan Muthukrishnan <ram@rkrishnan.org>");
MODULE_AUTHOR("Mikhail Khelik <mkhelik@cisco.com>");
MODULE_AUTHOR("Mats Randgaard <matrandg@cisco.com>");
MODULE_LICENSE("GPL");


#define I2C_MAX_XFER_SIZE	(512 + 2)
#define TC358748_MAX_FIFO_SIZE	512
#define TC358748_DEF_LINK_FREQ	0

#define TC358748_LINEINIT_MIN_US	110
#define TC358748_TWAKEUP_MIN_US		1200
#define TC358748_LPTXTIME_MIN_NS	55
#define TC358748_TCLKZERO_MIN_NS	305
#define TC358748_TCLKTRAIL_MIN_NS	65
#define TC358748_TCLKPOST_MIN_NS	65
#define TC358748_THSZERO_MIN_NS		150
#define TC358748_THSTRAIL_MIN_NS	65
#define TC358748_THSPREPARE_MIN_NS	45

static const struct v4l2_mbus_framefmt tc358748_def_fmt = {
	.width		= 640,
	.height		= 480,
	.code		= MEDIA_BUS_FMT_UYVY8_2X8,
	.field		= V4L2_FIELD_NONE,
	.colorspace	= V4L2_COLORSPACE_DEFAULT,
	.ycbcr_enc	= V4L2_YCBCR_ENC_DEFAULT,
	.quantization	= V4L2_QUANTIZATION_DEFAULT,
	.xfer_func	= V4L2_XFER_FUNC_DEFAULT,
};

struct tc358748_csi_param {
	unsigned char speed_range;
	unsigned int  unit_clk_hz;
	unsigned char unit_clk_mul;
	unsigned int speed_per_lane; /* bps / lane */
	unsigned short lane_num;
	bool is_continuous_clk;

	/* CSI2-TX Parameters */
	u32 lineinitcnt;
	u32 lptxtimecnt;
	u32 twakeupcnt;
	u32 tclk_preparecnt;
	u32 tclk_zerocnt;
	u32 tclk_trailcnt;
	u32 tclk_postcnt;
	u32 ths_preparecnt;
	u32 ths_zerocnt;
	u32 ths_trailcnt;

	unsigned int csi_hs_lp_hs_ps;
};
enum tc358748_csi_port {
	CSI_TX_NONE = 0,
	CSI_TX_0,
	CSI_TX_1,
	CSI_TX_BOTH
};



struct tc358748_platform_data {
	/* GPIOs */
	int reset_gpio;	

	#ifdef CONFIG_V4L2_FWNODE
	struct v4l2_fwnode_endpoint endpoint;
	#else
	struct v4l2_of_endpoint endpoint;
	#endif

	/* System clock connected to REFCLK (pin H5) */
	u32 refclk_hz; /* 26 MHz, 27 MHz or 42 MHz */



	/* CSI Output */
	enum tc358748_csi_port csi_port;  // TODO: Should this be port-index?

	/*
	 * The FIFO size is 512x32, so Toshiba recommend to set the default FIFO
	 * level to somewhere in the middle (e.g. 300), so it can cover speed
	 * mismatches in input and output ports.
	 */
	u16 fifo_level;

	/* Bps pr lane is (refclk_hz / pll_prd) * pll_fbd */
	u16 pll_prd;
	u16 pll_fbd;

	/* CSI
	 * Calculate CSI parameters with REF_02 for the highest resolution your
	 * CSI interface can handle. The driver will adjust the number of CSI
	 * lanes in use according to the pixel clock.
	 *
	 * The values in brackets are calculated with REF_02 when the number of
	 * bps pr lane is 823.5 MHz, and can serve as a starting point.
	 */
	u32 lineinitcnt;	
	u32 lptxtimecnt;	
	u32 tclk_headercnt;
	u32 tclk_trailcnt;	
	u32 ths_headercnt;	
	u32 twakeup;		
	u32 tclk_postcnt;
	u32 ths_trailcnt;	
	u32 hstxvregcnt;	

	};

struct tc358748_state {
	struct tc358748_platform_data pdata;
	struct v4l2_subdev sd;
	struct i2c_client *i2c_client;
	struct gpio_desc *reset_gpio;

	/*
	 * Generic
	 */
	struct media_pad pads[2];
	struct mutex confctl_mutex;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_ctrl_handler hdl;
	bool fmt_changed;
	bool test;

	/*
	 * Chip Clocks
	 */
	struct clk  *refclk;
	/* internal pll */
	unsigned int pllinclk_hz;
	u16 pll_prd;
	u16 pll_fbd;

	/*
	 * Video Buffer
	 */
	u16 vb_fifo; /* The FIFO size is 511x32 */

	/*
	 * CSI TX
	 */
	struct v4l2_ctrl	  *link_freq;
	struct tc358748_csi_param *link_freq_settings;
	u64			  *link_frequencies;
	unsigned int		   link_frequencies_num;

	/*
	 * Parallel input
	 */
	unsigned int pclk;
	unsigned int hblank;
};

struct tc358748_mbus_fmt {
	u32 code;
	u8 bus_width;
	u8 bpp;		 /* total bpp */
	u8 pdformat;	 /* peripheral data format */
	u8 pdataf;	 /* parallel data format option */
	u8 ppp;		 /* pclk per pixel */
	bool csitx_only; /* format only in csi-tx mode supported */
};

/* TODO: Add other formats as required */
static const struct tc358748_mbus_fmt tc358748_formats[] = {
	{
		.code = MEDIA_BUS_FMT_UYVY8_2X8,
		.bus_width = 8,
		.bpp = 16,
		.pdformat = DATAFMT_PDFMT_YCBCRFMT_422_8_BIT,
		.pdataf = CONFCTL_PDATAF_MODE0,
		.ppp = 2,
	}, {
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.bus_width = 16,
		.bpp = 16,
		.pdformat = DATAFMT_PDFMT_YCBCRFMT_422_8_BIT,
		.pdataf = CONFCTL_PDATAF_MODE1,
		.ppp = 1,
	}, {
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.bus_width = 16,
		.bpp = 16,
		.pdformat = DATAFMT_PDFMT_YCBCRFMT_422_8_BIT,
		.pdataf = CONFCTL_PDATAF_MODE2,
		.ppp = 1,
	}, {
		.code = MEDIA_BUS_FMT_UYVY10_2X10,
		.bus_width = 10,
		.bpp = 20,
		.pdformat = DATAFMT_PDFMT_YCBCRFMT_422_10_BIT,
		.pdataf = CONFCTL_PDATAF_MODE0, /* don't care */
		.ppp = 2,
	}, {
		/* in datasheet listed as YUV444 */
		.code = MEDIA_BUS_FMT_GBR888_1X24,
		.bus_width = 24,
		.bpp = 24,
		.pdformat = DATAFMT_PDFMT_YCBCRFMT_444,
		.pdataf = CONFCTL_PDATAF_MODE0, /* don't care */
		.ppp = 2,
		.csitx_only = true,
	},
};

/* --------------- HELPERS ------------ */
static void
tc358748_dump_csi(struct device *dev,
		  struct tc358748_csi_param *csi_setting)
{
	dev_dbg(dev, "Speed-Range value %u\n", csi_setting->speed_range);
	dev_dbg(dev, "Unit Clock %u Hz\n", csi_setting->unit_clk_hz);
	dev_dbg(dev, "Unit Clock Mul %u\n", csi_setting->unit_clk_mul);
	dev_dbg(dev, "CSI speed/lane %u bps/lane\n",
		csi_setting->speed_per_lane);
	dev_dbg(dev, "CSI lanes %u\n", csi_setting->lane_num);
	dev_dbg(dev, "CSI clock during LP %sabled\n",
		csi_setting->is_continuous_clk ? "en" : "dis");

	dev_dbg(dev, "lineinitcnt %u\n", csi_setting->lineinitcnt);
	dev_dbg(dev, "lptxtimecnt %u\n", csi_setting->lptxtimecnt);
	dev_dbg(dev, "tclk_preparecnt %u\n", csi_setting->tclk_preparecnt);
	dev_dbg(dev, "tclk_zerocnt %u\n", csi_setting->tclk_zerocnt);
	dev_dbg(dev, "tclk_trailcnt %u\n", csi_setting->tclk_trailcnt);
	dev_dbg(dev, "ths_preparecnt %u\n", csi_setting->ths_preparecnt);
	dev_dbg(dev, "ths_zerocnt %u\n", csi_setting->ths_zerocnt);
	dev_dbg(dev, "twakeupcnt %u\n", csi_setting->twakeupcnt);
	dev_dbg(dev, "tclk_postcnt %u\n", csi_setting->tclk_postcnt);
	dev_dbg(dev, "ths_trailcnt %u\n", csi_setting->ths_trailcnt);
	dev_dbg(dev, "csi_hs_lp_hs_ps %u (%u us)\n",
		csi_setting->csi_hs_lp_hs_ps,
		csi_setting->csi_hs_lp_hs_ps / 1000);
}

static void
tc358748_dump_pll(struct device *dev, struct tc358748_state *state)
{
	dev_dbg(dev, "refclk %lu Hz\n", clk_get_rate(state->refclk));
	dev_dbg(dev, "pll input clock %u Hz\n", state->pllinclk_hz);
	dev_dbg(dev, "PLL_PRD %u\n", state->pll_prd - 1);
	dev_dbg(dev, "PLL_FBD %u\n", state->pll_fbd - 1);
}

static inline struct tc358748_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358748_state, sd);
}

/* Find a data format by a pixel code */
static int tc358748_format_supported(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tc358748_formats); i++)
		if (tc358748_formats[i].code == code)
			return 0;

	return -EINVAL;
}

static struct tc358748_csi_param *
tc358748_g_cur_csi_settings(struct tc358748_state *state)
{
	int cur_freq = v4l2_ctrl_g_ctrl(state->link_freq);

	return &state->link_freq_settings[cur_freq];
}

static const struct tc358748_mbus_fmt *tc358748_get_format(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tc358748_formats); i++)
		if (tc358748_formats[i].code == code)
			return &tc358748_formats[i];

	return NULL;
}

static int
tc358748_adjust_fifo_size(struct tc358748_state *state,
			  const struct tc358748_mbus_fmt *format,
			  struct tc358748_csi_param *csi_settings,
			  int width, u16 *fifo_size)
{
	struct device *dev = &state->i2c_client->dev;
	int c_hactive_ps_diff, c_lp_active_ps_diff, c_fifo_delay_ps_diff;
	unsigned int p_hactive_ps, p_hblank_ps, p_htotal_ps;
	unsigned int c_hactive_ps, c_lp_active_ps, c_fifo_delay_ps;
	unsigned int csi_bps, csi_bps_period_ps;
	unsigned int csi_hsclk, csi_hsclk_period_ps;
	unsigned int pclk_period_ps;
	unsigned int _fifo_size;

	pclk_period_ps = 1000000000 / (state->pclk / 1000);
	csi_bps = csi_settings->speed_per_lane * csi_settings->lane_num;
	csi_bps_period_ps = 1000000000 / (csi_bps / 1000);
	csi_hsclk = csi_settings->speed_per_lane >> 3;
	csi_hsclk_period_ps = 1000000000 / (csi_hsclk / 1000);

	/*
	 * Calculation:
	 * p_hactive_ps = pclk_period_ps * pclk_per_pixel * h_active_pixel
	 */
	p_hactive_ps = pclk_period_ps * format->ppp * width;

	/*
	 * Calculation:
	 * p_hblank_ps = pclk_period_ps * h_blank_pixel
	 */
	p_hblank_ps = pclk_period_ps * state->hblank;
	p_htotal_ps = p_hblank_ps + p_hactive_ps;

	/*
	 * Adjust the fifo size to adjust the csi timing. Hopefully we can find
	 * a fifo size where the parallel input timings and the csi tx timings
	 * fit together.
	 */
	for (_fifo_size = 1; _fifo_size < TC358748_MAX_FIFO_SIZE;
	     _fifo_size++) {
		/*
		 * Calculation:
		 * c_fifo_delay_ps = (fifo_size * 32) / parallel_bus_width *
		 *		     pclk_period_ps + 4 * csi_hsclk_period_ps
		 */
		c_fifo_delay_ps = _fifo_size * 32 * pclk_period_ps;
		c_fifo_delay_ps /= format->bus_width;
		c_fifo_delay_ps += 4 * csi_hsclk_period_ps;

		/*
		 * Calculation:
		 * c_hactive_ps = csi_bps_period_ps * image_bpp * h_active_pixel
		 *		  + c_fifo_delay
		 */
		c_hactive_ps = csi_bps_period_ps * format->bpp * width;
		c_hactive_ps += c_fifo_delay_ps;

		/*
		 * Calculation:
		 * c_lp_active_ps = p_htotal_ps - c_hactive_ps
		 */
		c_lp_active_ps = p_htotal_ps - c_hactive_ps;

		c_hactive_ps_diff = c_hactive_ps - p_hactive_ps;
		c_fifo_delay_ps_diff = p_htotal_ps - c_hactive_ps;
		c_lp_active_ps_diff =
			c_lp_active_ps - csi_settings->csi_hs_lp_hs_ps;

		if (c_hactive_ps_diff > 0 &&
		    c_fifo_delay_ps_diff > 0 &&
		    c_lp_active_ps_diff > 0)
			break;
	}
	/*
	 * If we can't transfer the image using this csi link frequency try to
	 * use another link freq.
	 */

	dev_dbg(dev, "%s: found fifo-size %u\n", __func__, _fifo_size);
	*fifo_size = _fifo_size;
	return _fifo_size == TC358748_MAX_FIFO_SIZE ? -EINVAL : 0;
}

static int
tc358748_adjust_timings(struct tc358748_state *state,
			const struct tc358748_mbus_fmt *format,
			int *width, u16 *fifo_size)
{

	int cur_freq = v4l2_ctrl_g_ctrl(state->link_freq);
	int freq = cur_freq;
	struct tc358748_csi_param *csi_lane_setting;
	int err;
	int _width;

	/*
	 * Adjust timing:
	 * 1) Try to use the desired width and the current csi-link-frequency
	 * 2) If this doesn't fit try other csi-link-frequencies
	 * 3) If this doesn't fit too, reducing the desired width and test
	 *    it again width the current csi-link-frequency
	 * 4) Goto step 2 if it doesn't fit at all
	 */
	for (_width = *width; _width > 0; _width -= 10) {
		csi_lane_setting = &state->link_freq_settings[cur_freq];
		err = tc358748_adjust_fifo_size(state, format, csi_lane_setting,
						_width, fifo_size);
		if (!err)
			goto out;

		for (freq = 0; freq < state->link_frequencies_num; freq++) {
			if (freq == cur_freq)
				continue;

			csi_lane_setting = &state->link_freq_settings[freq];
			err = tc358748_adjust_fifo_size(state, format,
							csi_lane_setting,
							_width, fifo_size);
			if (!err)
				goto out;
		}
	}

out:
	*width = _width;
	return freq;
}



/* --------------- i2c helper ------------ */

static void i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358748_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	u8 buf[2] = { reg >> 8, reg & 0xff };
	u8 data[I2C_MAX_XFER_SIZE];

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = n,
			.buf = data,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		v4l2_err(sd, "%s: reading register 0x%x from 0x%x failed\n",
			 __func__, reg, client->addr);
	}

	switch (n) {
	case 1:
		values[0] = data[0];
		break;
	case 2:
		values[0] = data[1];
		values[1] = data[0];
		break;
	case 4:
		values[0] = data[1];
		values[1] = data[0];
		values[2] = data[3];
		values[3] = data[2];
		break;
	default:
		v4l2_info(sd, "unsupported I2C read %d bytes from address 0x%04x\n",
			  n, reg);
	}

	if (debug < 3)
		return;

	switch (n) {
	case 1:
		v4l2_info(sd, "I2C read 0x%04x = 0x%02x",
			  reg, data[0]);
		break;
	case 2:
		v4l2_info(sd, "I2C read 0x%04x = 0x%02x%02x",
			  reg, data[0], data[1]);
		break;
	case 4:
		v4l2_info(sd, "I2C read 0x%04x = 0x%02x%02x%02x%02x",
			  reg, data[2], data[3], data[0], data[1]);
		break;
	default:
		v4l2_info(sd, "I2C unsupported read %d bytes from address 0x%04x\n",
			  n, reg);
	}
}

static void i2c_wr(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358748_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	struct i2c_msg msg;
	u8 data[I2C_MAX_XFER_SIZE];

	if ((2 + n) > I2C_MAX_XFER_SIZE) {
		n = I2C_MAX_XFER_SIZE - 2;
		v4l2_warn(sd, "i2c wr reg=%04x: len=%d is too big!\n",
			  reg, 2 + n);
	}

	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2 + n;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg & 0xff;

	switch (n) {
	case 1:
		data[2 + 0] = values[0];
		break;
	case 2:
		data[2 + 0] = values[1];
		data[2 + 1] = values[0];
		break;
	case 4:
		data[2 + 0] = values[1];
		data[2 + 1] = values[0];
		data[2 + 2] = values[3];
		data[2 + 3] = values[2];
		break;
	default:
		v4l2_info(sd, "unsupported I2C write %d bytes from address 0x%04x\n",
			  n, reg);
	}

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		v4l2_err(sd, "%s: writing register 0x%x from 0x%x failed\n",
			 __func__, reg, client->addr);
		return;
	}

	if (debug < 3)
		return;

	switch (n) {
	case 1:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x", reg, data[2 + 0]);
		break;
	case 2:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x%02x", reg, data[2 + 0],
			  data[2 + 1]);
		break;
	case 4:
		v4l2_info(sd, "I2C write 0x%04x = 0x%02x%02x%02x%02x", reg,
			  data[2 + 2], data[2 + 3], data[2 + 0], data[2 + 1]);
		break;
	default:
		v4l2_info(sd, "I2C unsupported write %d bytes from address 0x%04x\n",
			  n, reg);
	}
}

static noinline u32 i2c_rdreg(struct v4l2_subdev *sd, u16 reg, u32 n)
{
	__le32 val = 0;

	i2c_rd(sd, reg, (u8 __force *)&val, n);

	return le32_to_cpu(val);
}

static noinline void i2c_wrreg(struct v4l2_subdev *sd, u16 reg, u32 val, u32 n)
{
	__le32 raw = cpu_to_le32(val);

	i2c_wr(sd, reg, (u8 __force *)&raw, n);
}

static u16 __maybe_unused i2c_rd8(struct v4l2_subdev *sd, u16 reg)
{
	return i2c_rdreg(sd, reg, 1);
}

static u16 __maybe_unused i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	return i2c_rdreg(sd, reg, 2);
}

static u32 __maybe_unused i2c_rd32(struct v4l2_subdev *sd, u16 reg)
{
	return i2c_rdreg(sd, reg, 4);
}

static void __maybe_unused i2c_wr8(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	i2c_wrreg(sd, reg, val, 1);
}

static void i2c_wr16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	i2c_wrreg(sd, reg, val, 2);
}

static void i2c_wr16_and_or(struct v4l2_subdev *sd, u16 reg, u32 mask, u16 val)
{
	u16 m = (u16) mask;

	i2c_wrreg(sd, reg, (i2c_rd16(sd, reg) & m) | val, 2);
}

static void i2c_wr32(struct v4l2_subdev *sd, u16 reg, u32 val)
{
	i2c_wrreg(sd, reg, val, 4);
}

/* --------------- init --------------- */

static void
tc358748_wr_csi_control(struct v4l2_subdev *sd, int val)
{
	struct tc358748_state *state = to_state(sd);
	u32 _val;

	val &= CSI_CONFW_DATA_MASK;
	_val = CSI_CONFW_MODE_SET_MASK | CSI_CONFW_ADDRESS_CSI_CONTROL_MASK |
		val;

	dev_dbg(&state->i2c_client->dev, "CSI_CONFW 0x%04x\n", _val);
	i2c_wr32(sd, CSI_CONFW, _val);
}

static inline void tc358748_sleep_mode(struct v4l2_subdev *sd, int enable)
{
	i2c_wr16_and_or(sd, SYSCTL, ~SYSCTL_SLEEP_MASK,
			enable ? SYSCTL_SLEEP_MASK : 0);
}

static inline void tc358748_sreset(struct v4l2_subdev *sd)
{
	i2c_wr16(sd, SYSCTL, SYSCTL_SRESET_MASK);
	udelay(10);
	i2c_wr16(sd, SYSCTL, 0);
}

static inline void tc358748_enable_stream(struct v4l2_subdev *sd, int enable)
{
	struct tc358748_state *state = to_state(sd);

	dev_dbg(&state->i2c_client->dev, "%sable\n", enable ? "en" : "dis");

	mutex_lock(&state->confctl_mutex);
	if (!enable) {
		i2c_wr16_and_or(sd, PP_MISC, ~PP_MISC_FRMSTOP_MASK,
				PP_MISC_FRMSTOP_MASK);
		i2c_wr16_and_or(sd, CONFCTL, ~CONFCTL_PPEN_MASK, 0);
		i2c_wr16_and_or(sd, PP_MISC, ~PP_MISC_RSTPTR_MASK,
				PP_MISC_RSTPTR_MASK);

		i2c_wr32(sd, CSIRESET, (CSIRESET_RESET_CNF_MASK |
					CSIRESET_RESET_MODULE_MASK));
		i2c_wr16(sd, DBG_ACT_LINE_CNT, 0);
	} else {
		i2c_wr16(sd, PP_MISC, 0);
		i2c_wr16_and_or(sd, CONFCTL, ~CONFCTL_PPEN_MASK,
				CONFCTL_PPEN_MASK);
	}
	mutex_unlock(&state->confctl_mutex);
}

static void tc358748_set_pll(struct v4l2_subdev *sd)
{
	struct tc358748_state *state = to_state(sd);
	struct tc358748_csi_param *csi_setting =
		tc358748_g_cur_csi_settings(state);
	struct device *dev = &state->i2c_client->dev;
	struct tc358743_platform_data *pdata = &state->pdata;
	u16 pllctl0 = i2c_rd16(sd, PLLCTL0);
	u16 pllctl1 = i2c_rd16(sd, PLLCTL1);
	u16 pll_frs = csi_setting->speed_range;
	u16 pllctl0_new;

	/*
	 * Calculation:
	 * speed_per_lane = (pllinclk_hz * (fbd + 1)) / 2^frs
	 *
	 * Calculation used by REF_02:
	 * speed_per_lane = (pllinclk_hz * fbd) / 2^frs
	 */
	pdata->pll_fbd = csi_setting->speed_per_lane / pdata->refclk_hz;
	pdata->pll_fbd <<= pll_frs;

	pllctl0_new = PLLCTL0_PLL_PRD_SET(pdata->pll_prd) |
		      PLLCTL0_PLL_FBD_SET(pdata->pll_prd);

	/*
	 * Only rewrite when needed (new value or disabled), since rewriting
	 * triggers another format change event.
	 */
	if ((pllctl0 != pllctl0_new) ||
	    ((pllctl1 & PLLCTL1_PLL_EN_MASK) == 0)) {
		u16 pllctl1_mask = (u16) ~(PLLCTL1_PLL_FRS_MASK |
					   PLLCTL1_RESETB_MASK  |
					   PLLCTL1_PLL_EN_MASK);
		u16 pllctl1_val = PLLCTL1_PLL_FRS_SET(pll_frs) |
				  PLLCTL1_RESETB_MASK | PLLCTL1_PLL_EN_MASK;

		dev_dbg(dev, "updating PLL clock\n");
		i2c_wr16(sd, PLLCTL0, pllctl0_new);
		i2c_wr16_and_or(sd, PLLCTL1, pllctl1_mask, pllctl1_val);
		udelay(1000);
		i2c_wr16_and_or(sd, PLLCTL1, ~PLLCTL1_CKEN_MASK,
				PLLCTL1_CKEN_MASK);
	}

	tc358748_dump_pll(dev, state);
}

static void tc358748_set_csi_color_space(struct v4l2_subdev *sd)
{
	struct tc358748_state *state = to_state(sd);
	const struct tc358748_mbus_fmt *tc358748_fmt =
		tc358748_get_format(state->fmt.code);

	/* currently no self defined csi user data type id's are supported */
	mutex_lock(&state->confctl_mutex);
	i2c_wr16_and_or(sd, DATAFMT,
			~(DATAFMT_PDFMT_MASK | DATAFMT_UDT_EN_MASK),
			DATAFMT_PDFMT_SET(tc358748_fmt->pdformat));
	i2c_wr16_and_or(sd, CONFCTL, ~CONFCTL_PDATAF_MASK,
			CONFCTL_PDATAF_SET(tc358748_fmt->pdataf));
	mutex_unlock(&state->confctl_mutex);
}

static void tc38764_debug_pattern_80(struct v4l2_subdev *sd)
{
	int i;

	i2c_wr16(sd, DBG_ACT_LINE_CNT, 0x8000);
	i2c_wr16(sd, DBG_LINE_WIDTH, 0x0396);
	i2c_wr16(sd, DBG_VERT_BLANK_LINE_CNT, 0x0000);

	for (i = 0; i < 80; i++)
		i2c_wr16(sd, DBG_VIDEO_DATA, 0xff7f);
	i2c_wr16(sd, DBG_VIDEO_DATA, 0xff00);
	for (i = 0; i < 40; i++)
		i2c_wr16(sd, DBG_VIDEO_DATA, 0xffff);
	i2c_wr16(sd, DBG_VIDEO_DATA, 0xc0ff);
	for (i = 0; i < 40; i++)
		i2c_wr16(sd, DBG_VIDEO_DATA, 0xc000);
	for (i = 0; i < 80; i++)
		i2c_wr16(sd, DBG_VIDEO_DATA, 0x7f00);
	for (i = 0; i < 80; i++)
		i2c_wr16(sd, DBG_VIDEO_DATA, 0x7fff);
	i2c_wr16(sd, DBG_VIDEO_DATA, 0x0000);
	for (i = 0; i < 40; i++)
		i2c_wr16(sd, DBG_VIDEO_DATA, 0x00ff);
	i2c_wr16(sd, DBG_VIDEO_DATA, 0x00ff);
	for (i = 0; i < 40; i++)
		i2c_wr16(sd, DBG_VIDEO_DATA, 0x0000);
	i2c_wr16(sd, DBG_VIDEO_DATA, 0x007f);

	i2c_wr16(sd, DBG_ACT_LINE_CNT, 0xC1DF);
}

static void tc358748_enable_csi_lanes(struct v4l2_subdev *sd, int enable)
{
	struct tc358748_state *state = to_state(sd);
	struct tc358748_csi_param *csi_setting =
		tc358748_g_cur_csi_settings(state);
	unsigned int lanes = csi_setting->lane_num;
	u32 val = 0;

	if (lanes < 1 || !enable)
		i2c_wr32(sd, CLW_CNTRL, CLW_CNTRL_CLW_LANEDISABLE_MASK);
	if (lanes < 1 || !enable)
		i2c_wr32(sd, D0W_CNTRL, D0W_CNTRL_D0W_LANEDISABLE_MASK);
	if (lanes < 2 || !enable)
		i2c_wr32(sd, D1W_CNTRL, D1W_CNTRL_D1W_LANEDISABLE_MASK);
	if (lanes < 3 || !enable)
		i2c_wr32(sd, D2W_CNTRL, D2W_CNTRL_D2W_LANEDISABLE_MASK);
	if (lanes < 4 || !enable)
		i2c_wr32(sd, D3W_CNTRL, D2W_CNTRL_D3W_LANEDISABLE_MASK);

	if (lanes > 0 && enable)
		val |= HSTXVREGEN_CLM_HSTXVREGEN_MASK |
			HSTXVREGEN_D0M_HSTXVREGEN_MASK;
	if (lanes > 1 && enable)
		val |= HSTXVREGEN_D1M_HSTXVREGEN_MASK;
	if (lanes > 2 && enable)
		val |= HSTXVREGEN_D2M_HSTXVREGEN_MASK;
	if (lanes > 3 && enable)
		val |= HSTXVREGEN_D3M_HSTXVREGEN_MASK;

	i2c_wr32(sd, HSTXVREGEN, val);
}

static void tc358748_set_csi(struct v4l2_subdev *sd)
{
	struct tc358748_state *state = to_state(sd);
	struct tc358748_csi_param *csi_setting =
		tc358748_g_cur_csi_settings(state);
	bool en_continuous_clk = csi_setting->is_continuous_clk;
	u32 val;

	val = TCLK_HEADERCNT_TCLK_ZEROCNT_SET(csi_setting->tclk_zerocnt) |
	      TCLK_HEADERCNT_TCLK_PREPARECNT_SET(csi_setting->tclk_preparecnt);
	i2c_wr32(sd, TCLK_HEADERCNT, val);
	val = THS_HEADERCNT_THS_ZEROCNT_SET(csi_setting->ths_zerocnt) |
	      THS_HEADERCNT_THS_PREPARECNT_SET(csi_setting->ths_preparecnt);
	i2c_wr32(sd, THS_HEADERCNT, val);
	i2c_wr32(sd, TWAKEUP, csi_setting->twakeupcnt);
	i2c_wr32(sd, TCLK_POSTCNT, csi_setting->tclk_postcnt);
	i2c_wr32(sd, THS_TRAILCNT, csi_setting->ths_trailcnt);
	i2c_wr32(sd, LINEINITCNT, csi_setting->lineinitcnt);
	i2c_wr32(sd, LPTXTIMECNT, csi_setting->lptxtimecnt);
	i2c_wr32(sd, TCLK_TRAILCNT, csi_setting->tclk_trailcnt);
	i2c_wr32(sd, TXOPTIONCNTRL,
		 en_continuous_clk ? TXOPTIONCNTRL_CONTCLKMODE_MASK : 0);

	if (state->test)
		tc38764_debug_pattern_80(sd);

	tc358748_dump_csi(&state->i2c_client->dev, csi_setting);
}

static void tc358748_enable_csi_module(struct v4l2_subdev *sd, int enable)
{
	struct tc358748_state *state = to_state(sd);
	struct tc358748_csi_param *csi_setting =
		tc358748_g_cur_csi_settings(state);
	unsigned int lanes = csi_setting->lane_num;
	u32 val;

	if (!enable)
		return;

	i2c_wr32(sd, STARTCNTRL, STARTCNTRL_START_MASK);
	i2c_wr32(sd, CSI_START, CSI_START_STRT_MASK);

	val = CSI_CONTROL_NOL_1_MASK;
	if (lanes == 2)
		val = CSI_CONTROL_NOL_2_MASK;
	else if (lanes == 3)
		val = CSI_CONTROL_NOL_3_MASK;
	else if (lanes == 4)
		val = CSI_CONTROL_NOL_4_MASK;

	val |= CSI_CONTROL_CSI_MODE_MASK | CSI_CONTROL_TXHSMD_MASK;
	tc358748_wr_csi_control(sd, val);
}

static void tc358748_set_buffers(struct v4l2_subdev *sd)
{
	struct tc358748_state *state = to_state(sd);
	struct device *dev = &state->i2c_client->dev;
	const struct tc358748_mbus_fmt *tc358748_mbusfmt =
		tc358748_get_format(state->fmt.code);
	unsigned int byte_per_line =
		(state->fmt.width * tc358748_mbusfmt->bpp) / 8;

	i2c_wr16(sd, FIFOCTL, state->vb_fifo);
	i2c_wr16(sd, WORDCNT, byte_per_line);
	dev_dbg(dev, "FIFOCTL 0x%02x: WORDCNT 0x%02x\n",
		state->vb_fifo, byte_per_line);
}

/* --------------- CORE OPS --------------- */

static int tc358748_log_status(struct v4l2_subdev *sd)
{
	struct tc358748_state *state = to_state(sd);
	uint16_t sysctl = i2c_rd16(sd, SYSCTL);

	v4l2_info(sd, "-----Chip status-----\n");
	v4l2_info(sd, "Chip ID: 0x%02x\n",
		  (i2c_rd16(sd, CHIPID) & CHIPID_CHIPID_MASK) >> 8);
	v4l2_info(sd, "Chip revision: 0x%02x\n",
		  i2c_rd16(sd, CHIPID) & CHIPID_REVID_MASK);
	v4l2_info(sd, "Sleep mode: %s\n", sysctl & SYSCTL_SLEEP_MASK ?
		  "on" : "off");

	v4l2_info(sd, "-----CSI-TX status-----\n");
	v4l2_info(sd, "Waiting for particular sync signal: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & CSI_STATUS_S_WSYNC_MASK) ?
			"yes" : "no");
	v4l2_info(sd, "Transmit mode: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & CSI_STATUS_S_TXACT_MASK) ?
			"yes" : "no");
	v4l2_info(sd, "Stopped: %s\n",
			(i2c_rd16(sd, CSI_STATUS) & CSI_STATUS_S_HLT_MASK) ?
			"yes" : "no");
	v4l2_info(sd, "Color space: %s\n",
			state->fmt.code == MEDIA_BUS_FMT_UYVY8_2X8 ?
			"YCbCr 422 8-bit" : "Unsupported");

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static void tc358748_print_register_map(struct v4l2_subdev *sd)
{
	v4l2_info(sd, "0x0000-0x0050: Global Register\n");
	v4l2_info(sd, "0x0056-0x0070: Rx Control Registers\n");
	v4l2_info(sd, "0x0080-0x00F8: Rx Status Registers\n");
	v4l2_info(sd, "0x0100-0x0150: Tx D-PHY Register\n");
	v4l2_info(sd, "0x0204-0x0238: Tx PPI Register\n");
	v4l2_info(sd, "0x040c-0x0518: Tx Control Register\n");
}

static int tc358748_get_reg_size(u16 address)
{
	if (address <= 0x00ff)
		return 2;
	else if ((address >= 0x0100) && (address <= 0x05FF))
		return 4;
	else
		return 1;
}

static int tc358748_g_register(struct v4l2_subdev *sd,
			       struct v4l2_dbg_register *reg)
{
	if (reg->reg > 0xffff) {
		tc358748_print_register_map(sd);
		return -EINVAL;
	}

	reg->size = tc358748_get_reg_size(reg->reg);

	reg->val = i2c_rdreg(sd, reg->reg, reg->size);

	return 0;
}

static int tc358748_s_register(struct v4l2_subdev *sd,
			       const struct v4l2_dbg_register *reg)
{
	if (reg->reg > 0xffff) {
		tc358748_print_register_map(sd);
		return -EINVAL;
	}

	i2c_wrreg(sd, (u16)reg->reg, reg->val,
			tc358748_get_reg_size(reg->reg));

	return 0;
}
#endif

/* --------------- video ops --------------- */

static int tc358748_g_mbus_config(struct v4l2_subdev *sd,
			     struct v4l2_mbus_config *cfg)
{
	struct tc358748_state *state = to_state(sd);
	struct tc358748_csi_param *csi_setting =
		tc358748_g_cur_csi_settings(state);

	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = csi_setting->is_continuous_clk ?
			V4L2_MBUS_CSI2_CONTINUOUS_CLOCK :
			V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK;

	switch (csi_setting->lane_num) {
	case 1:
		cfg->flags |= V4L2_MBUS_CSI2_1_LANE;
		break;
	case 2:
		cfg->flags |= V4L2_MBUS_CSI2_2_LANE;
		break;
	case 3:
		cfg->flags |= V4L2_MBUS_CSI2_3_LANE;
		break;
	case 4:
		cfg->flags |= V4L2_MBUS_CSI2_4_LANE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tc358748_s_power(struct v4l2_subdev *sd, int on)
{
	struct tc358748_state *state = to_state(sd);

	/*
	 * REF_01:
	 * Softreset don't reset configuration registers content but is needed
	 * during power-on to trigger a csi LP-11 state change and during
	 * power-off to disable the csi-module.
	 */
	tc358748_sreset(sd);

	if (state->fmt_changed) {
		tc358748_set_buffers(sd);
		tc358748_set_csi(sd);
		tc358748_set_csi_color_space(sd);

		/* as recommend in REF_01 */
		tc358748_sleep_mode(sd, 1);
		tc358748_set_pll(sd);
		tc358748_sleep_mode(sd, 0);

		state->fmt_changed = false;
	}

	tc358748_enable_csi_lanes(sd, on);
	tc358748_enable_csi_module(sd, on);
	tc358748_sleep_mode(sd, !on);

	return 0;
}

static int tc358748_s_stream(struct v4l2_subdev *sd, int enable)
{
	tc358748_enable_stream(sd, enable);

	return 0;
}

/* --------------- pad ops --------------- */

static int tc358748_enum_mbus_code(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= ARRAY_SIZE(tc358748_formats))
		return -EINVAL;

	code->code = tc358748_formats[code->index].code;

	return 0;
}

static struct v4l2_mbus_framefmt *
__tc358748_get_pad_format(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  unsigned int pad, u32 which)
{
	struct tc358748_state *state = to_state(sd);

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &state->fmt;
	default:
		return NULL;
	}
}

static int tc358748_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct tc358748_state *state = to_state(sd);

	if (format->pad != 0 && format->pad != 1)
		return -EINVAL;

	format->format.code = state->fmt.code;
	format->format.width = state->fmt.width;
	format->format.height = state->fmt.height;
	format->format.field = state->fmt.field;

	return 0;
}

static int tc358748_set_fmt(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_format *format)
{
	struct tc358748_state *state = to_state(sd);
	struct device *dev = &state->i2c_client->dev;
	struct media_pad *pad = &state->pads[format->pad];
	struct media_pad *remote_sensor_pad =
		media_entity_remote_pad(&state->pads[0]);
	struct v4l2_subdev *sensor_sd;
	struct v4l2_mbus_framefmt *mbusformat;
	const struct tc358748_mbus_fmt *tc358748_mbusformat;
	struct v4l2_ctrl *ctrl;
	unsigned int pclk, hblank;
	int new_freq, cur_freq = v4l2_ctrl_g_ctrl(state->link_freq);
	u16 vb_fifo;

	if (pad->flags == MEDIA_PAD_FL_SOURCE)
		return tc358748_get_fmt(sd, cfg, format);

	mbusformat = __tc358748_get_pad_format(sd, cfg, format->pad,
					       format->which);
	if (!mbusformat)
		return -EINVAL;

	tc358748_mbusformat = tc358748_get_format(format->format.code);
	if (!tc358748_mbusformat) {
		format->format.code = tc358748_def_fmt.code;
		tc358748_mbusformat = tc358748_get_format(format->format.code);
	}

	/*
	 * Some sensors change their hblank and pclk value on different formats,
	 * so we need to request it again.
	 */
	sensor_sd = media_entity_to_v4l2_subdev(remote_sensor_pad->entity);
	ctrl = v4l2_ctrl_find(sensor_sd->ctrl_handler, V4L2_CID_PIXEL_RATE);
	pclk = v4l2_ctrl_g_ctrl_int64(ctrl);
	if (pclk != state->pclk) {
		dev_dbg(dev, "Update pclk from %u to %u\n", state->pclk, pclk);
		state->pclk = pclk;
	}
	ctrl = v4l2_ctrl_find(sensor_sd->ctrl_handler, V4L2_CID_HBLANK);
	hblank = v4l2_ctrl_g_ctrl(ctrl);
	if (hblank != state->hblank) {
		dev_dbg(dev, "Update hblank from %u to %u\n", state->hblank,
			hblank);
		state->hblank = hblank;
	}

	/*
	 * Normaly the HW has no size limitations but we have to check if the
	 * csi timings are valid for this size. The timings can be adjust by the
	 * fifo size. If this doesn't work we have to do this check again with a
	 * other csi link frequency if it is possible.
	 */
	new_freq = tc358748_adjust_timings(state, tc358748_mbusformat,
					   &format->format.width, &vb_fifo);

	/* Currently only a few YUV based formats are supported */
	if (tc358748_format_supported(format->format.code))
		format->format.code = MEDIA_BUS_FMT_UYVY8_2X8;

	/* Currently only non interleaved images are supported */
	format->format.field = V4L2_FIELD_NONE;

	*mbusformat = format->format;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		state->fmt_changed = true;
		state->vb_fifo = vb_fifo;
		if (new_freq != cur_freq)
			v4l2_ctrl_s_ctrl(state->link_freq, new_freq);
	}

	return 0;
}

static int
tc358748_link_validate(struct v4l2_subdev *sd, struct media_link *link,
		       struct v4l2_subdev_format *source_fmt,
		       struct v4l2_subdev_format *sink_fmt)
{
	struct tc358748_state *state = to_state(sd);
	struct device *dev = &state->i2c_client->dev;
	const struct tc358748_mbus_fmt *tc358748_mbusformat;
	struct v4l2_subdev *sensor_sd;
	struct v4l2_ctrl *ctrl;
	unsigned int pclk, pclk_old = state->pclk;
	unsigned int hblank, hblank_old = state->hblank;
	int new_freq;
	u16 vb_fifo;

	/*
	 * Only validate if the timings are changed, after the link was already
	 * initialized. This can be happen if the parallel sensor frame interval
	 * is changed. Format checks are perfomed by the common code.
	 */

	tc358748_mbusformat = tc358748_get_format(sink_fmt->format.code);
	if (!tc358748_mbusformat)
		return -EINVAL; /* Format was changed too and is invalid */

	sensor_sd = media_entity_to_v4l2_subdev(link->source->entity);
	ctrl = v4l2_ctrl_find(sensor_sd->ctrl_handler, V4L2_CID_PIXEL_RATE);
	pclk = v4l2_ctrl_g_ctrl_int64(ctrl);
	if (pclk != state->pclk) {
		dev_dbg(dev, "%s pixel rate is changed\n", sensor_sd->name);
		state->pclk = pclk;
	}

	ctrl = v4l2_ctrl_find(sensor_sd->ctrl_handler, V4L2_CID_HBLANK);
	hblank = v4l2_ctrl_g_ctrl(ctrl);
	if (hblank != state->hblank) {
		dev_dbg(dev,
			"%s hblank interval is changed\n", sensor_sd->name);
		state->hblank = hblank;
	}

	new_freq = tc358748_adjust_timings(state, tc358748_mbusformat,
					   &source_fmt->format.width, &vb_fifo);

	if (new_freq != v4l2_ctrl_g_ctrl(state->link_freq)) {
		/*
		 * This can lead into undefined behaviour, so we don't support
		 * dynamic changes due to a to late re-configuration.
		 */
		dev_err(dev,
			"%s format can't be applied re-run the whole s_fmt\n",
			sensor_sd->name);
		state->pclk = pclk_old;
		state->hblank = hblank_old;

		return -EINVAL;
	}

	state->fmt_changed = true;
	state->vb_fifo = vb_fifo;

	return 0;
}

static int tc358764_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tc358748_state *state = container_of(ctrl->handler,
					       struct tc358748_state, hdl);
	struct device *dev = &state->i2c_client->dev;

	switch (ctrl->id) {
	case V4L2_CID_LINK_FREQ:
		dev_info(dev, "Update link-frequency %llu -> %llu\n",
			 state->link_frequencies[ctrl->cur.val],
			 state->link_frequencies[ctrl->val]);

		return 0;
	case V4L2_CID_TEST_PATTERN:
		state->test = ctrl->val;
		return 0;
	}

	return -EINVAL;
}

static int tc358748_link_setup(struct media_entity *entity,
			       const struct media_pad *local,
			       const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct v4l2_subdev *ps_sd = media_entity_to_v4l2_subdev(remote->entity);
	struct tc358748_state *state = to_state(sd);
	struct v4l2_ctrl *ctrl;

	/* no special requirements on source pads */
	if (local->flags & MEDIA_PAD_FL_SOURCE)
		return 0;

	dev_dbg(sd->dev, "link setup '%s':%d->'%s':%d[%d]",
		remote->entity->name, remote->index, local->entity->name,
		local->index, flags & MEDIA_LNK_FL_ENABLED);

	/*
	 * The remote parallel sensor must support pixel rate and hblank query
	 */
	ctrl = v4l2_ctrl_find(ps_sd->ctrl_handler, V4L2_CID_PIXEL_RATE);
	if (!ctrl) {
		dev_err(sd->dev, "Subdev %s must support V4L2_CID_PIXEL_RATE\n",
			ps_sd->name);
		return -EINVAL;
	}
	state->pclk = v4l2_ctrl_g_ctrl_int64(ctrl);

	ctrl = v4l2_ctrl_find(ps_sd->ctrl_handler, V4L2_CID_HBLANK);
	if (!ctrl) {
		dev_err(sd->dev, "Subdev %s must support V4L2_CID_HBLANK\n",
			ps_sd->name);
		return -EINVAL;
	}
	state->hblank = v4l2_ctrl_g_ctrl(ctrl);

	return 0;
}

/* -------------------------------------------------------------------------- */

static const struct v4l2_ctrl_ops tc358764_ctrl_ops = {
	.s_ctrl = tc358764_s_ctrl,
};

static const struct v4l2_subdev_core_ops tc358748_core_ops = {
	.log_status = tc358748_log_status,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = tc358748_g_register,
	.s_register = tc358748_s_register,
#endif
	.s_power = tc358748_s_power,
};

static const struct v4l2_subdev_video_ops tc358748_video_ops = {
	.g_mbus_config = tc358748_g_mbus_config,
	.s_stream = tc358748_s_stream,
};

static const struct v4l2_subdev_pad_ops tc358748_pad_ops = {
	.enum_mbus_code = tc358748_enum_mbus_code,
	.set_fmt = tc358748_set_fmt,
	.get_fmt = tc358748_get_fmt,
	.link_validate = tc358748_link_validate,
};

static const struct v4l2_subdev_ops tc358748_ops = {
	.core = &tc358748_core_ops,
	.video = &tc358748_video_ops,
	.pad = &tc358748_pad_ops,
};

static const struct media_entity_operations tc358748_entity_ops = {
	.link_setup = &tc358748_link_setup,
	.link_validate = &v4l2_subdev_link_validate,
};

/* --------------- PROBE / REMOVE --------------- */

/*static int tc358748_set_lane_settings(struct tc358748_state *state,
				      struct v4l2_of_endpoint *fw)
{
	struct device *dev = &state->i2c_client->dev;
	int i;

	for (i = 0; i < fw->nr_of_link_frequencies; i++) {
		struct tc358748_csi_param *s =
			&state->link_freq_settings[i];
		u32 bps_pr_lane;

		state->link_frequencies[i] = fw->link_frequencies[i];

		
		bps_pr_lane = 2 * fw->link_frequencies[i];
		if (bps_pr_lane < 62500000U || bps_pr_lane > 1000000000U) {
			dev_err(dev, "unsupported bps per lane: %u bps\n",
				bps_pr_lane);
			return -EINVAL;
		}

		if (bps_pr_lane > 500000000)
			s->speed_range = 0;
		else if (bps_pr_lane > 250000000)
			s->speed_range = 1;
		else if (bps_pr_lane > 125000000)
			s->speed_range = 2;
		else
			s->speed_range = 3;

		s->unit_clk_hz = state->pllinclk_hz >> s->speed_range;
		s->unit_clk_mul = bps_pr_lane / s->unit_clk_hz;
		s->speed_per_lane = bps_pr_lane;
		s->lane_num = fw->bus.mipi_csi2.num_data_lanes;
		s->is_continuous_clk = fw->bus.mipi_csi2.flags &
			V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

		if (s->speed_per_lane != 432000000U)
			dev_warn(dev, "untested bps per lane: %u bps\n",
				 s->speed_per_lane);

		dev_dbg(dev, "%s: lane setting %d\n", __func__, i);
		dev_dbg(dev, "unit_clk %uHz: unit_clk_mul %u: speed_range %u: speed_per_lane(bps/lane) %u: csi_lange_numbers %u\n",
			s->unit_clk_hz, s->unit_clk_mul, s->speed_range,
			s->speed_per_lane, s->lane_num);
	}

	state->link_frequencies_num = fw->nr_of_link_frequencies;

	return 0;
}*/
static bool tc358748_parse_dt(struct tc358748_platform_data *pdata,
		struct i2c_client *client)
{
	struct device_node *node = client->dev.of_node;
	const u32 *property;

	v4l_dbg(1, debug, client, "Device Tree Parameters:\n");

	pdata->reset_gpio = of_get_named_gpio(node, "reset-gpios", 0);
	if (pdata->reset_gpio == 0)
		return false;
	v4l_dbg(1, debug, client, "reset_gpio = %d\n", pdata->reset_gpio);

	property = of_get_property(node, "refclk_hz", NULL);
	if (property == NULL)
		return false;
	pdata->refclk_hz = be32_to_cpup(property);
	v4l_dbg(1, debug, client, "refclk_hz = %d\n", be32_to_cpup(property));

	return true;
}
static void tc358748_gpio_reset(struct tc358748_state *state)
{
	usleep_range(5000, 10000);
	gpiod_set_value(state->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value(state->reset_gpio, 0);
	msleep(20);
}

static int tc358748_probe_of(struct tc358748_state *state)
{
	struct device *dev = &state->i2c_client->dev;
	struct v4l2_of_endpoint *endpoint;
	struct device_node *ep;
	// struct clk *refclk;
	u32 bps_pr_lane;
	int ret = -EINVAL;

	// refclk = devm_clk_get(dev, "cam_mclk1");
	// if (IS_ERR(refclk)) {
	// 	if (PTR_ERR(refclk) != -EPROBE_DEFER)
	// 		dev_err(dev, "failed to get refclk: %ld\n",
	// 			PTR_ERR(refclk));
	// 	return PTR_ERR(refclk);
	// }

	ep = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!ep) {
		dev_err(dev, "missing endpoint node\n");
		return -EINVAL;
	}

	endpoint = v4l2_of_alloc_parse_endpoint(ep);
	if (IS_ERR(endpoint)) {
		dev_err(dev, "failed to parse endpoint\n");
		return PTR_ERR(endpoint);
	}

	if (endpoint->bus_type != V4L2_MBUS_CSI2 ||
	    endpoint->bus.mipi_csi2.num_data_lanes == 0 ||
	    endpoint->nr_of_link_frequencies == 0) {
		dev_err(dev, "missing CSI-2 properties in endpoint\n");
		goto free_endpoint;
	}

	pr_info("tc358748 endpoint->bus.mipi_csi2.flags %d\n",
		endpoint->bus.mipi_csi2.flags);
	pr_info("tc358748 endpoint->bus.mipi_csi2.clock_lane %d\n",
		endpoint->bus.mipi_csi2.clock_lane);
	pr_info("tc358748 endpoint->bus.mipi_csi2.num_data_lanes %d\n",
		endpoint->bus.mipi_csi2.num_data_lanes);
	pr_info("tc358748 endpoint->bus.mipi_csi2.data_lanes [%d-%d-%d-%d]\n",
			endpoint->bus.mipi_csi2.data_lanes[0],
			endpoint->bus.mipi_csi2.data_lanes[1],
			endpoint->bus.mipi_csi2.data_lanes[2],
			endpoint->bus.mipi_csi2.data_lanes[3]);
    pr_info("tc358748 endpoint->nr_of_link_frequencies %d\n",
    	endpoint->nr_of_link_frequencies);

	// state->bus = endpoint->bus.mipi_csi2;
    // pr_info("tc358748 state->bus %s\n",state->bus);
	// clk_prepare_enable(refclk);

	// state->pdata.refclk_hz = clk_get_rate(refclk);
	state->pdata.refclk_hz = 27000000;
    // if ((state->pdata.refclk_hz != 26000000) ||
    //     (state->pdata.refclk_hz != 27000000) ||
    //     (state->pdata.refclk_hz != 42000000))
    // {
    //     pr_info("Set new clock\n");
    //     if (0 != clk_set_rate(refclk,27000000))
    //     {
    //         pr_info("Error: Set new clock\n");
    //     }
    // }

	
	/* A FIFO level of 16 should be enough for 2-lane 720p60 at 594 MHz. */
	state->pdata.fifo_level = 16;
	/*
	 * The PLL input clock is obtained by dividing refclk by pll_prd.
	 * It must be between 6 MHz and 40 MHz, lower frequency is better.
	 */
	switch (state->pdata.refclk_hz) {
    //~ case 26322581:
        //~ state->pdata.refclk_hz = 26322581;
	case 26000000:
	case 27000000:
	//~ case 40800000: /* Tegra */
	case 42000000:
		state->pdata.pll_prd = state->pdata.refclk_hz / 6000000;
		break;
	default:
		dev_err(dev, "Unsupported refclk rate: %u Hz\n",
			state->pdata.refclk_hz);
		goto disable_clk;
	}

	/*
	 * The CSI bps per lane must be between 62.5 Mbps and 1 Gbps.
	 * The default is 594 Mbps for 4-lane 1080p60 or 2-lane 720p60.
	 */
	bps_pr_lane = 2 * endpoint->link_frequencies[0];
	/*if (bps_pr_lane < 62500000U || bps_pr_lane > 1000000000U) {
		dev_err(dev, "unsupported bps per lane: %u bps\n", bps_pr_lane);
		goto disable_clk;
	}*/

	/* The CSI speed per lane is refclk / pll_prd * pll_fbd */
	state->pdata.pll_fbd = bps_pr_lane /
			       state->pdata.refclk_hz * state->pdata.pll_prd;

	/*
	 * FIXME: These timings are from REF_02 for 594 Mbps per lane (297 MHz
	 * link frequency). In principle it should be possible to calculate
	 * them based on link frequency and resolution.
	 */
	if (bps_pr_lane != 594000000U)
		dev_warn(dev, "untested bps per lane: %u bps\n", bps_pr_lane);
	pr_info("tc358748 state->pdata.pll_prd=%d\r\n",state->pdata.pll_prd);
	pr_info("tc358748 state->pdata.pll_fbd=%d\r\n",state->pdata.pll_fbd);

	// freq = refclk / prd * fbd, default = 594 MHz
	state->pdata.lineinitcnt = 0xe80;
	state->pdata.lptxtimecnt = 0x003;
	/* tclk-preparecnt: 3, tclk-zerocnt: 20 */
	state->pdata.tclk_headercnt = 0x1403;
	state->pdata.tclk_trailcnt = 0x00;
	/* ths-preparecnt: 3, ths-zerocnt: 1 */
	state->pdata.ths_headercnt = 0x0103;
	state->pdata.twakeup = 0x4882;
	state->pdata.tclk_postcnt = 0x008;
	state->pdata.ths_trailcnt = 0x2;
	state->pdata.hstxvregcnt = 2;
/*richard you*/	
	state->pdata.pll_prd        = 4;
#ifdef tc358748_VOUT_RGB
	state->pdata.pll_fbd        = 132;
	
	// timing setting
	state->pdata.lineinitcnt    =0x00000fa0;
	state->pdata.lptxtimecnt    =0x00000005;
	state->pdata.tclk_headercnt =0x00001603;
	state->pdata.tclk_trailcnt  =0x00000001;
	state->pdata.ths_headercnt  =0x00000603;
	state->pdata.twakeup        =0x000032c8;
	state->pdata.tclk_postcnt   =0x00000008;
	state->pdata.ths_trailcnt   =0x00000002;
	state->pdata.hstxvregcnt    =0x00000005;
#else
	state->pdata.pll_fbd        = 88;

	// timing setting
	state->pdata.lineinitcnt    =0x00001770;
	state->pdata.lptxtimecnt    =0x00000005;
	state->pdata.tclk_headercnt =0x00002004;
	state->pdata.tclk_trailcnt  =0x00000001;
	state->pdata.ths_headercnt  =0x00000606;
	state->pdata.twakeup        =0x00004a38;
	state->pdata.tclk_postcnt   =0x00000008;
	state->pdata.ths_trailcnt   =0x00000005;
	state->pdata.hstxvregcnt    =0x00000005;
#endif
	
	//~ state->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    //~ GPIOD_OUT_LOW);
	//~ if (IS_ERR(state->reset_gpio)) {
		//~ dev_err(dev, "failed to get reset gpio\n");
		//~ ret = PTR_ERR(state->reset_gpio);
		//~ goto disable_clk;
	//~ }

	if (state->reset_gpio) {
		pr_info("Calling reset GPIO but NOT IMPLEMENTED!");
		tc358748_gpio_reset(state);
	}
	ret =0;
	
	goto free_endpoint;

disable_clk:
	// clk_disable_unprepare(refclk);
free_endpoint:
	v4l2_of_free_endpoint(endpoint);
	return ret;
}
/*#else
static inline int tc358748_probe_of(struct tc358748_state *state)
{
	return -ENODEV;
}
#endif*/

static int tc358748_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}

static const struct v4l2_subdev_internal_ops tc358748_subdev_internal_ops = {
	.open = tc358748_open,
};

static const struct media_entity_operations tc358748_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};


static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
};



static const char * const tc358764_test_pattern_menu[] = {
	"Disabled",
	"colorbar 80px",
};

static int tc358748_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	   
	struct tc358748_state *state;
	struct tc358748_platform_data *pdata = client->dev.platform_data;
	struct v4l2_subdev *sd;
	int err;
	//u16 chip_id_val;

    // pr_info("%s %s %s\n",__FUNCTION__,__DATE__,__TIME__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;
	v4l_dbg(1, debug, client, "chip found @0x%x (%s)\n",
		client->addr, client->adapter->name);

	state = devm_kzalloc(&client->dev, sizeof(struct tc358748_state),
			GFP_KERNEL);
	if (!state)
		return -ENOMEM;
    if (client->dev.of_node) {
        if (!tc358748_parse_dt(&state->pdata, client)) {
            pr_err("Couldn't parse device tree\n");
            return -ENODEV;
        }
    }

	state->i2c_client = client;

	/* platform data */
	if (pdata) {
		state->pdata = *pdata;
		pdata->endpoint.bus.mipi_csi2.flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else {
		err = tc358748_probe_of(state);
		if (err == -ENODEV)
			v4l_err(client, "No platform data!\n");
		if (err)
			return err;
	}

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &tc358748_ops);
	v4l2_info(sd,"Subdev init done\n");
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS;

	/* i2c access */
	/* i2c access */
	if (((i2c_rd16(sd, CHIPID) & CHIPID_CHIPID_MASK) >> 8) != 0x44) {
		v4l2_info(sd, "not a TC358748 on address 0x%x\n",
			  client->addr << 1);
		//return -ENODEV;
	}

	/* control handlers */
	v4l2_ctrl_handler_init(&state->hdl, 1);

	v4l2_ctrl_new_std_menu_items(&state->hdl,
			&tc358764_ctrl_ops, V4L2_CID_TEST_PATTERN,
			ARRAY_SIZE(tc358764_test_pattern_menu) - 1, 0, 0,
			tc358764_test_pattern_menu);

	state->link_freq =
		v4l2_ctrl_new_int_menu(&state->hdl, &tc358764_ctrl_ops,
				       V4L2_CID_LINK_FREQ,
				       state->link_frequencies_num - 1,
				       TC358748_DEF_LINK_FREQ,
				       state->link_frequencies);


	sd->ctrl_handler = &state->hdl;
	if (state->hdl.error) {
		err = state->hdl.error;
		goto err_hdl;
	}

	state->pads[1].flags = MEDIA_PAD_FL_SOURCE;
	state->pads[0].flags = MEDIA_PAD_FL_SINK;
	//sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &tc358748_entity_ops;
	err = media_entity_pads_init(&sd->entity, 2, state->pads);
	if (err < 0)
		goto err_hdl;

	mutex_init(&state->confctl_mutex);

	state->fmt = tc358748_def_fmt;

	/* apply default settings */
	tc358748_sreset(sd);
	tc358748_set_buffers(sd);
	tc358748_set_csi(sd);
	tc358748_set_csi_color_space(sd);
	tc358748_sleep_mode(sd, 1);
	tc358748_set_pll(sd);
	tc358748_enable_stream(sd, 0);

	err = v4l2_async_register_subdev(sd);
	if (err < 0)
		goto err_hdl;

	v4l2_info(sd, "%s found @ 0x%x (%s)\n", client->name,
		  client->addr << 1, client->adapter->name);

	return 0;

err_hdl:
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&state->hdl);
	return err;
}

static int tc358748_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tc358748_state *state = to_state(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_device_unregister_subdev(sd);
	mutex_destroy(&state->confctl_mutex);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&state->hdl);

	return 0;
}

static struct i2c_device_id tc358748_id[] = {
	{"tc358748", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tc358748_id);

static struct i2c_driver tc358748_driver = {
	.driver = {
		.name = "tc358748",
	},
	.probe = tc358748_probe,
	.remove = tc358748_remove,
	.id_table = tc358748_id,
};

module_i2c_driver(tc358748_driver);
