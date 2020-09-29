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
#include <media/i2c/tc358743.h>

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


static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "debug level (0-3)");

MODULE_DESCRIPTION("Toshiba TC358748 Parallel to CSI-2 bridge driver");
MODULE_AUTHOR("Marco Felsch <kernel@pengutronix.de>");
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

struct tc358748_state {
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

static int i2c_rd(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358743_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	int err;
	u8 buf[2] = { reg >> 8, reg &0xff };
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
			.buf = values,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		v4l2_err(sd, "%s: #### reading register0x%x from0x%x failed\n",
				__func__, reg, client->addr);
		return -1;
	}
	//udelay(10);
	return 0;
}

static int i2c_wr(struct v4l2_subdev *sd, u16 reg, u8 *values, u32 n)
{
	struct tc358743_state *state = to_state(sd);
	struct i2c_client *client = state->i2c_client;
	u8 *data = state->wr_data;
	int err, i;
	struct i2c_msg msg;

	if ((2 + n) > sizeof(state->wr_data)){
		v4l2_warn(sd, "i2c wr reg=%04x: len=%d is too big!\n",
			  reg, 2 + n);
		return -1;
	}

	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2 + n;
	msg.flags = 0;

	data[0] = reg >> 8;
	data[1] = reg &0xff;

	for (i = 0; i < n; i++)
		data[2 + i] = values[i];

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		v4l2_err(sd, "%s: writing register0x%x from0x%x failed\n",
				__func__, reg, client->addr);
		return -1;
	}
	return 0;
}
static u8 i2c_rd8(struct v4l2_subdev *sd, u16 reg)
{
	u8 val;

	i2c_rd(sd, reg, &val, 1);

	return val;
}

static void i2c_wr8(struct v4l2_subdev *sd, u16 reg, u8 val)
{
	i2c_wr(sd, reg, &val, 1);
}

static void i2c_wr8_and_or(struct v4l2_subdev *sd, u16 reg,	u8 mask, u8 val)
{
	i2c_wr8(sd, reg, (i2c_rd8(sd, reg) & mask) | val);
}

static u16 i2c_rd16(struct v4l2_subdev *sd, u16 reg)
{
	u16 val;
	int ret;
	// v4l2_info(sd, "Reading i2c_rd16\n");
	

	ret = i2c_rd(sd, reg, (u8 *)&val, 2);
	// v4l2_info(sd, "RET %d\n", ret);

	if (ret == -1) {
		// Read failed
		return 99;  // TODO. Make this better!
	}
	
	return val;
}

static void i2c_wr16(struct v4l2_subdev *sd, u16 reg, u16 val)
{
	i2c_wr(sd, reg, (u8 *)&val, 2);
}

static void i2c_wr16_and_or(struct v4l2_subdev *sd, u16 reg, u16 mask, u16 val)
{
	i2c_wr16(sd, reg, (i2c_rd16(sd, reg) & mask) | val);
}

static u32 i2c_rd32(struct v4l2_subdev *sd, u16 reg)
{
	u32 val;

	i2c_rd(sd, reg, (u8 *)&val, 4);

	return val;
}

static void i2c_wr32(struct v4l2_subdev *sd, u16 reg, u32 val)
{
	i2c_wr(sd, reg, (u8 *)&val, 4);
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


static int tc358743_probe_of(struct tc358743_state *state)
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

	
	state->refclk = 27000000;
   

	
	/* A FIFO level of 16 should be enough for 2-lane 720p60 at 594 MHz. */
	state->pdata.fifo_level = 16;
	/*
	 * The PLL input clock is obtained by dividing refclk by pll_prd.
	 * It must be between 6 MHz and 40 MHz, lower frequency is better.
	 */
	switch (state->refclk) {
    //~ case 26322581:
        //~ state->pdata.refclk_hz = 26322581;
	case 26000000:
	case 27000000:
	//~ case 40800000: /* Tegra */
	case 42000000:
		state-> pll_prd = state->refclk / 6000000;
		break;
	default:
		dev_err(dev, "Unsupported refclk rate: %u Hz\n",
			state->refclk);
		goto disable_clk;
	}

	/*
	 * The CSI bps per lane must be between 62.5 Mbps and 1 Gbps.
	 * The default is 594 Mbps for 4-lane 1080p60 or 2-lane 720p60.
	 */
	bps_pr_lane = 2 * endpoint->link_frequencies[0];
	if (bps_pr_lane < 62500000U || bps_pr_lane > 1000000000U) {
		dev_err(dev, "unsupported bps per lane: %u bps\n", bps_pr_lane);
		goto disable_clk;
	}

	/* The CSI speed per lane is refclk / pll_prd * pll_fbd */
	state->pdata.pll_fbd = bps_pr_lane /
			       state->refclk * state->pll_prd;

	/*
	 * FIXME: These timings are from REF_02 for 594 Mbps per lane (297 MHz
	 * link frequency). In principle it should be possible to calculate
	 * them based on link frequency and resolution.
	 */
	if (bps_pr_lane != 594000000U)
		dev_warn(dev, "untested bps per lane: %u bps\n", bps_pr_lane);
	pr_info("tc358743 state->pdata.pll_prd=%d\r\n",state->pll_prd);
	pr_info("tc358743 state->pdata.pll_fbd=%d\r\n",state->pll_fbd);

	
	ret = 0;
	goto free_endpoint;

disable_clk:
	// clk_disable_unprepare(refclk);
free_endpoint:
	v4l2_of_free_endpoint(endpoint);
	return ret;
}
#else
static inline int tc358743_probe_of(struct tc358743_state *state)
{
	return -ENODEV;
}
#endif

static int tc358743_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	return 0;
}


static const struct media_entity_operations tc358748_entity_ops = {
	.link_setup = &tc358748_link_setup,
	.link_validate = &v4l2_subdev_link_validate,
};

static int tc358748_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct tc358748_state *state;
	struct v4l2_subdev *sd;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	state = devm_kzalloc(&client->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->i2c_client = client;

	/* platform data */
if (client->dev.platform_data) {
		state->pdata = client->dev.platform_data;
		client->dev.platform_data->endpoint.bus.mipi_csi2.flags = V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	} else {
		err = tc358743_probe_of(state);
		if (err == -ENODEV)
			v4l_err(client, "No platform data!\n");
		if (err)
			return err;
	}

	sd = &state->sd;
	v4l2_i2c_subdev_init(sd, client, &tc358748_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	/* i2c access */
	if (((i2c_rd16(sd, CHIPID) & CHIPID_CHIPID_MASK) >> 8) != 0x44) {
		v4l2_info(sd, "not a TC358748 on address 0x%x\n",
			  client->addr << 1);
		return -ENODEV;
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
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &tc358748_entity_ops;
	err = media_entity_pads_init(&sd->entity, 2, state->pads);
	if (err < 0)
		goto err_hdl;

	mutex_init(&state->confctl_mutex);


	return 0;
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



static const struct i2c_device_id tc358748_id[] = {
	{"tc358748", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tc358748_id);

static struct i2c_driver tc358748_driver = {
	.driver = {
		.name = "tc358748",
		.of_match_table = of_match_ptr(tc358748_of_match),
	},
	.probe = tc358748_probe,
	.remove = tc358748_remove,
	.id_table = tc358748_id,
};

module_i2c_driver(tc358748_driver);
