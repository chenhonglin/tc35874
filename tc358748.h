/*
 * tc358748 - Toshiba PARALLEL to CSI-2 bridge
 *
 * Copyright 2015 Cisco Systems, Inc. and/or its affiliates. All rights
 * reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

/*
 * References (c = chapter, p = page):
 * REF_01 - Toshiba, TC358748XBG (H2C), Functional Specification, Rev 0.60
 * REF_02 - Toshiba, TC358748XBG_PARALLEL-CSI_Tv11p_nm.xls
 */

#ifndef _TC358748_
#define _TC358748_

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
