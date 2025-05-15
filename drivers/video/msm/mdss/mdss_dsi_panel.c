/* Copyright (c) 2012-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include <linux/platform_data/rt4501_bl.h>	//SW4-HL-Display-BringUpNT35521-00+_20150224

#include "mdss_dsi.h"
#include <linux/device.h>
#include <linux/time.h>	//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+_20151218

//SW4-HL-Display-BBox-00+{_20150610
/* Black Box */
#define BBOX_PANEL_GPIO_FAIL do {printk("BBox;%s: GPIO fail\n", __func__); printk("BBox::UEC;0::1\n");} while (0);
//SW4-HL-Display-BBox-00+}_20150610
/* E1M-634 - Add LCM BBS log */
#define BBOX_LCM_DISPLA_ON_FAIL do {printk("BBox;%s: LCM Display on fail\n", __func__); printk("BBox::UEC;0::2\n");} while (0);
#define BBOX_LCM_DISPLA_OFF_FAIL    do {printk("BBox;%s: LCM Display off fail\n", __func__); printk("BBox::UEC;0::3\n");} while (0);
#define BBOX_LCM_POWER_STATUS_ABNORMAL    do {printk("BBox;%s: LCM power status abnormal\n", __func__); printk("BBox::UEC;0::6\n");} while (0);
#define BBOX_LCM_OEM_FUNCTIONS_FAIL do {printk("BBox;%s: LCM OEM functions (CE or CT or BLF or CABC) functions fail!\n", __func__); printk("BBox::UEC;0::8\n");} while (0);
#define BBOX_BACKLIGHT_PWM_OPERATION_FAIL do {printk("BBox;%s: BL DCS cmd fail\n", __func__); printk("BBox::UEC;1::0\n");} while (0);

//TP add
extern void fih_fts_tp_lcm_resume(void);
extern void fih_fts_tp_lcm_suspend(void);
//TP add end

#define PANEL_REG_ADDR_LEN 8
void fih_get_panel_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata, u8 check);
/* end E1M-634 */

//SW4-HL-Display-BringUpNT35521-00+{_20150224
static int ce_status = 0;
static int ct_status = 0;
static int cabc_status = 0;
//SW4-HL-Display-BringUpNT35521-00+}_20150224

#define DT_CMD_HDR 6

/* NT35596 panel specific status variables */
#define NT35596_BUF_3_STATUS 0x02
#define NT35596_BUF_4_STATUS 0x40
#define NT35596_BUF_5_STATUS 0x80
#define NT35596_MAX_ERR_CNT 2

#define MIN_REFRESH_RATE 48
#define DEFAULT_MDP_TRANSFER_TIME 14000

//SW4-HL-Display-EnablePWMOutput-00+{_20150605
extern int SendCEOnlyAfterResume;
extern unsigned long ce_en;
extern int SendCTOnlyAfterResume;
extern unsigned long ct_set;
extern int SendCABCOnlyAfterResume;
extern unsigned long cabc_set;
//SW4-HL-Display-EnablePWMOutput-00+}_20150605

static int gDisplayOnEnable = 0;	//SW4-HL-Display-EnablePWMOutput-01+_20150611

static bool g350nitPanel = false;	//SW4-HL-Display-FineTuneBLMappingTable-04+_20150730

int gDisplayOnCmdAlreadySent = 1;	//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+_20151218

//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+{_20151218
struct timeval time_one;
struct timeval time_two;
//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+}_20151218

DEFINE_LED_TRIGGER(bl_led_trigger);

void mdss_dsi_panel_pwm_cfg(struct mdss_dsi_ctrl_pdata *ctrl)
{
	if (ctrl->pwm_pmi)
		return;

	ctrl->pwm_bl = pwm_request(ctrl->pwm_lpg_chan, "lcd-bklt");
	if (ctrl->pwm_bl == NULL || IS_ERR(ctrl->pwm_bl)) {
		pr_err("%s: Error: lpg_chan=%d pwm request failed",
				__func__, ctrl->pwm_lpg_chan);
	}
	ctrl->pwm_enabled = 0;
}

static void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	int ret;
	u32 duty;
	u32 period_ns;

	if (ctrl->pwm_bl == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

	if (level == 0) {
		if (ctrl->pwm_enabled) {
			ret = pwm_config_us(ctrl->pwm_bl, level,
					ctrl->pwm_period);
			if (ret)
				pr_err("%s: pwm_config_us() failed err=%d.\n",
						__func__, ret);
			pwm_disable(ctrl->pwm_bl);
		}
		ctrl->pwm_enabled = 0;
		return;
	}

	duty = level * ctrl->pwm_period;
	duty /= ctrl->bklt_max;

	pr_debug("%s: bklt_ctrl=%d pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__, ctrl->bklt_ctrl, ctrl->pwm_period,
				ctrl->pwm_pmic_gpio, ctrl->pwm_lpg_chan);

	pr_debug("%s: ndx=%d level=%d duty=%d\n", __func__,
					ctrl->ndx, level, duty);

	if (ctrl->pwm_period >= USEC_PER_SEC) {
		ret = pwm_config_us(ctrl->pwm_bl, duty, ctrl->pwm_period);
		if (ret) {
			pr_err("%s: pwm_config_us() failed err=%d.\n",
					__func__, ret);
			return;
		}
	} else {
		period_ns = ctrl->pwm_period * NSEC_PER_USEC;
		ret = pwm_config(ctrl->pwm_bl,
				level * period_ns / ctrl->bklt_max,
				period_ns);
		if (ret) {
			pr_err("%s: pwm_config() failed err=%d.\n",
					__func__, ret);
			return;
		}
	}

	if (!ctrl->pwm_enabled) {
		ret = pwm_enable(ctrl->pwm_bl);
		if (ret)
			pr_err("%s: pwm_enable() failed err=%d\n", __func__,
				ret);
		ctrl->pwm_enabled = 1;
	}
}

static char dcs_cmd[2] = {0x54, 0x00}; /* DTYPE_DCS_READ */
static struct dsi_cmd_desc dcs_read_cmd = {
	{DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(dcs_cmd)},
	dcs_cmd
};

u32 mdss_dsi_panel_cmd_read(struct mdss_dsi_ctrl_pdata *ctrl, char cmd0,
		char cmd1, void (*fxn)(int), char *rbuf, int len)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return -EINVAL;
	}

	dcs_cmd[0] = cmd0;
	dcs_cmd[1] = cmd1;
	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dcs_read_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = len;
	cmdreq.rbuf = rbuf;
	cmdreq.cb = fxn; /* call back */
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	/*
	 * blocked here, until call back called
	 */

	return 0;
}

//SW4-HL-Display-PowerPinControlPinAndInitCodeAPI-00*_20150519
int mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,	//SW4-HL-Display-EnhanceErrorHandling-00*_20150320
			struct dsi_panel_cmds *pcmds)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;
	int len = 1;	//SW4-HL-Display-EnhanceErrorHandling-00*_20150320

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return len;	//SW4-HL-Display-EnhanceErrorHandling-00*_20150320
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = CMD_REQ_COMMIT;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;
	else if (pcmds->link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	//SW4-HL-Display-EnhanceErrorHandling-00*{_20150320
	len = mdss_dsi_cmdlist_put(ctrl, &cmdreq);

	return len;
	//SW4-HL-Display-EnhanceErrorHandling-00*}_20150320
}

static char led_pwm1[3] = {0x51, 0x0, 0x0};	/* DTYPE_DCS_LWRITE */
static struct dsi_cmd_desc backlight_cmd = {
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(led_pwm1)},
	led_pwm1
};

static void mdss_dsi_panel_bklt_dcs(struct mdss_dsi_ctrl_pdata *ctrl, int level)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;
	int len = 1;	/* E1M-634 - Add LCM BBS log */

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return;
	}

	pr_debug("%s: level=%d\n", __func__, level);

	if (ctrl->panel_data.panel_info.pid == FT8716_720P_VIDEO_PANEL)
	{
		led_pwm1[1] = (unsigned char) ((level*1023/255) >> 2);
		led_pwm1[2] = (unsigned char) ((level*1023/255) & 0x3);
	} else {
		led_pwm1[1] = (unsigned char)level;
	}
	pr_debug("\n\n%s: level led_pwm1[1]=0x%x, led_pwm1[2]=0x%x\n", __func__, led_pwm1[1], led_pwm1[2]);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &backlight_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

/* E1M-634 - Add LCM BBS log */
	len = mdss_dsi_cmdlist_put(ctrl, &cmdreq);
	if (!len)
	{
		BBOX_BACKLIGHT_PWM_OPERATION_FAIL
	}
/* end E1M-634 */
}

static int mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;

	if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		rc = gpio_request(ctrl_pdata->disp_en_gpio,
						"disp_enable");
		if (rc) {
			pr_err("request disp_en gpio failed, rc=%d\n",
				       rc);
			BBOX_PANEL_GPIO_FAIL
			goto disp_en_gpio_err;
		}
	}
	rc = gpio_request(ctrl_pdata->rst_gpio, "disp_rst_n");
	if (rc) {
		pr_err("request reset gpio failed, rc=%d\n",
			rc);
		BBOX_PANEL_GPIO_FAIL
		goto rst_gpio_err;
	}
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
		rc = gpio_request(ctrl_pdata->bklt_en_gpio,
						"bklt_enable");
		if (rc) {
			pr_err("request bklt gpio failed, rc=%d\n",
				       rc);
			BBOX_PANEL_GPIO_FAIL
			goto bklt_en_gpio_err;
		}
	}
	if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
		rc = gpio_request(ctrl_pdata->mode_gpio, "panel_mode");
		if (rc) {
			pr_err("request panel mode gpio failed,rc=%d\n",
								rc);
			BBOX_PANEL_GPIO_FAIL
			goto mode_gpio_err;
		}
	}
	return rc;

mode_gpio_err:
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
		gpio_free(ctrl_pdata->bklt_en_gpio);
bklt_en_gpio_err:
	gpio_free(ctrl_pdata->rst_gpio);
rst_gpio_err:
	if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
		gpio_free(ctrl_pdata->disp_en_gpio);
disp_en_gpio_err:
	return rc;
}

int mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int i, rc = 0;

	pr_debug("\n\n******************** [HL] %s +++, enable = %d **********************\n\n", __func__, enable);

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}

	pr_debug("%s: enable = %d\n", __func__, enable);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		pr_debug("\n\n******************** [HL] %s, mdss_dsi_request_gpios(ctrl_pdata) **********************\n\n", __func__);

		if (!pinfo->cont_splash_enabled) {
			pr_debug("\n\n******************** [HL] %s, if (!pinfo->cont_splash_enabled) **********************\n\n", __func__);
			if (gpio_is_valid(ctrl_pdata->disp_en_gpio))
				gpio_set_value((ctrl_pdata->disp_en_gpio), 1);

			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep(pinfo->rst_seq[i] * 1000);
				pr_debug("\n\n******************** [HL] %s, i = %d **********************\n\n", __func__, i);
			}
			pr_debug("\n\n******************** [HL] %s, for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) **********************\n\n", __func__);

			if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
				gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
			gpio_free(ctrl_pdata->bklt_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
			gpio_set_value((ctrl_pdata->disp_en_gpio), 0);
			gpio_free(ctrl_pdata->disp_en_gpio);
		}
		gpio_set_value((ctrl_pdata->rst_gpio), 0);
		gpio_free(ctrl_pdata->rst_gpio);
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}

	pr_debug("\n\n******************** [HL] %s ---, rc = %d **********************\n\n", __func__, rc);

	return rc;
}

/**
 * mdss_dsi_roi_merge() -  merge two roi into single roi
 *
 * Function used by partial update with only one dsi intf take 2A/2B
 * (column/page) dcs commands.
 */
static int mdss_dsi_roi_merge(struct mdss_dsi_ctrl_pdata *ctrl,
					struct mdss_rect *roi)
{
	struct mdss_panel_info *l_pinfo;
	struct mdss_rect *l_roi;
	struct mdss_rect *r_roi;
	struct mdss_dsi_ctrl_pdata *other = NULL;
	int ans = 0;

	if (ctrl->ndx == DSI_CTRL_LEFT) {
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_RIGHT);
		if (!other)
			return ans;
		l_pinfo = &(ctrl->panel_data.panel_info);
		l_roi = &(ctrl->panel_data.panel_info.roi);
		r_roi = &(other->panel_data.panel_info.roi);
	} else  {
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_LEFT);
		if (!other)
			return ans;
		l_pinfo = &(other->panel_data.panel_info);
		l_roi = &(other->panel_data.panel_info.roi);
		r_roi = &(ctrl->panel_data.panel_info.roi);
	}

	if (l_roi->w == 0 && l_roi->h == 0) {
		/* right only */
		*roi = *r_roi;
		roi->x += l_pinfo->xres;/* add left full width to x-offset */
	} else {
		/* left only and left+righ */
		*roi = *l_roi;
		roi->w +=  r_roi->w; /* add right width */
		ans = 1;
	}

	return ans;
}

static char caset[] = {0x2a, 0x00, 0x00, 0x03, 0x00};	/* DTYPE_DCS_LWRITE */
static char paset[] = {0x2b, 0x00, 0x00, 0x05, 0x00};	/* DTYPE_DCS_LWRITE */

/* pack into one frame before sent */
static struct dsi_cmd_desc set_col_page_addr_cmd[] = {
	{{DTYPE_DCS_LWRITE, 0, 0, 0, 1, sizeof(caset)}, caset},	/* packed */
	{{DTYPE_DCS_LWRITE, 1, 0, 0, 1, sizeof(paset)}, paset},
};

static void mdss_dsi_send_col_page_addr(struct mdss_dsi_ctrl_pdata *ctrl,
					struct mdss_rect *roi)
{
	struct dcs_cmd_req cmdreq;

	roi = &ctrl->roi;

	caset[1] = (((roi->x) & 0xFF00) >> 8);
	caset[2] = (((roi->x) & 0xFF));
	caset[3] = (((roi->x - 1 + roi->w) & 0xFF00) >> 8);
	caset[4] = (((roi->x - 1 + roi->w) & 0xFF));
	set_col_page_addr_cmd[0].payload = caset;

	paset[1] = (((roi->y) & 0xFF00) >> 8);
	paset[2] = (((roi->y) & 0xFF));
	paset[3] = (((roi->y - 1 + roi->h) & 0xFF00) >> 8);
	paset[4] = (((roi->y - 1 + roi->h) & 0xFF));
	set_col_page_addr_cmd[1].payload = paset;

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds_cnt = 2;
	cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL | CMD_REQ_UNICAST;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	cmdreq.cmds = set_col_page_addr_cmd;
	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}

static int mdss_dsi_set_col_page_addr(struct mdss_panel_data *pdata)
{
	struct mdss_panel_info *pinfo;
	struct mdss_rect roi;
	struct mdss_rect *p_roi;
	struct mdss_rect *c_roi;
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_dsi_ctrl_pdata *other = NULL;
	int left_or_both = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pinfo = &pdata->panel_info;
	p_roi = &pinfo->roi;

	/*
	 * to avoid keep sending same col_page info to panel,
	 * if roi_merge enabled, the roi of left ctrl is used
	 * to compare against new merged roi and saved new
	 * merged roi to it after comparing.
	 * if roi_merge disabled, then the calling ctrl's roi
	 * and pinfo's roi are used to compare.
	 */
	if (pinfo->partial_update_roi_merge) {
		left_or_both = mdss_dsi_roi_merge(ctrl, &roi);
		other = mdss_dsi_get_ctrl_by_index(DSI_CTRL_LEFT);
		c_roi = &other->roi;
	} else {
		c_roi = &ctrl->roi;
		roi = *p_roi;
	}

	/* roi had changed, do col_page update */
	if (!mdss_rect_cmp(c_roi, &roi)) {
		pr_debug("%s: ndx=%d x=%d y=%d w=%d h=%d\n",
				__func__, ctrl->ndx, p_roi->x,
				p_roi->y, p_roi->w, p_roi->h);

		*c_roi = roi; /* keep to ctrl */
		if (c_roi->w == 0 || c_roi->h == 0) {
			/* no new frame update */
			pr_debug("%s: ctrl=%d, no partial roi set\n",
						__func__, ctrl->ndx);
			return 0;
		}

		if (pinfo->dcs_cmd_by_left) {
			if (left_or_both && ctrl->ndx == DSI_CTRL_RIGHT) {
				/* 2A/2B sent by left already */
				return 0;
			}
		}

		if (!mdss_dsi_sync_wait_enable(ctrl)) {
			if (pinfo->dcs_cmd_by_left)
				ctrl = mdss_dsi_get_ctrl_by_index(
							DSI_CTRL_LEFT);
			mdss_dsi_send_col_page_addr(ctrl, &roi);
		} else {
			/*
			 * when sync_wait_broadcast enabled,
			 * need trigger at right ctrl to
			 * start both dcs cmd transmission
			 */
			other = mdss_dsi_get_other_ctrl(ctrl);
			if (!other)
				goto end;

			if (mdss_dsi_is_left_ctrl(ctrl)) {
				mdss_dsi_send_col_page_addr(ctrl, &ctrl->roi);
				mdss_dsi_send_col_page_addr(other, &other->roi);
			} else {
				mdss_dsi_send_col_page_addr(other, &other->roi);
				mdss_dsi_send_col_page_addr(ctrl, &ctrl->roi);
			}
		}
	}

end:
	return 0;
}

static void mdss_dsi_panel_switch_mode(struct mdss_panel_data *pdata,
							int mode)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mipi_panel_info *mipi;
	struct dsi_panel_cmds *pcmds;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	mipi  = &pdata->panel_info.mipi;

	if (!mipi->dynamic_switch_enabled)
		return;

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (mode == DSI_CMD_MODE)
		pcmds = &ctrl_pdata->video2cmd;
	else
		pcmds = &ctrl_pdata->cmd2video;

	mdss_dsi_panel_cmds_send(ctrl_pdata, pcmds);

	return;
}

//SW4-HL-Display-FineTuneBLMappingTable-04*{_20150730
u32 transfer_bl_level(int pid, u32 trs_level)
{
	u32 after_trs_level;
	u8 quo,rem;

	pr_debug("\n\n******************** [HL] %s, pid = %d, level = %d  **********************\n\n", __func__, pid, trs_level);

	pr_debug("\n\n******************** [HL] %s, g350nitPanel = %d  **********************\n\n", __func__, g350nitPanel);
	if (g350nitPanel)	//For 350nit panel
	{
		if(trs_level == 30) //mapping 30 to 19 of virtual file
		{
			after_trs_level = 19;
		}
		else if((trs_level >= 5) && (trs_level <= 29)) //mapping 5~29 to 5~18 of virtual file
		{
			quo = trs_level / 3;
			rem = trs_level % 3;
			after_trs_level = quo * 2 + rem;
		}
		else if((trs_level >= 31) && (trs_level <= 180)) //mapping 31~180 to 20~180 of virtual file
		{
			switch (trs_level)
			{
				case 31:
					after_trs_level = 20;
					break;
				case 32:
					after_trs_level = 22;
					break;
				case 33:
					after_trs_level = 24;
					break;
				case 34:
					after_trs_level = 26;
					break;
				case 35:
					after_trs_level = 28;
					break;
				case 36:
					after_trs_level = 30;
					break;
				case 37:
					after_trs_level = 32;
					break;
				case 38:
					after_trs_level = 34;
					break;
				case 39:
					after_trs_level = 36;
					break;
				case 40:
					after_trs_level = 38;
					break;
				case 41:
					after_trs_level = 40;
					break;
				default:
					after_trs_level = trs_level;
					break;
			}
		}
		else	//others, ex: level = 0
		{
			after_trs_level = trs_level;
		}
	}
	else	//For 420nit/450nit panel or others
	{
		if(trs_level == 29) //mapping 29 to 21 of virtual file
		{
			after_trs_level = 21;
		}
		else if((trs_level >= 5) && (trs_level <= 28)) //mapping 5~28 to 5~20 of virtual file
		{
			quo = trs_level / 3;
			rem = trs_level % 3;
			after_trs_level = quo * 2 + rem;
		}
		else if((trs_level >= 30) && (trs_level <= 175)) //mapping 30 to 22~175 of virtual file
		{
			switch (trs_level)
			{
				case 30:
					after_trs_level = 22;
					break;
				case 31:
					after_trs_level = 24;
					break;
				case 32:
					after_trs_level = 26;
					break;
				case 33:
					after_trs_level = 28;
					break;
				case 34:
					after_trs_level = 30;
					break;
				case 35:
					after_trs_level = 32;
					break;
				case 36:
					after_trs_level = 34;
					break;
				case 37:
					after_trs_level = 36;
				default:
					after_trs_level = trs_level;
					break;
			}
		}
		else	//others, ex: level = 0
		{
			after_trs_level = trs_level;
		}
	}

	pr_debug("\n\n******************** [HL] %s, after_trs_level = %d  **********************\n\n", __func__, after_trs_level);

	return after_trs_level;
}
//SW4-HL-Display-FineTuneBLMappingTable-04*}_20150730

//SW4-HL-Dispay-BringUpNt35521sWithBlIcNt50568_ForD1M-00*{_20160603
static int old_bl = 0;
int nt50568_set_backlight_level(int bl_level)
{
	int rc = 0;

	pr_debug("\n\n******************** [HL] %s, +++, bl_level = %d **********************\n\n", __func__, bl_level);

	if (old_bl != bl_level)
	{
		if ((old_bl == 0) && (bl_level != 0))
		{
			pr_debug("\n\n******************** [HL] %s, (old_bl == 0) && (bl_level != 0) **********************\n\n", __func__);
			rc = 1;
		}
		else
		{
			rc = 0;
		}

		if ((bl_level == 0) || ((old_bl == 0) && (bl_level != 0)))
		{
			pr_debug("%s: level=%d\n", __func__, bl_level);
		}

		if ((old_bl != 0) && (bl_level == 0))
		{
			pr_debug("\n\n******************** [HL] %s, (old_bl != 0) && (bl_level == 0) **********************\n\n", __func__);
			rc = 2;
		}

		old_bl = bl_level;
	}
	else
	{
		//No set backlight since old_bl equals to brightness
		pr_debug("\n\n******************** [HL] %s, No set backlight since old_bl equals to brightness **********************\n\n", __func__);
	}

	pr_debug("\n\n******************** [HL] %s, --- **********************\n\n", __func__);

	return rc;
}
//SW4-HL-Display-D1M-BringUpNt35521sWithNt50568+}_20160603

static void mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata,
							u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_dsi_ctrl_pdata *sctrl = NULL;
	int rc = 0;	//SW4-HL-Display-EnablePWMOutput-00+_20150605
	int len = 1;	//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+_20151218

	pr_debug("[HL]%s: <-- start\n", __func__);
	pr_debug("[HL]%s: bl_level = %d\n", __func__, bl_level);

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	//SW4-HL-Display-EnablePWMOutput-03*{_20150623
	if (!ctrl_pdata->pre_ce_off_cmds.cmd_cnt &&
		!ctrl_pdata->pre_ce_on_cmds.cmd_cnt)
	{
		if (SendCEOnlyAfterResume)
		{
			mdss_dsi_panel_ce_onoff(ctrl_pdata, ce_en);
			SendCEOnlyAfterResume = 0;
		}
	}

	if (ctrl_pdata->display_on_cmds.cmd_cnt)
	{
		if (gDisplayOnEnable)
		{
			len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->display_on_cmds);
			if (!len)
			{
				pr_err("%s: cmds send fail\n", __func__);
				return;
			}

			gDisplayOnEnable = 0;

			//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+_20151218
			gDisplayOnCmdAlreadySent = 1;
		}
	}

	if (SendCTOnlyAfterResume)
	{
		mdss_dsi_panel_ct_set(ctrl_pdata, ct_set);
		SendCTOnlyAfterResume = 0;
	}
	//SW4-HL-Display-EnablePWMOutput-03*}_20150623

	/*
	 * Some backlight controllers specify a minimum duty cycle
	 * for the backlight brightness. If the brightness is less
	 * than it, the controller can malfunction.
	 */

	if ((bl_level < pdata->panel_info.bl_min) && (bl_level != 0))
		bl_level = pdata->panel_info.bl_min;
	//SW4-HL-Display-FineTuneBLMappingTable-00+{_20150616
	else if (bl_level > pdata->panel_info.bl_max)
		bl_level = pdata->panel_info.bl_max;
	//SW4-HL-Display-FineTuneBLMappingTable-00+}_20150616

	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		led_trigger_event(bl_led_trigger, bl_level);
		break;
	case BL_PWM:
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
		break;
	case BL_DCS_CMD:
		if (!mdss_dsi_sync_wait_enable(ctrl_pdata)) {
			//SW4-HL-Display-EnablePWMOutput-01*{_20150611
			//SW4-HL-Display-FineTuneBLMappingTable-03*_20150625
			if (ctrl_pdata->panel_data.panel_info.pid != FT8716_720P_VIDEO_PANEL)
				bl_level = transfer_bl_level(ctrl_pdata->panel_data.panel_info.pid, bl_level);
			//SW4-HL-Display-D1M-BringUpNt35521sWithNt50568*{_20160603
			if(strstr(saved_command_line, "androidboot.device=D1M")!=NULL ||
				strstr(saved_command_line, "androidboot.device=E1M")!=NULL ||
				strstr(saved_command_line, "androidboot.device=AT2")!=NULL )
			{
				rc = nt50568_set_backlight_level(bl_level);
			}
			else
			{
				rc = rt4501_set_backlight_level(bl_level);
			}
			//SW4-HL-Display-D1M-BringUpNt35521sWithNt50568*}_20160603
			//SW4-HL-Display-EnablePWMOutput-01*}_20150611

			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);

			//SW4-HL-Display-EnablePWMOutput-01*{_20150611
			if (rc >= 1)
			{
				pr_err("%s: level=%d\n", __func__, bl_level);
			}

			if (rc == 1)
			{
				if (ctrl_pdata->pwm_output_enable_cmds.cmd_cnt)
				{
					mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->pwm_output_enable_cmds);
/* E1M-634 - Add LCM BBS log */
					if( strstr(saved_command_line, "androidboot.device=E1M")!=NULL)
						fih_get_panel_status(ctrl_pdata, 0x9c);
/* end E1M-634 */
				}
			}
			//SW4-HL-Display-EnablePWMOutput-01*}_20150611
			break;
		}

		/*
		 * DCS commands to update backlight are usually sent at
		 * the same time to both the controllers. However, if
		 * sync_wait is enabled, we need to ensure that the
		 * dcs commands are first sent to the non-trigger
		 * controller so that when the commands are triggered,
		 * both controllers receive it at the same time.
		 */
		sctrl = mdss_dsi_get_other_ctrl(ctrl_pdata);
		if (mdss_dsi_sync_wait_trigger(ctrl_pdata)) {
			if (sctrl)
				mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
		} else {
			mdss_dsi_panel_bklt_dcs(ctrl_pdata, bl_level);
			if (sctrl)
				mdss_dsi_panel_bklt_dcs(sctrl, bl_level);
		}
		break;
	//SW4-HL-Dispay-BringUpNT35521S_ForM378M379-02*{_20151120
	//SW4-HL-Display-BringUpNT35521-00+{_20150224
	case BL_I2C:
		{
			switch (ctrl_pdata->panel_data.panel_info.pid)
			{
				case NT35521_720P_VIDEO_PANEL:
				case HX8394A_720P_VIDEO_PANEL:		//SW4-HL-Display-AddTianmaPanelHX8394DInsideSupport-00+_20150310
				case HX8394D_720P_VIDEO_PANEL:		//SW4-HL-Display-AddCTCPanelHX8394DInsideSupport-00+_20150317
				case NT35521S_720P_VIDEO_PANEL:		//SW4-HL-Dispay-BringUpNT35521S_ForM378M379-02+_20151120
				case NT35521S_NG_720P_VIDEO_PANEL:	//SW4-HL-Dispay-BringUpNT35521S_ForM378M379-02+_20151120
				case FT8716_1080P_VIDEO_PANEL:	//E1M
				case FT8716_720P_VIDEO_PANEL:	/* E1M-576 - gatycclu - Add 720P Video panel */
				default:
					{
						bl_level = transfer_bl_level(ctrl_pdata->panel_data.panel_info.pid, bl_level);	//SW4-HL-Dispay-BringUpNT35521S_ForM378M379-02+_20151120
						pr_debug("\n\n******************** [HL] %s: rt4501_set_backlight_level = %d  **********************\n\n",__func__, bl_level);
						rc = rt4501_set_backlight_level(bl_level);	//SW4-HL-FixBacklightAlwaysShowMaximumBrightnessWhenResume-00*_20150311
						if (rc >= 1)
						{
							pr_err("%s: level=%d\n", __func__, bl_level);
						}
					}
					break;
			}
		}
		break;
	//SW4-HL-Display-BringUpNT35521-00+}_20150224
	//SW4-HL-Dispay-BringUpNT35521S_ForM378M379-02*}_20151120
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
			__func__);
		break;
	}

	//SW4-HL-Display-EnablePWMOutput-01*{_20150611
	if (!ctrl_pdata->pre_cabc_off_cmds.cmd_cnt &&
		!ctrl_pdata->pre_cabc_ui_cmds.cmd_cnt &&
		!ctrl_pdata->pre_cabc_still_cmds.cmd_cnt &&
		!ctrl_pdata->pre_cabc_moving_cmds.cmd_cnt)
	{
		if (SendCABCOnlyAfterResume)
		{
			mdss_dsi_panel_cabc_set(ctrl_pdata, cabc_set);
			SendCABCOnlyAfterResume = 0;
		}
	}
	//SW4-HL-Display-EnablePWMOutput-01*}_20150611

	pr_debug("[HL]%s: <-- end\n", __func__);
}

//SW4-HL-Display-EnablePWMOutput-01+{_20150611
void mdss_dsi_panel_pre_ce_onoff(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned long enable)
{
	pr_debug("\n\n*** [HL] %s, enable = %ld ***\n\n", __func__,enable);

	switch (enable)
	{
		case 0:
			{
				mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->pre_ce_off_cmds);
			}
			break;
		case 1:
			{
				mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->pre_ce_on_cmds);
			}
			break;
	}

	pr_debug("\n\n******************** [HL] %s --- **********************\n\n", __func__);
}

void mdss_dsi_panel_pre_cabc_set(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned long value)
{
	pr_debug("\n\n*** [HL] %s, value = %ld ***\n\n", __func__,value);

	switch (value)
	{
		case CABC_OFF:
			{
				mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->pre_cabc_off_cmds);
			}
			break;
		case CABC_UI:
			{
				mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->pre_cabc_ui_cmds);
			}
			break;
		case CABC_STILL:
			{
				mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->pre_cabc_still_cmds);
			}
			break;
		case CABC_MOVING:
			{
				mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->pre_cabc_moving_cmds);
			}
			break;
	}

	pr_debug("\n\n******************** [HL] %s --- **********************\n\n", __func__);
}
//SW4-HL-Display-EnablePWMOutput-01+}_20150611

/* E1M-634 - Add LCM BBS log */
static char power_status_reg[2] = {0x0A, 0x00};
void fih_get_panel_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata, u8 check)
{
	char *rx_buf;

	rx_buf = kzalloc(PANEL_REG_ADDR_LEN, GFP_KERNEL);
	mdss_dsi_panel_cmd_read(ctrl_pdata, power_status_reg[0], power_status_reg[1],
							NULL, rx_buf, 1);
	pr_info("%s: LCM Driver status = 0x%x, check=0x%x\n", __func__, rx_buf[0], check);
	if( rx_buf[0] != check )
		BBOX_LCM_POWER_STATUS_ABNORMAL
	kfree(rx_buf);
}
/* end E1M-634 */

static int mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	int len = 1;		//SW4-HL-Display-EnhanceErrorHandling-00*_20150320
	int res = -EPERM;	//SW4-HL-Display-EnhanceErrorHandling-00*_20150320

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	if (ctrl->on_cmds.cmd_cnt)
	{
		//SW4-HL-Display-EnhanceErrorHandling-00*{_20150320
		len = mdss_dsi_panel_cmds_send(ctrl, &ctrl->on_cmds);
		if (!len)
		{
			goto cmds_fail;
		}
		//SW4-HL-Display-EnhanceErrorHandling-00*}_20150320

		//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+{_20151218
		if (!(ctrl->display_on_cmds.cmd_cnt))
		{
			gDisplayOnCmdAlreadySent = 1;
			pr_debug("\n\n[HL]%s: gDisplayOnCmdAlreadySent = 1\n", __func__);
		}
		//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+}_20151218
	}

	//SW4-HL-Display-EnablePWMOutput-01+{_20150611
	if (ctrl->pre_ce_off_cmds.cmd_cnt &&
		ctrl->pre_ce_on_cmds.cmd_cnt)
	{
		mdss_dsi_panel_pre_ce_onoff(ctrl, ce_en);
	}

	if (ctrl->pre_cabc_off_cmds.cmd_cnt &&
		ctrl->pre_cabc_ui_cmds.cmd_cnt &&
		ctrl->pre_cabc_still_cmds.cmd_cnt &&
		ctrl->pre_cabc_moving_cmds.cmd_cnt)
	{
		mdss_dsi_panel_pre_cabc_set(ctrl, cabc_set);
	}

	if (ctrl->sleep_out_cmds.cmd_cnt)
	{
		len = mdss_dsi_panel_cmds_send(ctrl, &ctrl->sleep_out_cmds);
		if (!len)
		{
			goto cmds_fail;
		}

		do_gettimeofday(&time_one);	//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+_20151218
	}

	if (ctrl->display_on_cmds.cmd_cnt)
	{
		gDisplayOnEnable = 1;
	}
	//SW4-HL-Display-EnablePWMOutput-01+}_20150611

	//SW4-HL-Display-FixShowBlackScreenAfterBootingIntoRecoveryMode-01*{_20150611
	if(strstr(saved_command_line, "androidboot.mode=1")!=NULL)
	{
		mdss_dsi_panel_bl_ctrl(pdata, 100);
	}
	//SW4-HL-Display-FixShowBlackScreenAfterBootingIntoRecoveryMode-01*}_20150611
/* E1M-576 - Add 720P Video panel */
	if(ctrl->panel_data.panel_info.pid == FT8716_1080P_VIDEO_PANEL ||
	    ctrl->panel_data.panel_info.pid == FT8716_720P_VIDEO_PANEL)
	  fih_fts_tp_lcm_resume();
/* end E1M-576 */

end:
	pinfo->blank_state = MDSS_PANEL_BLANK_UNBLANK;
	pr_debug("%s:-\n", __func__);
	return 0;

//SW4-HL-Display-EnhanceErrorHandling-00+{_20150320
cmds_fail:
	BBOX_LCM_DISPLA_ON_FAIL    /* E1M-634 - Add LCM BBS log */
	pr_err("%s: cmds send fail\n", __func__);
	return res;
//SW4-HL-Display-EnhanceErrorHandling-00+}_20150320
}
EXPORT_SYMBOL(gDisplayOnCmdAlreadySent);	//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+_20151218

static int mdss_dsi_post_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	struct dsi_panel_cmds *on_cmds;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);

	pinfo = &pdata->panel_info;
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

	on_cmds = &ctrl->post_panel_on_cmds;

	pr_debug("%s: ctrl=%pK cmd_cnt=%d\n", __func__, ctrl, on_cmds->cmd_cnt);

	if (on_cmds->cmd_cnt) {
		msleep(50);	/* wait for 3 vsync passed */
		mdss_dsi_panel_cmds_send(ctrl, on_cmds);
	}

end:
	pr_debug("%s:-\n", __func__);
	return 0;
}

//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+{_20151218
static int mdss_dsi_send_display_on_cmd(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int len = 1;
	int res = -EPERM;
	long time_diff=0;

	pr_debug("[HL]%s: <-- start\n", __func__);

	if (gDisplayOnEnable)
	{
		do_gettimeofday(&time_two);
		time_diff = time_two.tv_usec - time_one.tv_usec;
		if(time_diff < 0)
		{
			time_diff += 1000*1000;
		}

		if(time_diff < 0)
		{
			time_diff = 0;
		}

		pr_debug("\n\n[HL]%s: time_diff:%ld\n\n",__func__,time_diff);

		if(time_diff < 120 * 1000)
		{
			msleep((120 * 1000 - time_diff) / 1000);
			pr_debug("\n\n[HL]%s: need to msleep:%ld\n\n",__func__,(120 * 1000 - time_diff) / 1000);
		}

		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->display_on_cmds);
		if (!len)
		{
			goto cmds_fail;
		}

		gDisplayOnEnable = 0;

		gDisplayOnCmdAlreadySent = 1;
	}

	pr_debug("[HL]%s: <-- end\n", __func__);

	return len;

cmds_fail:
	pr_err("%s: cmds send fail\n", __func__);
	return res;
}
//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+}_20151218

static int mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;
	int len = 1;		//SW4-HL-Display-EnhanceErrorHandling-00*_20150320
	int res = -EPERM;	//SW4-HL-Display-EnhanceErrorHandling-00*_20150320

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d\n", __func__, ctrl, ctrl->ndx);

	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			goto end;
	}

/* E1M-576 - Add 720P Video panel */
	if(ctrl->panel_data.panel_info.pid == FT8716_1080P_VIDEO_PANEL ||
	    ctrl->panel_data.panel_info.pid == FT8716_720P_VIDEO_PANEL)
	    fih_fts_tp_lcm_suspend();
/* end E1M-576 */

	if (ctrl->off_cmds.cmd_cnt)
	{
		//SW4-HL-Display-EnhanceErrorHandling-00*{_20150320
		len = mdss_dsi_panel_cmds_send(ctrl, &ctrl->off_cmds);
		if (!len)
		{
			goto cmds_fail;
		}
		//SW4-HL-Display-EnhanceErrorHandling-00*}_20150320

		//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+_20151218
		gDisplayOnCmdAlreadySent = 0;
		pr_debug("\n\n[HL]%s: gDisplayOnCmdAlreadySent = 0\n", __func__);
	}

end:
	pinfo->blank_state = MDSS_PANEL_BLANK_BLANK;
/* E1M-634 - Add LCM BBS log */
	if( strstr(saved_command_line, "androidboot.device=E1M")!=NULL)
		fih_get_panel_status(ctrl, 0x08);
/* end E1M-634 */
	pr_debug("%s:-\n", __func__);
	return 0;

//SW4-HL-Display-EnhanceErrorHandling-00+{_20150320
cmds_fail:
	BBOX_LCM_DISPLA_OFF_FAIL    /* E1M-634 - Add LCM BBS log */
	pr_err("%s: cmds send fail\n", __func__);
	return res;
//SW4-HL-Display-EnhanceErrorHandling-00+}_20150320
}

static int mdss_dsi_panel_low_power_config(struct mdss_panel_data *pdata,
	int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_panel_info *pinfo;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pinfo = &pdata->panel_info;
	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	pr_debug("%s: ctrl=%pK ndx=%d enable=%d\n", __func__, ctrl, ctrl->ndx,
		enable);

	/* Any panel specific low power commands/config */
	if (enable)
		pinfo->blank_state = MDSS_PANEL_BLANK_LOW_POWER;
	else
		pinfo->blank_state = MDSS_PANEL_BLANK_UNBLANK;

	pr_debug("%s:-\n", __func__);
	return 0;
}

//SW4-HL-Display-EnhanceErrorHandling-00*{_20150320
//SW4-HL-Display-BringUpNT35521-00+{_20150224
int mdss_dsi_panel_ce_onoff(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned long enable)
{
	int len = 1;

	pr_debug("\n\n*** [HL] %s, enable = %ld ***\n\n", __func__,enable);

	if (!(ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT))
	{
		pr_err("%s, panel not init yet, not allow to set CE command!\n", __func__);
		return -EBUSY;
	}
	else
	{
		pr_debug("\n\n*** [HL] %s, panel already init, allow to set CE command! ***\n\n", __func__);
	}

	if (enable == 1)
	{
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->ce_on_cmds);
	}
	else if (enable == 0)
	{
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->ce_off_cmds);
	}
	else
	{
		pr_debug("\n\n*** %s, Invlid input parameter ***\n\n", __func__);
		return -EINVAL;
	}

/* E1M-634 - Add LCM BBS log */
	if (!len)
	{
		BBOX_LCM_OEM_FUNCTIONS_FAIL
	}
/* end E1M-634 */

	ce_status = enable;

	pr_debug("\n\n******************** [HL] %s ---, len = %d **********************\n\n", __func__, len);

	return len;
}

int mdss_dsi_panel_ct_set(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned long value)
{
	int len = 1;

	pr_debug("\n\n*** [HL] %s, value = %ld ***\n\n", __func__,value);

	if (!(ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT))
	{
		pr_err("%s, panel not init yet, not allow to set CT command!\n", __func__);
		return -EBUSY;
	}
	else
	{
		pr_debug("\n\n*** [HL] %s, panel already init, allow to set CT command! ***\n\n", __func__);
	}

	switch (value) {
	case COLOR_TEMP_NORMAL:
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->ct_normal_cmds);
		break;
	case COLOR_TEMP_WARM:
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->ct_warm_cmds);
		break;
	case COLOR_TEMP_COLD:
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->ct_cold_cmds);
		break;
	case BL_FILTER_DISABLE:
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->ct_normal_cmds);
		break;
	case BL_FILTER_10:
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->blf_10_cmds);
		break;
	case BL_FILTER_30:
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->blf_30_cmds);
		break;
	case BL_FILTER_50:
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->blf_50_cmds);
		break;
	case BL_FILTER_75:
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->blf_75_cmds);
		break;
	default:
		pr_debug("%s: unhandled value=%ld\n", __func__, value);
		return -EINVAL;
		break;
	}

/* E1M-634 - Add LCM BBS log */
	if (!len)
	{
		BBOX_LCM_OEM_FUNCTIONS_FAIL
	}
/* end E1M-634 */

	ct_status = value;

	pr_debug("\n\n******************** [HL] %s ---, len = %d **********************\n\n", __func__, len);

	return len;
}

int mdss_dsi_panel_cabc_set(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned long value)
{
	int len = 1;

	pr_debug("\n\n*** [HL] %s, value = %ld ***\n\n", __func__,value);

	if (!(ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT))
	{
		pr_err("%s, panel not init yet, not allow to set CABC command!\n", __func__);
		return -EBUSY;
	}
	else
	{
		pr_debug("\n\n*** [HL] %s, panel already init, allow to set CABC command! ***\n\n", __func__);
	}

	switch (value)
	{
		case CABC_OFF:
			len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->cabc_off_cmds);
			break;
		case CABC_UI:
			len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->cabc_ui_cmds);
			break;
		case CABC_STILL:
			len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->cabc_still_cmds);
			break;
		case CABC_MOVING:
			len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->cabc_moving_cmds);
			break;
		default:
			pr_debug("%s: unhandled value=%ld\n", __func__, value);
			return -EINVAL;
			break;
	}

/* E1M-634 - Add LCM BBS log */
	if (!len)
	{
		BBOX_LCM_OEM_FUNCTIONS_FAIL
	}
/* end E1M-634 */

	cabc_status = value;

	pr_debug("\n\n******************** [HL] %s ---, len = %d **********************\n\n", __func__, len);

	return len;
}
//SW4-HL-Display-BringUpNT35521-00+}_20150224
//SW4-HL-Display-EnhanceErrorHandling-00*}_20150320

/* E1M-4489 - Add SVI(AIE) setting */
int mdss_dsi_panel_svi_set(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned long value)
{
	int len = 1;

	pr_debug("\n\n*** [HL] %s, value = %ld ***\n\n", __func__,value);

	if (!(ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT))
	{
		pr_err("%s, panel not init yet, not allow to set SVI command!\n", __func__);
		return -EBUSY;
	}
	else
	{
		pr_debug("\n\n*** [HL] %s, panel already init, allow to set SVI command! ***\n\n", __func__);
	}

	if (value)
	{
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->svi_on_cmds);
	} else {
		len = mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->svi_off_cmds);
	}

	if (!len)
	{
		BBOX_LCM_OEM_FUNCTIONS_FAIL
	}

	pr_debug("\n\n******************** [HL] %s ---, len = %d **********************\n\n", __func__, len);

	return len;
}
/* end E1M-4489 */

/* E1M-576 - Add LCM mipi reg read/write command */
static int tot_reg_val_len = 0;
static char res_reg_val[2];
void mdss_dsi_panel_read_reg_get(char *reg_val)
{

	if (tot_reg_val_len < 2)
	{
		sprintf(reg_val, "0x%x\n", res_reg_val[0]);
	}
	else
	{
		sprintf(reg_val, "0x%x,0x%x\n", res_reg_val[0], res_reg_val[1]);
	}

	pr_err("\n\n******************** [HL] %s ---, reg_val = (%s) **********************\n\n", __func__, reg_val);

	return;
}

static char read_reg[2] = {0x0A, 0x00};
void mdss_dsi_panel_read_reg_set(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned int reg, unsigned int reg_len)
{
	char *rx_buf;
	int i = 0;

	pr_err("\n\n*** [HL] %s, reg = 0x%x, reg_len = %d ***\n\n", __func__, reg, reg_len);
	pr_err("\n\n*** [HL] %s, ctrl_pdata->ctrl_state = %d ***\n\n", __func__, ctrl_pdata->ctrl_state);

	if (!(ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT))
	{
		pr_err("%s, panel not init yet, not allow to set read reg command!\n", __func__);
		res_reg_val[0] = 0;
		res_reg_val[1] = 0;
		pr_err("%s, Clear the array which keeps the return value of lcm driver ic to 0x00!\n", __func__);
		return;
	}
	else
	{
		pr_err("\n\n*** [HL] %s, panel already init, allow to set read reg command! ***\n\n", __func__);
	}

	rx_buf = kzalloc(PANEL_REG_ADDR_LEN, GFP_KERNEL);

	read_reg[0] = reg;
	mdss_dsi_panel_cmd_read(ctrl_pdata, read_reg[0], read_reg[1],
					NULL, rx_buf, reg_len);

	pr_err("%s: (reg, value) = (0x%x, 0x%x)\n", __func__, reg, rx_buf[0]);

	//memcpy(res_reg_val, rx_buf, sizeof(res_reg_val));
	for (i = 0; i < reg_len; i++)
	{
		res_reg_val[i] = rx_buf[i];
	}
	tot_reg_val_len = reg_len;

	pr_err("\n\n******************** [HL] %s: res_reg_val = (0x%x) **********************\n\n", __func__, res_reg_val[0]);

	kfree(rx_buf);

	pr_err("\n\n******************** [HL] %s --- **********************\n\n", __func__);

	return;
}

void mdss_dsi_panel_write_reg_set(struct mdss_dsi_ctrl_pdata *ctrl_pdata, unsigned int len, char *data)
{
	char *delim = ",";
	char *token;
	int i = 0;
	long input = 0;

	pr_err("\n\n*** [HL] %s, len = %d, data = (%s) ***\n\n", __func__, len, data);

	if (!(ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT))
	{
		pr_err("%s, panel not init yet, not allow to set read reg command!\n", __func__);
		return;
	}
	else
	{
		pr_err("\n\n*** [HL] %s, panel already init, allow to set read reg command! ***\n\n", __func__);
	}

	if (ctrl_pdata->write_reg_cmds.cmd_cnt)
	{
		//Dcs command length
		ctrl_pdata->write_reg_cmds.blen = len;

		//Dcs command register and data
		for(token = strsep(&data, delim); token != NULL; token = strsep(&data, delim))
		{
			pr_err("\n\n******************** [HL] %s: data = %s **********************\n\n", __func__, token);
			if (strict_strtol(token, 16, &input))
			{
				return;
			}
			ctrl_pdata->write_reg_cmds.cmds->payload[i] = input;

			i++;
		}

		//Send Dcs command
		mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->write_reg_cmds);
	}
	else
	{
		pr_err("\n\n*** [HL] %s, not define write_reg_cmds in panel.dtsi ***\n\n", __func__);
	}

	pr_err("\n\n******************** [HL] %s ---**********************\n\n", __func__);

	return;
}
/* end E1M-576 */

static void mdss_dsi_parse_lane_swap(struct device_node *np, char *dlane_swap)
{
	const char *data;

	*dlane_swap = DSI_LANE_MAP_0123;
	data = of_get_property(np, "qcom,mdss-dsi-lane-map", NULL);
	if (data) {
		if (!strcmp(data, "lane_map_3012"))
			*dlane_swap = DSI_LANE_MAP_3012;
		else if (!strcmp(data, "lane_map_2301"))
			*dlane_swap = DSI_LANE_MAP_2301;
		else if (!strcmp(data, "lane_map_1230"))
			*dlane_swap = DSI_LANE_MAP_1230;
		else if (!strcmp(data, "lane_map_0321"))
			*dlane_swap = DSI_LANE_MAP_0321;
		else if (!strcmp(data, "lane_map_1032"))
			*dlane_swap = DSI_LANE_MAP_1032;
		else if (!strcmp(data, "lane_map_2103"))
			*dlane_swap = DSI_LANE_MAP_2103;
		else if (!strcmp(data, "lane_map_3210"))
			*dlane_swap = DSI_LANE_MAP_3210;
	}
}

static void mdss_dsi_parse_trigger(struct device_node *np, char *trigger,
		char *trigger_key)
{
	const char *data;

	*trigger = DSI_CMD_TRIGGER_SW;
	data = of_get_property(np, trigger_key, NULL);
	if (data) {
		if (!strcmp(data, "none"))
			*trigger = DSI_CMD_TRIGGER_NONE;
		else if (!strcmp(data, "trigger_te"))
			*trigger = DSI_CMD_TRIGGER_TE;
		else if (!strcmp(data, "trigger_sw_seof"))
			*trigger = DSI_CMD_TRIGGER_SW_SEOF;
		else if (!strcmp(data, "trigger_sw_te"))
			*trigger = DSI_CMD_TRIGGER_SW_TE;
	}
}


static int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key)
{
	const char *data;
	int blen = 0, len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	data = of_get_property(np, cmd_key, &blen);
	if (!data) {
		pr_err("%s: failed, key=%s\n", __func__, cmd_key);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, data, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len >= sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	/*Set default link state to LP Mode*/
	pcmds->link_state = DSI_LP_MODE;

	if (link_key) {
		data = of_get_property(np, link_key, NULL);
		if (data && !strcmp(data, "dsi_hs_mode"))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	}

	pr_debug("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%d\n", __func__,
		pcmds->buf[0], pcmds->blen, pcmds->cmd_cnt, pcmds->link_state);

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;
}


int mdss_panel_get_dst_fmt(u32 bpp, char mipi_mode, u32 pixel_packing,
				char *dst_format)
{
	int rc = 0;
	switch (bpp) {
	case 3:
		*dst_format = DSI_CMD_DST_FORMAT_RGB111;
		break;
	case 8:
		*dst_format = DSI_CMD_DST_FORMAT_RGB332;
		break;
	case 12:
		*dst_format = DSI_CMD_DST_FORMAT_RGB444;
		break;
	case 16:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB565;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB565;
			break;
		}
		break;
	case 18:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB666;
			break;
		default:
			if (pixel_packing == 0)
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666;
			else
				*dst_format = DSI_VIDEO_DST_FORMAT_RGB666_LOOSE;
			break;
		}
		break;
	case 24:
		switch (mipi_mode) {
		case DSI_VIDEO_MODE:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		case DSI_CMD_MODE:
			*dst_format = DSI_CMD_DST_FORMAT_RGB888;
			break;
		default:
			*dst_format = DSI_VIDEO_DST_FORMAT_RGB888;
			break;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}


static int mdss_dsi_parse_fbc_params(struct device_node *np,
				struct mdss_panel_info *panel_info)
{
	int rc, fbc_enabled = 0;
	u32 tmp;

	fbc_enabled = of_property_read_bool(np,	"qcom,mdss-dsi-fbc-enable");
	if (fbc_enabled) {
		pr_debug("%s:%d FBC panel enabled.\n", __func__, __LINE__);
		panel_info->fbc.enabled = 1;
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bpp", &tmp);
		panel_info->fbc.target_bpp =	(!rc ? tmp : panel_info->bpp);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-packing",
				&tmp);
		panel_info->fbc.comp_mode = (!rc ? tmp : 0);
		panel_info->fbc.qerr_enable = of_property_read_bool(np,
			"qcom,mdss-dsi-fbc-quant-error");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-bias", &tmp);
		panel_info->fbc.cd_bias = (!rc ? tmp : 0);
		panel_info->fbc.pat_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-pat-mode");
		panel_info->fbc.vlc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-vlc-mode");
		panel_info->fbc.bflc_enable = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-bflc-mode");
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-h-line-budget",
				&tmp);
		panel_info->fbc.line_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-budget-ctrl",
				&tmp);
		panel_info->fbc.block_x_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-block-budget",
				&tmp);
		panel_info->fbc.block_budget = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossless-threshold", &tmp);
		panel_info->fbc.lossless_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-threshold", &tmp);
		panel_info->fbc.lossy_mode_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-fbc-rgb-threshold",
				&tmp);
		panel_info->fbc.lossy_rgb_thd = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-lossy-mode-idx", &tmp);
		panel_info->fbc.lossy_mode_idx = (!rc ? tmp : 0);
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-slice-height", &tmp);
		panel_info->fbc.slice_height = (!rc ? tmp : 0);
		panel_info->fbc.pred_mode = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-2d-pred-mode");
		panel_info->fbc.enc_mode = of_property_read_bool(np,
				"qcom,mdss-dsi-fbc-ver2-mode");
		rc = of_property_read_u32(np,
				"qcom,mdss-dsi-fbc-max-pred-err", &tmp);
		panel_info->fbc.max_pred_err = (!rc ? tmp : 0);
	} else {
		pr_debug("%s:%d Panel does not support FBC.\n",
				__func__, __LINE__);
		panel_info->fbc.enabled = 0;
		panel_info->fbc.target_bpp =
			panel_info->bpp;
	}
	return 0;
}

static void mdss_panel_parse_te_params(struct device_node *np,
				       struct mdss_panel_info *panel_info)
{

	u32 tmp;
	int rc = 0;
	/*
	 * TE default: dsi byte clock calculated base on 70 fps;
	 * around 14 ms to complete a kickoff cycle if te disabled;
	 * vclk_line base on 60 fps; write is faster than read;
	 * init == start == rdptr;
	 */
	panel_info->te.tear_check_en =
		!of_property_read_bool(np, "qcom,mdss-tear-check-disable");
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-cfg-height", &tmp);
	panel_info->te.sync_cfg_height = (!rc ? tmp : 0xfff0);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-init-val", &tmp);
	panel_info->te.vsync_init_val = (!rc ? tmp : panel_info->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-start", &tmp);
	panel_info->te.sync_threshold_start = (!rc ? tmp : 4);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-sync-threshold-continue", &tmp);
	panel_info->te.sync_threshold_continue = (!rc ? tmp : 4);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-start-pos", &tmp);
	panel_info->te.start_pos = (!rc ? tmp : panel_info->yres);
	rc = of_property_read_u32
		(np, "qcom,mdss-tear-check-rd-ptr-trigger-intr", &tmp);
	panel_info->te.rd_ptr_irq = (!rc ? tmp : panel_info->yres + 1);
	rc = of_property_read_u32(np, "qcom,mdss-tear-check-frame-rate", &tmp);
	panel_info->te.refx100 = (!rc ? tmp : 6000);
}


static int mdss_dsi_parse_reset_seq(struct device_node *np,
		u32 rst_seq[MDSS_DSI_RST_SEQ_LEN], u32 *rst_len,
		const char *name)
{
	int num = 0, i;
	int rc;
	struct property *data;
	u32 tmp[MDSS_DSI_RST_SEQ_LEN];
	*rst_len = 0;
	data = of_find_property(np, name, &num);
	num /= sizeof(u32);
	if (!data || !num || num > MDSS_DSI_RST_SEQ_LEN || num % 2) {
		pr_debug("%s:%d, error reading %s, length found = %d\n",
			__func__, __LINE__, name, num);
	} else {
		rc = of_property_read_u32_array(np, name, tmp, num);
		if (rc)
			pr_debug("%s:%d, error reading %s, rc = %d\n",
				__func__, __LINE__, name, rc);
		else {
			for (i = 0; i < num; ++i)
				rst_seq[i] = tmp[i];
			*rst_len = num;
		}
	}
	return 0;
}

/*
 * Because msm8909 ESD driver do not support to read multiple registers to detect LCM IC status,
 * we upgrade ESD driver to msm8937 version
 * Add function: mdss_dsi_cmp_panel_reg_v2
 */
static bool mdss_dsi_cmp_panel_reg_v2(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int i, j;
	int len = 0, *lenp;
	int group = 0;

	lenp = ctrl->status_valid_params ?: ctrl->status_cmds_rlen;

	for (i = 0; i < ctrl->status_cmds.cmd_cnt; i++)
		len += lenp[i];

	for (j = 0; j < ctrl->groups; ++j) {
		for (i = 0; i < len; ++i) {
			pr_debug("%s: [LCM-ESD] panel status = 0x%x (0x%x)\n", __func__,
				 ctrl->return_buf[i], ctrl->status_value[group + i]);
			if (ctrl->return_buf[i] !=
				ctrl->status_value[group + i])
				break;
		}

		if (i == len)
			return true;
		group += len;
	}

	return false;
}

static int mdss_dsi_gen_read_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
/*
 * Because msm8909 ESD driver do not support to read multiple registers to detect LCM IC status,
 * we upgrade ESD driver to msm8937 version
 */
#if 0 //msm8909 version
	if (ctrl_pdata->status_buf.data[0] !=
					ctrl_pdata->status_value) {
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	} else {
		pr_info("%s: Panel Driver IC is alive!, status = 0x%x\n", __func__, ctrl_pdata->status_buf.data[0]);
		return 1;
	}
#else //msm8937 version
	if (!mdss_dsi_cmp_panel_reg_v2(ctrl_pdata)) {
		ctrl_pdata->panel_data.panel_info.old_bl = old_bl;
		pr_err("%s: [LCM-ESD] ctrl_pdata->panel_data.panel_info.old_bl=%d\n",
							__func__, ctrl_pdata->panel_data.panel_info.old_bl);
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	} else {
		return 1;
	}
#endif
}

static int mdss_dsi_nt35596_read_status(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
/*
 * Because msm8909 ESD driver do not support to read multiple registers to detect LCM IC status,
 * we upgrade ESD driver to msm8937 version
 */
#if 0 //msm8909 version
	if (ctrl_pdata->status_buf.data[0] !=
					ctrl_pdata->status_value) {
		ctrl_pdata->status_error_count = 0;
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	} else {
		if (ctrl_pdata->status_buf.data[3] != NT35596_BUF_3_STATUS) {
			ctrl_pdata->status_error_count = 0;
		} else {
			if ((ctrl_pdata->status_buf.data[4] ==
				NT35596_BUF_4_STATUS) ||
				(ctrl_pdata->status_buf.data[5] ==
				NT35596_BUF_5_STATUS))
				ctrl_pdata->status_error_count = 0;
			else
				ctrl_pdata->status_error_count++;
			if (ctrl_pdata->status_error_count >=
					NT35596_MAX_ERR_CNT) {
				ctrl_pdata->status_error_count = 0;
				pr_err("%s: Read value bad. Error_cnt = %i\n",
					 __func__,
					ctrl_pdata->status_error_count);
				return -EINVAL;
			}
		}
		return 1;
	}
#else //msm8937 version
	if (!mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
		ctrl_pdata->status_value, 0)) {
		ctrl_pdata->status_error_count = 0;
		pr_err("%s: Read back value from panel is incorrect\n",
							__func__);
		return -EINVAL;
	} else {
		if (!mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
			ctrl_pdata->status_value, 3)) {
			ctrl_pdata->status_error_count = 0;
		} else {
			if (mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
				ctrl_pdata->status_value, 4) ||
				mdss_dsi_cmp_panel_reg(ctrl_pdata->status_buf,
				ctrl_pdata->status_value, 5))
				ctrl_pdata->status_error_count = 0;
			else
				ctrl_pdata->status_error_count++;
			if (ctrl_pdata->status_error_count >=
					ctrl_pdata->max_status_error_count) {
				ctrl_pdata->status_error_count = 0;
				pr_err("%s: Read value bad. Error_cnt = %i\n",
					 __func__,
					ctrl_pdata->status_error_count);
				return -EINVAL;
			}
		}
		return 1;
	}
#endif
}

static void mdss_dsi_parse_roi_alignment(struct device_node *np,
		struct mdss_panel_info *pinfo)
{
	int len = 0;
	u32 value[6];
	struct property *data;
	data = of_find_property(np, "qcom,panel-roi-alignment", &len);
	len /= sizeof(u32);
	if (!data || (len != 6)) {
		pr_debug("%s: Panel roi alignment not found", __func__);
	} else {
		int rc = of_property_read_u32_array(np,
				"qcom,panel-roi-alignment", value, len);
		if (rc)
			pr_debug("%s: Error reading panel roi alignment values",
					__func__);
		else {
			pinfo->xstart_pix_align = value[0];
			pinfo->width_pix_align = value[1];
			pinfo->ystart_pix_align = value[2];
			pinfo->height_pix_align = value[3];
			pinfo->min_width = value[4];
			pinfo->min_height = value[5];
		}

		pr_debug("%s: ROI alignment: [%d, %d, %d, %d, %d, %d]",
				__func__, pinfo->xstart_pix_align,
				pinfo->width_pix_align, pinfo->ystart_pix_align,
				pinfo->height_pix_align, pinfo->min_width,
				pinfo->min_height);
	}
}

/*
 * Because msm8909 ESD driver do not support to read multiple registers to detect LCM IC status,
 * we upgrade ESD driver to msm8937 version
 * Add new function:
 * mdss_dsi_parse_esd_check_valid_params
 * mdss_dsi_parse_esd_status_len
 * mdss_dsi_parse_esd_params
 */
/* the length of all the valid values to be checked should not be great
 * than the length of returned data from read command.
 */
static bool
mdss_dsi_parse_esd_check_valid_params(struct mdss_dsi_ctrl_pdata *ctrl)
{
	int i;

	for (i = 0; i < ctrl->status_cmds.cmd_cnt; ++i) {
		if (ctrl->status_valid_params[i] > ctrl->status_cmds_rlen[i]) {
			pr_debug("%s: ignore valid params!\n", __func__);
			return false;
		}
	}

	return true;
}

static bool mdss_dsi_parse_esd_status_len(struct device_node *np,
	char *prop_key, u32 **target, u32 cmd_cnt)
{
	int tmp;

	if (!of_find_property(np, prop_key, &tmp))
		return false;

	tmp /= sizeof(u32);
	if (tmp != cmd_cnt) {
		pr_err("%s: request property number(%d) not match command count(%d)\n",
			__func__, tmp, cmd_cnt);
		return false;
	}

	*target = kcalloc(tmp, sizeof(u32), GFP_KERNEL);
	if (IS_ERR_OR_NULL(*target)) {
		pr_err("%s: Error allocating memory for property\n",
			__func__);
		return false;
	}

	if (of_property_read_u32_array(np, prop_key, *target, tmp)) {
		pr_err("%s: cannot get values from dts\n", __func__);
		kfree(*target);
		*target = NULL;
		return false;
	}

	return true;
}

static void mdss_dsi_parse_esd_params(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	u32 tmp;
	u32 i, status_len, *lenp;
	int rc;
	struct property *data;
	const char *string;
	struct mdss_panel_info *pinfo = &ctrl->panel_data.panel_info;

	pinfo->esd_check_enabled = of_property_read_bool(np,
		"qcom,esd-check-enabled");

	/* E1M: Fix no backlight while playing bootanimation*/
	pinfo->old_bl = -1;

	if (!pinfo->esd_check_enabled && pinfo->pid != FT8716_720P_VIDEO_PANEL)
		return;

	ctrl->status_mode = ESD_MAX;
	rc = of_property_read_string(np,
			"qcom,mdss-dsi-panel-status-check-mode", &string);
	if (!rc) {
		if (!strcmp(string, "bta_check")) {
			ctrl->status_mode = ESD_BTA;
		} else if (!strcmp(string, "reg_read")) {
			ctrl->status_mode = ESD_REG;
			ctrl->check_read_status =
				mdss_dsi_gen_read_status;
		} else if (!strcmp(string, "reg_read_nt35596")) {
			ctrl->status_mode = ESD_REG_NT35596;
			ctrl->status_error_count = 0;
			ctrl->check_read_status =
				mdss_dsi_nt35596_read_status;
		} else if (!strcmp(string, "te_signal_check")) {
			if (pinfo->mipi.mode == DSI_CMD_MODE) {
				ctrl->status_mode = ESD_TE;
			} else {
				pr_err("TE-ESD not valid for video mode\n");
				goto error;
			}
		} else {
			pr_err("No valid panel-status-check-mode string\n");
			goto error;
		}
	}

	if ((ctrl->status_mode == ESD_BTA) || (ctrl->status_mode == ESD_TE) ||
			(ctrl->status_mode == ESD_MAX))
		return;

	mdss_dsi_parse_dcs_cmds(np, &ctrl->status_cmds,
			"qcom,mdss-dsi-panel-status-command",
				"qcom,mdss-dsi-panel-status-command-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-max-error-count",
		&tmp);
	ctrl->max_status_error_count = (!rc ? tmp : 0);

	if (!mdss_dsi_parse_esd_status_len(np,
		"qcom,mdss-dsi-panel-status-read-length",
		&ctrl->status_cmds_rlen, ctrl->status_cmds.cmd_cnt)) {
		pinfo->esd_check_enabled = false;
		return;
	}

	if (mdss_dsi_parse_esd_status_len(np,
		"qcom,mdss-dsi-panel-status-valid-params",
		&ctrl->status_valid_params, ctrl->status_cmds.cmd_cnt)) {
		if (!mdss_dsi_parse_esd_check_valid_params(ctrl))
			goto error1;
	}

	status_len = 0;
	lenp = ctrl->status_valid_params ?: ctrl->status_cmds_rlen;
	for (i = 0; i < ctrl->status_cmds.cmd_cnt; ++i)
		status_len += lenp[i];

	data = of_find_property(np, "qcom,mdss-dsi-panel-status-value", &tmp);
	tmp /= sizeof(u32);
	if (!IS_ERR_OR_NULL(data) && tmp != 0 && (tmp % status_len) == 0) {
		ctrl->groups = tmp / status_len;
	} else {
		pr_err("%s: Error parse panel-status-value\n", __func__);
		goto error1;
	}

	ctrl->status_value = kzalloc(sizeof(u32) * status_len * ctrl->groups,
				GFP_KERNEL);
	if (!ctrl->status_value)
		goto error1;

	ctrl->return_buf = kcalloc(status_len * ctrl->groups,
			sizeof(unsigned char), GFP_KERNEL);
	if (!ctrl->return_buf)
		goto error2;

	rc = of_property_read_u32_array(np,
		"qcom,mdss-dsi-panel-status-value",
		ctrl->status_value, ctrl->groups * status_len);
	if (rc) {
		pr_debug("%s: Error reading panel status values\n",
				__func__);
		memset(ctrl->status_value, 0, ctrl->groups * status_len);
	}

	return;

error2:
	kfree(ctrl->status_value);
error1:
	kfree(ctrl->status_valid_params);
	kfree(ctrl->status_cmds_rlen);
error:
	pinfo->esd_check_enabled = false;
}

static int mdss_dsi_parse_panel_features(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	struct mdss_panel_info *pinfo;

	if (!np || !ctrl) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	pinfo = &ctrl->panel_data.panel_info;

	pinfo->cont_splash_enabled = of_property_read_bool(np,
		"qcom,cont-splash-enabled");

	if (pinfo->mipi.mode == DSI_CMD_MODE) {
		pinfo->partial_update_enabled = of_property_read_bool(np,
				"qcom,partial-update-enabled");
		pr_info("%s: partial_update_enabled=%d\n", __func__,
					pinfo->partial_update_enabled);
		if (pinfo->partial_update_enabled) {
			ctrl->set_col_page_addr = mdss_dsi_set_col_page_addr;
			pinfo->partial_update_roi_merge =
					of_property_read_bool(np,
					"qcom,partial-update-roi-merge");
		}

		pinfo->dcs_cmd_by_left = of_property_read_bool(np,
						"qcom,dcs-cmd-by-left");
	}

	pinfo->ulps_feature_enabled = of_property_read_bool(np,
		"qcom,ulps-enabled");
	pr_info("%s: ulps feature %s\n", __func__,
		(pinfo->ulps_feature_enabled ? "enabled" : "disabled"));

/*
 * Because msm8909 ESD driver do not support to read multiple registers to detect LCM IC status,
 * we upgrade ESD driver to msm8937 version
 */
#if 0 //msm8909 version, move to mdss_dsi_parse_esd_params
	pinfo->esd_check_enabled = of_property_read_bool(np,
		"qcom,esd-check-enabled");
#endif

	pinfo->ulps_suspend_enabled = of_property_read_bool(np,
		"qcom,suspend-ulps-enabled");
	pr_info("%s: ulps during suspend feature %s", __func__,
		(pinfo->ulps_suspend_enabled ? "enabled" : "disabled"));

	pinfo->mipi.dynamic_switch_enabled = of_property_read_bool(np,
		"qcom,dynamic-mode-switch-enabled");

	if (pinfo->mipi.dynamic_switch_enabled) {
		mdss_dsi_parse_dcs_cmds(np, &ctrl->video2cmd,
			"qcom,video-to-cmd-mode-switch-commands", NULL);

		mdss_dsi_parse_dcs_cmds(np, &ctrl->cmd2video,
			"qcom,cmd-to-video-mode-switch-commands", NULL);

		if (!ctrl->video2cmd.cmd_cnt || !ctrl->cmd2video.cmd_cnt) {
			pr_warn("No commands specified for dynamic switch\n");
			pinfo->mipi.dynamic_switch_enabled = 0;
		}
	}

	pr_info("%s: dynamic switch feature enabled: %d\n", __func__,
		pinfo->mipi.dynamic_switch_enabled);
	pinfo->panel_ack_disabled = of_property_read_bool(np,
				"qcom,panel-ack-disabled");

	mdss_dsi_parse_esd_params(np, ctrl); //msm8937 version

	if (pinfo->panel_ack_disabled && pinfo->esd_check_enabled) {
		pr_warn("ESD should not be enabled if panel ACK is disabled\n");
		pinfo->esd_check_enabled = false;
	}

	if (ctrl->disp_en_gpio <= 0) {
		ctrl->disp_en_gpio = of_get_named_gpio(
			np,
			"qcom,5v-boost-gpio", 0);

		if (!gpio_is_valid(ctrl->disp_en_gpio))
			pr_err("%s:%d, Disp_en gpio not specified\n",
					__func__, __LINE__);
	}

	return 0;
}

static void mdss_dsi_parse_panel_horizintal_line_idle(struct device_node *np,
	struct mdss_dsi_ctrl_pdata *ctrl)
{
	const u32 *src;
	int i, len, cnt;
	struct panel_horizontal_idle *kp;

	if (!np || !ctrl) {
		pr_err("%s: Invalid arguments\n", __func__);
		return;
	}

	src = of_get_property(np, "qcom,mdss-dsi-hor-line-idle", &len);
	if (!src || len == 0)
		return;

	cnt = len % 3; /* 3 fields per entry */
	if (cnt) {
		pr_err("%s: invalid horizontal idle len=%d\n", __func__, len);
		return;
	}

	cnt = len / sizeof(u32);

	kp = kzalloc(sizeof(*kp) * (cnt / 3), GFP_KERNEL);
	if (kp == NULL) {
		pr_err("%s: No memory\n", __func__);
		return;
	}

	ctrl->line_idle = kp;
	for (i = 0; i < cnt; i += 3) {
		kp->min = be32_to_cpu(src[i]);
		kp->max = be32_to_cpu(src[i+1]);
		kp->idle = be32_to_cpu(src[i+2]);
		kp++;
		ctrl->horizontal_idle_cnt++;
	}

	pr_debug("%s: horizontal_idle_cnt=%d\n", __func__,
				ctrl->horizontal_idle_cnt);
}

static int mdss_dsi_set_refresh_rate_range(struct device_node *pan_node,
		struct mdss_panel_info *pinfo)
{
	int rc = 0;
	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-min-refresh-rate",
			&pinfo->min_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read min refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since min refresh rate is not specified when dynamic
		 * fps is enabled, using minimum as 30
		 */
		pinfo->min_fps = MIN_REFRESH_RATE;
		rc = 0;
	}

	rc = of_property_read_u32(pan_node,
			"qcom,mdss-dsi-max-refresh-rate",
			&pinfo->max_fps);
	if (rc) {
		pr_warn("%s:%d, Unable to read max refresh rate\n",
				__func__, __LINE__);

		/*
		 * Since max refresh rate was not specified when dynamic
		 * fps is enabled, using the default panel refresh rate
		 * as max refresh rate supported.
		 */
		pinfo->max_fps = pinfo->mipi.frame_rate;
		rc = 0;
	}

	pr_info("dyn_fps: min = %d, max = %d\n",
			pinfo->min_fps, pinfo->max_fps);
	return rc;
}

static void mdss_dsi_parse_dfps_config(struct device_node *pan_node,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	const char *data;
	bool dynamic_fps;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	dynamic_fps = of_property_read_bool(pan_node,
			"qcom,mdss-dsi-pan-enable-dynamic-fps");

	if (!dynamic_fps)
		return;

	pinfo->dynamic_fps = true;
	data = of_get_property(pan_node, "qcom,mdss-dsi-pan-fps-update", NULL);
	if (data) {
		if (!strcmp(data, "dfps_suspend_resume_mode")) {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("dfps mode: suspend/resume\n");
		} else if (!strcmp(data, "dfps_immediate_clk_mode")) {
			pinfo->dfps_update = DFPS_IMMEDIATE_CLK_UPDATE_MODE;
			pr_debug("dfps mode: Immediate clk\n");
		} else if (!strcmp(data, "dfps_immediate_porch_mode")) {
			pinfo->dfps_update = DFPS_IMMEDIATE_PORCH_UPDATE_MODE;
			pr_debug("dfps mode: Immediate porch\n");
		} else {
			pinfo->dfps_update = DFPS_SUSPEND_RESUME_MODE;
			pr_debug("default dfps mode: suspend/resume\n");
		}
		mdss_dsi_set_refresh_rate_range(pan_node, pinfo);
	} else {
		pinfo->dynamic_fps = false;
		pr_debug("dfps update mode not configured: disable\n");
	}
	pinfo->new_fps = pinfo->mipi.frame_rate;

	return;
}

void mdss_dsi_unregister_bl_settings(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	if (ctrl_pdata->bklt_ctrl == BL_WLED)
		led_trigger_unregister_simple(bl_led_trigger);
}


/**
 * get_mdss_dsi_even_lane_clam_mask() - Computest DSI lane 0 & 2 clamps mask
 * @dlane_swap: dsi_lane_map_type
 * @lane_id: DSI lane (DSI_LANE_0)/(DSI_LANE_2)
 *
 * Return DSI lane 0 & 2 clamp mask based on lane swap configuration.
 * Clamp Bit for Physical lanes
 *      Lane Num        Bit     Mask
 *      Lane0           Bit 7   0x80
 *      Lane1           Bit 5   0x20
 *      Lane2           Bit 3   0x08
 *      Lane3           Bit 2   0x02
 */
u32 get_mdss_dsi_even_lane_clam_mask(char dlane_swap,
				     enum dsi_lane_ids lane_id)
{
	u32 lane0_mask = 0;
	u32 lane2_mask = 0;

	switch (dlane_swap) {
	case DSI_LANE_MAP_0123:
	case DSI_LANE_MAP_0321:
		lane0_mask = 0x80;
		lane2_mask = 0x08;
		break;
	case DSI_LANE_MAP_3012:
	case DSI_LANE_MAP_1032:
		lane0_mask = 0x20;
		lane2_mask = 0x02;
		break;
	case DSI_LANE_MAP_2301:
	case DSI_LANE_MAP_2103:
		lane0_mask = 0x08;
		lane2_mask = 0x80;
		break;
	case DSI_LANE_MAP_1230:
	case DSI_LANE_MAP_3210:
		lane0_mask = 0x02;
		lane2_mask = 0x20;
		break;
	default:
		lane0_mask = 0x00;
		lane2_mask = 0x00;
		break;
	}
	if (lane_id == DSI_LANE_0)
		return lane0_mask;
	else if (lane_id == DSI_LANE_2)
		return lane2_mask;
	else
		return 0;
}

/**
 * get_mdss_dsi_odd_lane_clam_mask() - Computest DSI lane 1 & 3 clamps mask
 * @dlane_swap: dsi_lane_map_type
 * @lane_id: DSI lane (DSI_LANE_1)/(DSI_LANE_3)
 *
 * Return DSI lane 1 & 3 clamp mask based on lane swap configuration.
 */
u32 get_mdss_dsi_odd_lane_clam_mask(char dlane_swap,
					enum dsi_lane_ids lane_id)
{
	u32 lane1_mask = 0;
	u32 lane3_mask = 0;

	switch (dlane_swap) {
	case DSI_LANE_MAP_0123:
	case DSI_LANE_MAP_2103:
		lane1_mask = 0x20;
		lane3_mask = 0x02;
		break;
	case DSI_LANE_MAP_3012:
	case DSI_LANE_MAP_3210:
		lane1_mask = 0x08;
		lane3_mask = 0x80;
		break;
	case DSI_LANE_MAP_2301:
	case DSI_LANE_MAP_0321:
		lane1_mask = 0x02;
		lane3_mask = 0x20;
		break;
	case DSI_LANE_MAP_1230:
	case DSI_LANE_MAP_1032:
		lane1_mask = 0x80;
		lane3_mask = 0x08;
		break;
	default:
		lane1_mask = 0x00;
		lane3_mask = 0x00;
		break;
	}
	if (lane_id == DSI_LANE_1)
		return lane1_mask;
	else if (lane_id == DSI_LANE_3)
		return lane3_mask;
	else
		return 0;
}
static void mdss_dsi_set_lane_clamp_mask(struct mipi_panel_info *mipi)
{
	u32 mask = 0;

	if (mipi->data_lane0)
		mask = get_mdss_dsi_even_lane_clam_mask(mipi->dlane_swap,
					DSI_LANE_0);
	if (mipi->data_lane1)
		mask |= get_mdss_dsi_odd_lane_clam_mask(mipi->dlane_swap,
					DSI_LANE_1);
	if (mipi->data_lane2)
		mask |= get_mdss_dsi_even_lane_clam_mask(mipi->dlane_swap,
					DSI_LANE_2);
	if (mipi->data_lane3)
		mask |= get_mdss_dsi_odd_lane_clam_mask(mipi->dlane_swap,
					DSI_LANE_3);

	mipi->phy_lane_clamp_mask = mask;
}


static int mdss_panel_parse_dt(struct device_node *np,
			struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp;
	int rc, i, len;
	const char *data;
	static const char *pdest;
	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	pr_debug("\n\n******************** [HL] %s +++ **********************\n\n", __func__);

	//SW4-HL-Display-AddCTCPanelHX8394DInsideSupport-00+{_20150317
	rc = of_property_read_u32(np, "fih,panel-id", &tmp);
	if (rc) {
		pr_err("%s:%d, panel id not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	else
	{
		pr_debug("\n\n******************** [HL] %s of_property_read_u32(np, \"fih,panel-id\", &tmp), tmp = %d **********************\n\n", __func__, tmp);
	}
	pinfo->pid = (!rc ? tmp : 0);
	//SW4-HL-Display-AddCTCPanelHX8394DInsideSupport-00+}_20150317

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-width", &tmp);
	if (rc) {
		pr_err("%s:%d, panel width not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->xres = (!rc ? tmp : 640);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-height", &tmp);
	if (rc) {
		pr_err("%s:%d, panel height not specified\n",
						__func__, __LINE__);
		return -EINVAL;
	}
	pinfo->yres = (!rc ? tmp : 480);

	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension", &tmp);
	pinfo->physical_width = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension", &tmp);
	pinfo->physical_height = (!rc ? tmp : 0);

	//SW4-HL-Display-BringUpNT35521-00+{_20150224
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-width-dimension-full", &tmp);
	pinfo->physical_width_full = (!rc ? tmp : 0);
	rc = of_property_read_u32(np,
		"qcom,mdss-pan-physical-height-dimension-full", &tmp);
	pinfo->physical_height_full = (!rc ? tmp : 0);
	//SW4-HL-Display-BringUpNT35521-00+}_20150224

	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-left-border", &tmp);
	pinfo->lcdc.xres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-right-border", &tmp);
	if (!rc)
		pinfo->lcdc.xres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-top-border", &tmp);
	pinfo->lcdc.yres_pad = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-bottom-border", &tmp);
	if (!rc)
		pinfo->lcdc.yres_pad += tmp;
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bpp", &tmp);
	if (rc) {
		pr_err("%s:%d, bpp not specified\n", __func__, __LINE__);
		return -EINVAL;
	}
	pinfo->bpp = (!rc ? tmp : 24);
	pinfo->mipi.mode = DSI_VIDEO_MODE;
	data = of_get_property(np, "qcom,mdss-dsi-panel-type", NULL);
	if (data && !strncmp(data, "dsi_cmd_mode", 12))
		pinfo->mipi.mode = DSI_CMD_MODE;
	tmp = 0;
	data = of_get_property(np, "qcom,mdss-dsi-pixel-packing", NULL);
	if (data && !strcmp(data, "loose"))
		pinfo->mipi.pixel_packing = 1;
	else
		pinfo->mipi.pixel_packing = 0;
	rc = mdss_panel_get_dst_fmt(pinfo->bpp,
		pinfo->mipi.mode, pinfo->mipi.pixel_packing,
		&(pinfo->mipi.dst_format));
	if (rc) {
		pr_debug("%s: problem determining dst format. Set Default\n",
			__func__);
		pinfo->mipi.dst_format =
			DSI_VIDEO_DST_FORMAT_RGB888;
	}
	pdest = of_get_property(np,
		"qcom,mdss-dsi-panel-destination", NULL);

	if (pdest) {
		if (strlen(pdest) != 9) {
			pr_err("%s: Unknown pdest specified\n", __func__);
			return -EINVAL;
		}
		if (!strcmp(pdest, "display_1"))
			pinfo->pdest = DISPLAY_1;
		else if (!strcmp(pdest, "display_2"))
			pinfo->pdest = DISPLAY_2;
		else {
			pr_debug("%s: incorrect pdest. Set Default\n",
				__func__);
			pinfo->pdest = DISPLAY_1;
		}
	} else {
		pr_debug("%s: pdest not specified. Set Default\n",
				__func__);
		pinfo->pdest = DISPLAY_1;
	}
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-front-porch", &tmp);
	pinfo->lcdc.h_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-back-porch", &tmp);
	pinfo->lcdc.h_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-pulse-width", &tmp);
	pinfo->lcdc.h_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-h-sync-skew", &tmp);
	pinfo->lcdc.hsync_skew = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-back-porch", &tmp);
	pinfo->lcdc.v_back_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-front-porch", &tmp);
	pinfo->lcdc.v_front_porch = (!rc ? tmp : 6);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-v-pulse-width", &tmp);
	pinfo->lcdc.v_pulse_width = (!rc ? tmp : 2);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-underflow-color", &tmp);
	pinfo->lcdc.underflow_clr = (!rc ? tmp : 0xff);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-border-color", &tmp);
	pinfo->lcdc.border_clr = (!rc ? tmp : 0);
	data = of_get_property(np, "qcom,mdss-dsi-panel-orientation", NULL);
	if (data) {
		pr_debug("panel orientation is %s\n", data);
		if (!strcmp(data, "180"))
			pinfo->panel_orientation = MDP_ROT_180;
		else if (!strcmp(data, "hflip"))
			pinfo->panel_orientation = MDP_FLIP_LR;
		else if (!strcmp(data, "vflip"))
			pinfo->panel_orientation = MDP_FLIP_UD;
	}

	ctrl_pdata->bklt_ctrl = UNKNOWN_CTRL;
	data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
	if (data) {
		if (!strncmp(data, "bl_ctrl_wled", 12)) {
			led_trigger_register_simple("bkl-trigger",
				&bl_led_trigger);
			pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
				__func__);
			ctrl_pdata->bklt_ctrl = BL_WLED;
		} else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
			ctrl_pdata->bklt_ctrl = BL_PWM;
			ctrl_pdata->pwm_pmi = of_property_read_bool(np,
					"qcom,mdss-dsi-bl-pwm-pmi");
			rc = of_property_read_u32(np,
				"qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
			if (rc) {
				pr_err("%s:%d, Error, panel pwm_period\n",
						__func__, __LINE__);
				return -EINVAL;
			}
			ctrl_pdata->pwm_period = tmp;
			if (ctrl_pdata->pwm_pmi) {
				ctrl_pdata->pwm_bl = of_pwm_get(np, NULL);
				if (IS_ERR(ctrl_pdata->pwm_bl)) {
					pr_err("%s: Error, pwm device\n",
								__func__);
					ctrl_pdata->pwm_bl = NULL;
					return -EINVAL;
				}
			} else {
				rc = of_property_read_u32(np,
					"qcom,mdss-dsi-bl-pmic-bank-select",
								 &tmp);
				if (rc) {
					pr_err("%s:%d, Error, lpg channel\n",
							__func__, __LINE__);
					return -EINVAL;
				}
				ctrl_pdata->pwm_lpg_chan = tmp;
				tmp = of_get_named_gpio(np,
					"qcom,mdss-dsi-pwm-gpio", 0);
				ctrl_pdata->pwm_pmic_gpio = tmp;
				pr_debug("%s: Configured PWM bklt ctrl\n",
								 __func__);
			}
		} else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
			ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
			pr_debug("%s: Configured DCS_CMD bklt ctrl\n",
								__func__);
		}
		//SW4-HL-Display-BringUpNT35521-00+{_20150224
		else if (!strncmp(data, "bl_ctrl_i2c", 11))
		{
			pr_debug("\n\n******************** [HL] %s, bl_ctrl_i2c  **********************\n\n", __func__);
			ctrl_pdata->bklt_ctrl = BL_I2C;
		}
		//SW4-HL-Display-BringUpNT35521-00+}_20150224
	}
	rc = of_property_read_u32(np, "qcom,mdss-brightness-max-level", &tmp);
	pinfo->brightness_max = (!rc ? tmp : MDSS_MAX_BL_BRIGHTNESS);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level", &tmp);
	pinfo->bl_min = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level", &tmp);
	pinfo->bl_max = (!rc ? tmp : 255);
	//SW4-HL-Display-FineTuneBLMappingTable-02+{_20150625
	if (strstr(saved_command_line, "bl-350nit") != NULL)
	{
		rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-min-level-350nit", &tmp);
		pinfo->bl_min = (!rc ? tmp : 0);
		rc = of_property_read_u32(np, "qcom,mdss-dsi-bl-max-level-350nit", &tmp);
		pinfo->bl_max = (!rc ? tmp : 255);
		g350nitPanel = true;	//SW4-HL-Display-FineTuneBLMappingTable-04+_20150730
	}
	//SW4-HL-Display-FineTuneBLMappingTable-02+}_20150625
	ctrl_pdata->bklt_max = pinfo->bl_max;

	rc = of_property_read_u32(np, "qcom,mdss-dsi-interleave-mode", &tmp);
	pinfo->mipi.interleave_mode = (!rc ? tmp : 0);

	pinfo->mipi.vsync_enable = of_property_read_bool(np,
		"qcom,mdss-dsi-te-check-enable");
	pinfo->mipi.hw_vsync_mode = of_property_read_bool(np,
		"qcom,mdss-dsi-te-using-te-pin");

	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-h-sync-pulse", &tmp);
	pinfo->mipi.pulse_mode_hsa_he = (!rc ? tmp : false);

	pinfo->mipi.hfp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hfp-power-mode");
	pinfo->mipi.hsa_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hsa-power-mode");
	pinfo->mipi.hbp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-hbp-power-mode");
	pinfo->mipi.last_line_interleave_en = of_property_read_bool(np,
		"qcom,mdss-dsi-last-line-interleave");
	pinfo->mipi.bllp_power_stop = of_property_read_bool(np,
		"qcom,mdss-dsi-bllp-power-mode");
	pinfo->mipi.eof_bllp_power_stop = of_property_read_bool(
		np, "qcom,mdss-dsi-bllp-eof-power-mode");
	pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_PULSE;
	data = of_get_property(np, "qcom,mdss-dsi-traffic-mode", NULL);
	if (data) {
		if (!strcmp(data, "non_burst_sync_event"))
			pinfo->mipi.traffic_mode = DSI_NON_BURST_SYNCH_EVENT;
		else if (!strcmp(data, "burst_mode"))
			pinfo->mipi.traffic_mode = DSI_BURST_MODE;
	}
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-dcs-command", &tmp);
	pinfo->mipi.insert_dcs_cmd =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-continue", &tmp);
	pinfo->mipi.wr_mem_continue =
			(!rc ? tmp : 0x3c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-wr-mem-start", &tmp);
	pinfo->mipi.wr_mem_start =
			(!rc ? tmp : 0x2c);
	rc = of_property_read_u32(np,
		"qcom,mdss-dsi-te-pin-select", &tmp);
	pinfo->mipi.te_sel =
			(!rc ? tmp : 1);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-virtual-channel-id", &tmp);
	pinfo->mipi.vc = (!rc ? tmp : 0);
	pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RGB;
	data = of_get_property(np, "qcom,mdss-dsi-color-order", NULL);
	if (data) {
		if (!strcmp(data, "rgb_swap_rbg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_RBG;
		else if (!strcmp(data, "rgb_swap_bgr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BGR;
		else if (!strcmp(data, "rgb_swap_brg"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_BRG;
		else if (!strcmp(data, "rgb_swap_grb"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GRB;
		else if (!strcmp(data, "rgb_swap_gbr"))
			pinfo->mipi.rgb_swap = DSI_RGB_SWAP_GBR;
	}
	pinfo->mipi.data_lane0 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-0-state");
	pinfo->mipi.data_lane1 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-1-state");
	pinfo->mipi.data_lane2 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-2-state");
	pinfo->mipi.data_lane3 = of_property_read_bool(np,
		"qcom,mdss-dsi-lane-3-state");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-pre", &tmp);
	pinfo->mipi.t_clk_pre = (!rc ? tmp : 0x24);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-t-clk-post", &tmp);
	pinfo->mipi.t_clk_post = (!rc ? tmp : 0x03);

	pinfo->mipi.rx_eot_ignore = of_property_read_bool(np,
		"qcom,mdss-dsi-rx-eot-ignore");
	pinfo->mipi.tx_eot_append = of_property_read_bool(np,
		"qcom,mdss-dsi-tx-eot-append");

	rc = of_property_read_u32(np, "qcom,mdss-dsi-stream", &tmp);
	pinfo->mipi.stream = (!rc ? tmp : 0);

	data = of_get_property(np, "qcom,mdss-dsi-panel-mode-gpio-state", NULL);
	if (data) {
		if (!strcmp(data, "high"))
			pinfo->mode_gpio_state = MODE_GPIO_HIGH;
		else if (!strcmp(data, "low"))
			pinfo->mode_gpio_state = MODE_GPIO_LOW;
	} else {
		pinfo->mode_gpio_state = MODE_GPIO_NOT_VALID;
	}

	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-framerate", &tmp);
	pinfo->mipi.frame_rate = (!rc ? tmp : 60);
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-clockrate", &tmp);
	pinfo->clk_rate = (!rc ? tmp : 0);
	rc = of_property_read_u32(np, "qcom,mdss-mdp-transfer-time-us", &tmp);
	pinfo->mdp_transfer_time_us = (!rc ? tmp : DEFAULT_MDP_TRANSFER_TIME);
	data = of_get_property(np, "qcom,mdss-dsi-panel-timings", &len);
	if ((!data) || (len != 12)) {
		pr_err("%s:%d, Unable to read Phy timing settings",
		       __func__, __LINE__);
		goto error;
	}
	for (i = 0; i < len; i++)
		pinfo->mipi.dsi_phy_db.timing[i] = data[i];

	pinfo->mipi.lp11_init = of_property_read_bool(np,
					"qcom,mdss-dsi-lp11-init");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-init-delay-us", &tmp);
	pinfo->mipi.init_delay = (!rc ? tmp : 0);

	rc = of_property_read_u32(np, "qcom,mdss-dsi-post-init-delay", &tmp);
	pinfo->mipi.post_init_delay = (!rc ? tmp : 0);

	mdss_dsi_parse_roi_alignment(np, pinfo);

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.mdp_trigger),
		"qcom,mdss-dsi-mdp-trigger");

	mdss_dsi_parse_trigger(np, &(pinfo->mipi.dma_trigger),
		"qcom,mdss-dsi-dma-trigger");

	mdss_dsi_parse_lane_swap(np, &(pinfo->mipi.dlane_swap));

	mdss_dsi_parse_fbc_params(np, pinfo);

	mdss_panel_parse_te_params(np, pinfo);

	mdss_dsi_parse_reset_seq(np, pinfo->rst_seq, &(pinfo->rst_seq_len),
		"qcom,mdss-dsi-reset-sequence");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->on_cmds,
		"qcom,mdss-dsi-on-command", "qcom,mdss-dsi-on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->post_panel_on_cmds,
		"qcom,mdss-dsi-post-panel-on-command", NULL);

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->off_cmds,
		"qcom,mdss-dsi-off-command", "qcom,mdss-dsi-off-command-state");

#if 0 //msm8909
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->status_cmds,
			"qcom,mdss-dsi-panel-status-command",
				"qcom,mdss-dsi-panel-status-command-state");
	rc = of_property_read_u32(np, "qcom,mdss-dsi-panel-status-value", &tmp);
	ctrl_pdata->status_value = (!rc ? tmp : 0);


	ctrl_pdata->status_mode = ESD_MAX;
	rc = of_property_read_string(np,
				"qcom,mdss-dsi-panel-status-check-mode", &data);
	if (!rc) {
		if (!strcmp(data, "bta_check")) {
			ctrl_pdata->status_mode = ESD_BTA;
		} else if (!strcmp(data, "reg_read")) {
			ctrl_pdata->status_mode = ESD_REG;
			ctrl_pdata->status_cmds_rlen = 1;
			ctrl_pdata->check_read_status =
						mdss_dsi_gen_read_status;
		} else if (!strcmp(data, "reg_read_nt35596")) {
			ctrl_pdata->status_mode = ESD_REG_NT35596;
			ctrl_pdata->status_error_count = 0;
			ctrl_pdata->status_cmds_rlen = 8;
			ctrl_pdata->check_read_status =
						mdss_dsi_nt35596_read_status;
		} else if (!strcmp(data, "te_signal_check")) {
			if (pinfo->mipi.mode == DSI_CMD_MODE)
				ctrl_pdata->status_mode = ESD_TE;
			else
				pr_err("TE-ESD not valid for video mode\n");
		}
	}
#endif

	pinfo->mipi.force_clk_lane_hs = of_property_read_bool(np,
		"qcom,mdss-dsi-force-clock-lane-hs");

	pinfo->mipi.always_on = of_property_read_bool(np,
		"qcom,mdss-dsi-always-on");

	rc = mdss_dsi_parse_panel_features(np, ctrl_pdata);
	if (rc) {
		pr_err("%s: failed to parse panel features\n", __func__);
		goto error;
	}

	mdss_dsi_parse_panel_horizintal_line_idle(np, ctrl_pdata);

	mdss_dsi_parse_dfps_config(np, ctrl_pdata);

	//SW4-HL-Display-BringUpNT35521-00+{_20150224
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->ce_on_cmds,
		"qcom,mdss-dsi-ce-on-command", "qcom,mdss-dsi-ce-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->ce_off_cmds,
		"qcom,mdss-dsi-ce-off-command", "qcom,mdss-dsi-ce-off-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->ct_normal_cmds,
		"qcom,mdss-dsi-ct-normal-command", "qcom,mdss-dsi-ct-normal-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->ct_warm_cmds,
		"qcom,mdss-dsi-ct-warm-command", "qcom,mdss-dsi-ct-warm-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->ct_cold_cmds,
		"qcom,mdss-dsi-ct-cold-command", "qcom,mdss-dsi-ct-cold-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->blf_10_cmds,
		"qcom,mdss-dsi-blf-10-command", "qcom,mdss-dsi-blf-10-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->blf_30_cmds,
		"qcom,mdss-dsi-blf-30-command", "qcom,mdss-dsi-blf-30-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->blf_50_cmds,
		"qcom,mdss-dsi-blf-50-command", "qcom,mdss-dsi-blf-50-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->blf_75_cmds,
		"qcom,mdss-dsi-blf-75-command", "qcom,mdss-dsi-blf-75-command-state");

/* E1M-4489 - Add SVI(AIE) setting */
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->svi_on_cmds,
		"fih,mdss-dsi-svi-on-command", "fih,mdss-dsi-svi-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->svi_off_cmds,
		"fih,mdss-dsi-svi-off-command", "fih,mdss-dsi-svi-off-command-state");
/* end E1M-4489 */
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_off_cmds,
		"qcom,mdss-dsi-cabc-off-command", "qcom,mdss-dsi-cabc-off-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_ui_cmds,
		"qcom,mdss-dsi-cabc-ui-command", "qcom,mdss-dsi-cabc-ui-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_still_cmds,
		"qcom,mdss-dsi-cabc-still-command", "qcom,mdss-dsi-cabc-still-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->cabc_moving_cmds,
		"qcom,mdss-dsi-cabc-moving-command", "qcom,mdss-dsi-cabc-moving-command-state");
	//SW4-HL-Display-BringUpNT35521-00+}_20150224

	//SW4-HL-Display-EnablePWMOutput-00*{_20150611
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->sleep_out_cmds,
		"fih,mdss-dsi-sleep-out-command", "fih,mdss-dsi-sleep-out-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->display_on_cmds,
		"fih,mdss-dsi-display-out-command", "fih,mdss-dsi-display-out-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->pre_ce_on_cmds,
		"fih,mdss-dsi-pre-ce-on-command", "fih,mdss-dsi-pre-ce-on-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->pre_ce_off_cmds,
		"fih,mdss-dsi-pre-ce-off-command", "fih,mdss-dsi-pre-ce-off-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->pre_cabc_off_cmds,
		"fih,mdss-dsi-cabc-off-command", "fih,mdss-dsi-pre-cabc-off-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->pre_cabc_ui_cmds,
		"fih,mdss-dsi-cabc-ui-command", "fih,mdss-dsi-pre-cabc-ui-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->pre_cabc_still_cmds,
		"fih,mdss-dsi-cabc-still-command", "fih,mdss-dsi-pre-cabc-still-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->pre_cabc_moving_cmds,
		"fih,mdss-dsi-cabc-moving-command", "fih,mdss-dsi-pre-cabc-moving-command-state");


	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->pwm_output_enable_cmds,
		"fih,lcm-pwm-output-enable-command", "fih,lcm-pwm-output-enable-command-state");
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->pwm_output_disable_cmds,
		"fih,lcm-pwm-output-disable-command", "fih,lcm-pwm-output-disable-command-state");
	//SW4-HL-Display-EnablePWMOutput-00*}_20150611

	//SW4-HL-Display-EnableDisplayCheckMechanism-00+{_20150714
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->open_setting_cmds,
		"fih,lcm-open-setting-command", "fih,lcm-open-setting-command-state");
	//SW4-HL-Display-EnableDisplayCheckMechanism-00+}_20150714

/* E1M-576 - Add LCM mipi reg read/write command */
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->write_reg_cmds,
		"fih,mdss-dsi-write-reg-command", "fih,mdss-dsi-write-reg-command-state");
/* end E1M-576 */
	rc = of_property_read_u32(np, "fih,default-cabc-mode", &tmp);
	cabc_set = (!rc ? tmp : 0);

	pr_debug("\n\n******************** [HL] %s ---, OK return 0 **********************\n\n", __func__);

	return 0;

error:
	pr_debug("\n\n******************** [HL] %s ---, return -EINVAL **********************\n\n", __func__);

	return -EINVAL;
}

int mdss_dsi_panel_init(struct device_node *node,
	struct mdss_dsi_ctrl_pdata *ctrl_pdata,
	bool cmd_cfg_cont_splash)
{
	int rc = 0;
	static const char *panel_name;
	struct mdss_panel_info *pinfo;

	if (!node || !ctrl_pdata) {
		pr_err("%s: Invalid arguments\n", __func__);
		return -ENODEV;
	}

	pinfo = &ctrl_pdata->panel_data.panel_info;

	pr_debug("%s:%d\n", __func__, __LINE__);
	pinfo->panel_name[0] = '\0';
	panel_name = of_get_property(node, "qcom,mdss-dsi-panel-name", NULL);
	if (!panel_name) {
		pr_info("%s:%d, Panel name not specified\n",
						__func__, __LINE__);
	} else {
		pr_info("%s: Panel Name = %s\n", __func__, panel_name);
		strlcpy(&pinfo->panel_name[0], panel_name, MDSS_MAX_PANEL_LEN);
	}
	rc = mdss_panel_parse_dt(node, ctrl_pdata);
	if (rc) {
		pr_err("%s:%d panel dt parse failed\n", __func__, __LINE__);
		return rc;
	}

	mdss_dsi_set_lane_clamp_mask(&pinfo->mipi);
	if (!cmd_cfg_cont_splash)
		pinfo->cont_splash_enabled = false;
	pr_info("%s: Continuous splash %s\n", __func__,
		pinfo->cont_splash_enabled ? "enabled" : "disabled");

	pinfo->dynamic_switch_pending = false;
	pinfo->is_lpm_mode = false;
	pinfo->esd_rdy = false;

	ctrl_pdata->on = mdss_dsi_panel_on;
	ctrl_pdata->post_panel_on = mdss_dsi_post_panel_on;
	ctrl_pdata->off = mdss_dsi_panel_off;
	ctrl_pdata->low_power_config = mdss_dsi_panel_low_power_config;
	ctrl_pdata->panel_data.set_backlight = mdss_dsi_panel_bl_ctrl;
	ctrl_pdata->switch_mode = mdss_dsi_panel_switch_mode;
	ctrl_pdata->cmds_send = mdss_dsi_panel_cmds_send;	//SW4-HL-Display-PowerPinControlPinAndInitCodeAPI-00+_20150519
	ctrl_pdata->send_display_on_cmd = mdss_dsi_send_display_on_cmd;	//SW4-HL-Display-FixLCMCanNotBringUpSinceAliveCheckMethodIsNotSetGoodEnough-00+_20151218
	return 0;
}
