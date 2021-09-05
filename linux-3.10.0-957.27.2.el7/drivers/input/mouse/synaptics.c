/*
 * Synaptics TouchPad PS/2 mouse driver
 *
 *   2003 Dmitry Torokhov <dtor@mail.ru>
 *     Added support for pass-through port. Special thanks to Peter Berg Larsen
 *     for explaining various Synaptics quirks.
 *
 *   2003 Peter Osterlund <petero2@telia.com>
 *     Ported to 2.5 input device infrastructure.
 *
 *   Copyright (C) 2001 Stefan Gmeiner <riddlebox@freesurf.ch>
 *     start merging tpconfig and gpm code to a xfree-input module
 *     adding some changes and extensions (ex. 3rd and 4th button)
 *
 *   Copyright (c) 1997 C. Scott Ananian <cananian@alumni.priceton.edu>
 *   Copyright (c) 1998-2000 Bruce Kalk <kall@compass.com>
 *     code for the special synaptics commands (from the tpconfig-source)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/dmi.h>
#include <linux/input/mt.h>
#include <linux/serio.h>
#include <linux/libps2.h>
#include <linux/rmi.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include "psmouse.h"
#include "synaptics.h"

/*
 * The x/y limits are taken from the Synaptics TouchPad interfacing Guide,
 * section 2.3.2, which says that they should be valid regardless of the
 * actual size of the sensor.
 * Note that newer firmware allows querying device for maximum useable
 * coordinates.
 */
#define XMIN 0
#define XMAX 6143
#define YMIN 0
#define YMAX 6143
#define XMIN_NOMINAL 1472
#define XMAX_NOMINAL 5472
#define YMIN_NOMINAL 1408
#define YMAX_NOMINAL 4448

/* Size in bits of absolute position values reported by the hardware */
#define ABS_POS_BITS 13

/*
 * These values should represent the absolute maximum value that will
 * be reported for a positive position value. Some Synaptics firmware
 * uses this value to indicate a finger near the edge of the touchpad
 * whose precise position cannot be determined.
 *
 * At least one touchpad is known to report positions in excess of this
 * value which are actually negative values truncated to the 13-bit
 * reporting range. These values have never been observed to be lower
 * than 8184 (i.e. -8), so we treat all values greater than 8176 as
 * negative and any other value as positive.
 */
#define X_MAX_POSITIVE 8176
#define Y_MAX_POSITIVE 8176

/*****************************************************************************
 *	Stuff we need even when we do not want native Synaptics support
 ****************************************************************************/

/*
 * Set the synaptics touchpad mode byte by special commands
 */
static int synaptics_mode_cmd(struct psmouse *psmouse, unsigned char mode)
{
	unsigned char param[1];

	if (ps2_sliced_command(&psmouse->ps2dev, mode))
		return -1;
	param[0] = SYN_PS_SET_MODE2;
	if (ps2_command(&psmouse->ps2dev, param, PSMOUSE_CMD_SETRATE))
		return -1;
	return 0;
}

int synaptics_detect(struct psmouse *psmouse, bool set_properties)
{
	struct ps2dev *ps2dev = &psmouse->ps2dev;
	unsigned char param[4];

	param[0] = 0;

	ps2_command(ps2dev, param, PSMOUSE_CMD_SETRES);
	ps2_command(ps2dev, param, PSMOUSE_CMD_SETRES);
	ps2_command(ps2dev, param, PSMOUSE_CMD_SETRES);
	ps2_command(ps2dev, param, PSMOUSE_CMD_SETRES);
	ps2_command(ps2dev, param, PSMOUSE_CMD_GETINFO);

	if (param[1] != 0x47)
		return -ENODEV;

	if (set_properties) {
		psmouse->vendor = "Synaptics";
		psmouse->name = "TouchPad";
	}

	return 0;
}

void synaptics_reset(struct psmouse *psmouse)
{
	/* reset touchpad back to relative mode, gestures enabled */
	synaptics_mode_cmd(psmouse, 0);
}

#if defined(CONFIG_MOUSE_PS2_SYNAPTICS) || \
    defined(CONFIG_MOUSE_PS2_SYNAPTICS_SMBUS)

/* This list has been kindly provided by Synaptics. */
static const char * const topbuttonpad_pnp_ids[] = {
	"LEN0017",
	"LEN0018",
	"LEN0019",
	"LEN0023",
	"LEN002A",
	"LEN002B",
	"LEN002C",
	"LEN002D",
	"LEN002E",
	"LEN0033", /* Helix */
	"LEN0034", /* T431s, L440, L540, T540, W540, X1 Carbon 2nd */
	"LEN0035", /* X240 */
	"LEN0036", /* T440 */
	"LEN0037", /* X1 Carbon 2nd */
	"LEN0038",
	"LEN0039", /* T440s */
	"LEN0041",
	"LEN0042", /* Yoga */
	"LEN0045",
	"LEN0047",
	"LEN0049",
	"LEN2000",
	"LEN2001", /* Edge E431 */
	"LEN2002", /* Edge E531 */
	"LEN2003",
	"LEN2004", /* L440 */
	"LEN2005",
	"LEN2006",
	"LEN2007",
	"LEN2008",
	"LEN2009",
	"LEN200A",
	"LEN200B",
	NULL
};

static const char * const smbus_pnp_ids[] = {
	/* all of the topbuttonpad_pnp_ids are valid, we just add some extras */
	"LEN0048", /* X1 Carbon 3 */
	"LEN0046", /* X250 */
	"LEN004a", /* W541 */
	"LEN200f", /* T450s */
	"LEN0071", /* T480 */
	"LEN0092", /* X1 Carbon 6th gen */
	"LEN0097", /* X280 -> ALPS trackpoint */
	NULL
};

/*
 * Send a command to the synpatics touchpad by special commands
 */
static int synaptics_send_cmd(struct psmouse *psmouse,
			      unsigned char c, unsigned char *param)
{
	int error;

	error = ps2_sliced_command(&psmouse->ps2dev, c);
	if (error)
		return error;

	error = ps2_command(&psmouse->ps2dev, param, PSMOUSE_CMD_GETINFO);
	if (error)
		return error;

	return 0;
}

static int synaptics_query_int(struct psmouse *psmouse, u8 query_cmd, u32 *val)
{
	int error;
	union {
		__be32 be_val;
		char buf[4];
	} resp = { 0 };

	error = synaptics_send_cmd(psmouse, query_cmd, resp.buf + 1);
	if (error)
		return error;

	*val = be32_to_cpu(resp.be_val);
	return 0;
}

/*
 * Identify Touchpad
 * See also the SYN_ID_* macros
 */
static int synaptics_identify(struct psmouse *psmouse,
			      struct synaptics_device_info *info)
{
	int error;

	error = synaptics_query_int(psmouse, SYN_QUE_IDENTIFY, &info->identity);
	if (error)
		return error;

	return SYN_ID_IS_SYNAPTICS(info->identity) ? 0 : -ENXIO;
}

/*
 * Read the model-id bytes from the touchpad
 * see also SYN_MODEL_* macros
 */
static int synaptics_model_id(struct psmouse *psmouse,
			      struct synaptics_device_info *info)
{
	return synaptics_query_int(psmouse, SYN_QUE_MODEL, &info->model_id);
}

/*
 * Read the firmware id from the touchpad
 */
static int synaptics_firmware_id(struct psmouse *psmouse,
				 struct synaptics_device_info *info)
{
	return synaptics_query_int(psmouse, SYN_QUE_FIRMWARE_ID,
				   &info->firmware_id);
}

/*
 * Read the board id and the "More Extended Queries" from the touchpad
 * The board id is encoded in the "QUERY MODES" response
 */
static int synaptics_query_modes(struct psmouse *psmouse,
				 struct synaptics_device_info *info)
{
	unsigned char bid[3];
	int error;

	/* firmwares prior 7.5 have no board_id encoded */
	if (SYN_ID_FULL(info->identity) < 0x705)
		return 0;

	error = synaptics_send_cmd(psmouse, SYN_QUE_MODES, bid);
	if (error)
		return error;

	info->board_id = ((bid[0] & 0xfc) << 6) | bid[1];

	if (SYN_MEXT_CAP_BIT(bid[0]))
		return synaptics_query_int(psmouse, SYN_QUE_MEXT_CAPAB_10,
					   &info->ext_cap_10);

	return 0;
}

/*
 * Read the capability-bits from the touchpad
 * see also the SYN_CAP_* macros
 */
static int synaptics_capability(struct psmouse *psmouse,
				struct synaptics_device_info *info)
{
	int error;

	error = synaptics_query_int(psmouse, SYN_QUE_CAPABILITIES,
				    &info->capabilities);
	if (error)
		return error;

	info->ext_cap = info->ext_cap_0c = 0;

	/*
	 * Older firmwares had submodel ID fixed to 0x47
	 */
	if (SYN_ID_FULL(info->identity) < 0x705 &&
	    SYN_CAP_SUBMODEL_ID(info->capabilities) != 0x47) {
		return -ENXIO;
	}

	/*
	 * Unless capExtended is set the rest of the flags should be ignored
	 */
	if (!SYN_CAP_EXTENDED(info->capabilities))
		info->capabilities = 0;

	if (SYN_EXT_CAP_REQUESTS(info->capabilities) >= 1) {
		error = synaptics_query_int(psmouse, SYN_QUE_EXT_CAPAB,
					    &info->ext_cap);
		if (error) {
			psmouse_warn(psmouse,
				     "device claims to have extended capabilities, but I'm not able to read them.\n");
		} else {
			/*
			 * if nExtBtn is greater than 8 it should be considered
			 * invalid and treated as 0
			 */
			if (SYN_CAP_MULTI_BUTTON_NO(info->ext_cap) > 8)
				info->ext_cap &= ~SYN_CAP_MB_MASK;
		}
	}

	if (SYN_EXT_CAP_REQUESTS(info->capabilities) >= 4) {
		error = synaptics_query_int(psmouse, SYN_QUE_EXT_CAPAB_0C,
					    &info->ext_cap_0c);
		if (error)
			psmouse_warn(psmouse,
				     "device claims to have extended capability 0x0c, but I'm not able to read it.\n");
	}

	return 0;
}

/*
 * Read touchpad resolution and maximum reported coordinates
 * Resolution is left zero if touchpad does not support the query
 */
static int synaptics_resolution(struct psmouse *psmouse,
				struct synaptics_device_info *info)
{
	unsigned char resp[3];
	int error;

	if (SYN_ID_MAJOR(info->identity) < 4)
		return 0;

	error = synaptics_send_cmd(psmouse, SYN_QUE_RESOLUTION, resp);
	if (!error) {
		if (resp[0] != 0 && (resp[1] & 0x80) && resp[2] != 0) {
			info->x_res = resp[0]; /* x resolution in units/mm */
			info->y_res = resp[2]; /* y resolution in units/mm */
		}
	}

	if (SYN_EXT_CAP_REQUESTS(info->capabilities) >= 5 &&
	    SYN_CAP_MAX_DIMENSIONS(info->ext_cap_0c)) {
		error = synaptics_send_cmd(psmouse,
					   SYN_QUE_EXT_MAX_COORDS, resp);
		if (error) {
			psmouse_warn(psmouse,
				     "device claims to have max coordinates query, but I'm not able to read it.\n");
		} else {
			info->x_max = (resp[0] << 5) | ((resp[1] & 0x0f) << 1);
			info->y_max = (resp[2] << 5) | ((resp[1] & 0xf0) >> 3);
			psmouse_info(psmouse,
				     "queried max coordinates: x [..%d], y [..%d]\n",
				     info->x_max, info->y_max);
		}
	}

	if (SYN_CAP_MIN_DIMENSIONS(info->ext_cap_0c) &&
	    (SYN_EXT_CAP_REQUESTS(info->capabilities) >= 7 ||
	     /*
	      * Firmware v8.1 does not report proper number of extended
	      * capabilities, but has been proven to report correct min
	      * coordinates.
	      */
	     SYN_ID_FULL(info->identity) == 0x801)) {
		error = synaptics_send_cmd(psmouse,
					   SYN_QUE_EXT_MIN_COORDS, resp);
		if (error) {
			psmouse_warn(psmouse,
				     "device claims to have min coordinates query, but I'm not able to read it.\n");
		} else {
			info->x_min = (resp[0] << 5) | ((resp[1] & 0x0f) << 1);
			info->y_min = (resp[2] << 5) | ((resp[1] & 0xf0) >> 3);
			psmouse_info(psmouse,
				     "queried min coordinates: x [%d..], y [%d..]\n",
				     info->x_min, info->y_min);
		}
	}

	return 0;
}

static int synaptics_query_hardware(struct psmouse *psmouse,
				    struct synaptics_device_info *info)
{
	int error;

	memset(info, 0, sizeof(*info));

	error = synaptics_identify(psmouse, info);
	if (error)
		return error;

	error = synaptics_model_id(psmouse, info);
	if (error)
		return error;

	error = synaptics_firmware_id(psmouse, info);
	if (error)
		return error;

	error = synaptics_query_modes(psmouse, info);
	if (error)
		return error;

	error = synaptics_capability(psmouse, info);
	if (error)
		return error;

	error = synaptics_resolution(psmouse, info);
	if (error)
		return error;

	return 0;
}

#endif /* CONFIG_MOUSE_PS2_SYNAPTICS || CONFIG_MOUSE_PS2_SYNAPTICS_SMBUS */

#ifdef CONFIG_MOUSE_PS2_SYNAPTICS

#define ANY_BOARD_ID 0
struct min_max_quirk {
	const char * const *pnp_ids;
	struct {
		u32 min, max;
	} board_id;
	u32 x_min, x_max, y_min, y_max;
};

static const struct min_max_quirk min_max_pnpid_table[] = {
	{
		(const char * const []){"LEN0033", NULL},
		{ANY_BOARD_ID, ANY_BOARD_ID},
		1024, 5052, 2258, 4832
	},
	{
		(const char * const []){"LEN0042", NULL},
		{ANY_BOARD_ID, ANY_BOARD_ID},
		1232, 5710, 1156, 4696
	},
	{
		(const char * const []){"LEN0034", "LEN0036", "LEN0037",
					"LEN0039", "LEN2002", "LEN2004",
					NULL},
		{ANY_BOARD_ID, 2961},
		1024, 5112, 2024, 4832
	},
	{
		(const char * const []){"LEN2001", NULL},
		{ANY_BOARD_ID, ANY_BOARD_ID},
		1024, 5022, 2508, 4832
	},
	{
		(const char * const []){"LEN2006", NULL},
		{ANY_BOARD_ID, ANY_BOARD_ID},
		1264, 5675, 1171, 4688
	},
	{ }
};

/*****************************************************************************
 *	Synaptics communications functions
 ****************************************************************************/

/*
 * Synaptics touchpads report the y coordinate from bottom to top, which is
 * opposite from what userspace expects.
 * This function is used to invert y before reporting.
 */
static int synaptics_invert_y(int y)
{
	return YMAX_NOMINAL + YMIN_NOMINAL - y;
}

/*
 * Apply quirk(s) if the hardware matches
 */
static void synaptics_apply_quirks(struct psmouse *psmouse,
				   struct synaptics_device_info *info)
{
	int i;

	for (i = 0; min_max_pnpid_table[i].pnp_ids; i++) {
		if (!psmouse_matches_pnp_id(psmouse,
					    min_max_pnpid_table[i].pnp_ids))
			continue;

		if (min_max_pnpid_table[i].board_id.min != ANY_BOARD_ID &&
		    info->board_id < min_max_pnpid_table[i].board_id.min)
			continue;

		if (min_max_pnpid_table[i].board_id.max != ANY_BOARD_ID &&
		    info->board_id > min_max_pnpid_table[i].board_id.max)
			continue;

		info->x_min = min_max_pnpid_table[i].x_min;
		info->x_max = min_max_pnpid_table[i].x_max;
		info->y_min = min_max_pnpid_table[i].y_min;
		info->y_max = min_max_pnpid_table[i].y_max;
		psmouse_info(psmouse,
			     "quirked min/max coordinates: x [%d..%d], y [%d..%d]\n",
			     info->x_min, info->x_max,
			     info->y_min, info->y_max);
		break;
	}
}

static bool synaptics_has_agm(struct synaptics_data *priv)
{
	return (SYN_CAP_ADV_GESTURE(priv->info.ext_cap_0c) ||
		SYN_CAP_IMAGE_SENSOR(priv->info.ext_cap_0c));
}

static int synaptics_set_advanced_gesture_mode(struct psmouse *psmouse)
{
	static unsigned char param = 0xc8;
	int error;

	error = ps2_sliced_command(&psmouse->ps2dev, SYN_QUE_MODEL);
	if (error)
		return error;

	if (ps2_command(&psmouse->ps2dev, &param, PSMOUSE_CMD_SETRATE))
		return -1;

	return 0;
}

static int synaptics_set_mode(struct psmouse *psmouse)
{
	struct synaptics_data *priv = psmouse->private;
	int error;

	priv->mode = 0;
	if (priv->absolute_mode)
		priv->mode |= SYN_BIT_ABSOLUTE_MODE;
	if (priv->disable_gesture)
		priv->mode |= SYN_BIT_DISABLE_GESTURE;
	if (psmouse->rate >= 80)
		priv->mode |= SYN_BIT_HIGH_RATE;
	if (SYN_CAP_EXTENDED(priv->info.capabilities))
		priv->mode |= SYN_BIT_W_MODE;

	if (synaptics_mode_cmd(psmouse, priv->mode))
		return -1;

	if (priv->absolute_mode && synaptics_has_agm(priv)) {
		error = synaptics_set_advanced_gesture_mode(psmouse);
		if (error) {
			psmouse_err(psmouse,
				    "Advanced gesture mode init failed: %d\n",
				    error);
			return error;
		}
	}

	return 0;
}

static void synaptics_set_rate(struct psmouse *psmouse, unsigned int rate)
{
	struct synaptics_data *priv = psmouse->private;

	if (rate >= 80) {
		priv->mode |= SYN_BIT_HIGH_RATE;
		psmouse->rate = 80;
	} else {
		priv->mode &= ~SYN_BIT_HIGH_RATE;
		psmouse->rate = 40;
	}

	synaptics_mode_cmd(psmouse, priv->mode);
}

/*****************************************************************************
 *	Synaptics pass-through PS/2 port support
 ****************************************************************************/
static int synaptics_pt_write(struct serio *serio, unsigned char c)
{
	struct psmouse *parent = serio_get_drvdata(serio->parent);
	char rate_param = SYN_PS_CLIENT_CMD; /* indicates that we want pass-through port */

	if (ps2_sliced_command(&parent->ps2dev, c))
		return -1;
	if (ps2_command(&parent->ps2dev, &rate_param, PSMOUSE_CMD_SETRATE))
		return -1;
	return 0;
}

static int synaptics_pt_start(struct serio *serio)
{
	struct psmouse *parent = serio_get_drvdata(serio->parent);
	struct synaptics_data *priv = parent->private;

	serio_pause_rx(parent->ps2dev.serio);
	priv->pt_port = serio;
	serio_continue_rx(parent->ps2dev.serio);

	return 0;
}

static void synaptics_pt_stop(struct serio *serio)
{
	struct psmouse *parent = serio_get_drvdata(serio->parent);
	struct synaptics_data *priv = parent->private;

	serio_pause_rx(parent->ps2dev.serio);
	priv->pt_port = NULL;
	serio_continue_rx(parent->ps2dev.serio);
}

static int synaptics_is_pt_packet(unsigned char *buf)
{
	return (buf[0] & 0xFC) == 0x84 && (buf[3] & 0xCC) == 0xC4;
}

static void synaptics_pass_pt_packet(struct serio *ptport,
				     unsigned char *packet)
{
	struct psmouse *child = serio_get_drvdata(ptport);

	if (child && child->state == PSMOUSE_ACTIVATED) {
		serio_interrupt(ptport, packet[1], 0);
		serio_interrupt(ptport, packet[4], 0);
		serio_interrupt(ptport, packet[5], 0);
		if (child->pktsize == 4)
			serio_interrupt(ptport, packet[2], 0);
	} else {
		serio_interrupt(ptport, packet[1], 0);
	}
}

static void synaptics_pt_activate(struct psmouse *psmouse)
{
	struct synaptics_data *priv = psmouse->private;
	struct psmouse *child = serio_get_drvdata(priv->pt_port);

	/* adjust the touchpad to child's choice of protocol */
	if (child) {
		if (child->pktsize == 4)
			priv->mode |= SYN_BIT_FOUR_BYTE_CLIENT;
		else
			priv->mode &= ~SYN_BIT_FOUR_BYTE_CLIENT;

		if (synaptics_mode_cmd(psmouse, priv->mode))
			psmouse_warn(psmouse,
				     "failed to switch guest protocol\n");
	}
}

static void synaptics_pt_create(struct psmouse *psmouse)
{
	struct serio *serio;

	serio = kzalloc(sizeof(struct serio), GFP_KERNEL);
	if (!serio) {
		psmouse_err(psmouse,
			    "not enough memory for pass-through port\n");
		return;
	}

	serio->id.type = SERIO_PS_PSTHRU;
	strlcpy(serio->name, "Synaptics pass-through", sizeof(serio->name));
	strlcpy(serio->phys, "synaptics-pt/serio0", sizeof(serio->name));
	serio->write = synaptics_pt_write;
	serio->start = synaptics_pt_start;
	serio->stop = synaptics_pt_stop;
	serio->parent = psmouse->ps2dev.serio;

	psmouse->pt_activate = synaptics_pt_activate;

	psmouse_info(psmouse, "serio: %s port at %s\n",
		     serio->name, psmouse->phys);
	serio_register_port(serio);
}

/*****************************************************************************
 *	Functions to interpret the absolute mode packets
 ****************************************************************************/

static void synaptics_mt_state_set(struct synaptics_mt_state *state, int count,
				   int sgm, int agm)
{
	state->count = count;
	state->sgm = sgm;
	state->agm = agm;
}

static void synaptics_parse_agm(const unsigned char buf[],
				struct synaptics_data *priv,
				struct synaptics_hw_state *hw)
{
	struct synaptics_hw_state *agm = &priv->agm;
	int agm_packet_type;

	agm_packet_type = (buf[5] & 0x30) >> 4;
	switch (agm_packet_type) {
	case 1:
		/* Gesture packet: (x, y, z) half resolution */
		agm->w = hw->w;
		agm->x = (((buf[4] & 0x0f) << 8) | buf[1]) << 1;
		agm->y = (((buf[4] & 0xf0) << 4) | buf[2]) << 1;
		agm->z = ((buf[3] & 0x30) | (buf[5] & 0x0f)) << 1;
		break;

	case 2:
		/* AGM-CONTACT packet: (count, sgm, agm) */
		synaptics_mt_state_set(&agm->mt_state, buf[1], buf[2], buf[4]);
		break;

	default:
		break;
	}

	/* Record that at least one AGM has been received since last SGM */
	priv->agm_pending = true;
}

static void synaptics_parse_ext_buttons(const unsigned char buf[],
					struct synaptics_data *priv,
					struct synaptics_hw_state *hw)
{
	unsigned int ext_bits =
		(SYN_CAP_MULTI_BUTTON_NO(priv->info.ext_cap) + 1) >> 1;
	unsigned int ext_mask = GENMASK(ext_bits - 1, 0);

	hw->ext_buttons = buf[4] & ext_mask;
	hw->ext_buttons |= (buf[5] & ext_mask) << ext_bits;
}

static int synaptics_parse_hw_state(const unsigned char buf[],
				    struct synaptics_data *priv,
				    struct synaptics_hw_state *hw)
{
	memset(hw, 0, sizeof(struct synaptics_hw_state));

	if (SYN_MODEL_NEWABS(priv->info.model_id)) {
		hw->w = (((buf[0] & 0x30) >> 2) |
			 ((buf[0] & 0x04) >> 1) |
			 ((buf[3] & 0x04) >> 2));

		hw->left  = (buf[0] & 0x01) ? 1 : 0;
		hw->right = (buf[0] & 0x02) ? 1 : 0;

		if (SYN_CAP_CLICKPAD(priv->info.ext_cap_0c)) {
			/*
			 * Clickpad's button is transmitted as middle button,
			 * however, since it is primary button, we will report
			 * it as BTN_LEFT.
			 */
			hw->left = ((buf[0] ^ buf[3]) & 0x01) ? 1 : 0;

		} else if (SYN_CAP_MIDDLE_BUTTON(priv->info.capabilities)) {
			hw->middle = ((buf[0] ^ buf[3]) & 0x01) ? 1 : 0;
			if (hw->w == 2)
				hw->scroll = (signed char)(buf[1]);
		}

		if (SYN_CAP_FOUR_BUTTON(priv->info.capabilities)) {
			hw->up   = ((buf[0] ^ buf[3]) & 0x01) ? 1 : 0;
			hw->down = ((buf[0] ^ buf[3]) & 0x02) ? 1 : 0;
		}

		if (synaptics_has_agm(priv) && hw->w == 2) {
			synaptics_parse_agm(buf, priv, hw);
			return 1;
		}

		hw->x = (((buf[3] & 0x10) << 8) |
			 ((buf[1] & 0x0f) << 8) |
			 buf[4]);
		hw->y = (((buf[3] & 0x20) << 7) |
			 ((buf[1] & 0xf0) << 4) |
			 buf[5]);
		hw->z = buf[2];

		if (SYN_CAP_MULTI_BUTTON_NO(priv->info.ext_cap) > 0 &&
		    ((buf[0] ^ buf[3]) & 0x02)) {
			synaptics_parse_ext_buttons(buf, priv, hw);
		}
	} else {
		hw->x = (((buf[1] & 0x1f) << 8) | buf[2]);
		hw->y = (((buf[4] & 0x1f) << 8) | buf[5]);

		hw->z = (((buf[0] & 0x30) << 2) | (buf[3] & 0x3F));
		hw->w = (((buf[1] & 0x80) >> 4) | ((buf[0] & 0x04) >> 1));

		hw->left  = (buf[0] & 0x01) ? 1 : 0;
		hw->right = (buf[0] & 0x02) ? 1 : 0;
	}

	/*
	 * Convert wrap-around values to negative. (X|Y)_MAX_POSITIVE
	 * is used by some firmware to indicate a finger at the edge of
	 * the touchpad whose precise position cannot be determined, so
	 * convert these values to the maximum axis value.
	 */
	if (hw->x > X_MAX_POSITIVE)
		hw->x -= 1 << ABS_POS_BITS;
	else if (hw->x == X_MAX_POSITIVE)
		hw->x = XMAX;

	if (hw->y > Y_MAX_POSITIVE)
		hw->y -= 1 << ABS_POS_BITS;
	else if (hw->y == Y_MAX_POSITIVE)
		hw->y = YMAX;

	return 0;
}

static void synaptics_report_semi_mt_slot(struct input_dev *dev, int slot,
					  bool active, int x, int y)
{
	input_mt_slot(dev, slot);
	input_mt_report_slot_state(dev, MT_TOOL_FINGER, active);
	if (active) {
		input_report_abs(dev, ABS_MT_POSITION_X, x);
		input_report_abs(dev, ABS_MT_POSITION_Y, synaptics_invert_y(y));
	}
}

static void synaptics_report_semi_mt_data(struct input_dev *dev,
					  const struct synaptics_hw_state *a,
					  const struct synaptics_hw_state *b,
					  int num_fingers)
{
	if (num_fingers >= 2) {
		synaptics_report_semi_mt_slot(dev, 0, true, min(a->x, b->x),
					      min(a->y, b->y));
		synaptics_report_semi_mt_slot(dev, 1, true, max(a->x, b->x),
					      max(a->y, b->y));
	} else if (num_fingers == 1) {
		synaptics_report_semi_mt_slot(dev, 0, true, a->x, a->y);
		synaptics_report_semi_mt_slot(dev, 1, false, 0, 0);
	} else {
		synaptics_report_semi_mt_slot(dev, 0, false, 0, 0);
		synaptics_report_semi_mt_slot(dev, 1, false, 0, 0);
	}
}

static void synaptics_report_ext_buttons(struct psmouse *psmouse,
					 const struct synaptics_hw_state *hw)
{
	struct input_dev *dev = psmouse->dev;
	struct synaptics_data *priv = psmouse->private;
	int ext_bits = (SYN_CAP_MULTI_BUTTON_NO(priv->info.ext_cap) + 1) >> 1;
	int i;

	if (!SYN_CAP_MULTI_BUTTON_NO(priv->info.ext_cap))
		return;

	/* Bug in FW 8.1 & 8.2, buttons are reported only when ExtBit is 1 */
	if ((SYN_ID_FULL(priv->info.identity) == 0x801 ||
	     SYN_ID_FULL(priv->info.identity) == 0x802) &&
	    !((psmouse->packet[0] ^ psmouse->packet[3]) & 0x02))
		return;

	if (!SYN_CAP_EXT_BUTTONS_STICK(priv->info.ext_cap_10)) {
		for (i = 0; i < ext_bits; i++) {
			input_report_key(dev, BTN_0 + 2 * i,
				hw->ext_buttons & BIT(i));
			input_report_key(dev, BTN_1 + 2 * i,
				hw->ext_buttons & BIT(i + ext_bits));
		}
		return;
	}

	/*
	 * This generation of touchpads has the trackstick buttons
	 * physically wired to the touchpad. Re-route them through
	 * the pass-through interface.
	 */
	if (priv->pt_port) {
		u8 pt_buttons;

		/* The trackstick expects at most 3 buttons */
		pt_buttons = SYN_EXT_BUTTON_STICK_L(hw->ext_buttons)      |
			     SYN_EXT_BUTTON_STICK_R(hw->ext_buttons) << 1 |
			     SYN_EXT_BUTTON_STICK_M(hw->ext_buttons) << 2;

		serio_interrupt(priv->pt_port,
				PSMOUSE_OOB_EXTRA_BTNS, SERIO_OOB_DATA);
		serio_interrupt(priv->pt_port, pt_buttons, SERIO_OOB_DATA);
	}
}

static void synaptics_report_buttons(struct psmouse *psmouse,
				     const struct synaptics_hw_state *hw)
{
	struct input_dev *dev = psmouse->dev;
	struct synaptics_data *priv = psmouse->private;

	input_report_key(dev, BTN_LEFT, hw->left);
	input_report_key(dev, BTN_RIGHT, hw->right);

	if (SYN_CAP_MIDDLE_BUTTON(priv->info.capabilities))
		input_report_key(dev, BTN_MIDDLE, hw->middle);

	if (SYN_CAP_FOUR_BUTTON(priv->info.capabilities)) {
		input_report_key(dev, BTN_FORWARD, hw->up);
		input_report_key(dev, BTN_BACK, hw->down);
	}

	synaptics_report_ext_buttons(psmouse, hw);
}

static void synaptics_report_slot(struct input_dev *dev, int slot,
				  const struct synaptics_hw_state *hw)
{
	input_mt_slot(dev, slot);
	input_mt_report_slot_state(dev, MT_TOOL_FINGER, (hw != NULL));
	if (!hw)
		return;

	input_report_abs(dev, ABS_MT_POSITION_X, hw->x);
	input_report_abs(dev, ABS_MT_POSITION_Y, synaptics_invert_y(hw->y));
	input_report_abs(dev, ABS_MT_PRESSURE, hw->z);
}

static void synaptics_report_mt_data(struct psmouse *psmouse,
				     struct synaptics_mt_state *mt_state,
				     const struct synaptics_hw_state *sgm)
{
	struct input_dev *dev = psmouse->dev;
	struct synaptics_data *priv = psmouse->private;
	struct synaptics_hw_state *agm = &priv->agm;
	struct synaptics_mt_state *old = &priv->mt_state;

	switch (mt_state->count) {
	case 0:
		synaptics_report_slot(dev, 0, NULL);
		synaptics_report_slot(dev, 1, NULL);
		break;
	case 1:
		if (mt_state->sgm == -1) {
			synaptics_report_slot(dev, 0, NULL);
			synaptics_report_slot(dev, 1, NULL);
		} else if (mt_state->sgm == 0) {
			synaptics_report_slot(dev, 0, sgm);
			synaptics_report_slot(dev, 1, NULL);
		} else {
			synaptics_report_slot(dev, 0, NULL);
			synaptics_report_slot(dev, 1, sgm);
		}
		break;
	default:
		/*
		 * If the finger slot contained in SGM is valid, and either
		 * hasn't changed, or is new, or the old SGM has now moved to
		 * AGM, then report SGM in MTB slot 0.
		 * Otherwise, empty MTB slot 0.
		 */
		if (mt_state->sgm != -1 &&
		    (mt_state->sgm == old->sgm ||
		     old->sgm == -1 || mt_state->agm == old->sgm))
			synaptics_report_slot(dev, 0, sgm);
		else
			synaptics_report_slot(dev, 0, NULL);

		/*
		 * If the finger slot contained in AGM is valid, and either
		 * hasn't changed, or is new, then report AGM in MTB slot 1.
		 * Otherwise, empty MTB slot 1.
		 *
		 * However, in the case where the AGM is new, make sure that
		 * that it is either the same as the old SGM, or there was no
		 * SGM.
		 *
		 * Otherwise, if the SGM was just 1, and the new AGM is 2, then
		 * the new AGM will keep the old SGM's tracking ID, which can
		 * cause apparent drumroll.  This happens if in the following
		 * valid finger sequence:
		 *
		 *  Action                 SGM  AGM (MTB slot:Contact)
		 *  1. Touch contact 0    (0:0)
		 *  2. Touch contact 1    (0:0, 1:1)
		 *  3. Lift  contact 0    (1:1)
		 *  4. Touch contacts 2,3 (0:2, 1:3)
		 *
		 * In step 4, contact 3, in AGM must not be given the same
		 * tracking ID as contact 1 had in step 3.  To avoid this,
		 * the first agm with contact 3 is dropped and slot 1 is
		 * invalidated (tracking ID = -1).
		 */
		if (mt_state->agm != -1 &&
		    (mt_state->agm == old->agm ||
		     (old->agm == -1 &&
		      (old->sgm == -1 || mt_state->agm == old->sgm))))
			synaptics_report_slot(dev, 1, agm);
		else
			synaptics_report_slot(dev, 1, NULL);
		break;
	}

	/* Don't use active slot count to generate BTN_TOOL events. */
	input_mt_report_pointer_emulation(dev, false);

	/* Send the number of fingers reported by touchpad itself. */
	input_mt_report_finger_count(dev, mt_state->count);

	synaptics_report_buttons(psmouse, sgm);

	input_sync(dev);
}

/* Handle case where mt_state->count = 0 */
static void synaptics_image_sensor_0f(struct synaptics_data *priv,
				      struct synaptics_mt_state *mt_state)
{
	synaptics_mt_state_set(mt_state, 0, -1, -1);
	priv->mt_state_lost = false;
}

/* Handle case where mt_state->count = 1 */
static void synaptics_image_sensor_1f(struct synaptics_data *priv,
				      struct synaptics_mt_state *mt_state)
{
	struct synaptics_hw_state *agm = &priv->agm;
	struct synaptics_mt_state *old = &priv->mt_state;

	/*
	 * If the last AGM was (0,0,0), and there is only one finger left,
	 * then we absolutely know that SGM contains slot 0, and all other
	 * fingers have been removed.
	 */
	if (priv->agm_pending && agm->z == 0) {
		synaptics_mt_state_set(mt_state, 1, 0, -1);
		priv->mt_state_lost = false;
		return;
	}

	switch (old->count) {
	case 0:
		synaptics_mt_state_set(mt_state, 1, 0, -1);
		break;
	case 1:
		/*
		 * If mt_state_lost, then the previous transition was 3->1,
		 * and SGM now contains either slot 0 or 1, but we don't know
		 * which.  So, we just assume that the SGM now contains slot 1.
		 *
		 * If pending AGM and either:
		 *   (a) the previous SGM slot contains slot 0, or
		 *   (b) there was no SGM slot
		 * then, the SGM now contains slot 1
		 *
		 * Case (a) happens with very rapid "drum roll" gestures, where
		 * slot 0 finger is lifted and a new slot 1 finger touches
		 * within one reporting interval.
		 *
		 * Case (b) happens if initially two or more fingers tap
		 * briefly, and all but one lift before the end of the first
		 * reporting interval.
		 *
		 * (In both these cases, slot 0 will becomes empty, so SGM
		 * contains slot 1 with the new finger)
		 *
		 * Else, if there was no previous SGM, it now contains slot 0.
		 *
		 * Otherwise, SGM still contains the same slot.
		 */
		if (priv->mt_state_lost ||
		    (priv->agm_pending && old->sgm <= 0))
			synaptics_mt_state_set(mt_state, 1, 1, -1);
		else if (old->sgm == -1)
			synaptics_mt_state_set(mt_state, 1, 0, -1);
		break;
	case 2:
		/*
		 * If mt_state_lost, we don't know which finger SGM contains.
		 *
		 * So, report 1 finger, but with both slots empty.
		 * We will use slot 1 on subsequent 1->1
		 */
		if (priv->mt_state_lost) {
			synaptics_mt_state_set(mt_state, 1, -1, -1);
			break;
		}
		/*
		 * Since the last AGM was NOT (0,0,0), it was the finger in
		 * slot 0 that has been removed.
		 * So, SGM now contains previous AGM's slot, and AGM is now
		 * empty.
		 */
		synaptics_mt_state_set(mt_state, 1, old->agm, -1);
		break;
	case 3:
		/*
		 * Since last AGM was not (0,0,0), we don't know which finger
		 * is left.
		 *
		 * So, report 1 finger, but with both slots empty.
		 * We will use slot 1 on subsequent 1->1
		 */
		synaptics_mt_state_set(mt_state, 1, -1, -1);
		priv->mt_state_lost = true;
		break;
	case 4:
	case 5:
		/* mt_state was updated by AGM-CONTACT packet */
		break;
	}
}

/* Handle case where mt_state->count = 2 */
static void synaptics_image_sensor_2f(struct synaptics_data *priv,
				      struct synaptics_mt_state *mt_state)
{
	struct synaptics_mt_state *old = &priv->mt_state;

	switch (old->count) {
	case 0:
		synaptics_mt_state_set(mt_state, 2, 0, 1);
		break;
	case 1:
		/*
		 * If previous SGM contained slot 1 or higher, SGM now contains
		 * slot 0 (the newly touching finger) and AGM contains SGM's
		 * previous slot.
		 *
		 * Otherwise, SGM still contains slot 0 and AGM now contains
		 * slot 1.
		 */
		if (old->sgm >= 1)
			synaptics_mt_state_set(mt_state, 2, 0, old->sgm);
		else
			synaptics_mt_state_set(mt_state, 2, 0, 1);
		break;
	case 2:
		/*
		 * If mt_state_lost, SGM now contains either finger 1 or 2, but
		 * we don't know which.
		 * So, we just assume that the SGM contains slot 0 and AGM 1.
		 */
		if (priv->mt_state_lost)
			synaptics_mt_state_set(mt_state, 2, 0, 1);
		/*
		 * Otherwise, use the same mt_state, since it either hasn't
		 * changed, or was updated by a recently received AGM-CONTACT
		 * packet.
		 */
		break;
	case 3:
		/*
		 * 3->2 transitions have two unsolvable problems:
		 *  1) no indication is given which finger was removed
		 *  2) no way to tell if agm packet was for finger 3
		 *     before 3->2, or finger 2 after 3->2.
		 *
		 * So, report 2 fingers, but empty all slots.
		 * We will guess slots [0,1] on subsequent 2->2.
		 */
		synaptics_mt_state_set(mt_state, 2, -1, -1);
		priv->mt_state_lost = true;
		break;
	case 4:
	case 5:
		/* mt_state was updated by AGM-CONTACT packet */
		break;
	}
}

/* Handle case where mt_state->count = 3 */
static void synaptics_image_sensor_3f(struct synaptics_data *priv,
				      struct synaptics_mt_state *mt_state)
{
	struct synaptics_mt_state *old = &priv->mt_state;

	switch (old->count) {
	case 0:
		synaptics_mt_state_set(mt_state, 3, 0, 2);
		break;
	case 1:
		/*
		 * If previous SGM contained slot 2 or higher, SGM now contains
		 * slot 0 (one of the newly touching fingers) and AGM contains
		 * SGM's previous slot.
		 *
		 * Otherwise, SGM now contains slot 0 and AGM contains slot 2.
		 */
		if (old->sgm >= 2)
			synaptics_mt_state_set(mt_state, 3, 0, old->sgm);
		else
			synaptics_mt_state_set(mt_state, 3, 0, 2);
		break;
	case 2:
		/*
		 * If the AGM previously contained slot 3 or higher, then the
		 * newly touching finger is in the lowest available slot.
		 *
		 * If SGM was previously 1 or higher, then the new SGM is
		 * now slot 0 (with a new finger), otherwise, the new finger
		 * is now in a hidden slot between 0 and AGM's slot.
		 *
		 * In all such cases, the SGM now contains slot 0, and the AGM
		 * continues to contain the same slot as before.
		 */
		if (old->agm >= 3) {
			synaptics_mt_state_set(mt_state, 3, 0, old->agm);
			break;
		}

		/*
		 * After some 3->1 and all 3->2 transitions, we lose track
		 * of which slot is reported by SGM and AGM.
		 *
		 * For 2->3 in this state, report 3 fingers, but empty all
		 * slots, and we will guess (0,2) on a subsequent 0->3.
		 *
		 * To userspace, the resulting transition will look like:
		 *    2:[0,1] -> 3:[-1,-1] -> 3:[0,2]
		 */
		if (priv->mt_state_lost) {
			synaptics_mt_state_set(mt_state, 3, -1, -1);
			break;
		}

		/*
		 * If the (SGM,AGM) really previously contained slots (0, 1),
		 * then we cannot know what slot was just reported by the AGM,
		 * because the 2->3 transition can occur either before or after
		 * the AGM packet. Thus, this most recent AGM could contain
		 * either the same old slot 1 or the new slot 2.
		 * Subsequent AGMs will be reporting slot 2.
		 *
		 * To userspace, the resulting transition will look like:
		 *    2:[0,1] -> 3:[0,-1] -> 3:[0,2]
		 */
		synaptics_mt_state_set(mt_state, 3, 0, -1);
		break;
	case 3:
		/*
		 * If, for whatever reason, the previous agm was invalid,
		 * Assume SGM now contains slot 0, AGM now contains slot 2.
		 */
		if (old->agm <= 2)
			synaptics_mt_state_set(mt_state, 3, 0, 2);
		/*
		 * mt_state either hasn't changed, or was updated by a recently
		 * received AGM-CONTACT packet.
		 */
		break;

	case 4:
	case 5:
		/* mt_state was updated by AGM-CONTACT packet */
		break;
	}
}

/* Handle case where mt_state->count = 4, or = 5 */
static void synaptics_image_sensor_45f(struct synaptics_data *priv,
				       struct synaptics_mt_state *mt_state)
{
	/* mt_state was updated correctly by AGM-CONTACT packet */
	priv->mt_state_lost = false;
}

static void synaptics_image_sensor_process(struct psmouse *psmouse,
					   struct synaptics_hw_state *sgm)
{
	struct synaptics_data *priv = psmouse->private;
	struct synaptics_hw_state *agm = &priv->agm;
	struct synaptics_mt_state mt_state;

	/* Initialize using current mt_state (as updated by last agm) */
	mt_state = agm->mt_state;

	/*
	 * Update mt_state using the new finger count and current mt_state.
	 */
	if (sgm->z == 0)
		synaptics_image_sensor_0f(priv, &mt_state);
	else if (sgm->w >= 4)
		synaptics_image_sensor_1f(priv, &mt_state);
	else if (sgm->w == 0)
		synaptics_image_sensor_2f(priv, &mt_state);
	else if (sgm->w == 1 && mt_state.count <= 3)
		synaptics_image_sensor_3f(priv, &mt_state);
	else
		synaptics_image_sensor_45f(priv, &mt_state);

	/* Send resulting input events to user space */
	synaptics_report_mt_data(psmouse, &mt_state, sgm);

	/* Store updated mt_state */
	priv->mt_state = agm->mt_state = mt_state;
	priv->agm_pending = false;
}

static bool synaptics_has_multifinger(struct synaptics_data *priv)
{
	if (SYN_CAP_MULTIFINGER(priv->info.capabilities))
		return true;

	/* Advanced gesture mode also sends multi finger data */
	return synaptics_has_agm(priv);
}

/*
 *  called for each full received packet from the touchpad
 */
static void synaptics_process_packet(struct psmouse *psmouse)
{
	struct input_dev *dev = psmouse->dev;
	struct synaptics_data *priv = psmouse->private;
	struct synaptics_device_info *info = &priv->info;
	struct synaptics_hw_state hw;
	int num_fingers;
	int finger_width;

	if (synaptics_parse_hw_state(psmouse->packet, priv, &hw))
		return;

	if (SYN_CAP_IMAGE_SENSOR(info->ext_cap_0c)) {
		synaptics_image_sensor_process(psmouse, &hw);
		return;
	}

	if (hw.scroll) {
		priv->scroll += hw.scroll;

		while (priv->scroll >= 4) {
			input_report_key(dev, BTN_BACK, !hw.down);
			input_sync(dev);
			input_report_key(dev, BTN_BACK, hw.down);
			input_sync(dev);
			priv->scroll -= 4;
		}
		while (priv->scroll <= -4) {
			input_report_key(dev, BTN_FORWARD, !hw.up);
			input_sync(dev);
			input_report_key(dev, BTN_FORWARD, hw.up);
			input_sync(dev);
			priv->scroll += 4;
		}
		return;
	}

	if (hw.z > 0 && hw.x > 1) {
		num_fingers = 1;
		finger_width = 5;
		if (SYN_CAP_EXTENDED(info->capabilities)) {
			switch (hw.w) {
			case 0 ... 1:
				if (synaptics_has_multifinger(priv))
					num_fingers = hw.w + 2;
				break;
			case 2:
				if (SYN_MODEL_PEN(info->model_id))
					;   /* Nothing, treat a pen as a single finger */
				break;
			case 4 ... 15:
				if (SYN_CAP_PALMDETECT(info->capabilities))
					finger_width = hw.w;
				break;
			}
		}
	} else {
		num_fingers = 0;
		finger_width = 0;
	}

	if (SYN_CAP_ADV_GESTURE(info->ext_cap_0c))
		synaptics_report_semi_mt_data(dev, &hw, &priv->agm,
					      num_fingers);

	/* Post events
	 * BTN_TOUCH has to be first as mousedev relies on it when doing
	 * absolute -> relative conversion
	 */
	if (hw.z > 30) input_report_key(dev, BTN_TOUCH, 1);
	if (hw.z < 25) input_report_key(dev, BTN_TOUCH, 0);

	if (num_fingers > 0) {
		input_report_abs(dev, ABS_X, hw.x);
		input_report_abs(dev, ABS_Y, synaptics_invert_y(hw.y));
	}
	input_report_abs(dev, ABS_PRESSURE, hw.z);

	if (SYN_CAP_PALMDETECT(info->capabilities))
		input_report_abs(dev, ABS_TOOL_WIDTH, finger_width);

	input_report_key(dev, BTN_TOOL_FINGER, num_fingers == 1);
	if (synaptics_has_multifinger(priv)) {
		input_report_key(dev, BTN_TOOL_DOUBLETAP, num_fingers == 2);
		input_report_key(dev, BTN_TOOL_TRIPLETAP, num_fingers == 3);
	}

	synaptics_report_buttons(psmouse, &hw);

	input_sync(dev);
}

static int synaptics_validate_byte(struct psmouse *psmouse,
				   int idx, unsigned char pkt_type)
{
	static const unsigned char newabs_mask[]	= { 0xC8, 0x00, 0x00, 0xC8, 0x00 };
	static const unsigned char newabs_rel_mask[]	= { 0xC0, 0x00, 0x00, 0xC0, 0x00 };
	static const unsigned char newabs_rslt[]	= { 0x80, 0x00, 0x00, 0xC0, 0x00 };
	static const unsigned char oldabs_mask[]	= { 0xC0, 0x60, 0x00, 0xC0, 0x60 };
	static const unsigned char oldabs_rslt[]	= { 0xC0, 0x00, 0x00, 0x80, 0x00 };
	const char *packet = psmouse->packet;

	if (idx < 0 || idx > 4)
		return 0;

	switch (pkt_type) {

	case SYN_NEWABS:
	case SYN_NEWABS_RELAXED:
		return (packet[idx] & newabs_rel_mask[idx]) == newabs_rslt[idx];

	case SYN_NEWABS_STRICT:
		return (packet[idx] & newabs_mask[idx]) == newabs_rslt[idx];

	case SYN_OLDABS:
		return (packet[idx] & oldabs_mask[idx]) == oldabs_rslt[idx];

	default:
		psmouse_err(psmouse, "unknown packet type %d\n", pkt_type);
		return 0;
	}
}

static unsigned char synaptics_detect_pkt_type(struct psmouse *psmouse)
{
	int i;

	for (i = 0; i < 5; i++)
		if (!synaptics_validate_byte(psmouse, i, SYN_NEWABS_STRICT)) {
			psmouse_info(psmouse, "using relaxed packet validation\n");
			return SYN_NEWABS_RELAXED;
		}

	return SYN_NEWABS_STRICT;
}

static psmouse_ret_t synaptics_process_byte(struct psmouse *psmouse)
{
	struct synaptics_data *priv = psmouse->private;

	if (psmouse->pktcnt >= 6) { /* Full packet received */
		if (unlikely(priv->pkt_type == SYN_NEWABS))
			priv->pkt_type = synaptics_detect_pkt_type(psmouse);

		if (SYN_CAP_PASS_THROUGH(priv->info.capabilities) &&
		    synaptics_is_pt_packet(psmouse->packet)) {
			if (priv->pt_port)
				synaptics_pass_pt_packet(priv->pt_port,
							 psmouse->packet);
		} else
			synaptics_process_packet(psmouse);

		return PSMOUSE_FULL_PACKET;
	}

	return synaptics_validate_byte(psmouse, psmouse->pktcnt - 1, priv->pkt_type) ?
		PSMOUSE_GOOD_DATA : PSMOUSE_BAD_DATA;
}

/*****************************************************************************
 *	Driver initialization/cleanup functions
 ****************************************************************************/
static void set_abs_position_params(struct input_dev *dev,
				    struct synaptics_device_info *info,
				    int x_code, int y_code)
{
	int x_min = info->x_min ?: XMIN_NOMINAL;
	int x_max = info->x_max ?: XMAX_NOMINAL;
	int y_min = info->y_min ?: YMIN_NOMINAL;
	int y_max = info->y_max ?: YMAX_NOMINAL;
	int fuzz = SYN_CAP_REDUCED_FILTERING(info->ext_cap_0c) ?
			SYN_REDUCED_FILTER_FUZZ : 0;

	input_set_abs_params(dev, x_code, x_min, x_max, fuzz, 0);
	input_set_abs_params(dev, y_code, y_min, y_max, fuzz, 0);
	input_abs_set_res(dev, x_code, info->x_res);
	input_abs_set_res(dev, y_code, info->y_res);
}

static void set_input_params(struct psmouse *psmouse,
			     struct synaptics_data *priv)
{
	struct input_dev *dev = psmouse->dev;
	struct synaptics_device_info *info = &priv->info;
	int i;

	/* Things that apply to both modes */
	__set_bit(INPUT_PROP_POINTER, dev->propbit);
	__set_bit(EV_KEY, dev->evbit);
	__set_bit(BTN_LEFT, dev->keybit);
	__set_bit(BTN_RIGHT, dev->keybit);

	if (SYN_CAP_MIDDLE_BUTTON(info->capabilities))
		__set_bit(BTN_MIDDLE, dev->keybit);

	if (!priv->absolute_mode) {
		/* Relative mode */
		__set_bit(EV_REL, dev->evbit);
		__set_bit(REL_X, dev->relbit);
		__set_bit(REL_Y, dev->relbit);
		return;
	}

	/* Absolute mode */
	__set_bit(EV_ABS, dev->evbit);
	set_abs_position_params(dev, &priv->info, ABS_X, ABS_Y);
	input_set_abs_params(dev, ABS_PRESSURE, 0, 255, 0, 0);

	if (SYN_CAP_IMAGE_SENSOR(info->ext_cap_0c)) {
		set_abs_position_params(dev, info,
					ABS_MT_POSITION_X, ABS_MT_POSITION_Y);
		/* Image sensors can report per-contact pressure */
		input_set_abs_params(dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
		input_mt_init_slots(dev, 2, INPUT_MT_POINTER);

		/* Image sensors can signal 4 and 5 finger clicks */
		__set_bit(BTN_TOOL_QUADTAP, dev->keybit);
		__set_bit(BTN_TOOL_QUINTTAP, dev->keybit);
	} else if (SYN_CAP_ADV_GESTURE(info->ext_cap_0c)) {
		/* Non-image sensors with AGM use semi-mt */
		__set_bit(INPUT_PROP_SEMI_MT, dev->propbit);
		input_mt_init_slots(dev, 2, 0);
		set_abs_position_params(dev, info, ABS_MT_POSITION_X,
					ABS_MT_POSITION_Y);
	}

	if (SYN_CAP_PALMDETECT(info->capabilities))
		input_set_abs_params(dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	__set_bit(BTN_TOUCH, dev->keybit);
	__set_bit(BTN_TOOL_FINGER, dev->keybit);

	if (synaptics_has_multifinger(priv)) {
		__set_bit(BTN_TOOL_DOUBLETAP, dev->keybit);
		__set_bit(BTN_TOOL_TRIPLETAP, dev->keybit);
	}

	if (SYN_CAP_FOUR_BUTTON(info->capabilities) ||
	    SYN_CAP_MIDDLE_BUTTON(info->capabilities)) {
		__set_bit(BTN_FORWARD, dev->keybit);
		__set_bit(BTN_BACK, dev->keybit);
	}

	if (!SYN_CAP_EXT_BUTTONS_STICK(info->ext_cap_10))
		for (i = 0; i < SYN_CAP_MULTI_BUTTON_NO(info->ext_cap); i++)
			__set_bit(BTN_0 + i, dev->keybit);

	__clear_bit(EV_REL, dev->evbit);
	__clear_bit(REL_X, dev->relbit);
	__clear_bit(REL_Y, dev->relbit);

	if (SYN_CAP_CLICKPAD(info->ext_cap_0c)) {
		__set_bit(INPUT_PROP_BUTTONPAD, dev->propbit);
		if (psmouse_matches_pnp_id(psmouse, topbuttonpad_pnp_ids) &&
		    !SYN_CAP_EXT_BUTTONS_STICK(info->ext_cap_10))
			__set_bit(INPUT_PROP_TOPBUTTONPAD, dev->propbit);
		/* Clickpads report only left button */
		__clear_bit(BTN_RIGHT, dev->keybit);
		__clear_bit(BTN_MIDDLE, dev->keybit);
	}
}

static ssize_t synaptics_show_disable_gesture(struct psmouse *psmouse,
					      void *data, char *buf)
{
	struct synaptics_data *priv = psmouse->private;

	return sprintf(buf, "%c\n", priv->disable_gesture ? '1' : '0');
}

static ssize_t synaptics_set_disable_gesture(struct psmouse *psmouse,
					     void *data, const char *buf,
					     size_t len)
{
	struct synaptics_data *priv = psmouse->private;
	unsigned int value;
	int err;

	err = kstrtouint(buf, 10, &value);
	if (err)
		return err;

	if (value > 1)
		return -EINVAL;

	if (value == priv->disable_gesture)
		return len;

	priv->disable_gesture = value;
	if (value)
		priv->mode |= SYN_BIT_DISABLE_GESTURE;
	else
		priv->mode &= ~SYN_BIT_DISABLE_GESTURE;

	if (synaptics_mode_cmd(psmouse, priv->mode))
		return -EIO;

	return len;
}

PSMOUSE_DEFINE_ATTR(disable_gesture, S_IWUSR | S_IRUGO, NULL,
		    synaptics_show_disable_gesture,
		    synaptics_set_disable_gesture);

static void synaptics_disconnect(struct psmouse *psmouse)
{
	struct synaptics_data *priv = psmouse->private;

	/*
	 * We might have left a breadcrumb when trying to
	 * set up SMbus companion.
	 */
	psmouse_smbus_cleanup(psmouse);

	if (!priv->absolute_mode &&
			SYN_ID_DISGEST_SUPPORTED(priv->info.identity))
		device_remove_file(&psmouse->ps2dev.serio->dev,
				   &psmouse_attr_disable_gesture.dattr);

	synaptics_reset(psmouse);
	kfree(priv);
	psmouse->private = NULL;
}

static int synaptics_reconnect(struct psmouse *psmouse)
{
	struct synaptics_data *priv = psmouse->private;
	struct synaptics_device_info info;
	unsigned char param[2];
	int retry = 0;
	int error;

	do {
		psmouse_reset(psmouse);
		if (retry) {
			/*
			 * On some boxes, right after resuming, the touchpad
			 * needs some time to finish initializing (I assume
			 * it needs time to calibrate) and start responding
			 * to Synaptics-specific queries, so let's wait a
			 * bit.
			 */
			ssleep(1);
		}
		ps2_command(&psmouse->ps2dev, param, PSMOUSE_CMD_GETID);
		error = synaptics_detect(psmouse, 0);
	} while (error && ++retry < 3);

	if (error)
		return -1;

	if (retry > 1)
		psmouse_dbg(psmouse, "reconnected after %d tries\n", retry);

	if (synaptics_query_hardware(psmouse, &info)) {
		psmouse_err(psmouse, "Unable to query device.\n");
		return -1;
	}

	if (synaptics_set_mode(psmouse)) {
		psmouse_err(psmouse, "Unable to initialize device.\n");
		return -1;
	}

	if (info.identity != priv->info.identity ||
	    info.model_id != priv->info.model_id ||
	    info.capabilities != priv->info.capabilities ||
	    info.ext_cap != priv->info.ext_cap) {
		psmouse_err(psmouse,
			    "hardware appears to be different: id(%u-%u), model(%u-%u), caps(%x-%x), ext(%x-%x).\n",
			    priv->info.identity, info.identity,
			    priv->info.model_id, info.model_id,
			    priv->info.capabilities, info.capabilities,
			    priv->info.ext_cap, info.ext_cap);
		return -1;
	}

	return 0;
}

static bool impaired_toshiba_kbc;

static const struct dmi_system_id toshiba_dmi_table[] __initconst = {
#if defined(CONFIG_DMI) && defined(CONFIG_X86)
	{
		/* Toshiba Satellite */
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "TOSHIBA"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Satellite"),
		},
	},
	{
		/* Toshiba Dynabook */
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "TOSHIBA"),
			DMI_MATCH(DMI_PRODUCT_NAME, "dynabook"),
		},
	},
	{
		/* Toshiba Portege M300 */
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "TOSHIBA"),
			DMI_MATCH(DMI_PRODUCT_NAME, "PORTEGE M300"),
		},

	},
	{
		/* Toshiba Portege M300 */
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "TOSHIBA"),
			DMI_MATCH(DMI_PRODUCT_NAME, "Portable PC"),
			DMI_MATCH(DMI_PRODUCT_VERSION, "Version 1.0"),
		},

	},
#endif
	{ }
};

static bool broken_olpc_ec;

static const struct dmi_system_id olpc_dmi_table[] __initconst = {
#if defined(CONFIG_DMI) && defined(CONFIG_OLPC)
	{
		/* OLPC XO-1 or XO-1.5 */
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "OLPC"),
			DMI_MATCH(DMI_PRODUCT_NAME, "XO"),
		},
	},
#endif
	{ }
};

void __init synaptics_module_init(void)
{
	impaired_toshiba_kbc = dmi_check_system(toshiba_dmi_table);
	broken_olpc_ec = dmi_check_system(olpc_dmi_table);
}

static int synaptics_init_ps2(struct psmouse *psmouse,
			      struct synaptics_device_info *info,
			      bool absolute_mode)
{
	struct synaptics_data *priv;
	int err;

	synaptics_apply_quirks(psmouse, info);

	psmouse->private = priv = kzalloc(sizeof(struct synaptics_data), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->info = *info;
	priv->absolute_mode = absolute_mode;
	if (SYN_ID_DISGEST_SUPPORTED(info->identity))
		priv->disable_gesture = true;

	err = synaptics_set_mode(psmouse);
	if (err) {
		psmouse_err(psmouse, "Unable to initialize device.\n");
		goto init_fail;
	}

	priv->pkt_type = SYN_MODEL_NEWABS(info->model_id) ?
					SYN_NEWABS : SYN_OLDABS;

	psmouse_info(psmouse,
		     "Touchpad model: %lu, fw: %lu.%lu, id: %#x, caps: %#x/%#x/%#x/%#x, board id: %u, fw id: %u\n",
		     SYN_ID_MODEL(info->identity),
		     SYN_ID_MAJOR(info->identity), SYN_ID_MINOR(info->identity),
		     info->model_id,
		     info->capabilities, info->ext_cap, info->ext_cap_0c,
		     info->ext_cap_10, info->board_id, info->firmware_id);

	set_input_params(psmouse, priv);

	/*
	 * Encode touchpad model so that it can be used to set
	 * input device->id.version and be visible to userspace.
	 * Because version is __u16 we have to drop something.
	 * Hardware info bits seem to be good candidates as they
	 * are documented to be for Synaptics corp. internal use.
	 */
	psmouse->model = ((info->model_id & 0x00ff0000) >> 8) |
			  (info->model_id & 0x000000ff);

	if (absolute_mode) {
		psmouse->protocol_handler = synaptics_process_byte;
		psmouse->pktsize = 6;
	} else {
		/* Relative mode follows standard PS/2 mouse protocol */
		psmouse->protocol_handler = psmouse_process_byte;
		psmouse->pktsize = 3;
	}

	psmouse->set_rate = synaptics_set_rate;
	psmouse->disconnect = synaptics_disconnect;
	psmouse->reconnect = synaptics_reconnect;
	psmouse->cleanup = synaptics_reset;
	/* Synaptics can usually stay in sync without extra help */
	psmouse->resync_time = 0;

	if (SYN_CAP_PASS_THROUGH(info->capabilities))
		synaptics_pt_create(psmouse);

	/*
	 * Toshiba's KBC seems to have trouble handling data from
	 * Synaptics at full rate.  Switch to a lower rate (roughly
	 * the same rate as a standard PS/2 mouse).
	 */
	if (psmouse->rate >= 80 && impaired_toshiba_kbc) {
		psmouse_info(psmouse,
			     "Toshiba %s detected, limiting rate to 40pps.\n",
			     dmi_get_system_info(DMI_PRODUCT_NAME));
		psmouse->rate = 40;
	}

	if (!priv->absolute_mode && SYN_ID_DISGEST_SUPPORTED(info->identity)) {
		err = device_create_file(&psmouse->ps2dev.serio->dev,
					 &psmouse_attr_disable_gesture.dattr);
		if (err) {
			psmouse_err(psmouse,
				    "Failed to create disable_gesture attribute (%d)",
				    err);
			goto init_fail;
		}
	}

	return 0;

 init_fail:
	kfree(priv);
	return err;
}

static int __synaptics_init(struct psmouse *psmouse, bool absolute_mode)
{
	struct synaptics_device_info info;
	int error;

	psmouse_reset(psmouse);

	error = synaptics_query_hardware(psmouse, &info);
	if (error) {
		psmouse_err(psmouse, "Unable to query device: %d\n", error);
		return error;
	}

	return synaptics_init_ps2(psmouse, &info, absolute_mode);
}

int synaptics_init_absolute(struct psmouse *psmouse)
{
	return __synaptics_init(psmouse, true);
}

int synaptics_init_relative(struct psmouse *psmouse)
{
	return __synaptics_init(psmouse, false);
}

static int synaptics_setup_ps2(struct psmouse *psmouse,
			       struct synaptics_device_info *info)
{
	bool absolute_mode = true;
	int error;

	/*
	 * The OLPC XO has issues with Synaptics' absolute mode; the constant
	 * packet spew overloads the EC such that key presses on the keyboard
	 * are missed.  Given that, don't even attempt to use Absolute mode.
	 * Relative mode seems to work just fine.
	 */
	if (broken_olpc_ec) {
		psmouse_info(psmouse,
			     "OLPC XO detected, forcing relative protocol.\n");
		absolute_mode = false;
	}

	error = synaptics_init_ps2(psmouse, info, absolute_mode);
	if (error)
		return error;

	return absolute_mode ? PSMOUSE_SYNAPTICS : PSMOUSE_SYNAPTICS_RELATIVE;
}

#else /* CONFIG_MOUSE_PS2_SYNAPTICS */

void __init synaptics_module_init(void)
{
}

static int __maybe_unused
synaptics_setup_ps2(struct psmouse *psmouse,
		    struct synaptics_device_info *info)
{
	return -ENOSYS;
}

#endif /* CONFIG_MOUSE_PS2_SYNAPTICS */

#ifdef CONFIG_MOUSE_PS2_SYNAPTICS_SMBUS

/*
 * The newest Synaptics device can use a secondary bus (called InterTouch) which
 * provides a better bandwidth and allow a better control of the touchpads.
 * This is used to decide if we need to use this bus or not.
 */
enum {
	SYNAPTICS_INTERTOUCH_NOT_SET = -1,
	SYNAPTICS_INTERTOUCH_OFF,
	SYNAPTICS_INTERTOUCH_ON,
};

static int synaptics_intertouch = IS_ENABLED(CONFIG_RMI4_SMB) ?
		SYNAPTICS_INTERTOUCH_NOT_SET : SYNAPTICS_INTERTOUCH_OFF;
module_param_named(synaptics_intertouch, synaptics_intertouch, int, 0644);
MODULE_PARM_DESC(synaptics_intertouch, "Use a secondary bus for the Synaptics device.");

static int synaptics_create_intertouch(struct psmouse *psmouse,
				       struct synaptics_device_info *info,
				       bool leave_breadcrumbs)
{
	bool topbuttonpad =
		psmouse_matches_pnp_id(psmouse, topbuttonpad_pnp_ids) &&
		!SYN_CAP_EXT_BUTTONS_STICK(info->ext_cap_10);
	const struct rmi_device_platform_data pdata = {
		.sensor_pdata = {
			.sensor_type = rmi_sensor_touchpad,
			.axis_align.flip_y = true,
			.kernel_tracking = false,
			.topbuttonpad = topbuttonpad,
		},
		.f30_data = {
			.buttonpad = SYN_CAP_CLICKPAD(info->ext_cap_0c),
			.trackstick_buttons =
				!!SYN_CAP_EXT_BUTTONS_STICK(info->ext_cap_10),
		},
	};
	const struct i2c_board_info intertouch_board = {
		I2C_BOARD_INFO("rmi4_smbus", 0x2c),
		.flags = I2C_CLIENT_HOST_NOTIFY,
	};

	return psmouse_smbus_init(psmouse, &intertouch_board,
				  &pdata, sizeof(pdata),
				  leave_breadcrumbs);
}

/**
 * synaptics_setup_intertouch - called once the PS/2 devices are enumerated
 * and decides to instantiate a SMBus InterTouch device.
 */
static int synaptics_setup_intertouch(struct psmouse *psmouse,
				      struct synaptics_device_info *info,
				      bool leave_breadcrumbs)
{
	int error;

	if (synaptics_intertouch == SYNAPTICS_INTERTOUCH_OFF)
		return -ENXIO;

	if (synaptics_intertouch == SYNAPTICS_INTERTOUCH_NOT_SET) {
		if (!psmouse_matches_pnp_id(psmouse, smbus_pnp_ids))
			return -ENXIO;
	}

	psmouse_info(psmouse, "Trying to set up SMBus access\n");

	error = synaptics_create_intertouch(psmouse, info, leave_breadcrumbs);
	if (error) {
		if (error == -EAGAIN)
			psmouse_info(psmouse, "SMbus companion is not ready yet\n");
		else
			psmouse_err(psmouse, "unable to create intertouch device\n");

		return error;
	}

	return 0;
}

int synaptics_init_smbus(struct psmouse *psmouse)
{
	struct synaptics_device_info info;
	int error;

	psmouse_reset(psmouse);

	error = synaptics_query_hardware(psmouse, &info);
	if (error) {
		psmouse_err(psmouse, "Unable to query device: %d\n", error);
		return error;
	}

	if (!SYN_CAP_INTERTOUCH(info.ext_cap_0c))
		return -ENXIO;

	return synaptics_create_intertouch(psmouse, &info, false);
}

#else /* CONFIG_MOUSE_PS2_SYNAPTICS_SMBUS */

static int __maybe_unused
synaptics_setup_intertouch(struct psmouse *psmouse,
			   struct synaptics_device_info *info,
			   bool leave_breadcrumbs)
{
	return -ENOSYS;
}

int synaptics_init_smbus(struct psmouse *psmouse)
{
	return -ENOSYS;
}

#endif /* CONFIG_MOUSE_PS2_SYNAPTICS_SMBUS */

#if defined(CONFIG_MOUSE_PS2_SYNAPTICS) || \
    defined(CONFIG_MOUSE_PS2_SYNAPTICS_SMBUS)

int synaptics_init(struct psmouse *psmouse)
{
	struct synaptics_device_info info;
	int error;
	int retval;

	psmouse_reset(psmouse);

	error = synaptics_query_hardware(psmouse, &info);
	if (error) {
		psmouse_err(psmouse, "Unable to query device: %d\n", error);
		return error;
	}

	if (SYN_CAP_INTERTOUCH(info.ext_cap_0c)) {
		error = synaptics_setup_intertouch(psmouse, &info, true);
		if (!error)
			return PSMOUSE_SYNAPTICS_SMBUS;
	}

	retval = synaptics_setup_ps2(psmouse, &info);
	if (retval < 0) {
		/*
		 * Not using any flavor of Synaptics support, so clean up
		 * SMbus breadcrumbs, if any.
		 */
		psmouse_smbus_cleanup(psmouse);
	}

	return retval;
}

#else /* CONFIG_MOUSE_PS2_SYNAPTICS || CONFIG_MOUSE_PS2_SYNAPTICS_SMBUS */

int synaptics_init(struct psmouse *psmouse)
{
	return -ENOSYS;
}

#endif /* CONFIG_MOUSE_PS2_SYNAPTICS || CONFIG_MOUSE_PS2_SYNAPTICS_SMBUS */
