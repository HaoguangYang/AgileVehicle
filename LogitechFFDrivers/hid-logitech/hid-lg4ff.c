/*
 *  Force feedback support for Logitech Gaming Wheels
 *
 *  Including G27, G25, DFP, DFGT, FFEX, Momo, Momo2 &
 *  Speed Force Wireless (WiiWheel)
 *
 *  Copyright (c) 2010 Simon Wood <simon@mungewell.org>
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */


#include <linux/input.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include "../KLGDFF/plugin/klgd_ff_plugin.h"

#include "usbhid/usbhid.h"
#include "hid-lg.h"
#include "hid-lg4ff.h"
#include "hid-ids.h"

#define to_hid_device(pdev) container_of(pdev, struct hid_device, dev)

#define LG4FF_MMODE_IS_MULTIMODE 0
#define LG4FF_MMODE_SWITCHED 1
#define LG4FF_MMODE_NOT_MULTIMODE 2

#define LG4FF_MODE_NATIVE_IDX 0
#define LG4FF_MODE_DFEX_IDX 1
#define LG4FF_MODE_DFP_IDX 2
#define LG4FF_MODE_G25_IDX 3
#define LG4FF_MODE_DFGT_IDX 4
#define LG4FF_MODE_G27_IDX 5
#define LG4FF_MODE_G29_IDX 6
#define LG4FF_MODE_MAX_IDX 7

#define LG4FF_MODE_NATIVE BIT(LG4FF_MODE_NATIVE_IDX)
#define LG4FF_MODE_DFEX BIT(LG4FF_MODE_DFEX_IDX)
#define LG4FF_MODE_DFP BIT(LG4FF_MODE_DFP_IDX)
#define LG4FF_MODE_G25 BIT(LG4FF_MODE_G25_IDX)
#define LG4FF_MODE_DFGT BIT(LG4FF_MODE_DFGT_IDX)
#define LG4FF_MODE_G27 BIT(LG4FF_MODE_G27_IDX)
#define LG4FF_MODE_G29 BIT(LG4FF_MODE_G29_IDX)

#define LG4FF_DFEX_TAG "DF-EX"
#define LG4FF_DFEX_NAME "Driving Force / Formula EX"
#define LG4FF_DFP_TAG "DFP"
#define LG4FF_DFP_NAME "Driving Force Pro"
#define LG4FF_G25_TAG "G25"
#define LG4FF_G25_NAME "G25 Racing Wheel"
#define LG4FF_G27_TAG "G27"
#define LG4FF_G27_NAME "G27 Racing Wheel"
#define LG4FF_G29_TAG "G29"
#define LG4FF_G29_NAME "G29 Racing Wheel"
#define LG4FF_DFGT_TAG "DFGT"
#define LG4FF_DFGT_NAME "Driving Force GT"

#define LG4FF_FFEX_REV_MAJ 0x21
#define LG4FF_FFEX_REV_MIN 0x00

#define EFFECT_COUNT 16

#define CMD_DOWNLOAD_FORCE	0
#define CMD_DOWNLOAD_PLAY_FORCE	1
#define CMD_PLAY_FORCE		2
#define CMD_STOP_FORCE		3
#define CMD_REFRESH_FORCE	12

static void lg4ff_set_range_dfp(struct hid_device *hid, u16 range);
static void lg4ff_set_range_g25(struct hid_device *hid, u16 range);

struct lg4ff_wheel_data {
	const u32 product_id;
	u16 range;
	const u16 min_range;
	const u16 max_range;
#ifdef CONFIG_LEDS_CLASS
	u8  led_state;
	struct led_classdev *led[5];
#endif
	const u32 alternate_modes;
	const char * const real_tag;
	const char * const real_name;
	const u16 real_product_id;

	void (*set_range)(struct hid_device *hid, u16 range);
	s16 effect_ids[EFFECT_COUNT];
	u8 slot_playing[EFFECT_COUNT];
	struct klgd_main klgd;
	struct klgd_plugin *ff_plugin;
	u16 gain;
	bool ffex_autocenter;
};

struct lg4ff_device_entry {
	spinlock_t report_lock; /* Protect output HID report */
	struct hid_report *report;
	struct lg4ff_wheel_data wdata;
};

static const signed short lg4ff_wheel_effects[] = {
	FF_CONSTANT,
	FF_PERIODIC,
	FF_SINE,
	FF_SQUARE,
	FF_SAW_UP,
	FF_SAW_DOWN,
	FF_TRIANGLE,
	FF_SPRING,
	FF_DAMPER,
	FF_FRICTION,
	FF_INERTIA,
	FF_AUTOCENTER,
	-1
};

struct lg4ff_wheel {
	const u32 product_id;
	const signed short *ff_effects;
	const u16 min_range;
	const u16 max_range;
	void (*set_range)(struct hid_device *hid, u16 range);
};

struct lg4ff_compat_mode_switch {
	const u8 cmd_count;	/* Number of commands to send */
	const u8 cmd[];
};

struct lg4ff_wheel_ident_info {
	const u32 modes;
	const u16 mask;
	const u16 result;
	const u16 real_product_id;
};

struct lg4ff_multimode_wheel {
	const u16 product_id;
	const u32 alternate_modes;
	const char *real_tag;
	const char *real_name;
};

struct lg4ff_alternate_mode {
	const u16 product_id;
	const char *tag;
	const char *name;
};

static const struct lg4ff_wheel lg4ff_devices[] = {
	{USB_DEVICE_ID_LOGITECH_WHEEL,       lg4ff_wheel_effects, 40, 270, NULL},
	{USB_DEVICE_ID_LOGITECH_MOMO_WHEEL,  lg4ff_wheel_effects, 40, 270, NULL},
	{USB_DEVICE_ID_LOGITECH_DFP_WHEEL,   lg4ff_wheel_effects, 40, 900, lg4ff_set_range_dfp},
	{USB_DEVICE_ID_LOGITECH_G25_WHEEL,   lg4ff_wheel_effects, 40, 900, lg4ff_set_range_g25},
	{USB_DEVICE_ID_LOGITECH_DFGT_WHEEL,  lg4ff_wheel_effects, 40, 900, lg4ff_set_range_g25},
	{USB_DEVICE_ID_LOGITECH_G27_WHEEL,   lg4ff_wheel_effects, 40, 900, lg4ff_set_range_g25},
	{USB_DEVICE_ID_LOGITECH_G29_WHEEL,   lg4ff_wheel_effects, 40, 900, lg4ff_set_range_g25},
	{USB_DEVICE_ID_LOGITECH_MOMO_WHEEL2, lg4ff_wheel_effects, 40, 270, NULL},
	{USB_DEVICE_ID_LOGITECH_WII_WHEEL,   lg4ff_wheel_effects, 40, 270, NULL}
};

static const struct lg4ff_multimode_wheel lg4ff_multimode_wheels[] = {
	{USB_DEVICE_ID_LOGITECH_DFP_WHEEL,
	 LG4FF_MODE_NATIVE | LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	 LG4FF_DFP_TAG, LG4FF_DFP_NAME},
	{USB_DEVICE_ID_LOGITECH_G25_WHEEL,
	 LG4FF_MODE_NATIVE | LG4FF_MODE_G25 | LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	 LG4FF_G25_TAG, LG4FF_G25_NAME},
	{USB_DEVICE_ID_LOGITECH_DFGT_WHEEL,
	 LG4FF_MODE_NATIVE | LG4FF_MODE_DFGT | LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	 LG4FF_DFGT_TAG, LG4FF_DFGT_NAME},
	{USB_DEVICE_ID_LOGITECH_G27_WHEEL,
	 LG4FF_MODE_NATIVE | LG4FF_MODE_G27 | LG4FF_MODE_G25 | LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	 LG4FF_G27_TAG, LG4FF_G27_NAME},
	{USB_DEVICE_ID_LOGITECH_G29_WHEEL,
	 LG4FF_MODE_NATIVE | LG4FF_MODE_G29 | LG4FF_MODE_G27 | LG4FF_MODE_G25 | LG4FF_MODE_DFGT | LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	 LG4FF_G29_TAG, LG4FF_G29_NAME},
};

static const struct lg4ff_alternate_mode lg4ff_alternate_modes[] = {
	[LG4FF_MODE_NATIVE_IDX] = {0, "native", ""},
	[LG4FF_MODE_DFEX_IDX] = {USB_DEVICE_ID_LOGITECH_WHEEL, LG4FF_DFEX_TAG, LG4FF_DFEX_NAME},
	[LG4FF_MODE_DFP_IDX] = {USB_DEVICE_ID_LOGITECH_DFP_WHEEL, LG4FF_DFP_TAG, LG4FF_DFP_NAME},
	[LG4FF_MODE_G25_IDX] = {USB_DEVICE_ID_LOGITECH_G25_WHEEL, LG4FF_G25_TAG, LG4FF_G25_NAME},
	[LG4FF_MODE_DFGT_IDX] = {USB_DEVICE_ID_LOGITECH_DFGT_WHEEL, LG4FF_DFGT_TAG, LG4FF_DFGT_NAME},
	[LG4FF_MODE_G27_IDX] = {USB_DEVICE_ID_LOGITECH_G27_WHEEL, LG4FF_G27_TAG, LG4FF_G27_NAME},
	[LG4FF_MODE_G29_IDX] = {USB_DEVICE_ID_LOGITECH_G29_WHEEL, LG4FF_G29_TAG, LG4FF_G29_NAME},
};

/* Multimode wheel identificators */
static const struct lg4ff_wheel_ident_info lg4ff_dfp_ident_info = {
	LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	0xf000,
	0x1000,
	USB_DEVICE_ID_LOGITECH_DFP_WHEEL
};

static const struct lg4ff_wheel_ident_info lg4ff_g25_ident_info = {
	LG4FF_MODE_G25 | LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	0xff00,
	0x1200,
	USB_DEVICE_ID_LOGITECH_G25_WHEEL
};

static const struct lg4ff_wheel_ident_info lg4ff_g27_ident_info = {
	LG4FF_MODE_G27 | LG4FF_MODE_G25 | LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	0xfff0,
	0x1230,
	USB_DEVICE_ID_LOGITECH_G27_WHEEL
};

static const struct lg4ff_wheel_ident_info lg4ff_dfgt_ident_info = {
	LG4FF_MODE_DFGT | LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	0xff00,
	0x1300,
	USB_DEVICE_ID_LOGITECH_DFGT_WHEEL
};

static const struct lg4ff_wheel_ident_info lg4ff_g29_ident_info = {
	LG4FF_MODE_G29 | LG4FF_MODE_G27 | LG4FF_MODE_G25 | LG4FF_MODE_DFGT | LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	0xfff8,
	0x1350,
	USB_DEVICE_ID_LOGITECH_G29_WHEEL
};

static const struct lg4ff_wheel_ident_info lg4ff_g29_ident_info2 = {
	LG4FF_MODE_G29 | LG4FF_MODE_G27 | LG4FF_MODE_G25 | LG4FF_MODE_DFGT | LG4FF_MODE_DFP | LG4FF_MODE_DFEX,
	0xff00,
	0x8900,
	USB_DEVICE_ID_LOGITECH_G29_WHEEL
};

/* Multimode wheel identification checklists */
static const struct lg4ff_wheel_ident_info *lg4ff_main_checklist[] = {
	&lg4ff_g29_ident_info,
	&lg4ff_g29_ident_info2,
	&lg4ff_dfgt_ident_info,
	&lg4ff_g27_ident_info,
	&lg4ff_g25_ident_info,
	&lg4ff_dfp_ident_info
};

/* Compatibility mode switching commands */
/* EXT_CMD9 - Understood by G27 and DFGT */
static const struct lg4ff_compat_mode_switch lg4ff_mode_switch_ext09_dfex = {
	2,
	{0xf8, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00,	/* Revert mode upon USB reset */
	 0xf8, 0x09, 0x00, 0x01, 0x00, 0x00, 0x00}	/* Switch mode to DF-EX with detach */
};

static const struct lg4ff_compat_mode_switch lg4ff_mode_switch_ext09_dfp = {
	2,
	{0xf8, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00,	/* Revert mode upon USB reset */
	 0xf8, 0x09, 0x01, 0x01, 0x00, 0x00, 0x00}	/* Switch mode to DFP with detach */
};

static const struct lg4ff_compat_mode_switch lg4ff_mode_switch_ext09_g25 = {
	2,
	{0xf8, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00,	/* Revert mode upon USB reset */
	 0xf8, 0x09, 0x02, 0x01, 0x00, 0x00, 0x00}	/* Switch mode to G25 with detach */
};

static const struct lg4ff_compat_mode_switch lg4ff_mode_switch_ext09_dfgt = {
	2,
	{0xf8, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00,	/* Revert mode upon USB reset */
	 0xf8, 0x09, 0x03, 0x01, 0x00, 0x00, 0x00}	/* Switch mode to DFGT with detach */
};

static const struct lg4ff_compat_mode_switch lg4ff_mode_switch_ext09_g27 = {
	2,
	{0xf8, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00,	/* Revert mode upon USB reset */
	 0xf8, 0x09, 0x04, 0x01, 0x00, 0x00, 0x00}	/* Switch mode to G27 with detach */
};

static const struct lg4ff_compat_mode_switch lg4ff_mode_switch_ext09_g29 = {
	2,
	{0xf8, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00,	/* Revert mode upon USB reset */
	 0xf8, 0x09, 0x05, 0x01, 0x01, 0x00, 0x00}	/* Switch mode to G29 with detach */
};

/* EXT_CMD1 - Understood by DFP, G25, G27 and DFGT */
static const struct lg4ff_compat_mode_switch lg4ff_mode_switch_ext01_dfp = {
	1,
	{0xf8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00}
};

/* EXT_CMD16 - Understood by G25 and G27 */
static const struct lg4ff_compat_mode_switch lg4ff_mode_switch_ext16_g25 = {
	1,
	{0xf8, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00}
};

/* Recalculates X axis value accordingly to currently selected range */
static s32 lg4ff_adjust_dfp_x_axis(s32 value, u16 range)
{
	u16 max_range;
	s32 new_value;

	if (range == 900)
		return value;
	else if (range == 200)
		return value;
	else if (range < 200)
		max_range = 200;
	else
		max_range = 900;

	new_value = 8192 + mult_frac(value - 8192, max_range, range);
	if (new_value < 0)
		return 0;
	else if (new_value > 16383)
		return 16383;
	else
		return new_value;
}

int lg4ff_adjust_input_event(struct hid_device *hid, struct hid_field *field,
			     struct hid_usage *usage, s32 value, struct lg_drv_data *drv_data)
{
	struct lg4ff_device_entry *entry = drv_data->device_props;
	s32 new_value = 0;

	if (!entry) {
		hid_err(hid, "Device properties not found");
		return 0;
	}

	switch (entry->wdata.product_id) {
	case USB_DEVICE_ID_LOGITECH_DFP_WHEEL:
		switch (usage->code) {
		case ABS_X:
			new_value = lg4ff_adjust_dfp_x_axis(value, entry->wdata.range);
			input_event(field->hidinput->input, usage->type, usage->code, new_value);
			return 1;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static void lg4ff_init_wheel_data(struct lg4ff_wheel_data * const wdata, const struct lg4ff_wheel *wheel,
				  const struct lg4ff_multimode_wheel *mmode_wheel,
				  const u16 real_product_id)
{
	u32 alternate_modes = 0;
	const char *real_tag = NULL;
	const char *real_name = NULL;

	if (mmode_wheel) {
		alternate_modes = mmode_wheel->alternate_modes;
		real_tag = mmode_wheel->real_tag;
		real_name = mmode_wheel->real_name;
	}

	{
		struct lg4ff_wheel_data t_wdata =  { .product_id = wheel->product_id,
						     .real_product_id = real_product_id,
						     .min_range = wheel->min_range,
						     .max_range = wheel->max_range,
						     .set_range = wheel->set_range,
						     .alternate_modes = alternate_modes,
						     .real_tag = real_tag,
						     .real_name = real_name,
						     .effect_ids = {-2, -1, -1, -1},
						     .gain = 0xffff};

		memcpy(wdata, &t_wdata, sizeof(t_wdata));
	}
}

static struct lg4ff_device_entry *lg4ff_get_device_entry(struct input_dev *dev)
{
	struct hid_device *hid = input_get_drvdata(dev);
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Private driver data not found!\n");
		return NULL;
	}

	entry = drv_data->device_props;
	if (!entry) {
		hid_err(hid, "Device properties not found!\n");
		return NULL;
	}
	return entry;
}

/* Sends default autocentering command compatible with
 * all wheels except Formula Force EX */
static int lg4ff_set_autocenter_default(struct lg4ff_device_entry *entry, struct klgd_command_stream *s, u16 magnitude)
{
	u32 expand_a, expand_b;
	struct klgd_command *c;

	if (magnitude == 0) {
		c = klgd_alloc_cmd(7);
		if (!c)
			return -EAGAIN;

		c->bytes[0] = 0xf5;
		c->bytes[1] = 0x00;
		c->bytes[2] = 0x00;
		c->bytes[3] = 0x00;
		c->bytes[4] = 0x00;
		c->bytes[5] = 0x00;
		c->bytes[6] = 0x00;

		return klgd_append_cmd(s, c);
	}

	if (magnitude <= 0xaaaa) {
		expand_a = 0x0c * magnitude;
		expand_b = 0x80 * magnitude;
	} else {
		expand_a = (0x0c * 0xaaaa) + 0x06 * (magnitude - 0xaaaa);
		expand_b = (0x80 * 0xaaaa) + 0xff * (magnitude - 0xaaaa);
	}

	/* Adjust for non-MOMO wheels */
	switch (entry->wdata.product_id) {
	case USB_DEVICE_ID_LOGITECH_MOMO_WHEEL:
	case USB_DEVICE_ID_LOGITECH_MOMO_WHEEL2:
		break;
	default:
		expand_a = expand_a >> 1;
		break;
	}

	c = klgd_alloc_cmd(7);
	if (!c)
		return -EAGAIN;

	c->bytes[0] = 0xfe;
	c->bytes[1] = 0x0d;
	c->bytes[2] = expand_a / 0xaaaa;
	c->bytes[3] = expand_a / 0xaaaa;
	c->bytes[4] = expand_b / 0xaaaa;
	c->bytes[5] = 0x00;
	c->bytes[6] = 0x00;

	klgd_append_cmd(s, c);
	
	c = klgd_alloc_cmd(7);
	if (!c)
		return -EAGAIN;

	/* Activate Auto-Center */
	c->bytes[0] = 0x14;
	c->bytes[1] = 0x00;
	c->bytes[2] = 0x00;
	c->bytes[3] = 0x00;
	c->bytes[4] = 0x00;
	c->bytes[5] = 0x00;
	c->bytes[6] = 0x00;

	return klgd_append_cmd(s, c);
}

/* Sends autocentering command compatible with Formula Force EX */
static int lg4ff_set_autocenter_ffex(struct lg4ff_device_entry *entry, struct klgd_command_stream *s, u16 magnitude)
{
	struct klgd_command *c;
	magnitude = magnitude * 90 / 65535;

	c = klgd_alloc_cmd(7);
	if (!c)
		return -EAGAIN;

	c->bytes[0] = 0xfe;
	c->bytes[1] = 0x03;
	c->bytes[2] = magnitude >> 14;
	c->bytes[3] = magnitude >> 14;
	c->bytes[4] = magnitude;
	c->bytes[5] = 0x00;
	c->bytes[6] = 0x00;

	return klgd_append_cmd(s, c);
}

static __s8 lg4ff_get_slot(struct lg4ff_device_entry *entry, __s16 effect_id)
{
	int i;
	
	printk(KERN_DEBUG "Looking for effect %i,    (state %i %i %i %i)", effect_id, entry->wdata.effect_ids[0], entry->wdata.effect_ids[1], entry->wdata.effect_ids[2], entry->wdata.effect_ids[3]);
	for (i = 0; i < 4; i++)
		if (entry->wdata.effect_ids[i] == effect_id) {
			printk(KERN_DEBUG "found slot %i.\n", i);
			return i;
		}
	printk(KERN_DEBUG "found nothing!\n");
	return -1;
}

static int lg4ff_erase(struct lg4ff_device_entry *entry, struct klgd_command_stream *s,
                       const struct ff_effect *effect)
{
	s8 slot;

	if (effect->type == FF_CONSTANT)
		return 0;

	slot = lg4ff_get_slot(entry, effect->id);
	
	if (slot > 0)
		entry->wdata.effect_ids[slot] = -1;
	return 0;
}

static int lg4ff_set_autocenter(struct lg4ff_device_entry *entry, struct klgd_command_stream *s, const u16 autocenter)
{
	if (entry->wdata.ffex_autocenter)
		return lg4ff_set_autocenter_ffex(entry, s, autocenter);
	return lg4ff_set_autocenter_default(entry, s, autocenter);
}

static int lg4ff_set_gain(struct lg4ff_device_entry *entry, const u16 gain)
{
	entry->wdata.gain = gain;
	printk(KERN_DEBUG "Setting gain to %u\n", gain);

	return 0;
}

static int lg4ff_start(struct lg4ff_device_entry *entry, struct klgd_command_stream *s,
                       const struct ff_effect *effect)
{
	struct klgd_command *c;
	s8 slot = lg4ff_get_slot(entry, effect->id);

	if (slot < 0)
		return -EINVAL;

	c = klgd_alloc_cmd(7);
	if (!c)
		return -ENOMEM;

	c->bytes[0] = (16 << slot) | CMD_PLAY_FORCE;
	entry->wdata.slot_playing[slot] = 1;

	return klgd_append_cmd(s, c);
}

static int lg4ff_stop(struct lg4ff_device_entry *entry, struct klgd_command_stream *s,
                      const struct ff_effect *effect)
{
	struct klgd_command *c;
	s8 slot = lg4ff_get_slot(entry, effect->id);

	if (effect->type == FF_CONSTANT)
		slot = 0;

	if (slot < 0)
		return -EINVAL;

	c = klgd_alloc_cmd(7);
	if (!c)
		return -ENOMEM;

	c->bytes[0] = (16 << slot) | CMD_STOP_FORCE;
	entry->wdata.slot_playing[slot] = 0;

	return klgd_append_cmd(s, c);
}

static int lg4ff_upload(struct lg4ff_device_entry *entry, struct klgd_command_stream *s,
                        const struct ff_effect *effect, bool start, bool overwrite)
{
	struct klgd_command *c;
	s8 cmd, slot = lg4ff_get_slot(entry, effect->id);
	int x, y, cr, cl;
((struct ff_effect*)effect)->direction = 16384;
	
	if (effect->type == FF_CONSTANT)
		slot = 0;
	if (slot >= 0) {
		// effect exists
		if (entry->wdata.slot_playing[slot] && !overwrite)
			cmd = CMD_REFRESH_FORCE;
		else
			cmd = start ? CMD_DOWNLOAD_PLAY_FORCE : CMD_DOWNLOAD_FORCE;
	} else {
		// get new effect slot
		slot = lg4ff_get_slot(entry, -1);

		if (slot < 0)
			return -EINVAL;

		entry->wdata.effect_ids[slot] = effect->id;
		cmd = start ? CMD_DOWNLOAD_PLAY_FORCE : CMD_DOWNLOAD_FORCE;
	}

	c = klgd_alloc_cmd(7);
	if (!c)
		return -ENOMEM;

	c->bytes[0] = (16 << slot) | cmd;
	if (start)
		entry->wdata.slot_playing[slot] = true;

	switch (effect->type) {
		case FF_CONSTANT:
		{
			const u16 gain = entry->wdata.gain;
			const u16 level = effect->u.constant.level * gain / 0xffff;

			ffpl_lvl_dir_to_x_y(level, effect->direction, &x, &y);
			printk(KERN_DEBUG "Wheel constant: %i, direction %u  => %i\n", effect->u.constant.level, effect->direction, x * 0x7f / 0x7fff);
			c->bytes[2+slot] = 0x80 - x * 0x7f / 0x7fff;
			break;
		}
		case FF_DAMPER:
		case FF_FRICTION:
		case FF_INERTIA:
			printk(KERN_DEBUG "Wheel damper: %i %i, sat %i %i\n", effect->u.condition[0].right_coeff
			                                                    , effect->u.condition[0].left_coeff
			                                                    , effect->u.condition[0].right_saturation
			                                                    , effect->u.condition[0].left_saturation);

			/* calculate damper force values */
			x = max(effect->u.condition[0].right_coeff / 2048, -15);
			y = max(effect->u.condition[0].left_coeff / 2048, -15);
			c->bytes[1] = 0x0C;
			c->bytes[2] = abs(y);
			c->bytes[3] = y < 0;
			c->bytes[4] = abs(x);
			c->bytes[5] = x < 0;
			// saturation only supported on DFP and newer
			c->bytes[6] = max(effect->u.condition[0].right_saturation / 256, 2);
			break;
		case FF_SPRING:
			printk(KERN_DEBUG "Wheel spring coef: %i % i, sat: %i center: %i deadband: %i\n",
			                   effect->u.condition[0].right_coeff,
			                   effect->u.condition[0].left_coeff,
			                   effect->u.condition[0].right_saturation,
			                   effect->u.condition[0].center,
			                   effect->u.condition[0].deadband);

			/* calculate offsets */
			x = clamp(effect->u.condition[0].center - effect->u.condition[0].deadband/2 + 0x8000, 0, 0xffff) >> 5;
			y = clamp(effect->u.condition[0].center + effect->u.condition[0].deadband/2 + 0x8000, 0, 0xffff) >> 5;
			printk(KERN_DEBUG "offsets %x %x\n", x, y);
			/* calculate coefs */
			cr = max(effect->u.condition[0].right_coeff / 2048, -15);
			cl = max(effect->u.condition[0].left_coeff / 2048, -15);
			c->bytes[1] = 0x0B;
			c->bytes[2] = x >> 3;
			c->bytes[3] = y >> 3;
			c->bytes[4] = 16 * abs(cr) + abs(cl);
			c->bytes[5] = (x & 7) * 32 + (cr & 16) + (y & 7) * 2 + (cl < 0);
			c->bytes[6] = max(effect->u.condition[0].right_saturation / 256, 1);
			break;
		default:
			printk(KERN_DEBUG "Unexpected force type %i!", effect->type);		
	}

	return klgd_append_cmd(s, c);
}

int lg4ff_klgd_callback(void *data, const struct klgd_command_stream *s)
{
	int ret;
	size_t idx;

	printk(KERN_DEBUG "Command count: %lu\n", s->count);
	for (idx = 0; idx < s->count; idx++) {
		const struct klgd_command *c = s->commands[idx];
	
		ret = hid_hw_output_report((struct hid_device *)data, c->bytes, 7);

		printk(KERN_DEBUG "Wheel command: %02x %02x %02x %02x %02x %02x %02x, xferred = %i\n", c->bytes[0], c->bytes[1], c->bytes[2], c->bytes[3], c->bytes[4], c->bytes[5], c->bytes[6], ret);

		/* Error handling by KLGD later? How to deal with partial failure when sending multiple commands? */
	}

	return 0;
}

int lg4ff_control(struct input_dev *dev, struct klgd_command_stream *s, const enum ffpl_control_command cmd, const union ffpl_control_data data, void *user)
{
	struct lg4ff_device_entry *entry = lg4ff_get_device_entry(dev);
	int err;

	if (!entry)
		return -EINVAL;

	if (!s)
		return -EINVAL;

	printk(KERN_WARNING "KLGD command %d\n", (int)cmd);

	if (cmd != FFPL_SET_GAIN && cmd != FFPL_SET_AUTOCENTER) {
		if (!data.effects.cur) {
			printk(KERN_WARNING "HID-LG4FF: NULL effect, this _cannot_ happen!\n");
			return -EINVAL;
		}
	}

	switch (cmd) {
	case FFPL_EMP_TO_UPL:
		return lg4ff_upload(entry, s, data.effects.cur, false, false);
	case FFPL_UPL_TO_SRT:
		return lg4ff_start(entry, s, data.effects.cur);
	case FFPL_SRT_TO_UPL:
		return lg4ff_stop(entry, s, data.effects.cur);
	case FFPL_UPL_TO_EMP:
		return lg4ff_erase(entry, s, data.effects.cur);
	case FFPL_SRT_TO_UDT:
		return lg4ff_upload(entry, s, data.effects.cur, false, false);
	case FFPL_EMP_TO_SRT:
		return lg4ff_upload(entry, s, data.effects.cur, true, false);
	case FFPL_SRT_TO_EMP:
		err = lg4ff_stop(entry, s, data.effects.cur);
		if (err)
			return err;
		return lg4ff_erase(entry, s, data.effects.cur);
	case FFPL_OWR_TO_UPL:
		return lg4ff_upload(entry, s, data.effects.cur, false, true);
	case FFPL_OWR_TO_SRT:
		return lg4ff_upload(entry, s, data.effects.cur, true, true);
	case FFPL_SET_GAIN:
		return lg4ff_set_gain(entry, data.gain);
	case FFPL_SET_AUTOCENTER:
		return lg4ff_set_autocenter(entry, s, data.autocenter);
	default:
		printk(KERN_NOTICE "HID-LG4FF - Unhandled command\n");
		break;
	}

	return 0;
}

/* Sends command to set range compatible with G25/G27/Driving Force GT */
static void lg4ff_set_range_g25(struct hid_device *hid, u16 range)
{
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;
	unsigned long flags;
	s32 *value;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Private driver data not found!\n");
		return;
	}

	entry = drv_data->device_props;
	if (!entry) {
		hid_err(hid, "Device properties not found!\n");
		return;
	}
	value = entry->report->field[0]->value;
	dbg_hid("G25/G27/DFGT: setting range to %u\n", range);

	spin_lock_irqsave(&entry->report_lock, flags);
	value[0] = 0xf8;
	value[1] = 0x81;
	value[2] = range & 0x00ff;
	value[3] = (range & 0xff00) >> 8;
	value[4] = 0x00;
	value[5] = 0x00;
	value[6] = 0x00;

	hid_hw_request(hid, entry->report, HID_REQ_SET_REPORT);
	spin_unlock_irqrestore(&entry->report_lock, flags);
}

/* Sends commands to set range compatible with Driving Force Pro wheel */
static void lg4ff_set_range_dfp(struct hid_device *hid, u16 range)
{
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;
	unsigned long flags;
	int start_left, start_right, full_range;
	s32 *value;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Private driver data not found!\n");
		return;
	}

	entry = drv_data->device_props;
	if (!entry) {
		hid_err(hid, "Device properties not found!\n");
		return;
	}
	value = entry->report->field[0]->value;
	dbg_hid("Driving Force Pro: setting range to %u\n", range);

	/* Prepare "coarse" limit command */
	spin_lock_irqsave(&entry->report_lock, flags);
	value[0] = 0xf8;
	value[1] = 0x00;	/* Set later */
	value[2] = 0x00;
	value[3] = 0x00;
	value[4] = 0x00;
	value[5] = 0x00;
	value[6] = 0x00;

	if (range > 200) {
		value[1] = 0x03;
		full_range = 900;
	} else {
		value[1] = 0x02;
		full_range = 200;
	}
	hid_hw_request(hid, entry->report, HID_REQ_SET_REPORT);

	/* Prepare "fine" limit command */
	value[0] = 0x81;
	value[1] = 0x0b;
	value[2] = 0x00;
	value[3] = 0x00;
	value[4] = 0x00;
	value[5] = 0x00;
	value[6] = 0x00;

	if (range == 200 || range == 900) {	/* Do not apply any fine limit */
		hid_hw_request(hid, entry->report, HID_REQ_SET_REPORT);
		spin_unlock_irqrestore(&entry->report_lock, flags);
		return;
	}

	/* Construct fine limit command */
	start_left = (((full_range - range + 1) * 2047) / full_range);
	start_right = 0xfff - start_left;

	value[2] = start_left >> 4;
	value[3] = start_right >> 4;
	value[4] = 0xff;
	value[5] = (start_right & 0xe) << 4 | (start_left & 0xe);
	value[6] = 0xff;

	hid_hw_request(hid, entry->report, HID_REQ_SET_REPORT);
	spin_unlock_irqrestore(&entry->report_lock, flags);
}

static const struct lg4ff_compat_mode_switch *lg4ff_get_mode_switch_command(const u16 real_product_id, const u16 target_product_id)
{
	switch (real_product_id) {
	case USB_DEVICE_ID_LOGITECH_DFP_WHEEL:
		switch (target_product_id) {
		case USB_DEVICE_ID_LOGITECH_DFP_WHEEL:
			return &lg4ff_mode_switch_ext01_dfp;
		/* DFP can only be switched to its native mode */
		default:
			return NULL;
		}
		break;
	case USB_DEVICE_ID_LOGITECH_G25_WHEEL:
		switch (target_product_id) {
		case USB_DEVICE_ID_LOGITECH_DFP_WHEEL:
			return &lg4ff_mode_switch_ext01_dfp;
		case USB_DEVICE_ID_LOGITECH_G25_WHEEL:
			return &lg4ff_mode_switch_ext16_g25;
		/* G25 can only be switched to DFP mode or its native mode */
		default:
			return NULL;
		}
		break;
	case USB_DEVICE_ID_LOGITECH_G27_WHEEL:
		switch (target_product_id) {
		case USB_DEVICE_ID_LOGITECH_WHEEL:
			return &lg4ff_mode_switch_ext09_dfex;
		case USB_DEVICE_ID_LOGITECH_DFP_WHEEL:
			return &lg4ff_mode_switch_ext09_dfp;
		case USB_DEVICE_ID_LOGITECH_G25_WHEEL:
			return &lg4ff_mode_switch_ext09_g25;
		case USB_DEVICE_ID_LOGITECH_G27_WHEEL:
			return &lg4ff_mode_switch_ext09_g27;
		/* G27 can only be switched to DF-EX, DFP, G25 or its native mode */
		default:
			return NULL;
		}
		break;
	case USB_DEVICE_ID_LOGITECH_G29_WHEEL:
		switch (target_product_id) {
		case USB_DEVICE_ID_LOGITECH_DFP_WHEEL:
			return &lg4ff_mode_switch_ext09_dfp;
		case USB_DEVICE_ID_LOGITECH_DFGT_WHEEL:
			return &lg4ff_mode_switch_ext09_dfgt;
		case USB_DEVICE_ID_LOGITECH_G25_WHEEL:
			return &lg4ff_mode_switch_ext09_g25;
		case USB_DEVICE_ID_LOGITECH_G27_WHEEL:
			return &lg4ff_mode_switch_ext09_g27;
		case USB_DEVICE_ID_LOGITECH_G29_WHEEL:
			return &lg4ff_mode_switch_ext09_g29;
		/* G29 can only be switched to DF-EX, DFP, DFGT, G25, G27 or its native mode */
		default:
			return NULL;
		}
		break;
	case USB_DEVICE_ID_LOGITECH_DFGT_WHEEL:
		switch (target_product_id) {
		case USB_DEVICE_ID_LOGITECH_WHEEL:
			return &lg4ff_mode_switch_ext09_dfex;
		case USB_DEVICE_ID_LOGITECH_DFP_WHEEL:
			return &lg4ff_mode_switch_ext09_dfp;
		case USB_DEVICE_ID_LOGITECH_DFGT_WHEEL:
			return &lg4ff_mode_switch_ext09_dfgt;
		/* DFGT can only be switched to DF-EX, DFP or its native mode */
		default:
			return NULL;
		}
		break;
	/* No other wheels have multiple modes */
	default:
		return NULL;
	}
}

static int lg4ff_switch_compatibility_mode(struct hid_device *hid, const struct lg4ff_compat_mode_switch *s)
{
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;
	unsigned long flags;
	s32 *value;
	u8 i;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Private driver data not found!\n");
		return -EINVAL;
	}

	entry = drv_data->device_props;
	if (!entry) {
		hid_err(hid, "Device properties not found!\n");
		return -EINVAL;
	}
	value = entry->report->field[0]->value;

	spin_lock_irqsave(&entry->report_lock, flags);
	for (i = 0; i < s->cmd_count; i++) {
		u8 j;

		for (j = 0; j < 7; j++)
			value[j] = s->cmd[j + (7*i)];

		hid_hw_request(hid, entry->report, HID_REQ_SET_REPORT);
	}
	spin_unlock_irqrestore(&entry->report_lock, flags);
	hid_hw_wait(hid);
	return 0;
}

static ssize_t lg4ff_alternate_modes_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hid_device *hid = to_hid_device(dev);
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;
	ssize_t count = 0;
	int i;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Private driver data not found!\n");
		return 0;
	}

	entry = drv_data->device_props;
	if (!entry) {
		hid_err(hid, "Device properties not found!\n");
		return 0;
	}

	if (!entry->wdata.real_name) {
		hid_err(hid, "NULL pointer to string\n");
		return 0;
	}

	for (i = 0; i < LG4FF_MODE_MAX_IDX; i++) {
		if (entry->wdata.alternate_modes & BIT(i)) {
			/* Print tag and full name */
			count += scnprintf(buf + count, PAGE_SIZE - count, "%s: %s",
					   lg4ff_alternate_modes[i].tag,
					   !lg4ff_alternate_modes[i].product_id ? entry->wdata.real_name : lg4ff_alternate_modes[i].name);
			if (count >= PAGE_SIZE - 1)
				return count;

			/* Mark the currently active mode with an asterisk */
			if (lg4ff_alternate_modes[i].product_id == entry->wdata.product_id ||
			    (lg4ff_alternate_modes[i].product_id == 0 && entry->wdata.product_id == entry->wdata.real_product_id))
				count += scnprintf(buf + count, PAGE_SIZE - count, " *\n");
			else
				count += scnprintf(buf + count, PAGE_SIZE - count, "\n");

			if (count >= PAGE_SIZE - 1)
				return count;
		}
	}

	return count;
}

static ssize_t lg4ff_alternate_modes_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct hid_device *hid = to_hid_device(dev);
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;
	const struct lg4ff_compat_mode_switch *s;
	u16 target_product_id = 0;
	int i, ret;
	char *lbuf;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Private driver data not found!\n");
		return -EINVAL;
	}

	entry = drv_data->device_props;
	if (!entry) {
		hid_err(hid, "Device properties not found!\n");
		return -EINVAL;
	}

	/* Allow \n at the end of the input parameter */
	lbuf = kasprintf(GFP_KERNEL, "%s", buf);
	if (!lbuf)
		return -ENOMEM;

	i = strlen(lbuf);
	if (lbuf[i-1] == '\n') {
		if (i == 1) {
			kfree(lbuf);
			return -EINVAL;
		}
		lbuf[i-1] = '\0';
	}

	for (i = 0; i < LG4FF_MODE_MAX_IDX; i++) {
		const u16 mode_product_id = lg4ff_alternate_modes[i].product_id;
		const char *tag = lg4ff_alternate_modes[i].tag;

		if (entry->wdata.alternate_modes & BIT(i)) {
			if (!strcmp(tag, lbuf)) {
				if (!mode_product_id)
					target_product_id = entry->wdata.real_product_id;
				else
					target_product_id = mode_product_id;
				break;
			}
		}
	}

	if (i == LG4FF_MODE_MAX_IDX) {
		hid_info(hid, "Requested mode \"%s\" is not supported by the device\n", lbuf);
		kfree(lbuf);
		return -EINVAL;
	}
	kfree(lbuf); /* Not needed anymore */

	if (target_product_id == entry->wdata.product_id) /* Nothing to do */
		return count;

	/* Automatic switching has to be disabled for the switch to DF-EX mode to work correctly */
	if (target_product_id == USB_DEVICE_ID_LOGITECH_WHEEL && !lg4ff_no_autoswitch) {
		hid_info(hid, "\"%s\" cannot be switched to \"DF-EX\" mode. Load the \"hid_logitech\" module with \"lg4ff_no_autoswitch=1\" parameter set and try again\n",
			 entry->wdata.real_name);
		return -EINVAL;
	}

	/* Take care of hardware limitations */
	if ((entry->wdata.real_product_id == USB_DEVICE_ID_LOGITECH_DFP_WHEEL || entry->wdata.real_product_id == USB_DEVICE_ID_LOGITECH_G25_WHEEL) &&
	    entry->wdata.product_id > target_product_id) {
		hid_info(hid, "\"%s\" cannot be switched back into \"%s\" mode\n", entry->wdata.real_name, lg4ff_alternate_modes[i].name);
		return -EINVAL;
	}

	s = lg4ff_get_mode_switch_command(entry->wdata.real_product_id, target_product_id);
	if (!s) {
		hid_err(hid, "Invalid target product ID %X\n", target_product_id);
		return -EINVAL;
	}

	ret = lg4ff_switch_compatibility_mode(hid, s);
	return (ret == 0 ? count : ret);
}
static DEVICE_ATTR(alternate_modes, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH, lg4ff_alternate_modes_show, lg4ff_alternate_modes_store);

/* Export the currently set range of the wheel */
static ssize_t lg4ff_range_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct hid_device *hid = to_hid_device(dev);
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;
	size_t count;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Private driver data not found!\n");
		return 0;
	}

	entry = drv_data->device_props;
	if (!entry) {
		hid_err(hid, "Device properties not found!\n");
		return 0;
	}

	count = scnprintf(buf, PAGE_SIZE, "%u\n", entry->wdata.range);
	return count;
}

/* Set range to user specified value, call appropriate function
 * according to the type of the wheel */
static ssize_t lg4ff_range_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct hid_device *hid = to_hid_device(dev);
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;
	u16 range = simple_strtoul(buf, NULL, 10);

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Private driver data not found!\n");
		return -EINVAL;
	}

	entry = drv_data->device_props;
	if (!entry) {
		hid_err(hid, "Device properties not found!\n");
		return -EINVAL;
	}

	if (range == 0)
		range = entry->wdata.max_range;

	/* Check if the wheel supports range setting
	 * and that the range is within limits for the wheel */
	if (entry->wdata.set_range && range >= entry->wdata.min_range && range <= entry->wdata.max_range) {
		entry->wdata.set_range(hid, range);
		entry->wdata.range = range;
	}

	return count;
}
static DEVICE_ATTR(range, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH, lg4ff_range_show, lg4ff_range_store);

static ssize_t lg4ff_real_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct hid_device *hid = to_hid_device(dev);
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;
	size_t count;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Private driver data not found!\n");
		return 0;
	}

	entry = drv_data->device_props;
	if (!entry) {
		hid_err(hid, "Device properties not found!\n");
		return 0;
	}

	if (!entry->wdata.real_tag || !entry->wdata.real_name) {
		hid_err(hid, "NULL pointer to string\n");
		return 0;
	}

	count = scnprintf(buf, PAGE_SIZE, "%s: %s\n", entry->wdata.real_tag, entry->wdata.real_name);
	return count;
}

static ssize_t lg4ff_real_id_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* Real ID is a read-only value */
	return -EPERM;
}
static DEVICE_ATTR(real_id, S_IRUGO, lg4ff_real_id_show, lg4ff_real_id_store);

#ifdef CONFIG_LEDS_CLASS
static void lg4ff_set_leds(struct hid_device *hid, u8 leds)
{
	struct lg_drv_data *drv_data;
	struct lg4ff_device_entry *entry;
	unsigned long flags;
	s32 *value;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Private driver data not found!\n");
		return;
	}

	entry = drv_data->device_props;
	if (!entry) {
		hid_err(hid, "Device properties not found!\n");
		return;
	}
	value = entry->report->field[0]->value;

	spin_lock_irqsave(&entry->report_lock, flags);
	value[0] = 0xf8;
	value[1] = 0x12;
	value[2] = leds;
	value[3] = 0x00;
	value[4] = 0x00;
	value[5] = 0x00;
	value[6] = 0x00;
	hid_hw_request(hid, entry->report, HID_REQ_SET_REPORT);
	spin_unlock_irqrestore(&entry->report_lock, flags);
}

static void lg4ff_led_set_brightness(struct led_classdev *led_cdev,
			enum led_brightness value)
{
	struct device *dev = led_cdev->dev->parent;
	struct hid_device *hid = to_hid_device(dev);
	struct lg_drv_data *drv_data = hid_get_drvdata(hid);
	struct lg4ff_device_entry *entry;
	int i, state = 0;

	if (!drv_data) {
		hid_err(hid, "Device data not found.");
		return;
	}

	entry = drv_data->device_props;

	if (!entry) {
		hid_err(hid, "Device properties not found.");
		return;
	}

	for (i = 0; i < 5; i++) {
		if (led_cdev != entry->wdata.led[i])
			continue;
		state = (entry->wdata.led_state >> i) & 1;
		if (value == LED_OFF && state) {
			entry->wdata.led_state &= ~(1 << i);
			lg4ff_set_leds(hid, entry->wdata.led_state);
		} else if (value != LED_OFF && !state) {
			entry->wdata.led_state |= 1 << i;
			lg4ff_set_leds(hid, entry->wdata.led_state);
		}
		break;
	}
}

static enum led_brightness lg4ff_led_get_brightness(struct led_classdev *led_cdev)
{
	struct device *dev = led_cdev->dev->parent;
	struct hid_device *hid = to_hid_device(dev);
	struct lg_drv_data *drv_data = hid_get_drvdata(hid);
	struct lg4ff_device_entry *entry;
	int i, value = 0;

	if (!drv_data) {
		hid_err(hid, "Device data not found.");
		return LED_OFF;
	}

	entry = drv_data->device_props;

	if (!entry) {
		hid_err(hid, "Device properties not found.");
		return LED_OFF;
	}

	for (i = 0; i < 5; i++)
		if (led_cdev == entry->wdata.led[i]) {
			value = (entry->wdata.led_state >> i) & 1;
			break;
		}

	return value ? LED_FULL : LED_OFF;
}
#endif

static u16 lg4ff_identify_multimode_wheel(struct hid_device *hid, const u16 reported_product_id, const u16 bcdDevice)
{
	u32 current_mode;
	int i;

	/* identify current mode from USB PID */
	for (i = 1; i < ARRAY_SIZE(lg4ff_alternate_modes); i++) {
		dbg_hid("Testing whether PID is %X\n", lg4ff_alternate_modes[i].product_id);
		if (reported_product_id == lg4ff_alternate_modes[i].product_id)
			break;
	}

	if (i == ARRAY_SIZE(lg4ff_alternate_modes))
		return 0;

	current_mode = BIT(i);

	for (i = 0; i < ARRAY_SIZE(lg4ff_main_checklist); i++) {
		const u16 mask = lg4ff_main_checklist[i]->mask;
		const u16 result = lg4ff_main_checklist[i]->result;
		const u16 real_product_id = lg4ff_main_checklist[i]->real_product_id;

		if ((current_mode & lg4ff_main_checklist[i]->modes) && \
				(bcdDevice & mask) == result) {
			dbg_hid("Found wheel with real PID %X whose reported PID is %X\n", real_product_id, reported_product_id);
			return real_product_id;
		}
	}

	/* No match found. This is either Driving Force or an unknown
	 * wheel model, do not touch it */
	dbg_hid("Wheel with bcdDevice %X was not recognized as multimode wheel, leaving in its current mode\n", bcdDevice);
	return 0;
}

static int lg4ff_handle_multimode_wheel(struct hid_device *hid, u16 *real_product_id, const u16 bcdDevice)
{
	const u16 reported_product_id = hid->product;
	int ret;

	*real_product_id = lg4ff_identify_multimode_wheel(hid, reported_product_id, bcdDevice);
	/* Probed wheel is not a multimode wheel */
	if (!*real_product_id) {
		*real_product_id = reported_product_id;
		dbg_hid("Wheel is not a multimode wheel\n");
		return LG4FF_MMODE_NOT_MULTIMODE;
	}

	/* Switch from "Driving Force" mode to native mode automatically.
	 * Otherwise keep the wheel in its current mode */
	if (reported_product_id == USB_DEVICE_ID_LOGITECH_WHEEL &&
	    reported_product_id != *real_product_id &&
	    !lg4ff_no_autoswitch) {
		const struct lg4ff_compat_mode_switch *s = lg4ff_get_mode_switch_command(*real_product_id, *real_product_id);

		if (!s) {
			hid_err(hid, "Invalid product id %X\n", *real_product_id);
			return LG4FF_MMODE_NOT_MULTIMODE;
		}

		ret = lg4ff_switch_compatibility_mode(hid, s);
		if (ret) {
			/* Wheel could not have been switched to native mode,
			 * leave it in "Driving Force" mode and continue */
			hid_err(hid, "Unable to switch wheel mode, errno %d\n", ret);
			return LG4FF_MMODE_IS_MULTIMODE;
		}
		return LG4FF_MMODE_SWITCHED;
	}

	return LG4FF_MMODE_IS_MULTIMODE;
}


int lg4ff_init(struct hid_device *hid)
{
	struct hid_input *hidinput = list_entry(hid->inputs.next, struct hid_input, list);
	struct input_dev *dev = hidinput->input;
	struct list_head *report_list = &hid->report_enum[HID_OUTPUT_REPORT].report_list;
	struct hid_report *report = list_entry(report_list->next, struct hid_report, list);
	const struct usb_device_descriptor *udesc = &(hid_to_usb_dev(hid)->descriptor);
	const u16 bcdDevice = le16_to_cpu(udesc->bcdDevice);
	const struct lg4ff_multimode_wheel *mmode_wheel = NULL;
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;
	int error, i, j;
	int mmode_ret, mmode_idx = -1;
	u16 real_product_id;

	/* Check that the report looks ok */
	if (!hid_validate_values(hid, HID_OUTPUT_REPORT, 0, 0, 7))
		return -1;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Cannot add device, private driver data not allocated\n");
		return -1;
	}
	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;
	spin_lock_init(&entry->report_lock);
	entry->report = report;
	drv_data->device_props = entry;

	/* Check if a multimode wheel has been connected and
	 * handle it appropriately */
	mmode_ret = lg4ff_handle_multimode_wheel(hid, &real_product_id, bcdDevice);

	/* Wheel has been told to switch to native mode. There is no point in going on
	 * with the initialization as the wheel will do a USB reset when it switches mode
	 */
	if (mmode_ret == LG4FF_MMODE_SWITCHED)
		return 0;
	else if (mmode_ret < 0) {
		hid_err(hid, "Unable to switch device mode during initialization, errno %d\n", mmode_ret);
		error = mmode_ret;
		goto err_init;
	}

	/* Check what wheel has been connected */
	for (i = 0; i < ARRAY_SIZE(lg4ff_devices); i++) {
		if (hid->product == lg4ff_devices[i].product_id) {
			dbg_hid("Found compatible device, product ID %04X\n", lg4ff_devices[i].product_id);
			break;
		}
	}

	if (i == ARRAY_SIZE(lg4ff_devices)) {
		hid_err(hid, "This device is flagged to be handled by the lg4ff module but this module does not know how to handle it. "
			     "Please report this as a bug to LKML, Simon Wood <simon@mungewell.org> or "
			     "Michal Maly <madcatxster@devoid-pointer.net>\n");
		error = -1;
		goto err_init;
	}

	if (mmode_ret == LG4FF_MMODE_IS_MULTIMODE) {
		for (mmode_idx = 0; mmode_idx < ARRAY_SIZE(lg4ff_multimode_wheels); mmode_idx++) {
			if (real_product_id == lg4ff_multimode_wheels[mmode_idx].product_id)
				break;
		}

		if (mmode_idx == ARRAY_SIZE(lg4ff_multimode_wheels)) {
			hid_err(hid, "Device product ID %X is not listed as a multimode wheel", real_product_id);
			error = -1;
			goto err_init;
		}
	}

	/* Initialize device properties */
	if (mmode_ret == LG4FF_MMODE_IS_MULTIMODE) {
		BUG_ON(mmode_idx == -1);
		mmode_wheel = &lg4ff_multimode_wheels[mmode_idx];
	}
	lg4ff_init_wheel_data(&entry->wdata, &lg4ff_devices[i], mmode_wheel, real_product_id);

	/* initialize klgd with a single plugin*/
	error = klgd_init(&entry->wdata.klgd, hid, lg4ff_klgd_callback, 1);
	if (error) {
		printk(KERN_ERR "Cannot initialize KLGD\n");
		goto err_init;
	}

	/* Set supported force feedback capabilities */
	for (j = 0; lg4ff_devices[i].ff_effects[j] >= 0; j++)
		set_bit(lg4ff_devices[i].ff_effects[j], dev->ffbit);

	/* initialize the klgd force feedback plugin */
	error = ffpl_init_plugin(&entry->wdata.ff_plugin, dev, EFFECT_COUNT,
	                         FFPL_HAS_EMP_TO_SRT | FFPL_HAS_SRT_TO_EMP | FFPL_REPLACE_UPLOADED | FFPL_REPLACE_STARTED |
				 FFPL_MEMLESS_CONSTANT | FFPL_MEMLESS_PERIODIC | FFPL_MEMLESS_RAMP | FFPL_TIMING_CONDITION,
	                         lg4ff_control, NULL);
	if (error) {
		printk(KERN_ERR "KLGDFF: Cannot init plugin\n");
		goto err_init;
	}

	error = klgd_register_plugin(&entry->wdata.klgd, 0, entry->wdata.ff_plugin, true);
	if (error) {
		printk(KERN_ERR "KLGDFF: Cannot register plugin\n");
		goto err_init;
	}

	/* Formula Force EX expects different autocentering command */
	if ((bcdDevice >> 8) == LG4FF_FFEX_REV_MAJ && (bcdDevice & 0xff) == LG4FF_FFEX_REV_MIN)
		entry->wdata.ffex_autocenter = true;
	else
		entry->wdata.ffex_autocenter = false;

	/* Create sysfs interface */
	error = device_create_file(&hid->dev, &dev_attr_range);
	if (error)
		hid_warn(hid, "Unable to create sysfs interface for \"range\", errno %d\n", error);
	if (mmode_ret == LG4FF_MMODE_IS_MULTIMODE) {
		error = device_create_file(&hid->dev, &dev_attr_real_id);
		if (error)
			hid_warn(hid, "Unable to create sysfs interface for \"real_id\", errno %d\n", error);
		error = device_create_file(&hid->dev, &dev_attr_alternate_modes);
		if (error)
			hid_warn(hid, "Unable to create sysfs interface for \"alternate_modes\", errno %d\n", error);
	}
	dbg_hid("sysfs interface created\n");

	/* Set the maximum range to start with */
	entry->wdata.range = entry->wdata.max_range;
	if (entry->wdata.set_range)
		entry->wdata.set_range(hid, entry->wdata.range);

#ifdef CONFIG_LEDS_CLASS
	/* register led subsystem - G27/G29 only */
	entry->wdata.led_state = 0;
	for (j = 0; j < 5; j++)
		entry->wdata.led[j] = NULL;

	if (lg4ff_devices[i].product_id == USB_DEVICE_ID_LOGITECH_G27_WHEEL ||
			lg4ff_devices[i].product_id == USB_DEVICE_ID_LOGITECH_G29_WHEEL) {
		struct led_classdev *led;
		size_t name_sz;
		char *name;

		lg4ff_set_leds(hid, 0);

		name_sz = strlen(dev_name(&hid->dev)) + 8;

		for (j = 0; j < 5; j++) {
			led = kzalloc(sizeof(struct led_classdev)+name_sz, GFP_KERNEL);
			if (!led) {
				hid_err(hid, "can't allocate memory for LED %d\n", j);
				goto err_leds;
			}

			name = (void *)(&led[1]);
			snprintf(name, name_sz, "%s::RPM%d", dev_name(&hid->dev), j+1);
			led->name = name;
			led->brightness = 0;
			led->max_brightness = 1;
			led->brightness_get = lg4ff_led_get_brightness;
			led->brightness_set = lg4ff_led_set_brightness;

			entry->wdata.led[j] = led;
			error = led_classdev_register(&hid->dev, led);

			if (error) {
				hid_err(hid, "failed to register LED %d. Aborting.\n", j);
err_leds:
				/* Deregister LEDs (if any) */
				for (j = 0; j < 5; j++) {
					led = entry->wdata.led[j];
					entry->wdata.led[j] = NULL;
					if (!led)
						continue;
					led_classdev_unregister(led);
					kfree(led);
				}
				goto out;	/* Let the driver continue without LEDs */
			}
		}
	}
out:
#endif
	hid_info(hid, "Force feedback support for Logitech Gaming Wheels\n");
	return 0;

err_init:
	drv_data->device_props = NULL;
	kfree(entry);
	return error;
}

int lg4ff_deinit(struct hid_device *hid)
{
	struct lg4ff_device_entry *entry;
	struct lg_drv_data *drv_data;

	drv_data = hid_get_drvdata(hid);
	if (!drv_data) {
		hid_err(hid, "Error while deinitializing device, no private driver data.\n");
		return -1;
	}
	entry = drv_data->device_props;
	if (!entry)
		goto out; /* Nothing more to do */

	/* Multimode devices will have at least the "MODE_NATIVE" bit set */
	if (entry->wdata.alternate_modes) {
		device_remove_file(&hid->dev, &dev_attr_real_id);
		device_remove_file(&hid->dev, &dev_attr_alternate_modes);
	}

	device_remove_file(&hid->dev, &dev_attr_range);
#ifdef CONFIG_LEDS_CLASS
	{
		int j;
		struct led_classdev *led;

		/* Deregister LEDs (if any) */
		for (j = 0; j < 5; j++) {

			led = entry->wdata.led[j];
			entry->wdata.led[j] = NULL;
			if (!led)
				continue;
			led_classdev_unregister(led);
			kfree(led);
		}
	}
#endif
	hid_hw_stop(hid);
	drv_data->device_props = NULL;

	klgd_deinit(&entry->wdata.klgd);

	kfree(entry);
out:
	dbg_hid("Device successfully unregistered\n");
	return 0;
}

