#include "klgd_ff_plugin_p.h"
#include <linux/slab.h>
#include <linux/fixp-arith.h>
#include <linux/jiffies.h>
#include <linux/module.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michal \"MadCatX\" Maly");
MODULE_DESCRIPTION("KLGD-FF Module");

#define FRAC_16 15
#define RECALC_DELTA_T_MSEC 20

/* Combining handlers */
#define FFPL_HANDLER_CF BIT(0)
#define FFPL_HANDLER_RUMBLE BIT(1)
#define FFPL_HANDLER_ANY (BIT(0) | BIT(1))
/* Macros to set which combining handlers have to be called */
#define NEEDS_UPDATE_SET(etype) \
	do { \
		switch (etype) { \
		case FF_CONSTANT: \
		case FF_RAMP: \
			needs_update_cf = true; \
			break; \
		case FF_RUMBLE: \
			if (priv->memless_rumble_emul) \
				needs_update_cf = true; \
			else \
				needs_update_rumble = true; \
			break; \
		case FF_PERIODIC: \
			if (priv->memless_periodic_emul) \
				needs_update_rumble = true; \
			else \
				needs_update_cf = true; \
			break; \
		default: \
			break; \
		} \
	} while (0);

#define ACTIVE_EFFECTS_INC(etype) \
	do { \
		switch (etype) { \
		case FF_CONSTANT: \
		case FF_RAMP: \
			active_effects_cf++; \
			break; \
		case FF_RUMBLE: \
			if (priv->memless_rumble_emul) \
				active_effects_cf++; \
			else \
				active_effects_rumble++; \
			break; \
		case FF_PERIODIC: \
			if (priv->memless_periodic_emul) \
				active_effects_rumble++; \
			else \
				active_effects_cf++; \
			break; \
		default: \
			break; \
		} \
	} while (0);

static int ffpl_handle_state_change(struct klgd_plugin_private *priv, struct klgd_command_stream *s, struct ffpl_effect *eff,
				    const unsigned long now);
static bool ffpl_needs_replacing(const struct ff_effect *ac_eff, const struct ff_effect *la_eff);

void ffpl_lvl_dir_to_x_y(const s32 level, const u16 direction, s32 *x, s32 *y)
{
	const int degrees = direction * 360 / 0xFFFF;

	printk(KERN_NOTICE "KLGDFF: DIR_TO_DEGREES > Dir: %u, Deg: %u\n", direction, degrees);
	*x = (level * -fixp_sin16(degrees)) >> FRAC_16;
	*y = (level * -fixp_cos16(degrees)) >> FRAC_16;
}
EXPORT_SYMBOL_GPL(ffpl_lvl_dir_to_x_y);

inline static bool ffpl_process_memless(const struct klgd_plugin_private *priv, const struct ff_effect *eff,
					const int handler)
{
	bool ret = false;

	switch (eff->type) {
	case FF_CONSTANT:
		return priv->memless_constant;
	case FF_PERIODIC:
		if (FFPL_HANDLER_CF & handler)
			ret |= priv->memless_periodic;
		if (FFPL_HANDLER_RUMBLE & handler)
			ret |= priv->memless_periodic_emul;
		break;
	case FF_RAMP:
		return priv->memless_ramp;
	case FF_RUMBLE:
		if (FFPL_HANDLER_CF & handler)
			ret |= priv->memless_rumble_emul;
		if (FFPL_HANDLER_RUMBLE & handler)
			ret |= priv->memless_rumble;
		break;
	default:
		return false;
	}

	return ret;
}

inline static bool ffpl_handle_timing(const struct klgd_plugin_private *priv, const struct ff_effect *eff)
{
	if (priv->timing_condition) {
		switch (eff->type) {
		case FF_DAMPER:
		case FF_FRICTION:
		case FF_INERTIA:
		case FF_SPRING:
			return true;
		}
	}

	return ffpl_process_memless(priv, eff, FFPL_HANDLER_ANY);
}

static const struct ff_envelope * ffpl_get_envelope(const struct ff_effect *ueff)
{
	switch (ueff->type) {
	case FF_CONSTANT:
		return &ueff->u.constant.envelope;
	case FF_PERIODIC:
		return &ueff->u.periodic.envelope;
	case FF_RAMP:
		return &ueff->u.ramp.envelope;
	default:
		return NULL;
	}
}

static bool ffpl_is_effect_valid(const struct ff_effect *ueff)
{
	const u16 length = ueff->replay.length;
	const struct ff_envelope *env = ffpl_get_envelope(ueff);

	/* Periodic effects must have a non-zero period */
	if (ueff->type == FF_PERIODIC) {
		if (!ueff->u.periodic.period)
			return false;
	}
	/* Ramp effects cannot be infinite */
	if (ueff->type == FF_RAMP && !length)
		return false;

	if (env) {
		int fade_from;

		/* Infinite effects cannot fade */
		if (!length && env->fade_length > 0)
			return false;

		/* Fade length cannot be greater than effect duration */
		fade_from = (int)length - (int)env->fade_length;
		if (fade_from < 0)
			return false;
		/* Envelope cannot start fading before it finishes attacking */
		if (fade_from < env->attack_length && fade_from > 0)
			return false;
	}

	return true;
}

static u16 ffpl_atan_int_octet(const u16 x, const u16 y)
{
    u32 result;

    if (!y)
        return 0x0000;

    /* 3rd order polynomial approximation beween 0 <= y/x <= 1 */
    result = y * 0x02e2 / x;
    result = y * (0x05dc + result) / x;
    result = y * (0x28be - result) / x;

    return result;
}

static u16 ffpl_atan_int_quarter(const u16 x, const u16 y)
{
    if (x == y)
        return 0x2000;
    else if (x > y)
        return ffpl_atan_int_octet(x, y);
    else
        return 0x4000 - ffpl_atan_int_octet(y, x);
}


static void ffpl_x_y_to_lvl_dir(const s32 x, const s32 y, s16 *level, u16 *direction)
{
	u16 angle;
	unsigned long pwr;
	u32 _x = abs(x);
	u32 _y = abs(y);

	/* It is necessary to make sure that neither of the coordinates exceeds 0xffff.
	 */
	if (abs(x) > 0xffff || abs(y) > 0xffff) {
		u32 div;
		u32 divx = _x / 0xfffe;
		u32 divy = _y / 0xfffe;

		div = (divx > divy) ? divx : divy;

		_x /= div;
		_y /= div;

		if (_x > 0xffff || _y > 0xffff) {
			_x >>= 1;
			_y >>= 1;
		}

	}
	angle = (!_x && !_y) ? 0x4000 : ffpl_atan_int_quarter(_x, _y);


	/* 1st quadrant */
	if (x >= 0 && y >= 0)
		*direction = 0xC000 - angle;
	/* 2nd quadrant */
	else if (x < 0 && y >= 0)
		*direction = 0x4000 + angle;
	/* 3rd quadrant */
	else if (x < 0 && y < 0)
		*direction = 0x4000 - angle;
	/* 4th quadrant */
	else if (x > 0 && y < 0)
		*direction = 0xC000 + angle;
	else
		*direction = 0x0000;

	if (abs(x) > 0x7fff || abs(y) > 0x7fff) {
		*level = 0x7fff;
		return;
	}

	pwr = int_sqrt(x * x + y * y);
	*level = (pwr > 0x7fff) ? 0x7fff : pwr;
}

static s32 ffpl_apply_envelope(const struct ffpl_effect *eff, const unsigned long now)
{
	const struct ff_effect *ueff = &eff->active;
	const struct ff_envelope *env = ffpl_get_envelope(ueff);
	s32 abs_level;
	s32 level;
	unsigned long atk_end;
	unsigned long fade_begin;

	if (!env)
		return 0;

	switch (ueff->type) {
	case FF_CONSTANT:
		level = ueff->u.constant.level;
		break;
	case FF_PERIODIC:
		level = ueff->u.periodic.magnitude;
		break;
	case FF_RAMP:
		printk(KERN_NOTICE "KLGDFF: FF_RAMP envelope is not handled yet\n");
		return 0;
	default:
		WARN(true, "KLGDFF: Invalid effect passed to envelope calculation. This cannot happen!\n");
		return 0;
	}
	abs_level = abs(level);
	atk_end = eff->start_at + msecs_to_jiffies(env->attack_length);
	fade_begin = eff->stop_at - msecs_to_jiffies(env->fade_length);

	/* Effect has an envelope with nonzero attack time */
	if (env->attack_length && time_before(now, atk_end)) {
		const s32 clength = jiffies_to_msecs(now - eff->start_at);
		const s32 alength = env->attack_length;
		const s32 dlevel = (abs_level - env->attack_level) * clength / alength;
		return level < 0 ? -(dlevel + env->attack_level) :
				    (dlevel + env->attack_level);
	} else if (env->fade_length && time_before_eq(fade_begin, now)) {
		const s32 clength = jiffies_to_msecs(now - fade_begin);
		const s32 flength = env->fade_length;
		const s32 dlevel = (env->fade_level - abs_level) * clength / flength;
		return level < 0 ? -(dlevel + abs_level) : (dlevel + abs_level);
	}

	return level;
}

static void ffpl_constant_to_x_y(const struct ffpl_effect *eff, s32 *x, s32 *y, const unsigned long now)
{
	const struct ff_effect *ueff = &eff->active;
	const s32 level = ffpl_apply_envelope(eff, now);

	ffpl_lvl_dir_to_x_y(level, ueff->direction, x, y);
}

static void ffpl_periodic_to_x_y(struct ffpl_effect *eff, s32 *x, s32 *y, const unsigned long now)
{
	const struct ff_effect *ueff = &eff->active;
	const u16 period = ueff->u.periodic.period;
	const s16 offset = ueff->u.periodic.offset;
	const s32 level = ffpl_apply_envelope(eff, now);
	s32 new = 0;
	u16 t;

	eff->playback_time += jiffies_to_msecs(now - eff->updated_at);
	eff->playback_time %= period;
	t = (eff->playback_time + ueff->u.periodic.phase) % period;

	printk(KERN_NOTICE "KLGDFF: PT: %u, t: %u, dt: %lu\n", eff->playback_time, t, now - eff->updated_at);

	switch (ueff->u.periodic.waveform) {
	case FF_SINE:
	{
		u16 degrees = (360 * t) / period;
		new = ((level * fixp_sin16(degrees)) >> FRAC_16) + offset;
		break;
	}
	case FF_SQUARE:
	{
		u16 degrees = (360 * t) / period;
		new = level * (degrees < 180 ? 1 : -1) + offset;
		break;
	}
	case FF_SAW_UP:
		new = 2 * level * t / period - level + offset;
		break;
	case FF_SAW_DOWN:
		new = level - 2 * level * t / period + offset;
		break;
	case FF_TRIANGLE:
	{
		new = (2 * abs(level - (2 * level * t) / period));
		new = new - abs(level) + offset;
		break;
	}
	case FF_CUSTOM:
		pr_err("Custom waveform is not handled by this driver\n");
		break;
	default:
		pr_err("Invalid waveform\n");
		break;
	}

	/* Ensure that the offset did not make the value exceed s16 range */
	new = clamp(new, -0x7fff, 0x7fff);
	printk(KERN_NOTICE "KLGDFF: Periodic %d\n", new);
	ffpl_lvl_dir_to_x_y(new, ueff->direction, x, y);
}

static void ffpl_ramp_to_x_y(struct ffpl_effect *eff, s32 *x, s32 *y, const unsigned long now)
{
	const struct ff_effect *ueff = &eff->active;
	const struct ff_envelope *env = ffpl_get_envelope(ueff);
	const u16 length = ueff->replay.length;
	const s16 mean = (ueff->u.ramp.start_level + ueff->u.ramp.end_level) / 2;
	const u16 t = jiffies_to_msecs(now - eff->start_at);
	const unsigned long attack_stop = eff->start_at + env->attack_length;
	const unsigned long fade_begin = eff->stop_at - env->fade_length;
	s32 start = ueff->u.ramp.start_level;
	s32 end = ueff->u.ramp.end_level;
	s32 new;

	if (env->attack_length && time_before(now, attack_stop)) {
		const s32 clength = jiffies_to_msecs(now - eff->start_at);
		const s32 alength = env->attack_length;
		s32 attack_level;
		if (end > start)
			attack_level = mean - env->attack_level;
		else
			attack_level = mean + env->attack_level;
		start = (start - attack_level) * clength / alength + attack_level;
	} else if (env->fade_length && time_before_eq(fade_begin, now)) {
		const s32 clength = jiffies_to_msecs(now - fade_begin);
		const s32 flength = env->fade_length;
		s32 fade_level;
		if (end > start)
			fade_level = mean + env->fade_level;
		else
			fade_level = mean - env->fade_level;
		end = (fade_level - end) * clength / flength + end;
	}

	new = ((end - start) * t) / length + start;
	new = clamp(new, -0x7fff, 0x7fff);
	ffpl_lvl_dir_to_x_y(new, ueff->direction, x, y);
}

/*
 * Emulate FF_RUMBLE effects through FF_CONSTANT
 */
static void ffpl_rumble_to_x_y(struct ffpl_effect *eff, s32 *x, s32 *y, const unsigned long now)
{
	bool direction_up;
	bool direction_left;
	const unsigned long update_rate = msecs_to_jiffies(RECALC_DELTA_T_MSEC);
	const struct ff_effect *ueff = &eff->active;
	const u16 strong = ueff->u.rumble.strong_magnitude;
	const u16 weak = ueff->u.rumble.weak_magnitude;
	/* To calculate 't', we pretend that mlnxeff->begin_at == 0, thus t == now.  */
	/* This will synchronise all simultaneously playing emul rumble effects,     */
	/* otherwise non-deterministic phase-inversions could occur depending on     */
	/* upload time, which could lead to undesired cancellation of these effects. */
	const unsigned long t = now % (4UL * update_rate);
	s32 level = 0;

	if (strong)
		level += (strong / 4) * (t < 2UL * update_rate ? 1 : -1);
	if (weak)
		level += (weak / 4) * (t < 2UL * update_rate ?
					(t < 1UL * update_rate ? 1 : -1) :
					(t < 3UL * update_rate ? 1 : -1));
	direction_up = (ueff->direction > 0x3fffU && ueff->direction <= 0xbfffU);
	direction_left = (ueff->direction <= 0x7fffU);

	*x += direction_left ? -level : level;
	*y += direction_up ? -level : level;
}

static void ffpl_recalc_combined_cf(struct klgd_plugin_private *priv, const unsigned long now)
{
	size_t idx;
	struct ff_effect *cb_latest = &priv->combined_effect_cf.latest;
	s32 x = 0;
	s32 y = 0;

	for (idx = 0; idx < priv->effect_count; idx++) {
		struct ffpl_effect *eff = &priv->effects[idx];
		struct ff_effect *ueff = &eff->active;
		s32 _x;
		s32 _y;

		if (!ffpl_process_memless(priv, ueff, FFPL_HANDLER_CF))
			continue;
		if (eff->state != FFPL_STARTED)
			continue;

		switch (ueff->type) {
		case FF_CONSTANT:
			ffpl_constant_to_x_y(eff, &_x, &_y, now);
			break;
		case FF_PERIODIC:
			ffpl_periodic_to_x_y(eff, &_x, &_y, now);
			break;
		case FF_RAMP:
			ffpl_ramp_to_x_y(eff, &_x, &_y, now);
			break;
		case FF_RUMBLE:
			ffpl_rumble_to_x_y(eff, &_x, &_y, now);
			break;
		default:
			continue;
			break;
		}

		eff->updated_at = now;
		x += _x;
		y += _y;
	}

	ffpl_x_y_to_lvl_dir(x, y, &cb_latest->u.constant.level, &cb_latest->direction);
	cb_latest->type = FF_CONSTANT;
	printk(KERN_NOTICE "KLGDFF: Resulting combined CF effect > x: %d, y: %d, level: %d, direction: %u\n", x, y, cb_latest->u.constant.level,
	       cb_latest->direction);
}

static u16 ffpl_set_rumble_direction(const u16 strong_dir, const u16 weak_dir)
{
	u16 dir = 0;

	if (strong_dir >= 0x8000)
		dir |= FFPL_RUMBLE_STRONG_RIGHT;
	if (weak_dir >= 0x8000)
		dir |= FFPL_RUMBLE_WEAK_RIGHT;
	if (strong_dir >= 0x4000 && strong_dir < 0xc000)
		dir |= FFPL_RUMBLE_STRONG_UP;
	if (weak_dir >= 0x4000 && weak_dir < 0xc000)
		dir |= FFPL_RUMBLE_WEAK_UP;

	return dir;
}

static void ffpl_recalc_combined_rumble(struct klgd_plugin_private *priv, const unsigned long now)
{
	size_t idx;
	struct ff_effect *cb_latest = &priv->combined_effect_rumble.latest;
	s32 strong_x = 0;
	s32 strong_y = 0;
	s32 weak_x = 0;
	s32 weak_y = 0;
	/* Resulting overall values expressed as direction and magnitude */
	s16 strong_mag;
	u16 strong_dir;
	s16 weak_mag;
	u16 weak_dir;

	for (idx = 0; idx < priv->effect_count; idx++) {
		struct ffpl_effect *eff = &priv->effects[idx];
		struct ff_effect *ueff = &eff->active;
		s32 _strong_x;
		s32 _strong_y;
		s32 _weak_x;
		s32 _weak_y;

		if (!ffpl_process_memless(priv, ueff, FFPL_HANDLER_RUMBLE))
			continue;
		if (eff->state != FFPL_STARTED)
			continue;

		switch (ueff->type) {
		case FF_RUMBLE:
			ffpl_lvl_dir_to_x_y(ueff->u.rumble.strong_magnitude / 2, ueff->direction, &_strong_x, &_strong_y);
			ffpl_lvl_dir_to_x_y(ueff->u.rumble.weak_magnitude / 2, ueff->direction, &_weak_x, &_weak_y);
			break;
		case FF_PERIODIC:
		{
			u32 new_level = abs(ffpl_apply_envelope(eff, now));
			ffpl_lvl_dir_to_x_y(new_level, ueff->direction, &_strong_x, &_strong_y);
			ffpl_lvl_dir_to_x_y(new_level, ueff->direction, &_weak_x, &_weak_y);
			break;
		}
		default:
			continue;
			break;
		}
		strong_x += _strong_x;
		strong_y += _strong_y;
		weak_x += _weak_x;
		weak_y += _weak_y;
	}

	ffpl_x_y_to_lvl_dir(strong_x, strong_y, &strong_mag, &strong_dir);
	ffpl_x_y_to_lvl_dir(weak_x, weak_y, &weak_mag, &weak_dir);
	cb_latest->direction = ffpl_set_rumble_direction(strong_dir, weak_dir);
	cb_latest->u.rumble.strong_magnitude = (u16)strong_mag * 2;
	cb_latest->u.rumble.weak_magnitude = (u16)weak_mag * 2;
	cb_latest->type = FF_RUMBLE;
}

static int ffpl_erase_effect(struct klgd_plugin_private *priv, struct klgd_command_stream *s, struct ffpl_effect *eff)
{
	if (eff->uploaded_to_device) {
		struct input_dev *dev = priv->dev;
		union ffpl_control_data data;
		int ret;

		data.effects.cur = &eff->active;
		data.effects.old = NULL;
		ret = priv->control(dev, s, FFPL_UPL_TO_EMP, data, priv->user);
		if (ret)
			return ret;
	}

	eff->state = FFPL_EMPTY;
	eff->uploaded_to_device = false;
	return 0;
}

static int ffpl_replace_effect(struct klgd_plugin_private *priv, struct klgd_command_stream *s, struct ffpl_effect *eff,
			       const enum ffpl_control_command cmd)
{
	struct input_dev *dev = priv->dev;
	union ffpl_control_data data;
	int ret;

	data.effects.cur = &eff->latest;
	data.effects.old = &eff->active;
	data.effects.repeat = eff->repeat;
	ret = priv->control(dev, s, cmd, data, priv->user);
	if (!ret) {
		eff->active = eff->latest;
		eff->state = (cmd == FFPL_OWR_TO_UPL) ? FFPL_UPLOADED : FFPL_STARTED;
		eff->replace = false;
		eff->change = FFPL_DONT_TOUCH;
		return 0;
	}
	return ret;
}

static int ffpl_start_effect(struct klgd_plugin_private *priv, struct klgd_command_stream *s, struct ffpl_effect *eff)
{
	struct input_dev *dev = priv->dev;
	union ffpl_control_data data;
	int ret;
	enum ffpl_control_command cmd;

	data.effects.old = NULL;
	data.effects.repeat = eff->repeat;
	if (priv->upload_when_started && eff->state == FFPL_UPLOADED) {
		data.effects.cur = &eff->active;
		if (eff->uploaded_to_device)
			cmd = FFPL_UPL_TO_SRT;
		else
			cmd = FFPL_EMP_TO_SRT;

		ret = priv->control(dev, s, cmd, data, priv->user);
		if (ret)
			return ret;
	} else {
		/* This can happen only if device supports "upload and start" */
		if (eff->state == FFPL_EMPTY) {
			data.effects.cur = &eff->latest;
			cmd = FFPL_EMP_TO_SRT;
		} else {
			data.effects.cur = &eff->active;
			cmd = FFPL_UPL_TO_SRT;
		}

		ret = priv->control(dev, s, cmd, data, priv->user);
		if (ret)
			return ret;
		if (cmd == FFPL_EMP_TO_SRT)
			eff->active = eff->latest;
	}

	eff->uploaded_to_device = true; /* Needed of devices that support "upload and start" but don't use "upload when started" */
	eff->state = FFPL_STARTED;
	return 0;
}

static int ffpl_stop_effect(struct klgd_plugin_private *priv, struct klgd_command_stream *s, struct ffpl_effect *eff)
{
	struct input_dev *dev = priv->dev;
	union ffpl_control_data data;
	int ret;
	enum ffpl_control_command cmd;

	data.effects.cur = &eff->active;
	data.effects.old = NULL;
	if (priv->erase_when_stopped || (priv->has_srt_to_emp && eff->change == FFPL_TO_ERASE))
		cmd = FFPL_SRT_TO_EMP;
	else
		cmd = FFPL_SRT_TO_UPL;

	ret = priv->control(dev, s, cmd, data, priv->user);
	if (ret)
		return ret;
	if (cmd == FFPL_SRT_TO_EMP)
		eff->uploaded_to_device = false;
	eff->state = FFPL_UPLOADED;


	/* Report back that the effect has stopped */
	if (eff->trigger == FFPL_TRIG_STOP)
		input_report_ff_status(dev, eff->active.id, FF_STATUS_STOPPED);

	return 0;
}

static int ffpl_update_effect(struct klgd_plugin_private *priv, struct klgd_command_stream *s, struct ffpl_effect *eff)
{
	struct input_dev *dev = priv->dev;
	union ffpl_control_data data;
	int ret;

	if (!eff->uploaded_to_device)
		return ffpl_start_effect(priv, s, eff);

	data.effects.cur = &eff->latest;
	data.effects.old = NULL;
	ret = priv->control(dev, s, FFPL_SRT_TO_UDT, data, priv->user);
	if (ret)
		return ret;
	eff->active = eff->latest;
	return 0;
}

static int ffpl_upload_effect(struct klgd_plugin_private *priv, struct klgd_command_stream *s, struct ffpl_effect *eff)
{
	if (!priv->upload_when_started) {
		struct input_dev *dev = priv->dev;
		union ffpl_control_data data;
		int ret;

		data.effects.cur = &eff->latest;
		data.effects.old = NULL;
		ret = priv->control(dev, s, FFPL_EMP_TO_UPL, data, priv->user);
		if (ret)
			return ret;
		eff->uploaded_to_device = true;
	}

	eff->state = FFPL_UPLOADED;
	eff->active = eff->latest;
	return 0;
}

static int ffpl_set_autocenter(struct klgd_plugin_private *priv, struct klgd_command_stream *s)
{
	union ffpl_control_data data;

	data.autocenter = priv->autocenter;
	return priv->control(priv->dev, s, FFPL_SET_AUTOCENTER, data, priv->user);
}

static int ffpl_set_gain(struct klgd_plugin_private *priv, struct klgd_command_stream *s)
{
	union ffpl_control_data data;

	data.gain = priv->gain;
	return priv->control(priv->dev, s, FFPL_SET_GAIN, data, priv->user);
}

static void ffpl_calculate_trip_times(struct ffpl_effect *eff, const unsigned long now)
{
	const struct ff_effect *ueff = &eff->latest;

	eff->start_at = now + msecs_to_jiffies(ueff->replay.delay);
	eff->updated_at = eff->start_at;
	eff->touch_at = eff->start_at;


	if (ueff->replay.length)
		eff->stop_at = eff->start_at + msecs_to_jiffies(ueff->replay.length);

}

static void ffpl_update_trip_times(struct ffpl_effect *eff, const unsigned long now)
{
	const struct ff_effect *ueff_la = &eff->latest;
	const struct ff_effect *ueff_ac = &eff->active;

	/* The effect has a delay which has not expired yet */
	if (time_after(eff->start_at, now)) {
		/* Adjust the time of start */
		eff->start_at = eff->start_at - msecs_to_jiffies(ueff_ac->replay.delay) + msecs_to_jiffies(ueff_la->replay.delay);
		eff->updated_at = eff->start_at;
		eff->touch_at = eff->start_at;
	}

	if (ueff_la->replay.length)
		eff->stop_at = eff->start_at + msecs_to_jiffies(ueff_la->replay.length);
}

/* Destroy request - input device is being destroyed */
static void ffpl_destroy_rq(struct ff_device *ff)
{
	struct klgd_plugin *self = ff->private;
	struct klgd_plugin_private *priv = self->private;

	kfree(priv->effects);
	kfree(priv);
}

/*
 * Handlers of queued userspace requests.
 * All of the all called with plugins_lock acqired and dev->event_lock held
 * Mind the fact that the handlers are not allowed to sleep!
 */

/*
 * Handle request to erase an effect within KLGDFF
 */
static void ffpl_erase_handler(struct klgd_plugin_private *priv, const int effect_id)
{
	struct ffpl_effect *eff = &priv->effects[effect_id];

	eff->change = FFPL_TO_ERASE;
	eff->trigger = FFPL_TRIG_NOW;
}

/*
 * Handle request to start or stop an effect within KLGDFF
 */
static void ffpl_playback_handler(struct klgd_plugin_private *priv, const struct ffpl_request_playback *pb, const unsigned long now)
{
	struct ffpl_effect *eff = &priv->effects[pb->effect_id];

	eff->repeat = pb->value;
	if (pb->value > 0) {
		if (ffpl_handle_timing(priv, &eff->latest))
			ffpl_calculate_trip_times(eff, now);
		else
			eff->start_at = now; /* Start the effect right away and let the device deal with the timing */
		eff->trigger = FFPL_TRIG_START;
	} else {
		eff->change = FFPL_TO_STOP;
		eff->trigger = FFPL_TRIG_NOW;
	}
}

/*
 * Handle request to upload an effect within KLGDFF
 */
static void ffpl_upload_handler(struct klgd_plugin_private *priv, const struct ff_effect *ueff, const unsigned int now)
{
	struct ffpl_effect *eff = &priv->effects[ueff->id];

	/* Copy the new effect to the "latest" slot */
	eff->latest = *ueff;

	if (eff->state != FFPL_EMPTY) {
		if (ffpl_needs_replacing(&eff->active, &eff->latest)) {
			eff->replace = true;
			eff->change = FFPL_TO_UPLOAD;
			eff->trigger = FFPL_TRIG_NOW;
		} else {
			eff->replace = false;
			if (ffpl_handle_timing(priv, ueff))
				ffpl_update_trip_times(eff, now);

			/* The effect is yet to be started, do not try to update it */
			if (eff->change == FFPL_TO_START)
				return;
			if (eff->state == FFPL_STARTED) {
				eff->change = FFPL_TO_UPDATE;
				eff->trigger = FFPL_TRIG_UPDATE;
			}
			/* Else: effect is not active, do nothing */
		}
	} else {
		eff->change = FFPL_TO_UPLOAD;
		eff->trigger = FFPL_TRIG_NOW;
	}
}

/*
 * Handle request for change of autocentering force within KLGDFF
 */
static void ffpl_set_autocenter_handler(struct klgd_plugin_private *priv, const u16 autocenter)
{
	priv->autocenter = autocenter;
	priv->change_autocenter = true;
}

/*
 * Handle request for change of gain within KLGDFF
 */
static void ffpl_set_gain_handler(struct klgd_plugin_private *priv, const u16 gain)
{
	size_t idx;

	priv->gain = gain;
	priv->change_gain = true;
	if (priv->has_native_gain)
		return;

	for (idx = 0; idx < priv->effect_count; idx++) {
		struct ffpl_effect *eff = &priv->effects[idx];

		if (eff->state != FFPL_STARTED)
			continue;

		if (eff->change == FFPL_DONT_TOUCH && eff->trigger != FFPL_TRIG_RECALC) {
			eff->change = FFPL_TO_UPDATE;
			eff->trigger = FFPL_TRIG_NOW;
		}
	}
}

/*
 * Here is where we process all queued requests.
 * We take hold of the dev->event_lock spinlock to make
 * sure that the queue is not modified while we are processing it.
 * We always process the whole queue in one shot.
 */
static void ffpl_request_work(struct work_struct *w)
{
	unsigned long flags;
	struct list_head *p, *n;
	struct klgd_plugin_private *priv = container_of(w, struct klgd_plugin_private, rqwq_work);
	struct klgd_plugin *self = priv->self;
	const unsigned long now = jiffies;

	klgd_lock_plugins(self->plugins_lock);
	spin_lock_irqsave(&priv->dev->event_lock, flags);

	if (list_empty(&priv->rq_list)) {
		spin_unlock_irqrestore(&priv->dev->event_lock, flags);
		klgd_unlock_plugins(self->plugins_lock);
	}

	list_for_each_safe(p, n, &priv->rq_list) {
		struct ffpl_request_task *t = list_entry(p, struct ffpl_request_task, rq_list);
		struct ffpl_request *rq = &t->rq;

		switch (rq->type) {
		case FFPL_RQ_UPLOAD:
			ffpl_upload_handler(priv, &rq->data.upload_effect, now);
			break;
		case FFPL_RQ_PLAYBACK:
			ffpl_playback_handler(priv, &rq->data.pb, now);
			break;
		case FFPL_RQ_ERASE:
			ffpl_erase_handler(priv, rq->data.effect_id);
			break;
		case FFPL_RQ_AUTOCENTER:
			ffpl_set_autocenter_handler(priv, rq->data.autocenter);
			break;
		case FFPL_RQ_GAIN:
			ffpl_set_gain_handler(priv, rq->data.gain);
			break;
		default:
			break;
		}

		list_del(p);
		kfree(t);
	}

	spin_unlock_irqrestore(&priv->dev->event_lock, flags);
	klgd_unlock_plugins_sched(self->plugins_lock);
}

/*
 * Some userspace requests are executed in atomic context, namely
 * playback, set_autocenter and change_gain.
 * To deal with the inability to sleep inside any of these we push
 * all requests on a queue and process them later from a workqueue.
 * To keep the order of requests consistent we push all userspace
 * requests on that queue.
 */

/*
 * Request to erase an effect coming from userspace
 * Explicit lock of dev->event_lock is required to keep the priv->rq_list consistent
 */
static int ffpl_erase_rq(struct input_dev *dev, int effect_id)
{
	unsigned long flags;
	struct ffpl_request_task *t;
	struct klgd_plugin *self = dev->ff->private;
	struct klgd_plugin_private *priv = self->private;
	/*struct ffpl_effect *eff = &priv->effects[effect_id];*/

	printk(KERN_NOTICE "KLGDFF: RQ erase (effect %d)\n", effect_id);

	t = kmalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->rq.type = FFPL_RQ_ERASE;
	t->rq.data.effect_id = effect_id;

	spin_lock_irqsave(&dev->event_lock, flags);
	list_add_tail(&t->rq_list, &priv->rq_list);
	queue_work(priv->rqwq, &priv->rqwq_work);
	spin_unlock_irqrestore(&dev->event_lock, flags);

	return 0;
}

/*
 * Request to start or stop playback of an effect coming from userspace.
 * Called with dev->event_lock held
 */
static int ffpl_playback_rq(struct input_dev *dev, int effect_id, int value)
{
	struct ffpl_request_task *t;
	struct klgd_plugin *self = dev->ff->private;
	struct klgd_plugin_private *priv = self->private;

	t = kmalloc(sizeof(*t), GFP_ATOMIC);
	if (!t)
		return -ENOMEM;

	t->rq.type = FFPL_RQ_PLAYBACK;
	t->rq.data.pb.value = value;
	t->rq.data.pb.effect_id = effect_id;
	list_add_tail(&t->rq_list, &priv->rq_list);
	queue_work(priv->rqwq, &priv->rqwq_work);

	return 0;
}

/*
 * Request to upload or update an effect coming from userspace.
 * Explicit lock of dev->event_lock is required to keep the priv->rq_list consistent
 */
static int ffpl_upload_rq(struct input_dev *dev, struct ff_effect *effect, struct ff_effect *old)
{
	unsigned long flags;
	struct ffpl_request_task *t;
	struct klgd_plugin *self = dev->ff->private;
	struct klgd_plugin_private *priv = self->private;

	printk(KERN_NOTICE "KLGDFF: RQ upload (effect %d)\n", effect->id);

	if (!ffpl_is_effect_valid(effect))
		return -EINVAL;

	t = kmalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->rq.type = FFPL_RQ_UPLOAD;
	t->rq.data.upload_effect = *effect;

	spin_lock_irqsave(&dev->event_lock, flags);
	list_add_tail(&t->rq_list, &priv->rq_list);
	queue_work(priv->rqwq, &priv->rqwq_work);
	spin_unlock_irqrestore(&dev->event_lock, flags);

	return 0;
}

/*
 * Request to change autocentering force coming from userspace.
 * Called with dev->event_lock held
 */
static void ffpl_set_autocenter_rq(struct input_dev *dev, u16 autocenter)
{
	struct ffpl_request_task *t;
	struct klgd_plugin *self = dev->ff->private;
	struct klgd_plugin_private *priv = self->private;

	t = kmalloc(sizeof(*t), GFP_ATOMIC);
	if (!t)
		return;

	t->rq.type = FFPL_RQ_AUTOCENTER;
	t->rq.data.autocenter = autocenter;
	list_add_tail(&t->rq_list, &priv->rq_list);
	queue_work(priv->rqwq, &priv->rqwq_work);
}

/*
 * Request to change overall gain coming from userspace.
 * Called with dev->event_lock held
 */
static void ffpl_set_gain_rq(struct input_dev *dev, u16 gain)
{
	struct ffpl_request_task *t;
	struct klgd_plugin *self = dev->ff->private;
	struct klgd_plugin_private *priv = self->private;

	t = kmalloc(sizeof(*t), GFP_ATOMIC);
	if (!t)
		return;

	t->rq.type = FFPL_RQ_GAIN;
	t->rq.data.gain = gain;
	list_add_tail(&t->rq_list, &priv->rq_list);
	queue_work(priv->rqwq, &priv->rqwq_work);
}

static void ffpl_deinit(struct klgd_plugin *self)
{
	struct list_head *p, *n;
	struct klgd_plugin_private *priv = self->private;

	flush_workqueue(priv->rqwq);
	destroy_workqueue(priv->rqwq);

	list_for_each_safe(p, n, &priv->rq_list) {
		list_del(p);
		kfree(p);
	}

	printk(KERN_DEBUG "KLGDFF: Deinit complete\n");
}

static int ffpl_handle_combinable_effects(struct klgd_plugin_private *priv, struct klgd_command_stream *s,
					  const unsigned long now)
{
	size_t idx;
	bool needs_update_cf = false;
	bool needs_update_rumble = false;
	size_t active_effects_cf = 0;
	size_t active_effects_rumble = 0;

	for (idx = 0; idx < priv->effect_count; idx++) {
		int ret;
		struct ffpl_effect *eff = &priv->effects[idx];

		if (time_before(now, eff->touch_at)) {
			printk(KERN_NOTICE "KLGDFF: Combinable effect is to be processed at a later time, skipping\n");
			continue;
		}

		if (eff->replace) {
			/* Uncombinable effect is about to be replaced by a combinable one */
			if (ffpl_process_memless(priv, &eff->latest, FFPL_HANDLER_ANY)) {
				printk(KERN_NOTICE "KLGDFF: Replacing uncombinable with combinable\n");
				switch (eff->state) {
				case FFPL_STARTED:
					ret = ffpl_stop_effect(priv, s, eff);
					if (ret)
						return ret;
				default:
					ret = ffpl_erase_effect(priv, s, eff);
					if (ret)
						return ret;
					break;
				}
				eff->replace = false;
			} else {
			/* Combinable effect is being replaced by an uncombinable one */
				printk(KERN_NOTICE "KLGDFF: Replacing combinable with uncombinable\n");
				if (eff->state == FFPL_STARTED)
					NEEDS_UPDATE_SET(eff->active.type);
				eff->state = FFPL_EMPTY;
				eff->replace = false;
				continue;
			}
		} else {
			if (!ffpl_process_memless(priv, &eff->latest, FFPL_HANDLER_ANY))
				continue;
		}

		switch (eff->change) {
		case FFPL_DONT_TOUCH:
			if (eff->state == FFPL_STARTED) {
				ACTIVE_EFFECTS_INC(eff->active.type);
				if (eff->recalculate) {
					NEEDS_UPDATE_SET(eff->active.type);
					eff->recalculate = false;
					printk(KERN_NOTICE "KLGDFF: Recalculable combinable effect, total active effects (CF/Rumble) %lu/%lu\n", active_effects_cf, active_effects_rumble);
				}
			} else
				printk(KERN_NOTICE "KLGDFF: Unchanged combinable effect, total active effects (CF/Rumble) %lu/%lu\n", active_effects_cf, active_effects_rumble);
			break;
		case FFPL_TO_START:
			eff->state = FFPL_STARTED;
		case FFPL_TO_UPDATE:
			eff->active = eff->latest;
			if (eff->state != FFPL_STARTED) {
				printk(KERN_NOTICE "KLGDFF: Updating a stopped combinable effect\n");
				break;
			}
			ACTIVE_EFFECTS_INC(eff->active.type);
			NEEDS_UPDATE_SET(eff->active.type);
			printk(KERN_NOTICE "KLGDFF: %s combinable effect, total active effects (CF/Rumble) %lu/%lu\n", eff->change == FFPL_TO_START ? "Started" : "Altered",
			       active_effects_cf, active_effects_rumble);
			break;
		case FFPL_TO_STOP:
			if (eff->state == FFPL_STARTED)
				NEEDS_UPDATE_SET(eff->active.type);
		case FFPL_TO_UPLOAD:
			eff->active = eff->latest;
			eff->state = FFPL_UPLOADED;
			printk(KERN_NOTICE "KLGDFF: Combinable effect to upload/stop, marking as uploaded\n");
			break;
		case FFPL_TO_ERASE:
			if (eff->state == FFPL_STARTED)
				NEEDS_UPDATE_SET(eff->active.type);
			eff->state = FFPL_EMPTY;
			printk(KERN_NOTICE "KLGDFF: Stopped combinable effect, total active effects (CF/Rumble) %lu/%lu\n", active_effects_cf, active_effects_rumble);
			break;
		default:
			printk(KERN_WARNING "KLGDFF: Unknown effect change!\n");
			break;
		}

		eff->change = FFPL_DONT_TOUCH;
	}

	/* Combined effect needs recalculation */
	if (needs_update_cf) {
		if (active_effects_cf) {
			printk(KERN_NOTICE "KLGDFF: Combined constant force effect needs an update, total effects active: %lu\n", active_effects_cf);
			ffpl_recalc_combined_cf(priv, now);
			if (priv->combined_effect_cf.state == FFPL_STARTED)
				priv->combined_effect_cf.change = FFPL_TO_UPDATE;
			else
				priv->combined_effect_cf.change = FFPL_TO_START;
		} else {
			/* No combinable effects are active, remove the effect from device */
			if (priv->combined_effect_cf.state != FFPL_EMPTY) {
				printk(KERN_NOTICE "KLGDFF: No combinable constant force effects are active, erase the combined constant force effect from device\n");
				priv->combined_effect_cf.change = FFPL_TO_ERASE;
			}
		}
	}

	if (needs_update_rumble) {
		if (active_effects_rumble) {
			printk(KERN_NOTICE "KLGDFF: Combined rumble effect needs an update, total effects active: %lu\n", active_effects_rumble);
			ffpl_recalc_combined_rumble(priv, now);
			if (priv->combined_effect_rumble.state == FFPL_STARTED)
				priv->combined_effect_rumble.change = FFPL_TO_UPDATE;
			else
				priv->combined_effect_rumble.change = FFPL_TO_START;
		} else {
			/* No combinable effects are active, remove the effect from device */
			if (priv->combined_effect_rumble.state != FFPL_EMPTY) {
				printk(KERN_NOTICE "KLGDFF: No combinable rumble effects are active, erase the combined effect from device\n");
				priv->combined_effect_rumble.change = FFPL_TO_ERASE;
			}
		}
	}

	return 0;
}

static unsigned long ffpl_get_ticking_recalculation_time(const struct ffpl_effect *eff, const unsigned long now)
{
	const struct ff_effect *ueff = &eff->active;

	switch (ueff->type) {
	case FF_PERIODIC:
		switch (ueff->u.periodic.waveform) {
		case FF_SQUARE:
			return now + msecs_to_jiffies(ueff->u.periodic.period / 2);
		default:
			return now + msecs_to_jiffies(RECALC_DELTA_T_MSEC);
		}
		break;
	case FF_RAMP:
		return now + msecs_to_jiffies(RECALC_DELTA_T_MSEC);
	default:
		WARN(true, "KLGDFF: Invalid type of effect passed to ticking ticking_recalculation. This cannot happen!\n");
		return 0;
	}
}

static unsigned long ffpl_get_env_recalculation_time(const struct ffpl_effect *eff, const unsigned long now)
{
	const struct ff_envelope *env = ffpl_get_envelope(&eff->active);
	unsigned long t;

	/* Is the envelope attacking */
	t = eff->start_at + msecs_to_jiffies(env->attack_length); /* Time of the end of the attack */
	if (time_before_eq(now, t)) {
		unsigned long st = now + msecs_to_jiffies(RECALC_DELTA_T_MSEC);
		printk(KERN_NOTICE "KLGDFF: Envelope attacking\n");
		/* Schedule an update for the end of the attack */
		if (time_after(st, t))
			return t;
		return st;
	}

	t = eff->stop_at - msecs_to_jiffies(env->fade_length); /* Time of the beginning of the fade */
	if (time_before_eq(now, t)) {
		printk(KERN_NOTICE "KLGDFF: Envelope waiting to fade\n");
		if (time_after(t, eff->stop_at))
			return eff->stop_at;
		return t;	/* Schedule an update for the beginning of the fade */
	}

	printk(KERN_NOTICE "KLGDFF: Envelope is fading\n");
	t = now + msecs_to_jiffies(RECALC_DELTA_T_MSEC); /* Continue fading */
	return (time_after(t, eff->stop_at)) ? eff->stop_at : t;
}

static unsigned long ffpl_get_recalculation_time(const struct klgd_plugin_private *priv, const struct ffpl_effect *eff,
						 const unsigned long now)
{
	const struct ff_envelope *env = ffpl_get_envelope(&eff->active);
	const bool ticks = eff->active.type == FF_PERIODIC || eff->active.type == FF_RAMP;
	bool has_envelope = false;

	if (eff->active.type == FF_RUMBLE && priv->memless_rumble_emul)
		return now + msecs_to_jiffies(RECALC_DELTA_T_MSEC);

	if (env)
		has_envelope = env->attack_length || env->fade_length;

	if (ticks && has_envelope) {
		const unsigned long t_tick = ffpl_get_ticking_recalculation_time(eff, now);
		const unsigned long t_env = ffpl_get_env_recalculation_time(eff, now);

		printk(KERN_NOTICE "KLGDFF: Effect has both envelope and ticks\n");
		return time_before(t_tick, t_env) ? t_tick : t_env;
	}
	if (ticks)
		return ffpl_get_ticking_recalculation_time(eff, now);
	if (!has_envelope) {
		WARN(true, "KLGDFF: Effect does not have to be recalculated but ffpl_get_recalculation_time() was called\n");
		return now;
	}

	return ffpl_get_env_recalculation_time(eff, now);
}

static bool ffpl_needs_recalculation(const struct klgd_plugin_private *priv, const struct ff_effect *ueff, const unsigned long start_at,
				     const unsigned long stop_at, const unsigned long now)
{
	const struct ff_envelope *env = ffpl_get_envelope(ueff);
	bool ticks = ueff->type == FF_PERIODIC || ueff->type == FF_RAMP;

	if (time_before(now, start_at)) {
		printk(KERN_NOTICE "KLGDFF: Effect has not started yet\n");
		return false;
	}

	/* Only effects handled by memless mode can be periodically reprocessed */
	if (!ffpl_process_memless(priv, ueff, FFPL_HANDLER_ANY)) {
		printk(KERN_NOTICE "KLGDFF: Effect not combinable, won't recalculate\n");
		return false;
	}

	/* Emulated rumble effect needs to be recalculated under all circuumstances */
	if (ueff->type == FF_RUMBLE && priv->memless_rumble_emul && !(ueff->replay.length && time_after_eq(now, stop_at)))
		return true;

	if (!env) {
		printk(KERN_NOTICE "KLGDFF: Effect type does not support envelope, no need to recalculate\n");
		return false;
	}

	if (!env->attack_length && !env->fade_length && !ticks) {
		printk(KERN_NOTICE "KLGDFF: Effect has no envelope and does not tick, no need to recalculate\n");
		return false;
	}

	if (!env->fade_length && time_after_eq(now, msecs_to_jiffies(env->attack_length) + start_at) && !ticks) {
		printk(KERN_NOTICE "KLGDFF: Envelope has finished attacking, it does not fade and it does not tick, no need to recalculate\n");
		return false;
	}

	/* Effect is done fading and shall be stopped.
	 * Stopping of the effect is handled elsewhere */
	if (ueff->replay.length && time_after_eq(now, stop_at)) {
		printk(KERN_NOTICE "KLGDFF: Effect has finished, no need to recalculate\n");
		return false;
	}

	printk(KERN_NOTICE "KLGDFF: Effect needs recalculation\n");
	return true;
}

static void ffpl_advance_trigger(const struct klgd_plugin_private *priv, struct ffpl_effect *eff, const unsigned long now)
{
	switch (eff->trigger) {
	case FFPL_TRIG_START:
		if (ffpl_needs_recalculation(priv, &eff->latest, eff->start_at, eff->stop_at, now)) {
			eff->trigger = FFPL_TRIG_RECALC;
			break;
		}
		if (eff->latest.replay.length && ffpl_handle_timing(priv, &eff->latest))
			eff->trigger = FFPL_TRIG_STOP;
		else
			eff->trigger = FFPL_TRIG_NONE;
		break;
	case FFPL_TRIG_RESTART:
		eff->trigger = FFPL_TRIG_STOP;
		break;
	case FFPL_TRIG_RECALC:
		if (ffpl_needs_recalculation(priv, &eff->active, eff->start_at, eff->stop_at, now))
			break;
		if (eff->active.replay.length && ffpl_process_memless(priv, &eff->active, FFPL_HANDLER_ANY)) {
			eff->trigger = FFPL_TRIG_STOP;
			break;
		}
		eff->trigger = FFPL_TRIG_NONE;
		break;
	case FFPL_TRIG_STOP:
		if (eff->repeat > 0 && ffpl_handle_timing(priv, &eff->active)) {
			eff->trigger = FFPL_TRIG_RESTART;
			break;
		}
	case FFPL_TRIG_NOW:
		eff->trigger = FFPL_TRIG_NONE;
		break;
	case FFPL_TRIG_UPDATE:
		if (ffpl_needs_recalculation(priv, &eff->active, eff->start_at, eff->stop_at, now) && eff->state == FFPL_STARTED)
			eff->trigger = FFPL_TRIG_RECALC;
		else
			eff->trigger = FFPL_TRIG_NONE;
		break;
	default:
		break;
	}
}

static int ffpl_get_commands(struct klgd_plugin *self, struct klgd_command_stream **s, const unsigned long now)
{
	struct klgd_plugin_private *priv = self->private;
	size_t idx;
	int ret;

	*s = klgd_alloc_stream();
	if (!s)
		return -EAGAIN;

	if (priv->change_autocenter) {
		ret = ffpl_set_autocenter(priv, *s);
		if (ret)
			goto out;
		priv->change_autocenter = false;
	}
	if (priv->change_gain) {
		ret = ffpl_set_gain(priv, *s);
		if (ret)
			goto out;
		priv->change_gain = false;
	}

	ret = ffpl_handle_combinable_effects(priv, *s, now);
	if (ret) {
		printk(KERN_WARNING "KLGDFF: Cannot process combinable effects, ret %d\n", ret);
		goto out;
	}

	/* Handle combined constant force effect here */
	printk(KERN_NOTICE "KLGDFF: Combined CF: ");
	ret = ffpl_handle_state_change(priv, *s, &priv->combined_effect_cf, now);
	if (ret) {
		printk(KERN_WARNING "KLGDFF: Cannot get command stream for combined constant force effect\n");
		goto out;
	}

	printk(KERN_NOTICE "KLGDFF: Combined Rumble: ");
	ret = ffpl_handle_state_change(priv, *s, &priv->combined_effect_rumble, now);
	if (ret) {
		printk(KERN_WARNING "KLGDFF: Cannot get command stream for combined rumble effect\n");
		goto out;
	}

	for (idx = 0; idx < priv->effect_count; idx++) {
		struct ffpl_effect *eff = &priv->effects[idx];

		//printk(KERN_NOTICE "KLGDFF: Processing effect %lu\n", idx);

		if (time_before(now, eff->touch_at)) {
			printk(KERN_NOTICE "KLGDFF: Next change of the effect is schededuled for the future\n");
			continue;
		}

		ret = ffpl_handle_state_change(priv, *s, eff, now);
		/* TODO: Do something useful with the return code */
		if (ret) {
			printk(KERN_WARNING "KLGDFF: Cannot get command stream for effect %lu\n", idx);
			goto out;
		}

		ffpl_advance_trigger(priv, eff, now);
	}

out:
	return ret;
}

static bool ffpl_get_update_time(struct klgd_plugin *self, const unsigned long now, unsigned long *t)
{
	struct klgd_plugin_private *priv = self->private;
	size_t idx;
	unsigned long events = 0;

	/* Handle device-wide changes first */
	if (priv->change_gain || priv->change_autocenter) {
		*t = now;
		return true;
	}

	for (idx = 0; idx < priv->effect_count; idx++) {
		unsigned long current_t;
		struct ffpl_effect *eff = &priv->effects[idx];

		switch (eff->trigger) {
		case FFPL_TRIG_NOW:
		case FFPL_TRIG_UPDATE:
			current_t = now;
			break;
		case FFPL_TRIG_RESTART:
			ffpl_calculate_trip_times(eff, now);
		case FFPL_TRIG_START:
			current_t = eff->start_at;
			eff->playback_time = 0;
			eff->change = FFPL_TO_START;
			break;
		case FFPL_TRIG_STOP:
			/* Small processing delays might make us to miss the precise stop point */
			current_t = time_before(eff->stop_at, now) ? now : eff->stop_at;
			eff->change = FFPL_TO_STOP;
			eff->repeat--;
			break;
		case FFPL_TRIG_RECALC:
			current_t = ffpl_get_recalculation_time(priv, eff, now);
			eff->recalculate = true;
			break;
		default:
			continue;
		}

		eff->touch_at = current_t;

		if (!events++) {
			printk(KERN_NOTICE "KLGDFF: First event\n");
			*t = current_t;
		} else if (time_before(current_t, *t))
			*t = current_t;
	}


	if (time_before(*t, now) && events) {
		WARN(true, KERN_ERR "Scheduling for the past (now: %lu, sched %lu), fixing by sheduling for now\n", now, *t);
		*t = now;
	}
	return events ? true : false;
}

static int ffpl_handle_state_change(struct klgd_plugin_private *priv, struct klgd_command_stream *s, struct ffpl_effect *eff,
				    const unsigned long now)
{
	int ret;

	/* Latest effect is of different type than currently active effect,
	 * remove it from the device and upload the latest one */
	if (eff->replace) {
		switch (eff->change) {
		case FFPL_TO_ERASE:
			printk(KERN_NOTICE "KLGDFF: Rpl chg - TO_ERASE\n");
			switch (eff->state) {
			case FFPL_STARTED:
				ret = ffpl_stop_effect(priv, s, eff);
				if (ret)
					break;
			case FFPL_UPLOADED:
				ret = ffpl_erase_effect(priv, s, eff);
				if (ret)
					break;
			default:
				/* Nothing to do - the effect that is replacing the old effect is about to be erased anyway
				 * State of the effect to be replaced should also never be EMPTY */
				ret = 0;
				break;
			}
			break;
		case FFPL_TO_UPLOAD:
		case FFPL_TO_STOP: /* There is no difference between stopping or uploading an effect when we are replacing it */
			printk("KLGDFF: Rpl chg - TO_UPLOAD/TO_STOP\n");
			switch (eff->state) {
			case FFPL_STARTED:
				/* Overwrite the currently active effect and set it to UPLOADED state */
				if (priv->has_owr_to_upl) {
					ret = ffpl_replace_effect(priv, s, eff, FFPL_OWR_TO_UPL);
					break;
				}
				ret = ffpl_stop_effect(priv, s, eff);
				if (ret)
					break;
			case FFPL_UPLOADED:
				ret = ffpl_erase_effect(priv, s, eff);
				if (ret)
					break;
			case FFPL_EMPTY: /* State cannot actually be FFPL_EMPTY becuase only uploaded or started effects have to be replaced like this.
					  * Only exception is when combinable effect is replaced */
				ret = ffpl_upload_effect(priv, s, eff);
				break;
			default:
				printk(KERN_WARNING "KLGDFF: Unhandled effect state\n");
				ret = -EINVAL;
			}
			break;
		case FFPL_TO_START:
		case FFPL_TO_UPDATE: /* There is no difference between staring or updating an effect when we are replacing it */
			printk("KLGDFF: Rpl chg - TO_START/TO_UPDATE\n");
			switch (eff->state) {
			case FFPL_STARTED:
				if (priv->has_owr_to_srt) {
					ret = ffpl_replace_effect(priv, s, eff, FFPL_OWR_TO_SRT);
					break;
				}
				ret = ffpl_stop_effect(priv, s, eff);
				if (ret)
					break;
			case FFPL_UPLOADED:
				ret = ffpl_erase_effect(priv, s, eff);
				if (ret)
					break;
			case FFPL_EMPTY: /* State cannot actually be FFPL_EMPTY - same as above applies */
				ret = ffpl_upload_effect(priv, s, eff);
				if (ret)
					break;
				ret = ffpl_start_effect(priv, s, eff);
				break;
			default:
				printk(KERN_WARNING "KLGDFF: Unhandled effect state\n");
				ret = -EINVAL;
				break;
			}
			break;
		case FFPL_DONT_TOUCH:
			printk(KERN_WARNING "KLGDFF: Got FFPL_DONT_TOUCH change for effect that should be replaced - this should not happen!\n");
			ret = -EINVAL;
			break;
		default:
			printk(KERN_WARNING "KLGDFF: Unhandled state change while replacing effect\n");
			ret = -EINVAL;
			break;
		}
		if (ret)
			printk(KERN_WARNING "KLGDFF: Error %d while replacing effect\n", ret);

		eff->replace = false;
		eff->change = FFPL_DONT_TOUCH;
		eff->updated_at = now;
		return ret;
	}

	switch (eff->change) {
	case FFPL_TO_UPLOAD:
		printk(KERN_INFO "KLGDFF: Chg TO_UPLOAD\n");
		switch (eff->state) {
		case FFPL_EMPTY:
			ret = ffpl_upload_effect(priv, s, eff);
			break;
		case FFPL_STARTED:
			ret = ffpl_stop_effect(priv, s, eff);
			break;
		default:
			ret = 0;
			break;
		}
		break;
	case FFPL_TO_START:
		printk(KERN_INFO "KLGDFF: Chg TO_START\n");
		switch (eff->state) {
		case FFPL_EMPTY:
			if (priv->has_emp_to_srt) {
				ret = ffpl_start_effect(priv, s, eff);
				break;
			}
			ret = ffpl_upload_effect(priv, s, eff);
			if (ret)
				return ret;
		case FFPL_UPLOADED:
			ret = ffpl_start_effect(priv, s, eff);
			break;
		default:
			ret = 0;
			break;
		}
		break;
	case FFPL_TO_STOP:
		printk(KERN_INFO "KLGDFF: Chg TO_STOP\n");
		switch (eff->state) {
		case FFPL_STARTED:
			ret = ffpl_stop_effect(priv, s, eff);
			break;
		case FFPL_EMPTY:
			ret = ffpl_upload_effect(priv, s, eff);
			break;
		default:
			ret = 0;
			break;
		}
		break;
	case FFPL_TO_ERASE:
		printk(KERN_INFO "KLGDFF: Chg TO_ERASE\n");
		switch (eff->state) {
		case FFPL_UPLOADED:
			ret = ffpl_erase_effect(priv, s, eff);
			break;
		case FFPL_STARTED:
			ret = ffpl_stop_effect(priv, s, eff);
			if (ret)
				break;
			ret = ffpl_erase_effect(priv, s, eff);
			break;
		default:
			ret = 0;
			break;
		}
		break;
	case FFPL_TO_UPDATE:
		printk(KERN_INFO "KLGDFF: Chg TO_UPDATE\n");
		ret = ffpl_update_effect(priv, s, eff);
		break;
	case FFPL_DONT_TOUCH:
		//printk(KERN_INFO "KLGDFF: Chg - NO CHANGE\n");
		return 0;
	default:
		return -EINVAL;
		pr_debug("KLGDFF: Unhandled effect state change\n");
	}

	eff->change = FFPL_DONT_TOUCH;
	eff->updated_at = now;

	return ret;
}

static int ffpl_init(struct klgd_plugin *self)
{
	struct klgd_plugin_private *priv = self->private;
	struct input_dev *dev = priv->dev;
	int ret;

	ret = input_ff_create(dev, priv->effect_count);
	if (ret)
		return ret;

	dev->ff->private = self;
	dev->ff->erase = ffpl_erase_rq;
	dev->ff->playback = ffpl_playback_rq;
	dev->ff->upload = ffpl_upload_rq;
	dev->ff->set_gain = ffpl_set_gain_rq;
	dev->ff->set_autocenter = ffpl_set_autocenter_rq;
	dev->ff->destroy = ffpl_destroy_rq;
	printk(KERN_NOTICE "KLGDFF: Init complete\n");

	return 0;
}

/* Initialize the plugin */
int ffpl_init_plugin(struct klgd_plugin **plugin, struct input_dev *dev, const size_t effect_count,
		     const unsigned long flags,
		     int (*control)(struct input_dev *dev, struct klgd_command_stream *s, const enum ffpl_control_command cmd, const union ffpl_control_data data, void *user),
		     void *user)
{
	struct klgd_plugin *self;
	struct klgd_plugin_private *priv;
	int ret, idx;

	self = kzalloc(sizeof(struct klgd_plugin), GFP_KERNEL);
	if (!self)
		return -ENOMEM;

	priv = kzalloc(sizeof(struct klgd_plugin_private), GFP_KERNEL);
	if (!priv) {
		ret = -ENOMEM;
		goto err_out1;
	}

	priv->effects = kzalloc(sizeof(struct ffpl_effect) * effect_count, GFP_KERNEL);
	if (!priv->effects) {
		ret = -ENOMEM;
		goto err_out2;
	}
	for (idx = 0; idx < effect_count; idx++) {
		priv->effects[idx].replace = false;
		priv->effects[idx].uploaded_to_device = false;
		priv->effects[idx].state = FFPL_EMPTY;
		priv->effects[idx].change = FFPL_DONT_TOUCH;
	}

	self->deinit = ffpl_deinit;
	self->get_commands = ffpl_get_commands;
	self->get_update_time = ffpl_get_update_time;
	self->init = ffpl_init;
	priv->effect_count = effect_count;
	priv->dev = dev;
	priv->control = control;
	priv->user = user;
	priv->gain = 0xFFFF;
	priv->rqwq = create_singlethread_workqueue("ffpl_request_work");
	if (!priv->rqwq) {
		ret = -ENOMEM;
		goto err_out2;
	}
	INIT_WORK(&priv->rqwq_work, ffpl_request_work);
	INIT_LIST_HEAD(&priv->rq_list);

	self->private = priv;
	priv->self = self;
	*plugin = self;

	if (FFPL_HAS_EMP_TO_SRT & flags) {
		priv->has_emp_to_srt = true;
		printk("KLGDFF: Using HAS EMP_TO_SRT\n");
	}
	if (FFPL_HAS_SRT_TO_EMP & flags) {
		priv->has_srt_to_emp = true;
		printk("KLGDFF: Using HAS SRT_TO_EMP\n");
	}
	if (FFPL_UPLOAD_WHEN_STARTED & flags) {
		priv->has_emp_to_srt = true;
		priv->upload_when_started = true;
		printk("KLGDFF: Using UPLOAD WHEN STARTED\n");
	}
	if (FFPL_ERASE_WHEN_STOPPED & flags) {
		priv->has_srt_to_emp = true;
		priv->erase_when_stopped = true;
		printk("KLGDFF: Using ERASE WHEN STOPPED\n");
	}
	if (FFPL_REPLACE_UPLOADED & flags) {
		priv->has_owr_to_upl = true;
		printk("KLGDFF: Using REPLACE UPLOADED\n");
	}
	if (FFPL_REPLACE_STARTED & flags) {
		priv->has_owr_to_srt = true;
		printk("KLGDFF: Using REPLACE STARTED\n");
	}

	/* Check if the requested memless modes make sense */
	if ((FFPL_MEMLESS_CONSTANT | FFPL_MEMLESS_PERIODIC | FFPL_MEMLESS_RAMP) & flags) {
		if (!test_bit(FF_CONSTANT, dev->ffbit)) {
			printk(KERN_ERR "The driver asked for constant force memless mode but the device does not support FF_CONSTANT\n");
			ret = -EINVAL;
			goto err_out3;
		}
	}
	if ((FFPL_MEMLESS_RUMBLE & flags) && !test_bit(FF_RUMBLE, dev->ffbit)) {
		printk(KERN_ERR "The driver asked for rumble memless mode but the device does not support FF_RUMBLE\n");
		ret = -EINVAL;
		goto err_out3;
	}

	/* Set up memless mode flags */
	if (FFPL_MEMLESS_CONSTANT & flags)
		priv->memless_constant = true;
	if (FFPL_MEMLESS_PERIODIC & flags)
		priv->memless_periodic = true;
	if (FFPL_MEMLESS_RAMP & flags)
		priv->memless_ramp = true;
	if (FFPL_MEMLESS_RUMBLE & flags)
		priv->memless_rumble = true;
	if (FFPL_TIMING_CONDITION & flags)
		priv->timing_condition = true;
	/* Set up emulation memless mode flags */
	/** Emulate rumble through constant force */
	if (test_bit(FF_CONSTANT, dev->ffbit) && !test_bit(FF_RUMBLE, dev->ffbit)) {
		printk(KERN_NOTICE "KLGDFF: Emulating FF_RUMBLE through FF_CONSTANT\n");
		input_set_capability(dev, EV_FF, FF_RUMBLE); /* Prevent ff-core from converting the effect to FF_PERIODIC */
		priv->memless_rumble_emul = true;
	}
	/** Emulate periodic through rumble */
	if (test_bit(FF_RUMBLE, dev->ffbit) && !test_bit(FF_PERIODIC, dev->ffbit)) {
		printk(KERN_NOTICE "KLGDFF: Emulating FF_PERIODIC through FF_RUMBLE\n");
		/* Fake full support of periodic effects*/
		input_set_capability(dev, EV_FF, FF_PERIODIC);
			input_set_capability(dev, EV_FF, FF_SINE);
			input_set_capability(dev, EV_FF, FF_SQUARE);
			input_set_capability(dev, EV_FF, FF_SAW_UP);
			input_set_capability(dev, EV_FF, FF_SAW_DOWN);
			input_set_capability(dev, EV_FF, FF_TRIANGLE);
		priv->memless_periodic_emul = true;
	}

	if (FFPL_HAS_NATIVE_GAIN & flags) {
		priv->has_native_gain = true;
		printk(KERN_NOTICE "KLGDFF: Using HAS_NATIVE_GAIN\n");
	}
	input_set_capability(dev, EV_FF, FF_GAIN);

	return 0;

err_out3:
	destroy_workqueue(priv->rqwq);
err_out2:
	kfree(priv);
err_out1:
	kfree(self);
	return ret;
}
EXPORT_SYMBOL_GPL(ffpl_init_plugin);

static bool ffpl_needs_replacing(const struct ff_effect *ac_eff, const struct ff_effect *la_eff)
{
	if (ac_eff->type != la_eff->type) {
		printk(KERN_NOTICE "KLGDFF: Effects are of different type - replacing (%d x %d)\n", ac_eff->type, la_eff->type);
		return true;
	}

	if (ac_eff->type == FF_PERIODIC) {
		if (ac_eff->u.periodic.waveform != la_eff->u.periodic.waveform) {
			printk(KERN_NOTICE "KLGDFF: Effects have different waveforms - replacing\n");
			return true;
		}
	}

	printk(KERN_NOTICE "KLGDFF: Effect does not have to be replaced, updating\n");
	return false;
}
