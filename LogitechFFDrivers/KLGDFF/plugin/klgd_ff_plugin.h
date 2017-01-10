#include <linux/input.h>
#include "../../KLGD/klgd.h"

/*
 * Preserving full direction for FF_RUMBLE effects is not necessary
 * since the rumble motors can effectively spin in only two directions.
 * The information about direction of rumble effects is reduced to
 * up, down, left and right. These values are stored as individual
 * bits in the effect.direction field as follows:
 *
 * BIT 0: 0 => Strong magnitude is turning left
 * BIT 0: 1 => Strong magnitude is turning right
 * BIT 1: 0 => Strong magnitude is turning down
 * BIT 1: 1 => Strong magnitide is turning up
 * BIT 2: 0 => Weak magnitude is turning left
 * BIT 2: 1 => Weak magnitude is turning right
 * BIT 3: 0 => Weak magnitude is turning down
 * BIT 3: 1 => Weak magnitide is turning up
 *
 * The scheme above stores directions for both strong and weak magnitude
 * in terms of quadrants. This is necessary when effect combining is used
 * and multiple effects are squashed together into one overall effect.
 *
 * Note that the scheme above is used ONLY when combining of rumble effects
 * is active. "Uncombined" rumble effects retain full information about
 * their direction.
 */
#define FFPL_RUMBLE_STRONG_RIGHT BIT(0)
#define FFPL_RUMBLE_STRONG_UP BIT(1)
#define FFPL_RUMBLE_WEAK_RIGHT BIT(2)
#define FFPL_RUMBLE_WEAK_UP BIT(3)

/* Allowed flag bits */
#define FFPL_HAS_EMP_TO_SRT BIT(0) /* Device supports direct "upload and start" */
#define FFPL_HAS_SRT_TO_EMP BIT(1) /* Device supports direct "stop and erase" */
#define FFPL_UPLOAD_WHEN_STARTED BIT(2) /* Upload effects only when they are started - this implies HAS_EMP_TO_SRT */
#define FFPL_ERASE_WHEN_STOPPED BIT(3) /* Erases effect from device when it is stopped - this implies HAS_SRT_TO_EMP */
#define FFPL_REPLACE_UPLOADED BIT(4) /* Device can accept a new effect to UPLOADED state without the need to explicitly stop and erase the previously uploaded effect beforehand */
#define FFPL_REPLACE_STARTED BIT(5) /* Device can accept a new effect to STARTED state without the need to explicitly stop and erase the previously uploaded effect beforehand */

#define FFPL_MEMLESS_CONSTANT BIT(6)     /* Device cannot process FF_CONSTANT by itself and requires KLGD-FF to calculate overall force.
					    Device must support FF_CONSTANT for this to work. */
#define FFPL_MEMLESS_PERIODIC BIT(7)	 /* Device cannot process FF_PERIODIC by itself and requires KLGD-FF to calculate the overall force.
					    Device must support FF_CONSTANT for this to work. */
#define FFPL_MEMLESS_RAMP BIT(8)	 /* Device cannot process FF_RAMP by itself and requires KLGD-FF to calculate the overall force.
					    Device must support FF_CONSTANT for this to work. */
#define FFPL_MEMLESS_RUMBLE BIT(9)	 /* Device cannot process FF_RUMBLE by itself and requires KLGD-FF to calculate the overall force.
					    Device must support FF_RUMBLE for this to work. */

#define FFPL_TIMING_CONDITION BIT(10)	 /* Let the plugin take care of starting and stopping of condition effects */

#define FFPL_HAS_NATIVE_GAIN BIT(15)  /* Device can adjust the gain by itself */


enum ffpl_control_command {
	/* Force feedback state transitions */
	FFPL_EMP_TO_UPL, /* Upload to empty slot */
	FFPL_UPL_TO_SRT, /* Start uploaded effect */
	FFPL_SRT_TO_UPL, /* Stop started effect */
	FFPL_UPL_TO_EMP, /* Erase uploaded effect */
	FFPL_SRT_TO_UDT, /* Update started effect */
	/* Optional force feedback state transitions */
	FFPL_EMP_TO_SRT, /* Upload and start effect */
	FFPL_SRT_TO_EMP, /* Stop and erase started effect */
	FFPL_OWR_TO_UPL, /* Overwrite an effect with a new one and set its state to UPLOADED */
	FFPL_OWR_TO_SRT, /* Overwrite an effect with a new one and set its state to STARTED */

	FFPL_SET_GAIN,	 /* Set gain */
	FFPL_SET_AUTOCENTER /*Set autocenter */
};

struct ffpl_effects {
	const struct ff_effect *cur;  /* Pointer to the effect that is being uploaded/started/stopped/erased */
	const struct ff_effect *old;  /* Pointer to the currently active effect. Valid only with OWR_* commands, otherwise NULL */
	int repeat; /* How many times to repeat playback - valid only with *_SRT commands */
};

union ffpl_control_data {
	struct ffpl_effects effects;
	u16 autocenter;
	u16 gain;
};

void ffpl_lvl_dir_to_x_y(const s32 level, const u16 direction, s32 *x, s32 *y);
int ffpl_init_plugin(struct klgd_plugin **plugin, struct input_dev *dev, const size_t effect_count,
		     const unsigned long flags,
		     int (*control)(struct input_dev *dev, struct klgd_command_stream *s, const enum ffpl_control_command cmd, const union ffpl_control_data data, void *user),
		     void *user);
