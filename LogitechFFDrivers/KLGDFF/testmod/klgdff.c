#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include "../plugin/klgd_ff_plugin.h"

#define EFFECT_COUNT 8

static struct kobject *klgdff_obj;

static struct input_dev *dev;
static struct klgd_main klgd;
static struct klgd_plugin *ff_plugin;
static u16 gain;
static u16 autocenter;
static u32 test_user = 0xC001CAFE;

static char *klgdff_combined_rumble_dir(const u16 dir)
{
	static char dir_str[32];

	dir_str[0] = '\0';

	strcat(dir_str, "STRONG: ");
	if (dir & FFPL_RUMBLE_STRONG_UP)
		strcat(dir_str, "Up ");
	else
		strcat(dir_str, "Down ");
	if (dir & FFPL_RUMBLE_STRONG_RIGHT)
		strcat(dir_str, "Right ");
	else
		strcat(dir_str, "Left ");

	strcat(dir_str, "WEAK: ");
	if (dir & FFPL_RUMBLE_WEAK_UP)
		strcat(dir_str, "Up ");
	else
		strcat(dir_str, "Down ");
	if (dir & FFPL_RUMBLE_WEAK_RIGHT)
		strcat(dir_str, "Right ");
	else
		strcat(dir_str, "Left ");

	return dir_str;
}

static int klgdff_erase(struct klgd_command_stream *s, const struct ff_effect *effect)
{
	char *text = kasprintf(GFP_KERNEL, "Erasing effect, type %d, id %d", effect->type, effect->id);
	size_t len = strlen(text);
	struct klgd_command *c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	memcpy(c->bytes, text, len);
	kfree(text);
	return klgd_append_cmd(s, c);
}

static int klgdff_owr_start(struct klgd_command_stream *s, const struct ff_effect *effect, const struct ff_effect *old_effect, const int repeat)
{
	char *text = kasprintf(GFP_KERNEL, "Overwriting effect to STARTED state, type %d, id %d, old type %d, repeat %d", effect->type, effect->id, old_effect->type, repeat);
	size_t len = strlen(text);
	struct klgd_command *c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	memcpy(c->bytes, text, len);
	kfree(text);
	return klgd_append_cmd(s, c);
}

static int klgdff_owr_upload(struct klgd_command_stream *s, const struct ff_effect *effect, const struct ff_effect *old_effect)
{
	char *text = kasprintf(GFP_KERNEL, "Overwriting effect to UPLOADED state, type %d, id %d, old type %d", effect->type, effect->id, old_effect->type);
	size_t len = strlen(text);
	struct klgd_command *c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	memcpy(c->bytes, text, len);
	kfree(text);
	return klgd_append_cmd(s, c);
}

static int klgdff_er_stop(struct klgd_command_stream *s, const struct ff_effect *effect)
{
	char *text = kasprintf(GFP_KERNEL, "Stopping and erasing effect, type %d, id %d", effect->type, effect->id);
	size_t len = strlen(text);
	struct klgd_command *c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	memcpy(c->bytes, text, len);
	kfree(text);
	return klgd_append_cmd(s, c);
}

static int klgdff_set_autocenter(struct klgd_command_stream *s, const u16 _autocenter)
{
	char *text = kasprintf(GFP_KERNEL, "Setting autocenter to: %u", _autocenter);
	size_t len = strlen(text);
	struct klgd_command *c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	autocenter = _autocenter;

	memcpy(c->bytes, text, len);
	c->user.ldata[0] = 0xDEADBEEF;
	kfree(text);
	return klgd_append_cmd(s, c);
}

static int klgdff_set_gain(struct klgd_command_stream *s, const u16 _gain)
{
	char *text = kasprintf(GFP_KERNEL, "Setting gain to: %u", _gain);
	size_t len = strlen(text);
	struct klgd_command *c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	gain = _gain;

	memcpy(c->bytes, text, len);
	kfree(text);
	return klgd_append_cmd(s, c);
}

static int klgdff_start(struct klgd_command_stream *s, const struct ff_effect *effect, const int repeat)
{
	char *text;
	size_t len;
	struct klgd_command *c;

	switch (effect->type) {
	case FF_CONSTANT:
	{
		s32 x;
		s32 y;
		s32 level = effect->u.constant.level * gain / 0xFFFF;

		ffpl_lvl_dir_to_x_y(level, effect->direction, &x, &y);
		text = kasprintf(GFP_KERNEL, "Playing FF_CONSTANT, level: %d, dir: %u, X: %d, Y: %d", level, effect->direction, x, y);
		break;
	}
	case FF_RUMBLE:
		text = kasprintf(GFP_KERNEL, "Playing FF_RUMBLE, strong: %u, weak: %u, direction: %s\n", effect->u.rumble.strong_magnitude,
				 effect->u.rumble.weak_magnitude, klgdff_combined_rumble_dir(effect->direction));
		break;
	default:
		text = kasprintf(GFP_KERNEL, "Playing effect, type %d, id %d, repeat %d", effect->type, effect->id, repeat);
		break;
	}

	len = strlen(text);
	c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	memcpy(c->bytes, text, len);
	kfree(text);
	return klgd_append_cmd(s, c);
}

static int klgdff_stop(struct klgd_command_stream *s, const struct ff_effect *effect)
{
	char *text = kasprintf(GFP_KERNEL, "Stopping effect, type %d, id %d", effect->type, effect->id);
	size_t len = strlen(text);
	struct klgd_command *c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	memcpy(c->bytes, text, len);
	kfree(text);
	return klgd_append_cmd(s, c);
}

static int klgdff_update(struct klgd_command_stream *s, const struct ff_effect *effect)
{
	char *text;
	size_t len;
	struct klgd_command *c;

	switch (effect->type) {
	case FF_CONSTANT:
	{
		s32 x;
		s32 y;
		s32 level = effect->u.constant.level * gain / 0xFFFF;

		ffpl_lvl_dir_to_x_y(level, effect->direction, &x, &y);
		text = kasprintf(GFP_KERNEL, "Updating FF_CONSTANT, level: %d, dir: %u, X: %d, Y: %d", level, effect->direction, x, y);
		break;
	}
	case FF_RUMBLE:
		text = kasprintf(GFP_KERNEL, "Updating FF_RUMBLE, strong: %u, weak: %u, direction: %s\n", effect->u.rumble.strong_magnitude,
				 effect->u.rumble.weak_magnitude, klgdff_combined_rumble_dir(effect->direction));
		break;
	default:
		text = kasprintf(GFP_KERNEL, "Updating, type %d, id %d", effect->type, effect->id);
		break;
	}

	len = strlen(text);
	c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	memcpy(c->bytes, text, len);
	kfree(text);
	return klgd_append_cmd(s, c);
}

static int klgdff_upload(struct klgd_command_stream *s, const struct ff_effect *effect)
{
	char *text = kasprintf(GFP_KERNEL, "Uploading effect, type %d, id %d", effect->type, effect->id);
	size_t len = strlen(text);
	struct klgd_command *c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	memcpy(c->bytes, text, len);
	kfree(text);
	return klgd_append_cmd(s, c);
}

static int klgdff_up_start(struct klgd_command_stream *s, const struct ff_effect *effect, const int repeat)
{
	char *text;
	size_t len;
	struct klgd_command *c;

	switch (effect->type) {
	case FF_CONSTANT:
	{
		s32 x;
		s32 y;
		s32 level = effect->u.constant.level * gain / 0xFFFF;

		ffpl_lvl_dir_to_x_y(level, effect->direction, &x, &y);
		text = kasprintf(GFP_KERNEL, "Uploading and starting FF_CONSTANT, level: %d, dir: %u, X: %d, Y: %d", level, effect->direction, x, y);
		break;
	}
	case FF_RUMBLE:
		text = kasprintf(GFP_KERNEL, "Uploading and starting FF_RUMBLE, strong: %u, weak: %u, direction: %s\n", effect->u.rumble.strong_magnitude,
				 effect->u.rumble.weak_magnitude, klgdff_combined_rumble_dir(effect->direction));
		break;
	default:
		text = kasprintf(GFP_KERNEL, "Uploading and starting effect, type %d, id %d, repeat %d", effect->type, effect->id, repeat);
		break;
	}

	len = strlen(text);
	c = klgd_alloc_cmd(len + 1);

	if (!c)
		return -ENOMEM;

	memcpy(c->bytes, text, len);
	kfree(text);
	return klgd_append_cmd(s, c);
}

int klgdff_callback(void *data, const struct klgd_command_stream *s)
{
	size_t idx;

	printk(KERN_NOTICE "KLGDTM - EFF...\n");
	for (idx = 0; idx < s->count; idx++) {
		printk(KERN_NOTICE "KLGDFF-TD: EFF %s\n", s->commands[idx]->bytes);
		if (s->commands[idx]->user.ldata[0])
			printk(KERN_NOTICE "KLGDFF-TD: User1 0x%X\n", s->commands[idx]->user.ldata[0]);
	}

	/* Simulate default USB polling rate of 125 Hz */
	/*usleep_range(7500, 8500);*/
	/* Long delay to test more complicated steps */
	usleep_range(25000, 35000);

	return 0;
}

int klgdff_control(struct input_dev *dev, struct klgd_command_stream *s, const enum ffpl_control_command cmd,
		   const union ffpl_control_data data, void *user)
{
	if (!s)
		return -EINVAL;

	printk(KERN_NOTICE "KLGDFF-TD: User data: 0x%X\n", *(u32 *)user);
	switch (cmd) {
	case FFPL_EMP_TO_UPL:
		return klgdff_upload(s, data.effects.cur);
		break;
	case FFPL_UPL_TO_SRT:
		return klgdff_start(s, data.effects.cur, data.effects.repeat);
		break;
	case FFPL_SRT_TO_UPL:
		return klgdff_stop(s, data.effects.cur);
		break;
	case FFPL_UPL_TO_EMP:
		return klgdff_erase(s, data.effects.cur);
		break;
	case FFPL_SRT_TO_UDT:
		return klgdff_update(s, data.effects.cur);
		break;
	/* "Uploadless/eraseless" commands */
	case FFPL_EMP_TO_SRT:
		return klgdff_up_start(s, data.effects.cur, data.effects.repeat);
		break;
	case FFPL_SRT_TO_EMP:
		return klgdff_er_stop(s, data.effects.cur);
		break;
	/* "Direct" replacing commands */
	case FFPL_OWR_TO_SRT:
		return klgdff_owr_start(s, data.effects.cur, data.effects.old, data.effects.repeat);
		break;
        case FFPL_OWR_TO_UPL:
		return klgdff_owr_upload(s, data.effects.cur, data.effects.old);
		break;
	case FFPL_SET_GAIN:
		return klgdff_set_gain(s, data.gain);
	case FFPL_SET_AUTOCENTER:
		return klgdff_set_autocenter(s, data.autocenter);
	default:
		printk(KERN_NOTICE "KLGDFF-TD - Unhandled command\n");
		break;
	}

	return 0;
}

static void __exit klgdff_exit(void)
{
	input_unregister_device(dev);
	klgd_deinit(&klgd);
	kobject_put(klgdff_obj);
	printk(KERN_NOTICE "KLGD FF sample module removed\n");
}

static int __init klgdff_init(void)
{
	int ret;

	klgdff_obj = kobject_create_and_add("klgdff_obj", kernel_kobj);
	if (!klgdff_obj)
		return -ENOMEM;


	ret = klgd_init(&klgd, NULL, klgdff_callback, 1);
	if (ret) {
		printk(KERN_ERR "KLGDFF-TD: Cannot initialize KLGD\n");
		goto errout_klgd;
	}

	dev = input_allocate_device();
	if (!dev) {
		ret = -ENODEV;
		printk(KERN_ERR "KLGDFF-TD: Cannot allocate input device\n");
		goto errout_idev;
	}
	dev->id.bustype = BUS_VIRTUAL;
	dev->id.vendor = 0xffff;
	dev->id.product = 0x8807;
	dev->id.version = 0x8807;
	dev->name = kasprintf(GFP_KERNEL, "KLGD-FF TestModule");
	dev->uniq = kasprintf(GFP_KERNEL, "KLGD-FF TestModule-X");
	dev->dev.parent = NULL;
	gain = 0xFFFF;

	input_set_capability(dev, EV_FF, FF_CONSTANT);
	input_set_capability(dev, EV_FF, FF_RUMBLE);
	input_set_capability(dev, EV_FF, FF_PERIODIC);
		input_set_capability(dev, EV_FF, FF_SINE);
		input_set_capability(dev, EV_FF, FF_SQUARE);
		input_set_capability(dev, EV_FF, FF_SAW_UP);
		input_set_capability(dev, EV_FF, FF_SAW_DOWN);
		input_set_capability(dev, EV_FF, FF_TRIANGLE);
	input_set_capability(dev, EV_FF, FF_RAMP);
	input_set_capability(dev, EV_FF, FF_SPRING);

	input_set_capability(dev, EV_ABS, ABS_X);
	input_set_capability(dev, EV_ABS, ABS_Y);
	input_set_capability(dev, EV_KEY, BTN_0);
	input_set_capability(dev, EV_KEY, BTN_TRIGGER);
	input_set_capability(dev, EV_FF, FF_AUTOCENTER);
	input_set_abs_params(dev, ABS_X, -0x7fff, 0x7fff, 0, 0);
	input_set_abs_params(dev, ABS_Y, -0x7fff, 0x7fff, 0, 0);

	ret = ffpl_init_plugin(&ff_plugin, dev, EFFECT_COUNT,
			       FFPL_HAS_EMP_TO_SRT | FFPL_REPLACE_STARTED |
			       FFPL_MEMLESS_CONSTANT |
			       FFPL_MEMLESS_PERIODIC |
			       FFPL_MEMLESS_RUMBLE |
			       FFPL_TIMING_CONDITION,
			       klgdff_control, &test_user);
	if (ret) {
		printk(KERN_ERR "KLGDFF-TD: Cannot init plugin\n");
		goto errout_idev;
	}
	ret = input_register_device(dev);
	if (ret) {
		printk(KERN_ERR "KLGDFF-TD: Cannot register input device\n");
		goto errout_regdev;
	}
	
	ret = klgd_register_plugin(&klgd, 0, ff_plugin, true);
	if (ret) {
		printk(KERN_ERR "KLGDFF-TD: Cannot register plugin\n");
		goto errout_idev;
	}



	printk(KERN_NOTICE "KLGDFF-TD: Sample module loaded\n");
	return 0;

errout_regdev:
	input_free_device(dev);
errout_idev:
	klgd_deinit(&klgd);
errout_klgd:
	kobject_put(klgdff_obj);
	return ret;
}

module_exit(klgdff_exit)
module_init(klgdff_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michal \"MadCatX\" Maly");
MODULE_DESCRIPTION("KLGD FF TestModule");


