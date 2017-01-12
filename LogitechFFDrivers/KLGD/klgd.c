#include <asm/atomic.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include <linux/workqueue.h>
#include "klgd.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Michal \"MadCatX\" Maly");
MODULE_DESCRIPTION("Pluginable framework of helper functions to handle gaming devices");

#define TRYAGAIN_DELAY msecs_to_jiffies(10)

struct klgd_main_private {
	struct delayed_work work;
	void *device_context;
	unsigned long earliest_update;
	size_t plugin_count;
	struct klgd_plugin **plugins;
	bool *dontfree;
	struct mutex plugins_lock;
	struct mutex send_lock;
	struct workqueue_struct *wq;
	bool endpoint_dead;
	bool try_again;

	int (*send_command_stream)(void *dev_ctx, const struct klgd_command_stream *stream);
};

static void klgd_schedule_update(struct klgd_main_private *priv);

struct klgd_command * klgd_alloc_cmd(const size_t length)
{
	struct klgd_command *cmd = kzalloc(sizeof(struct klgd_command), GFP_KERNEL);
	if (!cmd)
		return NULL;

	/* Cast away the const-ness */
	*(char **)(&cmd->bytes) = kzalloc(sizeof(char) * length, GFP_KERNEL);
	if (!cmd->bytes) {
		kfree(cmd);
		return NULL;
	}

	cmd->length = length;
	return cmd;
}
EXPORT_SYMBOL_GPL(klgd_alloc_cmd);

struct klgd_command_stream * klgd_alloc_stream(void)
{
	return kzalloc(sizeof(struct klgd_command_stream), GFP_KERNEL);
}
EXPORT_SYMBOL_GPL(klgd_alloc_stream);	

int klgd_append_cmd(struct klgd_command_stream *target, const struct klgd_command *cmd)
{
	const struct klgd_command **temp;

	if (!target) {
		printk(KERN_NOTICE "Cannot append to NULL stream\n");
		return -EINVAL;
	}
	if (!cmd) {
		printk(KERN_NOTICE "Cannot append NULL cmd\n");
		return -EINVAL;
	}

	temp = krealloc(target->commands, sizeof(struct klgd_command *) * (target->count + 1), GFP_KERNEL);
	if (!temp)
		return -ENOMEM;

	target->commands = temp;
	target->commands[target->count] = cmd;
	target->count++;

	return 0;
}
EXPORT_SYMBOL_GPL(klgd_append_cmd);


int klgd_append_stream(struct klgd_command_stream *target, const struct klgd_command_stream *source)
{
	const struct klgd_command **temp;
	size_t idx;

	/* We got NULL command, skip it */
	if (!source)
		return 0;
	/* We got command of zero length, skip it */
	if (!source->count)
		return 0;

	temp = krealloc(target->commands, sizeof(struct klgd_command *) * (target->count + source->count), GFP_KERNEL);
	if (!temp)
		return -ENOMEM;

	target->commands = temp;
	for (idx = 0; idx < source->count; idx++)
		target->commands[idx + target->count] = source->commands[idx];
	target->count += source->count;

	return 0;
}
EXPORT_SYMBOL_GPL(klgd_append_stream);

/**
 * Called with plugins_lock held
 */
int klgd_build_command_stream(struct klgd_main_private *priv, struct klgd_command_stream **s)
{
	const unsigned long now = jiffies;
	int ret = 0;
	size_t idx;

	*s = kzalloc(sizeof(struct klgd_command_stream), GFP_KERNEL);
	if (!s)
		return -EAGAIN;

	for (idx = 0; idx < priv->plugin_count; idx++) {
		struct klgd_plugin *plugin = priv->plugins[idx];
		struct klgd_command_stream *ss;

		ret = plugin->get_commands(plugin, &ss, now);
		/* Something happened while the plugin was building the command stream
		 * but the error is recoverable. Send the incomplete command stream
		 * and try again ASAP. */
		if (ret == -EAGAIN)
			break;

		/* Unrecoverable error, ditch the stream and bail out. */
		if (ret) {
			klgd_free_stream(ss);
			klgd_free_stream(*s);
			return -EFAULT;
		}

		if (klgd_append_stream(*s, ss)) {
			/* We could not append the stream for the current plugin to the total stream.
			 * This is bad because the plugin might have changed its internal state so we
			 * cannot just ask it for that stream again. Cowardly bailing out is the only
			 * safe thing to do. */
			klgd_free_stream(ss);
			klgd_free_stream(*s);
			return -EFAULT;
		}
	}

	if ((*s)->count) {
		printk(KERN_NOTICE "KLGD: Command stream built\n");
		return ret;
	}
	printk(KERN_NOTICE "KLGD: Command stream is empty\n");
	return -ENOENT;
}

static void klgd_delayed_work(struct work_struct *w)
{
	struct delayed_work *dw = container_of(w, struct delayed_work, work);
	struct klgd_main_private *priv = container_of(dw, struct klgd_main_private, work);
	struct klgd_command_stream *s;
	unsigned long now;
	int ret;

	printk(KERN_NOTICE "KLGD/WQ: --- WQ begins ---\n");
	mutex_lock(&priv->send_lock);
	if (priv->endpoint_dead) {
		printk(KERN_ERR "KLGD/WQ: Endpoint marked as dead, aborting\n");
		mutex_unlock(&priv->send_lock);
		return;
	}
	priv->try_again = false;

	printk(KERN_NOTICE "KLGD/WQ: Timer fired and send_lock acquired\n");

	mutex_lock(&priv->plugins_lock);
	printk(KERN_NOTICE "KLGD/WQ: Plugins state locked - building command stream\n");
	ret = klgd_build_command_stream(priv, &s);
	printk(KERN_NOTICE "KLGD/WQ: Command stream built\n");

	switch (ret) {
	case -EAGAIN:
		/* Unable to build complete command stream right now, try again */
		printk(KERN_WARNING "KLGD: Sending incomplete command stream and scheduling a new update ASAP.\n");
		priv->try_again = true;
		break;
	case -ENOENT:
		/* Empty command stream. Plugins have no work for us, exit */
		mutex_unlock(&priv->plugins_lock);
		printk(KERN_NOTICE "KLGD/WQ: Plugins state unlocked\n");
		goto out;
	case 0:
		break;
	default:
		/* Unrecoverable error, consider the endpoint dead */
		printk(KERN_ERR "KLGD: Unrecoverable error while building command stream, ret code %d. No further commands will be sent to the endpoint.\n", ret);
		priv->endpoint_dead = true;
		mutex_unlock(&priv->plugins_lock);
		mutex_unlock(&priv->send_lock);
		return;
	}
	mutex_unlock(&priv->plugins_lock);
	printk(KERN_NOTICE "KLGD/WQ: Plugins state unlocked\n");

	now = jiffies;
	ret = priv->send_command_stream(priv->device_context, s);
	if (ret) {
		printk(KERN_ERR "KLGD/WQ: Unable to send command stream, ret code %d. Marking endpoint as dead.\n", ret);
		priv->endpoint_dead = true;
		mutex_unlock(&priv->send_lock);
		return;
	} else
		printk(KERN_NOTICE "KLGD/WQ: Commands sent, time elapsed %u [msec]\n", jiffies_to_msecs(jiffies - now));
	kfree(s);

out:
	mutex_unlock(&priv->send_lock);

	if (priv->try_again) {
		queue_delayed_work(priv->wq, &priv->work, TRYAGAIN_DELAY);
		return;
	}

	/* We're done submitting, check if there is some work for us in the future */
	mutex_lock(&priv->plugins_lock);
	printk(KERN_NOTICE "KLGD/WQ: Plugins state unlocked - checking if there is more to do in the future\n");
	klgd_schedule_update(priv);
	mutex_unlock(&priv->plugins_lock);
	printk(KERN_NOTICE "KLGD/WQ: Plugins state unlocked - scheduling complete\n");
	printk(KERN_NOTICE "KLGD/WQ: --- WQ complete ---\n");
}

void klgd_free_command(const struct klgd_command *cmd)
{
	if (cmd) {
		kfree(cmd->bytes);
		kfree(cmd);
	}
}
EXPORT_SYMBOL_GPL(klgd_free_command);

void klgd_free_stream(struct klgd_command_stream *s)
{
	size_t idx;

	if (!s)
		return;

	for (idx = 0; idx < s->count; idx++)
		klgd_free_command(s->commands[idx]);
}
EXPORT_SYMBOL_GPL(klgd_free_stream);

void klgd_deinit(struct klgd_main *ctx)
{
	struct klgd_main_private *priv = ctx->private;
	size_t idx;

	if (!ctx)
		return;
	if (!ctx->private)
		return;
	priv = ctx->private;

	cancel_delayed_work(&priv->work);
	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);
	printk(KERN_NOTICE "KLGD deinit, workqueue terminated\n");

	for (idx = 0; idx < priv->plugin_count; idx++) {
		struct klgd_plugin *plugin = priv->plugins[idx];

		if (!plugin)
			continue;
		if (priv->dontfree[idx])
			continue;

		if (plugin->deinit)
			plugin->deinit(plugin);
		kfree(plugin);
	}
	kfree(priv->plugins);
	kfree(priv->dontfree);

	kfree(priv);
}
EXPORT_SYMBOL_GPL(klgd_deinit);

int klgd_init(struct klgd_main *ctx, void *dev_ctx, int (*callback)(void *, const struct klgd_command_stream *), const size_t plugin_count)
{
	struct klgd_main_private *priv = ctx->private;
	int ret;

	if (!ctx)
		return -EINVAL;
	if (plugin_count < 1)
		return -EINVAL;
	if (!callback)
		return -EINVAL;

	priv = kzalloc(sizeof(struct klgd_main_private), GFP_KERNEL);
	if (!priv) {
		printk(KERN_ERR "No memory for KLGD data\n");
		return -ENOMEM;
	}

	mutex_init(&priv->plugins_lock);
	mutex_init(&priv->send_lock);
	priv->wq = create_singlethread_workqueue("klgd_processing_loop");
	INIT_DELAYED_WORK(&priv->work, klgd_delayed_work);

	priv->plugins = kzalloc(sizeof(struct klgd_plugin *) * plugin_count, GFP_KERNEL);
	if (!priv->plugins) {
		printk(KERN_ERR "No memory for KLGD plugins\n");
		ret = -ENOMEM;
		goto err_out;
	}
	priv->dontfree = kzalloc(sizeof(bool) * plugin_count, GFP_KERNEL);
	if (!priv->dontfree) {
		printk(KERN_ERR "No memory for plugin quirks\n");
		ret = -ENOMEM;
		goto err_out2;
	}
	priv->plugin_count = plugin_count;

	priv->device_context = dev_ctx;
	priv->send_command_stream = callback;

	ctx->private = priv;
	return 0;

err_out2:
	kfree(priv->plugins);
err_out:
	destroy_workqueue(priv->wq);
	kfree(ctx->private);

	return ret;
}
EXPORT_SYMBOL_GPL(klgd_init);

void klgd_lock_plugins(struct mutex *lock)
{
	mutex_lock(lock);
	printk(KERN_DEBUG "KLGD: Plugins state locked\n");
}
EXPORT_SYMBOL_GPL(klgd_lock_plugins);

struct klgd_command * klgd_make_command(const char * const bytes, const size_t length)
{
	struct klgd_command *cmd = kzalloc(sizeof(struct klgd_command), GFP_KERNEL);
	if (!cmd)
		return NULL;

	*(const char **)(&cmd->bytes) = bytes;
	cmd->length = length;

	return cmd;
}
EXPORT_SYMBOL_GPL(klgd_make_command);

int klgd_register_plugin(struct klgd_main *ctx, size_t idx, struct klgd_plugin *plugin, bool dontfree)
{
	struct klgd_main_private *priv = ctx->private;

	if (priv->plugins[idx])
		return -EINVAL;

	plugin->plugins_lock = &priv->plugins_lock;
	priv->plugins[idx] = plugin;
	priv->dontfree[idx] = dontfree;
	if (plugin->init)
	      return plugin->init(plugin);

	return 0;
}
EXPORT_SYMBOL_GPL(klgd_register_plugin);

void klgd_unlock_plugins(struct mutex *lock)
{
	mutex_unlock(lock);
	printk(KERN_DEBUG "KLGD: Plugins state unlocked, NOT scheduled\n");
}
EXPORT_SYMBOL_GPL(klgd_unlock_plugins);


void klgd_unlock_plugins_sched(struct mutex *lock)
{
	struct klgd_main_private *priv = container_of(lock, struct klgd_main_private, plugins_lock);

	klgd_schedule_update(priv);
	mutex_unlock(lock);
	printk(KERN_DEBUG "KLGD: Plugins state unlocked, rescheduled\n");
}
EXPORT_SYMBOL_GPL(klgd_unlock_plugins_sched);

static void klgd_schedule_update(struct klgd_main_private *priv)
{
	const unsigned long now = jiffies;
	unsigned int events = 0;
	unsigned long earliest;
	size_t idx;

	/* Recovery update is scheduled. Do not allow any other updates until the recovery is done.
	 * Also do not send updates to a dead endpoint */
	if (priv->try_again || priv->endpoint_dead)
		return;

	for (idx = 0; idx < priv->plugin_count; idx++) {
		struct klgd_plugin *plugin = priv->plugins[idx];
		unsigned long t;

		if (plugin->get_update_time(plugin, now, &t)) {
			if (!events)
				earliest = t;
			else if (time_before(t, earliest))
				earliest = t;
			events++;
		}
	}

	if (!events) {
		bool ret;
		printk(KERN_NOTICE "No events, deactivating timer\n");
		ret = cancel_delayed_work(&priv->work);
		if (ret)
			printk(KERN_NOTICE "KLGD: Work canceled\n");
		else
			printk(KERN_NOTICE "KLGD: There was no work to cancel\n");
	} else {
		printk(KERN_NOTICE "Events: %u, earliest: %lu, now: %lu\n", events, earliest, now);
		if (time_before(earliest, now))
			printk(KERN_WARNING "KLGD: Time of earliest update is in the past. Is is probably caused by a buggy plugin. No update scheduled.\n");
		else {
			int ret = queue_delayed_work(priv->wq, &priv->work, earliest - now);
			if (!ret) {
				printk(KERN_NOTICE "KLGD: Work was already on the queue\n");
			}
		}
	}
}
