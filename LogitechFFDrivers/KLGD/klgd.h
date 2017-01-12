/**
 *
 * struct klgd_command
 * @bytes: Payload
 * @length: Number of bytes
 * @user: Arbitrary data that are not to be sent to the device.
 *	  Note that any data pointed to by the ptr will not
 *	  be free'd automatically when the command stream gets
 *	  disposed of.
 */
struct klgd_command {
	__u8 * const bytes;
	size_t length;
	union {
		u32 ldata[2];
		u16 sdata[4];
		u8  cdata[8];
		void *ptr;
	} user;
};

struct klgd_command_stream {
	const struct klgd_command **commands;
	size_t count;
};

struct klgd_main {
	struct klgd_main_private *private;
};

struct klgd_plugin {
	struct klgd_plugin_private *private;
	struct mutex *plugins_lock;

	void (*deinit)(struct klgd_plugin *ctx);
	int (*get_commands)(struct klgd_plugin *ctx, struct klgd_command_stream **s, const unsigned long now);
	bool (*get_update_time)(struct klgd_plugin *ctx, const unsigned long now, unsigned long *t);
	int (*init)(struct klgd_plugin *ctx);
};

struct klgd_command * klgd_alloc_cmd(const size_t length);
struct klgd_command_stream * klgd_alloc_stream(void);
int klgd_append_cmd(struct klgd_command_stream *target, const struct klgd_command *cmd);
int klgd_append_stream(struct klgd_command_stream *target, const struct klgd_command_stream *source);
void klgd_deinit(struct klgd_main *ctx);
void klgd_free_command(const struct klgd_command *cmd);
void klgd_free_stream(struct klgd_command_stream *s);
int klgd_init(struct klgd_main *ctx, void *dev_ctx, int (*callback)(void *, const struct klgd_command_stream *), const unsigned long plugin_count);
void klgd_lock_plugins(struct mutex *lock);
struct klgd_command * klgd_make_cmd(const char * const bytes);
int klgd_register_plugin(struct klgd_main *ctx, const size_t idx, struct klgd_plugin *plugin, bool dontfree);
void klgd_unlock_plugins(struct mutex *lock);
void klgd_unlock_plugins_sched(struct mutex *lock);
