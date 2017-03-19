#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x9d35aeec, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x561ed4e1, __VMLINUX_SYMBOL_STR(param_ops_bool) },
	{ 0xce13da85, __VMLINUX_SYMBOL_STR(hid_unregister_driver) },
	{ 0x8cafd792, __VMLINUX_SYMBOL_STR(__hid_register_driver) },
	{ 0x35076a58, __VMLINUX_SYMBOL_STR(device_remove_file) },
	{ 0x8c03d20c, __VMLINUX_SYMBOL_STR(destroy_workqueue) },
	{ 0x102a9e39, __VMLINUX_SYMBOL_STR(hid_disconnect) },
	{ 0x88bfa7e, __VMLINUX_SYMBOL_STR(cancel_work_sync) },
	{ 0x6dc6dd56, __VMLINUX_SYMBOL_STR(down) },
	{ 0x78e739aa, __VMLINUX_SYMBOL_STR(up) },
	{ 0x7c8f2713, __VMLINUX_SYMBOL_STR(hid_connect) },
	{ 0xa71e8daf, __VMLINUX_SYMBOL_STR(hid_open_report) },
	{ 0x9e88526, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0x17d070c2, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0xc9e537a9, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0x43a53735, __VMLINUX_SYMBOL_STR(__alloc_workqueue_key) },
	{ 0xc4a09274, __VMLINUX_SYMBOL_STR(device_create_file) },
	{ 0x47bec294, __VMLINUX_SYMBOL_STR(input_ff_create) },
	{ 0xc457c40d, __VMLINUX_SYMBOL_STR(input_free_device) },
	{ 0x15d8247d, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0xdaf16e7c, __VMLINUX_SYMBOL_STR(devm_kasprintf) },
	{ 0xeea3a725, __VMLINUX_SYMBOL_STR(input_register_device) },
	{ 0x31e76b0b, __VMLINUX_SYMBOL_STR(devm_input_allocate_device) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0x20000329, __VMLINUX_SYMBOL_STR(simple_strtoul) },
	{ 0x1ff5d0e2, __VMLINUX_SYMBOL_STR(dev_warn) },
	{ 0x51d204db, __VMLINUX_SYMBOL_STR(__dynamic_dev_dbg) },
	{ 0x2142697b, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x8a9809d6, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x4906a967, __VMLINUX_SYMBOL_STR(hid_snto32) },
	{ 0xa810c198, __VMLINUX_SYMBOL_STR(hid_field_extract) },
	{ 0x17b099d2, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0xe9963c90, __VMLINUX_SYMBOL_STR(input_mt_report_slot_state) },
	{ 0xb55cf82b, __VMLINUX_SYMBOL_STR(input_mt_get_slot_by_key) },
	{ 0x683c37ce, __VMLINUX_SYMBOL_STR(input_event) },
	{ 0xd1289e88, __VMLINUX_SYMBOL_STR(input_mt_sync_frame) },
	{ 0x2e0d2f7f, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0xa6bbd805, __VMLINUX_SYMBOL_STR(__wake_up) },
	{ 0xab7d7b5a, __VMLINUX_SYMBOL_STR(input_mt_init_slots) },
	{ 0x43d5a81c, __VMLINUX_SYMBOL_STR(input_set_capability) },
	{ 0x23788623, __VMLINUX_SYMBOL_STR(input_alloc_absinfo) },
	{ 0x5d2292e8, __VMLINUX_SYMBOL_STR(input_set_abs_params) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xf08242c2, __VMLINUX_SYMBOL_STR(finish_wait) },
	{ 0xd62c833f, __VMLINUX_SYMBOL_STR(schedule_timeout) },
	{ 0x2207a57f, __VMLINUX_SYMBOL_STR(prepare_to_wait_event) },
	{ 0xa1c76e0a, __VMLINUX_SYMBOL_STR(_cond_resched) },
	{ 0x347cd1b3, __VMLINUX_SYMBOL_STR(mutex_unlock) },
	{ 0x5495392, __VMLINUX_SYMBOL_STR(hid_debug) },
	{ 0x11761f56, __VMLINUX_SYMBOL_STR(mutex_lock) },
	{ 0x4ca9669f, __VMLINUX_SYMBOL_STR(scnprintf) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=hid";

MODULE_ALIAS("hid:b0003g0102v0000046Dp00004011");
MODULE_ALIAS("hid:b0003g0102v0000046Dp00004101");
MODULE_ALIAS("hid:b0005g*v0000046Dp0000B00C");
MODULE_ALIAS("hid:b0003g0102v0000046Dp0000402D");
MODULE_ALIAS("hid:b0003g0102v0000046Dp00004024");
MODULE_ALIAS("hid:b0003g0102v0000046Dp*");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C262");

MODULE_INFO(srcversion, "EBDE2E6B421C82AF68E4559");
