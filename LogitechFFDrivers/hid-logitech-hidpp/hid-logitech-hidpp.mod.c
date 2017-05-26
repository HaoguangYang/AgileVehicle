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
	{ 0x3df5362e, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x561ed4e1, __VMLINUX_SYMBOL_STR(param_ops_bool) },
	{ 0x5a4c4431, __VMLINUX_SYMBOL_STR(hid_unregister_driver) },
	{ 0x9d6a67cb, __VMLINUX_SYMBOL_STR(__hid_register_driver) },
	{ 0xbdf3a0bf, __VMLINUX_SYMBOL_STR(device_remove_file) },
	{ 0x8c03d20c, __VMLINUX_SYMBOL_STR(destroy_workqueue) },
	{ 0xf1aab468, __VMLINUX_SYMBOL_STR(hid_disconnect) },
	{ 0x88bfa7e, __VMLINUX_SYMBOL_STR(cancel_work_sync) },
	{ 0x6dc6dd56, __VMLINUX_SYMBOL_STR(down) },
	{ 0x78e739aa, __VMLINUX_SYMBOL_STR(up) },
	{ 0xac324c7e, __VMLINUX_SYMBOL_STR(hid_connect) },
	{ 0x37a5df8f, __VMLINUX_SYMBOL_STR(hid_open_report) },
	{ 0x9e88526, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0x17d070c2, __VMLINUX_SYMBOL_STR(__mutex_init) },
	{ 0xc9e537a9, __VMLINUX_SYMBOL_STR(devm_kmalloc) },
	{ 0x43a53735, __VMLINUX_SYMBOL_STR(__alloc_workqueue_key) },
	{ 0xf733060, __VMLINUX_SYMBOL_STR(device_create_file) },
	{ 0xfd2e9caa, __VMLINUX_SYMBOL_STR(input_ff_create) },
	{ 0x8148229a, __VMLINUX_SYMBOL_STR(input_free_device) },
	{ 0x84552a6f, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0xdaf16e7c, __VMLINUX_SYMBOL_STR(devm_kasprintf) },
	{ 0x466d08ce, __VMLINUX_SYMBOL_STR(input_register_device) },
	{ 0xfce66f14, __VMLINUX_SYMBOL_STR(devm_input_allocate_device) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0x20000329, __VMLINUX_SYMBOL_STR(simple_strtoul) },
	{ 0x7cd1e218, __VMLINUX_SYMBOL_STR(dev_warn) },
	{ 0x63445f1, __VMLINUX_SYMBOL_STR(__dynamic_dev_dbg) },
	{ 0xd6aa1fed, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0xf8a96516, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x4906a967, __VMLINUX_SYMBOL_STR(hid_snto32) },
	{ 0xbb481c37, __VMLINUX_SYMBOL_STR(hid_field_extract) },
	{ 0xd33ee5d5, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x6e7d3f97, __VMLINUX_SYMBOL_STR(input_mt_report_slot_state) },
	{ 0x70d1bd4e, __VMLINUX_SYMBOL_STR(input_mt_get_slot_by_key) },
	{ 0x26074d82, __VMLINUX_SYMBOL_STR(input_event) },
	{ 0x250a9873, __VMLINUX_SYMBOL_STR(input_mt_sync_frame) },
	{ 0x2e0d2f7f, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0x2d3385d3, __VMLINUX_SYMBOL_STR(system_wq) },
	{ 0xa6bbd805, __VMLINUX_SYMBOL_STR(__wake_up) },
	{ 0xb59dad5f, __VMLINUX_SYMBOL_STR(input_mt_init_slots) },
	{ 0x1b65c16b, __VMLINUX_SYMBOL_STR(input_set_capability) },
	{ 0x5ea4c414, __VMLINUX_SYMBOL_STR(input_alloc_absinfo) },
	{ 0x368807e2, __VMLINUX_SYMBOL_STR(input_set_abs_params) },
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
