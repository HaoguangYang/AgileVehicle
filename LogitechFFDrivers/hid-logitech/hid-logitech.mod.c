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
	{ 0xc6c01fa, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x5f31316b, __VMLINUX_SYMBOL_STR(device_remove_file) },
	{ 0xb4606292, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0xd2b09ce5, __VMLINUX_SYMBOL_STR(__kmalloc) },
	{ 0x37d9b65, __VMLINUX_SYMBOL_STR(klgd_alloc_cmd) },
	{ 0x526f90ad, __VMLINUX_SYMBOL_STR(hid_open_report) },
	{ 0x16abfc10, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0x754d539c, __VMLINUX_SYMBOL_STR(strlen) },
	{ 0x8c96afc5, __VMLINUX_SYMBOL_STR(led_classdev_register) },
	{ 0xe05983db, __VMLINUX_SYMBOL_STR(klgd_append_cmd) },
	{ 0x79aa04a2, __VMLINUX_SYMBOL_STR(get_random_bytes) },
	{ 0xd2fdf5a1, __VMLINUX_SYMBOL_STR(input_ff_create_memless) },
	{ 0x20000329, __VMLINUX_SYMBOL_STR(simple_strtoul) },
	{ 0xd46984a9, __VMLINUX_SYMBOL_STR(__hid_request) },
	{ 0xd235dd43, __VMLINUX_SYMBOL_STR(ffpl_lvl_dir_to_x_y) },
	{ 0x5495392, __VMLINUX_SYMBOL_STR(hid_debug) },
	{ 0xe2d5255a, __VMLINUX_SYMBOL_STR(strcmp) },
	{ 0x733c3b54, __VMLINUX_SYMBOL_STR(kasprintf) },
	{ 0xe925b8b4, __VMLINUX_SYMBOL_STR(input_event) },
	{ 0x9e88526, __VMLINUX_SYMBOL_STR(__init_waitqueue_head) },
	{ 0x49aae39c, __VMLINUX_SYMBOL_STR(klgd_register_plugin) },
	{ 0xfb578fc5, __VMLINUX_SYMBOL_STR(memset) },
	{ 0x7923106, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x1916e38c, __VMLINUX_SYMBOL_STR(_raw_spin_unlock_irqrestore) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xa1c76e0a, __VMLINUX_SYMBOL_STR(_cond_resched) },
	{ 0xec0c99a9, __VMLINUX_SYMBOL_STR(hid_disconnect) },
	{ 0x91ce3c98, __VMLINUX_SYMBOL_STR(device_create_file) },
	{ 0x31481d0b, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0x51216919, __VMLINUX_SYMBOL_STR(hid_connect) },
	{ 0x8e1906d4, __VMLINUX_SYMBOL_STR(klgd_init) },
	{ 0xdb7305a1, __VMLINUX_SYMBOL_STR(__stack_chk_fail) },
	{ 0xd62c833f, __VMLINUX_SYMBOL_STR(schedule_timeout) },
	{ 0xbdfb6dbb, __VMLINUX_SYMBOL_STR(__fentry__) },
	{ 0xb7afaf64, __VMLINUX_SYMBOL_STR(kmem_cache_alloc_trace) },
	{ 0x680ec266, __VMLINUX_SYMBOL_STR(_raw_spin_lock_irqsave) },
	{ 0x2207a57f, __VMLINUX_SYMBOL_STR(prepare_to_wait_event) },
	{ 0xc60ed5f7, __VMLINUX_SYMBOL_STR(led_classdev_unregister) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xaf3b160, __VMLINUX_SYMBOL_STR(__hid_register_driver) },
	{ 0x4ca9669f, __VMLINUX_SYMBOL_STR(scnprintf) },
	{ 0xf08242c2, __VMLINUX_SYMBOL_STR(finish_wait) },
	{ 0x79acebb, __VMLINUX_SYMBOL_STR(dev_warn) },
	{ 0xe7b30563, __VMLINUX_SYMBOL_STR(klgd_deinit) },
	{ 0x28318305, __VMLINUX_SYMBOL_STR(snprintf) },
	{ 0x9bf96efc, __VMLINUX_SYMBOL_STR(hid_validate_values) },
	{ 0xc15db649, __VMLINUX_SYMBOL_STR(hid_unregister_driver) },
	{ 0x82a02ad4, __VMLINUX_SYMBOL_STR(ffpl_init_plugin) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=klgd,hid,ff-memless,klgd_ff_plugin";

MODULE_ALIAS("hid:b0003g*v0000046Dp0000C513");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C50C");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C517");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C101");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C704");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C714");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C71F");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C30A");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C512");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C215");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C216");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C294");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C20A");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C211");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C219");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C283");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C286");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C295");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000CA03");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000CA04");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C299");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C29A");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C29B");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C298");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C29C");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C24F");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C293");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C218");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C287");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C626");
MODULE_ALIAS("hid:b0003g*v0000046Dp0000C623");

MODULE_INFO(srcversion, "AB9BBC136EC28F616648DDC");
