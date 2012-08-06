#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

struct module __this_module
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
	{ 0x60e37956, "module_layout" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0xf9a482f9, "msleep" },
	{ 0xff178f6, "__aeabi_idivmod" },
	{ 0xfbc74f64, "__copy_from_user" },
	{ 0x67c2fa54, "__copy_to_user" },
	{ 0x2e5810c6, "__aeabi_unwind_cpp_pr1" },
	{ 0x6a9cac4c, "dev_set_drvdata" },
	{ 0x2053f0b0, "i2c_del_driver" },
	{ 0xc8b57c27, "autoremove_wake_function" },
	{ 0x8728140, "malloc_sizes" },
	{ 0xceb5aa35, "i2c_transfer" },
	{ 0xc633495b, "schedule_work" },
	{ 0x43ab66c3, "param_array_get" },
	{ 0xb224fbe2, "param_get_short" },
	{ 0x45947727, "param_array_set" },
	{ 0xfb58121b, "__init_waitqueue_head" },
	{ 0x41344088, "param_get_charp" },
	{ 0xe707d823, "__aeabi_uidiv" },
	{ 0x95c1810b, "misc_register" },
	{ 0xfa2a45e, "__memzero" },
	{ 0xb5eeb329, "register_early_suspend" },
	{ 0x5f754e5a, "memset" },
	{ 0xea147363, "printk" },
	{ 0x71c90087, "memcmp" },
	{ 0xbb72d4fe, "__put_user_1" },
	{ 0x859c6dc7, "request_threaded_irq" },
	{ 0x2196324, "__aeabi_idiv" },
	{ 0x74fb9fe6, "i2c_register_driver" },
	{ 0xdc74cc24, "kmem_cache_alloc" },
	{ 0xd62c833f, "schedule_timeout" },
	{ 0x30eb966a, "dev_driver_string" },
	{ 0xe8fa4732, "i2c_master_recv" },
	{ 0x6ad065f4, "param_set_charp" },
	{ 0xb6c70a7d, "__wake_up" },
	{ 0x1d2e87c6, "do_gettimeofday" },
	{ 0x37a0cba, "kfree" },
	{ 0x9d669763, "memcpy" },
	{ 0x801678, "flush_scheduled_work" },
	{ 0x8085c7b1, "prepare_to_wait" },
	{ 0x4333eadb, "param_set_short" },
	{ 0x51493d94, "finish_wait" },
	{ 0xb227ae83, "unregister_early_suspend" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
	{ 0x3ab4c874, "i2c_get_adapter" },
	{ 0xad324b0b, "dev_get_drvdata" },
	{ 0xa15ad53a, "misc_deregister" },
	{ 0xf20dabd8, "free_irq" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("i2c:mpu3050");
