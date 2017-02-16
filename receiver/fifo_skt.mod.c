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
	{ 0x465ae224, "module_layout" },
	{ 0x6d044c26, "param_ops_uint" },
	{ 0xfe990052, "gpio_free" },
	{ 0xe61a6d2f, "gpio_unexport" },
	{ 0x432fd7f6, "__gpio_set_value" },
	{ 0xc2165d85, "__arm_iounmap" },
	{ 0xcb4a1f12, "rtdm_timer_destroy" },
	{ 0xf522ec3, "kthread_stop" },
	{ 0xdb760f52, "__kfifo_free" },
	{ 0xa180cf7a, "sock_release" },
	{ 0xa2577d47, "rtdm_tbase" },
	{ 0x4bbcb0b1, "wake_up_process" },
	{ 0x9a1a1249, "kthread_create_on_node" },
	{ 0x35525017, "rtdm_timer_start" },
	{ 0x328a05f1, "strncpy" },
	{ 0xa21a3d82, "__xntimer_init" },
	{ 0xc068440e, "__kfifo_alloc" },
	{ 0x40a6f522, "__arm_ioremap" },
	{ 0x65d6d0f0, "gpio_direction_input" },
	{ 0x82f776b7, "gpio_export" },
	{ 0xa8f59416, "gpio_direction_output" },
	{ 0x47229b5c, "gpio_request" },
	{ 0xb3f7646e, "kthread_should_stop" },
	{ 0x91715312, "sprintf" },
	{ 0x13d0adf7, "__kfifo_out" },
	{ 0xf9a482f9, "msleep" },
	{ 0x81915303, "sock_sendmsg" },
	{ 0x1b6314fd, "in_aton" },
	{ 0xfa2a45e, "__memzero" },
	{ 0xf91bed4f, "sock_create" },
	{ 0x27e1a049, "printk" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
	{ 0xac8f37b2, "outer_cache" },
	{ 0xf23fcb99, "__kfifo_in" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "87A83439DB77B5FC5335A96");
