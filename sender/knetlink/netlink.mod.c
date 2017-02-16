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
	{ 0xfe990052, "gpio_free" },
	{ 0xc2165d85, "__arm_iounmap" },
	{ 0x4624ee01, "netlink_kernel_release" },
	{ 0xcb4a1f12, "rtdm_timer_destroy" },
	{ 0xa2577d47, "rtdm_tbase" },
	{ 0x516d7c9b, "init_net" },
	{ 0x35525017, "rtdm_timer_start" },
	{ 0x328a05f1, "strncpy" },
	{ 0xa21a3d82, "__xntimer_init" },
	{ 0x40a6f522, "__arm_ioremap" },
	{ 0x65d6d0f0, "gpio_direction_input" },
	{ 0xa8f59416, "gpio_direction_output" },
	{ 0x47229b5c, "gpio_request" },
	{ 0xcc524ccd, "__netlink_kernel_create" },
	{ 0x5f754e5a, "memset" },
	{ 0x2196324, "__aeabi_idiv" },
	{ 0x37a0cba, "kfree" },
	{ 0x924d9740, "__nlmsg_put" },
	{ 0x739280b, "netlink_unicast" },
	{ 0x8a3e5a7c, "__alloc_skb" },
	{ 0x9d669763, "memcpy" },
	{ 0x12da5bb2, "__kmalloc" },
	{ 0x27e1a049, "printk" },
	{ 0x81fba240, "malloc_sizes" },
	{ 0xcf14fb1f, "kmem_cache_alloc_trace" },
	{ 0xac8f37b2, "outer_cache" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";


MODULE_INFO(srcversion, "A9F0F4B770239D963F9E780");
