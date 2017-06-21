#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/init.h>

static __init int add_pcspkr(void)
{
	struct platform_device *pd;

#ifdef CONFIG_IPIPE
	if (!boot_cpu_has(X86_FEATURE_TSC)) {
		printk("I-pipe: disabling PC speaker for TSC emulation.\n");
		return -EBUSY;
	}
#endif /* CONFIG_IPIPE */

	pd = platform_device_register_simple("pcspkr", -1, NULL, 0);

	return IS_ERR(pd) ? PTR_ERR(pd) : 0;
}
device_initcall(add_pcspkr);
