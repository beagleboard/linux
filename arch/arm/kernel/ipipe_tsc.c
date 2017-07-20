#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/timer.h>
#include <linux/clocksource.h>
#include <linux/ipipe_tickdev.h>
#include <linux/cpufreq.h>
#include <linux/ipipe.h>

#include <asm/cacheflush.h>
#include <asm/traps.h>

typedef unsigned long long __ipipe_tsc_t(void);

extern __ipipe_tsc_t __ipipe_freerunning_64,
	__ipipe_freerunning_32,
	__ipipe_freerunning_countdown_32,
	__ipipe_freerunning_16,
	__ipipe_freerunning_countdown_16,
	__ipipe_decrementer_16,
	__ipipe_freerunning_twice_16,
	__ipipe_freerunning_arch;
extern unsigned long __ipipe_tsc_addr;

static struct __ipipe_tscinfo tsc_info;

static struct clocksource clksrc = {
	.name = "ipipe_tsc",
	.rating = 0x7fffffff,
	.read = (typeof(clksrc.read))__ipipe_tsc_get,
	.mask = CLOCKSOURCE_MASK(64),
	.flags = CLOCK_SOURCE_IS_CONTINUOUS,
};

struct ipipe_tsc_value_t {
	unsigned long long last_tsc;
	unsigned last_cnt;
};

unsigned long __ipipe_kuser_tsc_freq;

struct ipipe_tsc_value_t *ipipe_tsc_value;
static struct timer_list ipipe_tsc_update_timer;

static void __ipipe_tsc_update_fn(unsigned long cookie)
{
	__ipipe_tsc_update();
	ipipe_tsc_update_timer.expires += cookie;
	add_timer(&ipipe_tsc_update_timer);
}

void __init __ipipe_tsc_register(struct __ipipe_tscinfo *info)
{
	struct ipipe_tsc_value_t *vector_tsc_value;
	unsigned long long wrap_ms;
	unsigned long *tsc_addr;
	__ipipe_tsc_t *implem;
	unsigned long flags;
	int registered;
	char *tsc_area;

#if !defined(CONFIG_CPU_USE_DOMAINS)
	extern char __ipipe_tsc_area_start[], __kuser_helper_end[];

	tsc_area = (char *)vectors_page + 0x1000
		+ (__ipipe_tsc_area_start - __kuser_helper_end);
	tsc_addr = (unsigned long *)
		(tsc_area + ((char *)&__ipipe_tsc_addr - __ipipe_tsc_area));
#else
	tsc_area = __ipipe_tsc_area;
	tsc_addr = &__ipipe_tsc_addr;
#endif
	registered = ipipe_tsc_value != NULL;

	if (WARN_ON(info->freq == 0))
		return;

	if (registered && info->freq < tsc_info.freq)
		return;

	ipipe_tsc_value = (struct ipipe_tsc_value_t *)tsc_area;
	vector_tsc_value = (struct ipipe_tsc_value_t *)__ipipe_tsc_area;

	switch(info->type) {
	case IPIPE_TSC_TYPE_FREERUNNING:
		switch(info->u.mask) {
		case 0xffff:
			implem = &__ipipe_freerunning_16;
			break;
		case 0xffffffff:
			implem = &__ipipe_freerunning_32;
			break;
		case 0xffffffffffffffffULL:
			implem = &__ipipe_freerunning_64;
			break;
		default:
			goto unimplemented;
		}
		break;

	case IPIPE_TSC_TYPE_DECREMENTER:
		if (info->u.mask != 0xffff)
			goto unimplemented;
		implem = &__ipipe_decrementer_16;
		break;

	case IPIPE_TSC_TYPE_FREERUNNING_COUNTDOWN:
		switch(info->u.mask) {
		case 0xffff:
			implem = &__ipipe_freerunning_countdown_16;
			break;
		case 0xffffffff:
			implem = &__ipipe_freerunning_countdown_32;
			break;
		default:
			goto unimplemented;
		}
		break;

	case IPIPE_TSC_TYPE_FREERUNNING_TWICE:
		if (info->u.mask != 0xffff)
			goto unimplemented;
		implem = &__ipipe_freerunning_twice_16;
		break;

	case IPIPE_TSC_TYPE_FREERUNNING_ARCH:
		implem = &__ipipe_freerunning_arch;
		break;

	default:
	unimplemented:
		printk("I-pipe: Unimplemented tsc configuration, "
		       "type: %d, mask: 0x%08Lx\n", info->type, info->u.mask);
		BUG();
	}

	tsc_info = *info;
	*tsc_addr = tsc_info.counter_vaddr;
	if (tsc_info.type == IPIPE_TSC_TYPE_DECREMENTER) {
		tsc_info.u.dec.last_cnt = &vector_tsc_value->last_cnt;
		tsc_info.u.dec.tsc = &vector_tsc_value->last_tsc;
	} else
		tsc_info.u.fr.tsc = &vector_tsc_value->last_tsc;

	flags = hard_local_irq_save();
	ipipe_tsc_value->last_tsc = 0;
	memcpy(tsc_area + 0x20, implem, 0x60);
	flush_icache_range((unsigned long)(tsc_area),
			   (unsigned long)(tsc_area + 0x80));
	hard_local_irq_restore(flags);

	__ipipe_kuser_tsc_freq = tsc_info.freq;

	wrap_ms = info->u.mask;
	do_div(wrap_ms, tsc_info.freq / 1000);

	printk(KERN_INFO "I-pipe, %u.%03u MHz clocksource, wrap in %Lu ms\n",
		tsc_info.freq / 1000000, (tsc_info.freq % 1000000) / 1000,
		wrap_ms);

	if (!registered) {
		init_timer(&ipipe_tsc_update_timer);
		clocksource_register_hz(&clksrc, tsc_info.freq);
	} else
		__clocksource_update_freq_hz(&clksrc, tsc_info.freq);

	wrap_ms *= HZ / 2;
	do_div(wrap_ms, 1000);
	if (wrap_ms > 0x7fffffff)
		wrap_ms = 0x7fffffff;
	ipipe_tsc_update_timer.data = wrap_ms;
	ipipe_tsc_update_timer.function = __ipipe_tsc_update_fn;
	mod_timer(&ipipe_tsc_update_timer,
		jiffies + ipipe_tsc_update_timer.data);

	__ipipe_tracer_hrclock_initialized();
}

void __ipipe_mach_get_tscinfo(struct __ipipe_tscinfo *info)
{
	*info = tsc_info;
}

void __ipipe_tsc_update(void)
{
	if (tsc_info.type == IPIPE_TSC_TYPE_DECREMENTER) {
		unsigned cnt = *(unsigned *)tsc_info.counter_vaddr;
		int offset = ipipe_tsc_value->last_cnt - cnt;
		if (offset < 0)
			offset += tsc_info.u.dec.mask + 1;
		ipipe_tsc_value->last_tsc += offset;
		ipipe_tsc_value->last_cnt = cnt;
		return;
	}

	/* Update last_tsc, in order to remain compatible with legacy
	   user-space 32 bits free-running counter implementation */
	ipipe_tsc_value->last_tsc = __ipipe_tsc_get() - 1;
}
EXPORT_SYMBOL(__ipipe_tsc_get);

void __ipipe_update_vsyscall(struct timekeeper *tk)
{
	if (tk->tkr_mono.clock == &clksrc)
		ipipe_update_hostrt(tk);
}

#if !IS_ENABLED(CONFIG_VDSO)
void update_vsyscall(struct timekeeper *tk)
{
	__ipipe_update_vsyscall(tk);
}

void update_vsyscall_tz(void)
{
}
#endif

#ifdef CONFIG_CPU_FREQ

static __init void update_timer_freq(void *data)
{
	unsigned int hrclock_freq = *(unsigned int *)data;

	__ipipe_timer_refresh_freq(hrclock_freq);
}

static __init int cpufreq_transition_handler(struct notifier_block *nb,
				      unsigned long state, void *data)
{
	struct cpufreq_freqs *freqs = data;
	unsigned int freq;

	if (state == CPUFREQ_POSTCHANGE &&
	    ipipe_tsc_value && tsc_info.refresh_freq) {
		freq = tsc_info.refresh_freq();
		if (freq) {
			if (freqs->cpu == 0) {
				int oldrate;
				tsc_info.freq = freq;
				__ipipe_tsc_register(&tsc_info);
				__ipipe_report_clockfreq_update(freq);
				/* force timekeeper to recalculate the clocksource */
				oldrate = clksrc.rating;
				clocksource_change_rating(&clksrc, 0);
				clocksource_change_rating(&clksrc, oldrate);
			}
			smp_call_function_single(freqs->cpu, update_timer_freq,
						 &freq, 1);
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block __initdata cpufreq_nb = {
	.notifier_call = cpufreq_transition_handler,
};

static __init int register_cpufreq_notifier(void)
{
	cpufreq_register_notifier(&cpufreq_nb,
				  CPUFREQ_TRANSITION_NOTIFIER);
	return 0;
}
core_initcall(register_cpufreq_notifier);

static __init int unregister_cpufreq_notifier(void)
{
	cpufreq_unregister_notifier(&cpufreq_nb,
				  CPUFREQ_TRANSITION_NOTIFIER);
	return 0;
}
late_initcall(unregister_cpufreq_notifier);

#endif /* CONFIG_CPUFREQ */
