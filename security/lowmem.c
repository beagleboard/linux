#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mman.h>
#include <linux/init.h>
#include <linux/security.h>
#include <linux/sysctl.h>
#include <linux/swap.h>
#include <linux/kobject.h>
#include <linux/pagemap.h>
#include <linux/hugetlb.h>
#include <linux/sysfs.h>

#define MY_NAME "lowmem"

#define LOWMEM_MAX_UIDS 8

enum {
	VM_LOWMEM_DENY = 1,
	VM_LOWMEM_LEVEL1_NOTIFY,
	VM_LOWMEM_LEVEL2_NOTIFY,
	VM_LOWMEM_NR_DECAY_PAGES,
	VM_LOWMEM_ALLOWED_UIDS,
	VM_LOWMEM_ALLOWED_PAGES,
	VM_LOWMEM_USED_PAGES,
};

static unsigned int deny_percentage;
static unsigned int l1_notify, l2_notify;
static unsigned int nr_decay_pages;
static unsigned long allowed_pages;
static long used_pages;
static unsigned int allowed_uids[LOWMEM_MAX_UIDS];
static unsigned int minuid = 1;
static unsigned int maxuid = 65535;

static ctl_table lowmem_table[] = {
	{
		.ctl_name = VM_LOWMEM_DENY,
		.procname = "lowmem_deny_watermark",
		.data = &deny_percentage,
		.maxlen = sizeof(unsigned int),
		.mode = 0644,
		.child = NULL,
		.proc_handler = &proc_dointvec,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_LEVEL1_NOTIFY,
		.procname = "lowmem_notify_low",
		.data = &l1_notify,
		.maxlen = sizeof(unsigned int),
		.mode = 0644,
		.child = NULL,
		.proc_handler = &proc_dointvec,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_LEVEL2_NOTIFY,
		.procname = "lowmem_notify_high",
		.data = &l2_notify,
		.maxlen = sizeof(unsigned int),
		.mode = 0644,
		.child = NULL,
		.proc_handler = &proc_dointvec,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_NR_DECAY_PAGES,
		.procname = "lowmem_nr_decay_pages",
		.data = &nr_decay_pages,
		.maxlen = sizeof(unsigned int),
		.mode = 0644,
		.child = NULL,
		.proc_handler = &proc_dointvec_minmax,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_ALLOWED_UIDS,
		.procname = "lowmem_allowed_uids",
		.data = &allowed_uids,
		.maxlen = LOWMEM_MAX_UIDS * sizeof(unsigned int),
		.mode = 0644,
		.child = NULL,
		.proc_handler = &proc_dointvec_minmax,
		.strategy = &sysctl_intvec,
		.extra1 = &minuid,
		.extra2 = &maxuid,
	}, {
		.ctl_name = VM_LOWMEM_ALLOWED_PAGES,
		.procname = "lowmem_allowed_pages",
		.data = &allowed_pages,
		.maxlen = sizeof(unsigned long),
		.mode = 0444,
		.child = NULL,
		.proc_handler = &proc_dointvec_minmax,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_USED_PAGES,
		.procname = "lowmem_used_pages",
		.data = &used_pages,
		.maxlen = sizeof(long),
		.mode = 0444,
		.child = NULL,
		.proc_handler = &proc_dointvec_minmax,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = 0
	}
};

static ctl_table lowmem_root_table[] = {
	{
		.ctl_name = CTL_VM,
		.procname = "vm",
		.mode = 0555,
		.child = lowmem_table,
	}, {
		.ctl_name = 0
	}
};

#define KERNEL_ATTR_RO(_name) \
static struct subsys_attribute _name##_attr = __ATTR_RO(_name)

static int low_watermark_reached, high_watermark_reached;

static ssize_t low_watermark_show(struct subsystem *subsys, char *page)
{
	return sprintf(page, "%u\n", low_watermark_reached);
}

static ssize_t high_watermark_show(struct subsystem *subsys, char *page)
{
	return sprintf(page, "%u\n", high_watermark_reached);
}

KERNEL_ATTR_RO(low_watermark);
KERNEL_ATTR_RO(high_watermark);

static void low_watermark_state(int new_state)
{
	int changed = 0;

	if (low_watermark_reached != new_state) {
		low_watermark_reached = new_state;
		changed = 1;
	}

	if (changed)
		sysfs_notify(&kernel_subsys.kset.kobj, NULL, "low_watermark");
}

static void high_watermark_state(int new_state)
{
	int changed = 0;

	if (high_watermark_reached != new_state) {
		high_watermark_reached = new_state;
		changed = 1;
	}

	if (changed)
		sysfs_notify(&kernel_subsys.kset.kobj, NULL, "high_watermark");
}

static int low_vm_enough_memory(long pages)
{
	unsigned long free, allowed;
	long deny_threshold, level1, level2, used;
	int cap_sys_admin = 0, notify;

	if (cap_capable(current, CAP_SYS_ADMIN) == 0)
		cap_sys_admin = 1;

	/* We activate ourselves only after both parameters have been
	 * configured. */
	if (deny_percentage == 0 || l1_notify == 0 || l2_notify == 0)
		return __vm_enough_memory(pages, cap_sys_admin);

	allowed = totalram_pages - hugetlb_total_pages();
	deny_threshold = allowed * deny_percentage / 100;
	level1 = allowed * l1_notify / 100;
	level2 = allowed * l2_notify / 100;

	vm_acct_memory(pages);

	/* Easily freed pages when under VM pressure or direct reclaim */
	free = global_page_state(NR_FILE_PAGES);
	free += nr_swap_pages;
	free += global_page_state(NR_SLAB_RECLAIMABLE);

	used = allowed - free;
	if (unlikely(used < 0))
		used = 0;

	/* The hot path, plenty of memory */
	if (likely(used < level1))
		goto enough_memory;

	/* No luck, lets make it more expensive and try again.. */
	used -= nr_free_pages();

	if (used >= deny_threshold) {
		int i;

		allowed_pages = allowed;
		used_pages = used;
		low_watermark_state(1);
		high_watermark_state(1);
		/* Memory allocations by root are always allowed */
		if (cap_sys_admin)
			return 0;

		/* uids from allowed_uids vector are also allowed no matter what */
		for (i = 0; i < LOWMEM_MAX_UIDS && allowed_uids[i]; i++)
			if (current->uid == allowed_uids[i])
				return 0;

		vm_unacct_memory(pages);
		if (printk_ratelimit()) {
			printk(MY_NAME ": denying memory allocation to process %d (%s)\n",
			       current->pid, current->comm);
		}
		return -ENOMEM;
	}

enough_memory:
	/* See if we need to notify level 1 */
	low_watermark_state(used >= level1);

	/*
	 * In the level 2 notification case things are more complicated,
	 * as the level that we drop the state and send a notification
	 * should be lower than when it is first triggered. Having this
	 * on the same watermark level ends up bouncing back and forth
	 * when applications are being stupid.
	 */
	notify = used >= level2;
	if (notify || used + nr_decay_pages < level2)
		high_watermark_state(notify);

	/* We have plenty of memory */
	allowed_pages = allowed;
	used_pages = used;
	return 0;
}

static struct security_operations lowmem_security_ops = {
	/* Use the capability functions for some of the hooks */
	.ptrace = cap_ptrace,
	.capget = cap_capget,
	.capset_check = cap_capset_check,
	.capset_set = cap_capset_set,
	.capable = cap_capable,

	.bprm_apply_creds = cap_bprm_apply_creds,
	.bprm_set_security = cap_bprm_set_security,

	.task_post_setuid = cap_task_post_setuid,
	.task_reparent_to_init = cap_task_reparent_to_init,
	.vm_enough_memory = low_vm_enough_memory,
};

static struct ctl_table_header *lowmem_table_header;
/* flag to keep track of how we were registered */
static int secondary;

static struct attribute *lowmem_attrs[] = {
	&low_watermark_attr.attr,
	&high_watermark_attr.attr,
	NULL,
};

static struct attribute_group lowmem_attr_group = {
	.attrs	= lowmem_attrs,
};

static int __init lowmem_init(void)
{
	int r;

	/* register ourselves with the security framework */
	if (register_security(&lowmem_security_ops)) {
		printk(KERN_ERR MY_NAME ": Failure registering with the kernel\n");
		/* try registering with primary module */
		if (mod_reg_security(MY_NAME, &lowmem_security_ops)) {
			printk(KERN_ERR ": Failure registering with the primary"
			       "security module.\n");
			return -EINVAL;
		}
		secondary = 1;
	}

	/* initialize the uids vector */
	memset(allowed_uids, 0, sizeof(allowed_uids));

	lowmem_table_header = register_sysctl_table(lowmem_root_table);
	if (unlikely(!lowmem_table_header))
		return -EPERM;

	kernel_subsys.kset.kobj.kset = &kernel_subsys.kset;

	r = sysfs_create_group(&kernel_subsys.kset.kobj,
			       &lowmem_attr_group);
	if (unlikely(r))
		return r;

	printk(KERN_INFO MY_NAME ": Module initialized.\n");

	return 0;
}

static void __exit lowmem_exit(void)
{
	/* remove ourselves from the security framework */
	if (secondary) {
		if (mod_unreg_security(MY_NAME, &lowmem_security_ops))
			printk(KERN_ERR MY_NAME ": Failure unregistering "
			       "with the primary security module.\n");
	} else {
		if (unregister_security(&lowmem_security_ops)) {
			printk(KERN_ERR MY_NAME ": Failure unregistering "
			       "with the kernel.\n");
		}
	}

	unregister_sysctl_table(lowmem_table_header);

	sysfs_remove_group(&kernel_subsys.kset.kobj, &lowmem_attr_group);

	printk(KERN_INFO MY_NAME ": Module removed.\n");
}

module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_DESCRIPTION("Low watermark LSM module");
MODULE_LICENSE("GPL");
