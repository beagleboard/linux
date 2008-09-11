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
#include <linux/oom.h>

#define MY_NAME "lowmem"

#define LOWMEM_MAX_UIDS 8

enum {
	VM_LOWMEM_DENY_PAGES = 1,
	VM_LOWMEM_NOTIFY_LOW_PAGES,
	VM_LOWMEM_NOTIFY_HIGH_PAGES,
	VM_LOWMEM_NR_DECAY_PAGES,
	VM_LOWMEM_ALLOWED_UIDS,
	VM_LOWMEM_ALLOWED_PAGES,
	VM_LOWMEM_FREE_PAGES,
	VM_LOWMEM_DENY,
	VM_LOWMEM_LEVEL1_NOTIFY,
	VM_LOWMEM_LEVEL2_NOTIFY,
	VM_LOWMEM_USED_PAGES
};

static long deny_pages;
static long notify_low_pages, notify_high_pages;
static unsigned int nr_decay_pages;
static unsigned long allowed_pages;
static unsigned long lowmem_free_pages;
static unsigned int allowed_uids[LOWMEM_MAX_UIDS];
static unsigned int minuid = 1;
static unsigned int maxuid = 65535;
static unsigned int deny_percentage;
static unsigned int l1_notify, l2_notify;
static long used_pages;

static int
proc_dointvec_used(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos);
static int
proc_dointvec_l1_notify(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos);
static int
proc_dointvec_l2_notify(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos);
static int
proc_dointvec_deny(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos);

static ctl_table lowmem_table[] = {
	{
		.ctl_name = VM_LOWMEM_DENY_PAGES,
		.procname = "lowmem_deny_watermark_pages",
		.data = &deny_pages,
		.maxlen = sizeof(long),
		.mode = 0644,
		.child = NULL,
		.proc_handler = &proc_dointvec,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_DENY,
		.procname = "lowmem_deny_watermark",
		.data = &deny_percentage,
		.maxlen = sizeof(unsigned int),
		.mode = 0444,
		.child = NULL,
		.proc_handler = &proc_dointvec_deny,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_LEVEL1_NOTIFY,
		.procname = "lowmem_notify_low",
		.data = &l1_notify,
		.maxlen = sizeof(unsigned int),
		.mode = 0444,
		.child = NULL,
		.proc_handler = &proc_dointvec_l1_notify,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_LEVEL2_NOTIFY,
		.procname = "lowmem_notify_high",
		.data = &l2_notify,
		.maxlen = sizeof(unsigned int),
		.mode = 0444,
		.child = NULL,
		.proc_handler = &proc_dointvec_l2_notify,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_USED_PAGES,
		.procname = "lowmem_used_pages",
		.data = &used_pages,
		.maxlen = sizeof(long),
		.mode = 0444,
		.child = NULL,
		.proc_handler = &proc_dointvec_used,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_NOTIFY_LOW_PAGES,
		.procname = "lowmem_notify_low_pages",
		.data = &notify_low_pages,
		.maxlen = sizeof(long),
		.mode = 0644,
		.child = NULL,
		.proc_handler = &proc_dointvec,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_NOTIFY_HIGH_PAGES,
		.procname = "lowmem_notify_high_pages",
		.data = &notify_high_pages,
		.maxlen = sizeof(long),
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
		.proc_handler = &proc_dointvec,
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
		.proc_handler = &proc_dointvec,
		.strategy = &sysctl_intvec,
	}, {
		.ctl_name = VM_LOWMEM_FREE_PAGES,
		.procname = "lowmem_free_pages",
		.data = &lowmem_free_pages,
		.maxlen = sizeof(unsigned long),
		.mode = 0444,
		.child = NULL,
		.proc_handler = &proc_dointvec,
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
static struct kobj_attribute _name##_attr = __ATTR_RO(_name)

static int low_watermark_reached, high_watermark_reached;

static int
proc_dointvec_l1_notify(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos)
{
	l1_notify =
	100 - (100 * notify_low_pages + allowed_pages / 2) / allowed_pages;
	return proc_dointvec(table, write, filp, buffer, lenp, ppos);
}

static int
proc_dointvec_l2_notify(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos)
{
	l2_notify =
	100 - (100 * notify_high_pages + allowed_pages / 2) / allowed_pages;
	return proc_dointvec(table, write, filp, buffer, lenp, ppos);
}

static int
proc_dointvec_deny(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos)
{
	deny_percentage =
	100 - (100 * deny_pages + allowed_pages / 2) / allowed_pages;
	return proc_dointvec(table, write, filp, buffer, lenp, ppos);
}

static int
proc_dointvec_used(ctl_table *table, int write, struct file *filp,
			void __user *buffer, size_t *lenp, loff_t *ppos)
{
	if (lowmem_free_pages > 0 && allowed_pages > lowmem_free_pages)
		used_pages = allowed_pages - lowmem_free_pages;
	else
		used_pages = 0;
	return proc_dointvec(table, write, filp, buffer, lenp, ppos);
}

static ssize_t low_watermark_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *page)
{
	return sprintf(page, "%u\n", low_watermark_reached);
}

static ssize_t high_watermark_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *page)
{
	return sprintf(page, "%u\n", high_watermark_reached);
}

KERNEL_ATTR_RO(low_watermark);
KERNEL_ATTR_RO(high_watermark);

static void low_watermark_state(int new_state)
{
	if (low_watermark_reached != new_state) {
		low_watermark_reached = new_state;
		sysfs_notify(kernel_kobj, NULL, "low_watermark");
	}
}

static void high_watermark_state(int new_state)
{
	if (high_watermark_reached != new_state) {
		high_watermark_reached = new_state;
		sysfs_notify(kernel_kobj, NULL, "high_watermark");
	}
}

static int low_vm_enough_memory(struct mm_struct *mm, long pages)
{
	unsigned long free, allowed;
	int cap_sys_admin = 0, notify;

	if (cap_capable(current, CAP_SYS_ADMIN) == 0)
		cap_sys_admin = 1;

	allowed = totalram_pages - hugetlb_total_pages();
	allowed_pages = allowed;

	/* We activate ourselves only after both parameters have been
	 * configured. */
	if (deny_pages == 0 || notify_low_pages == 0 || notify_high_pages == 0)
		return  __vm_enough_memory(mm, pages, cap_sys_admin);

	vm_acct_memory(pages);

	/* Easily freed pages when under VM pressure or direct reclaim */
	free = global_page_state(NR_FILE_PAGES);
	free += nr_swap_pages;
	free += global_page_state(NR_SLAB_RECLAIMABLE);

	if (likely(free > notify_low_pages))
		goto enough_memory;

	/* No luck, lets make it more expensive and try again.. */
	free += nr_free_pages();

	if (free < deny_pages) {
		int i;

		lowmem_free_pages = free;
		low_watermark_state(1);
		high_watermark_state(1);
		/* Memory allocations by root are always allowed */
		if (cap_sys_admin)
			return 0;

		/* OOM unkillable process is allowed to consume memory */
		if (current->oomkilladj == OOM_DISABLE)
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
	low_watermark_state(free < notify_low_pages);

	/*
	 * In the level 2 notification case things are more complicated,
	 * as the level that we drop the state and send a notification
	 * should be lower than when it is first triggered. Having this
	 * on the same watermark level ends up bouncing back and forth
	 * when applications are being stupid.
	 */
	notify = free < notify_high_pages;
	if (notify || free - nr_decay_pages > notify_high_pages)
		high_watermark_state(notify);

	/* We have plenty of memory */
	lowmem_free_pages = free;
	return 0;
}

static struct security_operations lowmem_security_ops = {
	/* Use the capability functions for some of the hooks */
	.ptrace_may_access = cap_ptrace_may_access,
	.ptrace_traceme = cap_ptrace_traceme,
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
		return -EINVAL;
	}

	/* initialize the uids vector */
	memset(allowed_uids, 0, sizeof(allowed_uids));

	lowmem_table_header = register_sysctl_table(lowmem_root_table);
	if (unlikely(!lowmem_table_header))
		return -EPERM;

	r = sysfs_create_group(kernel_kobj,
			       &lowmem_attr_group);
	if (unlikely(r))
		return r;

	printk(KERN_INFO MY_NAME ": Module initialized.\n");

	return 0;
}

module_init(lowmem_init);

MODULE_DESCRIPTION("Low watermark LSM module");
MODULE_LICENSE("GPL");
