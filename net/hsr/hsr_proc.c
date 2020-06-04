// SPDX-License-Identifier: GPL-2.0
/* hsr_proc.c : procfs file for HSR and PRP driver
 *
 * Copyright (C) 2018-2020 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author(s):
 *     Murali Karicheri <m-karicheri2@ti.com>
 */
#include <linux/netdevice.h>
#include <linux/proc_fs.h>

#include "hsr_main.h"

#define BUF_SIZE 64
#define LRE_STAT_OFS(m) offsetof(struct lre_stats, m)

static const char *const hsr_lre_stats[] = {
	"lreTxA",
	"lreTxB",
	"lreTxC",
	"lreErrWrongLanA",
	"lreErrWrongLanB",
	"lreErrWrongLanC",
	"lreRxA",
	"lreRxB",
	"lreRxC",
	"lreErrorsA",
	"lreErrorsB",
	"lreErrorsC",
	"lreNodes",
	"lreProxyNodes",
	"lreUniqueRxA",
	"lreUniqueRxB",
	"lreUniqueRxC",
	"lreDuplicateRxA",
	"lreDuplicateRxB",
	"lreDuplicateRxC",
	"lreMultiRxA",
	"lreMultiRxB",
	"lreMultiRxC",
	"lreOwnRxA",
	"lreOwnRxB",
};

static int hsr_lre_stats_show(struct seq_file *sfp, void *v)
{
	struct hsr_priv *priv = (struct hsr_priv *)sfp->private;
	struct lre_stats lower_stats, *upper_stats;
	int ret = 0, i;
	u32 *ptr;

	upper_stats = &priv->lre_stats;
	if (priv->rx_offloaded) {
		ret = hsr_lredev_get_lre_stats(priv, &lower_stats);
		if (ret < 0) {
			seq_puts(sfp, "Error in retrieving the stats\n");
			return 0;
		}
		ptr = (u32 *)&lower_stats;
	} else {
		ptr = (u32 *)upper_stats;
	}

	seq_puts(sfp, "LRE statistics:\n");
	seq_printf(sfp, "Rx Offloaded: %d\n", priv->rx_offloaded);
	for (i = 0; i < ARRAY_SIZE(hsr_lre_stats); i++) {
		/* for rx_c and tx_c, retrieve stats from hsr/prp device
		 * lre stats. Rest of the stats are retrieved from
		 * lower device.
		 */
		if (!strcmp("lreTxC", hsr_lre_stats[i])) {
			seq_printf(sfp, "\n     %s: %d",
				   hsr_lre_stats[i],
				   upper_stats->cnt_tx_c);
			continue;
		}

		if (!strcmp("lreRxC", hsr_lre_stats[i])) {
			seq_printf(sfp, "\n     %s: %d",
				   hsr_lre_stats[i],
				   upper_stats->cnt_rx_c);
			continue;
		}
		seq_printf(sfp, "\n     %s: %d", hsr_lre_stats[i],
			   *(ptr + i));
	}
	seq_puts(sfp, "\n");

	return 0;
}

static int hsr_lre_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, hsr_lre_stats_show, PDE_DATA(inode));
}

static const struct file_operations hsr_lre_stats_fops = {
	.owner		= THIS_MODULE,
	.open		= hsr_lre_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int hsr_node_show(struct seq_file *sfp, int index,
			 struct lre_node_table_entry *entry)
{
	seq_printf(sfp, "\nNode[%u]:\n", index);
	seq_printf(sfp, "MAC ADDR: %02x:%02x:%02x:%02x:%02x:%02x\n",
		   entry->mac_address[0],
		   entry->mac_address[1],
		   entry->mac_address[2],
		   entry->mac_address[3],
		   entry->mac_address[4],
		   entry->mac_address[5]);

	switch (entry->node_type) {
	case IEC62439_3_DANP:
		seq_puts(sfp, "DANP\n");
		break;
	case IEC62439_3_REDBOXP:
		seq_puts(sfp, "REDBOXP\n");
		break;
	case IEC62439_3_VDANP:
		seq_puts(sfp, "VDANP\n");
		break;
	case IEC62439_3_DANH:
		seq_puts(sfp, "DANH\n");
		break;
	case IEC62439_3_REDBOXH:
		seq_puts(sfp, "REDBOXH\n");
		break;
	case IEC62439_3_VDANH:
		seq_puts(sfp, "VDANH\n");
		break;
	default:
		seq_printf(sfp, "Unknown node type %u\n", entry->node_type);
		break;
	};

	seq_printf(sfp, "Time Last Seen: RxA=%u RxB=%u\n",
		   entry->time_last_seen_a,
		   entry->time_last_seen_b);
	return 0;
}

static int hsr_node_table_show(struct seq_file *sfp, void *v)
{
	struct hsr_priv *priv = (struct hsr_priv *)sfp->private;
	struct lre_node_table_entry *nt_table;
	int ret = 0, count, i;

	nt_table = kcalloc(LRE_MAX_NT_ENTRIES, sizeof(*nt_table), GFP_KERNEL);
	if (!nt_table)
		return -ENODEV;

	count = hsr_lredev_get_node_table(priv, nt_table,
					  LRE_MAX_NT_ENTRIES);
	if (count < 0)
		count = 0;

	seq_printf(sfp, "\nRemote nodes in network: %u\n", count);

	if (!count)
		return ret;

	for (i = 0; i < count; i++)
		hsr_node_show(sfp, i, &nt_table[i]);
	return ret;
}

static int hsr_node_table_open(struct inode *inode, struct file *file)
{
	return single_open(file, hsr_node_table_show, PDE_DATA(inode));
}

static const struct file_operations hsr_node_table_fops = {
	.owner		= THIS_MODULE,
	.open		= hsr_node_table_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static inline int get_set_param(struct hsr_priv *priv,
				const char __user *buffer, size_t count,
				enum lredev_attr_id id)
{
	struct lredev_attr temp_attr;
	char cmd_buffer[BUF_SIZE];
	int ret = -EINVAL;
	u32 val;

	if (count > (sizeof(cmd_buffer) - 1))
		goto err;

	if (copy_from_user(cmd_buffer, buffer, count)) {
		ret = -EFAULT;
		goto err;
	}
	cmd_buffer[count] = '\0';
	ret = kstrtou32(cmd_buffer, 0, &val);
	if (ret < 0)
		goto err;

	/* TODO. Update mode. Check if anything else needed for
	 * non offload case
	 */
	temp_attr.id = id;
	switch (id) {
	case LREDEV_ATTR_ID_HSR_MODE:
		if (val > IEC62439_3_HSR_MODE_M) {
			ret = -EINVAL;
			goto err;
		}
		if (!priv->rx_offloaded) {
			priv->hsr_mode = (enum iec62439_3_hsr_modes)val;
			return 0;
		}
		temp_attr.mode = (enum iec62439_3_hsr_modes)val;
		break;

	case LREDEV_ATTR_ID_PRP_TR:
		if (val > IEC62439_3_TR_PASS_RCT) {
			ret = -EINVAL;
			goto err;
		}
		if (!priv->rx_offloaded) {
			priv->prp_tr = (enum iec62439_3_tr_modes)val;
			goto out;
		}
		temp_attr.tr_mode = (enum iec62439_3_tr_modes)val;
		break;

	case LREDEV_ATTR_ID_DD_MODE:
		if (val > IEC62439_3_DD) {
			ret = -EINVAL;
			goto err;
		}
		if (!priv->rx_offloaded) {
			priv->dd_mode = (enum iec62439_3_dd_modes)val;
			goto out;
		}
		temp_attr.dd_mode = (enum iec62439_3_dd_modes)val;
		break;

	case LREDEV_ATTR_ID_DLRMT:
		if (!priv->rx_offloaded) {
			priv->dlrmt = val;
			goto out;
		}
		temp_attr.dl_reside_max_time = val;
		break;

	case LREDEV_ATTR_ID_CLEAR_NT:
		if (val > IEC62439_3_CLEAR_NT) {
			ret = -EINVAL;
			goto err;
		}
		if (!priv->rx_offloaded) {
			priv->clear_nt_cmd =
				(enum iec62439_3_clear_nt_cmd)val;
			goto out;
		}
		temp_attr.clear_nt_cmd = (enum iec62439_3_clear_nt_cmd)val;
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	/* pass this to lower layer device, i.e slave-1 */
	ret = hsr_lredev_attr_set(priv, &temp_attr);
	if (ret)
		return ret;

	/* update the local copy */
	switch (id) {
	case LREDEV_ATTR_ID_HSR_MODE:
		priv->hsr_mode = temp_attr.mode;
		break;
	case LREDEV_ATTR_ID_PRP_TR:
		priv->prp_tr = temp_attr.tr_mode;
		break;
	case LREDEV_ATTR_ID_DD_MODE:
		priv->dd_mode = temp_attr.dd_mode;
		break;
	case LREDEV_ATTR_ID_CLEAR_NT:
		priv->clear_nt_cmd = temp_attr.clear_nt_cmd;
		break;
	default: /* LREDEV_ATTR_ID_DLRMT */
		priv->dlrmt = temp_attr.dl_reside_max_time;
		break;
	}
out:
	return 0;
err:
	return ret;
}

static int hsr_mode_show(struct seq_file *sfp, void *v)
{
	struct hsr_priv *priv = (struct hsr_priv *)sfp->private;
	struct lredev_attr temp_attr;
	int err;

	if (!priv->rx_offloaded) {
		seq_printf(sfp, "%u\n", priv->hsr_mode);
		return 0;
	}

	temp_attr.id = LREDEV_ATTR_ID_HSR_MODE;
	err = hsr_lredev_attr_get(priv, &temp_attr);
	if (err)
		return err;

	seq_printf(sfp, "%u\n", temp_attr.mode);
	return 0;
}

static int hsr_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, hsr_mode_show, PDE_DATA(inode));
}

static ssize_t hsr_mode_store(struct file *file,
			      const char __user *buffer,
			      size_t count, loff_t *pos)
{
	struct hsr_priv *priv = (struct hsr_priv *)PDE_DATA(file_inode(file));
	int err;

	err = get_set_param(priv, buffer, count, LREDEV_ATTR_ID_HSR_MODE);
	if (err)
		return err;

	return  count;
}

static const struct file_operations hsr_mode_fops = {
	.owner		= THIS_MODULE,
	.open		= hsr_mode_open,
	.read		= seq_read,
	.write		= hsr_mode_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int prp_tr_show(struct seq_file *sfp, void *v)
{
	struct hsr_priv *priv = (struct hsr_priv *)sfp->private;
	struct lredev_attr temp_attr;
	int err;

	if (!priv->rx_offloaded) {
		seq_printf(sfp, "%u\n", priv->prp_tr);
		return 0;
	}

	temp_attr.id = LREDEV_ATTR_ID_PRP_TR;
	err = hsr_lredev_attr_get(priv, &temp_attr);
	if (err)
		return err;

	seq_printf(sfp, "%u\n", temp_attr.tr_mode);
	return 0;
}

static int prp_tr_open(struct inode *inode, struct file *file)
{
	return single_open(file, prp_tr_show, PDE_DATA(inode));
}

static ssize_t prp_tr_store(struct file *file,
			    const char __user *buffer,
			    size_t count, loff_t *pos)
{
	struct hsr_priv *priv = (struct hsr_priv *)PDE_DATA(file_inode(file));
	int err;

	err = get_set_param(priv, buffer, count, LREDEV_ATTR_ID_PRP_TR);
	if (err)
		return err;

	return  count;
}

static const struct file_operations prp_tr_fops = {
	.owner		= THIS_MODULE,
	.open		= prp_tr_open,
	.read		= seq_read,
	.write		= prp_tr_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dlrmt_show(struct seq_file *sfp, void *v)
{
	struct hsr_priv *priv = (struct hsr_priv *)sfp->private;
	struct lredev_attr temp_attr;
	int err;

	if (!priv->rx_offloaded) {
		seq_printf(sfp, "%u\n", priv->dlrmt);
		return 0;
	}

	temp_attr.id = LREDEV_ATTR_ID_DLRMT;
	err = hsr_lredev_attr_get(priv, &temp_attr);
	if (err)
		return err;

	seq_printf(sfp, "%u\n", temp_attr.dl_reside_max_time);
	return 0;
}

static int dlrmt_open(struct inode *inode, struct file *file)
{
	return single_open(file, dlrmt_show, PDE_DATA(inode));
}

static ssize_t dlrmt_store(struct file *file,
			   const char __user *buffer,
			   size_t count, loff_t *pos)
{
	struct hsr_priv *priv = (struct hsr_priv *)PDE_DATA(file_inode(file));
	int err;

	err = get_set_param(priv, buffer, count, LREDEV_ATTR_ID_DLRMT);
	if (err)
		return err;

	return  count;
}

static const struct file_operations dlrmt_fops = {
	.owner		= THIS_MODULE,
	.open		= dlrmt_open,
	.read		= seq_read,
	.write		= dlrmt_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dd_mode_show(struct seq_file *sfp, void *v)
{
	struct hsr_priv *priv = (struct hsr_priv *)sfp->private;
	struct lredev_attr temp_attr;
	int err;

	if (!priv->rx_offloaded) {
		seq_printf(sfp, "%u\n", priv->dd_mode);
		return 0;
	}

	temp_attr.id = LREDEV_ATTR_ID_DD_MODE;
	err = hsr_lredev_attr_get(priv, &temp_attr);
	if (err)
		return err;

	seq_printf(sfp, "%u\n", temp_attr.dd_mode);
	return 0;
}

static int dd_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, dd_mode_show, PDE_DATA(inode));
}

static ssize_t dd_mode_store(struct file *file,
			     const char __user *buffer,
			     size_t count, loff_t *pos)
{
	struct hsr_priv *priv = (struct hsr_priv *)PDE_DATA(file_inode(file));
	int err;

	err = get_set_param(priv, buffer, count, LREDEV_ATTR_ID_DD_MODE);
	if (err)
		return err;

	return  count;
}

static const struct file_operations dd_mode_fops = {
	.owner		= THIS_MODULE,
	.open		= dd_mode_open,
	.read		= seq_read,
	.write		= dd_mode_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int clear_nt_show(struct seq_file *sfp, void *v)
{
	struct hsr_priv *priv = (struct hsr_priv *)sfp->private;
	struct lredev_attr temp_attr;
	int err;

	if (!priv->rx_offloaded) {
		seq_printf(sfp, "%u\n", priv->clear_nt_cmd);
		return 0;
	}

	temp_attr.id = LREDEV_ATTR_ID_CLEAR_NT;
	err = hsr_lredev_attr_get(priv, &temp_attr);
	if (err)
		return err;

	seq_printf(sfp, "%u\n", temp_attr.clear_nt_cmd);
	return 0;
}

static int clear_nt_open(struct inode *inode, struct file *file)
{
	return single_open(file, clear_nt_show, PDE_DATA(inode));
}

static ssize_t clear_nt_store(struct file *file,
			      const char __user *buffer,
			      size_t count, loff_t *pos)
{
	struct hsr_priv *priv = (struct hsr_priv *)PDE_DATA(file_inode(file));
	int err;

	err = get_set_param(priv, buffer, count, LREDEV_ATTR_ID_CLEAR_NT);
	if (err)
		return err;

	return  count;
}

static const struct file_operations clear_nt_fops = {
	.owner		= THIS_MODULE,
	.open		= clear_nt_open,
	.read		= seq_read,
	.write		= clear_nt_store,
	.llseek		= seq_lseek,
	.release	= single_release,
};

int hsr_create_procfs(struct hsr_priv *priv, struct net_device *ndev)
{
	int ret = -ENODEV;

	priv->dir = proc_mkdir(ndev->name, NULL);
	if (!priv->dir)
		return ret;

	priv->lre_stats_file = proc_create_data("lre-stats", 0444, priv->dir,
						&hsr_lre_stats_fops,
						(void *)priv);
	if (!priv->lre_stats_file)
		goto fail_lre_stats;

	priv->node_table_file = proc_create_data("node-table", 0444, priv->dir,
						 &hsr_node_table_fops,
						 (void *)priv);
	if (!priv->node_table_file)
		goto fail_node_table;

	priv->hsr_mode_file = proc_create_data("hsr-mode", 0644, priv->dir,
					       &hsr_mode_fops, (void *)priv);
	if (!priv->hsr_mode_file)
		goto fail_hsr_mode;

	priv->dd_mode_file = proc_create_data("dd-mode", 0644, priv->dir,
					      &dd_mode_fops, (void *)priv);
	if (!priv->dd_mode_file)
		goto fail_dd_mode;

	priv->prp_tr_file = proc_create_data("prp-tr", 0644, priv->dir,
					     &prp_tr_fops, (void *)priv);
	if (!priv->prp_tr_file)
		goto fail_prp_tr;

	priv->clear_nt_file = proc_create_data("clear-nt", 0644, priv->dir,
					       &clear_nt_fops, (void *)priv);
	if (!priv->clear_nt_file)
		goto fail_clear_nt;

	priv->dlrmt_file = proc_create_data("dlrmt", 0644, priv->dir,
					    &dlrmt_fops, (void *)priv);
	if (!priv->dlrmt_file)
		goto fail_dlrmt;

	return 0;
fail_dlrmt:
	if (priv->clear_nt_file)
		remove_proc_entry("clear-nt", priv->dir);
fail_clear_nt:
	if (priv->prp_tr_file)
		remove_proc_entry("prp-tr", priv->dir);
fail_prp_tr:
	if (priv->dd_mode_file)
		remove_proc_entry("dd-mode", priv->dir);
fail_dd_mode:
	if (priv->hsr_mode_file)
		remove_proc_entry("hsr-mode", priv->dir);
fail_hsr_mode:
	if (priv->node_table_file)
		remove_proc_entry("node-table", priv->dir);
fail_node_table:
	if (priv->lre_stats_file)
		remove_proc_entry("lre-stats", priv->dir);
fail_lre_stats:
	remove_proc_entry(ndev->name, NULL);
	return ret;
}

void hsr_remove_procfs(struct hsr_priv *priv, struct net_device *ndev)
{
	remove_proc_entry("dlrmt", priv->dir);
	remove_proc_entry("clear-nt", priv->dir);
	remove_proc_entry("prp-tr", priv->dir);
	remove_proc_entry("dd-mode", priv->dir);
	remove_proc_entry("hsr-mode", priv->dir);
	remove_proc_entry("lre-stats", priv->dir);
	remove_proc_entry("node-table", priv->dir);
	remove_proc_entry(ndev->name, NULL);
	priv->dir = NULL;
}
