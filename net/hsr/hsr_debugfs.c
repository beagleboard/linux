/*
 * debugfs code for HSR & PRP
 * Copyright (C) 2019 Texas Instruments Incorporated
 *
 * Author(s):
 *	Murali Karicheri <m-karicheri2@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/debugfs.h>
#include "hsr_main.h"
#include "hsr_framereg.h"

static struct dentry *hsr_debugfs_root_dir;

static void print_mac_address(struct seq_file *sfp, unsigned char *mac)
{
	seq_printf(sfp, "%02x:%02x:%02x:%02x:%02x:%02x:",
		   mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/* hsr_node_table_show - Formats and prints node_table entries */
static int
hsr_node_table_show(struct seq_file *sfp, void *data)
{
	struct hsr_priv *priv = (struct hsr_priv *)sfp->private;
	struct hsr_node *node;

	seq_puts(sfp, "Node Table entries\n");
	seq_puts(sfp, "MAC-Address-A,   MAC-Address-B, time_in[A], ");
	seq_puts(sfp, "time_in[B], Address-B port");
	if (priv->prot_version == PRP_V1)
		seq_puts(sfp, ", san_a, san_b\n");
	else
		seq_puts(sfp, "\n");
	rcu_read_lock();
	list_for_each_entry_rcu(node, &priv->node_db, mac_list) {
		/* skip self node */
		if (hsr_addr_is_self(priv, node->macaddress_A))
			continue;
		print_mac_address(sfp, &node->macaddress_A[0]);
		seq_puts(sfp, " ");
		print_mac_address(sfp, &node->macaddress_B[0]);
		seq_printf(sfp, "0x%lx, ", node->time_in[HSR_PT_SLAVE_A]);
		seq_printf(sfp, "0x%lx ", node->time_in[HSR_PT_SLAVE_B]);
		seq_printf(sfp, "0x%x", node->addr_B_port);

		if (priv->prot_version == PRP_V1)
			seq_printf(sfp, ", %x, %x\n", node->san_a, node->san_b);
		else
			seq_puts(sfp, "\n");
	}
	rcu_read_unlock();
	return 0;
}

/* hsr_node_table_open - Open the node_table file
 *
 * Description:
 * This routine opens a debugfs file node_table of specific hsr
 * or prp device
 */
static int
hsr_node_table_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, hsr_node_table_show, inode->i_private);
}

void hsr_debugfs_rename(struct net_device *dev)
{
	struct hsr_priv *priv = netdev_priv(dev);
	struct dentry *d;

	d = debugfs_rename(hsr_debugfs_root_dir, priv->root_dir,
			   hsr_debugfs_root_dir, dev->name);
	if (IS_ERR(d))
		netdev_warn(dev, "failed to rename\n");
	else
		priv->root_dir = d;
}

static const struct file_operations hsr_node_table_fops = {
	.open	= hsr_node_table_open,
	.read	= seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* hsr_lre_info_show - Formats and prints debug info in the device
 */
static int
hsr_lre_info_show(struct seq_file *sfp, void *data)
{
	struct hsr_priv *priv = (struct hsr_priv *)sfp->private;
	bool prp = priv->prot_version > HSR_V1;

	seq_puts(sfp, "LRE debug information\n");
	seq_printf(sfp, "Protocol : %s\n", prp ? "PRP" : "HSR");
	seq_printf(sfp, "net_id: %d\n", priv->net_id);
	seq_printf(sfp, "Rx Offloaded: %s\n",
		   priv->rx_offloaded ? "Yes" : "No");
	if (!prp)
		seq_printf(sfp, "L2 fw Offloaded: %s\n",
			   priv->l2_fwd_offloaded ? "Yes" : "No");
	seq_printf(sfp, "vlan tag used in sv frame : %s\n",
		   priv->use_vlan_for_sv ? "Yes" : "No");
	if (priv->use_vlan_for_sv) {
		seq_printf(sfp, "SV Frame VID : %d\n",
			   priv->sv_frame_vid);
		seq_printf(sfp, "SV Frame PCP : %d\n",
			   priv->sv_frame_pcp);
		seq_printf(sfp, "SV Frame DEI : %d\n",
			   priv->sv_frame_dei);
	}
	seq_printf(sfp, "cnt_tx_sup = %d\n", priv->dbg_stats.cnt_tx_sup);
	seq_printf(sfp, "cnt_rx_sup_A = %d\n", priv->dbg_stats.cnt_rx_sup_a);
	seq_printf(sfp, "cnt_rx_sup_B = %d\n", priv->dbg_stats.cnt_rx_sup_b);
	seq_printf(sfp, "disable SV Frame = %d\n", priv->disable_sv_frame);
	seq_puts(sfp, "\n");
	return 0;
}

/* hsr_lre_info_open - open lre info file
 *
 * Description:
 * This routine opens a debugfs file lre_info of specific hsr or
 * prp device
 */
static int
hsr_lre_info_open(struct inode *inode, struct file *filp)
{
	return single_open(filp, hsr_lre_info_show, inode->i_private);
}

static const struct file_operations hsr_lre_info_fops = {
	.owner	= THIS_MODULE,
	.open	= hsr_lre_info_open,
	.read	= seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* hsr_debugfs_init - create debugfs to dump lre specific debug information
 *
 * Description:
 * dump lre info of hsr or prp device
 */
void hsr_debugfs_init(struct hsr_priv *priv, struct net_device *hsr_dev)
{
	struct dentry *de = NULL;

	de = debugfs_create_dir(hsr_dev->name, hsr_debugfs_root_dir);
	if (IS_ERR(de)) {
		pr_err("Cannot create hsr debugfs root directory %s\n",
		       hsr_dev->name);
		return;
	}

	priv->root_dir = de;

	de = debugfs_create_file("node_table", S_IFREG | 0444,
				 priv->root_dir, priv, &hsr_node_table_fops);
	if (IS_ERR(de)) {
		pr_err("Cannot create hsr node_table file\n");
		goto error_nt;
	}
	priv->node_tbl_file = de;

	de = debugfs_create_file("lre_info", S_IFREG | 0444,
				 priv->root_dir, priv, &hsr_lre_info_fops);
	if (IS_ERR(de)) {
		pr_err("Cannot create hsr-prp lre_info file\n");
		goto error_lre_info;
	}
	priv->lre_info_file = de;

	return;

error_lre_info:
	debugfs_remove(priv->node_tbl_file);
	priv->node_tbl_file = NULL;
error_nt:
	debugfs_remove(priv->root_dir);
	priv->root_dir = NULL;
} /* end of hst_debugfs_init */

/* hsr_debugfs_term - Tear down debugfs intrastructure
 *
 * Description:
 * When Debufs is configured this routine removes debugfs file system
 * elements that are specific to hsr
 */
void
hsr_debugfs_term(struct hsr_priv *priv)
{
	debugfs_remove(priv->node_tbl_file);
	priv->node_tbl_file = NULL;
	debugfs_remove(priv->lre_info_file);
	priv->lre_info_file = NULL;
	debugfs_remove(priv->root_dir);
	priv->root_dir = NULL;
}

void hsr_debugfs_create_root(void)
{
	hsr_debugfs_root_dir = debugfs_create_dir("hsr", NULL);
	if (IS_ERR(hsr_debugfs_root_dir)) {
		pr_err("Cannot create hsr debugfs root directory\n");
		hsr_debugfs_root_dir = NULL;
	}
}

void hsr_debugfs_remove_root(void)
{
	/* debugfs_remove() internally checks NULL and ERROR */
	debugfs_remove(hsr_debugfs_root_dir);
}
