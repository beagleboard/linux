// SPDX-License-Identifier: GPL-2.0
/**
 * I2C Address Translator
 *
 * Copyright (c) 2019 Luca Ceresoli <luca@lucaceresoli.net>
 *
 * An I2C Address Translator (ATR) is a device with an I2C slave parent
 * ("upstream") port and N I2C master child ("downstream") ports, and
 * forwards transactions from upstream to the appropriate downstream port
 * with a modified slave address. The address used on the parent bus is
 * called the "alias" and is (potentially) different from the physical
 * slave address of the child bus. Address translation is done by the
 * hardware.
 *
 * An ATR looks similar to an i2c-mux except:
 * - the address on the parent and child busses can be different
 * - there is normally no need to select the child port; the alias used on
 *   the parent bus implies it
 *
 * The ATR functionality can be provided by a chip with many other
 * features. This file provides a helper to implement an ATR within your
 * driver.
 *
 * The ATR creates a new I2C "child" adapter on each child bus. Adding
 * devices on the child bus ends up in invoking the driver code to select
 * an available alias. Maintaining an appropriate pool of available aliases
 * and picking one for each new device is up to the driver implementer. The
 * ATR maintains an table of currently assigned alias and uses it to modify
 * all I2C transactions directed to devices on the child buses.
 *
 * A typical example follows.
 *
 * Topology:
 *
 *                       Slave X @ 0x10
 *               .-----.   |
 *   .-----.     |     |---+---- B
 *   | CPU |--A--| ATR |
 *   `-----'     |     |---+---- C
 *               `-----'   |
 *                       Slave Y @ 0x10
 *
 * Alias table:
 *
 *   Client  Alias
 *   -------------
 *      X    0x20
 *      Y    0x30
 *
 * Transaction:
 *
 *  - Slave X driver sends a transaction (on adapter B), slave address 0x10
 *  - ATR driver rewrites messages with address 0x20, forwards to adapter A
 *  - Physical I2C transaction on bus A, slave address 0x20
 *  - ATR chip propagates transaction on bus B with address translated to 0x10
 *  - Slave X chip replies on bus B
 *  - ATR chip forwards reply on bus A
 *  - ATR driver rewrites messages with address 0x10
 *  - Slave X driver gets back the msgs[], with reply and address 0x10
 *
 * Usage:
 *
 *  1. In your driver (typically in the probe function) add an ATR by
 *     calling i2c_atr_new() passing your attach/detach callbacks
 *  2. When the attach callback is called pick an appropriate alias,
 *     configure it in your chip and return the chosen alias in the
 *     alias_id parameter
 *  3. When the detach callback is called, deconfigure the alias from
 *     your chip and put it back in the pool for later usage
 *
 * Originally based on i2c-mux.c
 */

#include <linux/i2c.h>
#include <linux/i2c-atr.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>

/**
 * struct i2c_atr_cli2alias_pair - Hold the alias assigned to a client.
 * @node:   List node
 * @client: Pointer to the client on the child bus
 * @alias:  I2C alias address assigned by the driver.
 *          This is the address that will be used to issue I2C transactions
 *          on the parent (physical) bus.
 */
struct i2c_atr_cli2alias_pair {
	struct list_head node;
	const struct i2c_client *client;
	u16 alias;
};

/*
 * Data for each channel (child bus)
 */
struct i2c_atr_chan {
	struct i2c_adapter adap;
	struct i2c_atr *atr;
	u32 chan_id;

	struct list_head alias_list;

	u16 *orig_addrs;
	unsigned int orig_addrs_size;
	struct mutex orig_addrs_lock; /* Lock orig_addrs during xfer */
};

static struct i2c_atr_cli2alias_pair *
i2c_atr_find_mapping_by_client(struct list_head *list,
			       const struct i2c_client *client)
{
	struct i2c_atr_cli2alias_pair *c2a;

	list_for_each_entry(c2a, list, node) {
		if (c2a->client == client)
			return c2a;
	}

	return NULL;
}

static struct i2c_atr_cli2alias_pair *
i2c_atr_find_mapping_by_addr(struct list_head *list,
			     u16 phys_addr)
{
	struct i2c_atr_cli2alias_pair *c2a;

	list_for_each_entry(c2a, list, node) {
		if (c2a->client->addr == phys_addr)
			return c2a;
	}

	return NULL;
}

/*
 * Replace all message addresses with their aliases, saving the original
 * addresses.
 *
 * This function is internal for use in i2c_atr_master_xfer(). It must be
 * followed by i2c_atr_unmap_msgs() to restore the original addresses.
 */
static int i2c_atr_map_msgs(struct i2c_atr_chan *chan,
			    struct i2c_msg msgs[], int num)

{
	struct i2c_atr *atr = chan->atr;
	static struct i2c_atr_cli2alias_pair *c2a;
	int i;

	/* Ensure we have enough room to save the original addresses */
	if (unlikely(chan->orig_addrs_size < num)) {
		void *new_buf = kmalloc(num * sizeof(chan->orig_addrs[0]),
					GFP_KERNEL);
		if (new_buf == NULL)
			return -ENOMEM;

		kfree(chan->orig_addrs);
		chan->orig_addrs = new_buf;
		chan->orig_addrs_size = num;
	}

	for (i = 0; i < num; i++) {
		chan->orig_addrs[i] = msgs[i].addr;

		c2a = i2c_atr_find_mapping_by_addr(&chan->alias_list,
						   msgs[i].addr);
		if (c2a) {
			msgs[i].addr = c2a->alias;
		} else {
			dev_err(atr->dev, "client 0x%02x not mapped!\n",
				msgs[i].addr);
			return -ENXIO;
		}
	}

	return 0;
}

/*
 * Restore all message address aliases with the original addresses.
 *
 * This function is internal for use in i2c_atr_master_xfer().
 *
 * @see i2c_atr_map_msgs()
 */
static void i2c_atr_unmap_msgs(struct i2c_atr_chan *chan,
			       struct i2c_msg msgs[], int num)
{
	int i;

	for (i = 0; i < num; i++)
		msgs[i].addr = chan->orig_addrs[i];
}

static int i2c_atr_master_xfer(struct i2c_adapter *adap,
			       struct i2c_msg msgs[], int num)
{
	struct i2c_atr_chan *chan = adap->algo_data;
	struct i2c_atr *atr = chan->atr;
	struct i2c_adapter *parent = atr->parent;
	int ret = 0;

	/* Switch to the right atr port */
	if (atr->ops->select) {
		ret = atr->ops->select(atr, chan->chan_id);
		if (ret < 0)
			goto out;
	}

	/* Translate addresses */
	mutex_lock(&chan->orig_addrs_lock);
	ret = i2c_atr_map_msgs(chan, msgs, num);
	if (ret < 0) {
		mutex_unlock(&chan->orig_addrs_lock);
		goto out;
	}

	/* Perform the transfer */
	ret = i2c_transfer(parent, msgs, num);

	/* Restore addresses */
	i2c_atr_unmap_msgs(chan, msgs, num);
	mutex_unlock(&chan->orig_addrs_lock);

out:
	if (atr->ops->deselect)
		atr->ops->deselect(atr, chan->chan_id);

	return ret;
}

static int i2c_atr_smbus_xfer(struct i2c_adapter *adap,
			      u16 addr, unsigned short flags,
			      char read_write, u8 command,
			      int size, union i2c_smbus_data *data)
{
	struct i2c_atr_chan *chan = adap->algo_data;
	struct i2c_atr *atr = chan->atr;
	struct i2c_adapter *parent = atr->parent;
	struct i2c_atr_cli2alias_pair *c2a;
	int err = 0;

	c2a = i2c_atr_find_mapping_by_addr(&chan->alias_list, addr);
	if (!c2a) {
		dev_err(atr->dev, "client 0x%02x not mapped!\n", addr);
		return -ENXIO;
	}

	if (atr->ops->select)
		err = atr->ops->select(atr, chan->chan_id);
	if (!err)
		err = i2c_smbus_xfer(parent, c2a->alias, flags,
				     read_write, command, size, data);
	if (atr->ops->deselect)
		atr->ops->deselect(atr, chan->chan_id);

	return err;
}

static u32 i2c_atr_functionality(struct i2c_adapter *adap)
{
	struct i2c_atr_chan *chan = adap->algo_data;
	struct i2c_adapter *parent = chan->atr->parent;

	return parent->algo->functionality(parent);
}

static void i2c_atr_lock_bus(struct i2c_adapter *adapter, unsigned int flags)
{
	struct i2c_atr_chan *chan = adapter->algo_data;
	struct i2c_atr *atr = chan->atr;

	mutex_lock(&atr->lock);
}

static int i2c_atr_trylock_bus(struct i2c_adapter *adapter, unsigned int flags)
{
	struct i2c_atr_chan *chan = adapter->algo_data;
	struct i2c_atr *atr = chan->atr;

	return mutex_trylock(&atr->lock);
}

static void i2c_atr_unlock_bus(struct i2c_adapter *adapter, unsigned int flags)
{
	struct i2c_atr_chan *chan = adapter->algo_data;
	struct i2c_atr *atr = chan->atr;

	mutex_unlock(&atr->lock);
}

static const struct i2c_lock_operations i2c_atr_lock_ops = {
	.lock_bus =    i2c_atr_lock_bus,
	.trylock_bus = i2c_atr_trylock_bus,
	.unlock_bus =  i2c_atr_unlock_bus,
};

static int i2c_atr_attach_client(struct i2c_adapter *adapter,
				 const struct i2c_board_info *info,
				 const struct i2c_client *client)
{
	struct i2c_atr_chan *chan = adapter->algo_data;
	struct i2c_atr *atr = chan->atr;
	struct i2c_atr_cli2alias_pair *c2a;
	u16 alias_id = 0;
	int err = 0;

	c2a = kzalloc(sizeof(struct i2c_atr_cli2alias_pair), GFP_KERNEL);
	if (!c2a) {
		err = -ENOMEM;
		goto err_alloc;
	}

	err = atr->ops->attach_client(atr, chan->chan_id, info, client,
				      &alias_id);
	if (err)
		goto err_attach;
	if (alias_id == 0) {
		err = -EINVAL;
		goto err_attach;
	}

	c2a->client = client;
	c2a->alias = alias_id;
	list_add(&c2a->node, &chan->alias_list);

	return 0;

err_attach:
	kfree(c2a);
err_alloc:
	return err;
}

static void i2c_atr_detach_client(struct i2c_adapter *adapter,
				  const struct i2c_client *client)
{
	struct i2c_atr_chan *chan = adapter->algo_data;
	struct i2c_atr *atr = chan->atr;
	struct i2c_atr_cli2alias_pair *c2a;

	atr->ops->detach_client(atr, chan->chan_id, client);

	c2a = i2c_atr_find_mapping_by_client(&chan->alias_list, client);
	if (c2a != NULL) {
		list_del(&c2a->node);
		kfree(c2a);
	}
}

static const struct i2c_attach_operations i2c_atr_attach_ops = {
	.attach_client = i2c_atr_attach_client,
	.detach_client = i2c_atr_detach_client,
};

/**
 * i2c_atr_add_adapter - Create a child ("downstream") I2C bus.
 * @atr:     The I2C ATR
 * @chan_id: Index of the new adapter (0 .. max_adapters-1).  This value is
 *           passed to the callbacks in `struct i2c_atr_ops`.
 *
 * After calling this function a new i2c bus will appear. Adding and
 * removing devices on the downstream bus will result in calls to the
 * `attach_client` and `detach_client` callbacks for the driver to assign
 * an alias to the device.
 *
 * If there is a device tree node under "i2c-atr" whose "reg" property
 * equals chan_id, the new adapter will receive that node and perhaps start
 * adding devices under it. The callbacks for those additions will be made
 * before i2c_atr_add_adapter() returns.
 *
 * Call i2c_atr_del_adapter() to remove the adapter.
 *
 * Return: 0 on success, a negative error code otherwise.
 */
int i2c_atr_add_adapter(struct i2c_atr *atr, u32 chan_id)
{
	struct i2c_adapter *parent = atr->parent;
	struct device *dev = atr->dev;
	struct i2c_atr_chan *chan;
	char symlink_name[20];
	int err;

	if (chan_id >= atr->max_adapters)
		return -EINVAL;

	if (atr->adapter[chan_id]) {
		dev_err(dev, "Adapter %d already present\n", chan_id);
		return -EEXIST;
	}

	chan = kzalloc(sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	chan->atr = atr;
	chan->chan_id = chan_id;
	INIT_LIST_HEAD(&chan->alias_list);
	mutex_init(&chan->orig_addrs_lock);

	snprintf(chan->adap.name, sizeof(chan->adap.name),
		 "i2c-%d-atr-%d", i2c_adapter_id(parent), chan_id);
	chan->adap.owner = THIS_MODULE;
	chan->adap.algo = &atr->algo;
	chan->adap.algo_data = chan;
	chan->adap.dev.parent = dev;
	chan->adap.retries = parent->retries;
	chan->adap.timeout = parent->timeout;
	chan->adap.quirks = parent->quirks;
	chan->adap.lock_ops = &i2c_atr_lock_ops;
	chan->adap.attach_ops = &i2c_atr_attach_ops;

	if (dev->of_node) {
		struct device_node *atr_node;
		struct device_node *child;
		u32 reg;

		atr_node = of_get_child_by_name(dev->of_node, "i2c-atr");

		for_each_child_of_node(atr_node, child) {
			err = of_property_read_u32(child, "reg", &reg);
			if (err)
				continue;
			if (chan_id == reg)
				break;
		}

		chan->adap.dev.of_node = child;
		of_node_put(atr_node);
	}

	err = i2c_add_adapter(&chan->adap);
	if (err) {
		dev_err(dev, "failed to add atr-adapter %u (error=%d)\n",
			chan_id, err);
		goto err_add_adapter;
	}

	WARN(sysfs_create_link(&chan->adap.dev.kobj, &dev->kobj, "atr_device"),
	     "can't create symlink to atr device\n");
	snprintf(symlink_name, sizeof(symlink_name), "channel-%u", chan_id);
	WARN(sysfs_create_link(&dev->kobj, &chan->adap.dev.kobj, symlink_name),
	     "can't create symlink for channel %u\n", chan_id);

	dev_dbg(dev, "Added ATR child bus %d\n", i2c_adapter_id(&chan->adap));

	atr->adapter[chan_id] = &chan->adap;
	return 0;

err_add_adapter:
	mutex_destroy(&chan->orig_addrs_lock);
	kfree(chan);
	return err;
}
EXPORT_SYMBOL_GPL(i2c_atr_add_adapter);

/**
 * i2c_atr_del_adapter - Remove a child ("downstream") I2C bus added by
 * i2c_atr_del_adapter().
 * @atr:     The I2C ATR
 * @chan_id: Index of the `adapter to be removed (0 .. max_adapters-1)
 */
void i2c_atr_del_adapter(struct i2c_atr *atr, u32 chan_id)
{
	char symlink_name[20];

	struct i2c_adapter *adap = atr->adapter[chan_id];
	struct i2c_atr_chan *chan = adap->algo_data;
	struct device_node *np = adap->dev.of_node;
	struct device *dev = atr->dev;

	if (atr->adapter[chan_id] == NULL) {
		dev_err(dev, "Adapter %d does not exist\n", chan_id);
		return;
	}

	dev_dbg(dev, "Removing ATR child bus %d\n", i2c_adapter_id(adap));

	atr->adapter[chan_id] = NULL;

	snprintf(symlink_name, sizeof(symlink_name),
		 "channel-%u", chan->chan_id);
	sysfs_remove_link(&dev->kobj, symlink_name);
	sysfs_remove_link(&chan->adap.dev.kobj, "atr_device");

	i2c_del_adapter(adap);
	of_node_put(np);
	mutex_destroy(&chan->orig_addrs_lock);
	kfree(chan->orig_addrs);
	kfree(chan);
}
EXPORT_SYMBOL_GPL(i2c_atr_del_adapter);

/**
 * i2c_atr_new() - Allocate and initialize an I2C ATR helper.
 * @parent:       The parent (upstream) adapter
 * @dev:          The device acting as an ATR
 * @ops:          Driver-specific callbacks
 * @max_adapters: Maximum number of child adapters
 *
 * The new ATR helper is connected to the parent adapter but has no child
 * adapters. Call i2c_atr_add_adapter() to add some.
 *
 * Call i2c_atr_delete() to remove.
 *
 * Return: pointer to the new ATR helper object, or ERR_PTR
 */
struct i2c_atr *i2c_atr_new(struct i2c_adapter *parent, struct device *dev,
			    const struct i2c_atr_ops *ops, int max_adapters)
{
	struct i2c_atr *atr;

	if (!ops || !ops->attach_client || !ops->detach_client)
		return ERR_PTR(-EINVAL);

	atr = devm_kzalloc(dev, sizeof(*atr)
			    + max_adapters * sizeof(atr->adapter[0]),
			    GFP_KERNEL);
	if (!atr)
		return ERR_PTR(-ENOMEM);

	mutex_init(&atr->lock);

	atr->parent = parent;
	atr->dev = dev;
	atr->ops = ops;
	atr->max_adapters = max_adapters;

	if (parent->algo->master_xfer)
		atr->algo.master_xfer = i2c_atr_master_xfer;
	if (parent->algo->smbus_xfer)
		atr->algo.smbus_xfer = i2c_atr_smbus_xfer;
	atr->algo.functionality = i2c_atr_functionality;

	return atr;
}
EXPORT_SYMBOL_GPL(i2c_atr_new);

/**
 * i2c_atr_delete - Delete an I2C ATR helper.
 * @atr: I2C ATR helper to be deleted.
 *
 * Precondition: all the adapters added with i2c_atr_add_adapter() mumst be
 * removed by calling i2c_atr_del_adapter().
 */
void i2c_atr_delete(struct i2c_atr *atr)
{
	mutex_destroy(&atr->lock);
}
EXPORT_SYMBOL_GPL(i2c_atr_delete);

MODULE_AUTHOR("Luca Ceresoli <luca@lucaceresoli.net>");
MODULE_DESCRIPTION("I2C Address Translator");
MODULE_LICENSE("GPL v2");
