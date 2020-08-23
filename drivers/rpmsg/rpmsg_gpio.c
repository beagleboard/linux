/*
 * PRU Remote Processor Messaging Driver with gpiochip interface
 * copyright (c) 2020 Deepankar Maithani
 * Codes examples from Lab no 5 from TI and has been used as boiler plate for rpmsg communication
 * https://processors.wiki.ti.com/index.php/PRU_Training:_Hands-on_Labs
 *
 * For learn more about the complete project visit:
 * https://github.com/deebot/Beaglebone-BidirectionBus/tree/dev
 * Steps to test the driver.
 * https://github.com/deebot/Beaglebone-BidirectionBus/blob/dev/bidirec_299/README.md
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/rpmsg.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kfifo.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/rpmsg/virtio_rpmsg.h>
#include <linux/stat.h>
#include <linux/bitops.h>
#define PRU_MAX_DEVICES				(16)
/* Matches the definition in virtio_rpmsg_bus.c */
#define RPMSG_BUF_SIZE				(512)
#define MAX_FIFO_MSG				(32)
#define FIFO_MSG_SIZE				RPMSG_BUF_SIZE
#define GPIO_NUM 9
static DEFINE_MUTEX(rpmsg_pru_lock);
static char rpmsg_pru_buf[RPMSG_BUF_SIZE];
static struct gpio_chip chip;
struct rpmsg_pru_dev {
	struct rpmsg_device *rpdev;
	struct device *dev;
	bool locked;
	dev_t devt;
	struct kfifo msg_fifo;
	u32 msg_len[MAX_FIFO_MSG];
	int msg_idx_rd;
	int msg_idx_wr;
	wait_queue_head_t wait_list;
	uint32_t gpio_state;
	long input_state;

};

struct rpmsg_pru_dev *prudev;
/*Reads the lines which are  set as input*/
static int mygpio_get_value(struct gpio_chip *gc, unsigned offset)
{
	uint32_t value;
	struct rpmsg_device *rpdev = container_of(gc->parent,
			struct rpmsg_device, dev);
	unsigned int mask = BIT(offset % 8);
	prudev = dev_get_drvdata(&rpdev->dev);

	value = (prudev->input_state & mask)>>offset;
	return value;
}
/* Writes to the lines and creates the gpio_state which is then sent as RPmsg */
static void mygpio_set_value(struct gpio_chip *gc, unsigned offset, int val)
{
	int ret;
	struct rpmsg_device *rpdev = container_of(gc->parent, struct rpmsg_device, dev);
	pr_info("set_value function triggered");
	pr_info("The bit number %d is set to value: %d", offset, val);

	prudev = dev_get_drvdata(&rpdev->dev);
	if (val == 0) {
		prudev->gpio_state &= ~(1<<offset);
	} else {
		prudev->gpio_state |= (1<<offset);
	}
	if (offset == GPIO_NUM-1) {
		/* copy the gpiostate in rpmsg buffer which will be sent over to PRU*/
		memcpy((void *)&rpmsg_pru_buf, (void *)&(prudev->gpio_state),
				 sizeof(&(prudev->gpio_state)));
		pr_info("A check for checking gpio_state: %d",
				prudev->gpio_state);
		/* This line actually sends the data to the other side*/
		ret = rpmsg_send(rpdev->ept, (void *)rpmsg_pru_buf, 2);
		if (ret)
			dev_err(gc->parent, "rpmsg_send failed: %d\n", ret);
	}

}

/*sets the  pin to output. Will be called when user sets one
 * of the gpiochip line as output which can be done manually
 * in sysfs or using libgpiod */

static int mygpio_direction_output(struct gpio_chip *gc,
				       unsigned offset, int val)
{
	pr_info("Direction of GPIO set to: out\n");
	return 0;
}
/*Runs When direction of a line is set as output*/
static int mygpio_direction_input(struct gpio_chip *gc,
				       unsigned offset)
{

	pr_info("Direction of GPIO set to: in \n");
    return 0;
}
/*This function gets called every time
 *an rpmsg_channel is created with a name that matches the .name
 *attribute of the rpmsg_driver_sample_id_table. It sets up suitable memory
 *and the gpiochip interface that can be seen in /sys/class/gpio and /dev.
 */
static int mygpio_rpmsg_pru_probe (struct rpmsg_device *rpdev)
{
	int ret;
	struct rpmsg_pru_dev *prudev;
	prudev = devm_kzalloc(&rpdev->dev, sizeof(*prudev), GFP_KERNEL);
	if (!prudev)
		return -ENOMEM;
	prudev->rpdev = rpdev;
	ret = kfifo_alloc(&prudev->msg_fifo, MAX_FIFO_MSG * FIFO_MSG_SIZE,
				  GFP_KERNEL);
	if (ret) {
			dev_err(&rpdev->dev, "Unable to allocate fifo for the rpmsg_pru device\n");
			return -ENOMEM;
		}
	init_waitqueue_head(&prudev->wait_list);
	dev_set_drvdata(&rpdev->dev, prudev);
	chip.label = rpdev->desc;
	chip.base = -1;
	chip.parent = &rpdev->dev;
	chip.owner = THIS_MODULE;
	chip.ngpio = GPIO_NUM;
	chip.can_sleep = 1;
	chip.get = mygpio_get_value;
	chip.set = mygpio_set_value;
	chip.direction_output = mygpio_direction_output;
	chip.direction_input = mygpio_direction_input;
	return gpiochip_add(&chip);
}
/* Callback function which gets called whenever a new rpmsg is received
 * The data received from the PRU is converted into long and then assigned to
 * input_state
 * @msg_fifo: kernel fifo used to buffer the messages between userspace and PRU
 * @msg_len: array storing the lengths of each message in the kernel fifo
 * @msg_idx_rd: kernel fifo read index
 * @msg_idx_wr: kernel fifo write index
 * */
static int mygpio_rpmsg_pru_cb(struct rpmsg_device *rpdev, void *data, int len,
			void *priv, u32 src)
{  int ret;
	u32 length;
	struct rpmsg_pru_dev *prudev;

	prudev = dev_get_drvdata(&rpdev->dev);

	if (kfifo_avail(&prudev->msg_fifo) < len) {
		dev_err(&rpdev->dev, "Not enough space on the FIFO\n");
		return -ENOSPC;
	}

	if ((prudev->msg_idx_wr + 1) % MAX_FIFO_MSG ==
		prudev->msg_idx_rd) {
		dev_err(&rpdev->dev, "Message length table is full\n");
		return -ENOSPC;
	}
   /* adds the data received into a fifo*/
	length = kfifo_in(&prudev->msg_fifo, data, len);
	prudev->msg_len[prudev->msg_idx_wr] = length;
	prudev->msg_idx_wr = (prudev->msg_idx_wr + 1) % MAX_FIFO_MSG;

	wake_up_interruptible(&prudev->wait_list);
	ret = kstrtol((char *) data, 10, &prudev->input_state);
	if (ret) {
	return ret;
	}
	pr_info("The shift register port state is: %ld", prudev->input_state);
	return 0;
}
static void mygpio_rpmsg_pru_remove(struct rpmsg_device *rpdev)
{
	struct rpmsg_pru_dev *prudev;
	prudev = dev_get_drvdata(&rpdev->dev);

	kfifo_free(&prudev->msg_fifo);
	gpiochip_remove(&chip);

}
/*
 * Matches this tag:If you change .name
 * PRU firmware should also be updated with same channel name
 */
static const struct rpmsg_device_id rpmsg_driver_pru_id_table[] = {
	{ .name	= "rpmsg-pru-gpio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_pru_id_table);

static struct rpmsg_driver rpmsg_pru_driver = {
	.drv.name	= KBUILD_MODNAME,
	.id_table	= rpmsg_driver_pru_id_table,
	.probe		= mygpio_rpmsg_pru_probe,
	.callback	= mygpio_rpmsg_pru_cb,
	.remove		= mygpio_rpmsg_pru_remove,
};

module_rpmsg_driver(rpmsg_pru_driver);
MODULE_AUTHOR("DeepankarMaithani <deepankar19910@gmail.com>");
MODULE_DESCRIPTION("A driver to send rpmsg data using sysfs and chardev interface");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

