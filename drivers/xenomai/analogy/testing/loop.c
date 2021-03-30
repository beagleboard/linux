#include <linux/module.h>
#include <rtdm/analogy/device.h>

#define LOOP_TASK_PERIOD 1000000
#define LOOP_NB_BITS 16

#define LOOP_INPUT_SUBD 0
#define LOOP_OUTPUT_SUBD 1

/* Channels descriptor */
static struct a4l_channels_desc loop_chandesc = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = 8,
	.chans = {
		{A4L_CHAN_AREF_GROUND, LOOP_NB_BITS},
	},
};

/* Ranges tab */
static struct a4l_rngtab loop_rngtab = {
	.length =  2,
	.rngs = {
		RANGE_V(-5,5),
		RANGE_V(-10,10),
	},
};
/* Ranges descriptor */
struct a4l_rngdesc loop_rngdesc = RNG_GLOBAL(loop_rngtab);

/* Command options mask */
static struct a4l_cmd_desc loop_cmd_mask = {
	.idx_subd = 0,
	.start_src = TRIG_NOW | TRIG_INT,
	.scan_begin_src = TRIG_TIMER,
	.convert_src = TRIG_NOW | TRIG_TIMER,
	.scan_end_src = TRIG_COUNT,
	.stop_src = TRIG_COUNT| TRIG_NONE,
};

/* Private data organization */
struct loop_priv {

	/* Task descriptor */
	rtdm_task_t loop_task;

	/* Misc fields */
	int loop_running;
	uint16_t loop_insn_value;
};
typedef struct loop_priv lpprv_t;

/* Attach arguments contents */
struct loop_attach_arg {
	unsigned long period;
};
typedef struct loop_attach_arg lpattr_t;

static void loop_task_proc(void *arg);

/* --- Task part --- */

/* Timer task routine  */
static void loop_task_proc(void *arg)
{
	struct a4l_device *dev = (struct a4l_device*)arg;
	struct a4l_subdevice *input_subd, *output_subd;
	lpprv_t *priv = (lpprv_t *)dev->priv;

	input_subd = a4l_get_subd(dev, LOOP_INPUT_SUBD);
	output_subd = a4l_get_subd(dev, LOOP_OUTPUT_SUBD);

	if (input_subd == NULL || output_subd == NULL) {
		a4l_err(dev, "loop_task_proc: subdevices unavailable\n");
		return;
	}

	while (1) {

		int running;

		running = priv->loop_running;

		if (running) {
			uint16_t value;
			int ret=0;

			while (ret==0) {

				ret = a4l_buf_get(output_subd,
						  &value, sizeof(uint16_t));
				if (ret == 0) {

					a4l_info(dev,
						 "loop_task_proc: "
						 "data available\n");

					a4l_buf_evt(output_subd, 0);

					ret = a4l_buf_put(input_subd,
							  &value,
							  sizeof(uint16_t));

					if (ret==0)
						a4l_buf_evt(input_subd, 0);
				}
			}
		}

		rtdm_task_sleep(LOOP_TASK_PERIOD);
	}
}

/* --- Analogy Callbacks --- */

/* Command callback */
int loop_cmd(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	a4l_info(subd->dev, "loop_cmd: (subd=%d)\n", subd->idx);

	return 0;

}

/* Trigger callback */
int loop_trigger(struct a4l_subdevice *subd, lsampl_t trignum)
{
	lpprv_t *priv = (lpprv_t *)subd->dev->priv;

	a4l_info(subd->dev, "loop_trigger: (subd=%d)\n", subd->idx);

	priv->loop_running = 1;

	return 0;
}

/* Cancel callback */
void loop_cancel(struct a4l_subdevice *subd)
{
	lpprv_t *priv = (lpprv_t *)subd->dev->priv;

	a4l_info(subd->dev, "loop_cancel: (subd=%d)\n", subd->idx);

	priv->loop_running = 0;
}

/* Read instruction callback */
int loop_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	lpprv_t *priv = (lpprv_t*)subd->dev->priv;
	uint16_t *data = (uint16_t *)insn->data;

	/* Checks the buffer size */
	if (insn->data_size != sizeof(uint16_t))
		return -EINVAL;

	/* Sets the memorized value */
	data[0] = priv->loop_insn_value;

	return 0;
}

/* Write instruction callback */
int loop_insn_write(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	lpprv_t *priv = (lpprv_t*)subd->dev->priv;
	uint16_t *data = (uint16_t *)insn->data;

	/* Checks the buffer size */
	if (insn->data_size != sizeof(uint16_t))
		return -EINVAL;

	/* Retrieves the value to memorize */
	priv->loop_insn_value = data[0];

	return 0;
}

void setup_input_subd(struct a4l_subdevice *subd)
{
	memset(subd, 0, sizeof(struct a4l_subdevice));

	subd->flags |= A4L_SUBD_AI;
	subd->flags |= A4L_SUBD_CMD;
	subd->flags |= A4L_SUBD_MMAP;
	subd->rng_desc = &loop_rngdesc;
	subd->chan_desc = &loop_chandesc;
	subd->do_cmd = loop_cmd;
	subd->cancel = loop_cancel;
	subd->cmd_mask = &loop_cmd_mask;
	subd->insn_read = loop_insn_read;
	subd->insn_write = loop_insn_write;
}

void setup_output_subd(struct a4l_subdevice *subd)
{
	memset(subd, 0, sizeof(struct a4l_subdevice));

	subd->flags = A4L_SUBD_AO;
	subd->flags |= A4L_SUBD_CMD;
	subd->flags |= A4L_SUBD_MMAP;
	subd->rng_desc = &loop_rngdesc;
	subd->chan_desc = &loop_chandesc;
	subd->do_cmd = loop_cmd;
	subd->cancel = loop_cancel;
	subd->trigger = loop_trigger;
	subd->cmd_mask = &loop_cmd_mask;
	subd->insn_read = loop_insn_read;
	subd->insn_write = loop_insn_write;
}

/* Attach callback */
int loop_attach(struct a4l_device *dev,
		a4l_lnkdesc_t *arg)
{
	int ret = 0;
	struct a4l_subdevice *subd;
	lpprv_t *priv = (lpprv_t *)dev->priv;

	/* Add the fake input subdevice */
	subd = a4l_alloc_subd(0, setup_input_subd);
	if (subd == NULL)
		return -ENOMEM;

	ret = a4l_add_subd(dev, subd);
	if (ret != LOOP_INPUT_SUBD)
		/* Let Analogy free the lately allocated subdevice */
		return (ret < 0) ? ret : -EINVAL;

	/* Add the fake output subdevice */
	subd = a4l_alloc_subd(0, setup_output_subd);
	if (subd == NULL)
		/* Let Analogy free the lately allocated subdevice */
		return -ENOMEM;

	ret = a4l_add_subd(dev, subd);
	if (ret != LOOP_OUTPUT_SUBD)
		/* Let Analogy free the lately allocated subdevices */
		return (ret < 0) ? ret : -EINVAL;

	priv->loop_running = 0;
	priv->loop_insn_value = 0;

	ret = rtmd_task_init(&priv->loop_task,
			    "a4l_loop task",
			    loop_task_proc,
			    dev, RTDM_TASK_HIGHEST_PRIORITY, 0);

	return ret;
}

/* Detach callback */
int loop_detach(struct a4l_device *dev)
{
	lpprv_t *priv = (lpprv_t *)dev->priv;

	rtdm_task_destroy(&priv->loop_task);

	return 0;
}

/* --- Module part --- */

static struct a4l_driver loop_drv = {
	.owner = THIS_MODULE,
	.board_name = "analogy_loop",
	.attach = loop_attach,
	.detach = loop_detach,
	.privdata_size = sizeof(lpprv_t),
};

static int __init a4l_loop_init(void)
{
	return a4l_register_drv(&loop_drv);
}

static void __exit a4l_loop_cleanup(void)
{
	a4l_unregister_drv(&loop_drv);
}

MODULE_DESCRIPTION("Analogy loop driver");
MODULE_LICENSE("GPL");

module_init(a4l_loop_init);
module_exit(a4l_loop_cleanup);
