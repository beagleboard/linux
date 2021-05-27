#include <linux/module.h>
#include <rtdm/analogy/device.h>

#define TASK_PERIOD 1000000

#define AI_SUBD 0
#define DIO_SUBD 1
#define AO_SUBD 2
#define AI2_SUBD 3

#define TRANSFER_SIZE 0x1000

/* --- Driver related structures --- */
struct fake_priv {
	/* Attach configuration parameters
	   (they should be relocated in ai_priv) */
	unsigned long amplitude_div;
	unsigned long quanta_cnt;

	/* Task descriptor */
	rtdm_task_t task;

	/* Statuses of the asynchronous subdevices */
	int ai_running;
	int ao_running;
	int ai2_running;
};

struct ai_priv {

	/* Specific timing fields */
	unsigned long scan_period_ns;
	unsigned long convert_period_ns;
	unsigned long current_ns;
	unsigned long reminder_ns;
	unsigned long long last_ns;

	/* Misc fields */
	unsigned long amplitude_div;
	unsigned long quanta_cnt;
};

struct ao_ai2_priv {
	/* Asynchronous loop stuff */
	uint8_t buffer[TRANSFER_SIZE];
	int count;
	/* Synchronous loop stuff */
	uint16_t insn_value;
};

struct dio_priv {
	/* Bits status */
	uint16_t bits_values;
};

/* --- Channels / ranges part --- */

/* Channels descriptors */

static struct a4l_channels_desc analog_chandesc = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = 8,
	.chans = {
		{A4L_CHAN_AREF_GROUND, 16},
	},
};

static struct a4l_channels_desc dio_chandesc = {
	.mode = A4L_CHAN_GLOBAL_CHANDESC,
	.length = 16,
	.chans = {
		{A4L_CHAN_AREF_GROUND, 1},
	},
};

/* Ranges tab */
static struct a4l_rngtab analog_rngtab = {
	.length = 2,
	.rngs = {
		RANGE_V(-5,5),
		RANGE_V(-10,10),
	},
};
/* Ranges descriptor */
static struct a4l_rngdesc analog_rngdesc = RNG_GLOBAL(analog_rngtab);

/* Command options masks */

static struct a4l_cmd_desc ai_cmd_mask = {
	.idx_subd = 0,
	.start_src = TRIG_NOW,
	.scan_begin_src = TRIG_TIMER,
	.convert_src = TRIG_NOW | TRIG_TIMER,
	.scan_end_src = TRIG_COUNT,
	.stop_src = TRIG_COUNT | TRIG_NONE,
};

static struct a4l_cmd_desc ao_cmd_mask = {
	.idx_subd = 0,
	.start_src = TRIG_NOW | TRIG_INT,
	.scan_begin_src = TRIG_TIMER,
	.convert_src = TRIG_NOW | TRIG_TIMER,
	.scan_end_src = TRIG_COUNT,
	.stop_src = TRIG_COUNT | TRIG_NONE,
};

/* --- Analog input simulation --- */

/* --- Values generation for 1st AI --- */

static inline uint16_t ai_value_output(struct ai_priv *priv)
{
	static uint16_t output_tab[8] = {
		0x0001, 0x2000, 0x4000, 0x6000,
		0x8000, 0xa000, 0xc000, 0xffff
	};
	static unsigned int output_idx;
	static DEFINE_RTDM_LOCK(output_lock);

	unsigned long flags;
	unsigned int idx;

	rtdm_lock_get_irqsave(&output_lock, flags);

	output_idx += priv->quanta_cnt;
	if(output_idx == 8)
		output_idx = 0;
	idx = output_idx;

	rtdm_lock_put_irqrestore(&output_lock, flags);

	return output_tab[idx] / priv->amplitude_div;
}

int ai_push_values(struct a4l_subdevice *subd)
{
	uint64_t now_ns, elapsed_ns = 0;
	struct a4l_cmd_desc *cmd;
	struct ai_priv *priv;
	int i = 0;

	if (!subd)
		return -EINVAL;

	priv = (struct ai_priv *)subd->priv;

	cmd = a4l_get_cmd(subd);
	if (!cmd)
		return -EPIPE;

	now_ns = a4l_get_time();
	elapsed_ns += now_ns - priv->last_ns + priv->reminder_ns;
	priv->last_ns = now_ns;

	while(elapsed_ns >= priv->scan_period_ns) {
		int j;

		for(j = 0; j < cmd->nb_chan; j++) {
			uint16_t value = ai_value_output(priv);
			a4l_buf_put(subd, &value, sizeof(uint16_t));
		}

		elapsed_ns -= priv->scan_period_ns;
		i++;
	}

	priv->current_ns += i * priv->scan_period_ns;
	priv->reminder_ns = elapsed_ns;

	if (i != 0)
		a4l_buf_evt(subd, 0);

	return 0;
}

/* --- Data retrieval for AO --- */

int ao_pull_values(struct a4l_subdevice *subd)
{
	struct ao_ai2_priv *priv = (struct ao_ai2_priv *)subd->priv;
	int err;

	/* Let's have a look at how many samples are available */
	priv->count = a4l_buf_count(subd) < TRANSFER_SIZE ?
		      a4l_buf_count(subd) : TRANSFER_SIZE;

	if (!priv->count)
		return 0;

	err = a4l_buf_get(subd, priv->buffer, priv->count);
	if (err < 0) {
		a4l_err(subd->dev, "ao_get_values: a4l_buf_get failed (err=%d)\n", err);
		priv->count = 0;
		return err;

	}

	a4l_info(subd->dev, " %d bytes added to private buffer from async p=%p\n",
		priv->count, subd->buf->buf);

	a4l_buf_evt(subd, 0);

	return 0;
}

/* --- Data redirection for 2nd AI (from AO) --- */

int ai2_push_values(struct a4l_subdevice *subd)
{
	struct ao_ai2_priv *priv = *((struct ao_ai2_priv **)subd->priv);
	int err = 0;

	if (priv->count) {
		err = a4l_buf_put(subd, priv->buffer, priv->count);

		/* If there is no more place in the asynchronous
		buffer, data are likely to be dropped; it is just a
		test driver so no need to implement trickier mechanism */
		err = (err == -EAGAIN) ? 0 : err;

		a4l_info(subd->dev, "%d bytes added to async buffer p=%p\n",
			priv->count, subd->buf->buf);

		priv->count = 0;
		if (err < 0)
			a4l_err(subd->dev,
				"ai2_push_values: "
				"a4l_buf_put failed (err=%d)\n", err);
		else
			a4l_buf_evt(subd, 0);
	}

	return err;
}

/* --- Asynchronous AI functions --- */

static int ai_cmd(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	struct fake_priv *priv = (struct fake_priv *)subd->dev->priv;
	struct ai_priv *ai_priv = (struct ai_priv *)subd->priv;

	ai_priv->scan_period_ns = cmd->scan_begin_arg;
	ai_priv->convert_period_ns = (cmd->convert_src==TRIG_TIMER)?
		cmd->convert_arg:0;

	a4l_dbg(1, drv_dbg, subd->dev, "scan_period=%luns convert_period=%luns\n",
		ai_priv->scan_period_ns, ai_priv->convert_period_ns);

	ai_priv->last_ns = a4l_get_time();

	ai_priv->current_ns = ((unsigned long)ai_priv->last_ns);
	ai_priv->reminder_ns = 0;

	priv->ai_running = 1;

	return 0;

}

static int ai_cmdtest(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	if(cmd->scan_begin_src == TRIG_TIMER)
	{
		if (cmd->scan_begin_arg < 1000)
			return -EINVAL;

		if (cmd->convert_src == TRIG_TIMER &&
		    cmd->scan_begin_arg < (cmd->convert_arg * cmd->nb_chan))
			return -EINVAL;
	}

	return 0;
}

static void ai_cancel(struct a4l_subdevice *subd)
{
	struct fake_priv *priv = (struct fake_priv *)subd->dev->priv;

	priv->ai_running = 0;
}

static void ai_munge(struct a4l_subdevice *subd, void *buf, unsigned long size)
{
	int i;

	for(i = 0; i < size / sizeof(uint16_t); i++)
		((uint16_t *)buf)[i] += 1;
}

/* --- Asynchronous A0 functions --- */

int ao_cmd(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	a4l_info(subd->dev, "(subd=%d)\n", subd->idx);
	return 0;
}

int ao_trigger(struct a4l_subdevice *subd, lsampl_t trignum)
{
	struct fake_priv *priv = (struct fake_priv *)subd->dev->priv;

	a4l_info(subd->dev, "(subd=%d)\n", subd->idx);
	priv->ao_running = 1;
	return 0;
}

void ao_cancel(struct a4l_subdevice *subd)
{
	struct fake_priv *priv = (struct fake_priv *)subd->dev->priv;
	struct ao_ai2_priv *ao_priv = (struct ao_ai2_priv *)subd->priv;
	int running;

	a4l_info(subd->dev, "(subd=%d)\n", subd->idx);
	priv->ao_running = 0;

	running = priv->ai2_running;
	if (running) {
		struct a4l_subdevice *ai2_subd =
			(struct a4l_subdevice *)a4l_get_subd(subd->dev, AI2_SUBD);
		/* Here, we have not saved the required amount of
		   data; so, we cannot know whether or not, it is the
		   end of the acquisition; that is why we force it */
		priv->ai2_running = 0;
		ao_priv->count = 0;

		a4l_info(subd->dev, "subd %d cancelling subd %d too \n",
			subd->idx, AI2_SUBD);

		a4l_buf_evt(ai2_subd, A4L_BUF_EOA);
	}
}

/* --- Asynchronous 2nd AI functions --- */

int ai2_cmd(struct a4l_subdevice *subd, struct a4l_cmd_desc *cmd)
{
	struct fake_priv *priv = (struct fake_priv *)subd->dev->priv;

	a4l_info(subd->dev, "(subd=%d)\n", subd->idx);
	priv->ai2_running = 1;
	return 0;
}

void ai2_cancel(struct a4l_subdevice *subd)
{
	struct fake_priv *priv = (struct fake_priv *)subd->dev->priv;
	struct ao_ai2_priv *ai2_priv = *((struct ao_ai2_priv **)subd->priv);

	int running;

	a4l_info(subd->dev, "(subd=%d)\n", subd->idx);
	priv->ai2_running = 0;

	running = priv->ao_running;
	if (running) {
		struct a4l_subdevice *ao_subd =
			(struct a4l_subdevice *)a4l_get_subd(subd->dev, AO_SUBD);
		/* Here, we have not saved the required amount of
		   data; so, we cannot know whether or not, it is the
		   end of the acquisition; that is why we force it */
		priv->ao_running = 0;
		ai2_priv->count = 0;

		a4l_info(subd->dev, "subd %d cancelling subd %d too \n",
			 subd->idx, AO_SUBD);

		a4l_buf_evt(ao_subd, A4L_BUF_EOA);
	}

}


/* --- Synchronous AI functions --- */

static int ai_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct ai_priv *priv = (struct ai_priv *)subd->priv;
	uint16_t *data = (uint16_t *)insn->data;
	int i;

	for(i = 0; i < insn->data_size / sizeof(uint16_t); i++)
		data[i] = ai_value_output(priv);

	return 0;
}

/* --- Synchronous DIO function --- */

static int dio_insn_bits(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct dio_priv *priv = (struct dio_priv *)subd->priv;
	uint16_t *data = (uint16_t *)insn->data;

	if (insn->data_size != 2 * sizeof(uint16_t))
		return -EINVAL;

	if (data[0] != 0) {
		priv->bits_values &= ~(data[0]);
		priv->bits_values |= (data[0] & data[1]);
	}

	data[1] = priv->bits_values;

	return 0;
}

/* --- Synchronous AO + AI2 functions --- */

int ao_insn_write(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct ao_ai2_priv *priv = (struct ao_ai2_priv *)subd->priv;
	uint16_t *data = (uint16_t *)insn->data;

	/* Checks the buffer size */
	if (insn->data_size != sizeof(uint16_t))
		return -EINVAL;

	/* Retrieves the value to memorize */
	priv->insn_value = data[0];

	return 0;
}

int ai2_insn_read(struct a4l_subdevice *subd, struct a4l_kernel_instruction *insn)
{
	struct ao_ai2_priv *priv = *((struct ao_ai2_priv **)subd->priv);
	uint16_t *data = (uint16_t *)insn->data;

	/* Checks the buffer size */
	if (insn->data_size != sizeof(uint16_t))
		return -EINVAL;

	/* Sets the memorized value */
	data[0] = priv->insn_value;

	return 0;
}

/* --- Global task part --- */

/* One task is enough for all the asynchronous subdevices, it is just a fake
 * driver after all
 */
static void task_proc(void *arg)
{
	struct a4l_subdevice *ai_subd, *ao_subd, *ai2_subd;
	struct a4l_device *dev;
	struct fake_priv *priv;
	int running;

	dev = arg;
	ai_subd = a4l_get_subd(dev, AI_SUBD);
	ao_subd = a4l_get_subd(dev, AO_SUBD);
	ai2_subd = a4l_get_subd(dev, AI2_SUBD);

	priv = dev->priv;

	while(!rtdm_task_should_stop()) {

		/* copy sample static data from the subd private buffer to the
		 * asynchronous buffer
		 */
		running = priv->ai_running;
		if (running && ai_push_values(ai_subd) < 0) {
			/* on error, wait for detach to destroy the task */
			rtdm_task_sleep(RTDM_TIMEOUT_INFINITE);
			continue;
		}

		/*
		 * pull the data from the output subdevice (asynchronous buffer)
		 * into its private buffer
		 */
		running = priv->ao_running;
		if (running && ao_pull_values(ao_subd) < 0) {
			rtdm_task_sleep(RTDM_TIMEOUT_INFINITE);
			continue;
		}

		running = priv->ai2_running;
		/*
		 * then loop it to the ai2 subd since their private data is shared: so
		 * pull the data from the private buffer back into the device's
		 * asynchronous buffer
		 */
		if (running && ai2_push_values(ai2_subd) < 0) {
			rtdm_task_sleep(RTDM_TIMEOUT_INFINITE);
			continue;
		}

		rtdm_task_sleep(TASK_PERIOD);
	}
}

/* --- Initialization functions --- */

void setup_ai_subd(struct a4l_subdevice *subd)
{
	/* Fill the subdevice structure */
	subd->flags |= A4L_SUBD_AI;
	subd->flags |= A4L_SUBD_CMD;
	subd->flags |= A4L_SUBD_MMAP;
	subd->rng_desc = &analog_rngdesc;
	subd->chan_desc = &analog_chandesc;
	subd->do_cmd = ai_cmd;
	subd->do_cmdtest = ai_cmdtest;
	subd->cancel = ai_cancel;
	subd->munge = ai_munge;
	subd->cmd_mask = &ai_cmd_mask;
	subd->insn_read = ai_insn_read;
}

void setup_dio_subd(struct a4l_subdevice *subd)
{
	/* Fill the subdevice structure */
	subd->flags |= A4L_SUBD_DIO;
	subd->chan_desc = &dio_chandesc;
	subd->rng_desc = &range_digital;
	subd->insn_bits = dio_insn_bits;
}

void setup_ao_subd(struct a4l_subdevice *subd)
{
	/* Fill the subdevice structure */
	subd->flags |= A4L_SUBD_AO;
	subd->flags |= A4L_SUBD_CMD;
	subd->flags |= A4L_SUBD_MMAP;
	subd->rng_desc = &analog_rngdesc;
	subd->chan_desc = &analog_chandesc;
	subd->do_cmd = ao_cmd;
	subd->cancel = ao_cancel;
	subd->trigger = ao_trigger;
	subd->cmd_mask = &ao_cmd_mask;
	subd->insn_write = ao_insn_write;
}

void setup_ai2_subd(struct a4l_subdevice *subd)
{
	/* Fill the subdevice structure */
	subd->flags |= A4L_SUBD_AI;
	subd->flags |= A4L_SUBD_CMD;
	subd->flags |= A4L_SUBD_MMAP;
	subd->rng_desc = &analog_rngdesc;
	subd->chan_desc = &analog_chandesc;
	subd->do_cmd = ai2_cmd;
	subd->cancel = ai2_cancel;
	subd->cmd_mask = &ai_cmd_mask;
	subd->insn_read = ai2_insn_read;
}

/* --- Attach / detach functions ---  */

int test_attach(struct a4l_device *dev, a4l_lnkdesc_t *arg)
{
	typedef void (*setup_subd_function) (struct a4l_subdevice *subd);
	struct fake_priv *priv = (struct fake_priv *) dev->priv;
	struct a4l_subdevice *subd;
	unsigned long tmp;
	struct ai_priv *r;
	int i, ret = 0;

	struct initializers {
		struct a4l_subdevice *subd;
		setup_subd_function init;
		int private_len;
		char *name;
		int index;
	} sds[] = {
		[AI_SUBD] = {
			.name = "AI",
			.private_len = sizeof(struct ai_priv),
			.init = setup_ai_subd,
			.index = AI_SUBD,
			.subd = NULL,
		},
		[DIO_SUBD] = {
			.name = "DIO",
			.private_len = sizeof(struct dio_priv),
			.init = setup_dio_subd,
			.index = DIO_SUBD,
			.subd = NULL,
		},
		[AO_SUBD] = {
			.name = "AO",
			.private_len = sizeof(struct ao_ai2_priv),
			.init = setup_ao_subd,
			.index = AO_SUBD,
			.subd = NULL,
		},
		[AI2_SUBD] = {
			.name = "AI2",
			.private_len = sizeof(struct ao_ai2_priv *),
			.init = setup_ai2_subd,
			.index = AI2_SUBD,
			.subd = NULL,
		},
	};

	a4l_dbg(1, drv_dbg, dev, "starting attach procedure...\n");

	/* Set default values for attach parameters */
	priv->amplitude_div = 1;
	priv->quanta_cnt = 1;
	if (arg->opts_size) {
		unsigned long *args = (unsigned long *)arg->opts;
		priv->amplitude_div = args[0];
		if (arg->opts_size == 2 * sizeof(unsigned long))
			priv->quanta_cnt = (args[1] > 7 || args[1] == 0) ?
				1 : args[1];
	}

	/* create and register the subdevices */
	for (i = 0; i < ARRAY_SIZE(sds) ; i++) {

		subd = a4l_alloc_subd(sds[i].private_len, sds[i].init);
		if (subd == NULL)
			return -ENOMEM;

		ret = a4l_add_subd(dev, subd);
		if (ret != sds[i].index)
			return (ret < 0) ? ret : -EINVAL;

		sds[i].subd = subd;

		a4l_dbg(1, drv_dbg, dev, " %s subdev registered \n", sds[i].name);
	}

	/* initialize specifics */
	r = (void *) sds[AI_SUBD].subd->priv;
	r->amplitude_div = priv->amplitude_div;
	r->quanta_cnt = priv->quanta_cnt;

	/* A0 and AI2 shared their private buffers */
	tmp = (unsigned long) sds[AO_SUBD].subd->priv;
	memcpy(sds[AI2_SUBD].subd->priv, &tmp, sds[AI2_SUBD].private_len) ;

	/* create the task */
	ret = rtdm_task_init(&priv->task, "Fake AI task", task_proc, dev,
		             RTDM_TASK_HIGHEST_PRIORITY, 0);
	if (ret)
		a4l_dbg(1, drv_dbg, dev, "Error creating A4L task \n");

	a4l_dbg(1, drv_dbg, dev, "attach procedure completed: "
				 "adiv = %lu, qcount = %lu \n"
		                  , priv->amplitude_div, priv->quanta_cnt);

	return ret;
}

int test_detach(struct a4l_device *dev)
{
	struct fake_priv *priv = (struct fake_priv *)dev->priv;

	rtdm_task_destroy(&priv->task);
	a4l_dbg(1, drv_dbg, dev, "detach procedure complete\n");

	return 0;
}

/* --- Module stuff --- */

static struct a4l_driver test_drv = {
	.owner = THIS_MODULE,
	.board_name = "analogy_fake",
	.driver_name = "fake",
	.attach = test_attach,
	.detach = test_detach,
	.privdata_size = sizeof(struct fake_priv),
};

static int __init a4l_fake_init(void)
{
	return a4l_register_drv(&test_drv);
}

static void __exit a4l_fake_cleanup(void)
{
	a4l_unregister_drv(&test_drv);
}

MODULE_DESCRIPTION("Analogy fake driver");
MODULE_LICENSE("GPL");

module_init(a4l_fake_init);
module_exit(a4l_fake_cleanup);
