/*
 * pps-gmtimer.c -- PPS client driver using OMAP Timers
 *
 * Copyright (C) 2014  Daniel Drown <dan-android@drown.org>
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * ref1 - http://www.kunen.org/uC/beagle/omap_dmtimer.html
 * ref2 - [kernel] linux/samples/kobject/kobject-example.c
 * ref3 - [kernel] linux/arch/arm/mach-omap2/timer.c
 * ref4 - am335x trm on ti's website
 * ref5 - linux/drivers/pps/clients/pps-gpio.c
 *
 * required in kernel config - CONFIG_OF
 */

#define MODULE_NAME "pps-gmtimer"
#define pr_fmt(fmt) MODULE_NAME ": " fmt

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pps_kernel.h>
#include <linux/clocksource.h>

#include <plat/dmtimer.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dan Drown");
MODULE_DESCRIPTION("PPS Client Driver using OMAP Timer hardware");
MODULE_VERSION("0.1.0");

struct pps_gmtimer_platform_data {
  struct omap_dm_timer *capture_timer;
  const char *timer_name;
  uint32_t frequency;
  unsigned int capture;
  unsigned int overflow;
  unsigned int count_at_interrupt;
  struct pps_event_time ts;
  struct timespec delta;
  struct pps_device *pps;
  struct pps_source_info info;
  int ready;
  struct clocksource clksrc;
};

/* kobject *******************/
static ssize_t timer_name_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%s\n", pdata->timer_name);
}

static DEVICE_ATTR(timer_name, S_IRUGO, timer_name_show, NULL);

static ssize_t stats_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "capture: %u\noverflow: %u\n", pdata->capture, pdata->overflow);
}

static DEVICE_ATTR(stats, S_IRUGO, stats_show, NULL);

static ssize_t interrupt_delta_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%lld.%09ld\n", (long long)pdata->delta.tv_sec, pdata->delta.tv_nsec);
}

static DEVICE_ATTR(interrupt_delta, S_IRUGO, interrupt_delta_show, NULL);

static ssize_t pps_ts_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%lld.%09ld\n", (long long)pdata->ts.ts_real.tv_sec, pdata->ts.ts_real.tv_nsec);
}

static DEVICE_ATTR(pps_ts, S_IRUGO, pps_ts_show, NULL);

static ssize_t count_at_interrupt_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%u\n", pdata->count_at_interrupt);
}

static DEVICE_ATTR(count_at_interrupt, S_IRUGO, count_at_interrupt_show, NULL);

static ssize_t capture_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%u\n",
      __omap_dm_timer_read(pdata->capture_timer, OMAP_TIMER_CAPTURE_REG, pdata->capture_timer->posted)
      );
}

static DEVICE_ATTR(capture, S_IRUGO, capture_show, NULL);

static ssize_t ctrlstatus_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  return sprintf(buf, "%x\n",
      __omap_dm_timer_read(pdata->capture_timer, OMAP_TIMER_CTRL_REG, pdata->capture_timer->posted)
      );
}

static DEVICE_ATTR(ctrlstatus, S_IRUGO, ctrlstatus_show, NULL);

static ssize_t timer_counter_show(struct device *dev, struct device_attribute *attr, char *buf) {
  struct pps_gmtimer_platform_data *pdata = dev->platform_data;
  unsigned int current_count = 0;

  current_count = omap_dm_timer_read_counter(pdata->capture_timer);
  return sprintf(buf, "%u\n", current_count);
}

static DEVICE_ATTR(timer_counter, S_IRUGO, timer_counter_show, NULL);

static struct attribute *attrs[] = {
   &dev_attr_timer_counter.attr,
   &dev_attr_ctrlstatus.attr,
   &dev_attr_capture.attr,
   &dev_attr_count_at_interrupt.attr,
   &dev_attr_pps_ts.attr,
   &dev_attr_interrupt_delta.attr,
   &dev_attr_stats.attr,
   &dev_attr_timer_name.attr,
   NULL,
};

static struct attribute_group attr_group = {
   .attrs = attrs,
};

/* timers ********************/
static irqreturn_t pps_gmtimer_interrupt(int irq, void *data) {
  struct pps_gmtimer_platform_data *pdata;

  pdata = data;

  if(pdata->ready) {
    unsigned int irq_status;

    irq_status = omap_dm_timer_read_status(pdata->capture_timer);
    if(irq_status & OMAP_TIMER_INT_CAPTURE) {
      uint32_t ps_per_hz;
      unsigned int count_at_capture;

      pps_get_ts(&pdata->ts);
      pdata->count_at_interrupt = omap_dm_timer_read_counter(pdata->capture_timer);
      count_at_capture = __omap_dm_timer_read(pdata->capture_timer, OMAP_TIMER_CAPTURE_REG, pdata->capture_timer->posted);

      pdata->delta.tv_sec = 0;

      // use picoseconds per hz to avoid floating point and limit the rounding error
      ps_per_hz = 1000000000 / (pdata->frequency / 1000);
      pdata->delta.tv_nsec = ((pdata->count_at_interrupt - count_at_capture) * ps_per_hz) / 1000;

      pps_sub_ts(&pdata->ts, pdata->delta);
      pps_event(pdata->pps, &pdata->ts, PPS_CAPTUREASSERT, NULL);

      pdata->capture++;

      __omap_dm_timer_write_status(pdata->capture_timer, OMAP_TIMER_INT_CAPTURE);
    }
    if(irq_status & OMAP_TIMER_INT_OVERFLOW) {
      pdata->overflow++;
      __omap_dm_timer_write_status(pdata->capture_timer, OMAP_TIMER_INT_OVERFLOW);
    }
  }

  return IRQ_HANDLED; // TODO: shared interrupts?
}

static void omap_dm_timer_setup_capture(struct omap_dm_timer *timer) {
  u32 ctrl;

  omap_dm_timer_set_source(timer, OMAP_TIMER_SRC_SYS_CLK);

  omap_dm_timer_enable(timer);

  ctrl = __omap_dm_timer_read(timer, OMAP_TIMER_CTRL_REG, timer->posted);

  // disable prescaler
  ctrl &= ~(OMAP_TIMER_CTRL_PRE | (0x07 << 2));

  // autoreload
  ctrl |= OMAP_TIMER_CTRL_AR;
  __omap_dm_timer_write(timer, OMAP_TIMER_LOAD_REG, 0, timer->posted);

  // start timer
  ctrl |= OMAP_TIMER_CTRL_ST;

  // set capture
  ctrl |= OMAP_TIMER_CTRL_TCM_LOWTOHIGH | OMAP_TIMER_CTRL_GPOCFG; // TODO: configurable direction

  __omap_dm_timer_load_start(timer, ctrl, 0, timer->posted);

  /* Save the context */
  timer->context.tclr = ctrl;
  timer->context.tldr = 0;
  timer->context.tcrr = 0;
}

/* if tclkin has no clock, writes to the timer registers will stall and you will get a message like:
 * Unhandled fault: external abort on non-linefetch (0x1028) at 0xfa044048
 */
static void omap_dm_timer_use_tclkin(struct pps_gmtimer_platform_data *pdata) {
  struct clk *gt_fclk;

  omap_dm_timer_set_source(pdata->capture_timer, OMAP_TIMER_SRC_EXT_CLK);
  gt_fclk = omap_dm_timer_get_fclk(pdata->capture_timer);
  pdata->frequency = clk_get_rate(gt_fclk);
  pr_info("timer(%s) switched to tclkin, rate=%uHz\n", pdata->timer_name, pdata->frequency);
}

static void pps_gmtimer_enable_irq(struct pps_gmtimer_platform_data *pdata) {
  unsigned int interrupt_mask;

  interrupt_mask = OMAP_TIMER_INT_CAPTURE|OMAP_TIMER_INT_OVERFLOW;
  __omap_dm_timer_int_enable(pdata->capture_timer, interrupt_mask);
  pdata->capture_timer->context.tier = interrupt_mask;
  pdata->capture_timer->context.twer = interrupt_mask;
}

static int pps_gmtimer_init_timer(struct device_node *timer_dn, struct pps_gmtimer_platform_data *pdata) {
  struct clk *gt_fclk;

  of_property_read_string_index(timer_dn, "ti,hwmods", 0, &pdata->timer_name);
  if (!pdata->timer_name) {
    pr_err("ti,hwmods property missing?\n");
    return -ENODEV;
  }

  pdata->capture_timer = omap_dm_timer_request_by_node(timer_dn);
  if(!pdata->capture_timer) {
    pr_err("request_by_node failed\n");
    return -ENODEV;
  }

  // TODO: use devm_request_irq?
  if(request_irq(pdata->capture_timer->irq, pps_gmtimer_interrupt, IRQF_TIMER, MODULE_NAME, pdata)) {
    pr_err("cannot register IRQ %d\n", pdata->capture_timer->irq);
    return -EIO;
  }

  omap_dm_timer_setup_capture(pdata->capture_timer);

  gt_fclk = omap_dm_timer_get_fclk(pdata->capture_timer);
  pdata->frequency = clk_get_rate(gt_fclk);

  pr_info("timer name=%s rate=%uHz\n", pdata->timer_name, pdata->frequency);

  return 0;
}

static void pps_gmtimer_cleanup_timer(struct pps_gmtimer_platform_data *pdata) {
  if(pdata->capture_timer) {
    omap_dm_timer_set_source(pdata->capture_timer, OMAP_TIMER_SRC_SYS_CLK); // in case TCLKIN is stopped during boot
    omap_dm_timer_set_int_disable(pdata->capture_timer, OMAP_TIMER_INT_CAPTURE|OMAP_TIMER_INT_OVERFLOW);
    free_irq(pdata->capture_timer->irq, pdata);
    omap_dm_timer_stop(pdata->capture_timer);
    omap_dm_timer_free(pdata->capture_timer);
    pdata->capture_timer = NULL;
    pr_info("Exiting.\n");
  }
}

/* clocksource ***************/
static struct pps_gmtimer_platform_data *clocksource_timer = NULL;

static cycle_t pps_gmtimer_read_cycles(struct clocksource *cs) {
  return (cycle_t)__omap_dm_timer_read_counter(clocksource_timer->capture_timer, clocksource_timer->capture_timer->posted);
}

static void pps_gmtimer_clocksource_init(struct pps_gmtimer_platform_data *pdata) {
  if(!clocksource_timer) {
    pdata->clksrc.name = pdata->timer_name;

    pdata->clksrc.rating = 299;
    pdata->clksrc.read = pps_gmtimer_read_cycles;
    pdata->clksrc.mask = CLOCKSOURCE_MASK(32);
    pdata->clksrc.flags = CLOCK_SOURCE_IS_CONTINUOUS;

    clocksource_timer = pdata;
    if (clocksource_register_hz(&pdata->clksrc, pdata->frequency)) {
      pr_err("Could not register clocksource %s\n", pdata->clksrc.name);
      clocksource_timer = NULL;
    } else {
      pr_info("clocksource: %s at %u Hz\n", pdata->clksrc.name, pdata->frequency);
    }
  }
}

static void pps_gmtimer_clocksource_cleanup(struct pps_gmtimer_platform_data *pdata) {
  if(pdata == clocksource_timer) {
    clocksource_unregister(&pdata->clksrc);
    clocksource_timer = NULL;
  }
}

/* module ********************/
static struct pps_gmtimer_platform_data *of_get_pps_gmtimer_pdata(struct platform_device *pdev) {
  struct device_node *np = pdev->dev.of_node, *timer_dn;
  struct pps_gmtimer_platform_data *pdata;
  const __be32 *timer_phandle;

  pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
  if (!pdata)
    return NULL;

  pdata->ready = 0;

  timer_phandle = of_get_property(np, "timer", NULL);
  if(!timer_phandle) {
    pr_err("timer property in devicetree null\n");
    goto fail;
  }

  timer_dn = of_find_node_by_phandle(be32_to_cpup(timer_phandle));
  if(!timer_dn) {
    pr_err("find_node_by_phandle failed\n");
    goto fail;
  }

  if(pps_gmtimer_init_timer(timer_dn, pdata) < 0) {
    goto fail2;
  }

  of_node_put(timer_dn);

  return pdata;

fail2:
  of_node_put(timer_dn);
fail:
  devm_kfree(&pdev->dev, pdata);
  return NULL;
}

static const struct of_device_id pps_gmtimer_dt_ids[] = {
	{ .compatible = "pps-gmtimer", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, pps_gmtimer_dt_ids);

static int pps_gmtimer_probe(struct platform_device *pdev) {
  const struct of_device_id *match;
  struct pps_gmtimer_platform_data *pdata;
  struct pinctrl *pinctrl;
  const __be32 *use_tclkin;

  match = of_match_device(pps_gmtimer_dt_ids, &pdev->dev);
  if (match) {
    pdev->dev.platform_data = of_get_pps_gmtimer_pdata(pdev);
  } else {
    pr_err("of_match_device failed\n");
  }
  pdata = pdev->dev.platform_data;
  if(!pdata)
    return -ENODEV;

  pdata->ready = 0;

  if(sysfs_create_group(&pdev->dev.kobj, &attr_group)) {
    pr_err("sysfs_create_group failed\n");
  }

  pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
  if (IS_ERR(pinctrl))
    pr_warning("pins are not configured from the driver\n");

  use_tclkin = of_get_property(pdev->dev.of_node, "use-tclkin", NULL);
  if(use_tclkin && be32_to_cpup(use_tclkin) == 1) {
    omap_dm_timer_use_tclkin(pdata);
  } else {
    pr_info("using system clock\n");
  }

  pdata->info.mode = PPS_CAPTUREASSERT | PPS_ECHOASSERT | PPS_CANWAIT | PPS_TSFMT_TSPEC;
  pdata->info.owner = THIS_MODULE;
  snprintf(pdata->info.name, PPS_MAX_NAME_LEN - 1, "%s", pdata->timer_name);

  pdata->pps = pps_register_source(&pdata->info, PPS_CAPTUREASSERT);
  if (pdata->pps == NULL) {
    pr_err("failed to register %s as PPS source\n", pdata->timer_name);
  } else {
    // ready to go
    pdata->ready = 1;
    pps_gmtimer_clocksource_init(pdata);

    pps_gmtimer_enable_irq(pdata);
  }

  return 0;
}

static int pps_gmtimer_remove(struct platform_device *pdev) {
  struct pps_gmtimer_platform_data *pdata;
  pdata = pdev->dev.platform_data;

  if(pdata) {
    pps_gmtimer_clocksource_cleanup(pdata);

    pps_gmtimer_cleanup_timer(pdata);

    if(pdata->pps) {
      pps_unregister_source(pdata->pps);
      pdata->pps = NULL;
    }

    devm_kfree(&pdev->dev, pdata);
    pdev->dev.platform_data = NULL;

    sysfs_remove_group(&pdev->dev.kobj, &attr_group);
  }

  platform_set_drvdata(pdev, NULL);

  return 0;
}

static struct platform_driver pps_gmtimer_driver = {
	.probe		= pps_gmtimer_probe,
	.remove		= pps_gmtimer_remove,
	.driver		= {
		.name	= MODULE_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(pps_gmtimer_dt_ids),
	},
};

module_platform_driver(pps_gmtimer_driver);
