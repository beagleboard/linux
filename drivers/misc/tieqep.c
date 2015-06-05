/*
 * TI eQEP driver for AM33xx devices
 *
 * Copyright (C) 2013 Nathaniel R. Lewis - http://teknoman117.wordpress.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 * sysfs entries
 *   - position = absolute - current position; relative - last latched value
 *   - mode => 0 - absolute; 1 - relative
 *   - period => sampling period for the hardware
 *   - enable => 0 - eQEP disabled, 1 - eQEP enabled
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>
#include <linux/of_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/input.h>

// Include the PWMSS subsystem headers, because this unit controls
// whether our clock source is enabled, and if eQEP is to be
// enabled, we need to start the clock
#include "../pwm/pwm-tipwmss.h"

// eQEP register offsets from its base IO address
#define QPOSCNT    0x0000
#define QPOSINIT   0x0004
#define QPOSMAX    0x0008
#define QPOSCMP    0x000C
#define QPOSILAT   0x0010
#define QPOSSLAT   0x0014
#define QPOSLAT    0x0018
#define QUTMR      0x001C
#define QUPRD      0x0020
#define QWDTMR     0x0024
#define QWDPRD     0x0026
#define QDECCTL    0x0028
#define QEPCTL     0x002A
#define QCAPCTL    0x002C
#define QPOSCTL    0x002E
#define QEINT      0x0030
#define QFLG       0x0032
#define QCLR       0x0034
#define QFRC       0x0036
#define QEPSTS     0x0038
#define QCTMR      0x003A
#define QCPRD      0x003C
#define QCTMRLAT   0x003E
#define QCPRDLAT   0x0040
#define QREVID     0x005C

// Bits for the QDECTL register
#define QSRC1      (0x0001 << 15)
#define QSRC0      (0x0001 << 14)
#define SOEN       (0x0001 << 13)
#define SPSEL      (0x0001 << 12)
#define XCR        (0x0001 << 11)
#define SWAP       (0x0001 << 10)
#define IGATE      (0x0001 << 9)
#define QAP        (0x0001 << 8)
#define QBP        (0x0001 << 7)
#define QIP        (0x0001 << 6)
#define QSP        (0x0001 << 5)

// Bits for the QEPCTL register
#define FREESOFT1  (0x0001 << 15)
#define FREESOFT0  (0x0001 << 14)
#define PCRM1      (0x0001 << 13)
#define PCRM0      (0x0001 << 12)
#define SEI1       (0x0001 << 11)
#define SEI0       (0x0001 << 10)
#define IEI1       (0x0001 << 9)
#define IEI0       (0x0001 << 8)
#define SWI        (0x0001 << 7)
#define SEL        (0x0001 << 6)
#define IEL1       (0x0001 << 5)
#define IEL0       (0x0001 << 4)
#define PHEN       (0x0001 << 3)
#define QCLM       (0x0001 << 2)
#define UTE        (0x0001 << 1)
#define WDE        (0x0001 << 0)

// Bits for the QCAPCTL register
#define CEN        (0x0001 << 15)
#define CCPS2      (0x0001 << 6)
#define CCPS0      (0x0001 << 5)
#define CCPS1      (0x0001 << 4)
#define UPPS3      (0x0001 << 3)
#define UPPS2      (0x0001 << 2)
#define UPPS1      (0x0001 << 1)
#define UPPS0      (0x0001 << 0)

// Bits for the QPOSCTL register
#define PCSHDW     (0x0001 << 15)
#define PCLOAD     (0x0001 << 14)
#define PCPOL      (0x0001 << 13)
#define PCE        (0x0001 << 12)
#define PCSPW11    (0x0001 << 11)
#define PCSPW10    (0x0001 << 10)
#define PCSPW9    (0x0001 << 9)
#define PCSPW8    (0x0001 << 8)
#define PCSPW7    (0x0001 << 7)
#define PCSPW6    (0x0001 << 6)
#define PCSPW5    (0x0001 << 5)
#define PCSPW4    (0x0001 << 4)
#define PCSPW3    (0x0001 << 3)
#define PCSPW2    (0x0001 << 2)
#define PCSPW1    (0x0001 << 1)
#define PCSPW0    (0x0001 << 0)

// Bits for the interrupt registers
#define EQEP_INTERRUPT_MASK (0x0FFF)
#define UTOF                (0x0001 << 11)

// Bits to control the clock in the PWMSS subsystem
#define PWMSS_EQEPCLK_EN        BIT(4)
#define PWMSS_EQEPCLK_STOP_REQ  BIT(5)
#define PWMSS_EQEPCLK_EN_ACK    BIT(4)

// Modes for the eQEP unit
//  Absolute - the position entry represents the current position of the encoder.
//             Poll this value and it will be notified every period nanoseconds
//  Relative - the position entry represents the last latched position of the encoder
//             This value is latched every period nanoseconds and the internal counter
//             is subsequenty reset
#define TIEQEP_MODE_ABSOLUTE    0
#define TIEQEP_MODE_RELATIVE    1

// Structure defining the characteristics of the eQEP unit
struct eqep_chip
{
    // Platform device for this eQEP unit
    struct platform_device *pdev;

    // Pointer to the base of the memory of the eQEP unit
    void __iomem           *mmio_base;

    // SYSCLKOUT to the eQEP unit
    u32                     clk_rate;

    // IRQ for the eQEP unit
    u16                     irq;

    // Mode of the eQEP unit
    u8                      mode;

    // work stuct for the notify userspace work
    struct work_struct      notify_work;

    // Backup for driver suspension
    u16                     prior_qepctl;
    u16                     prior_qeint;
};

// Notify userspace work
void notify_handler (struct work_struct *work)
{
    // Get a reference to the eQEP driver
    struct eqep_chip *eqep = container_of(work, struct eqep_chip, notify_work);

    // Notify the userspace
    sysfs_notify(&eqep->pdev->dev.kobj, NULL, "position");
}

// eQEP Interrupt handler
static irqreturn_t eqep_irq_handler(int irq, void *dev_id)
{
    // Get the instance information
    struct platform_device *pdev = dev_id;
    struct eqep_chip       *eqep = platform_get_drvdata(pdev);

    // Get the interrupt flags
    u16 iflags = readw(eqep->mmio_base + QFLG) & EQEP_INTERRUPT_MASK;

    // Check the interrupt source(s)
    if(iflags & UTOF)
    {
        // Handle the unit timer overflow interrupt by notifying any potential pollers
        schedule_work(&eqep->notify_work);
    }

    // Clear interrupt flags (write back triggered flags to the clear register)
    writew(iflags, eqep->mmio_base + QCLR);

    // Return that the IRQ was handled successfully
    return IRQ_HANDLED;
}

// Function to read whether the eQEP unit is enabled or disabled
static ssize_t eqep_get_enabled(struct device *dev, struct device_attribute *attr, char *buf)
{
    // Get the instance structure
    struct eqep_chip *eqep = dev_get_drvdata(dev);

    // Read the qep control register and mask all but the enabled bit
    u16 enabled = readw(eqep->mmio_base + QEPCTL) & PHEN;

    // Return the target in string format
    return sprintf(buf, "%u\n", (enabled) ? 1 : 0);
}

// Function to set if the eQEP is enabled
static ssize_t eqep_set_enabled(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    // Get the instance structure
    int rc;
    u16 val;
    u8  enabled;
    struct eqep_chip *eqep = dev_get_drvdata(dev);

     // Convert the input string to an 8 bit uint
    if ((rc = kstrtou8(buf, 0, &enabled)))
        return rc;

    // Get the existing state of QEPCTL
    val = readw(eqep->mmio_base + QEPCTL);

    // If we passed a number that is not 0, enable the eQEP
    if(enabled)
    {
        // Enable the eQEP (Set PHEN in QEPCTL)
        val = val | PHEN;
    } else
    {
        // Disable the eQEP (Clear PHEN in QEPCTL)
        val = val & ~PHEN;
    }

    // Write flags back to control register
    writew(val, eqep->mmio_base + QEPCTL);

    // Return buffer length consumed (all)
    return count;
}

// Function to read the current position of the eQEP
static ssize_t eqep_get_position(struct device *dev, struct device_attribute *attr, char *buf)
{
    // Get the instance structure
    struct eqep_chip *eqep = dev_get_drvdata(dev);

    // A variable to return the position with
    s32 position = 0;

    // Check the mode
    if(eqep->mode == TIEQEP_MODE_ABSOLUTE)
    {
        // If we are in absolute mode, we need the current value of the eQEP hardware
        position = readl(eqep->mmio_base + QPOSCNT);
    } else if(eqep->mode == TIEQEP_MODE_RELATIVE)
    {
        // If we are in relative mode, we need the last latched value of the eQEP hardware
        position = readl(eqep->mmio_base + QPOSLAT);
    }

    // Return the target in string format
    return sprintf(buf, "%d\n", position);
}

// Function to set the position of the eQEP hardware
static ssize_t eqep_set_position(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    // Get the instance structure
    int rc;
    s32 position;
    struct eqep_chip *eqep = dev_get_drvdata(dev);

     // Convert the input string to an 8 bit uint
    if ((rc = kstrtos32(buf, 0, &position)))
        return rc;

    // If we are in absolute mode, set the position of the encoder, discard relative mode because thats pointless
    if(eqep->mode == TIEQEP_MODE_ABSOLUTE)
    {
        // If we are in absolute mode, we need the current value of the eQEP hardware
        writel(position, eqep->mmio_base + QPOSCNT);
    }

    // Return buffer length consumed (all)
    return count;
}

// Function to read the period of the unit time event timer
static ssize_t eqep_get_timer_period(struct device *dev, struct device_attribute *attr, char *buf)
{
    // Get the instance structure
    struct eqep_chip *eqep = dev_get_drvdata(dev);
    u64               period = 0;

    // Check if the period timer is enabled, if not, return 0
    if(!(readw(eqep->mmio_base + QEPCTL) & UTE))
    {
        return sprintf(buf, "0\n");
    }

    // Convert from counts per interrupt back into period_ns
    period = readl(eqep->mmio_base + QUPRD);
    period = period * NSEC_PER_SEC;
    do_div(period, eqep->clk_rate);

    // Otherwise write out the data
    return sprintf(buf, "%llu\n", period);
}

// Function to set the unit timer period.  0 = off, greater than zero sets the period
static ssize_t eqep_set_timer_period(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int rc;
    u16 tmp;
    u64 period;

    // Get the instance structure
    struct eqep_chip *eqep = dev_get_drvdata(dev);

    // Convert the passed string to a 64 bit uint
    if((rc = kstrtou64(buf, 0, &period)))
        return rc;

    // Disable the unit timer before modifying its period register
    tmp = readw(eqep->mmio_base + QEPCTL);
    tmp = tmp & ~UTE & ~QCLM;
    writew(tmp, eqep->mmio_base + QEPCTL);

    // Zero the unit timer counter register
    writel(0x0, eqep->mmio_base + QUTMR);

    // If we want the timer enabled (a period that is non zero has been passed)
    if(period)
    {
        // Otherwise calculate the period
        period = period * eqep->clk_rate;
        do_div(period, NSEC_PER_SEC);

        // Set this period into the unit timer period register
        writel(period & 0x00000000FFFFFFFF, eqep->mmio_base + QUPRD);

        // Enable the unit timer, and latch QPOSLAT to QPOSCNT on overflow
        tmp = readw(eqep->mmio_base + QEPCTL);
        tmp = tmp | UTE | QCLM;
        writew(tmp, eqep->mmio_base + QEPCTL);
    }

    // Return consumed buffer count
    return count;
}

// Function to read the mode of the eQEP hardware
static ssize_t eqep_get_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
    // Get the instance structure
    struct eqep_chip *eqep = dev_get_drvdata(dev);

    // Return the mode
    return sprintf(buf, "%u\n", eqep->mode);
}

// Function to set the mode of the eQEP hardware
static ssize_t eqep_set_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    // Get the instance structure
    int rc;
    u16 val;
    u8  tmp_mode;
    struct eqep_chip *eqep = dev_get_drvdata(dev);

     // Convert the input string to an 8 bit uint
    if ((rc = kstrtou8(buf, 0, &tmp_mode)))
        return rc;

    // Get the existing state of QEPCTL
    val = readw(eqep->mmio_base + QEPCTL);

    // Check the mode that was passed
    if(tmp_mode == TIEQEP_MODE_ABSOLUTE)
    {
        // In absolute mode, we don't want to reset the intenal hardware based on time,
        // so disable the unit timer position reset (Set PCRM[1:0] = 0)
        val = val & ~PCRM1 & ~PCRM0;

        // Store the mode as absolute
        eqep->mode = TIEQEP_MODE_ABSOLUTE;
    } else if(tmp_mode == TIEQEP_MODE_RELATIVE)
    {
        // In relative mode, we want to latch the value of the eQEP hardware on the
        // overflow of the unit timer.  So enable the unit timer position reset
        // (Set PCRM[1:0] = 3)
        val = val | PCRM1 | PCRM0;

        // Store the mode as relative
        eqep->mode = TIEQEP_MODE_RELATIVE;
    }

    // Write the new value back to the control register
    writew(val, eqep->mmio_base + QEPCTL);

    // Return buffer length consumed (all)
    return count;
}

// Bind read/write functions to sysfs entries
static DEVICE_ATTR(enabled,  0644, eqep_get_enabled,      eqep_set_enabled);
static DEVICE_ATTR(position, 0644, eqep_get_position,     eqep_set_position);
static DEVICE_ATTR(period,   0644, eqep_get_timer_period, eqep_set_timer_period);
static DEVICE_ATTR(mode,     0644, eqep_get_mode,         eqep_set_mode);

// Array holding all of the sysfs entries
static const struct attribute *eqep_attrs[] = {
    &dev_attr_enabled.attr,
    &dev_attr_position.attr,
    &dev_attr_period.attr,
    &dev_attr_mode.attr,
    NULL,
};

// Driver function group
static const struct attribute_group eqep_device_attr_group = {
    .attrs = (struct attribute **) eqep_attrs,
};

// Driver compatibility list
static struct of_device_id eqep_of_match[] =
{
    { .compatible = "ti,am33xx-eqep" },
    { }
};

// Register our compatibilities for device trees
MODULE_DEVICE_TABLE(of, eqep_of_match);

// Create an instance of the eQEP driver
static int eqep_probe(struct platform_device *pdev)
{
    struct resource  *r;
    struct clk       *clk;
    struct eqep_chip *eqep;
    struct pinctrl   *pinctrl;
    int               ret;
    u64               period;
    u16               status;
    u32               value;

    // Select any default pins possible provided through the device tree
    pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
    if (IS_ERR(pinctrl))
    {
        dev_warn(&pdev->dev, "unable to select pin group\n");
    }

    // Allocate a eqep_driver object
    eqep = devm_kzalloc(&pdev->dev, sizeof(struct eqep_chip), GFP_KERNEL);
    if (!eqep) {
        dev_err(&pdev->dev, "failed to allocate memory\n");
        return -ENOMEM;
    }

    // Get a handle to the system clock object
    clk = devm_clk_get(&pdev->dev, "fck");
    if (IS_ERR(clk)) {
        dev_err(&pdev->dev, "failed to get clock\n");
        return PTR_ERR(clk);
    }

    // Get the frequency of the system clock
    eqep->clk_rate = clk_get_rate(clk);
    if (!eqep->clk_rate) {
        dev_err(&pdev->dev, "failed to get clock rate\n");
        return -EINVAL;
    }

    // Get a resource containing the IRQ for this eQEP controller
    r = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (unlikely(!r)) {
        dev_err(&pdev->dev, "Invalid IRQ resource\n");
        return -ENODEV;
    }

    // Store the irq
    eqep->irq = r->start;

    // Get a resource containing the requested (from DT) memory address and range of eQEP controller
    r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!r) {
        dev_err(&pdev->dev, "no memory resource defined\n");
        return -ENODEV;
    }

    // Remap the eQEP controller memory into our own memory space
    eqep->mmio_base = devm_ioremap_resource(&pdev->dev, r);
    if (IS_ERR(eqep->mmio_base))
        return PTR_ERR(eqep->mmio_base);

    // Store the platform device in our eQEP data structure for later usage
    eqep->pdev = pdev;

    // Subscribe to the eQEP interrupt
    if (request_irq(eqep->irq, eqep_irq_handler, IRQF_IRQPOLL, "eqep_interrupt", pdev))
    {
        dev_err(&pdev->dev, "unable to request irq for eQEP\n");
        return -ENODEV;
    }

    // Register controls to sysfs
    if (sysfs_create_group(&pdev->dev.kobj, &eqep_device_attr_group))
    {
        dev_err(&pdev->dev, "sysfs creation failed\n");
        return -EINVAL;
    }

    // Read decoder control settings
    status = readw(eqep->mmio_base + QDECCTL);

    // Quadrature mode or direction mode
    if(of_property_read_u32(pdev->dev.of_node, "count_mode", &value))
        status = status & ~QSRC1 & ~QSRC0;
    else
        status = ((value) ? status | QSRC0 : status & ~QSRC0) & ~QSRC1;

    // Should we invert the qa input
    if(of_property_read_u32(pdev->dev.of_node, "invert_qa", &value))
        status = status & ~QAP;
    else
        status = (value) ? status | QAP : status & ~QAP;

    // Should we invert the qb input
    if(of_property_read_u32(pdev->dev.of_node, "invert_qb", &value))
        status = status & ~QBP;
    else
        status = (value) ? status | QBP : status & ~QBP;

    // Should we invert the index input
    if(of_property_read_u32(pdev->dev.of_node, "invert_qi", &value))
        status = status & ~QIP;
    else
        status = (value) ? status | QIP : status & ~QIP;

    // Should we invert the strobe input
    if(of_property_read_u32(pdev->dev.of_node, "invert_qs", &value))
        status = status & ~QSP;
    else
        status = (value) ? status | QSP : status & ~QSP;

    // Should we swap the cha and chb inputs
    if(of_property_read_u32(pdev->dev.of_node, "swap_inputs", &value))
        status = status & ~SWAP;
    else
        status = (value) ? status | SWAP : status & ~SWAP;

    // Write the decoder control settings back to the control register
    writew(status, eqep->mmio_base + QDECCTL);

    // Initialize the position counter to zero
    writel(0, eqep->mmio_base + QPOSINIT);

    // This is pretty ingenious if I do say so myself.  The eQEP subsystem has a register
    // that defined the maximum value of the encoder as an unsigned.  If the counter is zero
    // and decrements again, it is set to QPOSMAX.  If you cast -1 (signed int) to
    // to an unsigned, you will notice that -1 == UINT_MAX.  So when the counter is set to this
    // maximum position, when read into a signed int, it will equal -1.  Two's complement for
    // the WIN!!
    writel(-1, eqep->mmio_base + QPOSMAX);

    // Enable some interrupts
    status = readw(eqep->mmio_base + QEINT);
    status = status | UTOF;
            // UTOF - Unit Time Period interrupt.  This is triggered when the unit timer period expires
            //
    writew(status, eqep->mmio_base + QEINT);

    // Calculate the timer ticks per second
    period = 1000000000;
    period = period * eqep->clk_rate;
    do_div(period, NSEC_PER_SEC);

    // Set this period into the unit timer period register
    writel(period & 0x00000000FFFFFFFF, eqep->mmio_base + QUPRD);

    // Enable the eQEP with basic position counting turned on
    status = readw(eqep->mmio_base + QEPCTL);
    status = status | PHEN | IEL0 | SWI | UTE | QCLM;
            // PHEN - Quadrature position counter enable bit
            // IEL0 - Latch QPOSILAT on index signal.  This can be rising or falling, IEL[1:0] = 0 is reserved
            // SWI  - Software initialization of position count register.  Basic set QPOSCNT <= QPOSINIT
            // UTE  - unit timer enable
            // QCLM - latch QPOSLAT to QPOSCNT upon unit timer overflow
    writew(status, eqep->mmio_base + QEPCTL);

    // We default to absolute mode
    eqep->mode = TIEQEP_MODE_ABSOLUTE;

    // Enable the power management runtime
    pm_runtime_enable(&pdev->dev);

    // Increment the device usage count and run pm_runtime_resume()
    pm_runtime_get_sync(&pdev->dev);

    // Enable the clock to the eQEP unit
    status = pwmss_submodule_state_change(pdev->dev.parent, PWMSS_EQEPCLK_EN);

    // If we failed to enable the clocks, fail out
    if (!(status & PWMSS_EQEPCLK_EN_ACK))
    {
        dev_err(&pdev->dev, "PWMSS config space clock enable failed\n");
        ret = -EINVAL;
        goto pwmss_clk_failure;
    }

    // Initialize the notify work struture
    INIT_WORK(&eqep->notify_work, notify_handler);

    // Decrement the device usage count (twice) and run pm_runtime_idle() if zero
    pm_runtime_put_sync(&pdev->dev);

    // Set the platform driver data to the data object we've been creating for the eQEP unit
    platform_set_drvdata(pdev, eqep);

    // Success!
    //printk(KERN_INFO "EQEP irq = %d, system clock = %u\n", eqep->irq, eqep->clk_rate);
    return 0;

    // If a failure occurred, stop the runtime power management
pwmss_clk_failure:
    pm_runtime_put_sync(&pdev->dev);
    pm_runtime_disable(&pdev->dev);
    return ret;
}

// Remove an instance of the eQEP driver
static int eqep_remove(struct platform_device *pdev)
{
    // Get the eQEP driver data from the platform device structure
    struct eqep_chip *eqep = platform_get_drvdata(pdev);

    // Cancel work
    cancel_work_sync(&eqep->notify_work);

    // Unmap from sysfs
    sysfs_remove_group(&pdev->dev.kobj, &eqep_device_attr_group);

    // Release important assets
    free_irq(eqep->irq, pdev);

    // Increment the device usage count and run pm_runtime_resume()
    pm_runtime_get_sync(&pdev->dev);

    // Disable the eQEP clock
    pwmss_submodule_state_change(pdev->dev.parent, PWMSS_EQEPCLK_STOP_REQ);

    // Decrement the device usage count (twice) and run pm_runtime_idle() if zero
    pm_runtime_put_sync(&pdev->dev);
    pm_runtime_put_sync(&pdev->dev);

    // Disable the runtime power management of this device
    pm_runtime_disable(&pdev->dev);

    // Return success
    return 0;
}

// Power management suspend device
static int eqep_suspend(struct device *dev)
{
    // Get the eqep driver information
    struct eqep_chip   *eqep = dev_get_drvdata(dev);
    u16                 tmp;

    // Shut down interrupts
    eqep->prior_qeint = readw(eqep->mmio_base + QEINT);
    tmp = eqep->prior_qeint & ~UTOF;
            // UTOF - Unit Time Period interrupt.
    writew(tmp, eqep->mmio_base + QEINT);

    // Get the existing state of QEPCTL
    eqep->prior_qepctl = readw(eqep->mmio_base + QEPCTL);

    // Disable eQEP controller
    writew(eqep->prior_qepctl & ~PHEN, eqep->mmio_base + QEPCTL);

    // Decrement the device usage count and run pm_runtime_idle() if zero
    pm_runtime_put_sync(dev);

    // Return success
    return 0;
}

// Power management wake device back up
static int eqep_resume(struct device *dev)
{
    // Get the eqep driver information
    struct eqep_chip *eqep = dev_get_drvdata(dev);

    // Restore interrupt enabled register
    writew(eqep->prior_qeint, eqep->mmio_base + QEINT);

    // Restore prior qep control register
    writew(eqep->prior_qepctl, eqep->mmio_base + QEPCTL);

    // Increment the device usage count and run pm_runtime_resume()
    pm_runtime_get_sync(dev);

    // Success
    return 0;
}

// create pm functions object
static SIMPLE_DEV_PM_OPS(eqep_pm_ops, eqep_suspend, eqep_resume);

// Platform driver information
static struct platform_driver eqep_driver = {
    .driver = {
        .name           = "eqep",
        .owner          = THIS_MODULE,
        .pm             = &eqep_pm_ops,
        .of_match_table = eqep_of_match,
    },
    .probe = eqep_probe,
    .remove = eqep_remove,
};

// Register this platform driver
module_platform_driver(eqep_driver);

// Module information
MODULE_DESCRIPTION("TI eQEP driver");
MODULE_AUTHOR("Nathaniel R. Lewis");
MODULE_LICENSE("GPL");
