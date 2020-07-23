/* Framework for configuring and reading PHY devices
 * Based on code in sungem_phy.c and gianfar_phy.c
 *
 * Author: Andy Fleming
 *
 * Copyright (c) 2004 Freescale Semiconductor, Inc.
 * Copyright (c) 2006, 2007  Maciej W. Rozycki
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/phy_led_triggers.h>
#include <linux/workqueue.h>
#include <linux/mdio.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/atomic.h>

#include <asm/irq.h>

#define PHY_STATE_STR(_state)			\
	case PHY_##_state:			\
		return __stringify(_state);	\

static const char *phy_state_to_str(enum phy_state st)
{
	switch (st) {
	PHY_STATE_STR(DOWN)
	PHY_STATE_STR(STARTING)
	PHY_STATE_STR(READY)
	PHY_STATE_STR(PENDING)
	PHY_STATE_STR(UP)
	PHY_STATE_STR(AN)
	PHY_STATE_STR(RUNNING)
	PHY_STATE_STR(NOLINK)
	PHY_STATE_STR(FORCING)
	PHY_STATE_STR(CHANGELINK)
	PHY_STATE_STR(HALTED)
	PHY_STATE_STR(RESUMING)
	}

	return NULL;
}


/**
 * phy_print_status - Convenience function to print out the current phy status
 * @phydev: the phy_device struct
 */
void phy_print_status(struct phy_device *phydev)
{
	if (phydev->link) {
		netdev_info(phydev->attached_dev,
			"Link is Up - %s/%s - flow control %s\n",
			phy_speed_to_str(phydev->speed),
			phy_duplex_to_str(phydev->duplex),
			phydev->pause ? "rx/tx" : "off");
	} else	{
		netdev_info(phydev->attached_dev, "Link is Down\n");
	}
}
EXPORT_SYMBOL(phy_print_status);

/**
 * phy_clear_interrupt - Ack the phy device's interrupt
 * @phydev: the phy_device struct
 *
 * If the @phydev driver has an ack_interrupt function, call it to
 * ack and clear the phy device's interrupt.
 *
 * Returns 0 on success or < 0 on error.
 */
static int phy_clear_interrupt(struct phy_device *phydev)
{
	if (phydev->drv->ack_interrupt)
		return phydev->drv->ack_interrupt(phydev);

	return 0;
}

/**
 * phy_config_interrupt - configure the PHY device for the requested interrupts
 * @phydev: the phy_device struct
 * @interrupts: interrupt flags to configure for this @phydev
 *
 * Returns 0 on success or < 0 on error.
 */
static int phy_config_interrupt(struct phy_device *phydev, u32 interrupts)
{
	phydev->interrupts = interrupts;
	if (phydev->drv->config_intr)
		return phydev->drv->config_intr(phydev);

	return 0;
}

/**
 * phy_restart_aneg - restart auto-negotiation
 * @phydev: target phy_device struct
 *
 * Restart the autonegotiation on @phydev.  Returns >= 0 on success or
 * negative errno on error.
 */
int phy_restart_aneg(struct phy_device *phydev)
{
	int ret;

	if (phydev->is_c45 && !(phydev->c45_ids.devices_in_package & BIT(0)))
		ret = genphy_c45_restart_aneg(phydev);
	else
		ret = genphy_restart_aneg(phydev);

	return ret;
}
EXPORT_SYMBOL_GPL(phy_restart_aneg);

/**
 * phy_aneg_done - return auto-negotiation status
 * @phydev: target phy_device struct
 *
 * Description: Return the auto-negotiation status from this @phydev
 * Returns > 0 on success or < 0 on error. 0 means that auto-negotiation
 * is still pending.
 */
int phy_aneg_done(struct phy_device *phydev)
{
	if (phydev->drv && phydev->drv->aneg_done)
		return phydev->drv->aneg_done(phydev);

	/* Avoid genphy_aneg_done() if the Clause 45 PHY does not
	 * implement Clause 22 registers
	 */
	if (phydev->is_c45 && !(phydev->c45_ids.devices_in_package & BIT(0)))
		return -EINVAL;

	return genphy_aneg_done(phydev);
}
EXPORT_SYMBOL(phy_aneg_done);

/**
 * phy_find_valid - find a PHY setting that matches the requested parameters
 * @speed: desired speed
 * @duplex: desired duplex
 * @supported: mask of supported link modes
 *
 * Locate a supported phy setting that is, in priority order:
 * - an exact match for the specified speed and duplex mode
 * - a match for the specified speed, or slower speed
 * - the slowest supported speed
 * Returns the matched phy_setting entry, or %NULL if no supported phy
 * settings were found.
 */
static const struct phy_setting *
phy_find_valid(int speed, int duplex, u32 supported)
{
	unsigned long mask = supported;

	return phy_lookup_setting(speed, duplex, &mask, BITS_PER_LONG, false);
}

/**
 * phy_supported_speeds - return all speeds currently supported by a phy device
 * @phy: The phy device to return supported speeds of.
 * @speeds: buffer to store supported speeds in.
 * @size:   size of speeds buffer.
 *
 * Description: Returns the number of supported speeds, and fills the speeds
 * buffer with the supported speeds. If speeds buffer is too small to contain
 * all currently supported speeds, will return as many speeds as can fit.
 */
unsigned int phy_supported_speeds(struct phy_device *phy,
				  unsigned int *speeds,
				  unsigned int size)
{
	unsigned long supported = phy->supported;

	return phy_speeds(speeds, size, &supported, BITS_PER_LONG);
}

/**
 * phy_check_valid - check if there is a valid PHY setting which matches
 *		     speed, duplex, and feature mask
 * @speed: speed to match
 * @duplex: duplex to match
 * @features: A mask of the valid settings
 *
 * Description: Returns true if there is a valid setting, false otherwise.
 */
static inline bool phy_check_valid(int speed, int duplex, u32 features)
{
	unsigned long mask = features;

	return !!phy_lookup_setting(speed, duplex, &mask, BITS_PER_LONG, true);
}

/**
 * phy_sanitize_settings - make sure the PHY is set to supported speed and duplex
 * @phydev: the target phy_device struct
 *
 * Description: Make sure the PHY is set to supported speeds and
 *   duplexes.  Drop down by one in this order:  1000/FULL,
 *   1000/HALF, 100/FULL, 100/HALF, 10/FULL, 10/HALF.
 */
static void phy_sanitize_settings(struct phy_device *phydev)
{
	const struct phy_setting *setting;
	u32 features = phydev->supported;

	/* Sanitize settings based on PHY capabilities */
	if ((features & SUPPORTED_Autoneg) == 0)
		phydev->autoneg = AUTONEG_DISABLE;

	setting = phy_find_valid(phydev->speed, phydev->duplex, features);
	if (setting) {
		phydev->speed = setting->speed;
		phydev->duplex = setting->duplex;
	} else {
		/* We failed to find anything (no supported speeds?) */
		phydev->speed = SPEED_UNKNOWN;
		phydev->duplex = DUPLEX_UNKNOWN;
	}
}

/**
 * phy_ethtool_sset - generic ethtool sset function, handles all the details
 * @phydev: target phy_device struct
 * @cmd: ethtool_cmd
 *
 * A few notes about parameter checking:
 *
 * - We don't set port or transceiver, so we don't care what they
 *   were set to.
 * - phy_start_aneg() will make sure forced settings are sane, and
 *   choose the next best ones from the ones selected, so we don't
 *   care if ethtool tries to give us bad values.
 */
int phy_ethtool_sset(struct phy_device *phydev, struct ethtool_cmd *cmd)
{
	u32 speed = ethtool_cmd_speed(cmd);

	if (cmd->phy_address != phydev->mdio.addr)
		return -EINVAL;

	/* We make sure that we don't pass unsupported values in to the PHY */
	cmd->advertising &= phydev->supported;

	/* Verify the settings we care about. */
	if (cmd->autoneg != AUTONEG_ENABLE && cmd->autoneg != AUTONEG_DISABLE)
		return -EINVAL;

	if (cmd->autoneg == AUTONEG_ENABLE && cmd->advertising == 0)
		return -EINVAL;

	if (cmd->autoneg == AUTONEG_DISABLE &&
	    ((speed != SPEED_1000 &&
	      speed != SPEED_100 &&
	      speed != SPEED_10) ||
	     (cmd->duplex != DUPLEX_HALF &&
	      cmd->duplex != DUPLEX_FULL)))
		return -EINVAL;

	phydev->autoneg = cmd->autoneg;

	phydev->speed = speed;

	phydev->advertising = cmd->advertising;

	if (AUTONEG_ENABLE == cmd->autoneg)
		phydev->advertising |= ADVERTISED_Autoneg;
	else
		phydev->advertising &= ~ADVERTISED_Autoneg;

	phydev->duplex = cmd->duplex;

	phydev->mdix_ctrl = cmd->eth_tp_mdix_ctrl;

	/* Restart the PHY */
	phy_start_aneg(phydev);

	return 0;
}
EXPORT_SYMBOL(phy_ethtool_sset);

int phy_ethtool_ksettings_set(struct phy_device *phydev,
			      const struct ethtool_link_ksettings *cmd)
{
	u8 autoneg = cmd->base.autoneg;
	u8 duplex = cmd->base.duplex;
	u32 speed = cmd->base.speed;
	u32 advertising;

	if (cmd->base.phy_address != phydev->mdio.addr)
		return -EINVAL;

	ethtool_convert_link_mode_to_legacy_u32(&advertising,
						cmd->link_modes.advertising);

	/* We make sure that we don't pass unsupported values in to the PHY */
	advertising &= phydev->supported;

	/* Verify the settings we care about. */
	if (autoneg != AUTONEG_ENABLE && autoneg != AUTONEG_DISABLE)
		return -EINVAL;

	if (autoneg == AUTONEG_ENABLE && advertising == 0)
		return -EINVAL;

	if (autoneg == AUTONEG_DISABLE &&
	    ((speed != SPEED_1000 &&
	      speed != SPEED_100 &&
	      speed != SPEED_10) ||
	     (duplex != DUPLEX_HALF &&
	      duplex != DUPLEX_FULL)))
		return -EINVAL;

	phydev->autoneg = autoneg;

	phydev->speed = speed;

	phydev->advertising = advertising;

	if (autoneg == AUTONEG_ENABLE)
		phydev->advertising |= ADVERTISED_Autoneg;
	else
		phydev->advertising &= ~ADVERTISED_Autoneg;

	phydev->duplex = duplex;

	phydev->mdix_ctrl = cmd->base.eth_tp_mdix_ctrl;

	/* Restart the PHY */
	phy_start_aneg(phydev);

	return 0;
}
EXPORT_SYMBOL(phy_ethtool_ksettings_set);

void phy_ethtool_ksettings_get(struct phy_device *phydev,
			       struct ethtool_link_ksettings *cmd)
{
	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
						phydev->supported);

	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.advertising,
						phydev->advertising);

	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.lp_advertising,
						phydev->lp_advertising);

	cmd->base.speed = phydev->speed;
	cmd->base.duplex = phydev->duplex;
	if (phydev->interface == PHY_INTERFACE_MODE_MOCA)
		cmd->base.port = PORT_BNC;
	else
		cmd->base.port = PORT_MII;
	cmd->base.transceiver = phy_is_internal(phydev) ?
				XCVR_INTERNAL : XCVR_EXTERNAL;
	cmd->base.phy_address = phydev->mdio.addr;
	cmd->base.autoneg = phydev->autoneg;
	cmd->base.eth_tp_mdix_ctrl = phydev->mdix_ctrl;
	cmd->base.eth_tp_mdix = phydev->mdix;
}
EXPORT_SYMBOL(phy_ethtool_ksettings_get);

/**
 * phy_mii_ioctl - generic PHY MII ioctl interface
 * @phydev: the phy_device struct
 * @ifr: &struct ifreq for socket ioctl's
 * @cmd: ioctl cmd to execute
 *
 * Note that this function is currently incompatible with the
 * PHYCONTROL layer.  It changes registers without regard to
 * current state.  Use at own risk.
 */
int phy_mii_ioctl(struct phy_device *phydev, struct ifreq *ifr, int cmd)
{
	struct mii_ioctl_data *mii_data = if_mii(ifr);
	u16 val = mii_data->val_in;
	bool change_autoneg = false;
	int ret;

	switch (cmd) {
	case SIOCGMIIPHY:
		mii_data->phy_id = phydev->mdio.addr;
		/* fall through */

	case SIOCGMIIREG:
		if (mdio_phy_id_is_c45(mii_data->phy_id)) {
			ret = phy_read_mmd(phydev,
					   mdio_phy_id_devad(mii_data->phy_id),
					   mii_data->reg_num);
			if (ret < 0)
				return ret;
			mii_data->val_out = ret;
			return 0;
		}

		mii_data->val_out = mdiobus_read(phydev->mdio.bus,
						 mii_data->phy_id,
						 mii_data->reg_num);
		return 0;

	case SIOCSMIIREG:
		if (mdio_phy_id_is_c45(mii_data->phy_id)) {
			ret = phy_write_mmd(phydev,
					    mdio_phy_id_devad(mii_data->phy_id),
					    mii_data->reg_num,
					    mii_data->val_in);
			return ret;
		}
		if (mii_data->phy_id == phydev->mdio.addr) {
			switch (mii_data->reg_num) {
			case MII_BMCR:
				if ((val & (BMCR_RESET | BMCR_ANENABLE)) == 0) {
					if (phydev->autoneg == AUTONEG_ENABLE)
						change_autoneg = true;
					phydev->autoneg = AUTONEG_DISABLE;
					if (val & BMCR_FULLDPLX)
						phydev->duplex = DUPLEX_FULL;
					else
						phydev->duplex = DUPLEX_HALF;
					if (val & BMCR_SPEED1000)
						phydev->speed = SPEED_1000;
					else if (val & BMCR_SPEED100)
						phydev->speed = SPEED_100;
					else phydev->speed = SPEED_10;
				}
				else {
					if (phydev->autoneg == AUTONEG_DISABLE)
						change_autoneg = true;
					phydev->autoneg = AUTONEG_ENABLE;
				}
				break;
			case MII_ADVERTISE:
				phydev->advertising = mii_adv_to_ethtool_adv_t(val);
				change_autoneg = true;
				break;
			default:
				/* do nothing */
				break;
			}
		}

		mdiobus_write(phydev->mdio.bus, mii_data->phy_id,
			      mii_data->reg_num, val);

		if (mii_data->phy_id == phydev->mdio.addr &&
		    mii_data->reg_num == MII_BMCR &&
		    val & BMCR_RESET)
			return phy_init_hw(phydev);

		if (change_autoneg)
			return phy_start_aneg(phydev);

		return 0;

	case SIOCSHWTSTAMP:
		if (phydev->drv && phydev->drv->hwtstamp)
			return phydev->drv->hwtstamp(phydev, ifr);
		/* fall through */

	default:
		return -EOPNOTSUPP;
	}
}
EXPORT_SYMBOL(phy_mii_ioctl);

/**
 * phy_start_aneg_priv - start auto-negotiation for this PHY device
 * @phydev: the phy_device struct
 * @sync: indicate whether we should wait for the workqueue cancelation
 *
 * Description: Sanitizes the settings (if we're not autonegotiating
 *   them), and then calls the driver's config_aneg function.
 *   If the PHYCONTROL Layer is operating, we change the state to
 *   reflect the beginning of Auto-negotiation or forcing.
 */
static int phy_start_aneg_priv(struct phy_device *phydev, bool sync)
{
	bool trigger = 0;
	int err;

	if (!phydev->drv)
		return -EIO;

	mutex_lock(&phydev->lock);

	if (AUTONEG_DISABLE == phydev->autoneg)
		phy_sanitize_settings(phydev);

	/* Invalidate LP advertising flags */
	phydev->lp_advertising = 0;

	err = phydev->drv->config_aneg(phydev);
	if (err < 0)
		goto out_unlock;

	if (phydev->state != PHY_HALTED) {
		if (AUTONEG_ENABLE == phydev->autoneg) {
			phydev->state = PHY_AN;
			phydev->link_timeout = PHY_AN_TIMEOUT;
		} else {
			phydev->state = PHY_FORCING;
			phydev->link_timeout = PHY_FORCE_TIMEOUT;
		}
	}

	/* Re-schedule a PHY state machine to check PHY status because
	 * negotiation may already be done and aneg interrupt may not be
	 * generated.
	 */
	if (phydev->irq != PHY_POLL && phydev->state == PHY_AN) {
		err = phy_aneg_done(phydev);
		if (err > 0) {
			trigger = true;
			err = 0;
		}
	}

out_unlock:
	mutex_unlock(&phydev->lock);

	if (trigger)
		phy_trigger_machine(phydev, sync);

	return err;
}

/**
 * phy_start_aneg - start auto-negotiation for this PHY device
 * @phydev: the phy_device struct
 *
 * Description: Sanitizes the settings (if we're not autonegotiating
 *   them), and then calls the driver's config_aneg function.
 *   If the PHYCONTROL Layer is operating, we change the state to
 *   reflect the beginning of Auto-negotiation or forcing.
 */
int phy_start_aneg(struct phy_device *phydev)
{
	return phy_start_aneg_priv(phydev, true);
}
EXPORT_SYMBOL(phy_start_aneg);

/**
 * phy_start_machine - start PHY state machine tracking
 * @phydev: the phy_device struct
 *
 * Description: The PHY infrastructure can run a state machine
 *   which tracks whether the PHY is starting up, negotiating,
 *   etc.  This function starts the delayed workqueue which tracks
 *   the state of the PHY. If you want to maintain your own state machine,
 *   do not call this function.
 */
void phy_start_machine(struct phy_device *phydev)
{
	queue_delayed_work(system_power_efficient_wq, &phydev->state_queue, HZ);
}
EXPORT_SYMBOL_GPL(phy_start_machine);

/**
 * phy_trigger_machine - trigger the state machine to run
 *
 * @phydev: the phy_device struct
 * @sync: indicate whether we should wait for the workqueue cancelation
 *
 * Description: There has been a change in state which requires that the
 *   state machine runs.
 */

void phy_trigger_machine(struct phy_device *phydev, bool sync)
{
	if (sync)
		cancel_delayed_work_sync(&phydev->state_queue);
	else
		cancel_delayed_work(&phydev->state_queue);
	queue_delayed_work(system_power_efficient_wq, &phydev->state_queue, 0);
}

/**
 * phy_stop_machine - stop the PHY state machine tracking
 * @phydev: target phy_device struct
 *
 * Description: Stops the state machine delayed workqueue, sets the
 *   state to UP (unless it wasn't up yet). This function must be
 *   called BEFORE phy_detach.
 */
void phy_stop_machine(struct phy_device *phydev)
{
	cancel_delayed_work_sync(&phydev->state_queue);

	mutex_lock(&phydev->lock);
	if (phydev->state > PHY_UP && phydev->state != PHY_HALTED)
		phydev->state = PHY_UP;
	mutex_unlock(&phydev->lock);
}

/**
 * phy_error - enter HALTED state for this PHY device
 * @phydev: target phy_device struct
 *
 * Moves the PHY to the HALTED state in response to a read
 * or write error, and tells the controller the link is down.
 * Must not be called from interrupt context, or while the
 * phydev->lock is held.
 */
static void phy_error(struct phy_device *phydev)
{
	mutex_lock(&phydev->lock);
	phydev->state = PHY_HALTED;
	mutex_unlock(&phydev->lock);

	phy_trigger_machine(phydev, false);
}

/**
 * phy_disable_interrupts - Disable the PHY interrupts from the PHY side
 * @phydev: target phy_device struct
 */
static int phy_disable_interrupts(struct phy_device *phydev)
{
	int err;

	/* Disable PHY interrupts */
	err = phy_config_interrupt(phydev, PHY_INTERRUPT_DISABLED);
	if (err)
		goto phy_err;

	/* Clear the interrupt */
	err = phy_clear_interrupt(phydev);
	if (err)
		goto phy_err;

	return 0;

phy_err:
	phy_error(phydev);

	return err;
}

/**
 * phy_change - Called by the phy_interrupt to handle PHY changes
 * @phydev: phy_device struct that interrupted
 */
static irqreturn_t phy_change(struct phy_device *phydev)
{
	if (phy_interrupt_is_valid(phydev)) {
		if (phydev->drv->did_interrupt &&
		    !phydev->drv->did_interrupt(phydev))
			goto ignore;

		if (phy_disable_interrupts(phydev))
			goto phy_err;
	}

	mutex_lock(&phydev->lock);
	if ((PHY_RUNNING == phydev->state) || (PHY_NOLINK == phydev->state))
		phydev->state = PHY_CHANGELINK;
	mutex_unlock(&phydev->lock);

	if (phy_interrupt_is_valid(phydev)) {
		atomic_dec(&phydev->irq_disable);
		enable_irq(phydev->irq);

		/* Reenable interrupts */
		if (PHY_HALTED != phydev->state &&
		    phy_config_interrupt(phydev, PHY_INTERRUPT_ENABLED))
			goto irq_enable_err;
	}

	/* reschedule state queue work to run as soon as possible */
	phy_trigger_machine(phydev, true);
	return IRQ_HANDLED;

ignore:
	atomic_dec(&phydev->irq_disable);
	enable_irq(phydev->irq);
	return IRQ_NONE;

irq_enable_err:
	disable_irq(phydev->irq);
	atomic_inc(&phydev->irq_disable);
phy_err:
	phy_error(phydev);
	return IRQ_NONE;
}

/**
 * phy_change_work - Scheduled by the phy_mac_interrupt to handle PHY changes
 * @work: work_struct that describes the work to be done
 */
void phy_change_work(struct work_struct *work)
{
	struct phy_device *phydev =
		container_of(work, struct phy_device, phy_queue);

	phy_change(phydev);
}

/**
 * phy_interrupt - PHY interrupt handler
 * @irq: interrupt line
 * @phy_dat: phy_device pointer
 *
 * Description: When a PHY interrupt occurs, the handler disables
 * interrupts, and uses phy_change to handle the interrupt.
 */
static irqreturn_t phy_interrupt(int irq, void *phy_dat)
{
	struct phy_device *phydev = phy_dat;

	if (PHY_HALTED == phydev->state)
		return IRQ_NONE;		/* It can't be ours.  */

	disable_irq_nosync(irq);
	atomic_inc(&phydev->irq_disable);

	return phy_change(phydev);
}

/**
 * phy_enable_interrupts - Enable the interrupts from the PHY side
 * @phydev: target phy_device struct
 */
static int phy_enable_interrupts(struct phy_device *phydev)
{
	int err = phy_clear_interrupt(phydev);

	if (err < 0)
		return err;

	return phy_config_interrupt(phydev, PHY_INTERRUPT_ENABLED);
}

/**
 * phy_start_interrupts - request and enable interrupts for a PHY device
 * @phydev: target phy_device struct
 *
 * Description: Request the interrupt for the given PHY.
 *   If this fails, then we set irq to PHY_POLL.
 *   Otherwise, we enable the interrupts in the PHY.
 *   This should only be called with a valid IRQ number.
 *   Returns 0 on success or < 0 on error.
 */
int phy_start_interrupts(struct phy_device *phydev)
{
	atomic_set(&phydev->irq_disable, 0);
	if (request_threaded_irq(phydev->irq, NULL, phy_interrupt,
				 IRQF_ONESHOT | IRQF_SHARED,
				 phydev_name(phydev), phydev) < 0) {
		pr_warn("%s: Can't get IRQ %d (PHY)\n",
			phydev->mdio.bus->name, phydev->irq);
		phydev->irq = PHY_POLL;
		return 0;
	}

	return phy_enable_interrupts(phydev);
}
EXPORT_SYMBOL(phy_start_interrupts);

/**
 * phy_stop_interrupts - disable interrupts from a PHY device
 * @phydev: target phy_device struct
 */
int phy_stop_interrupts(struct phy_device *phydev)
{
	int err = phy_disable_interrupts(phydev);

	if (err)
		phy_error(phydev);

	free_irq(phydev->irq, phydev);

	/* If work indeed has been cancelled, disable_irq() will have
	 * been left unbalanced from phy_interrupt() and enable_irq()
	 * has to be called so that other devices on the line work.
	 */
	while (atomic_dec_return(&phydev->irq_disable) >= 0)
		enable_irq(phydev->irq);

	return err;
}
EXPORT_SYMBOL(phy_stop_interrupts);

/**
 * phy_stop - Bring down the PHY link, and stop checking the status
 * @phydev: target phy_device struct
 */
void phy_stop(struct phy_device *phydev)
{
	mutex_lock(&phydev->lock);

	if (PHY_HALTED == phydev->state)
		goto out_unlock;

	if (phy_interrupt_is_valid(phydev)) {
		/* Disable PHY Interrupts */
		phy_config_interrupt(phydev, PHY_INTERRUPT_DISABLED);

		/* Clear any pending interrupts */
		phy_clear_interrupt(phydev);
	}

	phydev->state = PHY_HALTED;

out_unlock:
	mutex_unlock(&phydev->lock);

	/* Cannot call flush_scheduled_work() here as desired because
	 * of rtnl_lock(), but PHY_HALTED shall guarantee phy_change()
	 * will not reenable interrupts.
	 */
}
EXPORT_SYMBOL(phy_stop);

/**
 * phy_start - start or restart a PHY device
 * @phydev: target phy_device struct
 *
 * Description: Indicates the attached device's readiness to
 *   handle PHY-related work.  Used during startup to start the
 *   PHY, and after a call to phy_stop() to resume operation.
 *   Also used to indicate the MDIO bus has cleared an error
 *   condition.
 */
void phy_start(struct phy_device *phydev)
{
	int err = 0;

	mutex_lock(&phydev->lock);

	switch (phydev->state) {
	case PHY_STARTING:
		phydev->state = PHY_PENDING;
		break;
	case PHY_READY:
		phydev->state = PHY_UP;
		break;
	case PHY_HALTED:
		/* if phy was suspended, bring the physical link up again */
		__phy_resume(phydev);

		/* make sure interrupts are re-enabled for the PHY */
		if (phy_interrupt_is_valid(phydev)) {
			err = phy_enable_interrupts(phydev);
			if (err < 0)
				break;
		}

		phydev->state = PHY_RESUMING;
		break;
	default:
		break;
	}
	mutex_unlock(&phydev->lock);

	phy_trigger_machine(phydev, true);
}
EXPORT_SYMBOL(phy_start);

static void phy_link_up(struct phy_device *phydev)
{
	phydev->phy_link_change(phydev, true, true);
	phy_led_trigger_change_speed(phydev);
}

static void phy_link_down(struct phy_device *phydev, bool do_carrier)
{
	phydev->phy_link_change(phydev, false, do_carrier);
	phy_led_trigger_change_speed(phydev);
}

/**
 * phy_state_machine - Handle the state machine
 * @work: work_struct that describes the work to be done
 */
void phy_state_machine(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct phy_device *phydev =
			container_of(dwork, struct phy_device, state_queue);
	bool needs_aneg = false, do_suspend = false;
	enum phy_state old_state;
	int err = 0;
	int old_link;

	mutex_lock(&phydev->lock);

	old_state = phydev->state;

	if (phydev->drv && phydev->drv->link_change_notify)
		phydev->drv->link_change_notify(phydev);

	switch (phydev->state) {
	case PHY_DOWN:
	case PHY_STARTING:
	case PHY_READY:
	case PHY_PENDING:
		break;
	case PHY_UP:
		needs_aneg = true;

		phydev->link_timeout = PHY_AN_TIMEOUT;

		break;
	case PHY_AN:
		err = phy_read_status(phydev);
		if (err < 0)
			break;

		/* If the link is down, give up on negotiation for now */
		if (!phydev->link) {
			phydev->state = PHY_NOLINK;
			phy_link_down(phydev, true);
			break;
		}

		/* Check if negotiation is done.  Break if there's an error */
		err = phy_aneg_done(phydev);
		if (err < 0)
			break;

		/* If AN is done, we're running */
		if (err > 0) {
			phydev->state = PHY_RUNNING;
			phy_link_up(phydev);
		} else if (0 == phydev->link_timeout--)
			needs_aneg = true;
		break;
	case PHY_NOLINK:
		if (phy_interrupt_is_valid(phydev))
			break;

		err = phy_read_status(phydev);
		if (err)
			break;

		if (phydev->link) {
			if (AUTONEG_ENABLE == phydev->autoneg) {
				err = phy_aneg_done(phydev);
				if (err < 0)
					break;

				if (!err) {
					phydev->state = PHY_AN;
					phydev->link_timeout = PHY_AN_TIMEOUT;
					break;
				}
			}
			phydev->state = PHY_RUNNING;
			phy_link_up(phydev);
		}
		break;
	case PHY_FORCING:
		err = genphy_update_link(phydev);
		if (err)
			break;

		if (phydev->link) {
			phydev->state = PHY_RUNNING;
			phy_link_up(phydev);
		} else {
			if (0 == phydev->link_timeout--)
				needs_aneg = true;
			phy_link_down(phydev, false);
		}
		break;
	case PHY_RUNNING:
		/* Only register a CHANGE if we are polling and link changed
		 * since latest checking.
		 */
		if (phydev->irq == PHY_POLL) {
			old_link = phydev->link;
			err = phy_read_status(phydev);
			if (err)
				break;

			if (old_link != phydev->link)
				phydev->state = PHY_CHANGELINK;
		}
		/*
		 * Failsafe: check that nobody set phydev->link=0 between two
		 * poll cycles, otherwise we won't leave RUNNING state as long
		 * as link remains down.
		 */
		if (!phydev->link && phydev->state == PHY_RUNNING) {
			phydev->state = PHY_CHANGELINK;
			phydev_err(phydev, "no link in PHY_RUNNING\n");
		}
		break;
	case PHY_CHANGELINK:
		err = phy_read_status(phydev);
		if (err)
			break;

		if (phydev->link) {
			phydev->state = PHY_RUNNING;
			phy_link_up(phydev);
		} else {
			phydev->state = PHY_NOLINK;
			phy_link_down(phydev, true);
		}

		if (phy_interrupt_is_valid(phydev))
			err = phy_config_interrupt(phydev,
						   PHY_INTERRUPT_ENABLED);
		break;
	case PHY_HALTED:
		if (phydev->link) {
			phydev->link = 0;
			phy_link_down(phydev, true);
			do_suspend = true;
		}
		break;
	case PHY_RESUMING:
		if (AUTONEG_ENABLE == phydev->autoneg) {
			err = phy_aneg_done(phydev);
			if (err < 0)
				break;

			/* err > 0 if AN is done.
			 * Otherwise, it's 0, and we're  still waiting for AN
			 */
			if (err > 0) {
				err = phy_read_status(phydev);
				if (err)
					break;

				if (phydev->link) {
					phydev->state = PHY_RUNNING;
					phy_link_up(phydev);
				} else	{
					phydev->state = PHY_NOLINK;
					phy_link_down(phydev, false);
				}
			} else {
				phydev->state = PHY_AN;
				phydev->link_timeout = PHY_AN_TIMEOUT;
			}
		} else {
			err = phy_read_status(phydev);
			if (err)
				break;

			if (phydev->link) {
				phydev->state = PHY_RUNNING;
				phy_link_up(phydev);
			} else	{
				phydev->state = PHY_NOLINK;
				phy_link_down(phydev, false);
			}
		}
		break;
	}

	mutex_unlock(&phydev->lock);

	if (needs_aneg)
		err = phy_start_aneg_priv(phydev, false);
	else if (do_suspend)
		phy_suspend(phydev);

	if (err < 0)
		phy_error(phydev);

	if (old_state != phydev->state)
		phydev_dbg(phydev, "PHY state change %s -> %s\n",
			   phy_state_to_str(old_state),
			   phy_state_to_str(phydev->state));

	/* Only re-schedule a PHY state machine change if we are polling the
	 * PHY, if PHY_IGNORE_INTERRUPT is set, then we will be moving
	 * between states from phy_mac_interrupt()
	 */
	if (phydev->irq == PHY_POLL)
		queue_delayed_work(system_power_efficient_wq, &phydev->state_queue,
				   PHY_STATE_TIME * HZ);
}

/**
 * phy_mac_interrupt - MAC says the link has changed
 * @phydev: phy_device struct with changed link
 * @new_link: Link is Up/Down.
 *
 * Description: The MAC layer is able indicate there has been a change
 *   in the PHY link status. Set the new link status, and trigger the
 *   state machine, work a work queue.
 */
void phy_mac_interrupt(struct phy_device *phydev, int new_link)
{
	phydev->link = new_link;

	/* Trigger a state machine change */
	queue_work(system_power_efficient_wq, &phydev->phy_queue);
}
EXPORT_SYMBOL(phy_mac_interrupt);

/**
 * phy_init_eee - init and check the EEE feature
 * @phydev: target phy_device struct
 * @clk_stop_enable: PHY may stop the clock during LPI
 *
 * Description: it checks if the Energy-Efficient Ethernet (EEE)
 * is supported by looking at the MMD registers 3.20 and 7.60/61
 * and it programs the MMD register 3.0 setting the "Clock stop enable"
 * bit if required.
 */
int phy_init_eee(struct phy_device *phydev, bool clk_stop_enable)
{
	if (!phydev->drv)
		return -EIO;

	/* According to 802.3az,the EEE is supported only in full duplex-mode.
	 */
	if (phydev->duplex == DUPLEX_FULL) {
		int eee_lp, eee_cap, eee_adv;
		u32 lp, cap, adv;
		int status;

		/* Read phy status to properly get the right settings */
		status = phy_read_status(phydev);
		if (status)
			return status;

		/* First check if the EEE ability is supported */
		eee_cap = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_PCS_EEE_ABLE);
		if (eee_cap <= 0)
			goto eee_exit_err;

		cap = mmd_eee_cap_to_ethtool_sup_t(eee_cap);
		if (!cap)
			goto eee_exit_err;

		/* Check which link settings negotiated and verify it in
		 * the EEE advertising registers.
		 */
		eee_lp = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_LPABLE);
		if (eee_lp <= 0)
			goto eee_exit_err;

		eee_adv = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV);
		if (eee_adv <= 0)
			goto eee_exit_err;

		adv = mmd_eee_adv_to_ethtool_adv_t(eee_adv);
		lp = mmd_eee_adv_to_ethtool_adv_t(eee_lp);
		if (!phy_check_valid(phydev->speed, phydev->duplex, lp & adv))
			goto eee_exit_err;

		if (clk_stop_enable) {
			/* Configure the PHY to stop receiving xMII
			 * clock while it is signaling LPI.
			 */
			int val = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1);
			if (val < 0)
				return val;

			val |= MDIO_PCS_CTRL1_CLKSTOP_EN;
			phy_write_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1, val);
		}

		return 0; /* EEE supported */
	}
eee_exit_err:
	return -EPROTONOSUPPORT;
}
EXPORT_SYMBOL(phy_init_eee);

/**
 * phy_get_eee_err - report the EEE wake error count
 * @phydev: target phy_device struct
 *
 * Description: it is to report the number of time where the PHY
 * failed to complete its normal wake sequence.
 */
int phy_get_eee_err(struct phy_device *phydev)
{
	if (!phydev->drv)
		return -EIO;

	return phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_PCS_EEE_WK_ERR);
}
EXPORT_SYMBOL(phy_get_eee_err);

/**
 * phy_ethtool_get_eee - get EEE supported and status
 * @phydev: target phy_device struct
 * @data: ethtool_eee data
 *
 * Description: it reportes the Supported/Advertisement/LP Advertisement
 * capabilities.
 */
int phy_ethtool_get_eee(struct phy_device *phydev, struct ethtool_eee *data)
{
	int val;

	if (!phydev->drv)
		return -EIO;

	/* Get Supported EEE */
	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_PCS_EEE_ABLE);
	if (val < 0)
		return val;
	data->supported = mmd_eee_cap_to_ethtool_sup_t(val);

	/* Get advertisement EEE */
	val = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV);
	if (val < 0)
		return val;
	data->advertised = mmd_eee_adv_to_ethtool_adv_t(val);

	/* Get LP advertisement EEE */
	val = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_LPABLE);
	if (val < 0)
		return val;
	data->lp_advertised = mmd_eee_adv_to_ethtool_adv_t(val);

	return 0;
}
EXPORT_SYMBOL(phy_ethtool_get_eee);

/**
 * phy_ethtool_set_eee - set EEE supported and status
 * @phydev: target phy_device struct
 * @data: ethtool_eee data
 *
 * Description: it is to program the Advertisement EEE register.
 */
int phy_ethtool_set_eee(struct phy_device *phydev, struct ethtool_eee *data)
{
	int cap, old_adv, adv, ret;

	if (!phydev->drv)
		return -EIO;

	/* Get Supported EEE */
	cap = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_PCS_EEE_ABLE);
	if (cap < 0)
		return cap;

	old_adv = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV);
	if (old_adv < 0)
		return old_adv;

	adv = ethtool_adv_to_mmd_eee_adv_t(data->advertised) & cap;

	/* Mask prohibited EEE modes */
	adv &= ~phydev->eee_broken_modes;

	if (old_adv != adv) {
		ret = phy_write_mmd(phydev, MDIO_MMD_AN, MDIO_AN_EEE_ADV, adv);
		if (ret < 0)
			return ret;

		/* Restart autonegotiation so the new modes get sent to the
		 * link partner.
		 */
		ret = phy_restart_aneg(phydev);
		if (ret < 0)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(phy_ethtool_set_eee);

int phy_ethtool_set_wol(struct phy_device *phydev, struct ethtool_wolinfo *wol)
{
	if (phydev->drv && phydev->drv->set_wol)
		return phydev->drv->set_wol(phydev, wol);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL(phy_ethtool_set_wol);

void phy_ethtool_get_wol(struct phy_device *phydev, struct ethtool_wolinfo *wol)
{
	if (phydev->drv && phydev->drv->get_wol)
		phydev->drv->get_wol(phydev, wol);
}
EXPORT_SYMBOL(phy_ethtool_get_wol);

int phy_ethtool_get_link_ksettings(struct net_device *ndev,
				   struct ethtool_link_ksettings *cmd)
{
	struct phy_device *phydev = ndev->phydev;

	if (!phydev)
		return -ENODEV;

	phy_ethtool_ksettings_get(phydev, cmd);

	return 0;
}
EXPORT_SYMBOL(phy_ethtool_get_link_ksettings);

int phy_ethtool_set_link_ksettings(struct net_device *ndev,
				   const struct ethtool_link_ksettings *cmd)
{
	struct phy_device *phydev = ndev->phydev;

	if (!phydev)
		return -ENODEV;

	return phy_ethtool_ksettings_set(phydev, cmd);
}
EXPORT_SYMBOL(phy_ethtool_set_link_ksettings);

int phy_ethtool_nway_reset(struct net_device *ndev)
{
	struct phy_device *phydev = ndev->phydev;

	if (!phydev)
		return -ENODEV;

	if (!phydev->drv)
		return -EIO;

	return phy_restart_aneg(phydev);
}
EXPORT_SYMBOL(phy_ethtool_nway_reset);
