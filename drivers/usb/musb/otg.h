/******************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR
 * GRAPHICS SUPPORT CUSTOMER.
 ******************************************************************/

/*
 * Interface to a generic OTG state machine for use by an OTG controller.
 *
 * FIXME most of this must vanish; usbcore handles some of it, and
 * the OTG parts of a peripheral controller (and its driver) handle
 * other things.  Package it as an "otg transceiver".
 */

#ifndef __MUSB_LINUX_OTG_H__
#define __MUSB_LINUX_OTG_H__

#include <linux/spinlock.h>
#include <linux/timer.h>

/**
 * Introduction.
 * An OTG state machine for use by a controller driver for an OTG controller
 * that wishes to be OTG-aware.
 * The state machine requires relevant inputs and a couple of services
 * from the controller driver, and calls the controller driver to inform
 * it of the current state and errors.
 * Finally, it provides the necessary bus control service.
 */

/* sysfs flag to seletively force peripheral-only operation */
extern int musb_otg;

/****************************** CONSTANTS ********************************/

/*
 * Define this (in milliseconds) to a target-specific value to override default.
 * The OTG-spec minimum is 5000, and maximum is 6000 (see OTG spec errata).
 */
#ifndef MGC_OTG_T_B_SRP_FAIL
#define MGC_OTG_T_B_SRP_FAIL	5000
#endif

/*
 * Define this (in milliseconds) to a target-specific value to override default.
 * This is the time an A-device should wait for a B-device to connect.
 * The OTG-spec minimum is 1000.
 * As a special case, for normal host-like behavior, you can set this to 0.
 */
#ifndef MGC_OTG_T_A_WAIT_BCON
#define MGC_OTG_T_A_WAIT_BCON	1000
#endif

/*
 * Define this (in milliseconds) to a target-specific value to override default.
 * The OTG-spec minimum is 250.
 */
#ifndef MGC_OTG_T_AIDL_BDIS
#define MGC_OTG_T_AIDL_BDIS	250
#endif

//#define MGC_OTG_T_B_ASE0_BRST 4
#define MGC_OTG_T_B_ASE0_BRST	100

/*
 * MGC_OtgRequest.
 * A software request for the OTG state machine
 */
typedef enum {
	MGC_OTG_REQUEST_UNKNOWN,
    /** Request the bus */
	MGC_OTG_REQUEST_START_BUS,
    /** Drop the bus */
	MGC_OTG_REQUEST_DROP_BUS,
    /** Suspend the bus */
	MGC_OTG_REQUEST_SUSPEND_BUS,
    /** Reset the state machine */
	MGC_OTG_REQUEST_RESET
} MGC_OtgRequest;


/******************************** TYPES **********************************/

/*
 * MGC_OtgMachineInputs.
 * The set of inputs which drives the state machine
 * @field bSession TRUE when a session is in progress; FALSE when not
 * @field bConnectorId TRUE for B-device; FALSE for A-device
 * (assumed valid only when a bSession is TRUE)
 * @field bReset TRUE when reset is detected (peripheral role only)
 * @field bConnection TRUE when connection is detected (host role only)
 * @field bSuspend TRUE when bus suspend is detected
 * @field bVbusError TRUE when a Vbus error is detected
 */
typedef struct {
	u8 bSession;
	u8 bConnectorId;
	u8 bReset;
	u8 bConnection;
	u8 bSuspend;
	u8 bVbusError;
} MGC_OtgMachineInputs;

/*
 * OTG state machine instance data.
 * @field Lock spinlock
 * @field bState current state (one of the OTG_STATE_* constants)
 * @field pOtgServices pointer to OTG services
 * @field Timer interval timer for status change interrupts
 * @field bState current state
 * @field bRequest current pending request
 */
struct otg_machine {
	spinlock_t Lock;
	struct musb		*musb;
	enum usb_otg_state	bState;
	struct timer_list Timer;
	MGC_OtgRequest bRequest;

	/* FIXME standard Linux-USB host and peripheral code includes
	 * OTG support ... most of this "otg machine" must vanish
	 */

};

/****************************** FUNCTIONS ********************************/

/*
 * Initialize an OTG state machine.
 */
extern void MGC_OtgMachineInit(struct otg_machine * pMachine,
			     struct musb *musb);

/*
 * Destroy an OTG state machine
 * @param pMachine machine pointer
 * @see #MGC_OtgMachineInit
 */
extern void MGC_OtgMachineDestroy(struct otg_machine * pMachine);

/*
 * OTG inputs have changed.
 * A controller driver calls this when anything in the
 * MGC_OtgMachineInputs has changed
 * @param pMachine machine pointer
 * @param pInputs current inputs
 * @see #MGC_OtgMachineInit
 */
extern void MGC_OtgMachineInputsChanged(struct otg_machine * pMachine,
					const MGC_OtgMachineInputs * pInputs);

#endif				/* multiple inclusion protection */
