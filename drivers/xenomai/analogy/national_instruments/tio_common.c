/*
 * Hardware driver for NI general purpose counter
 * Copyright (C) 2006 Frank Mori Hess <fmhess@users.sourceforge.net>
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Description: National Instruments general purpose counters
 * This module is not used directly by end-users.  Rather, it is used
 * by other drivers (for example ni_660x and ni_pcimio) to provide
 * support for NI's general purpose counters.  It was originally based
 * on the counter code from ni_660x.c and ni_mio_common.c.
 *
 * Author:
 * J.P. Mellor <jpmellor@rose-hulman.edu>
 * Herman.Bruyninckx@mech.kuleuven.ac.be
 * Wim.Meeussen@mech.kuleuven.ac.be,
 * Klaas.Gadeyne@mech.kuleuven.ac.be,
 * Frank Mori Hess <fmhess@users.sourceforge.net>
 *
 * References:
 * DAQ 660x Register-Level Programmer Manual  (NI 370505A-01)
 * DAQ 6601/6602 User Manual (NI 322137B-01)
 * 340934b.pdf  DAQ-STC reference manual
 *
 * TODO:
 * - Support use of both banks X and Y
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <rtdm/analogy/device.h>

#include "ni_tio.h"
#include "ni_mio.h"

static inline void write_register(struct ni_gpct *counter,
				  unsigned int bits, enum ni_gpct_register reg)
{
	BUG_ON(reg >= NITIO_Num_Registers);
	counter->counter_dev->write_register(counter, bits, reg);
}

static inline unsigned int read_register(struct ni_gpct *counter,
				     enum ni_gpct_register reg)
{
	BUG_ON(reg >= NITIO_Num_Registers);
	return counter->counter_dev->read_register(counter, reg);
}

struct ni_gpct_device *a4l_ni_gpct_device_construct(struct a4l_device * dev,
	void (*write_register) (struct ni_gpct * counter, unsigned int bits,
		enum ni_gpct_register reg),
	unsigned int (*read_register) (struct ni_gpct * counter,
		enum ni_gpct_register reg), enum ni_gpct_variant variant,
	unsigned int num_counters)
{
	struct ni_gpct_device *counter_dev =
		kmalloc(sizeof(struct ni_gpct_device), GFP_KERNEL);
	if (counter_dev == NULL)
		return NULL;

	memset(counter_dev, 0, sizeof(struct ni_gpct_device));

	counter_dev->dev = dev;
	counter_dev->write_register = write_register;
	counter_dev->read_register = read_register;
	counter_dev->variant = variant;
	rtdm_lock_init(&counter_dev->regs_lock);
	BUG_ON(num_counters == 0);

	counter_dev->counters =
		kmalloc(sizeof(struct ni_gpct *) * num_counters, GFP_KERNEL);

	if (counter_dev->counters == NULL) {
		 kfree(counter_dev);
		return NULL;
	}

	memset(counter_dev->counters, 0, sizeof(struct ni_gpct *) * num_counters);

	counter_dev->num_counters = num_counters;
	return counter_dev;
}

void a4l_ni_gpct_device_destroy(struct ni_gpct_device *counter_dev)
{
	if (counter_dev->counters == NULL)
		return;
	kfree(counter_dev->counters);
	kfree(counter_dev);
}

static
int ni_tio_counting_mode_registers_present(const struct ni_gpct_device *counter_dev)
{
	switch (counter_dev->variant) {
	case ni_gpct_variant_e_series:
		return 0;
		break;
	case ni_gpct_variant_m_series:
	case ni_gpct_variant_660x:
		return 1;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static
int ni_tio_second_gate_registers_present(const struct ni_gpct_device *counter_dev)
{
	switch (counter_dev->variant) {
	case ni_gpct_variant_e_series:
		return 0;
		break;
	case ni_gpct_variant_m_series:
	case ni_gpct_variant_660x:
		return 1;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline
void ni_tio_set_bits_transient(struct ni_gpct *counter,
			       enum ni_gpct_register register_index,
			       unsigned int bit_mask,
			       unsigned int bit_values,
			       unsigned transient_bit_values)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	unsigned long flags;

	BUG_ON(register_index >= NITIO_Num_Registers);
	rtdm_lock_get_irqsave(&counter_dev->regs_lock, flags);
	counter_dev->regs[register_index] &= ~bit_mask;
	counter_dev->regs[register_index] |= (bit_values & bit_mask);
	write_register(counter,
		       counter_dev->regs[register_index] | transient_bit_values,
		       register_index);
	mmiowb();
	rtdm_lock_put_irqrestore(&counter_dev->regs_lock, flags);
}

/* ni_tio_set_bits( ) is for safely writing to registers whose bits
   may be twiddled in interrupt context, or whose software copy may be
   read in interrupt context. */
static inline void ni_tio_set_bits(struct ni_gpct *counter,
				   enum ni_gpct_register register_index,
				   unsigned int bit_mask,
				   unsigned int bit_values)
{
	ni_tio_set_bits_transient(counter,
				  register_index,
				  bit_mask, bit_values, 0x0);
}

/* ni_tio_get_soft_copy( ) is for safely reading the software copy of
   a register whose bits might be modified in interrupt context, or whose
   software copy might need to be read in interrupt context. */
static inline
unsigned int ni_tio_get_soft_copy(const struct ni_gpct *counter,
				  enum ni_gpct_register register_index)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	unsigned long flags;
	unsigned value;

	BUG_ON(register_index >= NITIO_Num_Registers);
	rtdm_lock_get_irqsave(&counter_dev->regs_lock, flags);
	value = counter_dev->regs[register_index];
	rtdm_lock_put_irqrestore(&counter_dev->regs_lock, flags);
	return value;
}

static void ni_tio_reset_count_and_disarm(struct ni_gpct *counter)
{
	write_register(counter, Gi_Reset_Bit(counter->counter_index),
		       NITIO_Gxx_Joint_Reset_Reg(counter->counter_index));
}

void a4l_ni_tio_init_counter(struct ni_gpct *counter)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;

	ni_tio_reset_count_and_disarm(counter);
	/* Initialize counter registers */
	counter_dev->regs[NITIO_Gi_Autoincrement_Reg(counter->counter_index)] =
		0x0;
	write_register(counter,
		counter_dev->regs[NITIO_Gi_Autoincrement_Reg(counter->
				counter_index)],
		NITIO_Gi_Autoincrement_Reg(counter->counter_index));
	ni_tio_set_bits(counter, NITIO_Gi_Command_Reg(counter->counter_index),
		~0, Gi_Synchronize_Gate_Bit);
	ni_tio_set_bits(counter, NITIO_Gi_Mode_Reg(counter->counter_index), ~0,
		0);
	counter_dev->regs[NITIO_Gi_LoadA_Reg(counter->counter_index)] = 0x0;
	write_register(counter,
		counter_dev->regs[NITIO_Gi_LoadA_Reg(counter->counter_index)],
		NITIO_Gi_LoadA_Reg(counter->counter_index));
	counter_dev->regs[NITIO_Gi_LoadB_Reg(counter->counter_index)] = 0x0;
	write_register(counter,
		counter_dev->regs[NITIO_Gi_LoadB_Reg(counter->counter_index)],
		NITIO_Gi_LoadB_Reg(counter->counter_index));
	ni_tio_set_bits(counter,
		NITIO_Gi_Input_Select_Reg(counter->counter_index), ~0, 0);
	if (ni_tio_counting_mode_registers_present(counter_dev)) {
		ni_tio_set_bits(counter,
			NITIO_Gi_Counting_Mode_Reg(counter->counter_index), ~0,
			0);
	}
	if (ni_tio_second_gate_registers_present(counter_dev)) {
		counter_dev->regs[NITIO_Gi_Second_Gate_Reg(counter->
				counter_index)] = 0x0;
		write_register(counter,
			counter_dev->regs[NITIO_Gi_Second_Gate_Reg(counter->
					counter_index)],
			NITIO_Gi_Second_Gate_Reg(counter->counter_index));
	}
	ni_tio_set_bits(counter,
		NITIO_Gi_DMA_Config_Reg(counter->counter_index), ~0, 0x0);
	ni_tio_set_bits(counter,
		NITIO_Gi_Interrupt_Enable_Reg(counter->counter_index), ~0, 0x0);
}

static lsampl_t ni_tio_counter_status(struct ni_gpct *counter)
{
	lsampl_t status = 0;
	unsigned int bits;

	bits = read_register(counter,NITIO_Gxx_Status_Reg(counter->counter_index));
	if (bits & Gi_Armed_Bit(counter->counter_index)) {
		status |= A4L_COUNTER_ARMED;
		if (bits & Gi_Counting_Bit(counter->counter_index))
			status |= A4L_COUNTER_COUNTING;
	}
	return status;
}

static
uint64_t ni_tio_clock_period_ps(const struct ni_gpct *counter,
				unsigned int generic_clock_source);
static
unsigned int ni_tio_generic_clock_src_select(const struct ni_gpct *counter);

static void ni_tio_set_sync_mode(struct ni_gpct *counter, int force_alt_sync)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned counting_mode_reg =
		NITIO_Gi_Counting_Mode_Reg(counter->counter_index);
	static const uint64_t min_normal_sync_period_ps = 25000;
	const uint64_t clock_period_ps = ni_tio_clock_period_ps(counter,
		ni_tio_generic_clock_src_select(counter));

	if (ni_tio_counting_mode_registers_present(counter_dev) == 0)
		return;

	switch (ni_tio_get_soft_copy(counter,
			counting_mode_reg) & Gi_Counting_Mode_Mask) {
	case Gi_Counting_Mode_QuadratureX1_Bits:
	case Gi_Counting_Mode_QuadratureX2_Bits:
	case Gi_Counting_Mode_QuadratureX4_Bits:
	case Gi_Counting_Mode_Sync_Source_Bits:
		force_alt_sync = 1;
		break;
	default:
		break;
	}

	/* It's not clear what we should do if clock_period is
	unknown, so we are not using the alt sync bit in that case,
	but allow the caller to decide by using the force_alt_sync
	parameter. */
	if (force_alt_sync ||
		(clock_period_ps
			&& clock_period_ps < min_normal_sync_period_ps)) {
		ni_tio_set_bits(counter, counting_mode_reg,
			Gi_Alternate_Sync_Bit(counter_dev->variant),
			Gi_Alternate_Sync_Bit(counter_dev->variant));
	} else {
		ni_tio_set_bits(counter, counting_mode_reg,
			Gi_Alternate_Sync_Bit(counter_dev->variant), 0x0);
	}
}

static int ni_tio_set_counter_mode(struct ni_gpct *counter, unsigned int mode)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	unsigned mode_reg_mask;
	unsigned mode_reg_values;
	unsigned input_select_bits = 0;

	/* these bits map directly on to the mode register */
	static const unsigned mode_reg_direct_mask =
		NI_GPCT_GATE_ON_BOTH_EDGES_BIT | NI_GPCT_EDGE_GATE_MODE_MASK |
		NI_GPCT_STOP_MODE_MASK | NI_GPCT_OUTPUT_MODE_MASK |
		NI_GPCT_HARDWARE_DISARM_MASK | NI_GPCT_LOADING_ON_TC_BIT |
		NI_GPCT_LOADING_ON_GATE_BIT | NI_GPCT_LOAD_B_SELECT_BIT;

	mode_reg_mask = mode_reg_direct_mask | Gi_Reload_Source_Switching_Bit;
	mode_reg_values = mode & mode_reg_direct_mask;
	switch (mode & NI_GPCT_RELOAD_SOURCE_MASK) {
	case NI_GPCT_RELOAD_SOURCE_FIXED_BITS:
		break;
	case NI_GPCT_RELOAD_SOURCE_SWITCHING_BITS:
		mode_reg_values |= Gi_Reload_Source_Switching_Bit;
		break;
	case NI_GPCT_RELOAD_SOURCE_GATE_SELECT_BITS:
		input_select_bits |= Gi_Gate_Select_Load_Source_Bit;
		mode_reg_mask |= Gi_Gating_Mode_Mask;
		mode_reg_values |= Gi_Level_Gating_Bits;
		break;
	default:
		break;
	}
	ni_tio_set_bits(counter, NITIO_Gi_Mode_Reg(counter->counter_index),
		mode_reg_mask, mode_reg_values);

	if (ni_tio_counting_mode_registers_present(counter_dev)) {
		unsigned counting_mode_bits = 0;
		counting_mode_bits |=
			(mode >> NI_GPCT_COUNTING_MODE_SHIFT) &
			Gi_Counting_Mode_Mask;
		counting_mode_bits |=
			((mode >> NI_GPCT_INDEX_PHASE_BITSHIFT) <<
			Gi_Index_Phase_Bitshift) & Gi_Index_Phase_Mask;
		if (mode & NI_GPCT_INDEX_ENABLE_BIT) {
			counting_mode_bits |= Gi_Index_Mode_Bit;
		}
		ni_tio_set_bits(counter,
			NITIO_Gi_Counting_Mode_Reg(counter->counter_index),
			Gi_Counting_Mode_Mask | Gi_Index_Phase_Mask |
			Gi_Index_Mode_Bit, counting_mode_bits);
		ni_tio_set_sync_mode(counter, 0);
	}

	ni_tio_set_bits(counter, NITIO_Gi_Command_Reg(counter->counter_index),
		Gi_Up_Down_Mask,
		(mode >> NI_GPCT_COUNTING_DIRECTION_SHIFT) << Gi_Up_Down_Shift);

	if (mode & NI_GPCT_OR_GATE_BIT) {
		input_select_bits |= Gi_Or_Gate_Bit;
	}
	if (mode & NI_GPCT_INVERT_OUTPUT_BIT) {
		input_select_bits |= Gi_Output_Polarity_Bit;
	}
	ni_tio_set_bits(counter,
		NITIO_Gi_Input_Select_Reg(counter->counter_index),
		Gi_Gate_Select_Load_Source_Bit | Gi_Or_Gate_Bit |
		Gi_Output_Polarity_Bit, input_select_bits);

	return 0;
}

static int ni_tio_arm(struct ni_gpct *counter, int arm, unsigned int start_trigger)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;

	unsigned int command_transient_bits = 0;

	if (arm) {
		switch (start_trigger) {
		case NI_GPCT_ARM_IMMEDIATE:
			command_transient_bits |= Gi_Arm_Bit;
			break;
		case NI_GPCT_ARM_PAIRED_IMMEDIATE:
			command_transient_bits |= Gi_Arm_Bit | Gi_Arm_Copy_Bit;
			break;
		default:
			break;
		}
		if (ni_tio_counting_mode_registers_present(counter_dev)) {
			unsigned counting_mode_bits = 0;

			switch (start_trigger) {
			case NI_GPCT_ARM_IMMEDIATE:
			case NI_GPCT_ARM_PAIRED_IMMEDIATE:
				break;
			default:
				if (start_trigger & NI_GPCT_ARM_UNKNOWN) {
					/* Pass-through the least
					significant bits so we can
					figure out what select later
					*/
					unsigned hw_arm_select_bits =
						(start_trigger <<
						Gi_HW_Arm_Select_Shift) &
						Gi_HW_Arm_Select_Mask
						(counter_dev->variant);

					counting_mode_bits |=
						Gi_HW_Arm_Enable_Bit |
						hw_arm_select_bits;
				} else {
					return -EINVAL;
				}
				break;
			}
			ni_tio_set_bits(counter,
				NITIO_Gi_Counting_Mode_Reg(counter->
					counter_index),
				Gi_HW_Arm_Select_Mask(counter_dev->
					variant) | Gi_HW_Arm_Enable_Bit,
				counting_mode_bits);
		}
	} else {
		command_transient_bits |= Gi_Disarm_Bit;
	}
	ni_tio_set_bits_transient(counter,
		NITIO_Gi_Command_Reg(counter->counter_index), 0, 0,
		command_transient_bits);
	return 0;
}

static unsigned int ni_660x_source_select_bits(lsampl_t clock_source)
{
	unsigned int ni_660x_clock;
	unsigned int i;
	const unsigned int clock_select_bits =
		clock_source & NI_GPCT_CLOCK_SRC_SELECT_MASK;

	switch (clock_select_bits) {
	case NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Timebase_1_Clock;
		break;
	case NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Timebase_2_Clock;
		break;
	case NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Timebase_3_Clock;
		break;
	case NI_GPCT_LOGIC_LOW_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Logic_Low_Clock;
		break;
	case NI_GPCT_SOURCE_PIN_i_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Source_Pin_i_Clock;
		break;
	case NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Next_Gate_Clock;
		break;
	case NI_GPCT_NEXT_TC_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Next_TC_Clock;
		break;
	default:
		for (i = 0; i <= ni_660x_max_rtsi_channel; ++i) {
			if (clock_select_bits == NI_GPCT_RTSI_CLOCK_SRC_BITS(i)) {
				ni_660x_clock = NI_660x_RTSI_Clock(i);
				break;
			}
		}
		if (i <= ni_660x_max_rtsi_channel)
			break;
		for (i = 0; i <= ni_660x_max_source_pin; ++i) {
			if (clock_select_bits ==
				NI_GPCT_SOURCE_PIN_CLOCK_SRC_BITS(i)) {
				ni_660x_clock = NI_660x_Source_Pin_Clock(i);
				break;
			}
		}
		if (i <= ni_660x_max_source_pin)
			break;
		ni_660x_clock = 0;
		BUG();
		break;
	}
	return Gi_Source_Select_Bits(ni_660x_clock);
}

static unsigned int ni_m_series_source_select_bits(lsampl_t clock_source)
{
	unsigned int ni_m_series_clock;
	unsigned int i;
	const unsigned int clock_select_bits =
		clock_source & NI_GPCT_CLOCK_SRC_SELECT_MASK;
	switch (clock_select_bits) {
	case NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Timebase_1_Clock;
		break;
	case NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Timebase_2_Clock;
		break;
	case NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Timebase_3_Clock;
		break;
	case NI_GPCT_LOGIC_LOW_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Logic_Low_Clock;
		break;
	case NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Next_Gate_Clock;
		break;
	case NI_GPCT_NEXT_TC_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Next_TC_Clock;
		break;
	case NI_GPCT_PXI10_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_PXI10_Clock;
		break;
	case NI_GPCT_PXI_STAR_TRIGGER_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_PXI_Star_Trigger_Clock;
		break;
	case NI_GPCT_ANALOG_TRIGGER_OUT_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Analog_Trigger_Out_Clock;
		break;
	default:
		for (i = 0; i <= ni_m_series_max_rtsi_channel; ++i) {
			if (clock_select_bits == NI_GPCT_RTSI_CLOCK_SRC_BITS(i)) {
				ni_m_series_clock = NI_M_Series_RTSI_Clock(i);
				break;
			}
		}
		if (i <= ni_m_series_max_rtsi_channel)
			break;
		for (i = 0; i <= ni_m_series_max_pfi_channel; ++i) {
			if (clock_select_bits == NI_GPCT_PFI_CLOCK_SRC_BITS(i)) {
				ni_m_series_clock = NI_M_Series_PFI_Clock(i);
				break;
			}
		}
		if (i <= ni_m_series_max_pfi_channel)
			break;
		__a4l_err("invalid clock source 0x%lx\n",
			     (unsigned long)clock_source);
		BUG();
		ni_m_series_clock = 0;
		break;
	}
	return Gi_Source_Select_Bits(ni_m_series_clock);
}

static void ni_tio_set_source_subselect(struct ni_gpct *counter,
					lsampl_t clock_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned second_gate_reg =
		NITIO_Gi_Second_Gate_Reg(counter->counter_index);

	if (counter_dev->variant != ni_gpct_variant_m_series)
		return;
	switch (clock_source & NI_GPCT_CLOCK_SRC_SELECT_MASK) {
		/* Gi_Source_Subselect is zero */
	case NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS:
	case NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS:
		counter_dev->regs[second_gate_reg] &= ~Gi_Source_Subselect_Bit;
		break;
		/* Gi_Source_Subselect is one */
	case NI_GPCT_ANALOG_TRIGGER_OUT_CLOCK_SRC_BITS:
	case NI_GPCT_PXI_STAR_TRIGGER_CLOCK_SRC_BITS:
		counter_dev->regs[second_gate_reg] |= Gi_Source_Subselect_Bit;
		break;
		/* Gi_Source_Subselect doesn't matter */
	default:
		return;
		break;
	}
	write_register(counter, counter_dev->regs[second_gate_reg],
		second_gate_reg);
}

static int ni_tio_set_clock_src(struct ni_gpct *counter,
				lsampl_t clock_source, lsampl_t period_ns)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	unsigned input_select_bits = 0;
	static const uint64_t pico_per_nano = 1000;

	/* FIXME: validate clock source */
	switch (counter_dev->variant) {
	case ni_gpct_variant_660x:
		input_select_bits |= ni_660x_source_select_bits(clock_source);
		break;
	case ni_gpct_variant_e_series:
	case ni_gpct_variant_m_series:
		input_select_bits |=
			ni_m_series_source_select_bits(clock_source);
		break;
	default:
		BUG();
		break;
	}
	if (clock_source & NI_GPCT_INVERT_CLOCK_SRC_BIT)
		input_select_bits |= Gi_Source_Polarity_Bit;
	ni_tio_set_bits(counter,
		NITIO_Gi_Input_Select_Reg(counter->counter_index),
		Gi_Source_Select_Mask | Gi_Source_Polarity_Bit,
		input_select_bits);
	ni_tio_set_source_subselect(counter, clock_source);
	if (ni_tio_counting_mode_registers_present(counter_dev)) {
		const unsigned prescaling_mode =
			clock_source & NI_GPCT_PRESCALE_MODE_CLOCK_SRC_MASK;
		unsigned counting_mode_bits = 0;

		switch (prescaling_mode) {
		case NI_GPCT_NO_PRESCALE_CLOCK_SRC_BITS:
			break;
		case NI_GPCT_PRESCALE_X2_CLOCK_SRC_BITS:
			counting_mode_bits |=
				Gi_Prescale_X2_Bit(counter_dev->variant);
			break;
		case NI_GPCT_PRESCALE_X8_CLOCK_SRC_BITS:
			counting_mode_bits |=
				Gi_Prescale_X8_Bit(counter_dev->variant);
			break;
		default:
			return -EINVAL;
			break;
		}
		ni_tio_set_bits(counter,
			NITIO_Gi_Counting_Mode_Reg(counter->counter_index),
			Gi_Prescale_X2_Bit(counter_dev->
				variant) | Gi_Prescale_X8_Bit(counter_dev->
				variant), counting_mode_bits);
	}
	counter->clock_period_ps = pico_per_nano * period_ns;
	ni_tio_set_sync_mode(counter, 0);
	return 0;
}

static unsigned int ni_tio_clock_src_modifiers(const struct ni_gpct *counter)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned counting_mode_bits = ni_tio_get_soft_copy(counter,
		NITIO_Gi_Counting_Mode_Reg(counter->counter_index));
	unsigned int bits = 0;

	if (ni_tio_get_soft_copy(counter,
			NITIO_Gi_Input_Select_Reg(counter->
				counter_index)) & Gi_Source_Polarity_Bit)
		bits |= NI_GPCT_INVERT_CLOCK_SRC_BIT;
	if (counting_mode_bits & Gi_Prescale_X2_Bit(counter_dev->variant))
		bits |= NI_GPCT_PRESCALE_X2_CLOCK_SRC_BITS;
	if (counting_mode_bits & Gi_Prescale_X8_Bit(counter_dev->variant))
		bits |= NI_GPCT_PRESCALE_X8_CLOCK_SRC_BITS;
	return bits;
}

static unsigned int ni_m_series_clock_src_select(const struct ni_gpct *counter)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned int second_gate_reg =
		NITIO_Gi_Second_Gate_Reg(counter->counter_index);
	unsigned int i, clock_source = 0;

	const unsigned int input_select = (ni_tio_get_soft_copy(counter,
			NITIO_Gi_Input_Select_Reg(counter->
				counter_index)) & Gi_Source_Select_Mask) >>
		Gi_Source_Select_Shift;

	switch (input_select) {
	case NI_M_Series_Timebase_1_Clock:
		clock_source = NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_Timebase_2_Clock:
		clock_source = NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_Timebase_3_Clock:
		if (counter_dev->
			regs[second_gate_reg] & Gi_Source_Subselect_Bit)
			clock_source =
				NI_GPCT_ANALOG_TRIGGER_OUT_CLOCK_SRC_BITS;
		else
			clock_source = NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_Logic_Low_Clock:
		clock_source = NI_GPCT_LOGIC_LOW_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_Next_Gate_Clock:
		if (counter_dev->
			regs[second_gate_reg] & Gi_Source_Subselect_Bit)
			clock_source = NI_GPCT_PXI_STAR_TRIGGER_CLOCK_SRC_BITS;
		else
			clock_source = NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_PXI10_Clock:
		clock_source = NI_GPCT_PXI10_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_Next_TC_Clock:
		clock_source = NI_GPCT_NEXT_TC_CLOCK_SRC_BITS;
		break;
	default:
		for (i = 0; i <= ni_m_series_max_rtsi_channel; ++i) {
			if (input_select == NI_M_Series_RTSI_Clock(i)) {
				clock_source = NI_GPCT_RTSI_CLOCK_SRC_BITS(i);
				break;
			}
		}
		if (i <= ni_m_series_max_rtsi_channel)
			break;
		for (i = 0; i <= ni_m_series_max_pfi_channel; ++i) {
			if (input_select == NI_M_Series_PFI_Clock(i)) {
				clock_source = NI_GPCT_PFI_CLOCK_SRC_BITS(i);
				break;
			}
		}
		if (i <= ni_m_series_max_pfi_channel)
			break;
		BUG();
		break;
	}
	clock_source |= ni_tio_clock_src_modifiers(counter);
	return clock_source;
}

static unsigned int ni_660x_clock_src_select(const struct ni_gpct *counter)
{
	unsigned int i, clock_source = 0;
	const unsigned input_select = (ni_tio_get_soft_copy(counter,
			NITIO_Gi_Input_Select_Reg(counter->
				counter_index)) & Gi_Source_Select_Mask) >>
		Gi_Source_Select_Shift;

	switch (input_select) {
	case NI_660x_Timebase_1_Clock:
		clock_source = NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS;
		break;
	case NI_660x_Timebase_2_Clock:
		clock_source = NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS;
		break;
	case NI_660x_Timebase_3_Clock:
		clock_source = NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS;
		break;
	case NI_660x_Logic_Low_Clock:
		clock_source = NI_GPCT_LOGIC_LOW_CLOCK_SRC_BITS;
		break;
	case NI_660x_Source_Pin_i_Clock:
		clock_source = NI_GPCT_SOURCE_PIN_i_CLOCK_SRC_BITS;
		break;
	case NI_660x_Next_Gate_Clock:
		clock_source = NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS;
		break;
	case NI_660x_Next_TC_Clock:
		clock_source = NI_GPCT_NEXT_TC_CLOCK_SRC_BITS;
		break;
	default:
		for (i = 0; i <= ni_660x_max_rtsi_channel; ++i) {
			if (input_select == NI_660x_RTSI_Clock(i)) {
				clock_source = NI_GPCT_RTSI_CLOCK_SRC_BITS(i);
				break;
			}
		}
		if (i <= ni_660x_max_rtsi_channel)
			break;
		for (i = 0; i <= ni_660x_max_source_pin; ++i) {
			if (input_select == NI_660x_Source_Pin_Clock(i)) {
				clock_source =
					NI_GPCT_SOURCE_PIN_CLOCK_SRC_BITS(i);
				break;
			}
		}
		if (i <= ni_660x_max_source_pin)
			break;
		BUG();
		break;
	}
	clock_source |= ni_tio_clock_src_modifiers(counter);
	return clock_source;
}

static unsigned int ni_tio_generic_clock_src_select(const struct ni_gpct *counter)
{
	switch (counter->counter_dev->variant) {
	case ni_gpct_variant_e_series:
	case ni_gpct_variant_m_series:
		return ni_m_series_clock_src_select(counter);
		break;
	case ni_gpct_variant_660x:
		return ni_660x_clock_src_select(counter);
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static uint64_t ni_tio_clock_period_ps(const struct ni_gpct *counter,
				       unsigned int generic_clock_source)
{
	uint64_t clock_period_ps;

	switch (generic_clock_source & NI_GPCT_CLOCK_SRC_SELECT_MASK) {
	case NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS:
		clock_period_ps = 50000;
		break;
	case NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS:
		clock_period_ps = 10000000;
		break;
	case NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS:
		clock_period_ps = 12500;
		break;
	case NI_GPCT_PXI10_CLOCK_SRC_BITS:
		clock_period_ps = 100000;
		break;
	default:
		/* Clock period is specified by user with prescaling
		   already taken into account. */
		return counter->clock_period_ps;
		break;
	}

	switch (generic_clock_source & NI_GPCT_PRESCALE_MODE_CLOCK_SRC_MASK) {
	case NI_GPCT_NO_PRESCALE_CLOCK_SRC_BITS:
		break;
	case NI_GPCT_PRESCALE_X2_CLOCK_SRC_BITS:
		clock_period_ps *= 2;
		break;
	case NI_GPCT_PRESCALE_X8_CLOCK_SRC_BITS:
		clock_period_ps *= 8;
		break;
	default:
		BUG();
		break;
	}
	return clock_period_ps;
}

static void ni_tio_get_clock_src(struct ni_gpct *counter,
				 unsigned int * clock_source,
				 unsigned int * period_ns)
{
	static const unsigned int pico_per_nano = 1000;
	uint64_t temp64;

	*clock_source = ni_tio_generic_clock_src_select(counter);
	temp64 = ni_tio_clock_period_ps(counter, *clock_source);
	do_div(temp64, pico_per_nano);
	*period_ns = temp64;
}

static void ni_tio_set_first_gate_modifiers(struct ni_gpct *counter,
					    lsampl_t gate_source)
{
	const unsigned int mode_mask = Gi_Gate_Polarity_Bit | Gi_Gating_Mode_Mask;
	unsigned int mode_values = 0;

	if (gate_source & CR_INVERT) {
		mode_values |= Gi_Gate_Polarity_Bit;
	}
	if (gate_source & CR_EDGE) {
		mode_values |= Gi_Rising_Edge_Gating_Bits;
	} else {
		mode_values |= Gi_Level_Gating_Bits;
	}
	ni_tio_set_bits(counter, NITIO_Gi_Mode_Reg(counter->counter_index),
		mode_mask, mode_values);
}

static int ni_660x_set_first_gate(struct ni_gpct *counter, lsampl_t gate_source)
{
	const unsigned int selected_gate = CR_CHAN(gate_source);
	/* Bits of selected_gate that may be meaningful to
	   input select register */
	const unsigned int selected_gate_mask = 0x1f;
	unsigned ni_660x_gate_select;
	unsigned i;

	switch (selected_gate) {
	case NI_GPCT_NEXT_SOURCE_GATE_SELECT:
		ni_660x_gate_select = NI_660x_Next_SRC_Gate_Select;
		break;
	case NI_GPCT_NEXT_OUT_GATE_SELECT:
	case NI_GPCT_LOGIC_LOW_GATE_SELECT:
	case NI_GPCT_SOURCE_PIN_i_GATE_SELECT:
	case NI_GPCT_GATE_PIN_i_GATE_SELECT:
		ni_660x_gate_select = selected_gate & selected_gate_mask;
		break;
	default:
		for (i = 0; i <= ni_660x_max_rtsi_channel; ++i) {
			if (selected_gate == NI_GPCT_RTSI_GATE_SELECT(i)) {
				ni_660x_gate_select =
					selected_gate & selected_gate_mask;
				break;
			}
		}
		if (i <= ni_660x_max_rtsi_channel)
			break;
		for (i = 0; i <= ni_660x_max_gate_pin; ++i) {
			if (selected_gate == NI_GPCT_GATE_PIN_GATE_SELECT(i)) {
				ni_660x_gate_select =
					selected_gate & selected_gate_mask;
				break;
			}
		}
		if (i <= ni_660x_max_gate_pin)
			break;
		return -EINVAL;
		break;
	}
	ni_tio_set_bits(counter,
		NITIO_Gi_Input_Select_Reg(counter->counter_index),
		Gi_Gate_Select_Mask, Gi_Gate_Select_Bits(ni_660x_gate_select));
	return 0;
}

static int ni_m_series_set_first_gate(struct ni_gpct *counter,
				      lsampl_t gate_source)
{
	const unsigned int selected_gate = CR_CHAN(gate_source);
	/* bits of selected_gate that may be meaningful to input select register */
	const unsigned int selected_gate_mask = 0x1f;
	unsigned int i, ni_m_series_gate_select;

	switch (selected_gate) {
	case NI_GPCT_TIMESTAMP_MUX_GATE_SELECT:
	case NI_GPCT_AI_START2_GATE_SELECT:
	case NI_GPCT_PXI_STAR_TRIGGER_GATE_SELECT:
	case NI_GPCT_NEXT_OUT_GATE_SELECT:
	case NI_GPCT_AI_START1_GATE_SELECT:
	case NI_GPCT_NEXT_SOURCE_GATE_SELECT:
	case NI_GPCT_ANALOG_TRIGGER_OUT_GATE_SELECT:
	case NI_GPCT_LOGIC_LOW_GATE_SELECT:
		ni_m_series_gate_select = selected_gate & selected_gate_mask;
		break;
	default:
		for (i = 0; i <= ni_m_series_max_rtsi_channel; ++i) {
			if (selected_gate == NI_GPCT_RTSI_GATE_SELECT(i)) {
				ni_m_series_gate_select =
					selected_gate & selected_gate_mask;
				break;
			}
		}
		if (i <= ni_m_series_max_rtsi_channel)
			break;
		for (i = 0; i <= ni_m_series_max_pfi_channel; ++i) {
			if (selected_gate == NI_GPCT_PFI_GATE_SELECT(i)) {
				ni_m_series_gate_select =
					selected_gate & selected_gate_mask;
				break;
			}
		}
		if (i <= ni_m_series_max_pfi_channel)
			break;
		return -EINVAL;
		break;
	}
	ni_tio_set_bits(counter,
		NITIO_Gi_Input_Select_Reg(counter->counter_index),
		Gi_Gate_Select_Mask,
		Gi_Gate_Select_Bits(ni_m_series_gate_select));
	return 0;
}

static int ni_660x_set_second_gate(struct ni_gpct *counter,
				   lsampl_t gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned int second_gate_reg =
		NITIO_Gi_Second_Gate_Reg(counter->counter_index);
	const unsigned int selected_second_gate = CR_CHAN(gate_source);
	/* bits of second_gate that may be meaningful to second gate register */
	static const unsigned int selected_second_gate_mask = 0x1f;
	unsigned int i, ni_660x_second_gate_select;

	switch (selected_second_gate) {
	case NI_GPCT_SOURCE_PIN_i_GATE_SELECT:
	case NI_GPCT_UP_DOWN_PIN_i_GATE_SELECT:
	case NI_GPCT_SELECTED_GATE_GATE_SELECT:
	case NI_GPCT_NEXT_OUT_GATE_SELECT:
	case NI_GPCT_LOGIC_LOW_GATE_SELECT:
		ni_660x_second_gate_select =
			selected_second_gate & selected_second_gate_mask;
		break;
	case NI_GPCT_NEXT_SOURCE_GATE_SELECT:
		ni_660x_second_gate_select =
			NI_660x_Next_SRC_Second_Gate_Select;
		break;
	default:
		for (i = 0; i <= ni_660x_max_rtsi_channel; ++i) {
			if (selected_second_gate == NI_GPCT_RTSI_GATE_SELECT(i)) {
				ni_660x_second_gate_select =
					selected_second_gate &
					selected_second_gate_mask;
				break;
			}
		}
		if (i <= ni_660x_max_rtsi_channel)
			break;
		for (i = 0; i <= ni_660x_max_up_down_pin; ++i) {
			if (selected_second_gate ==
				NI_GPCT_UP_DOWN_PIN_GATE_SELECT(i)) {
				ni_660x_second_gate_select =
					selected_second_gate &
					selected_second_gate_mask;
				break;
			}
		}
		if (i <= ni_660x_max_up_down_pin)
			break;
		return -EINVAL;
		break;
	};
	counter_dev->regs[second_gate_reg] |= Gi_Second_Gate_Mode_Bit;
	counter_dev->regs[second_gate_reg] &= ~Gi_Second_Gate_Select_Mask;
	counter_dev->regs[second_gate_reg] |=
		Gi_Second_Gate_Select_Bits(ni_660x_second_gate_select);
	write_register(counter, counter_dev->regs[second_gate_reg],
		second_gate_reg);
	return 0;
}

static int ni_m_series_set_second_gate(struct ni_gpct *counter,
				       lsampl_t gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned int second_gate_reg =
		NITIO_Gi_Second_Gate_Reg(counter->counter_index);
	const unsigned int selected_second_gate = CR_CHAN(gate_source);
	/* Bits of second_gate that may be meaningful to second gate register */
	static const unsigned int selected_second_gate_mask = 0x1f;
	unsigned int ni_m_series_second_gate_select;

	/* FIXME: We don't know what the m-series second gate codes
	   are, so we'll just pass the bits through for now. */
	switch (selected_second_gate) {
	default:
		ni_m_series_second_gate_select =
			selected_second_gate & selected_second_gate_mask;
		break;
	};
	counter_dev->regs[second_gate_reg] |= Gi_Second_Gate_Mode_Bit;
	counter_dev->regs[second_gate_reg] &= ~Gi_Second_Gate_Select_Mask;
	counter_dev->regs[second_gate_reg] |=
		Gi_Second_Gate_Select_Bits(ni_m_series_second_gate_select);
	write_register(counter, counter_dev->regs[second_gate_reg],
		second_gate_reg);
	return 0;
}

static int ni_tio_set_gate_src(struct ni_gpct *counter,
			       unsigned int gate_index, lsampl_t gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned int second_gate_reg =
		NITIO_Gi_Second_Gate_Reg(counter->counter_index);

	switch (gate_index) {
	case 0:
		if (CR_CHAN(gate_source) == NI_GPCT_DISABLED_GATE_SELECT) {
			ni_tio_set_bits(counter,
				NITIO_Gi_Mode_Reg(counter->counter_index),
				Gi_Gating_Mode_Mask, Gi_Gating_Disabled_Bits);
			return 0;
		}
		ni_tio_set_first_gate_modifiers(counter, gate_source);
		switch (counter_dev->variant) {
		case ni_gpct_variant_e_series:
		case ni_gpct_variant_m_series:
			return ni_m_series_set_first_gate(counter, gate_source);
			break;
		case ni_gpct_variant_660x:
			return ni_660x_set_first_gate(counter, gate_source);
			break;
		default:
			BUG();
			break;
		}
		break;
	case 1:
		if (ni_tio_second_gate_registers_present(counter_dev) == 0)
			return -EINVAL;
		if (CR_CHAN(gate_source) == NI_GPCT_DISABLED_GATE_SELECT) {
			counter_dev->regs[second_gate_reg] &=
				~Gi_Second_Gate_Mode_Bit;
			write_register(counter,
				counter_dev->regs[second_gate_reg],
				second_gate_reg);
			return 0;
		}
		if (gate_source & CR_INVERT) {
			counter_dev->regs[second_gate_reg] |=
				Gi_Second_Gate_Polarity_Bit;
		} else {
			counter_dev->regs[second_gate_reg] &=
				~Gi_Second_Gate_Polarity_Bit;
		}
		switch (counter_dev->variant) {
		case ni_gpct_variant_m_series:
			return ni_m_series_set_second_gate(counter,
				gate_source);
			break;
		case ni_gpct_variant_660x:
			return ni_660x_set_second_gate(counter, gate_source);
			break;
		default:
			BUG();
			break;
		}
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

static int ni_tio_set_other_src(struct ni_gpct *counter,
				unsigned int index, unsigned int source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;

	if (counter_dev->variant == ni_gpct_variant_m_series) {
		unsigned int abz_reg, shift, mask;

		abz_reg = NITIO_Gi_ABZ_Reg(counter->counter_index);
		switch (index) {
		case NI_GPCT_SOURCE_ENCODER_A:
			shift = 10;
			break;
		case NI_GPCT_SOURCE_ENCODER_B:
			shift = 5;
			break;
		case NI_GPCT_SOURCE_ENCODER_Z:
			shift = 0;
			break;
		default:
			return -EINVAL;
			break;
		}
		mask = 0x1f << shift;
		if (source > 0x1f) {
			/* Disable gate */
			source = 0x1f;
		}
		counter_dev->regs[abz_reg] &= ~mask;
		counter_dev->regs[abz_reg] |= (source << shift) & mask;
		write_register(counter, counter_dev->regs[abz_reg], abz_reg);
		return 0;
	}
	return -EINVAL;
}

static unsigned int ni_660x_first_gate_to_generic_gate_source(unsigned int ni_660x_gate_select)
{
	unsigned int i;

	switch (ni_660x_gate_select) {
	case NI_660x_Source_Pin_i_Gate_Select:
		return NI_GPCT_SOURCE_PIN_i_GATE_SELECT;
		break;
	case NI_660x_Gate_Pin_i_Gate_Select:
		return NI_GPCT_GATE_PIN_i_GATE_SELECT;
		break;
	case NI_660x_Next_SRC_Gate_Select:
		return NI_GPCT_NEXT_SOURCE_GATE_SELECT;
		break;
	case NI_660x_Next_Out_Gate_Select:
		return NI_GPCT_NEXT_OUT_GATE_SELECT;
		break;
	case NI_660x_Logic_Low_Gate_Select:
		return NI_GPCT_LOGIC_LOW_GATE_SELECT;
		break;
	default:
		for (i = 0; i <= ni_660x_max_rtsi_channel; ++i) {
			if (ni_660x_gate_select == NI_660x_RTSI_Gate_Select(i)) {
				return NI_GPCT_RTSI_GATE_SELECT(i);
				break;
			}
		}
		if (i <= ni_660x_max_rtsi_channel)
			break;
		for (i = 0; i <= ni_660x_max_gate_pin; ++i) {
			if (ni_660x_gate_select ==
				NI_660x_Gate_Pin_Gate_Select(i)) {
				return NI_GPCT_GATE_PIN_GATE_SELECT(i);
				break;
			}
		}
		if (i <= ni_660x_max_gate_pin)
			break;
		BUG();
		break;
	}
	return 0;
}

static unsigned int ni_m_series_first_gate_to_generic_gate_source(unsigned int
	ni_m_series_gate_select)
{
	unsigned int i;

	switch (ni_m_series_gate_select) {
	case NI_M_Series_Timestamp_Mux_Gate_Select:
		return NI_GPCT_TIMESTAMP_MUX_GATE_SELECT;
		break;
	case NI_M_Series_AI_START2_Gate_Select:
		return NI_GPCT_AI_START2_GATE_SELECT;
		break;
	case NI_M_Series_PXI_Star_Trigger_Gate_Select:
		return NI_GPCT_PXI_STAR_TRIGGER_GATE_SELECT;
		break;
	case NI_M_Series_Next_Out_Gate_Select:
		return NI_GPCT_NEXT_OUT_GATE_SELECT;
		break;
	case NI_M_Series_AI_START1_Gate_Select:
		return NI_GPCT_AI_START1_GATE_SELECT;
		break;
	case NI_M_Series_Next_SRC_Gate_Select:
		return NI_GPCT_NEXT_SOURCE_GATE_SELECT;
		break;
	case NI_M_Series_Analog_Trigger_Out_Gate_Select:
		return NI_GPCT_ANALOG_TRIGGER_OUT_GATE_SELECT;
		break;
	case NI_M_Series_Logic_Low_Gate_Select:
		return NI_GPCT_LOGIC_LOW_GATE_SELECT;
		break;
	default:
		for (i = 0; i <= ni_m_series_max_rtsi_channel; ++i) {
			if (ni_m_series_gate_select ==
				NI_M_Series_RTSI_Gate_Select(i)) {
				return NI_GPCT_RTSI_GATE_SELECT(i);
				break;
			}
		}
		if (i <= ni_m_series_max_rtsi_channel)
			break;
		for (i = 0; i <= ni_m_series_max_pfi_channel; ++i) {
			if (ni_m_series_gate_select ==
				NI_M_Series_PFI_Gate_Select(i)) {
				return NI_GPCT_PFI_GATE_SELECT(i);
				break;
			}
		}
		if (i <= ni_m_series_max_pfi_channel)
			break;
		BUG();
		break;
	}
	return 0;
}

static unsigned int ni_660x_second_gate_to_generic_gate_source(unsigned int
	ni_660x_gate_select)
{
	unsigned int i;

	switch (ni_660x_gate_select) {
	case NI_660x_Source_Pin_i_Second_Gate_Select:
		return NI_GPCT_SOURCE_PIN_i_GATE_SELECT;
		break;
	case NI_660x_Up_Down_Pin_i_Second_Gate_Select:
		return NI_GPCT_UP_DOWN_PIN_i_GATE_SELECT;
		break;
	case NI_660x_Next_SRC_Second_Gate_Select:
		return NI_GPCT_NEXT_SOURCE_GATE_SELECT;
		break;
	case NI_660x_Next_Out_Second_Gate_Select:
		return NI_GPCT_NEXT_OUT_GATE_SELECT;
		break;
	case NI_660x_Selected_Gate_Second_Gate_Select:
		return NI_GPCT_SELECTED_GATE_GATE_SELECT;
		break;
	case NI_660x_Logic_Low_Second_Gate_Select:
		return NI_GPCT_LOGIC_LOW_GATE_SELECT;
		break;
	default:
		for (i = 0; i <= ni_660x_max_rtsi_channel; ++i) {
			if (ni_660x_gate_select ==
				NI_660x_RTSI_Second_Gate_Select(i)) {
				return NI_GPCT_RTSI_GATE_SELECT(i);
				break;
			}
		}
		if (i <= ni_660x_max_rtsi_channel)
			break;
		for (i = 0; i <= ni_660x_max_up_down_pin; ++i) {
			if (ni_660x_gate_select ==
				NI_660x_Up_Down_Pin_Second_Gate_Select(i)) {
				return NI_GPCT_UP_DOWN_PIN_GATE_SELECT(i);
				break;
			}
		}
		if (i <= ni_660x_max_up_down_pin)
			break;
		BUG();
		break;
	}
	return 0;
}

static unsigned int ni_m_series_second_gate_to_generic_gate_source(unsigned int
	ni_m_series_gate_select)
{
	/* FIXME: the second gate sources for the m series are
	   undocumented, so we just return the raw bits for now. */
	switch (ni_m_series_gate_select) {
	default:
		return ni_m_series_gate_select;
		break;
	}
	return 0;
};

static int ni_tio_get_gate_src(struct ni_gpct *counter,
			       unsigned int gate_index,
			       unsigned int * gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned int mode_bits = ni_tio_get_soft_copy(counter,
		NITIO_Gi_Mode_Reg(counter->counter_index));
	const unsigned int second_gate_reg =
		NITIO_Gi_Second_Gate_Reg(counter->counter_index);
	unsigned int gate_select_bits;

	switch (gate_index) {
	case 0:
		if ((mode_bits & Gi_Gating_Mode_Mask) ==
			Gi_Gating_Disabled_Bits) {
			*gate_source = NI_GPCT_DISABLED_GATE_SELECT;
			return 0;
		} else {
			gate_select_bits =
				(ni_tio_get_soft_copy(counter,
					NITIO_Gi_Input_Select_Reg(counter->
						counter_index)) &
				Gi_Gate_Select_Mask) >> Gi_Gate_Select_Shift;
		}
		switch (counter_dev->variant) {
		case ni_gpct_variant_e_series:
		case ni_gpct_variant_m_series:
			*gate_source =
				ni_m_series_first_gate_to_generic_gate_source
				(gate_select_bits);
			break;
		case ni_gpct_variant_660x:
			*gate_source =
				ni_660x_first_gate_to_generic_gate_source
				(gate_select_bits);
			break;
		default:
			BUG();
			break;
		}
		if (mode_bits & Gi_Gate_Polarity_Bit) {
			*gate_source |= CR_INVERT;
		}
		if ((mode_bits & Gi_Gating_Mode_Mask) != Gi_Level_Gating_Bits) {
			*gate_source |= CR_EDGE;
		}
		break;
	case 1:
		if ((mode_bits & Gi_Gating_Mode_Mask) == Gi_Gating_Disabled_Bits
			|| (counter_dev->
				regs[second_gate_reg] & Gi_Second_Gate_Mode_Bit)
			== 0) {
			*gate_source = NI_GPCT_DISABLED_GATE_SELECT;
			return 0;
		} else {
			gate_select_bits =
				(counter_dev->
				regs[second_gate_reg] &
				Gi_Second_Gate_Select_Mask) >>
				Gi_Second_Gate_Select_Shift;
		}
		switch (counter_dev->variant) {
		case ni_gpct_variant_e_series:
		case ni_gpct_variant_m_series:
			*gate_source =
				ni_m_series_second_gate_to_generic_gate_source
				(gate_select_bits);
			break;
		case ni_gpct_variant_660x:
			*gate_source =
				ni_660x_second_gate_to_generic_gate_source
				(gate_select_bits);
			break;
		default:
			BUG();
			break;
		}
		if (counter_dev->
			regs[second_gate_reg] & Gi_Second_Gate_Polarity_Bit) {
			*gate_source |= CR_INVERT;
		}
		/* Second gate can't have edge/level mode set independently */
		if ((mode_bits & Gi_Gating_Mode_Mask) != Gi_Level_Gating_Bits) {
			*gate_source |= CR_EDGE;
		}
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

int a4l_ni_tio_insn_config(struct ni_gpct *counter, struct a4l_kernel_instruction *insn)
{
	unsigned int *data = (unsigned int *)insn->data;

	switch (data[0]) {
	case A4L_INSN_CONFIG_SET_COUNTER_MODE:
		return ni_tio_set_counter_mode(counter, data[1]);
		break;
	case A4L_INSN_CONFIG_ARM:
		return ni_tio_arm(counter, 1, data[1]);
		break;
	case A4L_INSN_CONFIG_DISARM:
		ni_tio_arm(counter, 0, 0);
		return 0;
		break;
	case A4L_INSN_CONFIG_GET_COUNTER_STATUS:
		data[1] = ni_tio_counter_status(counter);
		data[2] = counter_status_mask;
		return 0;
		break;
	case A4L_INSN_CONFIG_SET_CLOCK_SRC:
		return ni_tio_set_clock_src(counter, data[1], data[2]);
		break;
	case A4L_INSN_CONFIG_GET_CLOCK_SRC:
		ni_tio_get_clock_src(counter, &data[1], &data[2]);
		return 0;
		break;
	case A4L_INSN_CONFIG_SET_GATE_SRC:
		return ni_tio_set_gate_src(counter, data[1], data[2]);
		break;
	case A4L_INSN_CONFIG_GET_GATE_SRC:
		return ni_tio_get_gate_src(counter, data[1], &data[2]);
		break;
	case A4L_INSN_CONFIG_SET_OTHER_SRC:
		return ni_tio_set_other_src(counter, data[1], data[2]);
		break;
	case A4L_INSN_CONFIG_RESET:
		ni_tio_reset_count_and_disarm(counter);
		return 0;
		break;
	default:
		break;
	}
	return -EINVAL;
}

int a4l_ni_tio_rinsn(struct ni_gpct *counter, struct a4l_kernel_instruction *insn)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned int channel = CR_CHAN(insn->chan_desc);
	unsigned int first_read;
	unsigned int second_read;
	unsigned int correct_read;

	uint32_t *data = (uint32_t *)insn->data;

	if (insn->data_size != sizeof(uint32_t))
		return -EINVAL;

	switch (channel) {
	case 0:
		ni_tio_set_bits(counter,
			NITIO_Gi_Command_Reg(counter->counter_index),
			Gi_Save_Trace_Bit, 0);
		ni_tio_set_bits(counter,
			NITIO_Gi_Command_Reg(counter->counter_index),
			Gi_Save_Trace_Bit, Gi_Save_Trace_Bit);
		/* The count doesn't get latched until the next clock
		   edge, so it is possible the count may change (once)
		   while we are reading.  Since the read of the
		   SW_Save_Reg isn't atomic (apparently even when it's a
		   32 bit register according to 660x docs), we need to
		   read twice and make sure the reading hasn't changed.
		   If it has, a third read will be correct since the
		   count value will definitely have latched by then. */
		first_read =
			read_register(counter,
			NITIO_Gi_SW_Save_Reg(counter->counter_index));
		second_read =
			read_register(counter,
			NITIO_Gi_SW_Save_Reg(counter->counter_index));
		if (first_read != second_read)
			correct_read =
				read_register(counter,
				NITIO_Gi_SW_Save_Reg(counter->counter_index));
		else
			correct_read = first_read;
		data[0] = correct_read;
		return 0;
		break;
	case 1:
		data[0] = counter_dev->regs
			[NITIO_Gi_LoadA_Reg(counter->counter_index)];
		break;
	case 2:
		data[0] = counter_dev->regs
			[NITIO_Gi_LoadB_Reg(counter->counter_index)];
		break;
	};

	return 0;
}

static unsigned int ni_tio_next_load_register(struct ni_gpct *counter)
{
	const unsigned int bits = read_register(counter,
		NITIO_Gxx_Status_Reg(counter->counter_index));

	if (bits & Gi_Next_Load_Source_Bit(counter->counter_index)) {
		return NITIO_Gi_LoadB_Reg(counter->counter_index);
	} else {
		return NITIO_Gi_LoadA_Reg(counter->counter_index);
	}
}

int a4l_ni_tio_winsn(struct ni_gpct *counter, struct a4l_kernel_instruction *insn)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned int channel = CR_CHAN(insn->chan_desc);
	unsigned int load_reg;

	uint32_t *data = (uint32_t *)insn->data;

	if (insn->data_size != sizeof(uint32_t))
		return -EINVAL;

	switch (channel) {
	case 0:
		/* Unsafe if counter is armed.  Should probably check
		   status and return -EBUSY if armed. */
		/* Don't disturb load source select, just use
		   whichever load register is already selected. */
		load_reg = ni_tio_next_load_register(counter);
		write_register(counter, data[0], load_reg);
		ni_tio_set_bits_transient(counter,
			NITIO_Gi_Command_Reg(counter->counter_index), 0, 0,
			Gi_Load_Bit);
		/* Restore state of load reg to whatever the user set
		   last set it to */
		write_register(counter, counter_dev->regs[load_reg], load_reg);
		break;
	case 1:
		counter_dev->regs[NITIO_Gi_LoadA_Reg(counter->counter_index)] =
			data[0];
		write_register(counter, data[0],
			NITIO_Gi_LoadA_Reg(counter->counter_index));
		break;
	case 2:
		counter_dev->regs[NITIO_Gi_LoadB_Reg(counter->counter_index)] =
			data[0];
		write_register(counter, data[0],
			NITIO_Gi_LoadB_Reg(counter->counter_index));
		break;
	default:
		return -EINVAL;
		break;
	}

	return 0;
}

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

static void ni_tio_configure_dma(struct ni_gpct *counter,
				 short enable, short read_not_write)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	unsigned int input_select_bits = 0;

	if (enable) {
		if (read_not_write) {
			input_select_bits |= Gi_Read_Acknowledges_Irq;
		} else {
			input_select_bits |= Gi_Write_Acknowledges_Irq;
		}
	}
	ni_tio_set_bits(counter,
		NITIO_Gi_Input_Select_Reg(counter->counter_index),
		Gi_Read_Acknowledges_Irq | Gi_Write_Acknowledges_Irq,
		input_select_bits);
	switch (counter_dev->variant) {
	case ni_gpct_variant_e_series:
		break;
	case ni_gpct_variant_m_series:
	case ni_gpct_variant_660x:
		{
			unsigned gi_dma_config_bits = 0;

			if (enable) {
				gi_dma_config_bits |= Gi_DMA_Enable_Bit;
				gi_dma_config_bits |= Gi_DMA_Int_Bit;
			}
			if (read_not_write == 0) {
				gi_dma_config_bits |= Gi_DMA_Write_Bit;
			}
			ni_tio_set_bits(counter,
				NITIO_Gi_DMA_Config_Reg(counter->counter_index),
				Gi_DMA_Enable_Bit | Gi_DMA_Int_Bit |
				Gi_DMA_Write_Bit, gi_dma_config_bits);
		}
		break;
	}
}

/* TODO: a4l_ni_tio_input_inttrig is left unused because the trigger
   callback cannot be changed at run time */
int a4l_ni_tio_input_inttrig(struct ni_gpct *counter, lsampl_t trignum)
{
	unsigned long flags;
	int retval = 0;

	BUG_ON(counter == NULL);
	if (trignum != 0)
		return -EINVAL;

	rtdm_lock_get_irqsave(&counter->lock, flags);
	if (counter->mite_chan)
		a4l_mite_dma_arm(counter->mite_chan);
	else
		retval = -EIO;
	rtdm_lock_put_irqrestore(&counter->lock, flags);
	if (retval < 0)
		return retval;
	retval = ni_tio_arm(counter, 1, NI_GPCT_ARM_IMMEDIATE);

	/* TODO: disable trigger until a command is recorded.
	   Null trig at beginning prevent ao start trigger from executing
	   more than once per command (and doing things like trying to
	   allocate the ao dma channel multiple times) */

	return retval;
}

static int ni_tio_input_cmd(struct ni_gpct *counter, struct a4l_cmd_desc *cmd)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	int retval = 0;

	counter->mite_chan->dir = A4L_INPUT;
	switch (counter_dev->variant) {
	case ni_gpct_variant_m_series:
	case ni_gpct_variant_660x:
		a4l_mite_prep_dma(counter->mite_chan, 32, 32);
		break;
	case ni_gpct_variant_e_series:
		a4l_mite_prep_dma(counter->mite_chan, 16, 32);
		break;
	default:
		BUG();
		break;
	}
	ni_tio_set_bits(counter, NITIO_Gi_Command_Reg(counter->counter_index),
		Gi_Save_Trace_Bit, 0);
	ni_tio_configure_dma(counter, 1, 1);
	switch (cmd->start_src) {
	case TRIG_NOW:
		a4l_mite_dma_arm(counter->mite_chan);
		retval = ni_tio_arm(counter, 1, NI_GPCT_ARM_IMMEDIATE);
		break;
	case TRIG_INT:
		break;
	case TRIG_EXT:
		a4l_mite_dma_arm(counter->mite_chan);
		retval = ni_tio_arm(counter, 1, cmd->start_arg);
	case TRIG_OTHER:
		a4l_mite_dma_arm(counter->mite_chan);
		break;
	default:
		BUG();
		break;
	}
	return retval;
}

static int ni_tio_output_cmd(struct ni_gpct *counter, struct a4l_cmd_desc *cmd)
{
	__a4l_err("ni_tio: output commands not yet implemented.\n");
	return -ENOTSUPP;
}

static int ni_tio_cmd_setup(struct ni_gpct *counter, struct a4l_cmd_desc *cmd)
{
	int retval = 0, set_gate_source = 0;
	unsigned int gate_source;

	if (cmd->scan_begin_src == TRIG_EXT) {
		set_gate_source = 1;
		gate_source = cmd->scan_begin_arg;
	} else if (cmd->convert_src == TRIG_EXT) {
		set_gate_source = 1;
		gate_source = cmd->convert_arg;
	}
	if (set_gate_source) {
		retval = ni_tio_set_gate_src(counter, 0, gate_source);
	}
	if (cmd->flags & TRIG_WAKE_EOS) {
		ni_tio_set_bits(counter,
			NITIO_Gi_Interrupt_Enable_Reg(counter->counter_index),
			Gi_Gate_Interrupt_Enable_Bit(counter->counter_index),
			Gi_Gate_Interrupt_Enable_Bit(counter->counter_index));
	}
	return retval;
}

int a4l_ni_tio_cmd(struct ni_gpct *counter, struct a4l_cmd_desc *cmd)
{
	int retval = 0;
	unsigned long flags;

	rtdm_lock_get_irqsave(&counter->lock, flags);
	if (counter->mite_chan == NULL) {
		__a4l_err("a4l_ni_tio_cmd: commands only supported with DMA."
			     " Interrupt-driven commands not yet implemented.\n");
		retval = -EIO;
	} else {
		retval = ni_tio_cmd_setup(counter, cmd);
		if (retval == 0) {
			if (cmd->flags & A4L_CMD_WRITE) {
				retval = ni_tio_output_cmd(counter, cmd);
			} else {
				retval = ni_tio_input_cmd(counter, cmd);
			}
		}
	}
	rtdm_lock_put_irqrestore(&counter->lock, flags);
	return retval;
}

struct a4l_cmd_desc a4l_ni_tio_cmd_mask = {
	.idx_subd = 0,
	.start_src = TRIG_NOW | TRIG_INT | TRIG_OTHER | TRIG_EXT,
	.scan_begin_src = TRIG_FOLLOW | TRIG_EXT | TRIG_OTHER,
	.convert_src = TRIG_NOW | TRIG_EXT | TRIG_OTHER,
	.scan_end_src = TRIG_COUNT,
	.stop_src = TRIG_NONE,
};

int a4l_ni_tio_cmdtest(struct ni_gpct *counter, struct a4l_cmd_desc *cmd)
{
	/* Make sure trigger sources are trivially valid */

	if ((cmd->start_src & TRIG_EXT) != 0 &&
	    ni_tio_counting_mode_registers_present(counter->counter_dev) == 0)
		return -EINVAL;

	/* Make sure trigger sources are mutually compatible */

	if (cmd->convert_src != TRIG_NOW && cmd->scan_begin_src != TRIG_FOLLOW)
		return -EINVAL;

	/* Make sure arguments are trivially compatible */

	if (cmd->start_src != TRIG_EXT) {
		if (cmd->start_arg != 0) {
			return -EINVAL;
		}
	}

	if (cmd->scan_begin_src != TRIG_EXT) {
		if (cmd->scan_begin_arg) {
			return -EINVAL;
		}
	}

	if (cmd->convert_src != TRIG_EXT) {
		if (cmd->convert_arg) {
			return -EINVAL;
		}
	}

	if (cmd->scan_end_arg != cmd->nb_chan) {
		return -EINVAL;
	}

	if (cmd->stop_src == TRIG_NONE) {
		if (cmd->stop_arg != 0) {
			return -EINVAL;
		}
	}

	return 0;
}

int a4l_ni_tio_cancel(struct ni_gpct *counter)
{
	unsigned long flags;

	ni_tio_arm(counter, 0, 0);
	rtdm_lock_get_irqsave(&counter->lock, flags);
	if (counter->mite_chan) {
		a4l_mite_dma_disarm(counter->mite_chan);
	}
	rtdm_lock_put_irqrestore(&counter->lock, flags);
	ni_tio_configure_dma(counter, 0, 0);

	ni_tio_set_bits(counter,
		NITIO_Gi_Interrupt_Enable_Reg(counter->counter_index),
		Gi_Gate_Interrupt_Enable_Bit(counter->counter_index), 0x0);
	return 0;
}

/*  During buffered input counter operation for e-series, the gate
   interrupt is acked automatically by the dma controller, due to the
   Gi_Read/Write_Acknowledges_IRQ bits in the input select
   register. */
static int should_ack_gate(struct ni_gpct *counter)
{
	unsigned long flags;
	int retval = 0;

	switch (counter->counter_dev->variant) {
	case ni_gpct_variant_m_series:
	case ni_gpct_variant_660x:
		/* Not sure if 660x really supports gate interrupts
		   (the bits are not listed in register-level manual) */
		return 1;
		break;
	case ni_gpct_variant_e_series:
		rtdm_lock_get_irqsave(&counter->lock, flags);
		{
			if (counter->mite_chan == NULL ||
				counter->mite_chan->dir != A4L_INPUT ||
				(a4l_mite_done(counter->mite_chan))) {
				retval = 1;
			}
		}
		rtdm_lock_put_irqrestore(&counter->lock, flags);
		break;
	}
	return retval;
}

void a4l_ni_tio_acknowledge_and_confirm(struct ni_gpct *counter,
				    int *gate_error,
				    int *tc_error,
				    int *perm_stale_data, int *stale_data)
{
	const unsigned short gxx_status = read_register(counter,
		NITIO_Gxx_Status_Reg(counter->counter_index));
	const unsigned short gi_status = read_register(counter,
		NITIO_Gi_Status_Reg(counter->counter_index));
	unsigned ack = 0;

	if (gate_error)
		*gate_error = 0;
	if (tc_error)
		*tc_error = 0;
	if (perm_stale_data)
		*perm_stale_data = 0;
	if (stale_data)
		*stale_data = 0;

	if (gxx_status & Gi_Gate_Error_Bit(counter->counter_index)) {
		ack |= Gi_Gate_Error_Confirm_Bit(counter->counter_index);
		if (gate_error) {
			/* 660x don't support automatic
			   acknowledgement of gate interrupt via dma
			   read/write and report bogus gate errors */
			if (counter->counter_dev->variant !=
				ni_gpct_variant_660x) {
				*gate_error = 1;
			}
		}
	}
	if (gxx_status & Gi_TC_Error_Bit(counter->counter_index)) {
		ack |= Gi_TC_Error_Confirm_Bit(counter->counter_index);
		if (tc_error)
			*tc_error = 1;
	}
	if (gi_status & Gi_TC_Bit) {
		ack |= Gi_TC_Interrupt_Ack_Bit;
	}
	if (gi_status & Gi_Gate_Interrupt_Bit) {
		if (should_ack_gate(counter))
			ack |= Gi_Gate_Interrupt_Ack_Bit;
	}
	if (ack)
		write_register(counter, ack,
			NITIO_Gi_Interrupt_Acknowledge_Reg(counter->
				counter_index));
	if (ni_tio_get_soft_copy(counter,
			NITIO_Gi_Mode_Reg(counter->
				counter_index)) & Gi_Loading_On_Gate_Bit) {
		if (gxx_status & Gi_Stale_Data_Bit(counter->counter_index)) {
			if (stale_data)
				*stale_data = 1;
		}
		if (read_register(counter,
				NITIO_Gxx_Joint_Status2_Reg(counter->
					counter_index)) &
			Gi_Permanent_Stale_Bit(counter->counter_index)) {
			__a4l_err("%s: Gi_Permanent_Stale_Data detected.\n",
				    __FUNCTION__);
			if (perm_stale_data)
				*perm_stale_data = 1;
		}
	}
}

/* TODO: to be adapted after a4l_buf_evt review */
void a4l_ni_tio_handle_interrupt(struct ni_gpct *counter, struct a4l_device *dev)
{
	unsigned gpct_mite_status;
	unsigned long flags;
	int gate_error;
	int tc_error;
	int perm_stale_data;
	struct a4l_subdevice *subd =
		a4l_get_subd(dev, NI_GPCT_SUBDEV(counter->counter_index));

	a4l_ni_tio_acknowledge_and_confirm(counter, &gate_error, &tc_error,
		&perm_stale_data, NULL);
	if (gate_error) {
		__a4l_err("%s: Gi_Gate_Error detected.\n", __FUNCTION__);
		a4l_buf_evt(subd, A4L_BUF_ERROR);
	}
	if (perm_stale_data) {
		a4l_buf_evt(subd, A4L_BUF_ERROR);
	}
	switch (counter->counter_dev->variant) {
	case ni_gpct_variant_m_series:
	case ni_gpct_variant_660x:
		if (read_register(counter,
				  NITIO_Gi_DMA_Status_Reg(counter->counter_index))
		    & Gi_DRQ_Error_Bit) {
			__a4l_err("%s: Gi_DRQ_Error detected.\n", __FUNCTION__);
			a4l_buf_evt(subd, A4L_BUF_ERROR);
		}
		break;
	case ni_gpct_variant_e_series:
		break;
	}
	rtdm_lock_get_irqsave(&counter->lock, flags);
	if (counter->mite_chan == NULL) {
		rtdm_lock_put_irqrestore(&counter->lock, flags);
		return;
	}
	gpct_mite_status = a4l_mite_get_status(counter->mite_chan);
	if (gpct_mite_status & CHSR_LINKC) {
		writel(CHOR_CLRLC,
			counter->mite_chan->mite->mite_io_addr +
			MITE_CHOR(counter->mite_chan->channel));
	}
	a4l_mite_sync_input_dma(counter->mite_chan, subd);
	rtdm_lock_put_irqrestore(&counter->lock, flags);
}

void a4l_ni_tio_set_mite_channel(struct ni_gpct *counter,
			     struct mite_channel *mite_chan)
{
	unsigned long flags;

	rtdm_lock_get_irqsave(&counter->lock, flags);
	counter->mite_chan = mite_chan;
	rtdm_lock_put_irqrestore(&counter->lock, flags);
}

#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */

static int __init ni_tio_init_module(void)
{
	return 0;
}

static void __exit ni_tio_cleanup_module(void)
{
}

MODULE_DESCRIPTION("Analogy support for NI general-purpose counters");
MODULE_LICENSE("GPL");

module_init(ni_tio_init_module);
module_exit(ni_tio_cleanup_module);

EXPORT_SYMBOL_GPL(a4l_ni_tio_rinsn);
EXPORT_SYMBOL_GPL(a4l_ni_tio_winsn);
EXPORT_SYMBOL_GPL(a4l_ni_tio_insn_config);
EXPORT_SYMBOL_GPL(a4l_ni_tio_init_counter);
EXPORT_SYMBOL_GPL(a4l_ni_gpct_device_construct);
EXPORT_SYMBOL_GPL(a4l_ni_gpct_device_destroy);

#if (defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE) || \
     defined(CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE_MODULE))

EXPORT_SYMBOL_GPL(a4l_ni_tio_input_inttrig);
EXPORT_SYMBOL_GPL(a4l_ni_tio_cmd);
EXPORT_SYMBOL_GPL(a4l_ni_tio_cmd_mask);
EXPORT_SYMBOL_GPL(a4l_ni_tio_cmdtest);
EXPORT_SYMBOL_GPL(a4l_ni_tio_cancel);
EXPORT_SYMBOL_GPL(a4l_ni_tio_handle_interrupt);
EXPORT_SYMBOL_GPL(a4l_ni_tio_set_mite_channel);
EXPORT_SYMBOL_GPL(a4l_ni_tio_acknowledge_and_confirm);

#endif /* CONFIG_XENO_DRIVERS_ANALOGY_NI_MITE */
