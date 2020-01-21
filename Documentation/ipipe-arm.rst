.. include:: <isonum.txt>

===============================================
Porting the interrupt pipeline to a new ARM SoC
===============================================

:Copyright: |copy| 2014: Gilles Chanteperdrix
:Copyright: |copy| 2018: Philippe Gerum

Purpose
=======

This document is an adaptation of the original article [#f1]_ by
Gilles Chanteperdrix for the Xenomai project, detailing the changes
introduced by the interrupt pipeline into the ARM kernel.

It is aimed at guiding you through the task of porting the I-pipe core
to a new ARM SoC, for interfacing a co-kernel such as Xenomai with
Linux, in order to implement a real-time, dual kernel system.

.. [#f1] "Porting Xenomai dual kernel to a new ARM SoC"
    https://xenomai.org/2014/09/porting-xenomai-dual-kernel-to-a-new-arm-soc/

Terminology
-----------

If you are reading this document, chances are you want to get the
I-pipe to run on an ARM based *board*. Examples of **board**s are
*beagleboard*, *beaglebone*, *raspberry pi*.

This board uses an ARM based *SoC*. Examples of **SoC**s are Atmel
AT91RM9200, Atmel AT91SAM9263, TI OMAP3530, TI OMAP4430, Freescale
IMX53. We use *SoC family* to loosely designate groups of SoCs which
have so many peripherals in common that peripheral support code is
shared between them. For instance, there is an "AT91" family,
including the AT91RM9200 and AT91SAM9263 SoCs, and several others.

This SoC is based on a processor *core* implementing the ARM
instruction set, examples of such **core**s are ARM 926EJ-S,
Intel/Marvell Xscale, Marvell Feroceon, ARM Cortex A8, ARM Cortex A9.

Finally, this processor core implements an ARM *architecture*, sort of
revision of the ARM instruction set. Examples of ARM **architecture**s
are armv4, armv5, armv6 and armv7.

So, for instance, the IGEPv2 *board* uses the TI OMAP3530 *SoC*,
member of the OMAP *SoC family*, based on the ARM Cortex A8 *core*,
implementing the armv7 *architecture*.

.. CAUTION: Starting from kernel 4.14, the I-pipe does not support
   armv4 or armv5 architectures anymore, but only armv6 onward.

Locate the ARM-specific code to port
------------------------------------

Initially, you should identify what are the SoC, processor core and
architecture of the SoC used by your board, then locate the
corresponding SoC and board-specific code.

In order to figure out such information, you can use the Linux kernel
Kconfig and Makefiles in various sub-directories in the Linux kernel
sources. Linux code specific to an ARM based SoC or SoC family X is
located in arch/arm/mach-X or arch/arm/plat-X, some code may also live
in the drivers/ directory, typically in drivers/clocksource,
drivers/gpio or drivers/irqchip.

Some devices managed by the I-pipe core (hardware timer, high
resolution counter, interrupt controller, GPIO controller) may be
specific to each SoC, and has to be adapted to run with the I-pipe
core.

.. NOTE: If the processor core is an ARM Cortex A9, things are going
to be a bit easier, as such core contains an interrupt controller, a
hardware timer and a high resolution counter, for which drivers have
already been ported to the I-pipe.

.. _`hardware-timer`:
Hardware timer
--------------

A hardware timer programmable for ticking in one-shot mode is required
by I-pipe clients such as co-kernels. Support for such timer is
abstracted by the I-pipe core in ``struct ipipe_timer``.

For most ARM SoCs, the hardware timer details are specific to each SoC
or SoC family, therefore such ``ipipe_timer`` descriptor must be added
on a SoC per SoC basis. There are several ways, to implement this
timer descriptor in the I-pipe core.

.. _`A9-timer`:
The Cortex-A9 case
~~~~~~~~~~~~~~~~~~

If the SoC you use is not based on the ARM Cortex A9 core, skip to the
:ref:`non-A9-timer <next section>`. In case of SoCs based on the ARM
Cortex A9 core, the hardware timer is provided by the processor core,
and not specific to the SoC: the good news is that the timer code has
already been modified to implement the ``struct ipipe_timer``
descriptor into the I-pipe core (arch/arm/kernel/smp_twd.c). However,
you should make sure that the Linux kernel compiles and uses the ARM
Cortex A9 hardware timer code when compiled for your SoC.

To this end, you should make sure that the ``smp_twd`` timer is
registered. You should make sure it declares a clock source with a
*compatible* string containing *twd-timer*.

If the SoC does not use the ``smp_twd`` timer and there is no kernel
configuration option allowing to select it, you will have to register
per-cpu timers using :ref:`non-A9-timer <next section>`.

.. NOTE: In some cases, the Linux support code for the Cortex A9 timer
may give imprecise timer frequency calibration results when I-pipe
updates are patched in, resulting in early shots. With proper device
tree support for the board, the proper clock frequency may be
determined automatically by the driver without resorting to any
imprecise calibration.

.. _`non-A9-timer`
The general case
~~~~~~~~~~~~~~~~

You should look for the hardware timer support code for your SoC.
Usually, this may be found in drivers/clocksource or
arch/arm/mach-X/time.c or arch/arm/plat-Y/time.c. Assuming your board
uses a device tree file, you should look for a device with a
compatible string containing ``-timer`` and try and find the
corresponding file in one of the places mentioned above.

Assuming the hardware timer is driven by the ``clock_event_device``
infrastructure, and provides support for the one-shot mode (the
``features`` member of the clock_event_device structure contains
``CLOCK_EVT_FEAT_ONESHOT``), your job will be easy. Otherwise, you
should find the SoC data-sheet or reference guide containing the
documentation for the hardware timer registers, and try to find out
what type it is (decrementer or free-running counter with match
register), and how to use it in one-shot mode.

You have to decide finally if you choose to share the hardware timer
used by Linux with the co-kernel, or if you are going to use a
different hardware timer (some SoC have several hardware timers
available). As a rule of thumb, it is better to use the same timer.

The ``ipipe_timer`` structure somewhat piggybacks on the
``clock_event_device`` structure, adding a set of capabilities
required by co-kernels for receiving high-precision events from the
timer hardware via the interrupt pipeline. It is defined in
include/linux/ipipe_tickdev.h contains the following members:

* `int irq`

This is the IRQ number used for the timer interrupt. Providing it is
mandatory.

* `void (*request)(struct ipipe_timer *timer, int steal)`

This callback is invoked by the I-pipe core when the co-kernel starts
using the hardware timer. It should set the hardware timer to one-shot
mode.  The ``steal`` parameter is true if the co-kernel is taking
control over the timer currently in use by Linux.

If the hardware timer support code for Linux uses the
``clock_event_device`` infrastructure, supports one-shot mode, and the
I-pipe core is going to use the same timer as Linux, this handler may
be omitted. In such a case, the I-pipe core is going to call the
default ``set_mode`` handler defined by the corresponding
``clock_event_device`` structure.

* `int (*set)(unsigned long ticks, void *timer)`

This handler is called by the I-pipe core each time the co-kernel asks
for programming the next event into the hardware timer registers. It
should program the hardware timer to elapse in ``ticks`` in hardware
time unit.

For instance, if the hardware timer is based on a decrementer, this
handler should set the decrementer register with the ``ticks``
value.

If the hardware timer is based on a free-running counter and a match
register instead, this handler should set the match register to the
sum of the current value of the free-running counter and the ``ticks``
parameter.

This function should return 0 upon success or a negative value if the
delay is too short (in case of a free-running counter and a match
register, this can be detected by re-reading the free-running counter
after having programmed the match register, if the free-running
counter has now passed the match register value, the delay was too
short, and the programming may have failed).

If the hardware timer support code for Linux uses the
``clock_event_device`` infrastructure, supports one-shot mode, and the
I-pipe core is going to use the same timer as Linux, this handler may
be omitted. In such a case, the I-pipe core is going to call the
default ``set_next_event`` handler defined by the corresponding
``clock_event_device`` structure.

.. CAUTION: Care must be taken however that this handler is called
from co-kernel context, therefore it may neither call any regular
Linux services, nor hold regular spinlocks. Otherwise, a separate
handler must be implemented (or if a spinlock has to be held, the
original spinlock should be turned into an :ref:`hard-spinlocks
<I-pipe spinlock>`, provided the critical sections being covered by
such lock are short).

* `void (*ack)(void)`

This handler is called by the I-pipe core upon timer interrupt, and it
should acknowledge the timer interrupt at hardware timer level. It is
almost always necessary to provide such handler.

If the hardware timer is shared with Linux, the code implementing the
proper hardware acknowledge code is generally contained in the Linux
timer interrupt. This interrupt code should be modified to only
acknowledge the timer interrupt if the timer is not controlled by the
co-kernel. See the :ref:`Example <example>` for a way to do this
avoiding code duplication of the timer acknowledgement code.

* `void (*release)(struct ipipe_timer *timer)`

This handler is called by the I-pipe core when the co-kernel releases
the hardware timer. It should restore the timer to its state at the
time when the ``request`` handler was called. For instance, if the
timer was running in periodic mode, and the ``request`` handler
switched it to one-shot mode, this handler should turn it back to
periodic mode.

If the hardware timer support code for Linux uses the
``clock_event_device`` infrastructure, supports one-shot mode, and the
I-pipe core is going to use the same timer as Linux, this handler may
be omitted. In such a case, the I-pipe core is going to call the
default ``set_mode`` handler defined by the corresponding
``clock_event_device`` structure.

* `const char *name`

Name of the timer.

If the I-pipe core is going to use the same timer as Linux, this
setting may be omitted, in which case the name defined by the
``clock_event_device`` descriptor for such timer will be used.

* `unsigned int rating`

Rating of the timer. If support for several hardware timers is
provided with different ratings, the one with the highest rating will
be used by the co-kernel.

If the I-pipe core is going to use the same timer as Linux, this
setting may be omitted, in which case the rating defined by the
``clock_event_device`` descriptor for such timer will be used.

* `unsigned long freq`

Frequency of the hardware timer. Usually, this value can be obtained
from the clock framework (``clk_get_rate()``).

If the I-pipe core is going to use the same timer as Linux, this
setting may be omitted, in which case the frequency defined by the
``clock_event_device`` descriptor for such timer will be used.

* `unsigned int min_delay_ticks`

The hardware timer minimum delay as a count of ticks. Almost all timers
based on free-running counters and match register have a threshold below
which they can not be programmed. When you program such a timer with a
too short value, the free-running counter will need to wrap before it
matches the match register again, so the timer will appear to be stopped
for a long time, then suddenly restart.

In case when this minimum delay is known as a wallclock delay instead
of a count of hardware ticks, ``ipipe_timer_ns2ticks()`` can be used
to convert values, making sure the ``ipipe_timer.freq`` has been set
prior to that call.

If the I-pipe core is going to use the same timer as Linux, this
setting may be omitted, in which case the delay defined by the
``clock_event_device`` descriptor for such timer will be used.

* `const struct cpumask *cpumask`

A cpumask containing the set of cpus where this timer will be run. On
SMP systems, there should be several ``ipipe_timer`` structures
defined, each with only one cpu in the ``cpumask`` member.

If the I-pipe core is going to use the same timer as Linux, this
setting may be omitted, in which case the mask defined by the
``clock_event_device`` descriptor for such timer will be used.

Once this structure is defined, there are two ways to register it to the
I-pipe core:

* if the hardware timer support code for Linux uses the
``clock_event_device`` infrastructure and the I-pipe core is going to
use the same hardware timer as Linux, the member ``ipipe_timer`` of
the ``clock_event_device`` descriptor should be set to point at this
structure, causing an automatic registration of both descriptors when
``clockevents_register_device()`` is called by the regular kernel
code.

* otherwise, the ``ipipe_timer_register()`` service should be called
for registering the descriptor individually.

.. _example:
Example
~~~~~~~

As an example, let us look at the OMAP3 code in the I-pipe core.
Previous to the introduction of the I-pipe bits, the code looked like:

-------------------------------------------------------------------------------
 static irqreturn_t omap2_gp_timer_interrupt(int irq, void *dev_id)
 {
	struct clock_event_device *evt = &clockevent_gpt;

	__omap_dm_timer_write_status(&clkev, OMAP_TIMER_INT_OVERFLOW);

	evt->event_handler(evt);
	return IRQ_HANDLED;
 }
-------------------------------------------------------------------------------

The call to ``__omap_dm_timer_write_status()`` acknowledges the
interrupt hardware timer level.

-------------------------------------------------------------------------------
 static struct clock_event_device clockevent_gpt = {
	.name           = "gp timer",
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.shift          = 32,
	.set_next_event = omap2_gp_timer_set_next_event,
	.set_mode       = omap2_gp_timer_set_mode,
 };
-------------------------------------------------------------------------------

This shows that the Linux hardware timer support code handles one-shot
mode, and closer inspection reveals that
``omap2_gp_timer_set_next_event()`` does not call any Linux service
which could not be called from out-of-band context. Therefore, this
implementation can be safely shared with the co-kernel. The I-pipe
core modifies this code in the following way:

-------------------------------------------------------------------------------
 static void omap2_gp_timer_ack(void)
 {
	__omap_dm_timer_write_status(&clkev, OMAP_TIMER_INT_OVERFLOW);
 }

 static irqreturn_t omap2_gp_timer_interrupt(int irq, void *dev_id)
 {
	struct clock_event_device *evt = &clockevent_gpt;

	if (!clockevent_ipipe_stolen(evt))
		omap2_gp_timer_ack();

	evt->event_handler(evt);
	return IRQ_HANDLED;
 }

 #ifdef CONFIG_IPIPE
 static struct ipipe_timer omap_shared_itimer = {
	.ack			= omap2_gp_timer_ack,
	.min_delay_ticks	= 3,
 };
 #endif /* CONFIG_IPIPE */

 static struct clock_event_device clockevent_gpt = {
	.features		= CLOCK_EVT_FEAT_PERIODIC |
				  CLOCK_EVT_FEAT_ONESHOT,
	.rating			= 300,
	.set_next_event		= omap2_gp_timer_set_next_event,
	.set_state_shutdown	= omap2_gp_timer_shutdown,
	.set_state_periodic	= omap2_gp_timer_set_periodic,
	.set_state_oneshot	= omap2_gp_timer_shutdown,
	.tick_resume		= omap2_gp_timer_shutdown,
 };

 static void __init omap2_gp_clockevent_init(int gptimer_id,
						const char *fck_source)
 {
	/* ... */
 #ifdef CONFIG_IPIPE
	/* ... */
		omap_shared_itimer.irq = clkev.irq;
		clockevent_gpt.ipipe_timer = &omap_shared_itimer;
	/* ... */
 #endif /* CONFIG_IPIPE */

	clockevents_register_device(&clockevent_gpt);

	/* ... */
 }
-------------------------------------------------------------------------------

High resolution counter
-----------------------

Since the co-kernel timer management is based on a timer running in
one-shot mode, and in order for applications to be able to measure
short time intervals, a high resolution counter is needed.

Again, the hardware which can be used for such purposes depends on the
SoC. Reminiscent from the first Xenomai co-kernel using the I-pipe
which ran on the x86 processor architecture, this high resolution
counter is (abusively) called tsc (short for timestamp counter).

As in the case of timer management, a C structure named
``__ipipe_tscinfo`` must be filled and registered to the I-pipe
core. You should also ensure that the symbol
"CONFIG_IPIPE_ARM_KUSER_TSC" gets selected. For instance, in
arch/arm/mach-socfpga/Kconfig, you may find:

-------------------------------------------------------------------------------
menuconfig ARCH_SOCFPGA
	bool "Altera SOCFPGA family"
	depends on ARCH_MULTI_V7
	...
	select IPIPE_ARM_KUSER_TSC if IPIPE
-------------------------------------------------------------------------------

.. _`A9-counter`:
The Cortex A9 case
~~~~~~~~~~~~~~~~~~

If the SoC you use is not based on the ARM Cortex A9 core, skip to the
:ref:`non-A9-counter <next section>`. In case of SoCs based on the ARM
Cortex A9 core, the hardware used as high resolution counter is
provided by the ARM core (aka "global timer"). Since this hardware is
not SoC-specific, the existing addition of ``__ipipe_tscinfo`` to
support the I-pipe (arch/arm/kernel/smp_twd.c) can be reused for all
A9-based SoCs.

.. _`non-A9-counter`:
The general case
~~~~~~~~~~~~~~~~

The ``__ipipe_tscinfo`` C structure, defined in
arch/arm/include/asm/ipipe.h contains the following members:

* ``unsigned int type``

The type, possible values are:

** ``IPIPE_TSC_TYPE_FREERUNNING``

the tsc is based on a free-running counter

** ``IPIPE_TSC_TYPE_DECREMENTER``

the tsc is based on a decrementer

** ``IPIPE_TSC_TYPE_FREERUNNING_COUNTDOWN``

the tsc is based on a free-running counter, counting down

** ``IPIPE_TSC_TYPE_FREERUNNING_TWICE``

the tsc is based on a free-running counter which needs to be read
twice (it sometimes returns wrong values, but never twice in a row)

If the hardware you have at hand is not one of these, you need to

** add a define for the type of hardware you have
(``IPIPE_TSC_TYPE_xxx``)

** add an implementation (in assembly) for reading this counter and
extending it to a 64 bits value. See arch/arm/kernel/ipipe_tsc_asm.S and
arch/arm/kernel/ipipe_tsc.c for more details. Note that the assembly
implementation is limited in size to 96 bytes, or 24 x 32 bits
instructions.

* ``unsigned int freq``

The counter frequency

* ``unsigned long counter_vaddr``

The virtual address (in kernel-space) of the counter

* ``unsigned long u.counter_paddr``

The physical address of the counter

* ``unsigned long u.mask``

The mask of valid bits in the counter value.

For instance 0xffffffff for a 32 bits counter, or 0xffff for a 16 bits
counter. Only a limited set of values are supported for each counter
type. If you need an unsupported value, arch/arm/kernel/ipipe_tsc.c
and arch/arm/kernel/ipipe_tsc_asm.S must be modified.

Once a variable of type ``__ipipe_tscinfo`` is defined, it can be
registered to the I-pipe core with ``__ipipe_tsc_register()``.

For instance, in arch/arm/mach-davinci/time.c, we have:

-------------------------------------------------------------------------------
#ifdef CONFIG_IPIPE
static struct __ipipe_tscinfo tsc_info = {
	.type = IPIPE_TSC_TYPE_FREERUNNING,
	.u = {
			{
				.mask = 0xffffffff,
			},
	},
};
#endif /* CONFIG_IPIPE */

void __init davinci_timer_init(void)
{
#ifdef CONFIG_IPIPE
	tsc_info.freq = davinci_clock_tick_rate;
	tsc_info.counter_vaddr = (void *)(timers[TID_CLOCKSOURCE].base +
			timers[TID_CLOCKSOURCE].tim_off);
	tsc_info.u.counter_paddr = timers[TID_CLOCKSOURCE].pbase +
			timers[TID_CLOCKSOURCE].tim_off;
	__ipipe_tsc_register(&tsc_info);
	/* ... */
#endif /* CONFIG_IPIPE */
}

-------------------------------------------------------------------------------

Since the tsc implementation extends the precision of the underlying
hardware counter to 64 bits, it also needs to be refreshed at a lower
period than the hardware counter wrap time. This refreshing is done by
``__ipipe_tsc_update()``, which is called periodically for a
registered tsc.

If your hardware timer is based on a 16 bits counter,
``__ipipe_tsc_update()`` should be called in the ``ipipe_timer``'s
``set()`` handler as well, every time the hardware timer is
programmed.

.. _`interrupt-controller`:
Interrupt controller
--------------------

The I-pipe core interacts with the SoC interrupt controller, for
implementing the deferred interrupt model. An interrupt is first
acknowledged and masked at the interrupt controller level, but handled
then unmasked by the regular Linux IRQ handler only when all
out-of-band activity is done.

Fortunately, as for timer management, interrupt controllers specifics
are embedded in the ``irq_chip`` C structure, and interactions with
them are implemented in a generic way, so almost no modifications need
to be done in the SoC-specific code. There are a few things to which
you should pay attention to, though.

As in the case of the timer and high resolution counter, the Cortex A9
processor core contains an interrupt controller. If your SoC is based
on the Cortex A9 core, you can skip to :ref:`config-multi-irq-handler
<the CONFIG_MULTI_IRQ_HANDLER section>`.

Otherwise, you should locate the code for the interrupt controller
management. Usually, the IRQ controller driver is located in
drivers/irqchip, arch/arm/mach-X/irq.c or arch/arm/plat-Y/irq.c. As
for the hardware timer, the irqchip should be registered through the
device tree, so you should look in the SoC device tree file for a node
with one of the "compatible" strings passed to the IRQCHIP_DECLARE
macro in the kernel sources.

IC handlers
~~~~~~~~~~~

The following handlers defined by the ``irq_chip`` C structure may be
called from an out-of-band context immediately upon IRQ receipt, so
they must not call any regular Linux services:

* ``irq_ack``
* ``irq_mask_ack``
* ``irq_eoi``
* ``irq_mask``
* ``irq_unmask``

In particular, regular Linux spinlocks used in those routines should
be turned into an :ref:`hard-spinlocks <I-pipe spinlock>`, making sure
this would not entail unacceptable latencies from other places such
lock is held.

flow handlers
~~~~~~~~~~~~~

If the original flow handler for the IRQ is ``handle_fasteoi_irq()``,
two I-pipe specific IC handlers should be defined by the ``irq_chip``
descriptor:

* ``irq_hold`` should mask then EOI the interrupt line, i.e. same as
  calling ``irq_mask`` and ``irq_eoi`` subsequently.

* ``irq_release`` should unmask the interrupt line, i.e. same as
  calling ``irq_unmask``.

If the flow handler is ``handle_edge_irq()`` and the systems locks up
when the first interrupt is received, try turning the flow handler to
``handle_level_irq()`` instead.

.. _`config-multi-irq-handler`:
CONFIG_MULTI_IRQ_HANDLER
~~~~~~~~~~~~~~~~~~~~~~~~

If the SoC you use enables this option, look into the board file
between the MACHINE_START and MACHINE_END declarations for the
``handle_irq`` member. The original implementation of this routine
should be in the interrupt controller file, exhibiting a decoding loop
of interrupts numbers reading the hardware registers, eventually
calling ``handle_IRQ`` for each decoded IRQ.

Once again, you must make sure that no regular Linux routine is called
by this low-level IRQ decoding handler, invoking
``ipipe_handle_domain_irq`` instead of ``handle_domain_irq``.

Likewise, on SMP systems, calls to ``handle_IPI`` should be replaced
by a call to ``ipipe_handle_multi_ipi``.

multi-processor systems
~~~~~~~~~~~~~~~~~~~~~~~

On multi-processor systems, IPIs are mapped to virtual pipelined IRQs
(aka *virqs*), and the SoC support needs no addition.

.. _GPIOs
GPIOs
~~~~~

Most SoCs have GPIOs. In the context of a co-kernel, they are
interesting for two reasons:

* they may be used by real-time drivers as input or output for
communicating with external peripherals.
* they may be used as interrupt sources.

GPIOs in real-time drivers
~~~~~~~~~~~~~~~~~~~~~~~~~~

As for hardware timers and interrupt controllers, the specifics of a
GPIO controller are embedded in a structure, i.e. ``gpio_chip``.  The
definition for the SoC is usually found in one of the files:
drivers/gpio-X.c, arch/arm/mach-Y/gpio.c, arch/arm/plat-Z/gpio.c.

These handlers are accessible using the regular *gpiolib*
infrastructure.

For instance, the ``gpio_chip`` descriptor defines a ``get`` handler,
which is indirectly called from ``gpio_get_value`` for reading out the
current level for any given pin.

Here again, you must make sure that no regular Linux routine is called
by GPIO handlers. If this is the case:

* if the implementation of these handlers need to communicate with an
I2C or SPI chip, the code as it is needs significant changes to be made
available to real-time drivers, starting with rewriting the driver for
the I2C or SPI controller as a driver running in real-time domain;

* if the implementation of these handlers simply uses a spinlock, the
spinlock may be turned into an :ref:`hard-spinlocks <I-pipe spinlock>`
(pay attention, however, that there is not other Linux service called,
or actions which may take an unbounded time when holding the
spinlock).

GPIOs as interrupt sources
~~~~~~~~~~~~~~~~~~~~~~~~~~

Most SoCs have so many GPIOs, that each one can not have a separate
line at the interrupt controller level, so they are multiplexed. What
happens then is that there is a single line for a whole GPIO bank, the
interrupt handler for this irq line should read a GPIO controller
register to find out which of the GPIOs interrupts are pending, then
invoke the handler for each of them. The mechanism used by the Linux
kernel to handle this situation is called "chained interrupts", you
can find whether the SoC you use in this case if it calls the function
"irq_set_chained_handler".  It is usually found in
drivers/gpio/gpio-X.c, arch/arm/mach-Y/gpio.c, arch/arm/plat-Z/gpio.c,
arch/arm/mach-X/irq.c, or arch/arm/plat-Y/irq.c.

What will happen with the I-pipe core, is that the handler registered
with "irq_set_chained_handler" will be called in real-time context, so
should not use any Linux service which can not be used from real-time
context, in particular, calls to "generic_handle_irq", should be
replaced with calls to "ipipe_handle_demuxed_irq".

When GPIOs are used as interrupt sources, a "struct irq_chip" is
defined, allowing the kernel to see the GPIOs controller as an
interrupt controller, so, most of what is said in the
:ref:`interrupt-controller <"Interrupt controller" section>` also
applies to the GPIO controller. Most of the time, though, the "flow
handler" for these interrupts is "handle_simple_irq", and nothing
needs to be done.

.. _`hard-spinlocks`
I-pipe spinlocks
----------------

Occasionally, some spinlocks need to be shared between the real-time and
Linux domains. We have talked about this in the
:ref:`hardware-timer <"Hardware timer">`,
:ref:`interrupt-controller <"Interrupt controller">` and
:ref:`GPIOs <"GPIOs">` sections.

However, beware, this is not a panacea: calling a regular kernel
routine while holding this spinlock may end up in a train wreck for
the system, at the very least cause the response time skyrocket for
the co-kernel applications.

The I-pipe provides macros to turn a regular or raw kernel spinlock
definitions into I-pipe hard spinlocks, and others to declare the
latter.

[cols=",",]
|==============================================================
|Linux code |Should be replaced with
|``extern raw_spinlock_t foo`` |``IPIPE_DECLARE_RAW_SPINLOCK(foo)``
|``DEFINE_RAW_SPINLOCK(foo)`` |``IPIPE_DEFINE_RAW_SPINLOCK(foo)``
|``extern spinlock_t foo`` |``IPIPE_DECLARE_SPINLOCK(foo)``
|``DEFINE_SPINLOCK(foo)`` |``IPIPE_DEFINE_SPINLOCK(foo)``
|==============================================================

For instance, in arch/arm/mm/context.c

-------------------------------------------------------------------------------
 static DEFINE_RAW_SPINLOCK(cpu_asid_lock);
-------------------------------------------------------------------------------

is replaced with:

-------------------------------------------------------------------------------
 static IPIPE_DEFINE_RAW_SPINLOCK(cpu_asid_lock);
-------------------------------------------------------------------------------

In addition to the usual ``spin_lock()``, ``spin_unlock()``,
``spin_lock_irqsave()`` and ``spin_unlock_irqrestore()`` routines, the
I-pipe core provides the ``spin_lock_irqsave_cond()``,
``spin_unlock_irqrestore_cond()``.

These services are replaced with their ``spin_lock_irqsave()`` /
``spin_unlock_irqrestore()`` counterparts when compiling the Linux
kernel with the I-pipe core enabled, and replaced with ``spin_lock()``
/ ``spin_unlock()`` otherwise.

This is useful for protecting a critical section of the regular kernel
code against preemption from out-of-band IRQ handlers.

[[troubleshooting]]
Troubleshooting
---------------

When you have modified the I-pipe core for supporting your board, try:

* to boot the kernel for your board compiled without CONFIG_IPIPE
enabled
* boot the kernel for your board compiled with CONFIG_IPIPE enabled but
without the co-kernel (e.g. disable CONFIG_XENOMAI).
* boot the kernel for your board compiles with CONFIG_IPIPE and
the co-kernel (e.g. enable CONFIG_XENOMAI).
* run the latency test

If any of this step does not work correctly, do not go further, try and
debug the said step first.

Common issues include:

[[the-kernel-stops-after-the-message-uncompressing-linux]]
The kernel stops after the message "Uncompressing Linux... done, booting the kernel."
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The screen remains blank, nothing happens. It means that the kernel has
a oops, or lock-up early during the boot process. In order to understand
what happens:

* enable CONFIG_DEBUG_LL and CONFIG_EARLY_PRINTK in the kernel
configuration, recompile the kernel.

.. CAUTION: make sure to configure the debug UART properly, otherwise
   this may crash the kernel in the early boot stage.

* add "earlyprintk" to the kernel parameters

The kernel messages should then be displayed immediately, allowing to
figure out at what point in the boot process the kernel crashes or
locks up.

[[the-kernel-stops-after-the-message-calibrating-delay-loop]]
The kernel stops after the message "Calibrating delay loop..."
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It means that the timer interrupt is not ticking and that the delay
calibration routine is running an infinite loop at ``while (ticks ==
jiffies)`` in the function ``calibrate_delay()``, file
init/calibrate.c

This probably means that changes you made to the hardware timer support
or interrupt controller code broke something. To help debugging this
situation, you can print any hardware timer or interrupt controller
register in the ``while (ticks == jiffies)`` loop.

[[timer-issues]]
Timer issues
~~~~~~~~~~~~

Most issues when porting the I-pipe core to a new ARM SoC are timer
issues, the timer is the hardest part to get right.

When you boot the kernel without CONFIG_IPIPE, the timer code should
be almost not modified, except maybe for timer acknowledgement. If at
this point the kernel does not work, it probably means that you got
the timer acknowledgement wrong.

When you boot the kernel with CONFIG_IPIPE, but without enabling the
co-kernel (e.g. CONFIG_XENOMAI), the hardware timer remains controlled
by the Linux kernel. If at this point the kernel does not work, it
probably means that something else than the timer is wrong, most
likely in the interrupt controller support code.

When you boot the dual kernel system, the co-kernel usually takes
control over the hardware timer, invoking the ``struct ipipe_timer``
:ref:`non-A9-timer <handlers>` for processing a regular Linux timer
tick. If at this point the kernel does not work, it probably means
that some of those handlers are wrong.

Finally, only when running some co-kernel application like a latency
measurement test, the timer is eventually used to activate co-kernel
threads in the real-time domain. You should check that the latency
test prints a message every second, if it does not, it probably means
that the timer frequency is wrong, but in accordance with the tsc
frequency.

A *drift* in the minimum and maximum latency values indicates a
mismatch between the timer and the tsc frequency. Unacceptably large
latency values is likely caused by a section of code running for too
long with interrupts masked, or some issue caused by the idle loop.
