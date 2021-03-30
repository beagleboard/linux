.. include:: <isonum.txt>

===================================
The Interrupt Pipeline (aka I-pipe)
===================================

:Copyright: |copy| 2018: Philippe Gerum

Purpose
=======

Using Linux as a host for lightweight software cores specialized in
delivering very short and bounded response times has been a popular
way of supporting real-time applications in the embedded space over
the years.

This design - known as the *dual kernel* approach - introduces a small
real-time infrastructure which schedules time-critical activities
independently from the main kernel. Application threads co-managed by
this infrastructure still benefit from the ancillary kernel services
such as virtual memory management, and can also leverage the rich GPOS
feature set Linux provides such as networking, data storage or GUIs.

Although the real-time infrastructure has to present specific driver
stack and API implementations to applications, there are nonetheless
significant upsides to keeping the real-time core separate from the
GPOS infrastructure:

- because the two kernels are independent, real-time activities are
  not serialized with GPOS operations internally, removing potential
  delays which might be induced by the non time-critical
  work. Likewise, there is no requirement for keeping the GPOS
  operations fine-grained and highly preemptible at any time, which
  would otherwise induce noticeable overhead on low-end hardware, due
  to the requirement for pervasive task priority inheritance and IRQ
  threading.

- the functional isolation of the real-time infrastructure from the
  rest of the kernel code restricts common bug hunting to the scope of
  the smaller kernel, excluding most interactions with the very large
  GPOS kernel base.

- with a dedicated infrastructure providing a specific, well-defined
  set of real-time services, applications can unambiguously figure out
  which API calls are available for supporting time-critical work,
  excluding all the rest as being potentially non-deterministic with
  respect to response time.

To support such a *dual kernel system*, we need the kernel to exhibit
a high-priority execution context, for running out-of-band real-time
duties concurrently to the regular operations.

.. NOTE:: The I-pipe only introduces the basic mechanisms for hosting
such a real-time core, enabling the common programming model for its
applications in user-space. It does *not* implement the real-time core
per se, which should be provided by a separate kernel component.

The issue of interrupt response time
====================================

The real-time core has to act upon device interrupts with no delay,
regardless of the regular kernel operations which may be ongoing when
the interrupt is received by the CPU.

However, to protect from deadlocks and maintain data integrity, Linux
normally hard disables interrupts around any critical section of code
which must not be preempted by interrupt handlers on the same CPU,
enforcing a strictly serialized execution among those contexts.

The unpredictable delay this may cause before external events can be
handled is a major roadblock for kernel components requiring
predictable and very short response times to external events, in the
range of a few microseconds.

Therefore, there is a basic requirement for prioritizing interrupt
masking and delivery between the real-time core and GPOS operations,
while maintaining consistent internal serialization for the kernel.

To address this issue, the I-pipe implements a mechanism called
*interrupt pipelining* turns all device IRQs into NMIs, only to run
NMI-safe interrupt handlers from the perspective of the regular kernel
activities.

Two-stage IRQ pipeline
======================

.. _pipeline
Interrupt pipelining is a lightweight approach based on the
introduction of a separate, high-priority execution stage for running
out-of-band interrupt handlers immediately upon IRQ receipt, which
cannot be delayed by the in-band, regular kernel work even if the
latter serializes the execution by - seemingly - disabling interrupts.

IRQs which have no handlers in the high priority stage may be deferred
on the receiving CPU until the out-of-band activity has quiesced on
that CPU. Eventually, the preempted in-band code can resume normally,
which may involve handling the deferred interrupts.

In other words, interrupts are flowing down from the out-of-band to
the in-band interrupt stages, which form a two-stage pipeline for
prioritizing interrupt delivery.

The runtime context of the out-of-band interrupt handlers is known as
the *head stage* of the pipeline, as opposed to the in-band kernel
activities sitting on the *root stage*::

                    Out-of-band                 In-band
                    IRQ handlers()            IRQ handlers()
               __________   _______________________   ______
                  .     /  /  .             .     /  /  .
                  .    /  /   .             .    /  /   .
                  .   /  /    .             .   /  /    .
                  ___/  /______________________/  /     .
     [IRQ] -----> _______________________________/      .
                  .           .             .           .
                  .   Head    .             .   Root    .
                  .   Stage   .             .   Stage   .
               _____________________________________________


A software core may base its own activities on the head stage,
interposing on specific IRQ events, for delivering real-time
capabilities to a particular set of applications. Meanwhile, the
regular kernel operations keep going over the root stage unaffected,
only delayed by short preemption times for running the out-of-band
work.

.. NOTE:: Interrupt pipelining is a partial implementation of [#f2]_,
          in which an interrupt *stage* is a limited form of an
          operating system *domain*.

Virtual interrupt flag
----------------------

.. _flag:
As hinted earlier, predictable response time of out-of-band handlers
to IRQ receipts requires the in-band kernel work not to be allowed to
delay them by masking interrupts in the CPU.

However, critical sections delimited this way by the in-band code must
still be enforced for the *root stage*, so that system integrity is
not at risk. This means that although out-of-band IRQ handlers may run
at any time while the *head stage* is accepting interrupts, in-band
IRQ handlers should be allowed to run only when the root stage is
accepting interrupts too.

So we need to decouple the interrupt masking and delivery logic which
applies to the head stage from the one in effect on the root stage, by
implementing a dual interrupt control mechanism.

To this end, a software logic managing a virtual interrupt flag (aka
*IPIPE_STALL_FLAG*) is introduced by the interrupt pipeline between
the hardware and the generic IRQ management layer. This logic can mask
IRQs from the perspective of the regular kernel work when
:c:func:`local_irq_save`, :c:func:`local_irq_disable` or any
lock-controlled masking operations like :c:func:`spin_lock_irqsave` is
called, while still accepting IRQs from the CPU for immediate delivery
to out-of-band handlers.

The head stage protects from interrupts by disabling them in the CPU's
status register, while the root stage disables interrupts only
virtually. A stage for which interrupts are disabled is said to be
*stalled*. Conversely, *unstalling* a stage means re-enabling
interrupts for it.

Obviously, stalling the head stage implicitly means disabling
further IRQ receipts for the root stage too.

Interrupt deferral for the *root stage*
---------------------------------------

.. _deferral:
.. _deferred:
When the root stage is stalled by setting the virtual interrupt flag,
the occurrence of any incoming IRQ which was not delivered to the
*head stage* is recorded into a per-CPU log, postponing its actual
delivery to the root stage.

The delivery of the interrupt event to the corresponding in-band IRQ
handler is deferred until the in-band kernel code clears the virtual
interrupt flag by calling :c:func:`local_irq_enable` or any of its
variants, which unstalls the root stage. When this happens, the
interrupt state is resynchronized by playing the log, firing the
in-band handlers for which an IRQ was set pending.

::
   /* Both stages unstalled on entry */
   local_irq_save(flags);
   <IRQx received: no out-of-band handler>
       (pipeline logs IRQx event)
   ...
   local_irq_restore(flags);
       (pipeline plays IRQx event)
            handle_IRQx_interrupt();

If the root stage is unstalled at the time of the IRQ receipt, the
in-band handler is immediately invoked, just like with the
non-pipelined IRQ model.

.. NOTE:: The principle of deferring interrupt delivery based on a
          software flag coupled to an event log has been originally
          described as "Optimistic interrupt protection" in [#f1]_.

Device interrupts virtually turned into NMIs
--------------------------------------------

From the standpoint of the in-band kernel code (i.e. the one running
over the *root* interrupt stage) , the interrupt pipelining logic
virtually turns all device IRQs into NMIs, for running out-of-band
handlers.

.. _re-entry:
For this reason, out-of-band code may generally **NOT** re-enter
in-band code, for preventing creepy situations like this one::

   /* in-band context */
   spin_lock_irqsave(&lock, flags);
      <IRQx received: out-of-band handler installed>
         handle_oob_event();
            /* attempted re-entry to in-band from out-of-band. */
            in_band_routine();
               spin_lock_irqsave(&lock, flags);
               <DEADLOCK>
               ...
            ...
         ...
   ...
   spin_unlock irqrestore(&lock, flags);

Even in absence of any attempt to get a spinlock recursively, the
outer in-band code in the example above is entitled to assume that no
access race can occur on the current CPU while interrupts are
masked. Re-entering in-band code from an out-of-band handler would
invalidate this assumption.

In rare cases, we may need to fix up the in-band kernel routines in
order to allow out-of-band handlers to call them. Typically, atomic_
helpers are such routines, which serialize in-band and out-of-band
callers.

Virtual/Synthetic interrupt vectors
-----------------------------------

.. _synthetic:
.. _virtual:
The pipeline introduces an additional type of interrupts, which are
purely software-originated, with no hardware involvement. These IRQs
can be triggered by any kernel code. So-called virtual IRQs are
inherently per-CPU events.

Because the common pipeline flow_ applies to virtual interrupts, it
is possible to attach them to out-of-band and/or in-band handlers,
just like device interrupts.

.. NOTE:: virtual interrupts and regular softirqs differ in essence:
          the latter only exist in the in-band context, and therefore
          cannot trigger out-of-band activities.

Virtual interrupt vectors are allocated by a call to
:c:func:`ipipe_alloc_virq`, and conversely released with
:c:func:`ipipe_free_virq`.

For instance, a virtual interrupt can be used for triggering an
in-band activity on the root stage from the head stage as follows::

  #include <linux/ipipe.h>

  static void virq_handler(unsigned int virq, void *cookie)
  {
        do_in_band_work();
  }

  void install_virq(void)
  {
     unsigned int virq;
     ...
     virq = ipipe_alloc_virq();
     ...
     ipipe_request_irq(ipipe_root_domain, virq, virq_handler,
		       handler_arg, NULL);
  }

An out-of-band handler can schedule the execution of
:c:func:`virq_handler` like this::

  ipipe_post_irq_root(virq);

Conversely, a virtual interrupt can be handled from the out-of-band
context::

  static void virq_oob_handler(unsigned int virq, void *cookie)
  {
        do_oob_work();
  }

  void install_virq(void)
  {
     unsigned int virq;
     ...
     virq = ipipe_alloc_virq();
     ...
     ipipe_request_irq(ipipe_head_domain, virq, virq_oob_handler,
		       handler_arg, NULL);
  }

Any in-band code can trigger the immediate execution of
:c:func:`virq_oob_handler` on the head stage as follows::

  ipipe_post_irq_head(virq);

Pipelined interrupt flow
------------------------

.. _flow:
When interrupt pipelining is enabled, IRQs are first delivered to the
pipeline entry point via a call to the generic
:c:func:`__ipipe_dispatch_irq` routine. Before this happens, the event
has been propagated through the arch-specific code for handling an IRQ::

    asm_irq_entry
       -> irqchip_handle_irq()
          -> ipipe_handle_domain_irq()
             -> __ipipe_grab_irq()
                -> __ipipe_dispatch_irq()
                -> irq_flow_handler()
                <IRQ delivery logic>

Contrary to the non-pipelined model, the generic IRQ flow handler does
*not* call the in-band interrupt handler immediately, but only runs
the irqchip-specific handler for acknowledging the incoming IRQ event
in the hardware.

.. _Holding interrupt lines:
If the interrupt is either of the *level-triggered*, *fasteoi* or
*percpu* type, the irqchip is given a chance to hold the interrupt
line, typically by masking it, until either of the out-of-band or
in-band handler have run. This addresses the following scenario, which
happens for a similar reason while an IRQ thread waits for being
scheduled in, requiring the same kind of provision::

    /* root stage stalled on entry */
    asm_irq_entry
       ...
          -> __ipipe_dispatch_irq()
             ...
                <IRQ logged, delivery deferred>
    asm_irq_exit
    /*
     * CPU allowed to accept interrupts again with IRQ cause not
     * acknowledged in device yet => **IRQ storm**.
     */
    asm_irq_entry
       ...
    asm_irq_exit
    asm_irq_entry
       ...
    asm_irq_exit

IRQ delivery logic
------------------

If an out-of-band handler exists for the interrupt received,
:c:func:`__ipipe_dispatch_irq` invokes it immediately, after switching
the execution context to the head stage if not current yet.

Otherwise, if the execution context is currently over the root stage
and unstalled, the pipeline core delivers it immediately to the
in-band handler.

In all other cases, the interrupt is only set pending into the per-CPU
log, then the interrupt frame is left.

Alternate scheduling
====================

The I-pipe promotes the idea that a *dual kernel* system should keep
the functional overlap between the kernel and the real-time core
minimal. To this end, a real-time thread should be merely seen as a
regular task with additional scheduling capabilities guaranteeing very
low response times.

To support such idea, the I-pipe enables kthreads and regular user
tasks to run alternatively in the out-of-band execution context
introduced by the interrupt pipeline_ (aka *head* stage), or the
common in-band kernel context for GPOS operations (aka *root* stage).

As a result, real-time core applications in user-space benefit from
the common Linux programming model - including virtual memory
protection -, and still have access to the regular Linux services for
carrying out non time-critical work.

Task migration to the head stage
--------------------------------

Low latency response time to events can be achieved when Linux tasks
wait for them from the out-of-band execution context. The real-time
core is responsible for switching a task to such a context as part of
its task management rules; the I-pipe facilitates this migration with
dedicated services.

The migration process of a task from the GPOS/in-band context to the
high-priority, out-of-band context is as follows:

1. :c:func:`__ipipe_migrate_head` is invoked from the migrating task
   context, with the same prerequisites than for calling
   :c:func:`schedule` (preemption enabled, interrupts on).

.. _`in-band sleep operation`:
2. the caller is put to interruptible sleep state (S).

3. before resuming in-band operations, the next task picked by the
   (regular kernel) scheduler on the same CPU for replacing the
   migrating task fires :c:func:`ipipe_migration_hook` which the
   real-time core should override (*__weak* binding). Before the call,
   the head stage is stalled, interrupts are disabled in the CPU. The
   root execution stage is still current though.

4. the real-time core's implementation of
   :c:func:`ipipe_migration_hook` is passed a pointer to the
   task_struct descriptor of the migrating task. This routine is expected
   to perform the necessary steps for taking control over the task on
   behalf of the real-time core, re-scheduling its code appropriately
   over the head stage. This typically involves resuming it from the
   `out-of-band suspended state`_ applied during the converse migration
   path.

5. at some point later, when the migrated task is picked by the
   real-time scheduler, it resumes execution on the head stage with
   the register file previously saved by the kernel scheduler in
   :c:func:`switch_to` at step 1.

Task migration to the root stage
--------------------------------

Sometimes, a real-time thread may want to leave the out-of-band
context, continuing execution from the in-band context instead, so as
to:

- run non time-critical (in-band) work involving regular system calls
  handled by the kernel,

- recover from CPU exceptions, such as handling major memory access
  faults, for which there is no point in caring for response time, and
  therefore makes no sense to duplicate in the real-time core anyway.

.. NOTE: The discussion about exception_ handling covers the last
   point in details.

The migration process of a task from the high-priority, out-of-band
context to the GPOS/in-band context is as follows::

1. the real-time core schedules an in-band handler for execution which
   should call :c:func:`wake_up_process` to unblock the migrating task
   from the standpoint of the kernel scheduler. This is the
   counterpart of the :ref:`in-band sleep operation <in-band sleep
   operation>` from the converse migration path. A virtual_ IRQ can be
   used for scheduling such event from the out-of-band context.

.. _`out-of-band suspended state`:
2. the real-time core suspends execution of the current task from its
   own standpoint. The real-time scheduler is assumed to be using the
   common :c:func:`switch_to` routine for switching task contexts.

3. at some point later, the out-of-band context is exited by the
   current CPU when no more high-priority work is left, causing the
   preempted in-band kernel code to resume execution on the root
   stage. The handler scheduled at step 1 eventually runs, waking up
   the migrating task from the standpoint of the kernel.

4. the migrating task resumes from the tail scheduling code of the
   real-time scheduler, where it suspended in step 2. Noticing the
   migration, the real-time core eventually calls
   :c:func:`__ipipe_reenter_root` for finalizing the transition of the
   incoming task to the root stage.

Binding to the real-time core
-----------------------------

.. _binding:
The I-pipe facilitates fine-grained per-thread management from the
real-time core, as opposed to per-process. For this reason, the
real-time core should at least implement a mechanism for turning a
regular task into a real-time thread with extended capabilities,
binding it to the core.

The real-time core should inform the kernel about its intent to
receive notifications about that task, by calling
:c:func::`ipipe_enable_notifier` when such task is current.

For this reason, the binding operation is usually carried out by a
dedicated system call exposed by the real-time core, which a regular
task would invoke.

.. NOTE:: Whether there should be distinct procedures for binding
	  processes *and* threads to the real-time core, or only a
	  thread binding procedure is up to the real-time core
	  implementation.

Notifications
-------------

Exception handling
~~~~~~~~~~~~~~~~~~

.. _exception
If a processor exception is raised while the CPU is busy running a
real-time thread in the out-of-band context (e.g. due to some invalid
memory access, bad instruction, FPU or alignment error etc), the task
may have to leave such context immediately if the fault handler is not
protected against out-of-band interrupts, and therefore cannot be
properly serialized with out-of-band code.

The I-pipe notifies the real-time core about incoming exceptions early
from the low-level fault handlers, but only when some out-of-band code
was running when the exception was taken. The real-time core may then
take action, such as reconciling the current task's execution context
with the kernel's expectations before the task may traverse the
regular fault handling code.

.. HINT:: Enabling debuggers to trace real-time thread involves
          dealing with debug traps the former may poke into the
          debuggee's code for breakpointing duties.

The notification is issued by a call to :c:func:`__ipipe_notify_trap`
which in turn invokes the :c:func:`ipipe_trap_hook` routine the
real-time core should override for receiving those events (*__weak*
binding). Interrupts are **disabled** in the CPU when
:c:func:`ipipe_trap_hook` is called.::

     /* out-of-band code running */
     *bad_pointer = 42;
        [ACCESS EXCEPTION]
	   /* low-level fault handler in arch/<arch>/mm */
           -> do_page_fault()
	      -> __ipipe_notify_trap(...)
	         /* real-time core */
	         -> ipipe_trap_hook(...)
		    -> forced task migration to root stage
	   ...
           -> handle_mm_fault()

.. NOTE:: handling minor memory access faults only requiring quick PTE
          fixups should not involve switching the current task to the
          in-band context though. Instead, the fixup code should be
          made atomic_ for serializing accesses from any context.

System calls
~~~~~~~~~~~~

A real-time core interfaced with the kernel via the I-pipe may
introduce its own set of system calls. From the standpoint of the
kernel, this is a foreign set of calls, which can be distinguished
unambiguously from regular ones based on an arch-specific marker.

.. HINT:: Syscall numbers from this set might have a different base,
	  and/or some high-order bit set which regular syscall numbers
	  would not have.

If a task bound to the real-time core issues any system call,
regardless of which of the kernel or real-time core should handle it,
the latter must be given the opportunity to:

- perform the service directly, possibly switching the caller to
  out-of-band context first would the request require it.

- pass the request downward to the normal system call path on the root
  stage, possibly switching the caller to in-band context if needed.

If a regular task (i.e. *not* known from the real-time core [yet])
issues any foreign system call, the real-time core is given a chance
to handle it. This way, a foreign system call which would initially
bind a regular task to the real-time core would be delivered to the
real-time core as expected (see binding_).

The I-pipe intercepts system calls early in the kernel entry code,
delivering them to the proper handler according to the following
logic::

     is_foreign(syscall_nr)?
	    Y: is_bound(task)
	           Y: -> ipipe_fastcall_hook()
		   N: -> ipipe_syscall_hook()
            N: is_bound(task)
	           Y: -> ipipe_syscall_hook()
		   N: -> normal syscall handling

:c:func:`ipipe_fastcall_hook` is the fast path for handling foreign
system calls from tasks already running in out-of-band context.

:c:func:`ipipe_syscall_hook` is a slower path for handling requests
which might require the caller to switch to the out-of-band context
first before proceeding.

Kernel events
~~~~~~~~~~~~~

The last set of notifications involves pure kernel events which the
real-time core may need to know about, as they may affect its own task
management. Except for IPIPE_KEVT_CLEANUP which is called for *any*
exiting user-space task, all other notifications are only issued for
tasks bound to the real-time core (which may involve kthreads).

The notification is issued by a call to :c:func:`__ipipe_notify_kevent`
which in turn invokes the :c:func:`ipipe_kevent_hook` routine the
real-time core should override for receiving those events (*__weak*
binding). Interrupts are **enabled** in the CPU when
:c:func:`ipipe_kevent_hook` is called.

The notification hook is given the event type code, and a single
pointer argument which relates to the event type.

The following events are defined (include/linux/ipipe_domain.h):

- IPIPE_KEVT_SCHEDULE(struct task_struct *next)

  sent in preparation of a context switch, right before the memory
  context is switched to *next*.

- IPIPE_KEVT_SIGWAKE(struct task_struct *target)

  sent when *target* is about to receive a signal. The real-time core
  may decide to schedule a transition of the recipient to the root
  stage in order to have it handle that signal asap, which is commonly
  required for keeping the kernel sane. This notification is always
  sent from the context of the issuer.

- IPIPE_KEVT_SETAFFINITY(struct ipipe_migration_data *p)

  sent when p->task is about to move to CPU p->dest_cpu.

- IPIPE_KEVT_EXIT(struct task_struct *current)

  sent from :c:func:`do_exit` before the current task has dropped the
  files and mappings it owns.

- IPIPE_KEVT_CLEANUP(struct mm_struct *mm)

  sent before *mm* is entirely dropped, before the mappings are
  exited. Per-process resources which might be maintained by the
  real-time core could be released there, as all threads have exited.

  ..NOTE:: IPIPE_KEVT_SETSCHED is deprecated, and should not be used.

Prerequisites
=============

The interrupt pipeline requires the following features to be available
from the target kernel:

- Generic IRQ handling
- Clock event abstraction

Implementation
==============

The following kernel areas are involved in interrupt pipelining:

- Generic IRQ core

  * IRQ flow handlers

    Generic flow handlers acknowledge the incoming IRQ event in the
    hardware by calling the appropriate irqchip-specific
    handler. However, the generic flow_ handlers do not immediately
    invoke the in-band interrupt handlers, but leave this decision to
    the pipeline core which calls them, according to the pipelined
    delivery logic.

- Arch-specific bits

  * CPU interrupt mask handling

    The architecture-specific code which manipulates the interrupt
    flag in the CPU's state register
    (i.e. arch/<arch>/include/asm/irqflags.h) is split between real
    and virtual interrupt control:

    + the *hard_local_irq* level helpers affect the hardware state in
      the CPU.

    + the *arch_* level helpers affect the virtual interrupt flag_
      implemented by the pipeline core for controlling the root stage
      protection against interrupts.

    This means that generic helpers from <linux/irqflags.h> such as
    :c:func:`local_irq_disable` and :c:func:`local_irq_enable`
    actually refer to the virtual protection scheme when interrupts
    are pipelined, implementing interrupt deferral_ for the protected
    in-band code running over the root stage.

  * Assembly-level IRQ, exception paths

    Since interrupts are only virtually masked by the in-band code,
    IRQs can still be taken by the CPU although they should not be
    visible from the root stage when they happen in the following
    situations:

    + when the virtual protection flag_ is raised, meaning the root
      stage does not accept IRQs, in which case interrupt _deferral
      happens.

    + when the CPU runs out-of-band code, regardless of the state of
      the virtual protection flag.

    In both cases, the low-level assembly code handling incoming IRQs
    takes a fast exit path unwinding the interrupt frame early,
    instead of running the common in-band epilogue which checks for
    task rescheduling opportunities and pending signals.

    Likewise, the low-level fault/exception handling code also takes a
    fast exit path under the same circumstances. Typically, an
    out-of-band handler causing a minor page fault should benefit from
    a lightweight PTE fixup performed by the high-level fault handler,
    but is not allowed to traverse the rescheduling logic upon return
    from exception.

- Scheduler core

  * CPUIDLE support

    The logic of the CPUIDLE framework has to account for those
    specific issues the interrupt pipelining introduces:

    - the kernel might be idle in the sense that no in-band activity
    is scheduled yet, and planning to shut down the timer device
    suffering the C3STOP (mis)feature.  However, at the same time,
    some out-of-band code might wait for a tick event already
    programmed in the timer hardware controlled by some out-of-band
    code via the timer_ interposition mechanism.

    - switching the CPU to a power saving state may incur a
    significant latency, particularly for waking it up before it can
    handle an incoming IRQ, which is at odds with the purpose of
    interrupt pipelining.

    Obviously, we don't want the CPUIDLE logic to turn off the
    hardware timer when C3STOP is in effect for the timer device,
    which would cause the pending out-of-band event to be
    lost.

    Likewise, the wake up latency induced by entering a sleep state on
    a particular hardware may not always be acceptable.

    Since the in-band kernel code does not know about the out-of-band
    code plans by design, CPUIDLE calls :c:func:`ipipe_cpuidle_control`
    to figure out whether the out-of-band system is fine with entering
    the idle state as well.  This routine should be overriden by the
    out-of-band code for receiving such notification (*__weak*
    binding).

    If this hook returns a boolean *true* value, CPUIDLE proceeds as
    normally. Otherwise, the CPU is simply denied from entering the
    idle state, leaving the timer hardware enabled.

    ..CAUTION:: If some out-of-band code waiting for an external event
    cannot bear with the latency that might be induced by the default
    architecture-specific CPU idling code, then CPUIDLE is not usable
    and should be disabled at build time.

  * Kernel preemption control (PREEMPT)

    :c:func:`__preempt_schedule_irq` reconciles the virtual interrupt
    state - which has not been touched by the assembly level code upon
    kernel entry - with basic assumptions made by the scheduler core,
    such as entering with interrupts disabled. It should be called by
    the arch-specific assembly code in replacement of
    :c:func:`preempt_schedule_irq`, from the call site dealing with
    kernel preemption upon return from IRQ or system call.

- Timer management

  * Timer interposition

.. _timer:
    The timer interposition mechanism is designed for handing over
    control of the hardware tick device in use by the kernel to an
    out-of-band timing logic. Typically, a real-time co-kernel would
    make good use of this feature, for grabbing control over the timer
    hardware.

    Once some out-of-band logic has grabbed control over the timer
    device by calling :c:func:`ipipe_select_timers`, it can install
    its own out-of-band handlers using :c:func:`ipipe_timer_start`.
    From that point, it must carry out the timing requests from the
    in-band timer core (e.g. hrtimers) in addition to its own timing
    duties.

    In other words, once the interposition is set up, the
    functionality of the tick device is shared between the in-band and
    out-of-band contexts, with only the latter actually programming
    the hardware.

    This mechanism is based on the clock event abstraction (`struct
    clock_event_device`). Clock event devices which may be controlled
    by this way need their drivers to be specifically adapted for such
    use:

    + the interrupt handler receiving tick IRQs must be check with
    :c:func:`clockevent_ipipe_stolen` whether they actually control
    the hardware. A non-zero return from this routine means that it
    does not, and therefore should skip the timer acknowledge
    code, which would have run earlier in that case.

- Generic locking & atomic

  * Generic atomic ops

.. _atomic:
    The effect of virtualizing interrupt protection must be reversed
    for atomic helpers in <asm-generic/{atomic|bitops/atomic}.h> and
    <asm-generic/cmpxchg-local.h>, so that no interrupt can preempt
    their execution, regardless of the stage their caller live
    on.

    This is required to keep those helpers usable on data which
    might be accessed concurrently from both stages.

    The usual way to revert such virtualization consists of delimiting
    the protected section with :c:func:`hard_local_irq_save`,
    :c:func:`hard_local_irq_restore` calls, in replacement for
    :c:func:`local_irq_save`, :c:func:`local_irq_restore`
    respectively.

  * Hard spinlocks

    The pipeline core introduces one more spinlock type:

    + *hard* spinlocks manipulate the CPU interrupt mask, and don't
      affect the kernel preemption state in locking/unlocking
      operations.

      This type of spinlock is useful for implementing a critical
      section to serialize concurrent accesses from both in-band and
      out-of-band contexts, i.e. from root and head stages. Obviously,
      sleeping into a critical section protected by a hard spinlock
      would be a very bad idea.

      In other words, hard spinlocks are not subject to virtual
      interrupt masking, therefore can be used to serialize with
      out-of-band activities, including from the in-band kernel
      code. At any rate, those sections ought to be quite short, for
      keeping latency low.

- Drivers

  * IRQ chip drivers

    .. _irqchip:
    irqchip drivers need to be specifically adapted for supporting the
    pipelined interrupt model. The irqchip descriptor gains additional
    handlers:

    + irq_chip.irq_hold is an optional handler called by the pipeline
      core upon events from *level-triggered*, *fasteoi* and *percpu*
      types. See Holding_ interrupt lines.

      When specified in the descriptor, irq_chip.irq_hold should
      perform as follows, depending on the hardware acknowledge logic:

          + level   ->  mask[+ack]
          + percpu  ->  mask[+ack][+eoi]
          + fasteoi ->  mask+eoi

      .. CAUTION:: proper acknowledge and/or EOI is important when
                   holding a line, as those operations may also
                   decrease the current interrupt priority level for
                   the CPU, allowing same or lower priority
                   out-of-band interrupts to be taken while the
                   initial IRQ might be deferred_ for the root stage.

    + irq_chip.irq_release is the converse operation to
      irq_chip.irq_hold, releasing an interrupt line from the held
      state.

      The :c:func:`ipipe_end_irq` routine invokes the available
      handler for releasing the interrupt line. The pipeline core
      calls :c:func:`irq_release` automatically for each IRQ which has
      been accepted by an in-band handler (`IRQ_HANDLED` status). This
      routine should be called explicitly by out-of-band handlers
      before returning to their caller.

    `IRQCHIP_PIPELINE_SAFE` must be added to `struct irqchip::flags`
    member of a pipeline-aware irqchip driver.

    .. NOTE:: :c:func:`irq_set_chip` will complain loudly with a
              kernel warning whenever the irqchip descriptor passed
              does not bear the `IRQCHIP_PIPELINE_SAFE` flag and
              CONFIG_IPIPE is enabled.

- Misc

  * :c:func:`printk`

    :c:func:`printk` may be called by out-of-band code safely, without
    encurring extra latency. The output is delayed until the in-band
    code resumes, and the console driver(s) can handle it.

  * Tracing core

    Tracepoints can be traversed by out-of-band code safely. Dynamic
    tracing is available to a kernel running the pipelined interrupt
    model too.

Terminology
===========

.. _terminology:
======================   =======================================================
    Term                                       Definition
======================   =======================================================
Head stage               high-priority execution context trigged by out-of-band IRQs
Root stage               regular kernel context performing GPOS work
Out-of-band code         code running over the head stage
In-band code             code running over the root stage
Scheduler                the regular, Linux kernel scheduler
Real-time scheduler      the out-of-band task scheduling logic implemented on top of the I-pipe

Resources
=========

.. [#f1] Stodolsky, Chen & Bershad; "Fast Interrupt Priority Management in Operating System Kernels"
    https://www.usenix.org/legacy/publications/library/proceedings/micro93/full_papers/stodolsky.txt
.. [#f2] Yaghmour, Karim; "ADEOS - Adaptive Domain Environment for Operating Systems"
    https://www.opersys.com/ftp/pub/Adeos/adeos.pdf
