/*
 * Copyright (C) 2005 Philippe Gerum <rpm@xenomai.org>.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */
#ifndef _COBALT_UAPI_SYSCALL_H
#define _COBALT_UAPI_SYSCALL_H

#include <cobalt/uapi/asm-generic/syscall.h>

#define sc_cobalt_bind				0
#define sc_cobalt_thread_create			1
#define sc_cobalt_thread_getpid			2
#define sc_cobalt_thread_setmode		3
#define sc_cobalt_thread_setname		4
#define sc_cobalt_thread_join			5
#define sc_cobalt_thread_kill			6
#define sc_cobalt_thread_setschedparam_ex	7
#define sc_cobalt_thread_getschedparam_ex	8
#define sc_cobalt_thread_getstat		9
#define sc_cobalt_sem_init			10
#define sc_cobalt_sem_destroy			11
#define sc_cobalt_sem_post			12
#define sc_cobalt_sem_wait			13
#define sc_cobalt_sem_trywait			14
#define sc_cobalt_sem_getvalue			15
#define sc_cobalt_sem_open			16
#define sc_cobalt_sem_close			17
#define sc_cobalt_sem_unlink			18
#define sc_cobalt_sem_timedwait			19
#define sc_cobalt_sem_inquire			20
#define sc_cobalt_sem_broadcast_np		21
#define sc_cobalt_clock_getres			22
#define sc_cobalt_clock_gettime			23
#define sc_cobalt_clock_settime			24
#define sc_cobalt_clock_nanosleep		25
#define sc_cobalt_mutex_init			26
#define sc_cobalt_mutex_check_init		27
#define sc_cobalt_mutex_destroy			28
#define sc_cobalt_mutex_lock			29
#define sc_cobalt_mutex_timedlock		30
#define sc_cobalt_mutex_trylock			31
#define sc_cobalt_mutex_unlock			32
#define sc_cobalt_cond_init			33
#define sc_cobalt_cond_destroy			34
#define sc_cobalt_cond_wait_prologue		35
#define sc_cobalt_cond_wait_epilogue		36
#define sc_cobalt_mq_open			37
#define sc_cobalt_mq_close			38
#define sc_cobalt_mq_unlink			39
#define sc_cobalt_mq_getattr			40
#define sc_cobalt_mq_timedsend			41
#define sc_cobalt_mq_timedreceive		42
#define sc_cobalt_mq_notify			43
#define sc_cobalt_sched_minprio			44
#define sc_cobalt_sched_maxprio			45
#define sc_cobalt_sched_weightprio		46
#define sc_cobalt_sched_yield			47
#define sc_cobalt_sched_setscheduler_ex		48
#define sc_cobalt_sched_getscheduler_ex		49
#define sc_cobalt_sched_setconfig_np		50
#define sc_cobalt_sched_getconfig_np		51
#define sc_cobalt_timer_create			52
#define sc_cobalt_timer_delete			53
#define sc_cobalt_timer_settime			54
#define sc_cobalt_timer_gettime			55
#define sc_cobalt_timer_getoverrun		56
#define sc_cobalt_timerfd_create		57
#define sc_cobalt_timerfd_settime		58
#define sc_cobalt_timerfd_gettime		59
#define sc_cobalt_sigwait			60
#define sc_cobalt_sigwaitinfo			61
#define sc_cobalt_sigtimedwait			62
#define sc_cobalt_sigpending			63
#define sc_cobalt_kill				64
#define sc_cobalt_sigqueue			65
#define sc_cobalt_monitor_init			66
#define sc_cobalt_monitor_destroy		67
#define sc_cobalt_monitor_enter			68
#define sc_cobalt_monitor_wait			69
#define sc_cobalt_monitor_sync			70
#define sc_cobalt_monitor_exit			71
#define sc_cobalt_event_init			72
#define sc_cobalt_event_wait			73
#define sc_cobalt_event_sync			74
#define sc_cobalt_event_destroy			75
#define sc_cobalt_event_inquire			76
#define sc_cobalt_open				77
#define sc_cobalt_socket			78
#define sc_cobalt_close				79
#define sc_cobalt_ioctl				80
#define sc_cobalt_read				81
#define sc_cobalt_write				82
#define sc_cobalt_recvmsg			83
#define sc_cobalt_sendmsg			84
#define sc_cobalt_mmap				85
#define sc_cobalt_select			86
#define sc_cobalt_fcntl				87
#define sc_cobalt_migrate			88
#define sc_cobalt_archcall			89
#define sc_cobalt_trace				90
#define sc_cobalt_corectl			91
#define sc_cobalt_get_current			92
/* 93: formerly mayday */
#define sc_cobalt_backtrace			94
#define sc_cobalt_serialdbg			95
#define sc_cobalt_extend			96
#define sc_cobalt_ftrace_puts			97
#define sc_cobalt_recvmmsg			98
#define sc_cobalt_sendmmsg			99
#define sc_cobalt_clock_adjtime			100
#define sc_cobalt_thread_setschedprio		101

#define __NR_COBALT_SYSCALLS			128 /* Power of 2 */

#endif /* !_COBALT_UAPI_SYSCALL_H */
