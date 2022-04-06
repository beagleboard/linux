/*
 * Contains CPU specific errata definitions
 *
 * Copyright (C) 2014 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/arm-smccc.h>
#include <linux/psci.h>
#include <linux/types.h>
#include <asm/cachetype.h>
#include <asm/cpu.h>
#include <asm/cputype.h>
#include <asm/cpufeature.h>
#include <asm/vectors.h>

static bool __maybe_unused
is_affected_midr_range(const struct arm64_cpu_capabilities *entry, int scope)
{
	u32 midr = read_cpuid_id();

	WARN_ON(scope != SCOPE_LOCAL_CPU || preemptible());
	return is_midr_in_range(midr, &entry->midr_range);
}

static bool __maybe_unused
is_affected_midr_range_list(const struct arm64_cpu_capabilities *entry,
			    int scope)
{
	WARN_ON(scope != SCOPE_LOCAL_CPU || preemptible());
	return is_midr_in_range_list(read_cpuid_id(), entry->midr_range_list);
}

static bool
has_mismatched_cache_type(const struct arm64_cpu_capabilities *entry,
			  int scope)
{
	u64 mask = CTR_CACHE_MINLINE_MASK;

	/* Skip matching the min line sizes for cache type check */
	if (entry->capability == ARM64_MISMATCHED_CACHE_TYPE)
		mask ^= arm64_ftr_reg_ctrel0.strict_mask;

	WARN_ON(scope != SCOPE_LOCAL_CPU || preemptible());
	return (read_cpuid_cachetype() & mask) !=
	       (arm64_ftr_reg_ctrel0.sys_val & mask);
}

static void
cpu_enable_trap_ctr_access(const struct arm64_cpu_capabilities *__unused)
{
	/* Clear SCTLR_EL1.UCT */
	config_sctlr_el1(SCTLR_EL1_UCT, 0);
}

#ifdef CONFIG_HARDEN_BRANCH_PREDICTOR
#include <asm/mmu_context.h>
#include <asm/cacheflush.h>

static bool __hardenbp_enab;
DEFINE_PER_CPU_READ_MOSTLY(struct bp_hardening_data, bp_hardening_data);

#ifdef CONFIG_KVM
extern char __smccc_workaround_1_smc_start[];
extern char __smccc_workaround_1_smc_end[];
extern char __smccc_workaround_1_hvc_start[];
extern char __smccc_workaround_1_hvc_end[];
extern char __smccc_workaround_3_smc_start[];
extern char __smccc_workaround_3_smc_end[];
extern char __spectre_bhb_loop_k8_start[];
extern char __spectre_bhb_loop_k8_end[];
extern char __spectre_bhb_loop_k24_start[];
extern char __spectre_bhb_loop_k24_end[];
extern char __spectre_bhb_loop_k32_start[];
extern char __spectre_bhb_loop_k32_end[];
extern char __spectre_bhb_clearbhb_start[];
extern char __spectre_bhb_clearbhb_end[];

static void __copy_hyp_vect_bpi(int slot, const char *hyp_vecs_start,
				const char *hyp_vecs_end)
{
	void *dst = __bp_harden_hyp_vecs_start + slot * SZ_2K;
	int i;

	for (i = 0; i < SZ_2K; i += 0x80)
		memcpy(dst + i, hyp_vecs_start, hyp_vecs_end - hyp_vecs_start);

	flush_icache_range((uintptr_t)dst, (uintptr_t)dst + SZ_2K);
}

static DEFINE_SPINLOCK(bp_lock);
static int last_slot = -1;

static void __install_bp_hardening_cb(bp_hardening_cb_t fn,
				      const char *hyp_vecs_start,
				      const char *hyp_vecs_end)
{

	int cpu, slot = -1;

	spin_lock(&bp_lock);
	for_each_possible_cpu(cpu) {
		if (per_cpu(bp_hardening_data.fn, cpu) == fn) {
			slot = per_cpu(bp_hardening_data.hyp_vectors_slot, cpu);
			break;
		}
	}

	if (slot == -1) {
		last_slot++;
		BUG_ON(((__bp_harden_hyp_vecs_end - __bp_harden_hyp_vecs_start)
			/ SZ_2K) <= last_slot);
		slot = last_slot;
		__copy_hyp_vect_bpi(slot, hyp_vecs_start, hyp_vecs_end);
	}

	__this_cpu_write(bp_hardening_data.hyp_vectors_slot, slot);
	__this_cpu_write(bp_hardening_data.fn, fn);
	__this_cpu_write(bp_hardening_data.template_start, hyp_vecs_start);
	__hardenbp_enab = true;
	spin_unlock(&bp_lock);
}
#else
#define __smccc_workaround_1_smc_start		NULL
#define __smccc_workaround_1_smc_end		NULL
#define __smccc_workaround_1_hvc_start		NULL
#define __smccc_workaround_1_hvc_end		NULL

static void __install_bp_hardening_cb(bp_hardening_cb_t fn,
				      const char *hyp_vecs_start,
				      const char *hyp_vecs_end)
{
	__this_cpu_write(bp_hardening_data.fn, fn);
	__hardenbp_enab = true;
}
#endif	/* CONFIG_KVM */

static void  install_bp_hardening_cb(const struct arm64_cpu_capabilities *entry,
				     bp_hardening_cb_t fn,
				     const char *hyp_vecs_start,
				     const char *hyp_vecs_end)
{
	u64 pfr0;

	if (!entry->matches(entry, SCOPE_LOCAL_CPU))
		return;

	pfr0 = read_cpuid(ID_AA64PFR0_EL1);
	if (cpuid_feature_extract_unsigned_field(pfr0, ID_AA64PFR0_CSV2_SHIFT))
		return;

	__install_bp_hardening_cb(fn, hyp_vecs_start, hyp_vecs_end);
}

#include <uapi/linux/psci.h>
#include <linux/arm-smccc.h>
#include <linux/psci.h>

static void call_smc_arch_workaround_1(void)
{
	arm_smccc_1_1_smc(ARM_SMCCC_ARCH_WORKAROUND_1, NULL);
}

static void call_hvc_arch_workaround_1(void)
{
	arm_smccc_1_1_hvc(ARM_SMCCC_ARCH_WORKAROUND_1, NULL);
}

static void
enable_smccc_arch_workaround_1(const struct arm64_cpu_capabilities *entry)
{
	bp_hardening_cb_t cb;
	void *smccc_start, *smccc_end;
	struct arm_smccc_res res;

	if (!entry->matches(entry, SCOPE_LOCAL_CPU))
		return;

	if (psci_ops.smccc_version == SMCCC_VERSION_1_0)
		return;

	switch (psci_ops.conduit) {
	case PSCI_CONDUIT_HVC:
		arm_smccc_1_1_hvc(ARM_SMCCC_ARCH_FEATURES_FUNC_ID,
				  ARM_SMCCC_ARCH_WORKAROUND_1, &res);
		if ((int)res.a0 < 0)
			return;
		cb = call_hvc_arch_workaround_1;
		smccc_start = __smccc_workaround_1_hvc_start;
		smccc_end = __smccc_workaround_1_hvc_end;
		break;

	case PSCI_CONDUIT_SMC:
		arm_smccc_1_1_smc(ARM_SMCCC_ARCH_FEATURES_FUNC_ID,
				  ARM_SMCCC_ARCH_WORKAROUND_1, &res);
		if ((int)res.a0 < 0)
			return;
		cb = call_smc_arch_workaround_1;
		smccc_start = __smccc_workaround_1_smc_start;
		smccc_end = __smccc_workaround_1_smc_end;
		break;

	default:
		return;
	}

	install_bp_hardening_cb(entry, cb, smccc_start, smccc_end);

	return;
}
#endif	/* CONFIG_HARDEN_BRANCH_PREDICTOR */

void __init arm64_update_smccc_conduit(struct alt_instr *alt,
				       __le32 *origptr, __le32 *updptr,
				       int nr_inst)
{
	u32 insn;

	BUG_ON(nr_inst != 1);

	switch (psci_ops.conduit) {
	case PSCI_CONDUIT_HVC:
		insn = aarch64_insn_get_hvc_value();
		break;
	case PSCI_CONDUIT_SMC:
		insn = aarch64_insn_get_smc_value();
		break;
	default:
		return;
	}

	*updptr = cpu_to_le32(insn);
}

#ifdef CONFIG_ARM64_SSBD
DEFINE_PER_CPU_READ_MOSTLY(u64, arm64_ssbd_callback_required);

int ssbd_state __read_mostly = ARM64_SSBD_KERNEL;

static const struct ssbd_options {
	const char	*str;
	int		state;
} ssbd_options[] = {
	{ "force-on",	ARM64_SSBD_FORCE_ENABLE, },
	{ "force-off",	ARM64_SSBD_FORCE_DISABLE, },
	{ "kernel",	ARM64_SSBD_KERNEL, },
};

static int __init ssbd_cfg(char *buf)
{
	int i;

	if (!buf || !buf[0])
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ssbd_options); i++) {
		int len = strlen(ssbd_options[i].str);

		if (strncmp(buf, ssbd_options[i].str, len))
			continue;

		ssbd_state = ssbd_options[i].state;
		return 0;
	}

	return -EINVAL;
}
early_param("ssbd", ssbd_cfg);

void __init arm64_enable_wa2_handling(struct alt_instr *alt,
				      __le32 *origptr, __le32 *updptr,
				      int nr_inst)
{
	BUG_ON(nr_inst != 1);
	/*
	 * Only allow mitigation on EL1 entry/exit and guest
	 * ARCH_WORKAROUND_2 handling if the SSBD state allows it to
	 * be flipped.
	 */
	if (arm64_get_ssbd_state() == ARM64_SSBD_KERNEL)
		*updptr = cpu_to_le32(aarch64_insn_gen_nop());
}

void arm64_set_ssbd_mitigation(bool state)
{
	switch (psci_ops.conduit) {
	case PSCI_CONDUIT_HVC:
		arm_smccc_1_1_hvc(ARM_SMCCC_ARCH_WORKAROUND_2, state, NULL);
		break;

	case PSCI_CONDUIT_SMC:
		arm_smccc_1_1_smc(ARM_SMCCC_ARCH_WORKAROUND_2, state, NULL);
		break;

	default:
		WARN_ON_ONCE(1);
		break;
	}
}

static bool has_ssbd_mitigation(const struct arm64_cpu_capabilities *entry,
				    int scope)
{
	struct arm_smccc_res res;
	bool required = true;
	s32 val;

	WARN_ON(scope != SCOPE_LOCAL_CPU || preemptible());

	if (psci_ops.smccc_version == SMCCC_VERSION_1_0) {
		ssbd_state = ARM64_SSBD_UNKNOWN;
		return false;
	}

	switch (psci_ops.conduit) {
	case PSCI_CONDUIT_HVC:
		arm_smccc_1_1_hvc(ARM_SMCCC_ARCH_FEATURES_FUNC_ID,
				  ARM_SMCCC_ARCH_WORKAROUND_2, &res);
		break;

	case PSCI_CONDUIT_SMC:
		arm_smccc_1_1_smc(ARM_SMCCC_ARCH_FEATURES_FUNC_ID,
				  ARM_SMCCC_ARCH_WORKAROUND_2, &res);
		break;

	default:
		ssbd_state = ARM64_SSBD_UNKNOWN;
		return false;
	}

	val = (s32)res.a0;

	switch (val) {
	case SMCCC_RET_NOT_SUPPORTED:
		ssbd_state = ARM64_SSBD_UNKNOWN;
		return false;

	case SMCCC_RET_NOT_REQUIRED:
		pr_info_once("%s mitigation not required\n", entry->desc);
		ssbd_state = ARM64_SSBD_MITIGATED;
		return false;

	case SMCCC_RET_SUCCESS:
		required = true;
		break;

	case 1:	/* Mitigation not required on this CPU */
		required = false;
		break;

	default:
		WARN_ON(1);
		return false;
	}

	switch (ssbd_state) {
	case ARM64_SSBD_FORCE_DISABLE:
		pr_info_once("%s disabled from command-line\n", entry->desc);
		arm64_set_ssbd_mitigation(false);
		required = false;
		break;

	case ARM64_SSBD_KERNEL:
		if (required) {
			__this_cpu_write(arm64_ssbd_callback_required, 1);
			arm64_set_ssbd_mitigation(true);
		}
		break;

	case ARM64_SSBD_FORCE_ENABLE:
		pr_info_once("%s forced from command-line\n", entry->desc);
		arm64_set_ssbd_mitigation(true);
		required = true;
		break;

	default:
		WARN_ON(1);
		break;
	}

	return required;
}
#endif	/* CONFIG_ARM64_SSBD */

#define CAP_MIDR_RANGE(model, v_min, r_min, v_max, r_max)	\
	.matches = is_affected_midr_range,			\
	.midr_range = MIDR_RANGE(model, v_min, r_min, v_max, r_max)

#define CAP_MIDR_ALL_VERSIONS(model)					\
	.matches = is_affected_midr_range,				\
	.midr_range = MIDR_ALL_VERSIONS(model)

#define MIDR_FIXED(rev, revidr_mask) \
	.fixed_revs = (struct arm64_midr_revidr[]){{ (rev), (revidr_mask) }, {}}

#define ERRATA_MIDR_RANGE(model, v_min, r_min, v_max, r_max)		\
	.type = ARM64_CPUCAP_LOCAL_CPU_ERRATUM,				\
	CAP_MIDR_RANGE(model, v_min, r_min, v_max, r_max)

#define CAP_MIDR_RANGE_LIST(list)				\
	.matches = is_affected_midr_range_list,			\
	.midr_range_list = list

/* Errata affecting a range of revisions of  given model variant */
#define ERRATA_MIDR_REV_RANGE(m, var, r_min, r_max)	 \
	ERRATA_MIDR_RANGE(m, var, r_min, var, r_max)

/* Errata affecting a single variant/revision of a model */
#define ERRATA_MIDR_REV(model, var, rev)	\
	ERRATA_MIDR_RANGE(model, var, rev, var, rev)

/* Errata affecting all variants/revisions of a given a model */
#define ERRATA_MIDR_ALL_VERSIONS(model)				\
	.type = ARM64_CPUCAP_LOCAL_CPU_ERRATUM,			\
	CAP_MIDR_ALL_VERSIONS(model)

/* Errata affecting a list of midr ranges, with same work around */
#define ERRATA_MIDR_RANGE_LIST(midr_list)			\
	.type = ARM64_CPUCAP_LOCAL_CPU_ERRATUM,			\
	CAP_MIDR_RANGE_LIST(midr_list)

#ifdef CONFIG_HARDEN_BRANCH_PREDICTOR

/*
 * List of CPUs where we need to issue a psci call to
 * harden the branch predictor.
 */
static const struct midr_range arm64_bp_harden_smccc_cpus[] = {
	MIDR_ALL_VERSIONS(MIDR_CORTEX_A57),
	MIDR_ALL_VERSIONS(MIDR_CORTEX_A72),
	MIDR_ALL_VERSIONS(MIDR_CORTEX_A73),
	MIDR_ALL_VERSIONS(MIDR_CORTEX_A75),
	MIDR_ALL_VERSIONS(MIDR_BRCM_VULCAN),
	MIDR_ALL_VERSIONS(MIDR_CAVIUM_THUNDERX2),
	{},
};

#endif

const struct arm64_cpu_capabilities arm64_errata[] = {
#if	defined(CONFIG_ARM64_ERRATUM_826319) || \
	defined(CONFIG_ARM64_ERRATUM_827319) || \
	defined(CONFIG_ARM64_ERRATUM_824069)
	{
	/* Cortex-A53 r0p[012] */
		.desc = "ARM errata 826319, 827319, 824069",
		.capability = ARM64_WORKAROUND_CLEAN_CACHE,
		ERRATA_MIDR_REV_RANGE(MIDR_CORTEX_A53, 0, 0, 2),
		.cpu_enable = cpu_enable_cache_maint_trap,
	},
#endif
#ifdef CONFIG_ARM64_ERRATUM_819472
	{
	/* Cortex-A53 r0p[01] */
		.desc = "ARM errata 819472",
		.capability = ARM64_WORKAROUND_CLEAN_CACHE,
		ERRATA_MIDR_REV_RANGE(MIDR_CORTEX_A53, 0, 0, 1),
		.cpu_enable = cpu_enable_cache_maint_trap,
	},
#endif
#ifdef CONFIG_ARM64_ERRATUM_832075
	{
	/* Cortex-A57 r0p0 - r1p2 */
		.desc = "ARM erratum 832075",
		.capability = ARM64_WORKAROUND_DEVICE_LOAD_ACQUIRE,
		ERRATA_MIDR_RANGE(MIDR_CORTEX_A57,
				  0, 0,
				  1, 2),
	},
#endif
#ifdef CONFIG_ARM64_ERRATUM_834220
	{
	/* Cortex-A57 r0p0 - r1p2 */
		.desc = "ARM erratum 834220",
		.capability = ARM64_WORKAROUND_834220,
		ERRATA_MIDR_RANGE(MIDR_CORTEX_A57,
				  0, 0,
				  1, 2),
	},
#endif
#ifdef CONFIG_ARM64_ERRATUM_845719
	{
	/* Cortex-A53 r0p[01234] */
		.desc = "ARM erratum 845719",
		.capability = ARM64_WORKAROUND_845719,
		ERRATA_MIDR_REV_RANGE(MIDR_CORTEX_A53, 0, 0, 4),
	},
#endif
#ifdef CONFIG_CAVIUM_ERRATUM_23154
	{
	/* Cavium ThunderX, pass 1.x */
		.desc = "Cavium erratum 23154",
		.capability = ARM64_WORKAROUND_CAVIUM_23154,
		ERRATA_MIDR_REV_RANGE(MIDR_THUNDERX, 0, 0, 1),
	},
#endif
#ifdef CONFIG_CAVIUM_ERRATUM_27456
	{
	/* Cavium ThunderX, T88 pass 1.x - 2.1 */
		.desc = "Cavium erratum 27456",
		.capability = ARM64_WORKAROUND_CAVIUM_27456,
		ERRATA_MIDR_RANGE(MIDR_THUNDERX,
				  0, 0,
				  1, 1),
	},
	{
	/* Cavium ThunderX, T81 pass 1.0 */
		.desc = "Cavium erratum 27456",
		.capability = ARM64_WORKAROUND_CAVIUM_27456,
		ERRATA_MIDR_REV(MIDR_THUNDERX_81XX, 0, 0),
	},
#endif
	{
		.desc = "Mismatched cache line size",
		.capability = ARM64_MISMATCHED_CACHE_LINE_SIZE,
		.matches = has_mismatched_cache_type,
		.type = ARM64_CPUCAP_LOCAL_CPU_ERRATUM,
		.cpu_enable = cpu_enable_trap_ctr_access,
	},
	{
		.desc = "Mismatched cache type",
		.capability = ARM64_MISMATCHED_CACHE_TYPE,
		.matches = has_mismatched_cache_type,
		.type = ARM64_CPUCAP_LOCAL_CPU_ERRATUM,
		.cpu_enable = cpu_enable_trap_ctr_access,
	},
#ifdef CONFIG_HARDEN_BRANCH_PREDICTOR
	{
		.capability = ARM64_HARDEN_BRANCH_PREDICTOR,
		ERRATA_MIDR_RANGE_LIST(arm64_bp_harden_smccc_cpus),
		.cpu_enable = enable_smccc_arch_workaround_1,
	},
#endif
#ifdef CONFIG_ARM64_SSBD
	{
		.desc = "Speculative Store Bypass Disable",
		.type = ARM64_CPUCAP_LOCAL_CPU_ERRATUM,
		.capability = ARM64_SSBD,
		.matches = has_ssbd_mitigation,
	},
#endif
#ifdef CONFIG_ARM64_ERRATUM_1188873
	{
		/* Cortex-A76 r0p0 to r2p0 */
		.desc = "ARM erratum 1188873",
		.capability = ARM64_WORKAROUND_1188873,
		ERRATA_MIDR_RANGE(MIDR_CORTEX_A76, 0, 0, 2, 0),
	},
#endif
	{
		.desc = "Spectre-BHB",
		.type = ARM64_CPUCAP_LOCAL_CPU_ERRATUM,
		.capability = ARM64_SPECTRE_BHB,
		.matches = is_spectre_bhb_affected,
#ifdef CONFIG_MITIGATE_SPECTRE_BRANCH_HISTORY
		.cpu_enable = spectre_bhb_enable_mitigation,
#endif
	},
	{
	}
};

/*
 * We try to ensure that the mitigation state can never change as the result of
 * onlining a late CPU.
 */
static void __maybe_unused update_mitigation_state(enum mitigation_state *oldp,
						   enum mitigation_state new)
{
	enum mitigation_state state;

	do {
		state = READ_ONCE(*oldp);
		if (new <= state)
			break;
	} while (cmpxchg_relaxed(oldp, state, new) != state);
}

/*
 * Spectre BHB.
 *
 * A CPU is either:
 * - Mitigated by a branchy loop a CPU specific number of times, and listed
 *   in our "loop mitigated list".
 * - Mitigated in software by the firmware Spectre v2 call.
 * - Has the ClearBHB instruction to perform the mitigation.
 * - Has the 'Exception Clears Branch History Buffer' (ECBHB) feature, so no
 *   software mitigation in the vectors is needed.
 * - Has CSV2.3, so is unaffected.
 */
static enum mitigation_state spectre_bhb_state;

enum mitigation_state arm64_get_spectre_bhb_state(void)
{
	return spectre_bhb_state;
}

/*
 * This must be called with SCOPE_LOCAL_CPU for each type of CPU, before any
 * SCOPE_SYSTEM call will give the right answer.
 */
u8 spectre_bhb_loop_affected(int scope)
{
	u8 k = 0;
	static u8 max_bhb_k;

	if (scope == SCOPE_LOCAL_CPU) {
		static const struct midr_range spectre_bhb_k32_list[] = {
			MIDR_ALL_VERSIONS(MIDR_CORTEX_A78),
			MIDR_ALL_VERSIONS(MIDR_CORTEX_A78C),
			MIDR_ALL_VERSIONS(MIDR_CORTEX_X1),
			MIDR_ALL_VERSIONS(MIDR_CORTEX_A710),
			MIDR_ALL_VERSIONS(MIDR_CORTEX_X2),
			MIDR_ALL_VERSIONS(MIDR_NEOVERSE_N2),
			MIDR_ALL_VERSIONS(MIDR_NEOVERSE_V1),
			{},
		};
		static const struct midr_range spectre_bhb_k24_list[] = {
			MIDR_ALL_VERSIONS(MIDR_CORTEX_A77),
			MIDR_ALL_VERSIONS(MIDR_CORTEX_A76),
			MIDR_ALL_VERSIONS(MIDR_NEOVERSE_N1),
			{},
		};
		static const struct midr_range spectre_bhb_k8_list[] = {
			MIDR_ALL_VERSIONS(MIDR_CORTEX_A72),
			MIDR_ALL_VERSIONS(MIDR_CORTEX_A57),
			{},
		};

		if (is_midr_in_range_list(read_cpuid_id(), spectre_bhb_k32_list))
			k = 32;
		else if (is_midr_in_range_list(read_cpuid_id(), spectre_bhb_k24_list))
			k = 24;
		else if (is_midr_in_range_list(read_cpuid_id(), spectre_bhb_k8_list))
			k =  8;

		max_bhb_k = max(max_bhb_k, k);
	} else {
		k = max_bhb_k;
	}

	return k;
}

static enum mitigation_state spectre_bhb_get_cpu_fw_mitigation_state(void)
{
	int ret;
	struct arm_smccc_res res;

	if (psci_ops.smccc_version == SMCCC_VERSION_1_0)
		return SPECTRE_VULNERABLE;

	switch (psci_ops.conduit) {
	case PSCI_CONDUIT_HVC:
		arm_smccc_1_1_hvc(ARM_SMCCC_ARCH_FEATURES_FUNC_ID,
				  ARM_SMCCC_ARCH_WORKAROUND_3, &res);
		break;

	case PSCI_CONDUIT_SMC:
		arm_smccc_1_1_smc(ARM_SMCCC_ARCH_FEATURES_FUNC_ID,
				  ARM_SMCCC_ARCH_WORKAROUND_3, &res);
		break;

	default:
		return SPECTRE_VULNERABLE;
	}

	ret = res.a0;
	switch (ret) {
	case SMCCC_RET_SUCCESS:
		return SPECTRE_MITIGATED;
	case SMCCC_ARCH_WORKAROUND_RET_UNAFFECTED:
		return SPECTRE_UNAFFECTED;
	default:
	case SMCCC_RET_NOT_SUPPORTED:
		return SPECTRE_VULNERABLE;
	}
}

static bool is_spectre_bhb_fw_affected(int scope)
{
	static bool system_affected;
	enum mitigation_state fw_state;
	bool has_smccc = (psci_ops.smccc_version >= SMCCC_VERSION_1_1);
	static const struct midr_range spectre_bhb_firmware_mitigated_list[] = {
		MIDR_ALL_VERSIONS(MIDR_CORTEX_A73),
		MIDR_ALL_VERSIONS(MIDR_CORTEX_A75),
		{},
	};
	bool cpu_in_list = is_midr_in_range_list(read_cpuid_id(),
					 spectre_bhb_firmware_mitigated_list);

	if (scope != SCOPE_LOCAL_CPU)
		return system_affected;

	fw_state = spectre_bhb_get_cpu_fw_mitigation_state();
	if (cpu_in_list || (has_smccc && fw_state == SPECTRE_MITIGATED)) {
		system_affected = true;
		return true;
	}

	return false;
}

static bool __maybe_unused supports_ecbhb(int scope)
{
	u64 mmfr1;

	if (scope == SCOPE_LOCAL_CPU)
		mmfr1 = read_sysreg_s(SYS_ID_AA64MMFR1_EL1);
	else
		mmfr1 = read_system_reg(SYS_ID_AA64MMFR1_EL1);

	return cpuid_feature_extract_unsigned_field(mmfr1,
						    ID_AA64MMFR1_ECBHB_SHIFT);
}

bool is_spectre_bhb_affected(const struct arm64_cpu_capabilities *entry,
			     int scope)
{
	WARN_ON(scope != SCOPE_LOCAL_CPU || preemptible());

	if (supports_csv2p3(scope))
		return false;

	if (supports_clearbhb(scope))
		return true;

	if (spectre_bhb_loop_affected(scope))
		return true;

	if (is_spectre_bhb_fw_affected(scope))
		return true;

	return false;
}

#ifdef CONFIG_HARDEN_BRANCH_PREDICTOR
static void this_cpu_set_vectors(enum arm64_bp_harden_el1_vectors slot)
{
	const char *v = arm64_get_bp_hardening_vector(slot);

	if (slot < 0)
		return;

	__this_cpu_write(this_cpu_vector, v);

	/*
	 * When KPTI is in use, the vectors are switched when exiting to
	 * user-space.
	 */
	if (arm64_kernel_unmapped_at_el0())
		return;

	write_sysreg(v, vbar_el1);
	isb();
}

#ifdef CONFIG_KVM
static const char *kvm_bhb_get_vecs_end(const char *start)
{
	if (start == __smccc_workaround_3_smc_start)
		return __smccc_workaround_3_smc_end;
	else if (start == __spectre_bhb_loop_k8_start)
		return __spectre_bhb_loop_k8_end;
	else if (start == __spectre_bhb_loop_k24_start)
		return __spectre_bhb_loop_k24_end;
	else if (start == __spectre_bhb_loop_k32_start)
		return __spectre_bhb_loop_k32_end;
	else if (start == __spectre_bhb_clearbhb_start)
		return __spectre_bhb_clearbhb_end;

	return NULL;
}

static void kvm_setup_bhb_slot(const char *hyp_vecs_start)
{
	int cpu, slot = -1;
	const char *hyp_vecs_end;

	if (!IS_ENABLED(CONFIG_KVM) || !is_hyp_mode_available())
		return;

	hyp_vecs_end = kvm_bhb_get_vecs_end(hyp_vecs_start);
	if (WARN_ON_ONCE(!hyp_vecs_start || !hyp_vecs_end))
		return;

	spin_lock(&bp_lock);
	for_each_possible_cpu(cpu) {
		if (per_cpu(bp_hardening_data.template_start, cpu) == hyp_vecs_start) {
			slot = per_cpu(bp_hardening_data.hyp_vectors_slot, cpu);
			break;
		}
	}

	if (slot == -1) {
		last_slot++;
		BUG_ON(((__bp_harden_hyp_vecs_end - __bp_harden_hyp_vecs_start)
			/ SZ_2K) <= last_slot);
		slot = last_slot;
		__copy_hyp_vect_bpi(slot, hyp_vecs_start, hyp_vecs_end);
	}

	__this_cpu_write(bp_hardening_data.hyp_vectors_slot, slot);
	__this_cpu_write(bp_hardening_data.template_start, hyp_vecs_start);
	spin_unlock(&bp_lock);
}
#else
#define __smccc_workaround_3_smc_start NULL
#define __spectre_bhb_loop_k8_start NULL
#define __spectre_bhb_loop_k24_start NULL
#define __spectre_bhb_loop_k32_start NULL
#define __spectre_bhb_clearbhb_start NULL

static void kvm_setup_bhb_slot(const char *hyp_vecs_start) { };
#endif /* CONFIG_KVM */

static bool is_spectrev2_safe(void)
{
	return !is_midr_in_range_list(read_cpuid_id(),
				      arm64_bp_harden_smccc_cpus);
}

void spectre_bhb_enable_mitigation(const struct arm64_cpu_capabilities *entry)
{
	enum mitigation_state fw_state, state = SPECTRE_VULNERABLE;

	if (!is_spectre_bhb_affected(entry, SCOPE_LOCAL_CPU))
		return;

	if (!is_spectrev2_safe() &&  !__hardenbp_enab) {
		/* No point mitigating Spectre-BHB alone. */
	} else if (!IS_ENABLED(CONFIG_MITIGATE_SPECTRE_BRANCH_HISTORY)) {
		pr_info_once("spectre-bhb mitigation disabled by compile time option\n");
	} else if (cpu_mitigations_off()) {
		pr_info_once("spectre-bhb mitigation disabled by command line option\n");
	} else if (supports_ecbhb(SCOPE_LOCAL_CPU)) {
		state = SPECTRE_MITIGATED;
	} else if (supports_clearbhb(SCOPE_LOCAL_CPU)) {
		kvm_setup_bhb_slot(__spectre_bhb_clearbhb_start);
		this_cpu_set_vectors(EL1_VECTOR_BHB_CLEAR_INSN);

		state = SPECTRE_MITIGATED;
	} else if (spectre_bhb_loop_affected(SCOPE_LOCAL_CPU)) {
		switch (spectre_bhb_loop_affected(SCOPE_SYSTEM)) {
		case 8:
			kvm_setup_bhb_slot(__spectre_bhb_loop_k8_start);
			break;
		case 24:
			kvm_setup_bhb_slot(__spectre_bhb_loop_k24_start);
			break;
		case 32:
			kvm_setup_bhb_slot(__spectre_bhb_loop_k32_start);
			break;
		default:
			WARN_ON_ONCE(1);
		}
		this_cpu_set_vectors(EL1_VECTOR_BHB_LOOP);

		state = SPECTRE_MITIGATED;
	} else if (is_spectre_bhb_fw_affected(SCOPE_LOCAL_CPU)) {
		fw_state = spectre_bhb_get_cpu_fw_mitigation_state();
		if (fw_state == SPECTRE_MITIGATED) {
			kvm_setup_bhb_slot(__smccc_workaround_3_smc_start);
			this_cpu_set_vectors(EL1_VECTOR_BHB_FW);

			/*
			 * With WA3 in the vectors, the WA1 calls can be
			 * removed.
			 */
			__this_cpu_write(bp_hardening_data.fn, NULL);

			state = SPECTRE_MITIGATED;
		}
	}

	update_mitigation_state(&spectre_bhb_state, state);
}

/* Patched to correct the immediate */
void __init spectre_bhb_patch_loop_iter(struct alt_instr *alt,
					__le32 *origptr, __le32 *updptr, int nr_inst)
{
	u8 rd;
	u32 insn;
	u16 loop_count = spectre_bhb_loop_affected(SCOPE_SYSTEM);

	BUG_ON(nr_inst != 1); /* MOV -> MOV */

	if (!IS_ENABLED(CONFIG_MITIGATE_SPECTRE_BRANCH_HISTORY))
		return;

	insn = le32_to_cpu(*origptr);
	rd = aarch64_insn_decode_register(AARCH64_INSN_REGTYPE_RD, insn);
	insn = aarch64_insn_gen_movewide(rd, loop_count, 0,
					 AARCH64_INSN_VARIANT_64BIT,
					 AARCH64_INSN_MOVEWIDE_ZERO);
	*updptr++ = cpu_to_le32(insn);
}
#endif /* CONFIG_HARDEN_BRANCH_PREDICTOR */
