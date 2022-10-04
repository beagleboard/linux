/*============================================================================

Copyright (c) 2010-2017, The Regents of the University of California
(Regents).  All Rights Reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. Neither the name of the Regents nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT,
SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING
OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS
BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

=============================================================================*/

#include <linux/uaccess.h>
#include <asm/switch_to.h>
#include <asm/csr.h>
#include "riscv_decode.h"
#include "arith.h"
#define vector_elt(type) \
        type * vector_elt_##type(vector_state* vector, reg_t vReg, reg_t n){ \
          reg_t elts_per_reg;\
          type *regStart;\
          require(vector->vsew != 0); \
          require((vector->VLEN >> 3)/sizeof(type) > 0); \
          elts_per_reg = (vector->VLEN >> 3) / (sizeof(type)); \
          vReg += n / elts_per_reg; \
          n = n % elts_per_reg; \
          regStart = (type*)((char*)vector->regs + vReg * (vector->VLEN >> 3)); \
          return &regStart[n]; \
        } \

vector_elt(float32_t)
vector_elt(float64_t)
vector_elt(int8_t)
vector_elt(int16_t)
vector_elt(int32_t)
vector_elt(int64_t)
vector_elt(uint8_t)
vector_elt(uint16_t)
vector_elt(uint32_t)
vector_elt(uint64_t)

#define load_store(type) \
      type##_t load_##type(reg_t addr) { \
          type##_t tmp; \
          type##_t __user *in; \
          long err; \
          in = (type##_t __user *)addr; \
          err = __copy_from_user(&tmp, in , sizeof(type##_t)); \
	      if (unlikely(err)) \
		      panic("load memory error\n"); \
          return tmp; \
      } \
      \
      void store_##type(reg_t addr, type##_t val) { \
          type##_t __user *out; \
          long err; \
          out = (type##_t __user *)addr; \
          err = __copy_to_user(out, &val, sizeof(type##_t)); \
	      if (unlikely(err)) \
		      panic("store memory error\n"); \
          return; \
      } \
      
load_store(uint8)
load_store(uint16)
load_store(uint32)
load_store(uint64)

load_store(int8)
load_store(int16)
load_store(int32)
load_store(int64)

inline uint64_t x(insn_t b, int lo, int len) { return (b >> lo) & (((insn_bits_t)1 << len)-1); }
inline uint64_t xs(insn_t b, int lo, int len) { return (int64_t)b << (64-lo-len) >> (64-len); }
inline uint64_t imm_sign(insn_t b) { return xs(b, 63, 1); }
inline int length(insn_t b) { return insn_length(b); }
inline int64_t i_imm(insn_t b) { return (int64_t)b >> 20; }
inline int64_t shamt(insn_t b) { return x(b, 20, 6); }
inline int64_t s_imm(insn_t b) { return x(b, 7, 5) + (xs(b, 25, 7) << 5); }
inline int64_t sb_imm(insn_t b) { return (x(b, 8, 4) << 1) + (x(b, 25,6) << 5) + (x(b, 7,1) << 11) + (imm_sign(b) << 12); }
inline int64_t u_imm(insn_t b) { return (int64_t)b >> 12 << 12; }
inline int64_t uj_imm(insn_t b) { return (x(b, 21, 10) << 1) + (x(b, 20, 1) << 11) + (x(b, 12, 8) << 12) + (imm_sign(b) << 20); }
inline uint64_t rd(insn_t b) { return x(b, 7, 5); }
inline uint64_t insn_rs1(insn_t b) { return x(b, 15, 5); }
inline uint64_t rs2(insn_t b) { return x(b, 20, 5); }
inline uint64_t rs3(insn_t b) { return x(b, 27, 5); }
inline uint64_t rm(insn_t b) { return x(b, 12, 3); }
inline uint64_t csr(insn_t b) { return x(b, 20, 12); }

inline uint64_t v_vm(insn_t b) { return x(b, 25, 1); }
inline uint64_t v_wd(insn_t b) { return x(b, 26, 1); }
inline uint64_t v_nf(insn_t b) { return x(b, 29, 3); }
inline uint64_t v_simm5(insn_t b) { return xs(b, 15, 5); }
inline uint64_t v_zimm5(insn_t b) { return x(b, 15, 5); }
inline uint64_t v_zimm11(insn_t b) { return x(b, 20, 11); }
inline uint64_t v_lmul(insn_t b) { return 1 << x(b, 20, 2); }
inline uint64_t v_sew(insn_t b) { return 1 << (x(b, 22, 3) + 3); }

extern unsigned long elf_hwcap;
bool supports_extension(processor_t * state, unsigned char ext) {
  if (ext >= 'A' && ext <= 'Z')
    return ((elf_hwcap >> (ext - 'A')) & 1);
  return false;
}

int get_flen(processor_t * state) {
    return supports_extension(state, 'Q') ? 128 :
           supports_extension(state, 'D') ? 64 :
           supports_extension(state, 'F') ? 32 : 0;
}

inline float128_t defaultNaNF128(void)
{
  float128_t nan;
  nan.v[1] = defaultNaNF128UI64;
  nan.v[0] = defaultNaNF128UI0;
  return nan;
}
inline freg_t fsgnj128(freg_t a, freg_t b, bool n, bool x)
{
  a.v[1] = (a.v[1] & ~F64_SIGN) | (((x ? a.v[1] : n ? F64_SIGN : 0) ^ b.v[1]) & F64_SIGN);
  return a;
}
inline freg_t f128_negate(freg_t a)
{
  a.v[1] ^= F64_SIGN;
  return a;
}

inline float32_t f32(uint32_t v) { return (float32_t){v}; }
inline float64_t f64(uint64_t v) { return (float64_t){v}; }
inline float32_t f32_f(freg_t r) { return f32(unboxF32(r)); }
inline float64_t f64_f(freg_t r) { return f64(unboxF64(r)); }
inline float128_t f128(freg_t r) { return r; }
inline freg_t freg32(float32_t f) { return (float128_t){{((uint64_t)-1 << 32) | f.v, (uint64_t)-1}};}
inline freg_t freg64(float64_t f) { return (float128_t){{f.v, (uint64_t)-1}};}
inline freg_t freg128(float128_t f) { return f; }

inline int max_internal(int a, int b) {return a > b? a : b;}
inline int min_internal(int a, int b) {return a > b? b : a;}

inline bool is_overlaped(const int astart, const int asize,
                                const int bstart, const int bsize)
{
  const int aend = astart + asize;
  const int bend = bstart + bsize;
  return max_internal(aend, bend) - min_internal(astart, bstart) < asize + bsize;
}

reg_t set_vl(vector_state *vector, uint64_t regId, reg_t reqVL, reg_t newType){
  if (vector->vtype != newType){
    vector->vtype = newType;
    vector->vsew = 1 << (BITS(newType, 4, 2) + 3);
    vector->vlmul = 1 << BITS(newType, 1, 0);
    vector->vediv = 1 << BITS(newType, 6, 5);
    vector->vlmax = vector->VLEN/vector->vsew * vector->vlmul;
    vector->vmlen = vector->vsew / vector->vlmul;
    vector->reg_mask = (NVPR-1) & ~(vector->vlmul-1);
    vector->vill = false;
  }
  vector->vl = reqVL <= vector->vlmax ? (regId == 0)? vector->vlmax: reqVL : vector->vlmax;
  vector->vstart = 0;
  return vector->vl;
}

inline uint64_t mulhu(uint64_t a, uint64_t b)
{
  uint64_t t, a0, b0, a1, b1;
  uint32_t y1, y2, y3;
  a0 = (uint32_t)a;
  a1 = a >> 32;
  b0 = (uint32_t)b;
  b1 = b >> 32;

  t = a1*b0 + ((a0*b0) >> 32);
  y1 = t;
  y2 = t >> 32;

  t = a0*b1 + y1;
  y1 = t;

  t = a1*b1 + y2 + (t >> 32);
  y2 = t;
  y3 = t >> 32;

  return ((uint64_t)y3 << 32) | y2;
}

inline int64_t mulh(int64_t a, int64_t b)
{
  int negate;
  uint64_t res;
  negate  = (a < 0) != (b < 0);
  res = mulhu(a < 0 ? -a : a, b < 0 ? -b : b);
  return negate ? ~res + (a * b == 0) : res;
}

inline int64_t mulhsu(int64_t a, uint64_t b)
{
  int negate;
  uint64_t res;
  negate = a < 0;
  res = mulhu(a < 0 ? -a : a, b);
  return negate ? ~res + (a * b == 0) : res;
}

//ref:  https://locklessinc.com/articles/sat_arithmetic/
#define sat_add(T, UT) \
inline T sat_add_##T##_##UT(T x, T y, bool *sat) \
{ \
  UT ux, uy, res;\
  int sh; \
  ux = x; \
  uy = y; \
  res = ux + uy; \
  *sat = false; \
  sh = sizeof(T) * 8 - 1; \
  \
  /* Calculate overflowed result. (Don't change the sign bit of ux) */ \
  ux = (ux >> sh) + (((UT)0x1 << sh) - 1); \
  \
  /* Force compiler to use cmovns instruction */ \
  if ((T) ((ux ^ uy) | ~(uy ^ res)) >= 0) { \
    res = ux; \
    *sat = true; \
  } \
  \
  return res; \
}

sat_add(int8_t, uint8_t)
sat_add(int16_t, uint16_t)
sat_add(int32_t, uint32_t)
sat_add(int64_t, uint64_t)
#undef sat_add

#define sat_sub(T, UT) \
inline T sat_sub_##T##_##UT(T x, T y, bool *sat) { \
  UT ux, uy, res; \
  int sh; \
  ux = x; \
  uy = y; \
  res = ux - uy; \
  *sat = false; \
  sh  = sizeof(T) * 8 - 1; \
  \
  /* Calculate overflowed result. (Don't change the sign bit of ux) */ \
  ux = (ux >> sh) + (((UT)0x1 << sh) - 1); \
  \
  /* Force compiler to use cmovns instruction */ \
  if ((T) ((ux ^ uy) & (ux ^ res)) < 0) { \
    res = ux; \
    *sat = true; \
  } \
  \
  return res; \
}

sat_sub(int8_t, uint8_t)
sat_sub(int16_t, uint16_t)
sat_sub(int32_t, uint32_t)
sat_sub(int64_t, uint64_t)
#undef sat_sub

#define sat_addu(T) \
T sat_addu_##T(T x, T y, bool *sat) { \
  T res; \
  res = x + y; \
  *sat = false; \
  \
  *sat = res < x; \
  res |= -(res < x); \
  \
  return res; \
}

sat_addu(uint8_t)
sat_addu(uint16_t)
sat_addu(uint32_t)
sat_addu(uint64_t)
#undef sat_addu

#define sat_subu(T) \
T sat_subu_##T(T x, T y, bool *sat) { \
  T res; \
  res = x - y; \
  *sat = false; \
  \
  *sat = !(res <= x); \
  res &= -(res <= x); \
  \
  return res; \
}

sat_subu(uint8_t)
sat_subu(uint16_t)
sat_subu(uint32_t)
sat_subu(uint64_t)
#undef sat_subu

inline int get_xlen(void)
{
   return 64;// to fix
}

inline int get_max_xlen(void)
{
   return 64;//to fix
}

typedef reg_t (*insn_func_t)(processor_t* p, insn_t insn);
typedef struct
{
  insn_bits_t match;
  insn_bits_t mask;
  insn_func_t rv32;
  insn_func_t rv64;
} insn_desc_t;
#ifndef INSN_REG
#define DECLARE_INSN(name, match, mask) \
		{match, mask, rv32_##name, rv64_##name},
insn_desc_t insts[] = {
	#include "encoding.h"
	{0,0, NULL, NULL}
};
#undef DECLARE_INSN
#else
insn_desc_t insts[1000];
static int idx = 0;

void register_insn( insn_bits_t match, insn_bits_t mask, insn_func_t rv32, insn_func_t rv64)
{
  insts[idx].match = match;
  insts[idx].mask = mask;
  insts[idx].rv32 = rv32;
  insts[idx].rv64 = rv64;
  idx++;
}

#define REGISTER_INSN(name, match, mask) \
  register_insn(match, mask, rv32_##name, rv64_##name);

void register_base_instructions(void)
{
  pr_info("register base %x, %d\n", insts, sizeof(insn_desc_t));
  #define DECLARE_INSN(name, match, mask) \
    insn_bits_t name##_match , name##_mask; \
    name##_match = (match); \
    name##_mask = (mask);
  #include "encoding.h"
  #undef DECLARE_INSN

  #define DEFINE_INSN(name) \
    REGISTER_INSN(name, name##_match, name##_mask)
  #include "insn_list.h"
  #undef DEFINE_INSN

  register_insn(0, 0, NULL, NULL);
  pr_info("idx: %d\n", idx);
}
#endif
#define MAX_SEARCH_INSNS 100
#define GCC_SUPPORT_VSETVL
//#define GCC_SUPPORT_VSETVL
//#define DEBUG_SOFT_VECTER
int back_search_vsetvl(struct pt_regs *regs, processor_t *state, insn_t vinsn) {
#ifdef GCC_SUPPORT_VSETVL
    uint32_t insn;
    uint32_t __user *in;
    processor_t *p = state;
    reg_t addr;
	long err;
	if (vinsn != 0x32002057) { //vmv.x.s	zero,v0
		set_vl(VECTOR, current->thread.vsetvl_state.regid, current->thread.vsetvl_state.vl, current->thread.vsetvl_state.vtype);
		return 1;
	} else {
		addr = regs->epc - 4;
		in = (uint32_t __user *)addr;
		err = __copy_from_user(&insn, in , sizeof(uint32_t));
		if (unlikely(err)) { // cannot fetch insns
			printk("cannot fetch vsetvl insn, default:use last vsetvl info\n");
			set_vl(VECTOR, current->thread.vsetvl_state.regid, current->thread.vsetvl_state.vl, current->thread.vsetvl_state.vtype);
			return 1;
		}
		if ((insn & MASK_VSETVLI) == MATCH_VSETVLI) {
			WRITE_RD(set_vl(VECTOR, insn_rs1(insn), RS1, v_zimm11(insn)));
			current->thread.vsetvl_state.last_vector_pc = regs->epc;
			current->thread.vsetvl_state.regid = insn_rs1(insn);
			current->thread.vsetvl_state.vl = VECTOR->vl;
			current->thread.vsetvl_state.vtype = VECTOR->vtype;
			return 1;
		} else if((insn & MASK_VSETVL) == MATCH_VSETVL) {
			WRITE_RD(set_vl(VECTOR, insn_rs1(insn), RS1, RS2));
			current->thread.vsetvl_state.last_vector_pc = regs->epc;
			current->thread.vsetvl_state.regid = insn_rs1(insn);
			current->thread.vsetvl_state.vl = VECTOR->vl;
			current->thread.vsetvl_state.vtype = VECTOR->vtype;
			return 1;
		} else {// possible c-ext insns
			printk("cannot find vsetvl insn, default:use last vsetvl info\n");
			set_vl(VECTOR, current->thread.vsetvl_state.regid, current->thread.vsetvl_state.vl, current->thread.vsetvl_state.vtype);
			return 1;
		}
	}

#else
	reg_t addr;
	uint8_t match = 0;
	reg_t pos_list[MAX_SEARCH_INSNS];
	uint8_t pos_list_insn[MAX_SEARCH_INSNS];
	uint8_t pos_num;
	uint8_t insn_count = 0;
    uint32_t insn;
    uint32_t __user *in;
	long err;
	processor_t* p = state;
    pos_list[0] = regs->epc - 4;
    pos_list_insn[0] = 0;
    pos_num = 1;
	do {
		addr = pos_list[--pos_num];
		insn_count = pos_list_insn[pos_num];
		do { // search one insn per loop
			if (addr == current->thread.vsetvl_state.last_vector_pc) {// match last setvl
				set_vl(VECTOR, current->thread.vsetvl_state.regid, current->thread.vsetvl_state.vl, current->thread.vsetvl_state.vtype);
				current->thread.vsetvl_state.last_vector_pc = regs->epc;
				match = 1;
				break;
			}
			in = (uint32_t __user *)addr;
			err = __copy_from_user(&insn, in , sizeof(uint32_t));
			if (unlikely(err)) // cannot fetch insns
				break;
			if((insn & 0x3) == 3) { // possible 32-bit insn
				if ((insn & MASK_VSETVLI) == MATCH_VSETVLI) {
					WRITE_RD(set_vl(VECTOR, insn_rs1(insn), RS1, v_zimm11(insn)));
					current->thread.vsetvl_state.last_vector_pc = regs->epc;
					current->thread.vsetvl_state.regid = insn_rs1(insn);
					current->thread.vsetvl_state.vl = VECTOR->vl;
					current->thread.vsetvl_state.vtype = VECTOR->vtype;
					match = 1;
					break;
				} else if((insn & MASK_VSETVL) == MATCH_VSETVL) {
					WRITE_RD(set_vl(VECTOR, insn_rs1(insn), RS1, RS2));
					current->thread.vsetvl_state.last_vector_pc = regs->epc;
					current->thread.vsetvl_state.regid = insn_rs1(insn);
					current->thread.vsetvl_state.vl = VECTOR->vl;
					current->thread.vsetvl_state.vtype = VECTOR->vtype;
					match = 1;
					break;
				} else if (((insn >> 16) & 0x3) != 3) {// possible c-ext insns
					pos_list[pos_num] =  addr - 4;
					pos_list_insn[pos_num++] = insn_count + 1;
					addr = addr - 2;
					insn_count++;
					continue;
				}
				insn_count++;
				addr = addr - 4;
			} else if (((insn >> 16) & 0x3) != 3) { // c-ext insn
				addr = addr - 2;
				insn_count++;
				continue;
			} else //illegal insn
				break;
		} while (insn_count < MAX_SEARCH_INSNS); //search 100 insns at most
		if (match) {
			break;
		}
	} while(pos_num > 0);
	return match;
#endif
}

int get_processor_state(struct pt_regs *regs, processor_t * state, insn_t insn) {
  int i;
#ifdef DEBUG_SOFT_VECTER
  unsigned long *origin = (unsigned long *)&(regs->epc);
#endif
  state->pc = regs->epc;
  state->XPR[0] = 0;
  memcpy (&(state->XPR[1]), &(regs->ra), 31 * sizeof(unsigned long));
  state->saved_a0 = state->XPR[10];
  state->XPR[10] = regs->orig_a0;
  state->mstatus = regs->status;
  if (has_fpu) {
	fstate_save(current, regs);
    for (i = 0; i < 32; i++) {
      state->FPR[i] = freg64(f64(current->thread.fstate.f[i]));
      state->frm = (current->thread.fstate.fcsr >> 5) & 0x7;
      state->fflags = (current->thread.fstate.fcsr & 0x1f);
    }
  }
  if (has_vector) {
	vstate_save(current, regs);
    state->vector.VLEN = 128; //to fix
    state->vector.ELEN = 64;  //to fix
    state->vector.SLEN = 128;  //to fix
    {
		for (i = 0; i < 32; i++) {
		  state->vector.regs[i] = current->thread.vstate.v[i];
		}
    }
    state->vector.vstart = current->thread.vstate.vstart;
    state->vector.vxrm = current->thread.vstate.vxrm;
    state->vector.vxsat = current->thread.vstate.vxsat;
	state->vector.vl = current->thread.vstate.vl;
    state->vector.vtype = current->thread.vstate.vtype;
    state->vector.vill = BITS(state->vector.vtype, (get_xlen() - 1), (get_xlen() - 1));
    if (state->vector.vill != 0) {
    	if (back_search_vsetvl(regs, state, insn) != 1) {
    		printk("ERROR: cannot find related vsetvl\n");
    		return 1;
    	}
    } else {
		state->vector.vsew = 1 << (BITS(state->vector.vtype, 4, 2) + 3);
		state->vector.vlmul = 1 << BITS(state->vector.vtype, 1, 0);
		state->vector.vediv = 1 << BITS(state->vector.vtype, 6, 5);
		state->vector.vlmax = state->vector.VLEN/state->vector.vsew * state->vector.vlmul;
		state->vector.vmlen = state->vector.vsew / state->vector.vlmul;
		state->vector.reg_mask = (NVPR-1) & ~(state->vector.vlmul-1);
    }
  }
  return 0;
}

void restore_processor_state(struct pt_regs *regs, processor_t * state) {
	int i;
	memcpy (&(regs->ra), &(state->XPR[1]), 31 * sizeof(unsigned long));
	if (state->XPR[10] == regs->orig_a0)
		regs->a0 = state->saved_a0;
	regs->status = state->mstatus;
	if (has_fpu) {
		for (i = 0; i < 32; i++) {
			current->thread.fstate.f[i] = state->FPR[i].v[0];
		}
		current->thread.fstate.fcsr = (state->frm << 5) | state->fflags;
		fstate_restore(current, regs);
	}
	if (has_vector) {
		for (i = 0; i < 32; i++) {
		   current->thread.vstate.v[i] = state->vector.regs[i];
		}
		current->thread.vstate.vstart = state->vector.vstart;
		current->thread.vstate.vxrm = state->vector.vxrm;
		current->thread.vstate.vxsat = state->vector.vxsat;
		current->thread.vstate.vl = state->vector.vl;
		current->thread.vstate.vtype = state->vector.vtype;
	}
	regs->epc = state->pc;
	vstate_restore(current, regs);
}

bool decode_exec_insn(struct pt_regs *regs, insn_t insn)
{
	processor_t state;
	insn_desc_t *p;
	reg_t len;
#ifdef INSN_REG
	if (idx == 0) register_base_instructions();
#endif
	p = & insts[0];
    if (get_processor_state(regs, &state, insn))
    	return false;
    if (insn == 0x32002057) {
		state.pc += 4;
		restore_processor_state(regs, &state);
		return true;
    }
    while (p->rv64 && ((insn & p->mask) != p->match))
    	p++;
    if (p->rv64) {
    	if ((len = p->rv64(&state, insn))) {
    		state.pc += len;
    		restore_processor_state(regs, &state);
    		return true;
    	}
    	pr_info("exec insn fail %llx %lx\n", insn, regs->epc);
   } else
	    pr_info("decode insn %llx %lx fail\n", insn, regs->epc);

   return false;
}
