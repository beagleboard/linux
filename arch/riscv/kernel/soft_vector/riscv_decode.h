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

#ifndef _RISCV_DECODE_H
#define _RISCV_DECODE_H

#if (-1 != ~0) || ((-1 >> 1) != -1)
# error spike requires a two''s-complement c++ implementation
#endif

#ifdef WORDS_BIGENDIAN
# error spike requires a little-endian host
#endif

#include <asm/string.h>
#include "encoding.h"
#include "softfloat/config.h"
#include "softfloat/softfloat_types.h"
#include "softfloat/specialize.h"
#include "arith.h"
#include "softfloat/stdint.h"
#include "softfloat/stdbool.h"
#include "linux/compiler.h"
typedef enum VRM{
  RNU = 0,
  RNE,
  RDN,
  ROD,
  INVALID_RM
} VRM;

//#define   likely(x) __builtin_expect(x, 1)
//#define unlikely(x) __builtin_expect(x, 0)

#define NOINLINE __attribute__ ((noinline))

typedef int64_t sreg_t;
typedef unsigned long reg_t;
typedef __int128 int128_t;
typedef unsigned __int128 uint128_t;

#define NXPR 32
#define NFPR 32
#define NVPR 32
#define NCSR 4096

#define X_RA 1
#define X_SP 2

#define FP_RD_NE  0
#define FP_RD_0   1
#define FP_RD_DN  2
#define FP_RD_UP  3
#define FP_RD_NMM 4

#define FSR_RD_SHIFT 5
#define FSR_RD   (0x7 << FSR_RD_SHIFT)

#define FPEXC_NX 0x01
#define FPEXC_UF 0x02
#define FPEXC_OF 0x04
#define FPEXC_DZ 0x08
#define FPEXC_NV 0x10

#define FSR_AEXC_SHIFT 0
#define FSR_NVA  (FPEXC_NV << FSR_AEXC_SHIFT)
#define FSR_OFA  (FPEXC_OF << FSR_AEXC_SHIFT)
#define FSR_UFA  (FPEXC_UF << FSR_AEXC_SHIFT)
#define FSR_DZA  (FPEXC_DZ << FSR_AEXC_SHIFT)
#define FSR_NXA  (FPEXC_NX << FSR_AEXC_SHIFT)
#define FSR_AEXC (FSR_NVA | FSR_OFA | FSR_UFA | FSR_DZA | FSR_NXA)

#define insn_length(x) 4
#define MAX_INSN_LENGTH 8
#define PC_ALIGN 2

#ifndef TAIL_ZEROING
  #define TAIL_ZEROING true
#else
  #define TAIL_ZEROING false
#endif

typedef struct vector_state {
    __uint128_t regs[32];
    reg_t reg_mask, vlmax, vmlen;
    reg_t vstart, vxrm, vxsat, vl, vtype;
    reg_t vediv, vsew, vlmul;
    reg_t ELEN, VLEN, SLEN;
    bool vill;
} vector_state;

#define BITS(v, hi, lo) ((v >> lo) & ((2 << (hi - lo)) - 1))

#define trap_illegal_instruction(n) return n;
#define require(x) if (unlikely(!(x))) trap_illegal_instruction(0)

#define vector_elt(type) \
        type * vector_elt_##type(vector_state* vector, reg_t vReg, reg_t n);\

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
#undef vector_elt

typedef uint64_t insn_t;
typedef uint64_t insn_bits_t;
typedef float128_t freg_t;

//TODO
#define load_store(type) \
      type##_t load_##type(reg_t addr);\
      \
      void store_##type(reg_t addr, type##_t val);
      
load_store(uint8)
load_store(uint16)
load_store(uint32)
load_store(uint64)

load_store(int8)
load_store(int16)
load_store(int32)
load_store(int64)
#undef load_store

inline int get_max_xlen(void);
reg_t set_vl(vector_state *vector, uint64_t regId, reg_t reqVL, reg_t newType);

inline uint64_t x(insn_t b, int lo, int len);
inline uint64_t xs(insn_t b, int lo, int len);
inline uint64_t imm_sign(insn_t b);
inline int length(insn_t b);
inline int64_t i_imm(insn_t b);
inline int64_t shamt(insn_t b);
inline int64_t s_imm(insn_t b);
inline int64_t sb_imm(insn_t b);
inline int64_t u_imm(insn_t b);
inline int64_t uj_imm(insn_t b);
inline uint64_t rd(insn_t b);
inline uint64_t insn_rs1(insn_t b);
inline uint64_t rs2(insn_t b);
inline uint64_t rs3(insn_t b);
inline uint64_t rm(insn_t b);
inline uint64_t csr(insn_t b);

inline uint64_t v_vm(insn_t b);
inline uint64_t v_wd(insn_t b);
inline uint64_t v_nf(insn_t b);
inline uint64_t v_simm5(insn_t b);
inline uint64_t v_zimm5(insn_t b);
inline uint64_t v_zimm11(insn_t b);
inline uint64_t v_lmul(insn_t b);
inline uint64_t v_sew(insn_t b);

typedef struct processor{
  reg_t XPR[32];
  freg_t FPR[32];
  vector_state vector;
  reg_t saved_a0;
  reg_t mstatus;
  reg_t frm;
  reg_t fflags;
  reg_t pc;
} processor_t;

bool supports_extension(processor_t * state, unsigned char ext);

int get_flen(processor_t * state);
inline int get_xlen(void);
#define STATE (*p) 
#define VECTOR (&(p->vector))
#define FLEN (get_flen(p)) 

#define READ_REG(reg) STATE.XPR[reg]
#define READ_FREG(reg) STATE.FPR[reg]
# define WRITE_REG(reg, value) STATE.XPR[reg] = value

#define RD READ_REG(rd(insn))
#define RS1 READ_REG(insn_rs1(insn))
#define RS2 READ_REG(rs2(insn))
#define RS3 READ_REG(rs3(insn))
#define WRITE_RD(value) WRITE_REG(rd(insn), value)

// FPU macros
#define FRS1 READ_FREG(insn_rs1(insn))
#define FRS2 READ_FREG(rs2(insn))
#define FRS3 READ_FREG(rs3(insn))
#define dirty_fp_state (STATE.mstatus |= MSTATUS_FS | (xlen == 64 ? MSTATUS64_SD : MSTATUS32_SD))
#define dirty_ext_state (STATE.mstatus |= MSTATUS_XS | (xlen == 64 ? MSTATUS64_SD : MSTATUS32_SD))
#define DO_WRITE_FREG(reg, value) (STATE.FPR[reg] = value, dirty_fp_state)
#define WRITE_FREG(reg, value, len) DO_WRITE_FREG(reg, freg##len(value))
#define WRITE_FRD(value, len) WRITE_FREG(rd(insn), value, len)
 
#define SHAMT (i_imm(insn) & 0x3F)
#define RM ({ int rm = rm(insn); \
              if(rm == 7) rm = STATE.frm; \
              if(rm > 4) trap_illegal_instruction(0); \
              rm; })

#define require_rv64 require(xlen == 64)
#define require_rv32 require(xlen == 32)
#define require_extension(s) require(supports_extension(p,s))
#define require_fp require((STATE.mstatus & MSTATUS_FS) != 0)
#define require_accelerator require((STATE.mstatus & MSTATUS_XS) != 0)

#define set_fp_exceptions ({ if (softfloat_exceptionFlags) { \
                               dirty_fp_state; \
                               STATE.fflags |= softfloat_exceptionFlags; \
                             } \
                             softfloat_exceptionFlags = 0; })

#define sext32(x) ((sreg_t)(int32_t)(x))
#define zext32(x) ((reg_t)(uint32_t)(x))
#define sext_xlen(x) (((sreg_t)(x) << (64-xlen)) >> (64-xlen))
#define zext_xlen(x) (((reg_t)(x) << (64-xlen)) >> (64-xlen))

/* Convenience wrappers to simplify softfloat code sequences */
#define isBoxedF32(r) (isBoxedF64(r) && ((uint32_t)((r.v[0] >> 32) + 1) == 0))
#define unboxF32(r) (isBoxedF32(r) ? (uint32_t)r.v[0] : defaultNaNF32UI)
#define isBoxedF64(r) ((r.v[1] + 1) == 0)
#define unboxF64(r) (isBoxedF64(r) ? r.v[0] : defaultNaNF64UI)

inline float32_t f32(uint32_t v);
inline float64_t f64(uint64_t v);
inline float32_t f32_f(freg_t r);
inline float64_t f64_f(freg_t r);
inline float128_t f128(freg_t r);
inline freg_t freg32(float32_t f);
inline freg_t freg64(float64_t f);
inline freg_t freg128(float128_t f);
#define F32_SIGN ((uint32_t)1 << 31)
#define F64_SIGN ((uint64_t)1 << 63)
#define fsgnj32(a, b, n, x) \
  f32((f32(a).v & ~F32_SIGN) | ((((x) ? f32(a).v : (n) ? F32_SIGN : 0) ^ f32(b).v) & F32_SIGN))
#define fsgnj64(a, b, n, x) \
  f64((f64(a).v & ~F64_SIGN) | ((((x) ? f64(a).v : (n) ? F64_SIGN : 0) ^ f64(b).v) & F64_SIGN))

#define isNaNF128(x) isNaNF128UI(x.v[1], x.v[0])
inline float128_t defaultNaNF128(void);
inline freg_t fsgnj128(freg_t a, freg_t b, bool n, bool x);
inline freg_t f128_negate(freg_t a);

// Vector macros
#define e8 8      // 8b elements
#define e16 16    // 16b elements
#define e32 32    // 32b elements
#define e64 64    // 64b elements
#define e128 128  // 128b elements

#define vsext(x, sew) (((sreg_t)(x) << (64-sew)) >> (64-sew))
#define vzext(x, sew) (((reg_t)(x) << (64-sew)) >> (64-sew))

#define DEBUG_RVV_FP_VV
#define DEBUG_RVV_FP_VF
#define DEBUG_RVV_FMA_VV
#define DEBUG_RVV_FMA_VF
//
// vector: masking skip helper
//
#define VI_LOOP_ELEMENT_SKIP(BODY) \
  const int mlen = VECTOR->vmlen; \
  const int midx = (mlen * i) / 64; \
  const int mpos = (mlen * i) % 64; \
  if (v_vm(insn) == 0) { \
	bool skip; \
    BODY; \
    skip = ((*vector_elt_uint64_t(VECTOR, 0, midx) >> mpos) & 0x1) == 0; \
    if (skip) \
      continue; \
  }

#define VI_ELEMENT_SKIP(inx) \
  if (inx >= vl && TAIL_ZEROING) { \
    is_valid = false; \
  } else if (inx >= vl && !TAIL_ZEROING) { \
    continue; \
  } else if (inx < VECTOR->vstart) { \
    continue; \
  } else { \
    VI_LOOP_ELEMENT_SKIP(); \
  }

//
// vector: operation and register acccess check helper
//
inline bool is_overlaped(const int astart, const int asize,
                                const int bstart, const int bsize);

#define VI_NARROW_CHECK_COMMON \
  require(VECTOR->vlmul <= 4); \
  require(VECTOR->vsew * 2 <= VECTOR->ELEN); \
  require(rs2(insn) + VECTOR->vlmul * 2 <= 32);

#define VI_WIDE_CHECK_COMMON \
  require(!VECTOR->vill);\
  require(VECTOR->vlmul <= 4); \
  require(VECTOR->vsew * 2 <= VECTOR->ELEN); \
  require(rd(insn) + VECTOR->vlmul * 2 <= 32); \
  if (v_vm(insn) == 0) \
    require(rd(insn) != 0);

#define VI_CHECK_VREG_OVERLAP(v1, v2) \
  require(!is_overlaped(v1, VECTOR->vlmul, v2, VECTOR->vlmul));

#define VI_CHECK_SS \
  require(!is_overlaped(rd(insn), VECTOR->vlmul, rs2(insn), VECTOR->vlmul));

#define VI_CHECK_SD \
  require(!is_overlaped(rd(insn), VECTOR->vlmul, rs2(insn), VECTOR->vlmul * 2));

#define VI_CHECK_DSS(is_rs) \
  VI_WIDE_CHECK_COMMON; \
  require(!is_overlaped(rd(insn), VECTOR->vlmul * 2, rs2(insn), VECTOR->vlmul)); \
  if (is_rs) \
     require(!is_overlaped(rd(insn), VECTOR->vlmul * 2, insn_rs1(insn), VECTOR->vlmul));

#define VI_CHECK_DDS(is_rs) \
  VI_WIDE_CHECK_COMMON; \
  require(rs2(insn) + VECTOR->vlmul * 2 <= 32); \
  if (is_rs) \
     require(!is_overlaped(rd(insn), VECTOR->vlmul * 2, insn_rs1(insn), VECTOR->vlmul));

//
// vector: loop header and end helper
//
#define VI_GENERAL_LOOP_BASE \
  reg_t vl, sew, rd_num, rs1_num, rs2_num, i; \
  require(VECTOR->vsew == e8 || VECTOR->vsew == e16 || VECTOR->vsew == e32 || VECTOR->vsew == e64); \
  require(!VECTOR->vill);\
  vl = VECTOR->vl; \
  sew = VECTOR->vsew; \
  rd_num = rd(insn); \
  rs1_num = insn_rs1(insn); \
  rs2_num = rs2(insn); \
  for (i=VECTOR->vstart; i<vl; ++i){ 

#define VI_TAIL_ZERO(elm) \
  if (vl != 0 && vl < VECTOR->vlmax && TAIL_ZEROING) { \
    uint8_t *tail = vector_elt_uint8_t(VECTOR, rd_num, vl * ((sew >> 3) * elm)); \
    memset(tail, 0, (VECTOR->vlmax - vl) * ((sew >> 3) * elm)); \
  }

#define VI_TAIL_ZERO_MASK(dst) \
  if (vl != 0 && TAIL_ZEROING){ \
    for (i=vl; i<VECTOR->vlmax; ++i){ \
      const int mlen = VECTOR->vmlen; \
      const int midx = (mlen * i) / 64; \
      const int mpos = (mlen * i) % 64; \
      uint64_t mmask = (UINT64_MAX << (64 - mlen)) >> (64 - mlen - mpos); \
      uint64_t *vdi = vector_elt_uint64_t(VECTOR, dst, midx); \
      *vdi = (*vdi & ~mmask);\
    }\
  }\

#define VI_LOOP_BASE \
    VI_GENERAL_LOOP_BASE \
    VI_LOOP_ELEMENT_SKIP();

#define VI_LOOP_END \
  } \
  if (vl != 0 && vl < VECTOR->vlmax && TAIL_ZEROING){ \
    uint8_t *tail = vector_elt_uint8_t(VECTOR, rd_num, vl * ((sew >> 3) * 1)); \
    memset(tail, 0, (VECTOR->vlmax - vl) * ((sew >> 3) * 1)); \
  }\
  VECTOR->vstart = 0;

#define VI_LOOP_END_NO_TAIL_ZERO \
  } \
  VECTOR->vstart = 0;

#define VI_LOOP_WIDEN_END \
  } \
  if (vl != 0 && vl < VECTOR->vlmax && TAIL_ZEROING){ \
    uint8_t *tail = vector_elt_uint8_t(VECTOR, rd_num, vl * ((sew >> 3) * 2)); \
    memset(tail, 0, (VECTOR->vlmax - vl) * ((sew >> 3) * 2)); \
  }\
  VECTOR->vstart = 0;

#define VI_LOOP_REDUCTION_END(x) \
  } \
  if (vl > 0 && TAIL_ZEROING) { \
    uint8_t *tail = (uint8_t *)vector_elt_int##x##_t(VECTOR, rd_num, 1); \
    *vd_0_des = vd_0_res; \
    memset(tail, 0, (VECTOR->VLEN - x) >> 3); \
  } \
  VECTOR->vstart = 0; 

#define VI_LOOP_CMP_BASE \
  reg_t vl, sew, rd_num, rs1_num, rs2_num, i; \
  require(VECTOR->vsew == e8 || VECTOR->vsew == e16 || VECTOR->vsew == e32 || VECTOR->vsew == e64); \
  require(!VECTOR->vill);\
  vl = VECTOR->vl; \
  sew = VECTOR->vsew; \
  rd_num = rd(insn); \
  rs1_num = insn_rs1(insn); \
  rs2_num = rs2(insn); \
  for (i = VECTOR->vstart; i<vl; ++i){ \
	uint64_t mmask, res; \
	uint64_t *vdi; \
    VI_LOOP_ELEMENT_SKIP(); \
    mmask = (UINT64_MAX << (64 - mlen)) >> (64 - mlen - mpos); \
    vdi = vector_elt_uint64_t(VECTOR, rd(insn), midx); \
    res = 0;

#define VI_LOOP_CMP_END \
    *vdi = (*vdi & ~mmask) | (((res) << mpos) & mmask); \
  } \
  VI_TAIL_ZERO_MASK(rd_num); \
  VECTOR->vstart = 0;

#define VI_LOOP_MASK(op) \
  reg_t vl, i; \
  require(VECTOR->vsew <= e64); \
  vl = VECTOR->vl; \
  for (i = VECTOR->vstart; i < vl; ++i) { \
    int mlen = VECTOR->vmlen; \
    int midx = (mlen * i) / 64; \
    int mpos = (mlen * i) % 64; \
    uint64_t mmask = (UINT64_MAX << (64 - mlen)) >> (64 - mlen - mpos); \
    uint64_t vs2 = *vector_elt_uint64_t(VECTOR, rs2(insn), midx); \
    uint64_t vs1 = *vector_elt_uint64_t(VECTOR, insn_rs1(insn), midx); \
    uint64_t *res = vector_elt_uint64_t(VECTOR, rd(insn), midx); \
    *res = (*res & ~mmask) | ((op) & (1ULL << mpos)); \
  } \
  \
  if (TAIL_ZEROING) {\
  for (i = vl; i < VECTOR->vlmax && i > 0; ++i) { \
    int mlen = VECTOR->vmlen; \
    int midx = (mlen * i) / 64; \
    int mpos = (mlen * i) % 64; \
    uint64_t mmask = (UINT64_MAX << (64 - mlen)) >> (64 - mlen - mpos); \
    uint64_t *res = vector_elt_uint64_t(VECTOR, rd(insn), midx); \
    *res = (*res & ~mmask); \
    } \
  } \
  VECTOR->vstart = 0;

#define VI_LOOP_NSHIFT_BASE \
  require(VECTOR->vsew <= e32); \
  if (rd(insn) != 0){ \
    VI_CHECK_SD; \
  } \
  { \
  VI_GENERAL_LOOP_BASE; \
  { \
  VI_LOOP_ELEMENT_SKIP({\
    require(!(rd(insn) == 0 && VECTOR->vlmul > 1));\
  }); \
  }


#define INT_ROUNDING(result, xrm, gb) \
  if (gb > 0) { \
    switch(xrm) {\
      case RNU:\
        result += ((uint64_t)1 << ((gb) - 1));\
        break;\
      case RNE:\
        if ((result & ((uint64_t)0x3 << ((gb) - 1))) == 0x1){\
            result -= ((uint64_t)1 << ((gb) - 1));\
            }else if ((result & ((uint64_t)0x3 << ((gb) - 1))) == 0x3){\
            result += ((uint64_t)1 << ((gb) - 1));\
        }\
        break;\
      case RDN:\
        result = (result >> ((gb) - 1)) << ((gb) - 1);\
        break;\
      case ROD:\
        result |= ((uint64_t)1ul << (gb)); \
        break;\
      case INVALID_RM:\
        require(0);\
    } \
  } else if (gb == 0 && xrm == ROD) { \
    result |= 1ul; \
  }

//
// vector: integer and masking operand access helper
//
#define VXI_PARAMS(x) \
  int##x##_t *vd; \
  int##x##_t vs1, vs2; \
  int##x##_t rs1; \
  int##x##_t simm5; \
  vd = vector_elt_int##x##_t(VECTOR, rd_num, i); \
  rs1 = (int##x##_t)RS1; \
  simm5 = (int##x##_t)v_simm5(insn); \
  vs1 = *vector_elt_int##x##_t(VECTOR, rs1_num, i); \
  vs2 = *vector_elt_int##x##_t(VECTOR, rs2_num, i);

#define VV_U_PARAMS(x) \
  uint##x##_t *vd; \
  uint##x##_t vs1 = *vector_elt_uint##x##_t(VECTOR, rs1_num, i); \
  uint##x##_t vs2 = *vector_elt_uint##x##_t(VECTOR, rs2_num, i); \
  vd = vector_elt_uint##x##_t(VECTOR, rd_num, i);

#define VX_U_PARAMS(x) \
  uint##x##_t *vd; \
  uint##x##_t rs1 = (uint##x##_t)RS1; \
  uint##x##_t vs2 = *vector_elt_uint##x##_t(VECTOR, rs2_num, i); \
  vd = vector_elt_uint##x##_t(VECTOR, rd_num, i);

#define VI_U_PARAMS(x) \
  uint##x##_t *vd; \
  uint##x##_t simm5 = (uint##x##_t)v_zimm5(insn); \
  uint##x##_t vs2 = *vector_elt_uint##x##_t(VECTOR, rs2_num, i); \
  vd = vector_elt_uint##x##_t(VECTOR, rd_num, i);

#define VV_PARAMS(x) \
  int##x##_t *vd; \
  int##x##_t vs1; \
  int##x##_t vs2; \
  vs1 = *vector_elt_int##x##_t(VECTOR, rs1_num, i); \
  vs2 = *vector_elt_int##x##_t(VECTOR, rs2_num, i); \
  vd = vector_elt_int##x##_t(VECTOR, rd_num, i);

#define VX_PARAMS(x) \
  int##x##_t *vd; \
  int##x##_t rs1 = (int##x##_t)RS1; \
  int##x##_t vs2; \
  vs2 = *vector_elt_int##x##_t(VECTOR, rs2_num, i); \
  vd = vector_elt_int##x##_t(VECTOR, rd_num, i);

#define VI_PARAMS(x) \
  int##x##_t *vd; \
  int##x##_t simm5 = (int##x##_t)v_simm5(insn); \
  int##x##_t vs2 = *vector_elt_int##x##_t(VECTOR, rs2_num, i);\
  vd = vector_elt_int##x##_t(VECTOR, rd_num, i);

#define XV_PARAMS(x) \
  int##x##_t *vd = vector_elt_int##x##_t(VECTOR, rd_num, i); \
  uint##x##_t vs2 = *vector_elt_uint##x##_t(VECTOR, rs2_num, RS1);

#define VI_XI_SLIDEDOWN_PARAMS(x, off) \
  int##x##_t *vd = vector_elt_int##x##_t(VECTOR, rd_num, i); \
  int##x##_t vs2 = *vector_elt_int##x##_t(VECTOR, rs2_num, i + off);

#define VI_XI_SLIDEUP_PARAMS(x, offset) \
  int##x##_t *vd = vector_elt_int##x##_t(VECTOR, rd_num, i); \
  int##x##_t vs2 = *vector_elt_int##x##_t(VECTOR, rs2_num, i - offset);

#define VI_NSHIFT_PARAMS(sew1, sew2) \
  uint##sew1##_t *vd = vector_elt_uint##sew1##_t(VECTOR, rd_num, i); \
  uint##sew2##_t  vs2_u; \
  uint##sew2##_t vs2; \
  uint##sew1##_t zimm5 = (uint##sew1##_t)v_zimm5(insn); \
  vs2_u = *vector_elt_uint##sew2##_t(VECTOR, rs2_num, i); \
  vs2 = *vector_elt_int##sew2##_t(VECTOR, rs2_num, i); \

#define VX_NSHIFT_PARAMS(sew1, sew2) \
  uint##sew1##_t *vd = vector_elt_uint##sew1##_t(VECTOR, rd_num, i); \
  uint##sew2##_t vs2_u; \
  uint##sew2##_t vs2; \
  uint##sew1##_t rs1 = (int##sew1##_t)RS1; \
  vs2_u = *vector_elt_uint##sew2##_t(VECTOR, rs2_num, i); \
  vs2 = *vector_elt_int##sew2##_t(VECTOR, rs2_num, i);

#define VV_NSHIFT_PARAMS(sew1, sew2) \
  uint##sew1##_t *vd = vector_elt_uint##sew1##_t(VECTOR, rd_num, i); \
  uint##sew2##_t vs2_u; \
  uint##sew2##_t vs2; \
  uint##sew1##_t vs1 = *vector_elt_int##sew1##_t(VECTOR, rs1_num, i); \
  vs2_u = *vector_elt_uint##sew2##_t(VECTOR, rs2_num, i); \
  vs2 = *vector_elt_int##sew2##_t(VECTOR, rs2_num, i);

#define XI_CARRY_PARAMS(x) \
  int##x##_t vs2 = *vector_elt_int##x##_t(VECTOR, rs2_num, i); \
  int##x##_t rs1; \
  int##x##_t simm5; \
  uint64_t *vd = vector_elt_uint64_t(VECTOR, rd_num, midx); \
  rs1 = (int##x##_t)RS1; \
  simm5 = (int##x##_t)v_simm5(insn); \

#define VV_CARRY_PARAMS(x) \
  int##x##_t vs2 = *vector_elt_int##x##_t(VECTOR, rs2_num, i); \
  int##x##_t vs1 = *vector_elt_int##x##_t(VECTOR, rs1_num, i); \
  uint64_t *vd = vector_elt_uint64_t(VECTOR, rd_num, midx);

//
// vector: integer and masking operation loop
//

// comparision result to masking register
#define VI_VV_LOOP_CMP(BODY) \
{ \
  VI_LOOP_CMP_BASE \
  if (sew == e8){ \
    VV_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VV_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VV_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VV_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_CMP_END \
}

#define VI_VX_LOOP_CMP(BODY) \
{ \
  VI_LOOP_CMP_BASE \
  if (sew == e8){ \
    VX_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VX_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VX_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VX_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_CMP_END \
}

#define VI_VI_LOOP_CMP(BODY) \
{ \
  VI_LOOP_CMP_BASE \
  if (sew == e8){ \
    VI_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VI_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VI_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VI_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_CMP_END \
}

#define VI_VV_ULOOP_CMP(BODY) \
{ \
  VI_LOOP_CMP_BASE \
  if (sew == e8){ \
    VV_U_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VV_U_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VV_U_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VV_U_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_CMP_END \
}

#define VI_VX_ULOOP_CMP(BODY) \
{ \
  VI_LOOP_CMP_BASE \
  if (sew == e8){ \
    VX_U_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VX_U_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VX_U_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VX_U_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_CMP_END \
}

#define VI_VI_ULOOP_CMP(BODY) \
{ \
  VI_LOOP_CMP_BASE \
  if (sew == e8){ \
    VI_U_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VI_U_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VI_U_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VI_U_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_CMP_END \
}

// merge and copy loop
#define VI_VVXI_MERGE_LOOP(BODY) \
{ \
  VI_GENERAL_LOOP_BASE \
  if (sew == e8){ \
    VXI_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VXI_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VXI_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VXI_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_END \
}

// reduction loop - signed
#define VI_LOOP_REDUCTION_BASE(x) \
  reg_t vl = VECTOR->vl; \
  reg_t rd_num = rd(insn); \
  reg_t rs1_num = insn_rs1(insn); \
  reg_t rs2_num = rs2(insn); \
  int##x##_t *vd_0_des = vector_elt_int##x##_t(VECTOR, rd_num, 0); \
  int##x##_t vd_0_res = *vector_elt_int##x##_t(VECTOR, rs1_num, 0); \
  int##x##_t vs2; \
  reg_t i; \
  require(x == e8 || x == e16 || x == e32 || x == e64); \
  require(!VECTOR->vill);\
  for (i = VECTOR->vstart; i<vl; ++i){ \
    VI_LOOP_ELEMENT_SKIP(); \
    vs2 = *vector_elt_int##x##_t(VECTOR, rs2_num, i); \

#define REDUCTION_LOOP(x, BODY) \
  VI_LOOP_REDUCTION_BASE(x) \
  BODY; \
  VI_LOOP_REDUCTION_END(x)

#define VI_VV_LOOP_REDUCTION(BODY) \
  reg_t sew = VECTOR->vsew; \
  if (sew == e8) { \
    REDUCTION_LOOP(8, BODY) \
  } else if(sew == e16) { \
    REDUCTION_LOOP(16, BODY) \
  } else if(sew == e32) { \
    REDUCTION_LOOP(32, BODY) \
  } else if(sew == e64) { \
    REDUCTION_LOOP(64, BODY) \
  }

// reduction loop - unsgied
#define VI_ULOOP_REDUCTION_BASE(x) \
  reg_t vl = VECTOR->vl; \
  reg_t rd_num = rd(insn); \
  reg_t rs1_num = insn_rs1(insn); \
  reg_t rs2_num = rs2(insn); \
  uint##x##_t *vd_0_des = vector_elt_uint##x##_t(VECTOR, rd_num, 0); \
  uint##x##_t vd_0_res = *vector_elt_uint##x##_t(VECTOR, rs1_num, 0); \
  uint##x##_t vs2; \
  reg_t i; \
  for (i=VECTOR->vstart; i<vl; ++i){ \
    VI_LOOP_ELEMENT_SKIP(); \
    vs2 = *vector_elt_uint##x##_t(VECTOR, rs2_num, i);

#define REDUCTION_ULOOP(x, BODY) \
  VI_ULOOP_REDUCTION_BASE(x) \
  BODY; \
  VI_LOOP_REDUCTION_END(x)

#define VI_VV_ULOOP_REDUCTION(BODY) \
  reg_t sew = VECTOR->vsew; \
  if (sew == e8){ \
    REDUCTION_ULOOP(8, BODY) \
  } else if(sew == e16) { \
    REDUCTION_ULOOP(16, BODY) \
  } else if(sew == e32) { \
    REDUCTION_ULOOP(32, BODY) \
  } else if(sew == e64) { \
    REDUCTION_ULOOP(64, BODY) \
  }

// genearl VXI signed/unsgied loop
#define VI_VV_ULOOP(BODY) \
{ \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VV_U_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VV_U_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VV_U_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VV_U_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_END \
}

#define VI_VV_LOOP(BODY) \
{ \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VV_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VV_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VV_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VV_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_END \
}

#define VI_VX_ULOOP(BODY) \
{ \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VX_U_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VX_U_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VX_U_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VX_U_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_END \
}

#define VI_VX_LOOP(BODY) \
{ \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VX_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VX_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VX_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VX_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_END \
}

#define VI_VI_ULOOP(BODY) \
{ \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VI_U_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VI_U_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VI_U_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VI_U_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_END \
}

#define VI_VI_LOOP(BODY) \
{ \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VI_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VI_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VI_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VI_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_END \
}

// narrow operation loop
#define VI_VV_LOOP_NARROW(BODY) \
VI_NARROW_CHECK_COMMON; \
{ \
VI_LOOP_BASE \
if (sew == e8){ \
  VI_NARROW_SHIFT(8, 16) \
  BODY; \
}else if(sew == e16){ \
  VI_NARROW_SHIFT(16, 32) \
  BODY; \
}else if(sew == e32){ \
  VI_NARROW_SHIFT(32, 64) \
  BODY; \
} \
VI_LOOP_END \
}

#define VI_NARROW_SHIFT(sew1, sew2) \
  uint##sew1##_t *vd = vector_elt_uint##sew1##_t(VECTOR, rd_num, i); \
  uint##sew2##_t vs2_u; \
  uint##sew1##_t zimm5; \
  int##sew2##_t vs2; \
  int##sew1##_t vs1; \
  int##sew1##_t rs1; \
  rs1 = (int##sew1##_t)RS1; \
  vs2 = *vector_elt_int##sew2##_t(VECTOR, rs2_num, i); \
  vs1 = *vector_elt_int##sew1##_t(VECTOR, rs1_num, i); \
  vs2_u = *vector_elt_uint##sew2##_t(VECTOR, rs2_num, i); \
  zimm5 = (uint##sew1##_t)v_zimm5(insn);

#define VI_VVXI_LOOP_NARROW(BODY) \
  require(VECTOR->vsew <= e32); \
  { \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VI_NARROW_SHIFT(8, 16) \
    BODY; \
  } else if (sew == e16) { \
    VI_NARROW_SHIFT(16, 32) \
    BODY; \
  } else if (sew == e32) { \
    VI_NARROW_SHIFT(32, 64) \
    BODY; \
  } \
  VI_LOOP_END \
  }

#define VI_VI_LOOP_NSHIFT(BODY) \
{ \
  VI_LOOP_NSHIFT_BASE \
  if (sew == e8){ \
    VI_NSHIFT_PARAMS(8, 16) \
    BODY; \
  } else if (sew == e16) { \
    VI_NSHIFT_PARAMS(16, 32) \
    BODY; \
  } else if (sew == e32) { \
    VI_NSHIFT_PARAMS(32, 64) \
    BODY; \
  } \
  VI_LOOP_END \
  } \
}

#define VI_VX_LOOP_NSHIFT(BODY) \
{ \
  VI_LOOP_NSHIFT_BASE \
  if (sew == e8){ \
    VX_NSHIFT_PARAMS(8, 16) \
    BODY; \
  } else if (sew == e16) { \
    VX_NSHIFT_PARAMS(16, 32) \
    BODY; \
  } else if (sew == e32) { \
    VX_NSHIFT_PARAMS(32, 64) \
    BODY; \
  } \
  VI_LOOP_END \
} \
}

#define VI_VV_LOOP_NSHIFT(BODY) \
{ \
  VI_LOOP_NSHIFT_BASE \
  if (sew == e8){ \
    VV_NSHIFT_PARAMS(8, 16) \
    BODY; \
  } else if (sew == e16) { \
    VV_NSHIFT_PARAMS(16, 32) \
    BODY; \
  } else if (sew == e32) { \
    VV_NSHIFT_PARAMS(32, 64) \
    BODY; \
  } \
  VI_LOOP_END \
}\
}

// widen operation loop
#define VI_VV_LOOP_WIDEN(BODY) \
{ \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VV_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VV_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VV_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VV_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_WIDEN_END \
}

#define VI_VX_LOOP_WIDEN(BODY) \
{ \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VX_PARAMS(8); \
    BODY; \
  }else if(sew == e16){ \
    VX_PARAMS(16); \
    BODY; \
  }else if(sew == e32){ \
    VX_PARAMS(32); \
    BODY; \
  }else if(sew == e64){ \
    VX_PARAMS(64); \
    BODY; \
  } \
  VI_LOOP_WIDEN_END \
}

#define VI_WIDE_OP_AND_ASSIGN(var0, var1, var2, op0, op1, sign) \
  switch(VECTOR->vsew) { \
  case e8: { \
    sign##16_t vd_w; \
    vd_w = *vector_elt_##sign##16_t(VECTOR, rd_num, i); \
    *vector_elt_uint16_t(VECTOR, rd_num, i) = \
      op1((sign##16_t)(sign##8_t)var0 op0 (sign##16_t)(sign##8_t)var1) + var2; \
    } \
    break; \
  case e16: { \
    sign##32_t vd_w; \
    vd_w = *vector_elt_##sign##32_t(VECTOR, rd_num, i); \
    *vector_elt_uint32_t(VECTOR, rd_num, i) = \
      op1((sign##32_t)(sign##16_t)var0 op0 (sign##32_t)(sign##16_t)var1) + var2; \
    } \
    break; \
  default: { \
    sign##64_t vd_w; \
    vd_w = *vector_elt_##sign##64_t(VECTOR, rd_num, i); \
    *vector_elt_uint64_t(VECTOR, rd_num, i) = \
      op1((sign##64_t)(sign##32_t)var0 op0 (sign##64_t)(sign##32_t)var1) + var2; \
    } \
    break; \
  }

#define VI_WIDE_OP_AND_ASSIGN_MIX(var0, var1, var2, op0, op1, sign_d, sign_1, sign_2) \
  switch(VECTOR->vsew) { \
  case e8: { \
    sign_d##16_t vd_w = *vector_elt_##sign_d##16_t(VECTOR, rd_num, i); \
    *vector_elt_uint16_t(VECTOR, rd_num, i) = \
      op1((sign_1##16_t)(sign_1##8_t)var0 op0 (sign_2##16_t)(sign_2##8_t)var1) + var2; \
    } \
    break; \
  case e16: { \
    sign_d##32_t vd_w = *vector_elt_##sign_d##32_t(VECTOR, rd_num, i); \
    *vector_elt_uint32_t(VECTOR, rd_num, i) = \
      op1((sign_1##32_t)(sign_1##16_t)var0 op0 (sign_2##32_t)(sign_2##16_t)var1) + var2; \
    } \
    break; \
  default: { \
    sign_d##64_t vd_w = *vector_elt_##sign_d##64_t(VECTOR, rd_num, i); \
    *vector_elt_uint64_t(VECTOR, rd_num, i) = \
      op1((sign_1##64_t)(sign_1##32_t)var0 op0 (sign_2##64_t)(sign_2##32_t)var1) + var2; \
    } \
    break; \
  }

#define VI_WIDE_WVX_OP(var0, op0, sign) \
  switch(VECTOR->vsew) { \
  case e8: { \
    sign##16_t *vd_w = vector_elt_##sign##16_t(VECTOR, rd_num, i); \
    sign##16_t vs2_w = *vector_elt_##sign##16_t(VECTOR, rs2_num, i); \
    *vd_w = vs2_w op0 (sign##16_t)(sign##8_t)var0; \
    } \
    break; \
  case e16: { \
    sign##32_t *vd_w = vector_elt_##sign##32_t(VECTOR, rd_num, i); \
    sign##32_t vs2_w = *vector_elt_##sign##32_t(VECTOR, rs2_num, i); \
    *vd_w = vs2_w op0 (sign##32_t)(sign##16_t)var0; \
    } \
    break; \
  default: { \
    sign##64_t *vd_w = vector_elt_##sign##64_t(VECTOR, rd_num, i); \
    sign##64_t vs2_w = *vector_elt_##sign##64_t(VECTOR, rs2_num, i); \
    *vd_w = vs2_w op0 (sign##64_t)(sign##32_t)var0; \
    } \
    break; \
  }

#define VI_WIDE_SSMA(sew1, sew2, opd) \
  int##sew2##_t *vd = vector_elt_int##sew2##_t(VECTOR, rd_num, i); \
  int##sew1##_t vs1 ; \
  int##sew1##_t vs2 = *vector_elt_int##sew1##_t(VECTOR, rs2_num, i); \
  int##sew1##_t rs1; \
  int##sew2##_t res; \
  bool sat = false; \
  const int gb = sew1 / 2; \
  VRM vrm = VECTOR->vxrm; \
  rs1 = (int##sew1##_t)RS1; \
  vs1 = *vector_elt_int##sew1##_t(VECTOR, rs1_num, i); \
  res = (int##sew2##_t)vs2 * (int##sew2##_t)opd; \
  INT_ROUNDING(res, vrm, gb); \
  res = res >> gb; \
  *vd = sat_add_int##sew2##_t_uint##sew2##_t(*vd, res, &sat); \
  VECTOR->vxsat |= sat;

#define VI_VVX_LOOP_WIDE_SSMA(opd) \
  VI_WIDE_CHECK_COMMON \
  { \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VI_WIDE_SSMA(8, 16, opd); \
  } else if(sew == e16){ \
    VI_WIDE_SSMA(16, 32, opd); \
  } else if(sew == e32){ \
    VI_WIDE_SSMA(32, 64, opd); \
  } \
  VI_LOOP_WIDEN_END \
  }

#define VI_WIDE_USSMA(sew1, sew2, opd) \
  int##sew2##_t *vd = vector_elt_uint##sew2##_t(VECTOR, rd_num, i); \
  int##sew1##_t vs1; \
  int##sew1##_t vs2 = *vector_elt_uint##sew1##_t(VECTOR, rs2_num, i); \
  int##sew1##_t rs1; \
  uint##sew2##_t res; \
  bool sat = false; \
  const int gb = sew1 / 2; \
  VRM vrm = VECTOR->vxrm; \
  rs1 = (uint##sew1##_t)RS1; \
  vs1 = *vector_elt_uint##sew1##_t(VECTOR, rs1_num, i); \
  res = (uint##sew2##_t)vs2 * (uint##sew2##_t)opd; \
  INT_ROUNDING(res, vrm, gb); \
  \
  res = res >> gb; \
  *vd = sat_addu_uint##sew2##_t(*vd, res, &sat); \
  VECTOR->vxsat |= sat;

#define VI_VVX_LOOP_WIDE_USSMA(opd) \
  VI_WIDE_CHECK_COMMON \
  { \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VI_WIDE_USSMA(8, 16, opd); \
  } else if(sew == e16){ \
    VI_WIDE_USSMA(16, 32, opd); \
  } else if(sew == e32){ \
    VI_WIDE_USSMA(32, 64, opd); \
  } \
  VI_LOOP_WIDEN_END \
  }

#define VI_WIDE_SU_SSMA(sew1, sew2, opd) \
  int##sew2##_t *vd = vector_elt_int##sew2##_t(VECTOR, rd_num, i); \
  int##sew1##_t vs1; \
  uint##sew1##_t vs2 = *vector_elt_uint##sew1##_t(VECTOR, rs2_num, i); \
  int##sew1##_t rs1; \
  int##sew2##_t res; \
  bool sat = false; \
  const int gb = sew1 / 2; \
  VRM vrm = VECTOR->vxrm; \
  rs1 = (int##sew1##_t)RS1; \
  vs1 = *vector_elt_int##sew1##_t(VECTOR, rs1_num, i); \
  res = (uint##sew2##_t)vs2 * (int##sew2##_t)opd; \
  INT_ROUNDING(res, vrm, gb); \
  \
  res = res >> gb; \
  *vd = sat_sub_int##sew2##_t_uint##sew2##_t(*vd, res, &sat); \
  VECTOR->vxsat |= sat;

#define VI_VVX_LOOP_WIDE_SU_SSMA(opd) \
  VI_WIDE_CHECK_COMMON \
  { \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VI_WIDE_SU_SSMA(8, 16, opd); \
  } else if(sew == e16){ \
    VI_WIDE_SU_SSMA(16, 32, opd); \
  } else if(sew == e32){ \
    VI_WIDE_SU_SSMA(32, 64, opd); \
  } \
  VI_LOOP_WIDEN_END \
  }

#define VI_WIDE_US_SSMA(sew1, sew2, opd) \
  int##sew2##_t *vd = vector_elt_int##sew2##_t(VECTOR, rd_num, i); \
  uint##sew1##_t vs1; \
  int##sew1##_t vs2 = *vector_elt_int##sew1##_t(VECTOR, rs2_num, i); \
  uint##sew1##_t rs1; \
  int##sew2##_t res; \
  bool sat = false; \
  const int gb = sew1 / 2; \
  VRM vrm = VECTOR->vxrm; \
  rs1 = (uint##sew1##_t)RS1; \
  vs1 = *vector_elt_uint##sew1##_t(VECTOR, rs1_num, i); \
  res = (int##sew2##_t)vs2 * (uint##sew2##_t)opd; \
  INT_ROUNDING(res, vrm, gb); \
  \
  res = res >> gb; \
  *vd = sat_sub_int##sew2##_t_uint##sew2##_t(*vd, res, &sat); \
  VECTOR->vxsat |= sat;

#define VI_VVX_LOOP_WIDE_US_SSMA(opd) \
  VI_WIDE_CHECK_COMMON \
  { \
  VI_LOOP_BASE \
  if (sew == e8){ \
    VI_WIDE_US_SSMA(8, 16, opd); \
  } else if(sew == e16){ \
    VI_WIDE_US_SSMA(16, 32, opd); \
  } else if(sew == e32){ \
    VI_WIDE_US_SSMA(32, 64, opd); \
  } \
  VI_LOOP_WIDEN_END \
  }

// wide reduction loop - signed
#define VI_LOOP_WIDE_REDUCTION_BASE(sew1, sew2) \
  reg_t vl = VECTOR->vl; \
  reg_t rd_num = rd(insn); \
  reg_t rs1_num = insn_rs1(insn); \
  reg_t rs2_num = rs2(insn); \
  int##sew1##_t vs2; \
  int##sew2##_t *vd_0_des = vector_elt_int##sew2##_t(VECTOR, rd_num, 0); \
  int##sew2##_t vd_0_res = *vector_elt_int##sew2##_t(VECTOR, rs1_num, 0); \
  reg_t i;\
  VI_CHECK_DSS(false); \
  for (i=VECTOR->vstart; i<vl; ++i){ \
    VI_LOOP_ELEMENT_SKIP(); \
    vs2 = *vector_elt_int##sew1##_t(VECTOR, rs2_num, i);

#define WIDE_REDUCTION_LOOP(sew1, sew2, BODY) \
  VI_LOOP_WIDE_REDUCTION_BASE(sew1, sew2) \
  BODY; \
  VI_LOOP_REDUCTION_END(sew2)

#define VI_VV_LOOP_WIDE_REDUCTION(BODY) \
  reg_t sew = VECTOR->vsew; \
  require(!VECTOR->vill);\
  if (sew == e8){ \
    WIDE_REDUCTION_LOOP(8, 16, BODY) \
  } else if(sew == e16){ \
    WIDE_REDUCTION_LOOP(16, 32, BODY) \
  } else if(sew == e32){ \
    WIDE_REDUCTION_LOOP(32, 64, BODY) \
  }

// wide reduction loop - unsigned
#define VI_ULOOP_WIDE_REDUCTION_BASE(sew1, sew2) \
  reg_t vl = VECTOR->vl; \
  reg_t rd_num = rd(insn); \
  reg_t rs1_num = insn_rs1(insn); \
  reg_t rs2_num = rs2(insn); \
  uint##sew2##_t *vd_0_des = vector_elt_uint##sew2##_t(VECTOR, rd_num, 0); \
  uint##sew2##_t vd_0_res = *vector_elt_uint##sew2##_t(VECTOR, rs1_num, 0); \
  reg_t i;\
  VI_CHECK_DSS(false); \
  for (i =VECTOR->vstart; i<vl; ++i) { \
	uint##sew1##_t vs2; \
    VI_LOOP_ELEMENT_SKIP(); \
    vs2 = *vector_elt_uint##sew1##_t(VECTOR, rs2_num, i);

#define WIDE_REDUCTION_ULOOP(sew1, sew2, BODY) \
  VI_ULOOP_WIDE_REDUCTION_BASE(sew1, sew2) \
  BODY; \
  VI_LOOP_REDUCTION_END(sew2)

#define VI_VV_ULOOP_WIDE_REDUCTION(BODY) \
  reg_t sew = VECTOR->vsew; \
  require(!VECTOR->vill);\
  if (sew == e8){ \
    WIDE_REDUCTION_ULOOP(8, 16, BODY) \
  } else if(sew == e16){ \
    WIDE_REDUCTION_ULOOP(16, 32, BODY) \
  } else if(sew == e32){ \
    WIDE_REDUCTION_ULOOP(32, 64, BODY) \
  }

// carry/borrow bit loop
#define VI_VV_LOOP_CARRY(BODY) \
  VI_LOOP_BASE \
    if (sew == e8){ \
      VV_CARRY_PARAMS(8) \
      BODY; \
    } else if (sew == e16) { \
      VV_CARRY_PARAMS(16) \
      BODY; \
    } else if (sew == e32) { \
      VV_CARRY_PARAMS(32) \
      BODY; \
    } else if (sew == e64) { \
      VV_CARRY_PARAMS(64) \
      BODY; \
    } \
  } \
  VI_TAIL_ZERO_MASK(rd_num);

#define VI_XI_LOOP_CARRY(BODY) \
  VI_LOOP_BASE \
    if (sew == e8){ \
      XI_CARRY_PARAMS(8) \
      BODY; \
    } else if (sew == e16) { \
      XI_CARRY_PARAMS(16) \
      BODY; \
    } else if (sew == e32) { \
      XI_CARRY_PARAMS(32) \
      BODY; \
    } else if (sew == e64) { \
      XI_CARRY_PARAMS(64) \
      BODY; \
    } \
  } \
  VI_TAIL_ZERO_MASK(rd_num);

// average loop
#define VI_VVX_LOOP_AVG(opd, op) \
{ \
VRM xrm = VECTOR->vxrm; \
VI_LOOP_BASE \
  switch(sew) { \
    case e8: { \
        int8_t rs1; \
        int32_t res; \
     VV_PARAMS(8); \
     rs1 = RS1; \
     res = (int32_t)vs2 op opd; \
     INT_ROUNDING(res, xrm, 1); \
     *vd = res >> 1; \
     break; \
    } \
    case e16: { \
     int16_t rs1; \
     int32_t res; \
     VV_PARAMS(16); \
     rs1 = RS1; \
     res = (int32_t)vs2 op opd; \
     INT_ROUNDING(res, xrm, 1); \
     *vd = res >> 1; \
     break; \
    } \
    case e32: { \
    int32_t rs1; \
    int64_t res; \
     VV_PARAMS(32); \
     rs1 = RS1; \
     res = (int64_t)vs2 op opd; \
     INT_ROUNDING(res, xrm, 1); \
     *vd = res >> 1; \
     break; \
    } \
    default: { \
     int64_t rs1; \
     int128_t res; \
     VV_PARAMS(64); \
     rs1 = RS1; \
     res = (int128_t)vs2 op opd; \
     INT_ROUNDING(res, xrm, 1); \
     *vd = res >> 1; \
     break; \
    } \
  } \
VI_LOOP_END \
}

//
// vector: load/store helper 
//
#define VI_STRIP(inx) \
  reg_t elems_per_strip = VECTOR->SLEN / VECTOR->vsew; \
  reg_t elems_per_vreg = VECTOR->VLEN /VECTOR->vsew; \
  reg_t elems_per_lane = VECTOR->vlmul * elems_per_strip; \
  reg_t strip_index = (inx) / elems_per_lane; \
  reg_t index_in_strip = (inx) % elems_per_strip; \
  int32_t lmul_inx = (int32_t)(((inx) % elems_per_lane) / elems_per_strip); \
  reg_t vreg_inx = lmul_inx * elems_per_vreg + strip_index * elems_per_strip + index_in_strip;


#define VI_DUPLICATE_VREG(v, vlmax) \
reg_t index[32]; \
reg_t i = 0;\
for (; i < vlmax; ++i) { \
  switch(VECTOR->vsew) { \
    case e32: \
      index[i] = *vector_elt_int32_t(VECTOR, v, i); \
      break; \
    case e64: \
      index[i] = *vector_elt_int64_t(VECTOR, v, i); \
      break; \
    default: \
	  require(0); \
  } \
}

#define VI_ST_WITH_I(stride, offset, st_width, elt_byte) \
  const reg_t nf = v_nf(insn) + 1; \
  const reg_t vl = VECTOR->vl; \
  const reg_t baseAddr = RS1; \
  const reg_t vs3 = rd(insn); \
  const reg_t vlmax = VECTOR->vlmax; \
  const reg_t vlmul = VECTOR->vlmul; \
  i = 0;\
  require((nf * VECTOR->vlmul) <= (NVPR / 4)); \
  for (; i < vlmax && vl != 0; ++i) { \
    bool is_valid = true; \
    reg_t fn = 0;\
    VI_STRIP(i) \
    VI_ELEMENT_SKIP(i); \
    if (!is_valid) \
      continue; \
    for (; fn < nf; ++fn) { \
      st_width##_t val = 0; \
      switch (VECTOR->vsew) { \
      case e8: \
        val = *vector_elt_uint8_t(VECTOR, vs3 + fn * vlmul, vreg_inx); \
        break; \
      case e16: \
        val = *vector_elt_uint16_t(VECTOR, vs3 + fn * vlmul, vreg_inx); \
        break; \
      case e32: \
        val = *vector_elt_uint32_t(VECTOR, vs3 + fn * vlmul, vreg_inx); \
        break; \
      default: \
        val = *vector_elt_uint64_t(VECTOR, vs3 + fn * vlmul, vreg_inx); \
        break; \
      } \
      store_##st_width(baseAddr + (stride) + (offset) * elt_byte, val); \
    } \
  } \
  VECTOR->vstart = 0; 

#define VI_ST(stride, offset, st_width, elt_byte) \
  reg_t i; \
  VI_ST_WITH_I(stride, offset, st_width, elt_byte)

#define VI_LD_WITH_I(stride, offset, ld_width, elt_byte) \
  const reg_t nf = v_nf(insn) + 1; \
  const reg_t vl = VECTOR->vl; \
  const reg_t baseAddr = RS1; \
  const reg_t vd = rd(insn); \
  const reg_t vlmax = VECTOR->vlmax; \
  const reg_t vlmul = VECTOR->vlmul; \
  i = 0; \
  require((nf * VECTOR->vlmul) <= (NVPR / 4)); \
  for (; i < vlmax && vl != 0; ++i) { \
    bool is_valid = true; \
    VI_ELEMENT_SKIP(i); \
    { \
    reg_t fn = 0; \
    VI_STRIP(i); \
    for (; fn < nf; ++fn) { \
      ld_width##_t val = load_##ld_width(baseAddr + (stride) + (offset) * elt_byte); \
      if (vd + fn >= NVPR){ \
         VECTOR->vstart = vreg_inx;\
         require(false); \
      } \
      switch(VECTOR->vsew){ \
        case e8: \
          *vector_elt_uint8_t(VECTOR, vd + fn * vlmul, vreg_inx) = is_valid ? val : 0; \
          break; \
        case e16: \
          *vector_elt_uint16_t(VECTOR, vd + fn * vlmul, vreg_inx) = is_valid ? val : 0; \
          break; \
        case e32: \
          *vector_elt_uint32_t(VECTOR, vd + fn * vlmul, vreg_inx) = is_valid ? val : 0; \
          break; \
        default: \
          *vector_elt_uint64_t(VECTOR, vd + fn * vlmul, vreg_inx) = is_valid ? val : 0; \
      } \
    } \
    } \
  } \
  VECTOR->vstart = 0;

#define VI_LD(stride, offset, ld_width, elt_byte) \
  reg_t i; \
  VI_LD_WITH_I(stride, offset, ld_width, elt_byte)


#define VI_LDST_FF(itype, tsew) \
  const reg_t nf = v_nf(insn) + 1; \
  const reg_t sew = VECTOR->vsew; \
  const reg_t vl = VECTOR->vl; \
  const reg_t baseAddr = RS1; \
  const reg_t rd_num = rd(insn); \
  bool early_stop = false; \
  const reg_t vlmax = VECTOR->vlmax; \
  const reg_t vlmul = VECTOR->vlmul; \
  reg_t i = 0; \
  require(VECTOR->vsew >= e##tsew && VECTOR->vsew <= e64); \
  require((nf * VECTOR->vlmul) <= (NVPR / 4)); \
  for (; i < vlmax && vl != 0; ++i) { \
    bool is_valid = true; \
    VI_STRIP(i); \
    VI_ELEMENT_SKIP(i); \
    { \
    reg_t fn = 0; \
    for (; fn < nf; ++fn) { \
      itype##64_t val = load_##itype##tsew(baseAddr + (i * nf + fn) * (tsew / 8)); \
      \
      switch (sew) { \
      case e8: \
        *vector_elt_uint8_t(VECTOR, rd_num + fn * vlmul, vreg_inx) = is_valid ? val : 0; \
        break; \
      case e16: \
        *vector_elt_uint16_t(VECTOR, rd_num + fn * vlmul, vreg_inx) = is_valid ? val : 0; \
        break; \
      case e32: \
        *vector_elt_uint32_t(VECTOR, rd_num + fn * vlmul, vreg_inx) = is_valid ? val : 0; \
        break; \
      case e64: \
        *vector_elt_uint64_t(VECTOR, rd_num + fn * vlmul, vreg_inx) = is_valid ? val : 0; \
        break; \
      } \
       \
      if (val == 0 && is_valid) { \
        VECTOR->vl = i; \
        early_stop = true; \
        break; \
      } \
    } \
    }\
    if (early_stop) { \
      break; \
    } \
  } \
  VECTOR->vstart = 0;


//
// vector: vfp helper
//
#define VI_VFP_COMMON \
  reg_t vl = VECTOR->vl; \
  reg_t rd_num = rd(insn); \
  reg_t rs1_num; \
  reg_t rs2_num; \
  rs2_num = rs2(insn); \
  rs1_num = insn_rs1(insn); \
  softfloat_roundingMode = STATE.frm; \
  require_fp; \
  require((VECTOR->vsew == e32 && supports_extension(p, 'F')) || \
          (VECTOR->vsew == e64 && supports_extension(p, 'D'))); \
  require(!VECTOR->vill);

#define VI_VFP_LOOP_BASE \
  VI_VFP_COMMON \
  { \
  reg_t i=VECTOR->vstart; \
  for (; i<vl; ++i){ \
    VI_LOOP_ELEMENT_SKIP();

#define VI_VFP_LOOP_CMP_BASE \
  VI_VFP_COMMON \
  { \
  reg_t i = VECTOR->vstart; \
  for (; i < vl; ++i) { \
	uint64_t mmask, res; \
	uint64_t *vdi; \
    float64_t vs2 = *vector_elt_float64_t(VECTOR, rs2_num, i); \
    float64_t vs1 = *vector_elt_float64_t(VECTOR, rs1_num, i); \
    float64_t rs1 = f64_f(READ_FREG(rs1_num)); \
    VI_LOOP_ELEMENT_SKIP(); \
    mmask = (UINT64_MAX << (64 - mlen)) >> (64 - mlen - mpos); \
    vdi = vector_elt_uint64_t(VECTOR, rd_num, midx); \
    res = 0;

#define VI_VFP_LOOP_REDUCTION_BASE \
  VI_VFP_COMMON \
  { \
  float64_t vd_0 = *vector_elt_float64_t(VECTOR, rd_num, 0); \
  float64_t vs1_0 = *vector_elt_float64_t(VECTOR, rs1_num, 0); \
  reg_t i = VECTOR->vstart; \
  vd_0 = vs1_0;\
  for (; i<vl; ++i){ \
    VI_LOOP_ELEMENT_SKIP(); \

#define VI_VFP_LOOP_WIDE_REDUCTION_BASE \
  VI_VFP_COMMON \
  { \
  float64_t vd_0 = f64(vector_elt_float64_t(VECTOR, rs1_num, 0)->v); \
  reg_t i =VECTOR->vstart; \
  for (; i<vl; ++i) { \
    VI_LOOP_ELEMENT_SKIP();

#define VI_VFP_LOOP_END \
  } \
  } \
  if (vl != 0 && vl < VECTOR->vlmax && TAIL_ZEROING){ \
    uint8_t *tail = vector_elt_uint8_t(VECTOR, rd_num, vl * ((VECTOR->vsew >> 3) * 1)); \
    memset(tail, 0, (VECTOR->vlmax - vl) * ((VECTOR->vsew >> 3) * 1)); \
  }\
  VECTOR->vstart = 0; \

#define VI_VFP_LOOP_WIDE_END \
  } \
  } \
  if (vl != 0 && vl < VECTOR->vlmax && TAIL_ZEROING){ \
    uint8_t *tail = vector_elt_uint8_t(VECTOR, rd_num, vl * ((VECTOR->vsew >> 3) * 2)); \
    memset(tail, 0, (VECTOR->vlmax - vl) * ((VECTOR->vsew >> 3) * 2)); \
  }\
  VECTOR->vstart = 0; \
  set_fp_exceptions;

#define VI_VFP_LOOP_REDUCTION_END(x) \
  } \
  VECTOR->vstart = 0; \
  set_fp_exceptions; \
  *vector_elt_int##x##_t(VECTOR, rd_num, 0) = vd_0.v; \
  if (vl > 0 && TAIL_ZEROING) { \
    for (i = 1; i < (VECTOR->VLEN / x); ++i) { \
       *vector_elt_int##x##_t(VECTOR, rd_num, i) = 0; \
    } \
  } \
  }

#define VI_VFP_LOOP_CMP_END \
  switch(VECTOR->vsew) { \
    case e64: \
    case e32: { \
      *vdi = (*vdi & ~mmask) | (((res) << mpos) & mmask); \
      break; \
    } \
    case e16: \
    case e8: \
    default: \
      require(0); \
      break; \
    }; \
    rs1.v = vs1.v = 0; \
  } \
  if (vl != 0 && TAIL_ZEROING){ \
    for (i=vl; i<VECTOR->vlmax; ++i){ \
      const int mlen = VECTOR->vmlen; \
      const int midx = (mlen * i) / 64; \
      const int mpos = (mlen * i) % 64; \
      uint64_t mmask = (UINT64_MAX << (64 - mlen)) >> (64 - mlen - mpos); \
      uint64_t *vdi = vector_elt_uint64_t(VECTOR, rd(insn), midx); \
      *vdi = (*vdi & ~mmask);\
    }\
  }\
  }\
  VECTOR->vstart = 0; \
  set_fp_exceptions;

#define VI_VFP_VV_LOOP(BODY32, BODY64) \
{ \
  VI_VFP_LOOP_BASE \
  switch(VECTOR->vsew) { \
    case e32: {\
      float32_t *vd; \
      float32_t vs1, vs2; \
      vs1 = *vector_elt_float32_t(VECTOR, rs1_num, i); \
      vs2 = *vector_elt_float32_t(VECTOR, rs2_num, i); \
      vd = vector_elt_float32_t(VECTOR, rd_num, i); \
      BODY32; \
      set_fp_exceptions; \
      break; \
    }\
    case e64: {\
      float64_t *vd; \
      float64_t vs1, vs2; \
      vs2 = *vector_elt_float64_t(VECTOR, rs2_num, i); \
      vs1 = *vector_elt_float64_t(VECTOR, rs1_num, i); \
      vd = vector_elt_float64_t(VECTOR, rd_num, i); \
      BODY64; \
      set_fp_exceptions; \
      break; \
    }\
    case e16: \
    case e8: \
    default: \
      require(0); \
      break; \
  }; \
  DEBUG_RVV_FP_VV; \
  VI_VFP_LOOP_END \
}

#define VI_VFP_VV_LOOP_REDUCTION(BODY) \
{ \
  VI_VFP_LOOP_REDUCTION_BASE \
  { \
  float64_t vs2 = *vector_elt_float64_t(VECTOR, rs2_num, i); \
  BODY; \
  DEBUG_RVV_FP_VV; \
  } \
  VI_VFP_LOOP_REDUCTION_END(64) \
}

#define VI_VFP_VV_LOOP_WIDE_REDUCTION(BODY) \
{ \
  VI_VFP_LOOP_WIDE_REDUCTION_BASE \
  { \
  float64_t vs2 = f32_to_f64(*vector_elt_float32_t(VECTOR, rs2_num, i)); \
  BODY; \
  DEBUG_RVV_FP_VV; \
  } \
  VI_VFP_LOOP_REDUCTION_END(64) \
}

#define VI_VFP_VF_LOOP(BODY32, BODY64) \
{ \
  VI_VFP_LOOP_BASE \
  switch(VECTOR->vsew) { \
    case e32: {\
      float32_t *vd = vector_elt_float32_t(VECTOR, rd_num, i); \
      float32_t rs1 = f32_f(READ_FREG(rs1_num)); \
      float32_t vs2 = *vector_elt_float32_t(VECTOR, rs2_num, i); \
      BODY32; \
      set_fp_exceptions; \
      break; \
    }\
    case e64: {\
      float64_t *vd = vector_elt_float64_t(VECTOR, rd_num, i); \
      float64_t rs1 = f64_f(READ_FREG(rs1_num)); \
      float64_t vs2 = *vector_elt_float64_t(VECTOR, rs2_num, i); \
      BODY64; \
      set_fp_exceptions; \
      break; \
    }\
    case e16: \
    case e8: \
    default: \
      require(0); \
      break; \
  }; \
  DEBUG_RVV_FP_VF; \
  VI_VFP_LOOP_END \
}

#define VI_VFP_LOOP_CMP(BODY) \
{ \
  VI_VFP_LOOP_CMP_BASE \
  BODY; \
  DEBUG_RVV_FP_VV; \
  VI_VFP_LOOP_CMP_END \
}

#define VI_VFP_VF_LOOP_WIDE(BODY) \
{ \
  VI_VFP_LOOP_BASE \
  VI_CHECK_DSS(false); \
  switch(VECTOR->vsew) { \
    case e32: {\
      float64_t *vd = vector_elt_float64_t(VECTOR, rd_num, i); \
      float64_t vs2 = f32_to_f64(*vector_elt_float32_t(VECTOR, rs2_num, i)); \
      float64_t rs1 = f32_to_f64(f32_f(READ_FREG(rs1_num))); \
      BODY; \
      set_fp_exceptions; \
      break; \
    }\
    case e16: \
    case e8: \
    default: \
      require(0); \
      break; \
  }; \
  DEBUG_RVV_FP_VV; \
  VI_VFP_LOOP_WIDE_END \
}

#define VI_VFP_VV_LOOP_WIDE(BODY) \
{ \
  VI_VFP_LOOP_BASE \
  VI_CHECK_DSS(true); \
  switch(VECTOR->vsew) { \
    case e32: {\
      float64_t *vd = vector_elt_float64_t(VECTOR, rd_num, i); \
      float64_t vs2 = f32_to_f64(*vector_elt_float32_t(VECTOR, rs2_num, i)); \
      float64_t vs1 = f32_to_f64(*vector_elt_float32_t(VECTOR, rs1_num, i)); \
      BODY; \
      set_fp_exceptions; \
      break; \
    }\
    case e16: \
    case e8: \
    default: \
      require(0); \
      break; \
  }; \
  DEBUG_RVV_FP_VV; \
  VI_VFP_LOOP_WIDE_END \
}

#define VI_VFP_WF_LOOP_WIDE(BODY) \
{ \
  VI_VFP_LOOP_BASE \
  VI_CHECK_DDS(false); \
  switch(VECTOR->vsew) { \
    case e32: {\
      float64_t *vd = vector_elt_float64_t(VECTOR, rd_num, i); \
      float64_t vs2 = *vector_elt_float64_t(VECTOR, rs2_num, i); \
      float64_t rs1 = f32_to_f64(f32_f(READ_FREG(rs1_num))); \
      BODY; \
      set_fp_exceptions; \
      break; \
    }\
    case e16: \
    case e8: \
    default: \
      require(0); \
  }; \
  DEBUG_RVV_FP_VV; \
  VI_VFP_LOOP_WIDE_END \
}

#define VI_VFP_WV_LOOP_WIDE(BODY) \
{ \
  VI_VFP_LOOP_BASE \
  VI_CHECK_DDS(true); \
  switch(VECTOR->vsew) { \
    case e32: {\
      float64_t *vd = vector_elt_float64_t(VECTOR, rd_num, i); \
      float64_t vs2 = *vector_elt_float64_t(VECTOR, rs2_num, i); \
      float64_t vs1 = f32_to_f64(*vector_elt_float32_t(VECTOR, rs1_num, i)); \
      BODY; \
      set_fp_exceptions; \
      break; \
    }\
    case e16: \
    case e8: \
    default: \
      require(0); \
  }; \
  DEBUG_RVV_FP_VV; \
  VI_VFP_LOOP_WIDE_END \
}

#define VI_AMO(op, type) \
  const reg_t vl = VECTOR->vl; \
  const reg_t baseAddr = RS1; \
  const reg_t vd = rd(insn); \
  require(!VECTOR->vill);\
  require(rd(insn) + VECTOR->vlmul <= 32); \
  if (v_vm(insn) == 0) \
	require(rd(insn) != 0); \
  require(VECTOR->vsew <= get_xlen() && VECTOR->vsew >= 32 && VECTOR->vsew >= type); \
  { \
  VI_DUPLICATE_VREG(rs2(insn), VECTOR->vsew); \
  for (i = VECTOR->vstart; i < vl; ++i) { \
    bool is_valid = true; \
    VI_ELEMENT_SKIP(i); \
    { \
    VI_STRIP(i); \
    switch (VECTOR->vsew) { \
    case e32: {\
      int32_t vs3 = *vector_elt_int32_t(VECTOR, vd, vreg_inx); \
      int32_t lhs = load_int##type(baseAddr + index[i]); \
      int32_t val; \
      op\
      store_int##type(baseAddr + index[i], val); \
      if (v_wd(insn)) \
        *vector_elt_int32_t(VECTOR, vd, vreg_inx) = lhs; \
      } \
      break; \
    case e64: {\
      int64_t vs3 = *vector_elt_int64_t(VECTOR, vd, vreg_inx); \
      int64_t lhs = load_int##type(baseAddr + index[i]); \
      int64_t val; \
      op\
      store_int##type(baseAddr + index[i], val); \
      if (v_wd(insn)) \
        *vector_elt_int64_t(VECTOR, vd, vreg_inx) = lhs; \
      } \
      break; \
    default: \
      require(0); \
      break; \
    } \
    } \
  } \
  } \
  VECTOR->vstart = 0;

#define DEFINE_INSN(name) \
   reg_t rv32_##name(processor_t* p, insn_t insn); \
   reg_t rv64_##name(processor_t* p, insn_t insn); \
   
#include "insn_list.h"
#undef DEFINE_INSN
// Seems that 0x0 doesn't work.
#define DEBUG_START             0x100
#define DEBUG_END               (0x1000 - 1)
#endif
