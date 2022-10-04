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

#ifndef _RISCV_ARITH_H
#define _RISCV_ARITH_H

inline uint64_t mulhu(uint64_t a, uint64_t b);

inline int64_t mulh(int64_t a, int64_t b);

inline int64_t mulhsu(int64_t a, uint64_t b);

//ref:  https://locklessinc.com/articles/sat_arithmetic/
#define sat_add(T, UT) \
inline T sat_add_##T##_##UT(T x, T y, bool *sat);

sat_add(int8_t, uint8_t)
sat_add(int16_t, uint16_t)
sat_add(int32_t, uint32_t)
sat_add(int64_t, uint64_t)
#undef sat_add

#define sat_sub(T, UT) \
inline T sat_sub_##T##_##UT(T x, T y, bool *sat);

sat_sub(int8_t, uint8_t)
sat_sub(int16_t, uint16_t)
sat_sub(int32_t, uint32_t)
sat_sub(int64_t, uint64_t)
#undef sat_sub

#define sat_addu(T) \
T sat_addu_##T(T x, T y, bool *sat);

sat_addu(uint8_t)
sat_addu(uint16_t)
sat_addu(uint32_t)
sat_addu(uint64_t)
#undef sat_addu

#define sat_subu(T) \
T sat_subu_##T(T x, T y, bool *sat);

sat_subu(uint8_t)
sat_subu(uint16_t)
sat_subu(uint32_t)
sat_subu(uint64_t)
#undef sat_subu
#endif
