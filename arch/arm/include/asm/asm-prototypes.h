#include <linux/arm-smccc.h>
#include <linux/bitops.h>
#include <linux/ftrace.h>
#include <linux/io.h>
#include <linux/platform_data/asoc-imx-ssi.h>
#include <linux/string.h>
#include <linux/uaccess.h>

#include <asm/checksum.h>
#include <asm/div64.h>
#include <asm/memory.h>

extern void __aeabi_idivmod(void);
extern void __aeabi_idiv(void);
extern void __aeabi_lasr(void);
extern void __aeabi_llsl(void);
extern void __aeabi_llsr(void);
extern void __aeabi_lmul(void);
extern void __aeabi_uidivmod(void);
extern void __aeabi_uidiv(void);
extern void __aeabi_ulcmp(void);

extern void __ashldi3(void);
extern void __ashrdi3(void);
extern void __bswapdi2(void);
extern void __bswapsi2(void);
extern void __divsi3(void);
extern void __do_div64(void);
extern void __lshrdi3(void);
extern void __modsi3(void);
extern void __muldi3(void);
extern void __ucmpdi2(void);
extern void __udivsi3(void);
extern void __umodsi3(void);
