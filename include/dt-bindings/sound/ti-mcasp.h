#ifndef _DT_BINDINGS_TI_MCASP_H
#define _DT_BINDINGS_TI_MCASP_H

/* Source of High-frequency transmit/receive clock */
#define MCASP_CLK_HCLK_AHCLK		0 /* AHCLKX/R */
#define MCASP_CLK_HCLK_AUXCLK		1 /* Internal functional clock */

/* clock divider IDs */
#define MCASP_CLKDIV_AUXCLK		0 /* HCLK divider from AUXCLK */
#define MCASP_CLKDIV_BCLK		1 /* BCLK divider from HCLK */
#define MCASP_CLKDIV_BCLK_FS_RATIO	2 /* to set BCLK FS ration */

#endif /* _DT_BINDINGS_TI_MCASP_H */
