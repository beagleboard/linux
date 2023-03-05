#ifndef _LIGHT_I2S_H
#define _LIGHT_I2S_H

#include <linux/io.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/scatterlist.h>
#include <linux/sh_dma.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/dmaengine_pcm.h>

#include "light-pcm.h"
#include <linux/spinlock.h>

#define I2S_IISEN          0x000 /* IIS Enable Register */
#define I2S_FUNCMODE       0x004 /* IIS function mode */
#define I2S_IISCNF_IN      0x008 /* IIS interface configuration in (on RX side) */
#define I2S_FSSTA          0x00c /* IIS ATX audio input control/state register */
#define I2S_IISCNF_OUT     0x010 /* IIS interface configuration in (on TX side) */
#define I2S_FADTLR         0x014 /* IIS Fs auto detected Threshold level register */
#define I2S_SCCR           0x018 /* Sample compress control register */
#define I2S_TXFTLR         0x01c /* Transmit FIFO Threshold Level */
#define I2S_RXFTLR         0x020 /* Receive FIFO Threshold Level */
#define I2S_TXFLR          0x024 /* Transmit FIFO Level Register */
#define I2S_RXFLR          0x028 /* Receive FIFO Level Register */
#define I2S_SR             0x02c /* Status Register */
#define I2S_IMR            0x030 /* Interrupt Mask Register */
#define I2S_ISR            0x034 /* Interrupt Status Register */
#define I2S_RISR           0x038 /* Raw Interrupt Status Register */
#define I2S_ICR            0x03c /* FIFO Interrupt Clear Register */
#define I2S_DMACR          0x040 /* DMA Control Register */
#define I2S_DMATDLR        0x044 /* DMA Transmit Data Level */
#define I2S_DMARDLR        0x048 /* DMA Receive Data Level */
#define I2S_DR             0x04C /* Data Register */
#define I2S_DIV0_LEVEL     0x050 /* Divide i2s_clkgen source clock, get mclk_o */
#define I2S_DIV3_LEVEL     0x054 /* Divide i2s_clkgen source clock, get reference clock */
#define RESERVED           0x058 /* RESERVED */
#define RESERVED1          0x05C /* RESERVED */
#define I2S_DR1            0x060 /* CH1_Data Register */
#define I2S_DR2            0x064 /* CH2_Data Register */
#define I2S_DR3            0x068 /* CH3_Data Register */
#define I2S_DR4            0x06C /* CH4_Data Register */

#define CPR_PERI_DIV_SEL_REG  0x004 /*audio sys i2s clock div Register*/
#define CPR_PERI_CLK_SEL_REG  0x008 /*audio sys i2s clock selection Register*/

/* IISEN , offset: 0x00 */
#define IISEN_I2SEN_POS                         (0U)
#define IISEN_I2SEN_MSK                         (0x1U << IISEN_I2SEN_POS)
#define IISEN_I2SEN                             IISEN_I2SEN_MSK

/* FUNCMODE, offset: 0x04 */
#define FUNCMODE_TMODE_Pos                      (0U)
#define FUNCMODE_TMODE_Msk                      (0x1U << FUNCMODE_TMODE_Pos)
#define FUNCMODE_TMODE                          FUNCMODE_TMODE_Msk

#define FUNCMODE_TMODE_WEN_Pos                  (1U)
#define FUNCMODE_TMODE_WEN_Msk                  (0x1U << FUNCMODE_TMODE_WEN_Pos)
#define FUNCMODE_TMODE_WEN                      FUNCMODE_TMODE_WEN_Msk

#define FUNCMODE_RMODE_Pos                      (4U)
#define FUNCMODE_RMODE_Msk                      (0x1U << FUNCMODE_RMODE_Pos)
#define FUNCMODE_RMODE                          FUNCMODE_RMODE_Msk

#define FUNCMODE_RMODE_WEN_Pos                  (5U)
#define FUNCMODE_RMODE_WEN_Msk                  (0x1U << FUNCMODE_RMODE_WEN_Pos)
#define FUNCMODE_RMODE_WEN                      FUNCMODE_RMODE_WEN_Msk

#define FUNCMODE_CH0_ENABLE_Pos                 (8U)
#define FUNCMODE_CH0_ENABLE_Msk                 (0x1U << FUNCMODE_CH0_ENABLE_Pos)
#define FUNCMODE_CH0_ENABLE                     FUNCMODE_CH0_ENABLE_Msk

#define FUNCMODE_CH1_ENABLE_Pos                 (9U)
#define FUNCMODE_CH1_ENABLE_Msk                 (0x1U << FUNCMODE_CH1_ENABLE_Pos)
#define FUNCMODE_CH1_ENABLE                     FUNCMODE_CH1_ENABLE_Msk

#define FUNCMODE_CH2_ENABLE_Pos                 (10U)
#define FUNCMODE_CH2_ENABLE_Msk                 (0x1U << FUNCMODE_CH2_ENABLE_Pos)
#define FUNCMODE_CH2_ENABLE                     FUNCMODE_CH2_ENABLE_Msk

#define FUNCMODE_CH3_ENABLE_Pos                 (11U)
#define FUNCMODE_CH3_ENABLE_Msk                 (0x1U << FUNCMODE_CH3_ENABLE_Pos)
#define FUNCMODE_CH3_ENABLE                     FUNCMODE_CH3_ENABLE_Msk

#define FUNCMODE_CH4_ENABLE_Pos                 (12U)
#define FUNCMODE_CH4_ENABLE_Msk                 (0x1U << FUNCMODE_CH4_ENABLE_Pos)
#define FUNCMODE_CH4_ENABLE                     FUNCMODE_CH4_ENABLE_Msk

/* IISCNFIN, offset: 0x08 */
#define CNFIN_RSAFS_Pos                      (0U)
#define CNFIN_RSAFS_Msk                      (0x3U << CNFIN_RSAFS_Pos)
#define CNFIN_RSAFS_I2S                      (0x0U << CNFIN_RSAFS_Pos)
#define CNFIN_RSAFS_RIGHT_JUSTIFIED          (0x1U << CNFIN_RSAFS_Pos)
#define CNFIN_RSAFS_LEFT_JUSTIFIED           (0x2U << CNFIN_RSAFS_Pos)
#define CNFIN_RSAFS_PCM                      (0x3U << CNFIN_RSAFS_Pos)

#define CNFIN_RALOLRC_Pos                    (2U)
#define CNFIN_RALOLRC_Msk                    (0x1U << CNFIN_RALOLRC_Pos)
#define CNFIN_RALOLRC_HIGHFORLEFT            CNFIN_RALOLRC_Msk

#define CNFIN_RVOICEEN_Pos                   (4U)
#define CNFIN_RVOICEEN_Msk                   (0x1U << CNFIN_RVOICEEN_Pos)
#define CNFIN_RVOICEEN_MONO                  CNFIN_RVOICEEN_Msk

#define CNFIN_RX_CH_SEL_Pos                  (5U)
#define CNFIN_RX_CH_SEL_Msk                  (0x1U << CNFIN_RX_CH_SEL_Pos)
#define CNFIN_RX_CH_SEL_LEFT                 CNFIN_RX_CH_SEL_Msk

#define CNFIN_I2S_RXMODE_Pos                 (8U)
#define CNFIN_I2S_RXMODE_Msk                 (0x1U << CNFIN_I2S_RXMODE_Pos)
#define CNFIN_I2S_RXMODE_MASTER_MODE         CNFIN_I2S_RXMODE_Msk

#define CNFIN_I2S_RX_CLK_SEL_Pos             (9U)
#define CNFIN_I2S_RX_CLK_SEL_Msk             (0x1U << CNFIN_I2S_RX_CLK_SEL_Pos)
#define CNFIN_I2S_RX_CLK_SEL_MCLK            CNFIN_I2S_RX_CLK_SEL_Msk

#define CNFIN_I2S_RDELAY_Pos                 (12U)
#define CNFIN_I2S_RDELAY_Msk                 (0x3U << CNFIN_I2S_RDELAY_Pos)
#define CNFIN_I2S_RDELAY_NO_DELAY            (0x0U << CNFIN_I2S_RDELAY_Pos)
#define CNFIN_I2S_RDELAY_1_DELAY             (0x1U << CNFIN_I2S_RDELAY_Pos)
#define CNFIN_I2S_RDELAY_2_DELAY             (0x2U << CNFIN_I2S_RDELAY_Pos)
#define CNFIN_I2S_RDELAY_3_DELAY             (0x3U << CNFIN_I2S_RDELAY_Pos)

/* FSSTA , offset: 0x0C */
#define FSSTA_AIRAD_Pos                         (0U)
#define FSSTA_AIRAD_Msk                         (0x1U << FSSTA_AIRAD_Pos)
#define FSSTA_AIRAD                             FSSTA_AIRAD_Msk

#define FSSTA_AFR_Pos                           (4U)
#define FSSTA_AFR_Msk                           (0x3U << FSSTA_AFR_Pos)
#define FSSTA_AFR_88_2KSPS                      (0x0U << FSSTA_AFR_Pos)
#define FSSTA_AFR_96KSPS                        (0x1U << FSSTA_AFR_Pos)
#define FSSTA_AFR_64KSPS                        (0x2U << FSSTA_AFR_Pos)
#define FSSTA_AFR_192KSPS                       (0x3U << FSSTA_AFR_Pos)

#define FSSTA_ARS_Pos                           (6U)
#define FSSTA_ARS_Msk                           (0x3U << FSSTA_ARS_Pos)
#define FSSTA_ARS_1                             (0x0U << FSSTA_ARS_Pos)
#define FSSTA_ARS_0_5                           (0x1U << FSSTA_ARS_Pos)
#define FSSTA_ARS_0_25                          (0x2U << FSSTA_ARS_Pos)
#define FSSTA_ARS_0_125                         (0x3U << FSSTA_ARS_Pos)

#define FSSTA_DATAWTH_Pos                       (8U)
#define FSSTA_DATAWTH_Msk                       (0xFU << FSSTA_DATAWTH_Pos)

#define FSSTA_SCLK_SEL_Pos                      (12U)
#define FSSTA_SCLK_SEL_Msk                      (0x3U << FSSTA_SCLK_SEL_Pos)
#define FSSTA_SCLK_SEL_32                       (0x0U << FSSTA_SCLK_SEL_Pos)
#define FSSTA_SCLK_SEL_48                       (0x1U << FSSTA_SCLK_SEL_Pos)
#define FSSTA_SCLK_SEL_64                       (0x2U << FSSTA_SCLK_SEL_Pos)
#define FSSTA_SCLK_SEL_16                       (0x3U << FSSTA_SCLK_SEL_Pos)


#define FSSTA_MCLK_SEL_Pos                      (16U)
#define FSSTA_MCLK_SEL_Msk                      (0x1U << FSSTA_MCLK_SEL_Pos)
#define FSSTA_MCLK_SEL_384                      FSSTA_MCLK_SEL_Msk

/* IISCNFOUT, offset: 0x10 */
#define IISCNFOUT_TSAFS_POS                     (0U)
#define IISCNFOUT_TSAFS_MSK                     (0x3U << IISCNFOUT_TSAFS_POS)
#define IISCNFOUT_TSAFS_I2S                     (0x0U << IISCNFOUT_TSAFS_POS)
#define IISCNFOUT_TSAFS_RIGHT_JUSTIFIED         (0x1U << IISCNFOUT_TSAFS_POS)
#define IISCNFOUT_TSAFS_LEFT_JUSTIFIED          (0x2U << IISCNFOUT_TSAFS_POS)
#define IISCNFOUT_TSAFS_PCM                     (0x3U << IISCNFOUT_TSAFS_POS)

#define IISCNFOUT_TALOLRC_Pos                   (2U)
#define IISCNFOUT_TALOLRC_Msk                   (0x1U << IISCNFOUT_TALOLRC_Pos)
#define IISCNFOUT_TALOLRC_HIGHFORLEFT           IISCNFOUT_TALOLRC_Msk

#define IISCNFOUT_TX_VOICE_EN_Pos               (3U)
#define IISCNFOUT_TX_VOICE_EN_Msk               (0x1U << IISCNFOUT_TX_VOICE_EN_Pos)
#define IISCNFOUT_TX_VOICE_EN_MONO              IISCNFOUT_TX_VOICE_EN_Msk

#define IISCNFOUT_I2S_TXMODE_Pos                (4U)
#define IISCNFOUT_I2S_TXMODE_Msk                (0x1U << IISCNFOUT_I2S_TXMODE_Pos)
#define IISCNFOUT_I2S_TXMODE_SLAVE              IISCNFOUT_I2S_TXMODE_Msk

#define IISCNFOUT_TX_CLK_SEL_Pos                (5U)
#define IISCNFOUT_TX_CLK_SEL_Msk                (0x1U << IISCNFOUT_TX_CLK_SEL_Pos)
#define IISCNFOUT_TX_CLK_SEL_MCLK               IISCNFOUT_TX_CLK_SEL_Msk

/* FADTLR, offset: 0x14 */
#define FADTLR_96FTR_Pos                        (0U)
#define FADTLR_96FTR_Msk                        (0x3FU << FADTLR_96FTR_Pos)

#define FADTLR_88FTR_Pos                        (8U)
#define FADTLR_88FTR_Msk                        (0x3FU << FADTLR_88FTR_Pos)

#define FADTLR_64FTR_Pos                        (16U)
#define FADTLR_64FTR_Msk                        (0x3FU << FADTLR_64FTR_Pos)

#define FADTLR_192FTR_Pos                       (24U)
#define FADTLR_192FTR_Msk                       (0x3FU << FADTLR_192FTR_Pos)

/* SCCR, offset: 0x18 */
#define SCCR_RVCCR_Pos                          (0U)
#define SCCR_RVCCR_Msk                          (0x1FU << SCCR_RVCCR_Pos)

#define SCCR_SSRCR_Pos                          (5U)
#define SCCR_SSRCR_Msk                          (0x3U << SCCR_SSRCR_Pos)
#define SCCR_SSRCR_NO_COMPRESS                  (0x0U << SCCR_SSRCR_Pos)
#define SCCR_SSRCR_ONE_COMPRESS                 (0x1U << SCCR_SSRCR_Pos)
#define SCCR_SSRCR_THREE_COMPRESS               (0x2U << SCCR_SSRCR_Pos)

#define SCCR_TVCCR_Pos                          (8U)
#define SCCR_TVCCR_Msk                          (0x3U << SCCR_TVCCR_Pos)
#define SCCR_TVCCR_NO_COMPRESS                  (0x0U << SCCR_TVCCR_Pos)
#define SCCR_TVCCR_ONE_COMPRESS                 (0x1U << SCCR_TVCCR_Pos)
#define SCCR_TVCCR_THREE_COMPRESS               (0x2U << SCCR_TVCCR_Pos)

/* TXFTLR, offset: 0x1C */
#define TXFTLR_TFT_Pos                          (0U)
#define TXFTLR_TFT_Msk                          (0xFU << TXFTLR_TFT_Pos)

/* RXFTLR, offset: 0x20 */
#define RXFTLR_RFT_Pos                          (0U)
#define RXFTLR_RFT_Msk                          (0xFU << RXFTLR_RFT_Pos)

/* TXFLR, offset: 0x24 */
#define TXFLR_TXTFL_Pos                         (0U)

/* RXFLR, offset: 0x28 */
#define RXFLR_RXTFL_Pos                         (0U)

/* SR, offset: 0x2C */
#define SR_RXBUSY_Pos                           (0U)
#define SR_RXBUSY_Msk                           (0x1U << SR_RXBUSY_Pos)
#define SR_RXBUSY_STATUS                        SR_RXBUSY_Msk

#define SR_TXBUSY_Pos                           (1U)
#define SR_TXBUSY_Msk                           (0x1U << SR_TXBUSY_Pos)
#define SR_TXBUSY_STATUS                        SR_TXBUSY_Msk

#define SR_TFNF_Pos                             (2U)
#define SR_TFNF_Msk                             (0x1U << SR_TFNF_Pos)
#define SR_TFNF_TX_FIFO_NOT_FULL                SR_TFNF_Msk

#define SR_TFE_Pos                              (3U)
#define SR_TFE_Msk                              (0x1U << SR_TFE_Pos)
#define SR_TFE_TX_FIFO_EMPTY                    SR_TFE_Msk

#define SR_RFNE_Pos                             (4U)
#define SR_RFNE_Msk                             (0x1U << SR_RFNE_Pos)
#define SR_RFNE_RX_FIFO_NOT_EMPTY               SR_RFNE_Msk

#define SR_RFF_Pos                              (5U)
#define SR_RFF_Msk                              (0x1U << SR_RFF_Pos)
#define SR_RFF_RX_FIFO_FULL                     SR_RFF_Msk

/* IMR, offset: 0x30 */
#define IMR_WADEM_Pos                            (0U)
#define IMR_WADEM_Msk                            (0x1U << IMR_WADEM_Pos)
#define IMR_WADEM_INTR_MSK                       IMR_WADEM_Msk

#define IMR_TXUIRM_Pos                           (1U)
#define IMR_TXUIRM_Msk                           (0x1U << IMR_TXUIRM_Pos)
#define IMR_TXUIRM_INTR_MSK                      IMR_TXUIRM_Msk

#define IMR_TXOIM_Pos                            (2U)
#define IMR_TXOIM_Msk                            (0x1U << IMR_TXOIM_Pos)
#define IMR_TXOIM_INTR_MSK                       IMR_TXOIM_Msk

#define IMR_RXUIM_Pos                            (3U)
#define IMR_RXUIM_Msk                            (0x1U << IMR_RXUIM_Pos)
#define IMR_RXUIM_INTR_MSK                       IMR_RXUIM_Msk

#define IMR_RXOIM_Pos                            (4U)
#define IMR_RXOIM_Msk                            (0x1U << IMR_RXOIM_Pos)
#define IMR_RXOIM_INTR_MSK                       IMR_RXOIM_Msk

#define IMR_TXEIM_Pos                            (5U)
#define IMR_TXEIM_Msk                            (0x1U << IMR_TXEIM_Pos)
#define IMR_TXEIM_INTR_MSK                       IMR_TXEIM_Msk

#define IMR_RXFIM_Pos                            (6U)
#define IMR_RXFIM_Msk                            (0x1U << IMR_RXFIM_Pos)
#define IMR_RXFIM_INTR_MSK                       IMR_RXFIM_Msk

#define IMR_IRBFCM_Pos                           (7U)
#define IMR_IRBFCM_Msk                           (0x1U << IMR_IRBFCM_Pos)
#define IMR_IRBFCM_INTR_MSK                      IMR_IRBFCM_Msk

#define IMR_ITBFCM_Pos                           (8U)
#define IMR_ITBFCM_Msk                           (0x1U << IMR_ITBFCM_Pos)
#define IMR_ITBFCM_INTR_MSK                      IMR_ITBFCM_Msk

#define IMR_IFSCM_Pos                            (9U)
#define IMR_IFSCM_Msk                            (0x1U << IMR_IFSCM_Pos)
#define IMR_IFSCM_INTR_MSK                       IMR_IFSCM_Msk

/* ISR, offset: 0x34 */
#define ISR_WADES_Pos                            (0U)
#define ISR_WADES_Msk                            (0x1U << ISR_WADES_Pos)
#define ISR_WADES_STATUS                         ISR_WADES_Msk

#define ISR_TXUIRS_Pos                           (1U)
#define ISR_TXUIRS_Msk                           (0x1U << ISR_TXUIRS_Pos)
#define ISR_TXUIRS_STATUS                        ISR_TXUIRS_Msk

#define ISR_TXOIS_Pos                            (2U)
#define ISR_TXOIS_Msk                            (0x1U << ISR_TXOIS_Pos)
#define ISR_TXOIS_STATUS                         ISR_TXOIS_Msk

#define ISR_RXUIS_Pos                            (3U)
#define ISR_RXUIS_Msk                            (0x1U << ISR_RXUIS_Pos)
#define ISR_RXUIS_STATUS                         ISR_RXUIS_Msk

#define ISR_RXOIS_Pos                            (4U)
#define ISR_RXOIS_Msk                            (0x1U << ISR_RXOIS_Pos)
#define ISR_RXOIS_STATUS                         ISR_RXOIS_Msk

#define ISR_TXEIS_Pos                            (5U)
#define ISR_TXEIS_Msk                            (0x1U << ISR_TXEIS_Pos)
#define ISR_TXEIS_STATUS                         ISR_TXEIS_Msk

#define ISR_RXFIS_Pos                            (6U)
#define ISR_RXFIS_Msk                            (0x1U << ISR_RXFIS_Pos)
#define ISR_RXFIS_STATUS                         ISR_RXFIS_Msk

#define ISR_IRBFCS_Pos                           (7U)
#define ISR_IRBFCS_Msk                           (0x1U << ISR_IRBFCS_Pos)
#define ISR_IRBFCS_STATUS                        ISR_IRBFCS_Msk

#define ISR_ITBFCS_Pos                           (8U)
#define ISR_ITBFCS_Msk                           (0x1U << ISR_ITBFCS_Pos)
#define ISR_ITBFCS_STATUS                        ISR_ITBFCS_Msk

#define ISR_IFSCS_Pos                            (9U)
#define ISR_IFSCS_Msk                            (0x1U << ISR_IFSCS_Pos)
#define ISR_IFSCS_STATUS                         ISR_IFSCS_Msk

/* RISR, offset: 0x38 */
#define RISR_RWADES_Pos                           (0U)
#define RISR_RWADES_Msk                           (0x1U << RISR_RWADES_Pos)
#define RISR_RWADES_RAW                           RISR_RWADES_Msk

#define RISR_TXUIR_Pos                            (1U)
#define RISR_TXUIR_Msk                            (0x1U << RISR_TXUIR_Pos)
#define RISR_TXUIR_RAW                            RISR_TXUIR_Msk

#define RISR_TXOIR_Pos                           (2U)
#define RISR_TXOIR_Msk                           (0x1U << RISR_TXOIR_Pos)
#define RISR_TXOIR_RAW                           RISR_TXOIR_Msk

#define RISR_RXUIR_Pos                           (3U)
#define RISR_RXUIR_Msk                           (0x1U << RISR_RXUIR_Pos)
#define RISR_RXUIR_RAW                           RISR_RXUIR_Msk

#define RISR_RXOIR_Pos                           (4U)
#define RISR_RXOIR_Msk                           (0x1U << RISR_RXOIR_Pos)
#define RISR_RXOIR_RAW                           RISR_RXOIR_Msk

#define RISR_TXEIR_Pos                           (5U)
#define RISR_TXEIR_Msk                           (0x1U << RISR_TXEIR_Pos)
#define RISR_TXEIR_RAW                           RISR_TXEIR_Msk

#define RISR_RXFIR_Pos                           (6U)
#define RISR_RXFIR_Msk                           (0x1U << RISR_RXFIR_Pos)
#define RISR_RXFIR_RAW                           RISR_RXFIR_Msk

#define RISR_RIRBFCS_Pos                         (7U)
#define RISR_RIRBFCS_Msk                         (0x1U << RISR_RIRBFCS_Pos)
#define RISR_RIRBFCS_RAW                         RISR_RIRBFCS_Msk

#define RISR_RITBFCS_Pos                         (8U)
#define RISR_RITBFCS_Msk                         (0x1U << RISR_RITBFCS_Pos)
#define RISR_RITBFCS_RAW                         RISR_RITBFCS_Msk

#define RISR_RIFSCS_Pos                          (9U)
#define RISR_RIFSCS_Msk                          (0x1U << RISR_RIFSCS_Pos)
#define RISR_RIFSCS_RAW                          RISR_RIFSCS_Msk

/* ICR, offset: 0x3C */
#define ICR_CWADEC_Pos                           (0U)
#define ICR_CWADEC_Msk                           (0x1U << ICR_CWADEC_Pos)
#define ICR_CWADEC_CLEAR                         ICR_CWADEC_Msk

#define ICR_TXUIC_Pos                            (1U)
#define ICR_TXUIC_Msk                            (0x1U << ICR_TXUIC_Pos)
#define ICR_TXUIC_CLEAR                          ICR_TXUIC_Msk

#define ICR_TXOIC_Pos                            (2U)
#define ICR_TXOIC_Msk                            (0x1U << ICR_TXOIC_Pos)
#define ICR_TXOIC_CLEAR                          ICR_TXOIC_Msk

#define ICR_RXUIC_Pos                            (3U)
#define ICR_RXUIC_Msk                            (0x1U << ICR_RXUIC_Pos)
#define ICR_RXUIC_CLEAR                          ICR_RXUIC_Msk

#define ICR_RXOIC_Pos                            (4U)
#define ICR_RXOIC_Msk                            (0x1U << ICR_RXOIC_Pos)
#define ICR_RXOIC_CLEAR                          ICR_RXOIC_Msk

#define ICR_TXEIC_Pos                            (5U)
#define ICR_TXEIC_Msk                            (0x1U << ICR_TXEIC_Pos)
#define ICR_TXEIC_CLEAR                          ICR_TXEIC_Msk

#define ICR_RXFIC_Pos                            (6U)
#define ICR_RXFIC_Msk                            (0x1U << ICR_RXFIC_Pos)
#define ICR_RXFIC_CLEAR                          ICR_RXFIC_Msk

#define ICR_CRIRBFC_Pos                          (7U)
#define ICR_CRIRBFC_Msk                          (0x1U << ICR_CRIRBFC_Pos)
#define ICR_CRIRBFC_CLEAR                        ICR_CRIRBFC_Msk

#define ICR_CRITBFC_Pos                          (8U)
#define ICR_CRITBFC_Msk                          (0x1U << ICR_CRITBFC_Pos)
#define ICR_CRITBFC_CLEAR                        ICR_CRITBFC_Msk

#define ICR_CRIFSC_Pos                           (9U)
#define ICR_CRIFSC_Msk                           (0x1U << ICR_CRIFSC_Pos)
#define ICR_CRIFSC_CLEAR                         ICR_CRIFSC_Msk

/* DMACR, offset: 0x40 */
#define DMACR_RDMAE_POS                          (0U)
#define DMACR_RDMAE_MSK                          (0x1U << DMACR_RDMAE_POS)
#define DMACR_RDMAE_EN                           DMACR_RDMAE_MSK

#define DMACR_TDMAE_POS                          (1U)
#define DMACR_TDMAE_MSK                          (0x1U << DMACR_TDMAE_POS)
#define DMACR_TDMAE_EN                           DMACR_TDMAE_MSK

/* DMATDLR, offset: 0x44 */
#define DMATDLR_DMATDL_Pos                       (0U)
#define DMATDLR_DMATDL_Msk                       (0x1FU << DMATDLR_DMATDL_Pos)

/* DMARDLR, offset: 0x48 */
#define DMARDLR_DMATDL_Pos                       (0U)
#define DMARDLR_DMATDL_Msk                       (0x1FU << DMARDLR_DMATDL_Pos)

/* DR, offset: 0x4C */
#define DR_DR_Pos                                (0U)

/* DIV0LEVEL, offset: 0x50 */
#define DIV0LEVEL_DIV0_Pos                       (0U)
#define DIV0LEVEL_DIV0_Msk                       (0XFFU << DIV0LEVEL_DIV0_Pos)

/* DIV3LEVEL, offset: 0x54 */
#define DIV3LEVEL_DIV3_Pos                       (0U)
#define DIV0LEVEL_DIV3_Msk                       (0XFFU << DIV3LEVEL_DIV3_Pos)

#define I2S_DATA_WIDTH_8BIT                      (0xFU << FSSTA_DATAWTH_Pos)
#define I2S_DATA_8BIT_WIDTH_32BIT                (0xEU << FSSTA_DATAWTH_Pos)
#define I2S_DATA_WIDTH_16BIT                     (0U << FSSTA_DATAWTH_Pos)
#define I2S_DATA_16BIT_WIDTH_32BIT		 (0x2U << FSSTA_DATAWTH_Pos)
#define I2S_DATA_WIDTH_24BIT                     (0x5U << FSSTA_DATAWTH_Pos)
#define I2S_DATA_24BIT_WIDTH_32BIT		 (0x7U << FSSTA_DATAWTH_Pos)
#define I2S_DATA_WIDTH_32BIT                     (0xAU << FSSTA_DATAWTH_Pos)
#define I2S_DATA_WIDTH_32BIT_OUPUT               (0x8U << FSSTA_DATAWTH_Pos)

#define TXFIFO_IRQ_TH                               (0x8U)
#define RXFIFO_IRQ_TH                               (0x20U)
#define I2S_MAX_FIFO                                (0x20U)

/* AUDIO SYS DIV SEL REG, offset: 0x4 */
#define CPR_AUDIO_DIV1_SEL_POS                     (12U)
#define CPR_AUDIO_DIV1_SEL_MSK                     (0x1FU << CPR_AUDIO_DIV1_SEL_POS)
#define CPR_AUDIO_DIV1_SEL(X)                      (X << CPR_AUDIO_DIV1_SEL_POS)

/* AUDIO SYS CLK SEL REG, offset: 0x8 */
#define CPR_I2S0_SRC_SEL_POS                   (0U)
#define CPR_I2S0_SRC_SEL_MSK                   (0x3U << CPR_I2S0_SRC_SEL_POS)
#define CPR_I2S0_SRC_SEL(X)                    (X << CPR_I2S0_SRC_SEL_POS)
#define CPR_I2S0_SRC_SEL_24M                   (0x1U << AUDIOSYS_I2S0_SRC_SEL_POS)
#define CPR_I2S0_SRC_SEL_AUDIO_DIVCLK1         (0x2U << AUDIOSYS_I2S0_SRC_SEL_POS)

#define CPR_I2S1_SRC_SEL_POS                   (4U)
#define CPR_I2S1_SRC_SEL_MSK                   (0x3U << CPR_I2S1_SRC_SEL_POS)
#define CPR_I2S1_SRC_SEL(X)                    (X << CPR_I2S1_SRC_SEL_POS)
#define CPR_I2S1_SRC_SEL_24M                   (0x1U << AUDIOSYS_I2S1_SRC_SEL_POS)
#define CPR_I2S1_SRC_SEL_AUDIO_DIVCLK1         (0x2U << AUDIOSYS_I2S1_SRC_SEL_POS)

#define CPR_I2S2_SRC_SEL_POS                   (8U)
#define CPR_I2S2_SRC_SEL_MSK                   (0x3U << CPR_I2S2_SRC_SEL_POS)
#define CPR_I2S2_SRC_SEL(X)                    (X << CPR_I2S2_SRC_SEL_POS)
#define CPR_I2S2_SRC_SEL_24M                   (0x1U << AUDIOSYS_I2S2_SRC_SEL_POS)
#define CPR_I2S2_SRC_SEL_AUDIO_DIVCLK1         (0x2U << AUDIOSYS_I2S2_SRC_SEL_POS)

struct light_i2s_priv {
	void __iomem *base;
	phys_addr_t phys;

	void __iomem            *regs;
	struct regmap *regmap;
	struct regmap *audio_pin_regmap;
	struct regmap *audio_cpr_regmap;
	struct clk *clk;
	struct snd_dmaengine_dai_dma_data dma_params_tx;
	struct snd_dmaengine_dai_dma_data dma_params_rx;
	u32 fmt;
	unsigned int dai_fmt;
	u32 dma_maxburst;
	unsigned int cfg_off;

	struct device *dev;
	char name[16];
	int chan_num:16;
	unsigned int clk_master:1;
};

#endif /* _LIGHT_I2S_H */

