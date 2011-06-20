#ifndef __CONFIG_CONFIG_PWM
#define __CONFIG_CONFIG_PWM

#define AM33XX_CONFIG_BASE      (0x0)
#define AM33XX_CONFIG_SIZE      (AM33XX_CONFIG_BASE + 0x10)
#define AM33XX_ECAP_BASE        (0x0100)
#define AM33XX_ECAP_SIZE        (AM33XX_ECAP_BASE + 0x080)
#define AM33XX_EQEP_BASE        (0x0180)
#define AM33XX_EQeP_SIZE        (AM33XX_EQEP_BASE + 0x080)
#define AM33XX_EPWM_BASE        (0x0200)
#define AM33XX_EPWM_SIZE        (AM33XX_EPWM_BASE + 0x0100)

#define PWMSS_CLKCONFIG          (0x08)
#define ECAP_CLK_EN              (0x0)
#define ECAP_CLK_STOP_REQ        (0x1)
#define EQEP_CLK_EN              (0x4)
#define EQEP_CLK_STOP_REQ        (0x5)
#define EPWM_CLK_EN              (0x8)
#define EPWM_CLK_STOP_REQ        (0x9)

#define SET                      (1)
#define CLEAR                    (0)

#define PWM_CON_ID_STRING_LENGTH (12)

#endif
