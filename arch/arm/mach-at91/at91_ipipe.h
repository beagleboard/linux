#ifndef AT91_IPIPE_H
#define AT91_IPIPE_H

#include <linux/ipipe.h>

#ifdef CONFIG_IPIPE
void at91_pic_muter_register(void);
#else /* !CONFIG_IPIPE */
#define at91_pic_muter_register() do { } while (0)
#endif /* CONFIG_IPIPE */

#endif /* AT91_IPIPE_TIME_H */
