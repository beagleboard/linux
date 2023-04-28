#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/skbuff.h>

#define	DHD_STATIC_VERSION_STR		"101.10.361.26 (wlan=r892223-20221011-1)"
#define STATIC_ERROR_LEVEL	BIT(0)
#define STATIC_TRACE_LEVEL	BIT(1)
#define STATIC_MSG_LEVEL	BIT(0)
uint static_msg_level = STATIC_ERROR_LEVEL | STATIC_MSG_LEVEL;

#define DHD_STATIC_MSG(x, args...) \
	do { \
		if (static_msg_level & STATIC_MSG_LEVEL) { \
			pr_err("[dhd] STATIC-MSG) %s : " x, __func__, ## args); \
		} \
	} while (0)
#define DHD_STATIC_ERROR(x, args...) \
	do { \
		if (static_msg_level & STATIC_ERROR_LEVEL) { \
			pr_err("[dhd] STATIC-ERROR) %s : " x, __func__, ## args); \
		} \
	} while (0)
#define DHD_STATIC_TRACE(x, args...) \
	do { \
		if (static_msg_level & STATIC_TRACE_LEVEL) { \
			pr_err("[dhd] STATIC-TRACE) %s : " x, __func__, ## args); \
		} \
	} while (0)

#ifdef DHD_STATIC_IN_DRIVER
#if ANDROID_VERSION > 0
#define CONFIG_BCMDHD_VTS { : = y}
#define CONFIG_BCMDHD_DEBUG { : = y}
#endif
#else
#define BCMSDIO
#define BCMPCIE
//#define BCMDBUS
#define CONFIG_BCMDHD_VTS { : = y}
#define CONFIG_BCMDHD_DEBUG { : = y}
//#define BCMDHD_UNUSE_MEM
#endif

#ifndef MAX_NUM_ADAPTERS
#define MAX_NUM_ADAPTERS	1
#endif

enum dhd_prealloc_index {
	DHD_PREALLOC_PROT = 0,
#if defined(BCMSDIO)
	DHD_PREALLOC_RXBUF = 1,
	DHD_PREALLOC_DATABUF = 2,
#endif /* BCMSDIO */
	DHD_PREALLOC_OSL_BUF = 3,
	DHD_PREALLOC_SKB_BUF = 4,
	DHD_PREALLOC_WIPHY_ESCAN0 = 5,
	DHD_PREALLOC_WIPHY_ESCAN1 = 6,
	DHD_PREALLOC_DHD_INFO = 7,
#if defined(BCMSDIO) || defined(BCMDBUS)
	DHD_PREALLOC_DHD_WLFC_INFO = 8,
#endif /* BCMSDIO | BCMDBUS */
#ifdef BCMPCIE
	DHD_PREALLOC_IF_FLOW_LKUP = 9,
#endif /* BCMPCIE */
	DHD_PREALLOC_MEMDUMP_BUF = 10,
#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
	DHD_PREALLOC_MEMDUMP_RAM = 11,
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */
#if defined(BCMSDIO) || defined(BCMDBUS)
	DHD_PREALLOC_DHD_WLFC_HANGER = 12,
#endif /* BCMSDIO | BCMDBUS */
	DHD_PREALLOC_PKTID_MAP = 13,
	DHD_PREALLOC_PKTID_MAP_IOCTL = 14,
#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
	DHD_PREALLOC_DHD_LOG_DUMP_BUF = 15,
	DHD_PREALLOC_DHD_LOG_DUMP_BUF_EX = 16,
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */
	DHD_PREALLOC_DHD_PKTLOG_DUMP_BUF = 17,
	DHD_PREALLOC_STAT_REPORT_BUF = 18,
	DHD_PREALLOC_WL_ESCAN = 19,
	DHD_PREALLOC_FW_VERBOSE_RING = 20,
	DHD_PREALLOC_FW_EVENT_RING = 21,
	DHD_PREALLOC_DHD_EVENT_RING = 22,
#if defined(BCMDHD_UNUSE_MEM)
	DHD_PREALLOC_NAN_EVENT_RING = 23,
#endif /* BCMDHD_UNUSE_MEM */
	DHD_PREALLOC_MAX
};

#define STATIC_BUF_MAX_NUM	20
#define STATIC_BUF_SIZE	(PAGE_SIZE * 2)

#ifndef CUSTOM_LOG_DUMP_BUFSIZE_MB
/* DHD_LOG_DUMP_BUF_SIZE 4 MB static memory in kernel */
#define CUSTOM_LOG_DUMP_BUFSIZE_MB	4
#endif /* CUSTOM_LOG_DUMP_BUFSIZE_MB */

#define DHD_PREALLOC_PROT_SIZE	(16 * 1024)
#define DHD_PREALLOC_RXBUF_SIZE	(24 * 1024)
#define DHD_PREALLOC_DATABUF_SIZE	(64 * 1024)
#define DHD_PREALLOC_OSL_BUF_SIZE	(STATIC_BUF_MAX_NUM * STATIC_BUF_SIZE)
#define DHD_PREALLOC_WIPHY_ESCAN0_SIZE	(64 * 1024)
#define DHD_PREALLOC_DHD_INFO_SIZE	(43 * 1024)
#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
#define DHD_PREALLOC_MEMDUMP_RAM_SIZE	(1290 * 1024)
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */
#define DHD_PREALLOC_DHD_WLFC_HANGER_SIZE	(73 * 1024)
#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
#define DHD_PREALLOC_DHD_LOG_DUMP_BUF_SIZE (1024 * 1024 * CUSTOM_LOG_DUMP_BUFSIZE_MB)
#define DHD_PREALLOC_DHD_LOG_DUMP_BUF_EX_SIZE (8 * 1024)
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */
#define DHD_PREALLOC_WL_ESCAN_SIZE	(70 * 1024)
#ifdef CONFIG_64BIT
#define DHD_PREALLOC_IF_FLOW_LKUP_SIZE	(20 * 1024 * 2)
#else
#define DHD_PREALLOC_IF_FLOW_LKUP_SIZE	(20 * 1024)
#endif
#define FW_VERBOSE_RING_SIZE		(256 * 1024)
#define FW_EVENT_RING_SIZE		(64 * 1024)
#define DHD_EVENT_RING_SIZE		(64 * 1024)
#define NAN_EVENT_RING_SIZE		(64 * 1024)

#if defined(CONFIG_64BIT)
#define WLAN_DHD_INFO_BUF_SIZE	(24 * 1024)
#define WLAN_DHD_WLFC_BUF_SIZE	(64 * 1024)
#define WLAN_DHD_IF_FLOW_LKUP_SIZE	(64 * 1024)
#else
#define WLAN_DHD_INFO_BUF_SIZE	(16 * 1024)
#define WLAN_DHD_WLFC_BUF_SIZE	(64 * 1024)
#define WLAN_DHD_IF_FLOW_LKUP_SIZE	(20 * 1024)
#endif /* CONFIG_64BIT */
#define WLAN_DHD_MEMDUMP_SIZE	(800 * 1024)

#define DHD_SKB_1PAGE_BUFSIZE	(PAGE_SIZE * 1)
#define DHD_SKB_2PAGE_BUFSIZE	(PAGE_SIZE * 2)
#define DHD_SKB_4PAGE_BUFSIZE	(PAGE_SIZE * 4)

#ifdef BCMPCIE
#define DHD_SKB_1PAGE_BUF_NUM	0
#define DHD_SKB_2PAGE_BUF_NUM	192
#elif defined(BCMSDIO)
#define DHD_SKB_1PAGE_BUF_NUM	8
#define DHD_SKB_2PAGE_BUF_NUM	8
#endif /* BCMPCIE */
#define DHD_SKB_4PAGE_BUF_NUM	1

/* The number is defined in linux_osl.c
 * WLAN_SKB_1_2PAGE_BUF_NUM => STATIC_PKT_1_2PAGE_NUM
 * WLAN_SKB_BUF_NUM => STATIC_PKT_MAX_NUM
 */
#if defined(BCMSDIO) || defined(BCMPCIE)
#define WLAN_SKB_1_2PAGE_BUF_NUM ((DHD_SKB_1PAGE_BUF_NUM) + \
		(DHD_SKB_2PAGE_BUF_NUM))
#define WLAN_SKB_BUF_NUM ((WLAN_SKB_1_2PAGE_BUF_NUM) + (DHD_SKB_4PAGE_BUF_NUM))
#endif

void *wlan_static_prot[MAX_NUM_ADAPTERS] = {};
void *wlan_static_rxbuf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_databuf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_osl_buf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_scan_buf0[MAX_NUM_ADAPTERS] = {};
void *wlan_static_scan_buf1[MAX_NUM_ADAPTERS] = {};
void *wlan_static_dhd_info_buf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_dhd_wlfc_info_buf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_if_flow_lkup[MAX_NUM_ADAPTERS] = {};
void *wlan_static_dhd_memdump_ram_buf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_dhd_wlfc_hanger_buf[MAX_NUM_ADAPTERS] = {};
#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
void *wlan_static_dhd_log_dump_buf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_dhd_log_dump_buf_ex[MAX_NUM_ADAPTERS] = {};
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */
void *wlan_static_wl_escan_info_buf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_fw_verbose_ring_buf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_fw_event_ring_buf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_dhd_event_ring_buf[MAX_NUM_ADAPTERS] = {};
void *wlan_static_nan_event_ring_buf[MAX_NUM_ADAPTERS] = {};

#if defined(BCMSDIO) || defined(BCMPCIE)
static struct sk_buff *wlan_static_skb[MAX_NUM_ADAPTERS][WLAN_SKB_BUF_NUM] = {};
#endif /* BCMSDIO | BCMPCIE */

void *
dhd_wlan_mem_prealloc(
#if defined(BCMDHD_MDRIVER) && !defined(DHD_STATIC_IN_DRIVER)
	uint bus_type, int index,
#endif
	int section, unsigned long size)
{
#if !defined(BCMDHD_MDRIVER) || defined(DHD_STATIC_IN_DRIVER)
	int index = 0;
#endif

#if defined(BCMDHD_MDRIVER) && !defined(DHD_STATIC_IN_DRIVER)
	DHD_STATIC_MSG("bus_type %d, index %d, section %d, size %ld\n",
		bus_type, index, section, size);
#else
	DHD_STATIC_MSG("section %d, size %ld\n", section, size);
#endif

	if (section == DHD_PREALLOC_PROT)
		return wlan_static_prot[index];

#if defined(BCMSDIO)
	if (section == DHD_PREALLOC_RXBUF)
		return wlan_static_rxbuf[index];

	if (section == DHD_PREALLOC_DATABUF)
		return wlan_static_databuf[index];
#endif /* BCMSDIO */

#if defined(BCMSDIO) || defined(BCMPCIE)
	if (section == DHD_PREALLOC_SKB_BUF)
		return wlan_static_skb[index];
#endif /* BCMSDIO | BCMPCIE */

	if (section == DHD_PREALLOC_WIPHY_ESCAN0)
		return wlan_static_scan_buf0[index];

	if (section == DHD_PREALLOC_WIPHY_ESCAN1)
		return wlan_static_scan_buf1[index];

	if (section == DHD_PREALLOC_OSL_BUF) {
		if (size > DHD_PREALLOC_OSL_BUF_SIZE) {
			DHD_STATIC_ERROR("request OSL_BUF(%lu) > %ld\n",
				size, DHD_PREALLOC_OSL_BUF_SIZE);
			return NULL;
		}
		return wlan_static_osl_buf[index];
	}

	if (section == DHD_PREALLOC_DHD_INFO) {
		if (size > DHD_PREALLOC_DHD_INFO_SIZE) {
			DHD_STATIC_ERROR("request DHD_INFO(%lu) > %d\n",
				size, DHD_PREALLOC_DHD_INFO_SIZE);
			return NULL;
		}
		return wlan_static_dhd_info_buf[index];
	}
#if defined(BCMSDIO) || defined(BCMDBUS)
	if (section == DHD_PREALLOC_DHD_WLFC_INFO) {
		if (size > WLAN_DHD_WLFC_BUF_SIZE) {
			DHD_STATIC_ERROR("request DHD_WLFC_INFO(%lu) > %d\n",
				size, WLAN_DHD_WLFC_BUF_SIZE);
			return NULL;
		}
		return wlan_static_dhd_wlfc_info_buf[index];
	}
#endif /* BCMSDIO | BCMDBUS */
#ifdef BCMPCIE
	if (section == DHD_PREALLOC_IF_FLOW_LKUP)  {
		if (size > DHD_PREALLOC_IF_FLOW_LKUP_SIZE) {
			DHD_STATIC_ERROR("request DHD_IF_FLOW_LKUP(%lu) > %d\n",
				size, DHD_PREALLOC_IF_FLOW_LKUP_SIZE);
			return NULL;
		}
		return wlan_static_if_flow_lkup[index];
	}
#endif /* BCMPCIE */
#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
	if (section == DHD_PREALLOC_MEMDUMP_RAM) {
		if (size > DHD_PREALLOC_MEMDUMP_RAM_SIZE) {
			DHD_STATIC_ERROR("request DHD_PREALLOC_MEMDUMP_RAM(%lu) > %d\n",
				size, DHD_PREALLOC_MEMDUMP_RAM_SIZE);
			return NULL;
		}
		return wlan_static_dhd_memdump_ram_buf[index];
	}
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */
#if defined(BCMSDIO) || defined(BCMDBUS)
	if (section == DHD_PREALLOC_DHD_WLFC_HANGER) {
		if (size > DHD_PREALLOC_DHD_WLFC_HANGER_SIZE) {
			DHD_STATIC_ERROR("request DHD_WLFC_HANGER(%lu) > %d\n",
				size, DHD_PREALLOC_DHD_WLFC_HANGER_SIZE);
			return NULL;
		}
		return wlan_static_dhd_wlfc_hanger_buf[index];
	}
#endif /* BCMSDIO | BCMDBUS */
#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
	if (section == DHD_PREALLOC_DHD_LOG_DUMP_BUF) {
		if (size > DHD_PREALLOC_DHD_LOG_DUMP_BUF_SIZE) {
			DHD_STATIC_ERROR("request DHD_PREALLOC_DHD_LOG_DUMP_BUF(%lu) > %d\n",
				size, DHD_PREALLOC_DHD_LOG_DUMP_BUF_SIZE);
			return NULL;
		}
		return wlan_static_dhd_log_dump_buf[index];
	}
	if (section == DHD_PREALLOC_DHD_LOG_DUMP_BUF_EX) {
		if (size > DHD_PREALLOC_DHD_LOG_DUMP_BUF_EX_SIZE) {
			DHD_STATIC_ERROR("request DHD_PREALLOC_DHD_LOG_DUMP_BUF_EX(%lu) > %d\n",
				size, DHD_PREALLOC_DHD_LOG_DUMP_BUF_EX_SIZE);
			return NULL;
		}
		return wlan_static_dhd_log_dump_buf_ex[index];
	}
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */
	if (section == DHD_PREALLOC_WL_ESCAN) {
		if (size > DHD_PREALLOC_WL_ESCAN_SIZE) {
			DHD_STATIC_ERROR("request DHD_PREALLOC_WL_ESCAN(%lu) > %d\n",
				size, DHD_PREALLOC_WL_ESCAN_SIZE);
			return NULL;
		}
		return wlan_static_wl_escan_info_buf[index];
	}
	if (section == DHD_PREALLOC_FW_VERBOSE_RING) {
		if (size > FW_VERBOSE_RING_SIZE) {
			DHD_STATIC_ERROR("request DHD_PREALLOC_FW_VERBOSE_RING(%lu) > %d\n",
				size, FW_VERBOSE_RING_SIZE);
			return NULL;
		}
		return wlan_static_fw_verbose_ring_buf[index];
	}
	if (section == DHD_PREALLOC_FW_EVENT_RING) {
		if (size > FW_EVENT_RING_SIZE) {
			DHD_STATIC_ERROR("request DHD_PREALLOC_FW_EVENT_RING(%lu) > %d\n",
				size, FW_EVENT_RING_SIZE);
			return NULL;
		}
		return wlan_static_fw_event_ring_buf[index];
	}
	if (section == DHD_PREALLOC_DHD_EVENT_RING) {
		if (size > DHD_EVENT_RING_SIZE) {
			DHD_STATIC_ERROR("request DHD_PREALLOC_DHD_EVENT_RING(%lu) > %d\n",
				size, DHD_EVENT_RING_SIZE);
			return NULL;
		}
		return wlan_static_dhd_event_ring_buf[index];
	}
#if defined(BCMDHD_UNUSE_MEM)
	if (section == DHD_PREALLOC_NAN_EVENT_RING) {
		if (size > NAN_EVENT_RING_SIZE) {
			DHD_STATIC_ERROR("request DHD_PREALLOC_NAN_EVENT_RING(%lu) > %d\n",
				size, NAN_EVENT_RING_SIZE);
			return NULL;
		}
		return wlan_static_nan_event_ring_buf[index];
	}
#endif /* BCMDHD_UNUSE_MEM */
	if ((section < 0) || (section > DHD_PREALLOC_MAX))
		DHD_STATIC_ERROR("request section id(%d) is out of max index %d\n",
			section, DHD_PREALLOC_MAX);

	DHD_STATIC_ERROR("failed to alloc section %d, size=%ld\n",
		section, size);

	return NULL;
}
#ifndef DHD_STATIC_IN_DRIVER
EXPORT_SYMBOL(dhd_wlan_mem_prealloc);
#endif

static void
dhd_deinit_wlan_mem(int index)
{
#if defined(BCMSDIO) || defined(BCMPCIE)
	int i;
#endif /* BCMSDIO | BCMPCIE */

	if (wlan_static_prot[index])
		kfree(wlan_static_prot[index]);
#if defined(BCMSDIO)
	if (wlan_static_rxbuf[index])
		kfree(wlan_static_rxbuf[index]);
	if (wlan_static_databuf[index])
		kfree(wlan_static_databuf[index]);
#endif /* BCMSDIO */
	if (wlan_static_osl_buf[index])
		kfree(wlan_static_osl_buf[index]);
	if (wlan_static_scan_buf0[index])
		kfree(wlan_static_scan_buf0[index]);
	if (wlan_static_scan_buf1[index])
		kfree(wlan_static_scan_buf1[index]);
	if (wlan_static_dhd_info_buf[index])
		kfree(wlan_static_dhd_info_buf[index]);
#if defined(BCMSDIO) || defined(BCMDBUS)
	if (wlan_static_dhd_wlfc_info_buf[index])
		kfree(wlan_static_dhd_wlfc_info_buf[index]);
#endif /* BCMSDIO | BCMDBUS */
#ifdef BCMPCIE
	if (wlan_static_if_flow_lkup[index])
		kfree(wlan_static_if_flow_lkup[index]);
#endif /* BCMPCIE */
#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
	if (wlan_static_dhd_memdump_ram_buf[index])
		kfree(wlan_static_dhd_memdump_ram_buf[index]);
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */
#if defined(BCMSDIO) || defined(BCMDBUS)
	if (wlan_static_dhd_wlfc_hanger_buf[index])
		kfree(wlan_static_dhd_wlfc_hanger_buf[index]);
#endif /* BCMSDIO | BCMDBUS */
#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
	if (wlan_static_dhd_log_dump_buf[index])
		kfree(wlan_static_dhd_log_dump_buf[index]);
	if (wlan_static_dhd_log_dump_buf_ex[index])
		kfree(wlan_static_dhd_log_dump_buf_ex[index]);
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */
	if (wlan_static_wl_escan_info_buf[index])
		kfree(wlan_static_wl_escan_info_buf[index]);
	if (wlan_static_fw_verbose_ring_buf[index])
		kfree(wlan_static_fw_verbose_ring_buf[index]);
	if (wlan_static_fw_event_ring_buf[index])
		kfree(wlan_static_fw_event_ring_buf[index]);
	if (wlan_static_dhd_event_ring_buf[index])
		kfree(wlan_static_dhd_event_ring_buf[index]);
#if defined(BCMDHD_UNUSE_MEM)
	if (wlan_static_nan_event_ring_buf[index])
		kfree(wlan_static_nan_event_ring_buf[index]);
#endif /* BCMDHD_UNUSE_MEM */

#if defined(BCMSDIO) || defined(BCMPCIE)
	for (i=0; i<WLAN_SKB_BUF_NUM; i++) {
		if (wlan_static_skb[index][i])
			dev_kfree_skb(wlan_static_skb[index][i]);
	}
#endif /* BCMSDIO | BCMPCIE */

	return;
}

static int
dhd_init_wlan_mem(int index)
{
#if defined(BCMSDIO) || defined(BCMPCIE)
	int i;
#endif
	unsigned long size = 0;

#if defined(BCMSDIO) || defined(BCMPCIE)
	for (i = 0; i < WLAN_SKB_BUF_NUM; i++)
		wlan_static_skb[index][i] = NULL;

	for (i = 0; i < DHD_SKB_1PAGE_BUF_NUM; i++) {
		wlan_static_skb[index][i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[index][i])
			goto err_mem_alloc;
		size += DHD_SKB_1PAGE_BUFSIZE;
		DHD_STATIC_TRACE("section %d skb[%d], size=%ld\n",
			DHD_PREALLOC_SKB_BUF, i, DHD_SKB_1PAGE_BUFSIZE);
	}

	for (i = DHD_SKB_1PAGE_BUF_NUM; i < WLAN_SKB_1_2PAGE_BUF_NUM; i++) {
		wlan_static_skb[index][i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[index][i])
			goto err_mem_alloc;
		size += DHD_SKB_2PAGE_BUFSIZE;
		DHD_STATIC_TRACE("section %d skb[%d], size=%ld\n",
			DHD_PREALLOC_SKB_BUF, i, DHD_SKB_2PAGE_BUFSIZE);
	}
#endif /* BCMSDIO | BCMPCIE */

#if defined(BCMSDIO)
	wlan_static_skb[index][i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[index][i])
		goto err_mem_alloc;
	size += DHD_SKB_4PAGE_BUFSIZE;
	DHD_STATIC_TRACE("section %d skb[%d], size=%ld\n",
		DHD_PREALLOC_SKB_BUF, i, DHD_SKB_4PAGE_BUFSIZE);
#endif /* BCMSDIO */

	wlan_static_prot[index] = kmalloc(DHD_PREALLOC_PROT_SIZE, GFP_KERNEL);
	if (!wlan_static_prot[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_PROT_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_PROT, DHD_PREALLOC_PROT_SIZE);

#if defined(BCMSDIO)
	wlan_static_rxbuf[index] = kmalloc(DHD_PREALLOC_RXBUF_SIZE, GFP_KERNEL);
	if (!wlan_static_rxbuf[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_RXBUF_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_RXBUF, DHD_PREALLOC_RXBUF_SIZE);

	wlan_static_databuf[index] = kmalloc(DHD_PREALLOC_DATABUF_SIZE, GFP_KERNEL);
	if (!wlan_static_databuf[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_DATABUF_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_DATABUF, DHD_PREALLOC_DATABUF_SIZE);
#endif /* BCMSDIO */

	wlan_static_osl_buf[index] = kmalloc(DHD_PREALLOC_OSL_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_osl_buf[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_OSL_BUF_SIZE;
	DHD_STATIC_TRACE("section %d, size=%ld\n",
		DHD_PREALLOC_OSL_BUF, DHD_PREALLOC_OSL_BUF_SIZE);

	wlan_static_scan_buf0[index] = kmalloc(DHD_PREALLOC_WIPHY_ESCAN0_SIZE, GFP_KERNEL);
	if (!wlan_static_scan_buf0[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_WIPHY_ESCAN0_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_WIPHY_ESCAN0, DHD_PREALLOC_WIPHY_ESCAN0_SIZE);

	wlan_static_dhd_info_buf[index] = kmalloc(DHD_PREALLOC_DHD_INFO_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_info_buf[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_DHD_INFO_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_DHD_INFO, DHD_PREALLOC_DHD_INFO_SIZE);

#if defined(BCMSDIO) || defined(BCMDBUS)
	wlan_static_dhd_wlfc_info_buf[index] = kmalloc(WLAN_DHD_WLFC_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_wlfc_info_buf[index])
		goto err_mem_alloc;
	size += WLAN_DHD_WLFC_BUF_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_DHD_WLFC_INFO, WLAN_DHD_WLFC_BUF_SIZE);
#endif /* BCMSDIO | BCMDBUS */

#ifdef BCMPCIE
	wlan_static_if_flow_lkup[index] = kmalloc(DHD_PREALLOC_IF_FLOW_LKUP_SIZE, GFP_KERNEL);
	if (!wlan_static_if_flow_lkup[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_IF_FLOW_LKUP_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_IF_FLOW_LKUP, DHD_PREALLOC_IF_FLOW_LKUP_SIZE);
#endif /* BCMPCIE */

#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
	wlan_static_dhd_memdump_ram_buf[index] = kmalloc(DHD_PREALLOC_MEMDUMP_RAM_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_memdump_ram_buf[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_MEMDUMP_RAM_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_MEMDUMP_RAM, DHD_PREALLOC_MEMDUMP_RAM_SIZE);
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */

#if defined(BCMSDIO) || defined(BCMDBUS)
	wlan_static_dhd_wlfc_hanger_buf[index] = kmalloc(DHD_PREALLOC_DHD_WLFC_HANGER_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_wlfc_hanger_buf[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_DHD_WLFC_HANGER_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_DHD_WLFC_HANGER, DHD_PREALLOC_DHD_WLFC_HANGER_SIZE);
#endif /* BCMSDIO | BCMDBUS */

#if defined(CONFIG_BCMDHD_VTS) || defined(CONFIG_BCMDHD_DEBUG)
	wlan_static_dhd_log_dump_buf[index] = kmalloc(DHD_PREALLOC_DHD_LOG_DUMP_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_log_dump_buf[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_DHD_LOG_DUMP_BUF_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_DHD_LOG_DUMP_BUF, DHD_PREALLOC_DHD_LOG_DUMP_BUF_SIZE);

	wlan_static_dhd_log_dump_buf_ex[index] = kmalloc(DHD_PREALLOC_DHD_LOG_DUMP_BUF_EX_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_log_dump_buf_ex[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_DHD_LOG_DUMP_BUF_EX_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_DHD_LOG_DUMP_BUF_EX, DHD_PREALLOC_DHD_LOG_DUMP_BUF_EX_SIZE);
#endif /* CONFIG_BCMDHD_VTS | CONFIG_BCMDHD_DEBUG */

	wlan_static_wl_escan_info_buf[index] = kmalloc(DHD_PREALLOC_WL_ESCAN_SIZE, GFP_KERNEL);
	if (!wlan_static_wl_escan_info_buf[index])
		goto err_mem_alloc;
	size += DHD_PREALLOC_WL_ESCAN_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_WL_ESCAN, DHD_PREALLOC_WL_ESCAN_SIZE);

	wlan_static_fw_verbose_ring_buf[index] = kmalloc(FW_VERBOSE_RING_SIZE, GFP_KERNEL);
	if (!wlan_static_fw_verbose_ring_buf[index])
		goto err_mem_alloc;
	size += FW_VERBOSE_RING_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_FW_VERBOSE_RING, FW_VERBOSE_RING_SIZE);

	wlan_static_fw_event_ring_buf[index] = kmalloc(FW_EVENT_RING_SIZE, GFP_KERNEL);
	if (!wlan_static_fw_event_ring_buf[index])
		goto err_mem_alloc;
	size += FW_EVENT_RING_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_FW_EVENT_RING, FW_EVENT_RING_SIZE);

	wlan_static_dhd_event_ring_buf[index] = kmalloc(DHD_EVENT_RING_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_event_ring_buf[index])
		goto err_mem_alloc;
	size += DHD_EVENT_RING_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_DHD_EVENT_RING, DHD_EVENT_RING_SIZE);

#if defined(BCMDHD_UNUSE_MEM)
	wlan_static_nan_event_ring_buf[index] = kmalloc(NAN_EVENT_RING_SIZE, GFP_KERNEL);
	if (!wlan_static_nan_event_ring_buf[index])
		goto err_mem_alloc;
	size += NAN_EVENT_RING_SIZE;
	DHD_STATIC_TRACE("section %d, size=%d\n",
		DHD_PREALLOC_NAN_EVENT_RING, NAN_EVENT_RING_SIZE);
#endif /* BCMDHD_UNUSE_MEM */

	DHD_STATIC_MSG("prealloc ok for index %d: %ld(%ldK)\n", index, size, size / 1024);
	return 0;

err_mem_alloc:
	DHD_STATIC_ERROR("Failed to allocate memory for index %d\n", index);

	return -ENOMEM;
}

#ifdef DHD_STATIC_IN_DRIVER
int
#else
static int __init
#endif
dhd_static_buf_init(void)
{
	int i, ret = 0;

 	DHD_STATIC_MSG("%s\n", DHD_STATIC_VERSION_STR);

	for (i=0; i<MAX_NUM_ADAPTERS; i++) {
		ret = dhd_init_wlan_mem(i);
		if (ret)
			break;
	}

	if (ret) {
		for (i=0; i<MAX_NUM_ADAPTERS; i++)
			dhd_deinit_wlan_mem(i);
	}

	return ret;
}

#ifdef DHD_STATIC_IN_DRIVER
void
#else
static void __exit
#endif
dhd_static_buf_exit(void)
{
	int i;

 	DHD_STATIC_MSG("Enter\n");

	for (i=0; i<MAX_NUM_ADAPTERS; i++)
		dhd_deinit_wlan_mem(i);
}

#ifndef DHD_STATIC_IN_DRIVER
module_init(dhd_static_buf_init);
module_exit(dhd_static_buf_exit);
#endif
