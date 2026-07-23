#include <linux/device.h>
#include <linux/kstrtox.h>
#include "rx.h"
#include "debug.h"
#include "cc33xx.h"
#include "sysfs.h"
#include "acx.h"

static ssize_t ble_enable_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&cc->mutex);
	len = sysfs_emit(buf, "%d\n", cc->ble_enable);
	mutex_unlock(&cc->mutex);

	return len;
}

static ssize_t ble_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	int ret;
	unsigned long value;

	ret = kstrtoul(buf, 10, &value);
	if (value != 1) {
		cc33xx_warning("illegal value in ble_enable (only value allowed is is 1)");
		cc33xx_warning("ble_enable cant be disabled after being enabled.");
		return -EINVAL;
	}

	if (value == cc->ble_enable) {
		cc33xx_warning("ble_enable is already %d",cc->ble_enable);
		return -EINVAL;
	}

	mutex_lock(&cc->mutex);

	if (unlikely(cc->state != CC33XX_STATE_ON)) {
		/* this will show up on "read" in case we are off */
		cc->ble_enable = value;
		goto out;
	}

	cc33xx_ble_enable(cc, value);
out:
	mutex_unlock(&cc->mutex);
	return count;
}
static DEVICE_ATTR_RW(ble_enable);

static ssize_t slow_clock_type_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	ssize_t len;

	cc33xx_acx_get_slow_clock_type(cc);

	len = sysfs_emit(buf, "%d\n", cc->is_ext_slw_clk);

	return len;
}

static ssize_t slow_clock_type_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	return 0;
}
static DEVICE_ATTR_RW(slow_clock_type);

static ssize_t cc33xx_sysfs_read_fwlog(struct file *filp, struct kobject *kobj,
				       struct bin_attribute *bin_attr,
				       char *buffer, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct cc33xx *cc = dev_get_drvdata(dev);
	ssize_t len;
	int ret;

	ret = mutex_lock_interruptible(&cc->mutex);
	if (ret < 0)
		return -ERESTARTSYS;

	/* Check if the fwlog is still valid */
	if (cc->fwlog_size < 0) {
		mutex_unlock(&cc->mutex);
		return 0;
	}

	/* Seeking is not supported - old logs are not kept. Disregard pos. */
	len = min_t(size_t, count, cc->fwlog_size);
	cc->fwlog_size -= len;
	memcpy(buffer, cc->fwlog, len);

	/* Make room for new messages */
	memmove(cc->fwlog, cc->fwlog + len, cc->fwlog_size);

	mutex_unlock(&cc->mutex);

	return len;
}

static const struct bin_attribute fwlog_attr = {
	.attr = { .name = "fwlog", .mode = 0400 },
	.read = cc33xx_sysfs_read_fwlog,
};

static ssize_t regdomain_txControl_param_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	return 0;
}

static ssize_t regdomain_txControl_param_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	struct acx_phy_regdomain_tx_control_params params;
	int ret = 0;
	char* buffer;
	char * pToken;
	int converted_token = 0;

	buffer = kzalloc(count, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		cc33xx_warning("error in regdomain and tx control params set (memory): %d", ret);
		return ret;
	}

	strncpy(buffer, buf, count);
	if(-EFAULT == ret){
		cc33xx_warning("error in regdomain and tx control params set: %d", ret);
		kfree(buffer);
		return ret;
	}

	pToken = strsep(&buffer, " ");

	ret = kstrtoint(pToken, 10, &converted_token);
	if (ret < 0)
	{
		ret = -EINVAL;
		kfree(buffer);
		cc33xx_warning("error in bitmask value parsing");
		return ret;
    }

	params.bitmask = converted_token;


	pToken = strsep(&buffer, " ");

	ret = kstrtoint(pToken, 10, &converted_token);
	if (ret < 0)
	{
		ret = -EINVAL;
		kfree(buffer);
		cc33xx_warning("error in reg_domain value parsing");
		return ret;
    }

	params.reg_domain = converted_token;

	pToken = strsep(&buffer, " ");

	for(int i = 0; i < BLE_LIM_CHANNELS_COUNT ; i++)
	{
		ret = kstrtoint(pToken, 10, &converted_token);
		if (ret < 0)
		{
			ret = -EINVAL;
			kfree(buffer);
			cc33xx_warning("error in ble_ch_lim_1M value parsing");
			return ret;
		}

		params.ble_ch_lim_1M[i] = converted_token;

		pToken = strsep(&buffer, " ");
	}

	for(int i = 0; i < BLE_LIM_CHANNELS_COUNT ;i++)
	{

		ret = kstrtoint(pToken, 10, &converted_token);

		if (ret < 0)
		{
			ret = -EINVAL;
			kfree(buffer);
			cc33xx_warning("error in ble_ch_lim_2M value parsing");
			return ret;
		}

		params.ble_ch_lim_2M[i] = converted_token;

		pToken = strsep(&buffer, " ");
	}

	ret = kstrtoint(pToken, 10, &converted_token);
	if (ret < 0)
	{
		ret = -EINVAL;
		kfree(buffer);
		cc33xx_warning("error in country_code value parsing");
		return ret;
	}

	params.country_code = converted_token;

	pToken = strsep(&buffer, " ");

	for(int i = 0; i < REG_RULES_COUNT ; i++)
	{
		ret = kstrtoint(pToken, 10, &converted_token);
		if (ret < 0)
		{
			ret = -EINVAL;
			kfree(buffer);
			cc33xx_warning("error in country_code value parsing");
			return ret;
		}

		params.per_channel_power_limit[i] = converted_token;

		pToken = strsep(&buffer, " ");
	}

	kfree(buffer);

	ret = cc33xx_acx_set_regdoamin_and_tx_control_params(cc,&params);


	mutex_lock(&cc->mutex);

	if (unlikely(cc->state != CC33XX_STATE_ON)) {
		goto out;
	}



out:
	mutex_unlock(&cc->mutex);
	return count;
}

static DEVICE_ATTR_RW(regdomain_txControl_param);


static ssize_t wowlan_arp_offload_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	ssize_t len;

	if (!cc)
		return sysfs_emit(buf, "Device not ready\n");

	mutex_lock(&cc->mutex);
	len = sysfs_emit(buf, "Current state: %s\n\n",
			 cc->wowlan_arp_offload ? "ENABLED" : "DISABLED");
	mutex_unlock(&cc->mutex);

	len += sysfs_emit_at(buf, len, "Usage:\n");
	len += sysfs_emit_at(buf, len, "  echo 1 > wowlan_arp_offload    # Enable WoWLAN ARP offload\n");
	len += sysfs_emit_at(buf, len, "  echo 0 > wowlan_arp_offload    # Disable WoWLAN ARP offload\n");

	return len;
}

static ssize_t wowlan_arp_offload_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	unsigned int value;
	int ret;

	ret = kstrtouint(buf, 10, &value);
	if (ret < 0)
		return ret;

	if (value != 0 && value != 1) {
		cc33xx_warning("invalid WoWLAN ARP offload value %u (must be 0 or 1)", value);
		return -EINVAL;
	}

	mutex_lock(&cc->mutex);

	cc->wowlan_arp_offload = value;
	cc33xx_info("WoWLAN ARP offload %s (will take effect on next suspend)\n",
		    cc->wowlan_arp_offload ? "enabled" : "disabled");

	mutex_unlock(&cc->mutex);

	return count;
}

static DEVICE_ATTR_RW(wowlan_arp_offload);

static ssize_t wowlan_pattern_clear_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	int ret;
	unsigned int clear_flag;

	ret = kstrtouint(buf, 10, &clear_flag);
	if (ret < 0)
		return ret;

	if (clear_flag != 1) {
		cc33xx_error("Invalid value %u: must be 1 to clear patterns", clear_flag);
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&cc->mutex);
	if (ret < 0)
	return -ERESTARTSYS;

	if (unlikely(cc->state != CC33XX_STATE_ON)) {
		cc33xx_error("Device not ready for pattern clear");
		ret = -EAGAIN;
		goto out_unlock;
	}

	ret = cc33xx_clear_wowlan_search_patterns(cc);
	if (ret < 0)
		goto out_unlock;

	mutex_unlock(&cc->mutex);
	return count;

out_unlock:
	mutex_unlock(&cc->mutex);
	return ret;
}

static ssize_t wowlan_pattern_clear_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	return sysfs_emit(buf, "Write 1 to clear all WoWLAN patterns\n");
}

static DEVICE_ATTR_RW(wowlan_pattern_clear);

static ssize_t wowlan_mode_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	unsigned int enable;
	int ret;

	ret = kstrtouint(buf, 10, &enable);
	if (ret < 0)
		return ret;

	if (enable != 0 && enable != 1) {
		cc33xx_error("Invalid value %u: must be 0 (disable) or 1 (enable)", enable);
		return -EINVAL;
	}

	ret = mutex_lock_interruptible(&cc->mutex);
	if (ret < 0)
	return -ERESTARTSYS;

	if (unlikely(cc->state != CC33XX_STATE_ON)) {
		cc33xx_error("Device not ready (state: %d)", cc->state);
		ret = -EAGAIN;
		goto out;
	}

	ret = cc33xx_set_wowlan_search_mode(cc, enable != 0);
	if (ret < 0)
		goto out;

	mutex_unlock(&cc->mutex);
	return count;

out:
	mutex_unlock(&cc->mutex);
	return ret;
}

static ssize_t wowlan_mode_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	ssize_t len;

	if (!cc)
		return sysfs_emit(buf, "Device not ready\n");

	len = sysfs_emit(buf, "Current mode: %s\n\n",
			 cc->wowlan_search.enabled ? "ENABLED" : "DISABLED");
	len += sysfs_emit_at(buf, len, "Usage:\n");
	len += sysfs_emit_at(buf, len, "  echo 1 > wowlan_mode    # Enable WoWLAN\n");
	len += sysfs_emit_at(buf, len, "  echo 0 > wowlan_mode    # Disable WoWLAN\n");

	return len;
}

static DEVICE_ATTR_RW(wowlan_mode);

static ssize_t wowlan_pattern_search_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct cc33xx *cc = dev_get_drvdata(dev);
	char *input;
	int ret;

	if (unlikely(cc->state != CC33XX_STATE_ON)) {
		cc33xx_error("Device not ready for adding pattern");
		return -EAGAIN;
	}

	input = kzalloc(count + 1, GFP_KERNEL);
	if (!input)
		return -ENOMEM;

	memcpy(input, buf, count);
	input[count] = '\0';

	if (count > 0 && input[count - 1] == '\n')
		input[count - 1] = '\0';

	ret = mutex_lock_interruptible(&cc->mutex);
	if (ret < 0) {
		ret = -ERESTARTSYS;
		goto out_free;
	}

	ret = cc33xx_add_wowlan_search_pattern(cc, input);
	mutex_unlock(&cc->mutex);

	if (ret < 0)
		goto out_free;

	kfree(input);
	return count;

out_free:
	kfree(input);
	return ret;
}

static ssize_t wowlan_pattern_search_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	ssize_t len;
	struct cc33xx_rx_filter *filter;
	struct cc33xx_rx_filter_field *field;
	struct cc33xx *cc = dev_get_drvdata(dev);
	int i, j, k;
	u16 offset;
	bool has_mask;

	if (!cc)
		return sysfs_emit(buf, "Device not ready\n");

	if (mutex_lock_interruptible(&cc->mutex))
		return -ERESTARTSYS;

	len = sysfs_emit(buf, "Active filters: %d/%d\n",
			 cc->wowlan_search.filter_count, CC33XX_MAX_RX_FILTERS);

	if (cc->wowlan_search.filter_count > 0) {
		len += sysfs_emit_at(buf, len, "\n");
		for (i = 0; i < cc->wowlan_search.filter_count; i++) {
			filter = cc->wowlan_search.active_filters[i];
			if (!filter || filter->num_fields == 0)
				continue;

			for (j = 0; j < filter->num_fields; j++) {
				field = &filter->fields[j];
				offset = le16_to_cpu(field->offset);
				has_mask = field->flags & CC33XX_RX_FILTER_FLAG_MASKED;

				len += sysfs_emit_at(buf, len, "  [%d] offset=%d: ", i, offset);

				for (k = 0; k < field->len; k++) {
					len += sysfs_emit_at(buf, len, "%02x%s",
							     field->pattern[k],
							     k < field->len - 1 ? ":" : "");
				}

				if (has_mask)
					len += sysfs_emit_at(buf, len, " (masked)");

				len += sysfs_emit_at(buf, len, "\n");
			}
		}
	}

	mutex_unlock(&cc->mutex);

	len += sysfs_emit_at(buf, len, "\nWoWLAN Pattern Matching - Wake device on specific packet patterns\n\n");
	len += sysfs_emit_at(buf, len, "Mode:\n");
	len += sysfs_emit_at(buf, len, "  -s (default)  - Search mode (sliding window, pattern found anywhere from offset)\n");
	len += sysfs_emit_at(buf, len, "  -f            - Fixed mode (payload matches at exact offset)\n");
	len += sysfs_emit_at(buf, len, "  Note: Header (first 14 bytes) is always matched at fixed positions\n\n");
	len += sysfs_emit_at(buf, len, "Format:\n");
	len += sysfs_emit_at(buf, len, "  Header only:     [14-byte Ethernet header]\n");
	len += sysfs_emit_at(buf, len, "  Header+Payload: [-s|-f] [14-byte header]|[offset+][payload]\n\n");
	len += sysfs_emit_at(buf, len, "Pattern syntax:\n");
	len += sysfs_emit_at(buf, len, "  AA:BB:CC    - Match specific bytes (hex)\n");
	len += sysfs_emit_at(buf, len, "  -           - Wildcard (match any byte)\n");
	len += sysfs_emit_at(buf, len, "  |           - Separator between header and payload\n");
	len += sysfs_emit_at(buf, len, "  N+          - Start payload at byte N (search/fixed mode)\n\n");
	len += sysfs_emit_at(buf, len, "Ethernet header (14 bytes): dst_MAC(6):src_MAC(6):EtherType(2)\n\n");
	len += sysfs_emit_at(buf, len, "Examples:\n");
	len += sysfs_emit_at(buf, len, "  1. Match destination MAC:\n");
	len += sysfs_emit_at(buf, len, "     echo \"4d:41:47:49:43:55:-:-:-:-:-:-:-:-\" > wowlan_pattern_search\n\n");
	len += sysfs_emit_at(buf, len, "  2. Match source MAC:\n");
	len += sysfs_emit_at(buf, len, "     echo \"-:-:-:-:-:-:00:0a:cd:48:0b:7a:-:-\" > wowlan_pattern_search\n\n");
	len += sysfs_emit_at(buf, len, "  3. Match EtherType 0x0806 (ARP) with payload pattern (search mode):\n");
	len += sysfs_emit_at(buf, len, "     echo \"-s -:-:-:-:-:-:-:-:-:-:-:-:08:06|10+01:02:03\" > wowlan_pattern_search\n\n");
	len += sysfs_emit_at(buf, len, "  4. Match payload anywhere (search mode):\n");
	len += sysfs_emit_at(buf, len, "     echo \"-:-:-:-:-:-:-:-:-:-:-:-:-:-|31:32:-:34:35:36\" > wowlan_pattern_search\n\n");
	len += sysfs_emit_at(buf, len, "  5. Match EtherType 0x0806 (ARP) at fixed payload offset:\n");
	len += sysfs_emit_at(buf, len, "     echo \"-f -:-:-:-:-:-:-:-:-:-:-:-:08:06|10+01:02:03\" > wowlan_pattern_search\n\n");
	len += sysfs_emit_at(buf, len, "Limits: Max %d filters, Max %d byte pattern\n",
			     CC33XX_MAX_RX_FILTERS, CC33XX_RX_FILTER_MAX_PATTERN_SIZE);
	len += sysfs_emit_at(buf, len, "Clear:  echo 1 > wowlan_pattern_clear\n");

	return len;
}

static DEVICE_ATTR_RW(wowlan_pattern_search);

int cc33xx_sysfs_init(struct cc33xx *cc)
{
	int ret;

	ret = device_create_file(cc->dev, &dev_attr_ble_enable);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file ble_enable");

	ret = device_create_file(cc->dev, &dev_attr_slow_clock_type);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file slow_clock_type");

	ret = device_create_file(cc->dev, &dev_attr_regdomain_txControl_param);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file regdomain_txControl_param");

	ret = device_create_file(cc->dev, &dev_attr_wowlan_arp_offload);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file wowlan_arp_offload");

	ret = device_create_file(cc->dev, &dev_attr_wowlan_pattern_clear);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file wowlan_pattern_clear");

	ret = device_create_file(cc->dev, &dev_attr_wowlan_pattern_search);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file wowlan_pattern_search");

	ret = device_create_file(cc->dev, &dev_attr_wowlan_mode);
	if (ret < 0)
		cc33xx_error("failed to create sysfs file wowlan_mode");

	/* Create sysfs file for the FW log */
	ret = device_create_bin_file(cc->dev, &fwlog_attr);
	if (ret < 0) {
		cc33xx_error("failed to create sysfs file fwlog");
	}

	return ret;
}

void cc33xx_sysfs_free(struct cc33xx *cc)
{
	device_remove_file(cc->dev, &dev_attr_ble_enable);
}
