// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - https://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <linux/console.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/pm_domain.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_module.h>
#include <drm/drm_probe_helper.h>

#include "tidss_dispc.h"
#include "tidss_drv.h"
#include "tidss_kms.h"
#include "tidss_irq.h"
#include "tidss_oldi.h"

/* Power management */

int tidss_runtime_get(struct tidss_device *tidss)
{
	int r;

	dev_dbg(tidss->dev, "%s\n", __func__);

	r = pm_runtime_resume_and_get(tidss->dev);
	WARN_ON(r < 0);
	return r;
}

void tidss_runtime_put(struct tidss_device *tidss)
{
	int r;

	dev_dbg(tidss->dev, "%s\n", __func__);

	pm_runtime_mark_last_busy(tidss->dev);

	r = pm_runtime_put_autosuspend(tidss->dev);
	WARN_ON(r < 0);
}

static int __maybe_unused tidss_pm_runtime_suspend(struct device *dev)
{
	struct tidss_device *tidss = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	return dispc_runtime_suspend(tidss->dispc);
}

static int __maybe_unused tidss_pm_runtime_resume(struct device *dev)
{
	struct tidss_device *tidss = dev_get_drvdata(dev);
	int r;

	dev_dbg(dev, "%s\n", __func__);

	r = dispc_runtime_resume(tidss->dispc);
	if (r)
		return r;

	return 0;
}

static int __maybe_unused tidss_suspend(struct device *dev)
{
	struct tidss_device *tidss = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	return drm_mode_config_helper_suspend(&tidss->ddev);
}

static int __maybe_unused tidss_resume(struct device *dev)
{
	struct tidss_device *tidss = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	return drm_mode_config_helper_resume(&tidss->ddev);
}

static __maybe_unused const struct dev_pm_ops tidss_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tidss_suspend, tidss_resume)
	SET_RUNTIME_PM_OPS(tidss_pm_runtime_suspend, tidss_pm_runtime_resume, NULL)
};

/* DRM device Information */

static void tidss_release(struct drm_device *ddev)
{
	drm_kms_helper_poll_fini(ddev);
}

DEFINE_DRM_GEM_DMA_FOPS(tidss_fops);

static const struct drm_driver tidss_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &tidss_fops,
	.release		= tidss_release,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.name			= "tidss",
	.desc			= "TI Keystone DSS",
	.date			= "20180215",
	.major			= 1,
	.minor			= 0,
};

static int tidss_detach_pm_domains(struct tidss_device *tidss)
{
	int i;

	if (tidss->num_domains <= 1)
		return 0;

	for (i = 0; i < tidss->num_domains; i++) {
		if (tidss->pd_link[i] && !IS_ERR(tidss->pd_link[i]))
			device_link_del(tidss->pd_link[i]);
		if (tidss->pd_dev[i] && !IS_ERR(tidss->pd_dev[i]))
			dev_pm_domain_detach(tidss->pd_dev[i], true);
		tidss->pd_dev[i] = NULL;
		tidss->pd_link[i] = NULL;
	}

	return 0;
}

static int tidss_attach_pm_domains(struct tidss_device *tidss)
{
	struct device *dev = tidss->dev;
	int i;
	int ret;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *np = pdev->dev.of_node;

	tidss->num_domains = of_count_phandle_with_args(np, "power-domains",
							"#power-domain-cells");
	if (tidss->num_domains <= 1) {
		dev_dbg(dev, "One or less power domains, no need to do attach domains\n");
		return 0;
	}

	tidss->pd_dev = devm_kmalloc_array(dev, tidss->num_domains,
					   sizeof(*tidss->pd_dev), GFP_KERNEL);
	if (!tidss->pd_dev)
		return -ENOMEM;

	tidss->pd_link = devm_kmalloc_array(dev, tidss->num_domains,
					    sizeof(*tidss->pd_link), GFP_KERNEL);
	if (!tidss->pd_link)
		return -ENOMEM;

	for (i = 0; i < tidss->num_domains; i++) {
		tidss->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(tidss->pd_dev[i])) {
			ret = PTR_ERR(tidss->pd_dev[i]);
			goto fail;
		}

		tidss->pd_link[i] = device_link_add(dev, tidss->pd_dev[i],
						    DL_FLAG_STATELESS |
						    DL_FLAG_PM_RUNTIME | DL_FLAG_RPM_ACTIVE);
		if (!tidss->pd_link[i]) {
			ret = -EINVAL;
			goto fail;
		}
	}

	return 0;
fail:
	tidss_detach_pm_domains(tidss);
	return ret;
}

static int tidss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tidss_device *tidss;
	struct drm_device *ddev;
	int ret;
	int irq;

	dev_dbg(dev, "%s\n", __func__);

	tidss = devm_drm_dev_alloc(&pdev->dev, &tidss_driver,
				   struct tidss_device, ddev);
	if (IS_ERR(tidss))
		return PTR_ERR(tidss);

	ddev = &tidss->ddev;

	tidss->dev = dev;
	tidss->feat = of_device_get_match_data(dev);

	platform_set_drvdata(pdev, tidss);

	spin_lock_init(&tidss->wait_lock);

	ret = tidss_oldi_init(tidss);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init OLDI\n");

	/* powering up associated OLDI domains */
	ret = tidss_attach_pm_domains(tidss);
	if (ret < 0) {
		dev_err(dev, "failed to attach power domains %d\n", ret);
		goto err_oldi_deinit;
	}

	ret = dispc_init(tidss);
	if (ret) {
		dev_err(dev, "failed to initialize dispc: %d\n", ret);
		goto err_oldi_deinit;
	}

	pm_runtime_enable(dev);

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

#ifndef CONFIG_PM
	/* If we don't have PM, we need to call resume manually */
	dispc_runtime_resume(tidss->dispc);
#endif

	ret = tidss_modeset_init(tidss);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to init DRM/KMS (%d)\n", ret);
		goto err_runtime_suspend;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto err_runtime_suspend;
	}
	tidss->irq = irq;

	ret = tidss_irq_install(ddev, irq);
	if (ret) {
		dev_err(dev, "tidss_irq_install failed: %d\n", ret);
		goto err_runtime_suspend;
	}

	drm_kms_helper_poll_init(ddev);

	drm_mode_config_reset(ddev);

	ret = drm_dev_register(ddev, 0);
	if (ret) {
		dev_err(dev, "failed to register DRM device\n");
		goto err_irq_uninstall;
	}

	drm_fbdev_dma_setup(ddev, 32);

	dev_dbg(dev, "%s done\n", __func__);

	return 0;

err_irq_uninstall:
	tidss_irq_uninstall(ddev);

err_runtime_suspend:
#ifndef CONFIG_PM
	dispc_runtime_suspend(tidss->dispc);
#endif
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);

err_oldi_deinit:
	tidss_detach_pm_domains(tidss);
	tidss_oldi_deinit(tidss);

	return ret;
}

static void tidss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tidss_device *tidss = platform_get_drvdata(pdev);
	struct drm_device *ddev = &tidss->ddev;

	dev_dbg(dev, "%s\n", __func__);

	drm_dev_unregister(ddev);

	drm_atomic_helper_shutdown(ddev);

	tidss_irq_uninstall(ddev);

#ifndef CONFIG_PM
	/* If we don't have PM, we need to call suspend manually */
	dispc_runtime_suspend(tidss->dispc);
#endif
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);

	/* devm allocated dispc goes away with the dev so mark it NULL */
	dispc_remove(tidss);

	tidss_detach_pm_domains(tidss);
	dev_dbg(dev, "%s done\n", __func__);
}

static void tidss_shutdown(struct platform_device *pdev)
{
	drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static const struct of_device_id tidss_of_table[] = {
	{ .compatible = "ti,k2g-dss", .data = &dispc_k2g_feats, },
	{ .compatible = "ti,am625-dss", .data = &dispc_am625_feats, },
	{ .compatible = "ti,am62a7-dss", .data = &dispc_am62a7_feats, },
	{ .compatible = "ti,am65x-dss", .data = &dispc_am65x_feats, },
	{ .compatible = "ti,j721e-dss", .data = &dispc_j721e_feats, },
	{ }
};

MODULE_DEVICE_TABLE(of, tidss_of_table);

static struct platform_driver tidss_platform_driver = {
	.probe		= tidss_probe,
	.remove_new	= tidss_remove,
	.shutdown	= tidss_shutdown,
	.driver		= {
		.name	= "tidss",
		.pm	= pm_ptr(&tidss_pm_ops),
		.of_match_table = tidss_of_table,
		.suppress_bind_attrs = true,
	},
};

drm_module_platform_driver(tidss_platform_driver);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("TI Keystone DSS Driver");
MODULE_LICENSE("GPL v2");
