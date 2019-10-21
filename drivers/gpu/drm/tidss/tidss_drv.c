// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
 * Author: Tomi Valkeinen <tomi.valkeinen@ti.com>
 */

#include <linux/console.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "tidss_dispc.h"
#include "tidss_drv.h"
#include "tidss_irq.h"
#include "tidss_kms.h"

/* -----------------------------------------------------------------------------
 * Device Information
 */

DEFINE_DRM_GEM_CMA_FOPS(tidss_fops);

static struct drm_driver tidss_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_PRIME
				| DRIVER_ATOMIC | DRIVER_HAVE_IRQ,
	.gem_free_object_unlocked = drm_gem_cma_free_object,
	.gem_vm_ops		= &drm_gem_cma_vm_ops,
	.prime_handle_to_fd	= drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle	= drm_gem_prime_fd_to_handle,
	.gem_prime_import	= drm_gem_prime_import,
	.gem_prime_export	= drm_gem_prime_export,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_vmap		= drm_gem_cma_prime_vmap,
	.gem_prime_vunmap	= drm_gem_cma_prime_vunmap,
	.gem_prime_mmap		= drm_gem_cma_prime_mmap,
	.dumb_create		= drm_gem_cma_dumb_create,
	.fops			= &tidss_fops,
	.name			= "tidss",
	.desc			= "TI Keystone DSS",
	.date			= "20180215",
	.major			= 1,
	.minor			= 0,

	.irq_preinstall		= tidss_irq_preinstall,
	.irq_postinstall	= tidss_irq_postinstall,
	.irq_handler		= tidss_irq_handler,
	.irq_uninstall		= tidss_irq_uninstall,
};

#ifdef CONFIG_PM
/* -----------------------------------------------------------------------------
 * Power management
 */

static int tidss_pm_runtime_suspend(struct device *dev)
{
	struct tidss_device *tidss = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	return tidss->dispc_ops->runtime_suspend(tidss->dispc);
}

static int tidss_pm_runtime_resume(struct device *dev)
{
	struct tidss_device *tidss = dev_get_drvdata(dev);
	int r;

	dev_dbg(dev, "%s\n", __func__);

	r = tidss->dispc_ops->runtime_resume(tidss->dispc);
	if (r)
		return r;

	tidss_irq_resume(tidss->ddev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tidss_suspend(struct device *dev)
{
	struct tidss_device *tidss = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	return drm_mode_config_helper_suspend(tidss->ddev);
}

static int tidss_resume(struct device *dev)
{
	struct tidss_device *tidss = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	return drm_mode_config_helper_resume(tidss->ddev);
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops tidss_pm_ops = {
	.runtime_suspend = tidss_pm_runtime_suspend,
	.runtime_resume = tidss_pm_runtime_resume,
	SET_SYSTEM_SLEEP_PM_OPS(tidss_suspend, tidss_resume)
};

#endif /* CONFIG_PM */

/* -----------------------------------------------------------------------------
 * Platform driver
 */

static int tidss_init_remote_device(struct tidss_device *tidss)
{
	int ret;
	struct device *dev = tidss->dev;
	struct device_node *dss_remote_dev_node;
	const char *name;

	dss_remote_dev_node = of_get_child_by_name(dev->of_node, "dss-remote");
	if (!dss_remote_dev_node)
		return 0;

	if (!of_find_property(dss_remote_dev_node, "remote-name", NULL)) {
		ret = 0;
		goto out;
	}

	ret = of_property_read_string(dss_remote_dev_node, "remote-name", &name);
	if (ret) {
		dev_err(dev, "%s: could not read remote-name property\n", __func__);
		goto out;
	}

	tidss->rdev = rpmsg_remotedev_get_named_device(name);
	if (!tidss->rdev) {
		ret = -EPROBE_DEFER;
		goto out;
	} else if (IS_ERR(tidss->rdev)) {
		ret = PTR_ERR(tidss->rdev);
		goto out;
	}

	tidss->rdev->cb_data = tidss;

	if (tidss->rdev->device.display.ops->ready == NULL ||
			tidss->rdev->device.display.ops->get_res_info == NULL ||
			tidss->rdev->device.display.ops->commit == NULL) {
		dev_err(dev, "%s: rpmsg remotedev ops not complete\n", __func__);
		ret = -EINVAL;
		goto disconnect;
	}

	/* Cant really do much if the remotedev is not ready yet */
	if (!tidss->rdev->device.display.ops->ready(tidss->rdev)) {
		ret = -EPROBE_DEFER;
		goto disconnect;
	}

	goto out;

disconnect:
	rpmsg_remotedev_put_device(tidss->rdev);
	tidss->rdev->cb_data = NULL;
	tidss->rdev = NULL;
out:
	of_node_put(dss_remote_dev_node);
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

	tidss = devm_kzalloc(dev, sizeof(*tidss), GFP_KERNEL);
	if (tidss == NULL)
		return -ENOMEM;

	tidss->dev = dev;
	tidss->features = of_device_get_match_data(dev);

	ret = tidss_init_remote_device(tidss);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, tidss);

	ddev = drm_dev_alloc(&tidss_driver, dev);
	if (IS_ERR(ddev)) {
		ret = PTR_ERR(ddev);
		goto err_fini_rdev;
	}

	tidss->ddev = ddev;
	ddev->dev_private = tidss;

	pm_runtime_enable(dev);

	ret = tidss->features->dispc_init(tidss);
	if (ret) {
		dev_err(dev, "failed to initialize dispc: %d\n", ret);
		goto err_disable_pm;
	}

#ifndef CONFIG_PM
	/* If we don't have PM, we need to call resume manually */
	tidss->dispc_ops->runtime_resume(tidss->dispc);
#endif

	ret = tidss_modeset_init(tidss);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to init DRM/KMS (%d)\n", ret);
		goto err_runtime_suspend;
	}

	irq = tidss->dispc_ops->get_irq(tidss->dispc);
	if (irq < 0) {
		ret = irq;
		dev_err(dev, "failed to get dispc irq: %d\n", ret);
		goto err_modeset_cleanup;
	}

	ret = drm_irq_install(ddev, irq);
	if (ret) {
		dev_err(dev, "drm_irq_install failed: %d\n", ret);
		goto err_modeset_cleanup;
	}

	drm_kms_helper_poll_init(ddev);

	if (tidss->dispc_ops->has_writeback(tidss->dispc)) {
		ret = tidss_wb_init(ddev);
		if (ret)
			dev_warn(dev, "failed to initialize writeback\n");
		else
			tidss->wb_initialized = true;
	}

	ret = drm_dev_register(ddev, 0);
	if (ret) {
		dev_err(dev, "failed to register DRM device\n");
		goto err_poll_fini;
	}

	drm_fbdev_generic_setup(ddev, 32);

	dev_dbg(dev, "%s done\n", __func__);

	return 0;

err_poll_fini:
	if (tidss->wb_initialized)
		tidss_wb_cleanup(ddev);

	drm_kms_helper_poll_fini(ddev);

	drm_atomic_helper_shutdown(ddev);

	drm_irq_uninstall(ddev);

err_modeset_cleanup:
	tidss_modeset_cleanup(tidss);

err_runtime_suspend:
#ifndef CONFIG_PM
	/* If we don't have PM, we need to call suspend manually */
	tidss->dispc_ops->runtime_suspend(tidss->dispc);
#endif

	tidss->dispc_ops->remove(tidss->dispc);

err_disable_pm:
	pm_runtime_disable(dev);

err_fini_rdev:
	if (tidss->rdev)
		rpmsg_remotedev_put_device(tidss->rdev);

	drm_dev_put(ddev);

	return ret;
}

static int tidss_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tidss_device *tidss = platform_get_drvdata(pdev);
	struct drm_device *ddev = tidss->ddev;

	dev_dbg(dev, "%s\n", __func__);

	drm_dev_unregister(ddev);

	if (tidss->wb_initialized)
		tidss_wb_cleanup(ddev);

	drm_kms_helper_poll_fini(ddev);

	drm_atomic_helper_shutdown(ddev);

	drm_irq_uninstall(ddev);

	tidss_modeset_cleanup(tidss);

#ifndef CONFIG_PM
	/* If we don't have PM, we need to call suspend manually */
	tidss->dispc_ops->runtime_suspend(tidss->dispc);
#endif

	tidss->dispc_ops->remove(tidss->dispc);

	pm_runtime_disable(dev);

	drm_dev_put(ddev);

	dev_dbg(dev, "%s done\n", __func__);

	return 0;
}

#ifdef CONFIG_DRM_TIDSS_DSS6
static const struct tidss_features tidss_k2g_features = {
	.dispc_init = dispc6_init,
};
#endif

#ifdef CONFIG_DRM_TIDSS_DSS7
static const struct tidss_features tidss_am6_features = {
	.dispc_init = dispc7_init,
};

static const struct tidss_features tidss_j721e_features = {
	.dispc_init = dispc7_init,
};
#endif
static const struct of_device_id tidss_of_table[] = {
#ifdef CONFIG_DRM_TIDSS_DSS6
	{ .compatible = "ti,k2g-dss", .data = &tidss_k2g_features },
#endif
#ifdef CONFIG_DRM_TIDSS_DSS7
	{ .compatible = "ti,am6-dss", .data = &tidss_am6_features },
	{ .compatible = "ti,j721e-dss", .data = &tidss_j721e_features },
#endif
	{ }
};

MODULE_DEVICE_TABLE(of, tidss_of_table);

static struct platform_driver tidss_platform_driver = {
	.probe		= tidss_probe,
	.remove		= tidss_remove,
	.driver		= {
		.name	= "tidss",
#ifdef CONFIG_PM
		.pm	= &tidss_pm_ops,
#endif
		.of_match_table = tidss_of_table,
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(tidss_platform_driver);

MODULE_AUTHOR("Tomi Valkeinen <tomi.valkeinen@ti.com>");
MODULE_DESCRIPTION("TI Keystone DSS Driver");
MODULE_LICENSE("GPL v2");
