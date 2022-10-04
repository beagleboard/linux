/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2021 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************
*
*    The GPL License (GPL)
*
*    Copyright (C) 2014 - 2021 Vivante Corporation
*
*    This program is free software; you can redistribute it and/or
*    modify it under the terms of the GNU General Public License
*    as published by the Free Software Foundation; either version 2
*    of the License, or (at your option) any later version.
*
*    This program is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with this program; if not, write to the Free Software Foundation,
*    Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
*
*****************************************************************************
*
*    Note: This software is released under dual MIT and GPL licenses. A
*    recipient may use this file under the terms of either the MIT license or
*    GPL License. If you wish to use only one license not the other, you can
*    indicate your decision by deleting one of the above license notices in your
*    version of this file.
*
*****************************************************************************/


#include "gc_hal_kernel_linux.h"
#include "gc_hal_kernel_platform.h"
#include "gc_hal_kernel_platform_default.h"

/* Disable MSI for internal FPGA build except PPC */
#if gcdFPGA_BUILD
#define USE_MSI     0
#else
#define USE_MSI     1
#endif

gceSTATUS
_AdjustParam(
    IN gcsPLATFORM *Platform,
    OUT gcsMODULE_PARAMETERS *Args
    );

gceSTATUS
_GetGPUPhysical(
    IN gcsPLATFORM * Platform,
    IN gctPHYS_ADDR_T CPUPhysical,
    OUT gctPHYS_ADDR_T *GPUPhysical
    );

#if gcdENABLE_MP_SWITCH
gceSTATUS
_SwitchCoreCount(
    IN gcsPLATFORM *Platform,
    OUT gctUINT32 *Count
    );
#endif

static struct _gcsPLATFORM_OPERATIONS default_ops =
{
    .adjustParam   = _AdjustParam,
    .getGPUPhysical = _GetGPUPhysical,
#if gcdENABLE_MP_SWITCH
    .switchCoreCount = _SwitchCoreCount,
#endif
};


#if gcdSUPPORT_DEVICE_TREE_SOURCE
static int gpu_parse_dt(struct platform_device *pdev, gcsMODULE_PARAMETERS *params);
static void gpu_add_power_domains(void);

static struct _gcsPLATFORM default_platform =
{
    .name = __FILE__,
    .ops  = &default_ops,
};

static gcsPOWER_DOMAIN domains[] =
{
    [gcvCORE_MAJOR] =
    {
        .base =
        {
            .name = "pd-major",
        },
    },
    [gcvCORE_3D1] =
    {
        .base =
        {
            .name = "pd-3d1",
        },
    },
    [gcvCORE_3D2] =
    {
        .base =
        {
            .name = "pd-3d2",
        },
    },
    [gcvCORE_3D3] =
    {
        .base =
        {
            .name = "pd-3d3",
        },
    },
    [gcvCORE_3D4] =
    {
        .base =
        {
            .name = "pd-3d4",
        },
    },
    [gcvCORE_3D5] =
    {
        .base =
        {
            .name = "pd-3d5",
        },
    },
    [gcvCORE_3D6] =
    {
        .base =
        {
            .name = "pd-3d6",
        },
    },
    [gcvCORE_3D7] =
    {
        .base =
        {
            .name = "pd-3d7",
        },
    },
    [gcvCORE_2D] =
    {
        .base =
        {
            .name = "pd-2d",
        },
    },
    [gcvCORE_VG] =
    {
        .base =
        {
            .name = "pd-vg",
        },
    },
#if gcdDEC_ENABLE_AHB
    [gcvCORE_DEC] =
    {
        .base =
        {
            .name = "pd-dec",
        },
    },
#endif
    [gcvCORE_2D1] =
    {
        .base =
        {
            .name = "pd-2d1",
        },
    },
};

static inline gcsPOWER_DOMAIN *to_gc_power_domain(struct generic_pm_domain *gpd)
{
    return gcmCONTAINEROF(gpd, gcsPOWER_DOMAIN, base);
}

static int gc_power_domain_power_on(    struct generic_pm_domain *gpd)
{
    return 0;
}

static int gc_power_domain_power_off(    struct generic_pm_domain *gpd)
{
    return 0;
}

static int gc_power_domain_probe(struct platform_device *pdev)
{
    gceSTATUS status;
    int ret = 0;
    struct device_node *np = pdev->dev.of_node;
    int core_id;

    ret = of_property_read_u32(np, "core-id", &core_id);
    if (ret)
    {
        gcmONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }
    if (core_id >= gcvCORE_COUNT)
    {
        gcmONERROR(gcvSTATUS_INVALID_ARGUMENT);
    }

    ret = platform_device_add_data(pdev, (gctPOINTER)&domains[core_id], sizeof(gcsPOWER_DOMAIN));
    if (ret)
    {
        gcmONERROR(gcvSTATUS_NOT_SUPPORTED);
    }

    return 0;
OnError:
    return ret;
}

static int gc_power_domain_remove(struct platform_device *pdev)
{
    gcsPOWER_DOMAIN *domain = pdev->dev.platform_data;

    if (IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS))
    {
        of_genpd_del_provider(domain->pdev->dev.of_node);
    }

    return 0;
}

static const struct of_device_id gc_power_domain_dt_ids[] =
{
    {.compatible = "verisilicon,pd-vip",},
    {.compatible = "verisilicon,pd-gpu2d",},
    {.compatible = "verisilicon,pd-gpu3d",},

    {/* sentinel */}
};

static struct platform_driver gc_power_domain_driver =
{
    .driver = {
        .owner = THIS_MODULE,
        .name = "gc-pm-domains",
        .of_match_table = gc_power_domain_dt_ids,
    },
    .probe = gc_power_domain_probe,
    .remove = gc_power_domain_remove,
};

static int gpu_power_domain_init(void)
{
    return platform_driver_register(&gc_power_domain_driver);
}


static void gpu_power_domain_exit(void)
{
    platform_driver_unregister(&gc_power_domain_driver);
}

static void gpu_add_power_domains(void)
{
    struct device_node *np = gcvNULL;
    int ret;
    gceSTATUS status;

    for_each_matching_node(np, gc_power_domain_dt_ids)
    {
        struct platform_device *pdev;
        gcsPOWER_DOMAIN *domain = domains;
        int core_id;

        if (!of_device_is_available(np))
        {
            continue;
        }

        pdev = of_find_device_by_node(np);

        if (!pdev)
            break;

        ret = of_property_read_u32(np, "core-id", &core_id);
        if (ret)
        {
            gcmONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }
        if (core_id >= gcvCORE_COUNT)
        {
            gcmONERROR(gcvSTATUS_INVALID_ARGUMENT);
        }

        domain[core_id].pdev = pdev;
        domain[core_id].core_id = core_id;

        domain[core_id].base.power_on = gc_power_domain_power_on;
        domain[core_id].base.power_off = gc_power_domain_power_off;

        if (IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS))
        {
            ret = pm_genpd_init(&domain[core_id].base, gcvNULL, true);
            if (ret)
            {
                continue;
            }

            ret = of_genpd_add_provider_simple(np, &domain[core_id].base);
            if (ret)
            {
                continue;
            }
        }
    }
OnError:
    return;
}

static int gpu_parse_dt(struct platform_device *pdev, gcsMODULE_PARAMETERS *params)
{
    struct device_node *root = pdev->dev.of_node;
    struct resource* res;
    gctUINT32 i;
    const gctUINT32 *value;
    const char *core_names[] =
    {
        "core_major",
        "core_3d1",
        "core_3d2",
        "core_3d3",
        "core_3d4",
        "core_3d5",
        "core_3d6",
        "core_3d7",
        "core_2d",
        "core_vg",
#if gcdDEC_ENABLE_AHB
        "core_dec",
#endif
        "core_2d1",
    };

    gcmSTATIC_ASSERT(gcvCORE_COUNT == gcmCOUNTOF(core_names),
                     "core_names array does not match core types");

    /* parse the irqs config */
    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, core_names[i]);
        if (res)
        {
            params->irqs[i] = res->start;
        }
    }

    /* parse the registers config */
    for (i = 0; i < gcvCORE_COUNT; i++)
    {
        res = platform_get_resource_byname(pdev, IORESOURCE_MEM, core_names[i]);
        if (res)
        {
            params->registerBases[i] = res->start;
            params->registerSizes[i] = res->end - res->start + 1;
        }
    }

    /* parse the contiguous mem */
    value = of_get_property(root, "contiguous-size", gcvNULL);
    if (value && *value != 0)
    {
        gctUINT64 addr;

        of_property_read_u64(root, "contiguous-base", &addr);
        params->contiguousSize = *value;
        params->contiguousBase = addr;
    }

    value = of_get_property(root, "contiguous-requested", gcvNULL);
    if (value)
    {
        params->contiguousRequested = *value ? gcvTRUE : gcvFALSE;
    }

    /* parse the external mem */
    value = of_get_property(root, "external-size", gcvNULL);
    if (value && *value != 0)
    {
        gctUINT64 addr;

        of_property_read_u64(root, "external-base", &addr);
        params->externalSize = *value;
        params->externalBase = addr;
    }

    value = of_get_property(root, "phys-size", gcvNULL);
    if (value && *value != 0)
    {
        gctUINT64 addr;

        of_property_read_u64(root, "base-address", &addr);
        params->physSize = *value;
        params->baseAddress = addr;
    }

    value = of_get_property(root, "phys-size", gcvNULL);
    if (value)
    {
        params->bankSize = *value;
    }

    value = of_get_property(root, "recovery", gcvNULL);
    if (value)
    {
        params->recovery = *value;
    }

    value = of_get_property(root, "power-management", gcvNULL);
    if (value)
    {
        params->powerManagement = *value;
    }

    value = of_get_property(root, "enable-mmu", gcvNULL);
    if (value)
    {
        params->enableMmu = *value;
    }

    value = of_get_property(root, "user-cluster-mask", gcvNULL);
    if (value)
    {
        params->userClusterMask = *value;
    }

    value = of_get_property(root, "stuck-dump", gcvNULL);
    if (value)
    {
        params->stuckDump = *value;
    }

    value = of_get_property(root, "gpu-profiler", gcvNULL);
    if (value)
    {
        params->gpuProfiler = *value;
    }

    value = of_get_property(root, "show-args", gcvNULL);
    if (value)
    {
        params->showArgs = *value;
    }

    value = of_get_property(root, "mmu-page-table-pool", gcvNULL);
    if (value)
    {
        params->mmuPageTablePool = *value;
    }

    value = of_get_property(root, "mmu-dynamic-map", gcvNULL);
    if (value)
    {
        params->mmuDynamicMap = *value;
    }

    value = of_get_property(root, "all-map-in-one", gcvNULL);
    if (value)
    {
        params->allMapInOne = *value;
    }

    value = of_get_property(root, "isr-poll-mask", gcvNULL);
    if (value)
    {
        params->isrPoll = *value;
    }

    return 0;
}

static const struct of_device_id gpu_dt_ids[] = {
    { .compatible = "verisilicon,galcore",},

    {/* sentinel */}
};

#elif USE_LINUX_PCIE

#define MAX_PCIE_DEVICE 4
#define MAX_PCIE_BAR    6

typedef struct _gcsBARINFO
{
    gctPHYS_ADDR_T base;
    gctSIZE_T size;
    gctPOINTER logical;
}
gcsBARINFO, *gckBARINFO;

struct _gcsPCIEInfo
{
    gcsBARINFO bar[MAX_PCIE_BAR];
    struct pci_dev *pdev;
    gctPHYS_ADDR_T sram_bases[gcvSRAM_EXT_COUNT];
    gctPHYS_ADDR_T sram_gpu_bases[gcvSRAM_EXT_COUNT];
    uint32_t sram_sizes[gcvSRAM_EXT_COUNT];
    int sram_bars[gcvSRAM_EXT_COUNT];
    int sram_offsets[gcvSRAM_EXT_COUNT];
};

struct _gcsPLATFORM_PCIE
{
    struct _gcsPLATFORM base;
    struct _gcsPCIEInfo pcie_info[MAX_PCIE_DEVICE];
    unsigned int device_number;
};


struct _gcsPLATFORM_PCIE default_platform =
{
    .base =
    {
        .name = __FILE__,
        .ops  = &default_ops,
    },
};

void
_QueryBarInfo(
    struct pci_dev *Pdev,
    gctPHYS_ADDR_T *BarAddr,
    gctSIZE_T *BarSize,
    gctUINT BarNum
    )
{
    gctUINT addr;
    gctUINT size;

    /* Read the bar address */
    if (pci_read_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, &addr) < 0)
    {
        return;
    }

    /* Read the bar size */
    if (pci_write_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, 0xffffffff) < 0)
    {
        return;
    }

    if (pci_read_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, &size) < 0)
    {
        return;
    }

    size &= 0xfffffff0;
    size  = ~size;
    size += 1;

    /* Write back the bar address */
    if (pci_write_config_dword(Pdev, PCI_BASE_ADDRESS_0 + BarNum * 0x4, addr) < 0)
    {
        return;
    }

    gcmkPRINT("Bar%d addr=0x%x size=0x%x", BarNum, addr, size);

    *BarAddr = addr;
    *BarSize = size;
}

static const struct pci_device_id vivpci_ids[] = {
  {
    .class = 0x000000,
    .class_mask = 0x000000,
    .vendor = 0x10ee,
    .device = 0x7012,
    .subvendor = PCI_ANY_ID,
    .subdevice = PCI_ANY_ID,
    .driver_data = 0
  }, { /* End: all zeroes */ }
};

MODULE_DEVICE_TABLE(pci, vivpci_ids);


static int gpu_sub_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 24)
    static u64 dma_mask = DMA_BIT_MASK(40);
#else
    static u64 dma_mask = DMA_40BIT_MASK;
#endif

    gcmkPRINT("PCIE DRIVER PROBED");
    if (pci_enable_device(pdev)) {
        printk(KERN_ERR "galcore: pci_enable_device() failed.\n");
    }

    if (pci_set_dma_mask(pdev, dma_mask)) {
        printk(KERN_ERR "galcore: Failed to set DMA mask.\n");
    }

    pci_set_master(pdev);

    if (pci_request_regions(pdev, "galcore")) {
        printk(KERN_ERR "galcore: Failed to get ownership of BAR region.\n");
    }

#if USE_MSI
    if (pci_enable_msi(pdev)) {
        printk(KERN_ERR "galcore: Failed to enable MSI.\n");
    }
#endif

#if defined(CONFIG_PPC)
    /* On PPC platform, enable bus master, enable irq. */
    if (pci_write_config_word(pdev, 0x4, 0x0006) < 0) {
        printk(KERN_ERR "galcore: Failed to enable bus master on PPC.\n");
    }
#endif

    default_platform.pcie_info[default_platform.device_number++].pdev = pdev;
    return 0;
}

static void gpu_sub_remove(struct pci_dev *pdev)
{
    pci_set_drvdata(pdev, NULL);
#if USE_MSI
    pci_disable_msi(pdev);
#endif
    pci_clear_master(pdev);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    return;
}

static struct pci_driver gpu_pci_subdriver = {
    .name = DEVICE_NAME,
    .id_table = vivpci_ids,
    .probe = gpu_sub_probe,
    .remove = gpu_sub_remove
};
#else
static struct _gcsPLATFORM default_platform =
{
    .name = __FILE__,
    .ops  = &default_ops,
};
#endif

gceSTATUS
_AdjustParam(
    IN gcsPLATFORM *Platform,
    OUT gcsMODULE_PARAMETERS *Args
    )
{
#if gcdSUPPORT_DEVICE_TREE_SOURCE
    gpu_parse_dt(Platform->device, Args);
    gpu_add_power_domains();
#elif USE_LINUX_PCIE
    struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)Platform;
    struct pci_dev *pdev = pcie_platform->pcie_info[0].pdev;
    int irqline = pdev->irq;
    unsigned int i;
    gctPOINTER ptr = gcvNULL;

    unsigned int dev_index, bar_index = 0;
    int sram_bar, sram_offset;

    if (Args->irqs[gcvCORE_2D] != -1)
    {
        Args->irqs[gcvCORE_2D] = irqline;
    }
    if (Args->irqs[gcvCORE_MAJOR] != -1)
    {
        Args->irqs[gcvCORE_MAJOR] = irqline;
    }

    for (dev_index = 0; dev_index < pcie_platform->device_number; dev_index++)
    {
        struct pci_dev * pcieDev = pcie_platform->pcie_info[dev_index].pdev;

        for (i = 0; i < MAX_PCIE_BAR; i++)
        {
            _QueryBarInfo(
                pcieDev,
                &pcie_platform->pcie_info[dev_index].bar[i].base,
                &pcie_platform->pcie_info[dev_index].bar[i].size,
                i
                );
        }

        for (i = 0; i < gcvCORE_COUNT; i++)
        {
            if (Args->bars[i] != -1)
            {
                bar_index = Args->bars[i];
                Args->irqs[i] = pcieDev->irq;

                if (Args->regOffsets[i])
                {
                    gcmkASSERT(Args->regOffsets[i] + Args->registerSizes[i]
                               < pcie_platform->pcie_info[dev_index].bar[bar_index].size);
                }

                ptr =  pcie_platform->pcie_info[dev_index].bar[bar_index].logical;
                if (!ptr)
                {
                    ptr = pcie_platform->pcie_info[dev_index].bar[bar_index].logical = (gctPOINTER)pci_iomap(pcieDev, bar_index, Args->registerSizes[i]);
                }

                if (ptr)
                {
                    Args->registerBasesMapped[i] = (gctPOINTER)((gctCHAR*)ptr + Args->regOffsets[i]);
                }
            }
        }

        for (i = 0; i < gcvSRAM_EXT_COUNT; i++)
        {
            pcie_platform->pcie_info[dev_index].sram_bases[i] =
            pcie_platform->pcie_info[dev_index].sram_gpu_bases[i] = Args->extSRAMBases[i];

            pcie_platform->pcie_info[dev_index].sram_sizes[i] = Args->extSRAMSizes[i];

            pcie_platform->pcie_info[dev_index].sram_bars[i] = sram_bar = Args->sRAMBars[i];
            pcie_platform->pcie_info[dev_index].sram_offsets[i] = sram_offset = Args->sRAMOffsets[i];

            /* Get CPU view SRAM base address from bar address and bar inside offset. */
            if (sram_bar != -1 && sram_offset != -1)
            {
                pcie_platform->pcie_info[dev_index].sram_bases[i] = Args->extSRAMBases[i]
                                                                  = pcie_platform->pcie_info[dev_index].bar[sram_bar].base
                                                                  + sram_offset;
            }
        }
    }

    Args->contiguousRequested = gcvTRUE;
#endif
    return gcvSTATUS_OK;
}

gceSTATUS
_GetGPUPhysical(
    IN gcsPLATFORM * Platform,
    IN gctPHYS_ADDR_T CPUPhysical,
    OUT gctPHYS_ADDR_T *GPUPhysical
    )
{
#if USE_LINUX_PCIE
    struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)Platform;
    /* Only support 1 external shared SRAM currently. */
    gctPHYS_ADDR_T sram_base = pcie_platform->pcie_info[0].sram_bases[0];
    gctPHYS_ADDR_T sram_gpu_base = pcie_platform->pcie_info[0].sram_gpu_bases[0];
    uint32_t sram_size = pcie_platform->pcie_info[0].sram_sizes[0];

    if (!sram_size && Platform->dev && Platform->dev->extSRAMSizes[0])
    {
        sram_size = Platform->dev->extSRAMSizes[0];
    }

    if (sram_base != gcvINVALID_PHYSICAL_ADDRESS && sram_gpu_base != gcvINVALID_PHYSICAL_ADDRESS && sram_size)
    {
        if ((CPUPhysical >= sram_base) && (CPUPhysical < (sram_base + sram_size)))
        {
            *GPUPhysical = CPUPhysical - sram_base + sram_gpu_base;
        }
        else
        {
            *GPUPhysical = CPUPhysical;
        }
    }
    else
#endif
    {
        *GPUPhysical = CPUPhysical;
    }

    return gcvSTATUS_OK;
}

#if gcdENABLE_MP_SWITCH
gceSTATUS
_SwitchCoreCount(
    IN gcsPLATFORM *Platform,
    OUT gctUINT32 *Count
    )
{
    *Count = Platform->coreCount;

    return gcvSTATUS_OK;
}
#endif

int gckPLATFORM_Init(struct platform_driver *pdrv,
            struct _gcsPLATFORM **platform)
{
    int ret = 0;
#if !gcdSUPPORT_DEVICE_TREE_SOURCE
    struct platform_device *default_dev = platform_device_alloc(pdrv->driver.name, -1);

    if (!default_dev) {
        printk(KERN_ERR "galcore: platform_device_alloc failed.\n");
        return -ENOMEM;
    }

    /* Add device */
    ret = platform_device_add(default_dev);
    if (ret) {
        printk(KERN_ERR "galcore: platform_device_add failed.\n");
        platform_device_put(default_dev);
        return ret;
    }

#if USE_LINUX_PCIE
    ret = pci_register_driver(&gpu_pci_subdriver);
    if (ret)
    {
        platform_device_unregister(default_dev);
        return ret;
    }
#endif
#else
    pdrv->driver.of_match_table = gpu_dt_ids;
    ret = gpu_power_domain_init();
    if (ret) {
        printk(KERN_ERR "galcore: gpu_gpc_init failed.\n");
    }
#endif

    *platform = (gcsPLATFORM *)&default_platform;
    return ret;
}

int gckPLATFORM_Terminate(struct _gcsPLATFORM *platform)
{
#if !gcdSUPPORT_DEVICE_TREE_SOURCE
    if (platform->device) {
        platform_device_unregister(platform->device);
        platform->device = NULL;
    }

#if USE_LINUX_PCIE
    {
        unsigned int dev_index;
        struct _gcsPLATFORM_PCIE *pcie_platform = (struct _gcsPLATFORM_PCIE *)platform;
        for (dev_index = 0; dev_index < pcie_platform->device_number; dev_index++)
        {
            unsigned int i;
            for (i = 0; i < MAX_PCIE_BAR; i++)
            {
                if (pcie_platform->pcie_info[dev_index].bar[i].logical != 0)
                {
                    pci_iounmap(pcie_platform->pcie_info[dev_index].pdev, pcie_platform->pcie_info[dev_index].bar[i].logical);
                }
            }
        }

        pci_unregister_driver(&gpu_pci_subdriver);
    }
#endif
#else
    gpu_power_domain_exit();
#endif
    return 0;
}
