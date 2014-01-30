/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */


/* MHI Includes */
#include "mhi_sys.h"
#include "mhi.h"
#include "mhi_macros.h"
#include "mhi_hwio.h"
#include "mhi_bhi.h"

#define BHI_DISABLED 1
int mhi_probe(struct pci_dev* mhi_device,
		const struct pci_device_id* mhi_device_id);
static void mhi_remove(struct pci_dev* mhi_device);
static int mhi_startup_thread(void* ctxt);
int rmnet_mhi_remove(struct pci_dev *dev);

/* MHI's list of all available pcie devices which support the MHI protocol */
mhi_pcie_devices mhi_devices;

static const struct pci_device_id mhi_pcie_device_id[] =
{
	{ MHI_PCIE_VENDOR_ID, MHI_PCIE_DEVICE_ID,PCI_ANY_ID,PCI_ANY_ID,0,0,0},
	{ 0, },
};
/*
   static struct attribute_group mhi_stats_attr[] =
   {
   __ATTR(probe, S_IWUSR | S_IRUSR, show_mhi_state, NULL),
   __ATTR_NULL,
   }; */

/* Publicize the existance of this device to userspace */
MODULE_DEVICE_TABLE(pci,mhi_pcie_device_id);
struct pci_driver mhi_pcie_driver =
{
	.name = "mhi_driver",
	.id_table = mhi_pcie_device_id,
	.probe = mhi_probe,
	.remove = mhi_remove,
	.suspend = mhi_suspend,
	.resume = mhi_resume,
};

int mhi_probe(struct pci_dev* pcie_device,
		const struct pci_device_id* mhi_device_id)
{
	int ret_val			 = 0;
	mhi_pcie_dev_info *mhi_pcie_dev = NULL;
	osal_thread mhi_startup_thread_handle = {0};

	mhi_log(MHI_MSG_INFO, "Entering.\n");


	mhi_pcie_dev = &mhi_devices.device_list[mhi_devices.nr_of_devices];
	if (mhi_devices.nr_of_devices + 1 > MHI_MAX_SUPPORTED_DEVICES)
	{
		mhi_log(MHI_MSG_ERROR,"Error: Too many devices\n");
		return -1;
	}

	mhi_devices.nr_of_devices++;
	mhi_pcie_dev->pcie_device = pcie_device;

	if (MHI_STATUS_SUCCESS != (ret_val = mhi_spawn_thread(mhi_pcie_dev,
					mhi_startup_thread,
					&mhi_startup_thread_handle,
					"MHI_DPROBE_THREAD")))
	{
		mhi_log(MHI_MSG_ERROR,"Failed to spawn deferred probe thread\n");
	}

	return ret_val;
}
static void mhi_remove(struct pci_dev* mhi_device)
{
	rmnet_mhi_remove(NULL);
	return;
}
static void __exit mhi_exit(void)
{
	pci_unregister_driver(&mhi_pcie_driver);
}

static int __init mhi_init(void)
{
	if (0!= pci_register_driver(&mhi_pcie_driver))
		return -EIO;
	return 0;
}

static void mhi_msm_fixup(struct pci_dev *pcie_device)
{
	if (pcie_device->class == PCI_CLASS_NOT_DEFINED)
	{
		mhi_log(MHI_MSG_INFO,"Setting msm pcie class\n");
		pcie_device->class = PCI_CLASS_STORAGE_SCSI;
	}

	/* @brief Client requests, a device by the pcie device id */
}

static int mhi_startup_thread(void* ctxt)
{
	int ret_val			 = 0;
	mhi_pcie_dev_info* mhi_pcie_dev = (mhi_pcie_dev_info*)ctxt;
	struct pci_dev* pcie_device = (struct pci_dev*)mhi_pcie_dev->pcie_device;

	if (NULL == ctxt)
		return -1;

	ret_val = mhi_init_pcie_device(mhi_pcie_dev);

	if (0 != ret_val)
	{
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to initialize pcie device, ret %d\n",
			ret_val);
	}
	ret_val = mhi_init_device_ctxt(mhi_pcie_dev,
					&mhi_pcie_dev->mhi_ctxt);
	if (MHI_STATUS_SUCCESS != ret_val)
		goto msi_config_err;

	/* Register for MSI */
	if (0 != (ret_val = pci_enable_msi(pcie_device)))
	{
		mhi_log(MHI_MSG_ERROR, "Failed to enable MSIs for pcie dev.\n");
		goto msi_config_err;
	}
	ret_val = request_irq(pcie_device->irq, (irq_handler_t)irq_cb,
					IRQF_NO_SUSPEND,
					"mhi_drv",
					(void*)&pcie_device->dev);
	if (0 != ret_val)
	{
		mhi_log(MHI_MSG_ERROR, "Failed to register handler for MSI.\n");
		goto msi_config_err;
	}
	ret_val = mhi_init_gpios(mhi_pcie_dev);
	if (0 != ret_val)
	{
		mhi_log(MHI_MSG_ERROR | MHI_DBG_POWER,
			"Failed to register for GPIO.\n");
		goto msi_config_err;
	}
	ret_val = mhi_init_pm_sysfs(&pcie_device->dev);
	if (0 != ret_val)
	{
		mhi_log(MHI_MSG_ERROR, "Failed to setup sysfs.\n");
		goto sysfs_config_err;
	}
	if (0 != mhi_init_debugfs(mhi_pcie_dev->mhi_ctxt))
	{
		mhi_log(MHI_MSG_ERROR, "Failed to init debugfs.\n");
	}

	pci_set_master(pcie_device);
	mhi_pcie_dev->mhi_ctxt->mmio_addr = (mhi_pcie_dev->core.bar0_base);
	pcie_device->dev.platform_data = (void*)&mhi_pcie_dev->mhi_ctxt;


#ifdef BHI_DISABLED
	/* Fire off the state transition  thread */
	ret_val = mhi_init_state_transition(mhi_pcie_dev->mhi_ctxt,
					STATE_TRANSITION_RESET);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_CRITICAL, "Failed to start state change event\n");
		goto mhi_state_transition_error;
	}
#else
	/* Register BHI, at this point the device can have firmware loaded */
	if (MHI_STATUS_SUCCESS != bhi_probe(mhi_pcie_dev)) {
		mhi_log(MHI_MSG_CRITICAL, "Failed to start BHI\n");
		goto mhi_state_transition_error;
	}
	ret_val = mhi_init_state_transition(mhi_pcie_dev->mhi_ctxt,
					STATE_TRANSITION_BHI);
#endif

	mhi_log(MHI_MSG_INFO,
			"Finished all driver probing returning ret_val %d.\n", ret_val);
	return ret_val;

mhi_state_transition_error:
	if (MHI_STATUS_SUCCESS != mhi_clean_init_stage(mhi_pcie_dev->mhi_ctxt,
				MHI_INIT_ERROR_STAGE_UNWIND_ALL))
		mhi_log(MHI_MSG_ERROR, "Could not clean up context\n");
sysfs_config_err:
	gpio_free(MHI_DEVICE_WAKE_GPIO);
msi_config_err:
	pci_disable_msi(pcie_device);
	pci_disable_device(pcie_device);
	return ret_val;
}

DECLARE_PCI_FIXUP_HEADER(MHI_PCIE_VENDOR_ID,
			MHI_PCIE_DEVICE_ID,
			mhi_msm_fixup);
module_exit(mhi_exit);
module_init(mhi_init);
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("MHI_CORE");
MODULE_AUTHOR("Andrei Danaila <adanaila@codeaurora.org>");
MODULE_DESCRIPTION("MHI Host Driver");
