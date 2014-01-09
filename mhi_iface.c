/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/sysfs.h>

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
static struct pci_driver mhi_pcie_driver =
{
	.name = "mhi_driver",
	.id_table = mhi_pcie_device_id,
	.probe = mhi_probe,
	.remove = mhi_remove,
	.suspend = NULL,
	.resume = NULL,
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
		goto mhi_pcie_read_ep_config_err;
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

mhi_pcie_read_ep_config_err:
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
	long int sleep_time = 100000;
	u32 j = 0;

	if (NULL == ctxt)
		return -1;

	mhi_log(MHI_MSG_INFO, "Entering deferred probe.\n");

	/* Enable the device */
	do {
		if (0 != (ret_val = pci_enable_device(mhi_pcie_dev->pcie_device))) {
			mhi_log(MHI_MSG_ERROR, 
				"Failed to enable pcie device ret_val=%d\n", ret_val);
			mhi_log(MHI_MSG_ERROR,
				"Sleeping for ~ %li uS, and retrying.\n", sleep_time);
			usleep(sleep_time);
		}
	} while(ret_val != 0);

	mhi_log(MHI_MSG_INFO,"Successfully enabled pcie device.\n" );

	mhi_pcie_dev->core.bar0_base = (uintptr_t)ioremap_nocache(pci_resource_start(pcie_device,0),
			pci_resource_len(pcie_device,0));
	mhi_pcie_dev->core.bar0_end = mhi_pcie_dev->core.bar0_base +
		pci_resource_len(pcie_device,0);
	mhi_pcie_dev->core.bar2_base = (uintptr_t)ioremap_nocache(pci_resource_start(pcie_device,2),
			pci_resource_len(pcie_device,2));
	mhi_pcie_dev->core.bar2_end = mhi_pcie_dev->core.bar2_base +
		pci_resource_len(pcie_device,2);

	if (0 == mhi_pcie_dev->core.bar0_base)
	{
		mhi_log(MHI_MSG_ERROR,"Failed to register for pcie resources\n");
		goto mhi_pcie_read_ep_config_err;
	}

	mhi_log(MHI_MSG_INFO,"Device BAR0 address is at 0x%llx\n",
			mhi_pcie_dev->core.bar0_base);

	if (0 != (ret_val = pci_request_region(pcie_device,0, mhi_pcie_driver.name)))
	{
		mhi_log(MHI_MSG_ERROR, "Could not request BAR0 region\n");
		goto mhi_pcie_read_ep_config_err;
	}

	mhi_pcie_dev->core.manufact_id = pcie_device->vendor;
	mhi_pcie_dev->core.dev_id = pcie_device->device;

	if (mhi_pcie_dev->core.manufact_id != MHI_PCIE_VENDOR_ID ||
			mhi_pcie_dev->core.dev_id != MHI_PCIE_DEVICE_ID)
	{
		mhi_log(MHI_MSG_ERROR,"Incorrect device/manufacturer ID\n");
		goto mhi_pcie_read_ep_config_err;
	}


	/* We need to ensure that the link is stable before we kick off MHI */
	j = 0;
	while ( 0xFFFFffff == pcie_read(mhi_pcie_dev->core.bar0_base , MHIREGLEN)
			&& j <= MHI_MAX_LINK_RETRIES)
	{
		mhi_log(MHI_MSG_ERROR, "LINK INSTABILITY DETECTED, retry %d\n",j);
		msleep(MHI_LINK_STABILITY_WAIT_MS);
		if (MHI_MAX_LINK_RETRIES == j)
		{
			ret_val = -EIO;
			mhi_log(MHI_MSG_ERROR, "LINK INSTABILITY DETECTED, FAILING!\n");
			goto mhi_device_list_error;
		}
		j++;
	}
	if (MHI_STATUS_SUCCESS != (ret_val = mhi_init_device_ctxt(mhi_pcie_dev,
					&mhi_pcie_dev->mhi_ctxt)))
		goto mhi_device_list_error;

	/* Register for MSI */
	if (0 != (ret_val = pci_enable_msi(pcie_device)))
	{
		mhi_log(MHI_MSG_ERROR, "Failed to enable MSIs for pcie dev.\n");
		goto mhi_pcie_read_ep_config_err;
	}

	if (0!= (ret_val = request_irq(pcie_device->irq, (irq_handler_t)irq_cb,
					IRQF_NO_SUSPEND,
					"mhi_drv",
					(void*)&pcie_device->dev)))
	{
		mhi_log(MHI_MSG_ERROR, "Failed to register handler for MSI.\n");
		goto mhi_pcie_read_ep_config_err;
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


	mhi_log(MHI_MSG_INFO, "Finished all driver probing returning ret_val %d.\n", ret_val);
	return ret_val;

mhi_state_transition_error:
	if (MHI_STATUS_SUCCESS != mhi_clean_init_stage(mhi_pcie_dev->mhi_ctxt,
				MHI_INIT_ERROR_STAGE_UNWIND_ALL))
		mhi_log(MHI_MSG_ERROR, "Could not clean up context\n");
	pci_disable_msi(pcie_device);
mhi_device_list_error:
	pci_disable_device(pcie_device);
mhi_pcie_read_ep_config_err:
	return ret_val;
}

DECLARE_PCI_FIXUP_HEADER(MHI_PCIE_VENDOR_ID, MHI_PCIE_DEVICE_ID,mhi_msm_fixup);
module_exit(mhi_exit);
module_init(mhi_init);
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("MHI_CORE");
MODULE_AUTHOR("Andrei Danaila <adanaila@codeaurora.org>");
MODULE_DESCRIPTION("MHI Host Driver");
