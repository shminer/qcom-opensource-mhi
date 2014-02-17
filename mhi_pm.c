/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
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

#include "mhi_sys.h"
#include "mhi.h"
#include "mhi_hwio.h"

/* Write only sysfs attributes */
static DEVICE_ATTR(MHI_M3, S_IWUSR, NULL, sysfs_init_M3);
static DEVICE_ATTR(MHI_M0, S_IWUSR, NULL, sysfs_init_M0);
static DEVICE_ATTR(MHI_M1, S_IWUSR, NULL, sysfs_init_M1);

/* Read only sysfs attributes */
static DEVICE_ATTR(MHI_STATE, S_IRUSR, sysfs_get_mhi_state, NULL);

static struct attribute *mhi_attributes[] = {
	&dev_attr_MHI_M3.attr,
	&dev_attr_MHI_M0.attr,
	&dev_attr_MHI_M1.attr,
	&dev_attr_MHI_STATE.attr,
	NULL,
};

static struct attribute_group mhi_attribute_group = {
	.attrs = mhi_attributes,
};

int mhi_suspend(struct pci_dev *pcie_dev, pm_message_t state)
{
	int ret_val = 0;
	mhi_device_ctxt *mhi_dev_ctxt =
		*(mhi_device_ctxt **)((pcie_dev->dev).platform_data);
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "Entered, state %d\n",
						state.event);
	if (NULL == mhi_dev_ctxt)
		return 0;

	if (MHI_STATUS_SUCCESS != mhi_initiate_M3(mhi_dev_ctxt))
		return -EIO;

	ret_val = pci_save_state(pcie_dev);
	if (ret_val) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to save pci device state ret %d\n",
				ret_val);
		return MHI_STATUS_ERROR;
	}

	ret_val = pci_set_power_state(pcie_dev, PCI_D3hot);
	if (ret_val) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"Failed to put device in D3 hot ret %d\n", ret_val);
		return MHI_STATUS_ERROR;
	}
	return 0;
}

int mhi_resume(struct pci_dev *pcie_dev)
{
	int r = 0;
	mhi_device_ctxt *mhi_dev_ctxt =
			*(mhi_device_ctxt **)((pcie_dev->dev).platform_data);
	if (NULL == mhi_dev_ctxt)
		return 0;
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "Entered\n");
	r = msm_pcie_pm_control(MSM_PCIE_RESUME,
				pcie_dev->bus->number,
				pcie_dev,
				NULL,
				0);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to resume pcie bus ret 0x%x\n", r);
		return -EIO;
	}
	r = pci_set_power_state(pcie_dev, PCI_D0);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to set power state 0x%x\n", r);
		return -EIO;
	}
	pci_restore_state(pcie_dev);

	if (MHI_STATUS_SUCCESS != mhi_init_state_transition(mhi_dev_ctxt,
				STATE_TRANSITION_M0))
		mhi_log(MHI_MSG_CRITICAL, "Failed to transition back to M0\n");

	mhi_initiate_M0(mhi_devices.device_list[0].mhi_ctxt);

	/* We could go to M1 */
	wait_event_interruptible(*mhi_dev_ctxt->M0_event,
			mhi_dev_ctxt->mhi_state != MHI_STATE_M3);
	return 0;
}

enum hrtimer_restart mhi_initiate_M1(struct hrtimer *timer)
{
	int ret_val = 0;
	unsigned long flags;
	mhi_device_ctxt *mhi_dev_ctxt = container_of(timer,
						mhi_device_ctxt,
						inactivity_tmr);
	write_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);

	/* We will allow M1 if no data is pending, the current
	 * state is M0 and no M3 transition is pending */
	if (0 == atomic_read(&mhi_dev_ctxt->data_pending) &&
			MHI_STATE_M0 == mhi_dev_ctxt->mhi_state &&
			0 == mhi_dev_ctxt->pending_M3) {
		mhi_dev_ctxt->mhi_state = MHI_STATE_M1;
		ret_val = gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 0);
		mhi_dev_ctxt->m0_m1++;
		if (ret_val)
			mhi_log(MHI_MSG_ERROR | MHI_DBG_POWER,
				"Could not set DEVICE WAKE GPIO LOW\n");
	}
	write_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);
	return HRTIMER_NORESTART;
}

MHI_STATUS mhi_initiate_M0(mhi_device_ctxt *mhi_dev_ctxt)
{
	int ret_val = 0;
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Initializing state transiton to M0\n");

	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Setting WAKE GPIO HIGH.\n");
	ret_val = gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 1);
	if (ret_val)
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Failed to set DEVICE WAKE GPIO ret 0x%d.\n", ret_val);
	MHI_REG_WRITE_FIELD(mhi_dev_ctxt->mmio_addr, MHICTRL,
			MHICTRL_MHISTATE_MASK,
			MHICTRL_MHISTATE_SHIFT,
			MHI_STATE_M0);
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS mhi_initiate_M3(mhi_device_ctxt *mhi_dev_ctxt)
{
	unsigned long flags = 0;
	u32 i = 0;
	u32 failed_stop = 1;

	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Entering...\n");
	write_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);
	mhi_dev_ctxt->pending_M3 = 1;
	gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 1);
	write_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);
	if (mhi_dev_ctxt->mhi_state == MHI_STATE_M2)
		wait_event_interruptible(*mhi_dev_ctxt->M0_event,
			MHI_STATE_M0 == mhi_dev_ctxt->mhi_state);

	while (i < MHI_MAX_SUSPEND_RETRIES) {
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Waiting for clients to stop, clients still active %d\n",
			atomic_read(&mhi_dev_ctxt->data_pending));
		if (atomic_read(&mhi_dev_ctxt->data_pending) > 0) {
			++i;
			usleep(20000);
		} else {
			failed_stop = 0;
			break;
		}
	}
	if (failed_stop)
		return -EPERM;

	/* Since we are going down, inform all clients
	 * that no further reads are possible */
	MHI_REG_WRITE_FIELD(mhi_dev_ctxt->mmio_addr, MHICTRL,
			MHICTRL_MHISTATE_MASK,
			MHICTRL_MHISTATE_SHIFT,
			MHI_STATE_M3);
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Waiting for M3 completion.\n");
	wait_event_interruptible(*mhi_dev_ctxt->M3_event,
			mhi_dev_ctxt->mhi_state == MHI_STATE_M3);
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"M3 completion received\n");
	return MHI_STATUS_SUCCESS;
}

int mhi_init_pm_sysfs(struct device *dev)
{
	return sysfs_create_group(&dev->kobj, &mhi_attribute_group);
}

ssize_t sysfs_init_M3(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int r = 0;
	r = mhi_initiate_M3(mhi_devices.device_list[0].mhi_ctxt);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to suspend %d\n", r);
		return -EIO;
	}
	r = pci_save_state(
		mhi_devices.device_list[0].mhi_ctxt->dev_info->pcie_device);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to save pci device state ret %d\n", r);
		return r;
	}
	r = pci_set_power_state(
		mhi_devices.device_list[0].mhi_ctxt->dev_info->pcie_device,
		PCI_D3hot);
	if (r)
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to set pcie power state ret: %x\n", r);

	r = msm_pcie_pm_control(MSM_PCIE_SUSPEND,
			mhi_devices.device_list[0].pcie_device->bus->number,
			mhi_devices.device_list[0].pcie_device,
			NULL,
			0);
	if (r)
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to suspend pcie bus ret 0x%x\n", r);

	return count;
}

ssize_t sysfs_get_mhi_state(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return 0;
}

ssize_t sysfs_init_M0(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int r = 0;
	r = msm_pcie_pm_control(MSM_PCIE_RESUME,
			mhi_devices.device_list[0].pcie_device->bus->number,
			mhi_devices.device_list[0].pcie_device,
			NULL,
			0);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to resume pcie bus ret 0x%x\n", r);
		return -EIO;
	}

	r = pci_set_power_state(mhi_devices.device_list[0].pcie_device,
				PCI_D0);
	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to set power state 0x%x\n", r);
		return -EIO;
	}

	pci_restore_state(mhi_devices.device_list[0].pcie_device);

	mhi_initiate_M0(mhi_devices.device_list[0].mhi_ctxt);
	mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"Waiting for M0 event\n");
	mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"Current mhi_state = 0x%x\n",
			mhi_devices.device_list[0].mhi_ctxt->mhi_state);
	wait_event_interruptible(*mhi_devices.device_list[0].mhi_ctxt->M0_event,
		mhi_devices.device_list[0].mhi_ctxt->mhi_state != MHI_STATE_M3);
	mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"M0 event received\n");
	return count;
}

ssize_t sysfs_init_M1(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	return count;
}
