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

#include "msm_mhi.h"
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

	if (mhi_dev_ctxt->mhi_state == MHI_STATE_M0 ||
	    mhi_dev_ctxt->mhi_state == MHI_STATE_M1 ||
	    mhi_dev_ctxt->mhi_state == MHI_STATE_M2) {
	if (0 != mhi_initiate_M3(mhi_dev_ctxt))
		return -EIO;
	} else {
		return 0;
	}
	ret_val =
		msm_bus_scale_client_update_request(mhi_dev_ctxt->bus_client, 0);
	if (!mhi_dev_ctxt->link_up) {
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Link is not up, nothing to do.\n");
		return 0;
	}
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
	gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 0);
	return 0;
}

int mhi_resume(struct pci_dev *pcie_dev)
{
	int r = 0;
	mhi_device_ctxt *mhi_dev_ctxt =
			*(mhi_device_ctxt **)((pcie_dev->dev).platform_data);
	if (NULL == mhi_dev_ctxt)
		return 0;

	if (r)
		mhi_log(MHI_MSG_ERROR | MHI_DBG_POWER,
			"Could not set DEVICE WAKE GPIO HIGH\n");

	if (r) {
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"Failed to set power state 0x%x\n", r);
		return -EIO;
	}
	if (!mhi_dev_ctxt->link_up) {
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Link is not up, nothing to do.\n");
		return 0;
	}

	if (mhi_dev_ctxt->pending_M3) {
		mhi_log(MHI_MSG_CRITICAL,
			"Device did not ACK previous suspend request MHI STATE is 0x%x\n",
			mhi_dev_ctxt->mhi_state);
		return -ENETRESET;
		}

	mhi_initiate_M0(mhi_devices.device_list[0].mhi_ctxt);

	r = wait_event_interruptible_timeout(*mhi_devices.device_list[0].mhi_ctxt->M0_event,
		mhi_devices.device_list[0].mhi_ctxt->mhi_state != MHI_STATE_M3,
		msecs_to_jiffies(MHI_MAX_RESUME_TIMEOUT));
	switch(r) {
	case 0:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"MDM failed to resume after 0x%x ms\n",
			MHI_MAX_RESUME_TIMEOUT);
		r = -ETIMEDOUT;
		break;
	case -ERESTARTSYS:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"Going Down...\n");
		r = -ENETRESET;
		break;
	default:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"M0 event received\n");
		r = 0;
		break;
	}
	return r;
}

enum hrtimer_restart mhi_initiate_M1(struct hrtimer *timer)
{
	int ret_val = 0;
	unsigned long flags;
	ktime_t curr_time, timer_inc;
	mhi_device_ctxt *mhi_dev_ctxt = container_of(timer,
						mhi_device_ctxt,
						inactivity_tmr);
	write_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);

	/* We will allow M1 if no data is pending, the current
	 * state is M0 and no M3 transition is pending */
	if ((0 == atomic_read(&mhi_dev_ctxt->data_pending)) &&
			(MHI_STATE_M1 == mhi_dev_ctxt->mhi_state ||
			 MHI_STATE_M0 == mhi_dev_ctxt->mhi_state) &&
			(0 == mhi_dev_ctxt->pending_M3) &&
			mhi_dev_ctxt->mhi_initialized) {
		mhi_dev_ctxt->mhi_state = MHI_STATE_M1;
		ret_val = gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 0);
			mhi_log(MHI_MSG_VERBOSE,
					"Allowing M1.\n");
		mhi_dev_ctxt->m0_m1++;
		if (ret_val)
			mhi_log(MHI_MSG_ERROR | MHI_DBG_POWER,
				"Could not set DEVICE WAKE GPIO LOW\n");
	}
	write_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);
	curr_time = ktime_get();
	timer_inc = ktime_set(0, MHI_M1_ENTRY_DELAY_MS * 1E6L);
	hrtimer_forward(timer, curr_time, timer_inc);
	return HRTIMER_RESTART;
}

int mhi_initiate_M0(mhi_device_ctxt *mhi_dev_ctxt)
{
	int ret_val = 0;
	int r = 0;
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Initializing state transiton to M0\n");

	r =
		msm_bus_scale_client_update_request(mhi_dev_ctxt->bus_client, 1);
	if (r)
		mhi_log(MHI_MSG_CRITICAL,
			"Could not set bus frequency ret: %d\n",
			r);
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Setting WAKE GPIO HIGH.\n");
	ret_val = gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 1);
	if (ret_val)
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Failed to set DEVICE WAKE GPIO ret 0x%d.\n", ret_val);
	if (!mhi_dev_ctxt->link_up) {
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Link is not up, nothing to do, quitting.\n");
		return 0;
	}
	if (mhi_dev_ctxt->mhi_state == MHI_STATE_M2) {
		r = wait_event_interruptible_timeout(*mhi_dev_ctxt->M0_event,
		mhi_dev_ctxt->mhi_state != MHI_STATE_M2,
		msecs_to_jiffies(MHI_MAX_RESUME_TIMEOUT));
		if (r) {
			mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
				"MDM failed to come out of M2.\n");
			return -ENETRESET;
		}
	} else {
	MHI_REG_WRITE_FIELD(mhi_dev_ctxt->mmio_addr, MHICTRL,
			MHICTRL_MHISTATE_MASK,
			MHICTRL_MHISTATE_SHIFT,
			MHI_STATE_M0);
	}
	r = wait_event_interruptible_timeout(*mhi_devices.device_list[0].mhi_ctxt->M0_event,
		mhi_devices.device_list[0].mhi_ctxt->mhi_state != MHI_STATE_M3,
		msecs_to_jiffies(MHI_MAX_RESUME_TIMEOUT));
	switch(r) {
	case 0:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"MDM failed to resume after 0x%x ms\n",
			MHI_MAX_RESUME_TIMEOUT);
		mhi_dev_ctxt->m0_event_timeouts++;
		break;
	case -ERESTARTSYS:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"Going Down...\n");
		break;
	default:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
				"M0 event received\n");
		break;
	}
	return 0;
}

int mhi_initiate_M3(mhi_device_ctxt *mhi_dev_ctxt)
{
	unsigned long flags = 0;
	u32 i = 0;
	u32 failed_stop = 1;
	u32 ret_val = 0;
	int r = 0;

	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER, "Entering...\n");


	if (ret_val)
		mhi_log(MHI_MSG_CRITICAL,
			"Could not set bus frequency ret: %d\n",
			ret_val);
	write_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);
	mhi_dev_ctxt->pending_M3 = 1;
	gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 1);
	write_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);

	if (mhi_dev_ctxt->mhi_state == MHI_STATE_M2)
		r = wait_event_interruptible(*mhi_dev_ctxt->M0_event,
			MHI_STATE_M0 == mhi_dev_ctxt->mhi_state);
	if (r)
		return r;
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
	ret_val = wait_event_interruptible_timeout(*mhi_dev_ctxt->M3_event,
			mhi_dev_ctxt->mhi_state == MHI_STATE_M3,
		msecs_to_jiffies(MHI_MAX_SUSPEND_TIMEOUT));
	switch(ret_val) {
	case 0:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"MDM failed to suspend after %d ms\n",
			MHI_MAX_SUSPEND_TIMEOUT);
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"STT RP %p WP %p BASE %p Len %ld\n",
			mhi_dev_ctxt->state_change_work_item_list.q_info.rp,
			mhi_dev_ctxt->state_change_work_item_list.q_info.wp,
			mhi_dev_ctxt->state_change_work_item_list.q_info.base,
		mhi_dev_ctxt->state_change_work_item_list.q_info.len);
		mhi_dev_ctxt->m3_event_timeouts++;
		ret_val = -ETIMEDOUT;
		break;
	case -ERESTARTSYS:
		mhi_log(MHI_MSG_CRITICAL | MHI_DBG_POWER,
			"Going Down...\n");
		ret_val = -ENETRESET;
		break;
	default:
		mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"M3 completion received\n");
		ret_val = 0;
		break;
	}
	gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 0);
	return ret_val;
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
		return r;
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

	return count;
}

ssize_t sysfs_init_M1(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	return count;
}
