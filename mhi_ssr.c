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

#include <mhi_sys.h>
#include <mhi.h>
extern mhi_pcie_devices mhi_devices;
int mhi_ssr_notify_cb(struct notifier_block *nb,
			unsigned long action, void *data);


static struct notifier_block mhi_ssr_nb = {
	.notifier_call = mhi_ssr_notify_cb,
};

static void esoc_parse_link_type(mhi_device_ctxt* mhi_dev_ctxt)
{
	int ret_val;
	ret_val = strncmp(mhi_dev_ctxt->esoc_handle->link,
				"HSIC+PCIe",
				sizeof("HSIC+PCIe"));
	mhi_log(MHI_MSG_VERBOSE, "Link type is %s as indicated by ESOC\n",
					mhi_dev_ctxt->esoc_handle->link);
	if (ret_val)
		mhi_dev_ctxt->base_state = STATE_TRANSITION_BHI;
	else
		mhi_dev_ctxt->base_state = STATE_TRANSITION_RESET;
}

int mhi_esoc_register(mhi_device_ctxt* mhi_dev_ctxt)
{
	int ret_val = 0;
	struct device_node *np;
	struct pci_driver *mhi_driver;
	struct device *dev = &mhi_dev_ctxt->dev_info->pcie_device->dev;

	mhi_driver = mhi_dev_ctxt->dev_info->mhi_pcie_driver;
	np = dev->of_node;
	mhi_dev_ctxt->esoc_handle = devm_register_esoc_client(dev, "mdm");
	mhi_log(MHI_MSG_VERBOSE,
		"Of table of pcie device property is dev->of_node %p \n",
		np);
	if (IS_ERR_OR_NULL(mhi_dev_ctxt->esoc_handle)) {
		mhi_log(MHI_MSG_CRITICAL,
			"Failed to register for SSR, ret %lx\n",
			(uintptr_t)mhi_dev_ctxt->esoc_handle);
		return -EIO;
	}

	esoc_parse_link_type(mhi_dev_ctxt);

	mhi_dev_ctxt->esoc_ssr_handle = subsys_notif_register_notifier(
					mhi_dev_ctxt->esoc_handle->name,
					&mhi_ssr_nb);
	if (IS_ERR_OR_NULL(mhi_dev_ctxt->esoc_ssr_handle)) {
		ret_val = PTR_RET(mhi_dev_ctxt->esoc_ssr_handle);
		mhi_log(MHI_MSG_CRITICAL,
			"Can't find esoc desc ret 0x%lx\n",
			(uintptr_t)mhi_dev_ctxt->esoc_ssr_handle);
	}

	return ret_val;
}


void mhi_notify_clients(mhi_device_ctxt *mhi_dev_ctxt, MHI_CB_REASON reason)
{
	u32 i;
	mhi_client_handle *client_handle = NULL;
	mhi_cb_info cb_info = {0};
	mhi_result result = {0};
	for (i = 0; i < MHI_MAX_CHANNELS; ++i) {
		if (VALID_CHAN_NR(i))
			client_handle =
				mhi_dev_ctxt->client_handle_list[i];
			cb_info.result = NULL;
			cb_info.cb_reason = reason;
			if (NULL != client_handle &&
			    NULL != client_handle->client_info.mhi_client_cb) {
				result.user_data = client_handle->user_data;
				cb_info.result = &result;
				client_handle->client_info.mhi_client_cb(&cb_info);
			}
	}
}
void mhi_link_state_cb(struct msm_pcie_notify *notify)
{
	unsigned long flags;
	MHI_STATUS ret_val;
	int r;
	mhi_pcie_dev_info *mhi_pcie_dev = notify->data;
	mhi_device_ctxt *mhi_dev_ctxt = NULL;
	mhi_log(MHI_MSG_INFO, "Entered\n");
	if (NULL == notify || NULL == notify->data) {
		mhi_log(MHI_MSG_CRITICAL,
		"Incomplete handle received\n");
		return;
	}
	mhi_dev_ctxt = mhi_pcie_dev->mhi_ctxt;
	switch (notify->event){
	case MSM_PCIE_EVENT_LINKDOWN:
		mhi_log(MHI_MSG_INFO,
			"Received Link Down Callback\n");
		if (NULL == mhi_dev_ctxt)
			return;
		mhi_dev_ctxt->link_up = 0;
		mhi_log(MHI_MSG_CRITICAL,
			"Informing clients of MHI that link is down\n");
		write_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);
		mhi_dev_ctxt->mhi_state = MHI_STATE_RESET;
		ret_val = gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 0);
		write_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);
		//mhi_notify_clients(mhi_dev_ctxt, MHI_CB_MHI_DISABLED);
		mhi_dev_ctxt->mhi_initialized = 0;
		r =
		msm_bus_scale_client_update_request(mhi_dev_ctxt->bus_client, 0);
		if (r)
			mhi_log(MHI_MSG_INFO,
				"Failed to scale bus request to sleep set.\n");
		mhi_pcie_dev->link_down_cntr++;
		break;
	case MSM_PCIE_EVENT_LINKUP:
		ret_val = gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 1);
		if (0 == mhi_pcie_dev->link_up_cntr) {
		mhi_log(MHI_MSG_INFO,
			"Initializing MHI for the first time\n");
			mhi_startup_thread(mhi_pcie_dev);
			mhi_dev_ctxt = mhi_pcie_dev->mhi_ctxt;
			pci_set_master(mhi_pcie_dev->pcie_device);
		} else {
			mhi_log(MHI_MSG_INFO,
		"Received Link Up Callback\n");
		}
		mhi_dev_ctxt->link_up = 1;
		r =
		msm_bus_scale_client_update_request(mhi_dev_ctxt->bus_client, 1);
		if (r)
		mhi_log(MHI_MSG_INFO,
			"Failed to scale bus request to active set.\n");
		ret_val = mhi_init_state_transition(mhi_dev_ctxt,
				mhi_dev_ctxt->base_state);
		if (MHI_STATUS_SUCCESS != ret_val) {
			mhi_log(MHI_MSG_CRITICAL,
			"Failed to start state change event, to %d\n",
			mhi_dev_ctxt->base_state);
		}
		mhi_pcie_dev->link_up_cntr++;
		break;
	default:
		mhi_log(MHI_MSG_INFO,
			"Received bad link event\n");
		return;
		break;
		}
	mhi_log(MHI_MSG_INFO, "Exited\n");
}
int mhi_ssr_notify_cb(struct notifier_block *nb,
			unsigned long action, void *data)
{
	unsigned long flags;
	int ret_val = 0;
	mhi_device_ctxt *mhi_dev_ctxt = mhi_devices.device_list[0].mhi_ctxt;
	mhi_pcie_dev_info *mhi_pcie_dev = NULL;
	mhi_pcie_dev = &mhi_devices.device_list[mhi_devices.nr_of_devices];
	mhi_log(MHI_MSG_VERBOSE,
		"Received Subsystem event 0x%lx, from esoc %p\n",
		action, data);
	switch (action) {
	case SUBSYS_AFTER_POWERUP:
		mhi_log(MHI_MSG_VERBOSE,
		"Received Subsystem event AFTER_POWERUP\n");
		break;
	case SUBSYS_BEFORE_SHUTDOWN:
		mhi_log(MHI_MSG_VERBOSE,
		"Received Subsystem event BEFORE_SHUTDOWN\n");
		write_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);
		/* After the operations in this spinlock are complete.
		 * no more dbs can be written to the device */
		mhi_dev_ctxt->link_up = 0;
		mhi_dev_ctxt->mhi_state = MHI_STATE_RESET;
		ret_val = gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 1);
		write_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);
		mhi_notify_clients(mhi_dev_ctxt, MHI_CB_MHI_DISABLED);
		mhi_dev_ctxt->mhi_initialized = 0;
		mhi_pcie_dev->link_down_cntr++;
		break;
	case SUBSYS_AFTER_SHUTDOWN:
		mhi_log(MHI_MSG_VERBOSE,
		"Received Subsystem event AFTER_SHUTDOWN\n");
		break;
	case SUBSYS_BEFORE_POWERUP:
		mhi_log(MHI_MSG_VERBOSE,
		"Received Subsystem event BEFORE_POWERUP\n");
		break;
	case SUBSYS_RAMDUMP_NOTIFICATION:
		mhi_log(MHI_MSG_VERBOSE,
		"Received Subsystem event RAMDUMP_NOTIFICATION\n");
		break;
	case SUBSYS_PROXY_VOTE:
		mhi_log(MHI_MSG_VERBOSE,
		"Received Subsystem event SYBSYS_PROXY_VOTE\n");
		break;
	case SUBSYS_PROXY_UNVOTE:
		mhi_log(MHI_MSG_VERBOSE,
		"Received Subsystem event SYBSYS_PROXY_UNVOTE\n");
		break;
	case SUBSYS_POWERUP_FAILURE:
		mhi_log(MHI_MSG_VERBOSE,
		"Received Subsystem event SYBSYS_POWERUP_FAILURE\n");
		break;
	}
	return NOTIFY_OK;
}
