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
#include "mhi_sys.h"
#include "mhi.h"
#include "mhi_hwio.h"
#include "mhi_macros.h"

extern struct pci_driver mhi_pcie_driver;
int mhi_init_pcie_device (mhi_pcie_dev_info *mhi_pcie_dev)
{
	int ret_val = 0;
	u32 j = 0;
	long int sleep_time = 100000;
	struct pci_dev* pcie_device = (struct pci_dev*)mhi_pcie_dev->pcie_device;
	/* Enable the device */
	do {
		ret_val = pci_enable_device(mhi_pcie_dev->pcie_device);
		if (0 != ret_val) {
			mhi_log(MHI_MSG_ERROR,
				"Failed to enable pcie device ret_val=%d\n",
				ret_val);
			mhi_log(MHI_MSG_ERROR,
				"Sleeping for ~ %li uS, and retrying.\n",
				sleep_time);
			usleep(sleep_time);
		}
	} while(ret_val != 0);

	mhi_log(MHI_MSG_INFO, "Successfully enabled pcie device.\n");

	mhi_pcie_dev->core.bar0_base =
		(uintptr_t)ioremap_nocache(pci_resource_start(pcie_device, 0),
			pci_resource_len(pcie_device, 0));
	mhi_pcie_dev->core.bar0_end = mhi_pcie_dev->core.bar0_base +
		pci_resource_len(pcie_device, 0);
	mhi_pcie_dev->core.bar2_base =
		(uintptr_t)ioremap_nocache(pci_resource_start(pcie_device, 2),
			pci_resource_len(pcie_device, 2));
	mhi_pcie_dev->core.bar2_end = mhi_pcie_dev->core.bar2_base +
		pci_resource_len(pcie_device, 2);

	if (0 == mhi_pcie_dev->core.bar0_base)
	{
		mhi_log(MHI_MSG_ERROR,
			"Failed to register for pcie resources\n");
		goto mhi_pcie_read_ep_config_err;
	}

	mhi_log(MHI_MSG_INFO,"Device BAR0 address is at 0x%llx\n",
			mhi_pcie_dev->core.bar0_base);

	if (0 != (ret_val = pci_request_region(pcie_device,
					0, mhi_pcie_driver.name)))
	{
		mhi_log(MHI_MSG_ERROR, "Could not request BAR0 region\n");
	}

	mhi_pcie_dev->core.manufact_id = pcie_device->vendor;
	mhi_pcie_dev->core.dev_id = pcie_device->device;

	if (mhi_pcie_dev->core.manufact_id != MHI_PCIE_VENDOR_ID ||
			mhi_pcie_dev->core.dev_id != MHI_PCIE_DEVICE_ID)
	{
		mhi_log(MHI_MSG_ERROR, "Incorrect device/manufacturer ID\n");
		goto mhi_pcie_read_ep_config_err;
	}
	/* We need to ensure that the link is stable before we kick off MHI */
	j = 0;
	while (0xFFFFffff == pcie_read(mhi_pcie_dev->core.bar0_base, MHIREGLEN)
			&& j <= MHI_MAX_LINK_RETRIES) {
		mhi_log(MHI_MSG_ERROR, "LINK INSTABILITY DETECTED, retry %d\n", j);
		msleep(MHI_LINK_STABILITY_WAIT_MS);
		if (MHI_MAX_LINK_RETRIES == j)
		{
			ret_val = -EIO;
			mhi_log(MHI_MSG_ERROR,
				"LINK INSTABILITY DETECTED, FAILING!\n");
			goto mhi_device_list_error;
		}
		j++;
	}
	return 0;
mhi_device_list_error:
	pci_disable_device(pcie_device);
mhi_pcie_read_ep_config_err:
	return -EIO;
}

int mhi_init_gpios(mhi_pcie_dev_info *mhi_pcie_dev)
{
	int ret_val = 0;
	mhi_log(MHI_MSG_VERBOSE | MHI_DBG_POWER,
			"Attempting to grab DEVICE_WAKE gpio\n");

	ret_val = gpio_request(MHI_DEVICE_WAKE_GPIO, "mhi");
	if (ret_val) {
		mhi_log(MHI_MSG_VERBOSE | MHI_DBG_POWER,
			"Could not obtain device WAKE gpio\n");
	}
	mhi_log(MHI_MSG_VERBOSE | MHI_DBG_POWER,
		"Attempting to set output direction to DEVICE_WAKE gpio\n");
	/* This GPIO must never sleep as it can be set in timer ctxt */
	gpio_set_value_cansleep(MHI_DEVICE_WAKE_GPIO, 0);
	if (ret_val)
		mhi_log(MHI_MSG_ERROR | MHI_DBG_POWER,
		"Could not set GPIO to not sleep!\n");

	ret_val = gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 1);

	if (ret_val)
	{
		mhi_log(MHI_MSG_ERROR | MHI_DBG_POWER,
				"Failed to set output direction of DEVICE_WAKE gpio\n");
		goto mhi_gpio_dir_err;
	}
	return 0;

mhi_gpio_dir_err:
	gpio_free(MHI_DEVICE_WAKE_GPIO);
	return -EIO;
}
MHI_STATUS mhi_open_channel(mhi_client_handle **client_handle,
		MHI_CLIENT_CHANNEL chan, s32 device_index,
		mhi_client_cbs_t *cbs, void *UserData)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_control_seg *mhi_ctrl_seg = NULL;

	if (!VALID_CHAN_NR(chan)) {
		ret_val = MHI_STATUS_INVALID_CHAN_ERR;
		goto error_handle;
	}
	if (NULL == client_handle || device_index < 0 ||
			device_index >= mhi_devices.nr_of_devices) {
		ret_val = MHI_STATUS_ERROR;
		goto error_handle;
	}
	mhi_log(MHI_MSG_INFO,
			"Opened channel 0x%x for client\n", chan);

	atomic_inc(&mhi_devices.device_list[device_index].ref_count);

	*client_handle = mhi_malloc(sizeof(mhi_client_handle));
	if (NULL == *client_handle) {
		ret_val = MHI_STATUS_ALLOC_ERROR;
		goto error_handle;
	}
	mhi_memset(*client_handle, 0, sizeof(mhi_client_handle));
	(*client_handle)->chan = chan;
	(*client_handle)->mhi_dev_ctxt =
		mhi_devices.device_list[device_index].mhi_ctxt;
	mhi_ctrl_seg = (*client_handle)->mhi_dev_ctxt->mhi_ctrl_seg;

	(*client_handle)->mhi_dev_ctxt->client_handle_list[chan] =
							*client_handle;
	if (NULL != cbs)
		(*client_handle)->client_cbs = *cbs;

	(*client_handle)->user_data = UserData;
	(*client_handle)->event_ring_index =
		mhi_ctrl_seg->mhi_cc_list[chan].mhi_event_ring_index;

error_handle:
	return ret_val;
}

void mhi_close_channel(mhi_client_handle *mhi_handle)
{
	u32 index = 0;
	if (NULL == mhi_handle)
		return;
	index = mhi_handle->device_index;
	mhi_handle->mhi_dev_ctxt->client_handle_list[mhi_handle->chan] = NULL;
	atomic_dec(&(mhi_devices.device_list[index].ref_count));
	mhi_free(mhi_handle);
}

/**
 * @brief Add elements to event ring for the device to use
 *
 * @param mhi device context
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_add_elements_to_event_rings(mhi_device_ctxt *device,
					MHI_STATE_TRANSITION new_state)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	switch (new_state){
	case STATE_TRANSITION_READY:
		ret_val = mhi_init_event_ring(device,
				EV_EL_PER_RING,
				device->alloced_ev_rings[PRIMARY_EVENT_RING]);
		if (MHI_STATUS_SUCCESS != ret_val) {

			mhi_log(MHI_MSG_ERROR,
				"Failed to add ev el on event ring\n");
			return MHI_STATUS_ERROR;
		}
		break;
	case STATE_TRANSITION_M0:
		ret_val = mhi_init_event_ring(device,
				EV_EL_PER_RING,
				device->alloced_ev_rings[SECONDARY_EVENT_RING]);
		if (MHI_STATUS_SUCCESS != ret_val) {

			mhi_log(MHI_MSG_ERROR,
				"Failed to add ev el on event ring\n");
			return MHI_STATUS_ERROR;
		}
		ret_val = mhi_init_event_ring(device,
				EV_EL_PER_RING,
				device->alloced_ev_rings[TERTIARY_EVENT_RING]);
		if (MHI_STATUS_SUCCESS != ret_val) {
			mhi_log(MHI_MSG_ERROR,
				"Failed to add ev el on event ring\n");
			return MHI_STATUS_ERROR;
		}
	break;
	default:
		mhi_log(MHI_MSG_ERROR,
			"Unrecognized event stage, %d\n", new_state);
		ret_val = MHI_STATUS_ERROR;
		break;
	}
	return ret_val;
}
/**
 * @brief Add available TRBs to an IN channel
 *
 * @param device mhi device context
 * @param chan mhi channel number
 *
 * @return MHI_STATUS
 */

/**
 * @brief Function for sending data on an outbound channel.
 * This function only sends on TRE's worth of
 * data and may chain the TRE as specified by the caller.
 *
 * @param device [IN ] Pointer to mhi context used to send the TRE
 * @param chan [IN ] Channel number to send the TRE on
 * @param buf [IN ] Physical address of buffer to be linked to descriptor
 * @param buf_len [IN ] Length of buffer, which will be populated in the TRE
 * @param chain [IN ] Specification on whether this TRE should be chained
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_queue_xfer(mhi_client_handle *client_handle,
		uintptr_t buf, size_t buf_len, u32 chain)
{
	u64 db_value = 0;
	mhi_xfer_pkt *pkt_loc = NULL;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	MHI_CLIENT_CHANNEL chan = 0;
	mhi_device_ctxt *mhi_dev_ctxt = NULL;
	unsigned long flags;
	uintptr_t trb_index = 0;

	if (NULL == client_handle || !VALID_CHAN_NR(client_handle->chan) ||
		0 == buf || chain >= MHI_TRE_CHAIN_LIMIT || 0 == buf_len) {
		mhi_log(MHI_MSG_CRITICAL, "Bad input args\n");
		return MHI_STATUS_ERROR;
	}

	MHI_OSAL_ASSERT(VALID_BUF(buf, buf_len));
	mhi_dev_ctxt = client_handle->mhi_dev_ctxt;
	chan = client_handle->chan;

	if (MHI_CHAN_STATE_RUNNING !=
		mhi_dev_ctxt->mhi_local_chan_ctxt[chan].mhi_chan_state) {
		mhi_log(MHI_MSG_CRITICAL, "Channel not ready\n");
		return MHI_STATUS_CHAN_NOT_READY;
	}

	/* Bump up the vote for pending data */
	read_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);

	if (1 == atomic_add_return(1, &mhi_dev_ctxt->data_pending))
	{
		hrtimer_try_to_cancel(&mhi_dev_ctxt->inactivity_tmr);
		mhi_dev_ctxt->m1_m0++;
	}
	gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 1);

	read_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);

	/* Add the TRB to the correct transfer ring */
	ret_val = add_element(&mhi_dev_ctxt->mhi_local_chan_ctxt[chan],
				(void *)&pkt_loc);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_INFO, "Failed to insert trb in xfer ring\n");
		goto error;
	}

	pkt_loc->data_tx_pkt.buffer_ptr = (uintptr_t)buf;

	if (MHI_TRE_CHAIN_ON == chain)
		MHI_TRB_SET_INFO(TX_TRB_CHAIN, pkt_loc, MHI_TRE_CHAIN_ON);
	else
		MHI_TRB_SET_INFO(TX_TRB_CHAIN, pkt_loc, MHI_TRE_CHAIN_OFF);

	get_element_index(&mhi_dev_ctxt->mhi_local_chan_ctxt[chan],
				pkt_loc, &trb_index);

	/* OUTBOUND hardware channel */
	/*
	   if (IS_HARDWARE_CHANNEL(chan) && ((chan %2) == 0))
	   {
	   if ( (trb_index % mhi_dev_ctxt->hw_intmod_rate) == 0)
	   {

	   MHI_TRB_SET_INFO(TX_TRB_BEI ,pkt_loc, 0);
	   MHI_TRB_SET_INFO(TX_TRB_IEOT ,pkt_loc, 1);
	   }
	   else
	   {
	   MHI_TRB_SET_INFO(TX_TRB_BEI ,pkt_loc, 1);
	   MHI_TRB_SET_INFO(TX_TRB_IEOT ,pkt_loc, 0);
	   }
	   }
	   else
	   {
	   MHI_TRB_SET_INFO(TX_TRB_BEI ,pkt_loc, 0);
	   MHI_TRB_SET_INFO(TX_TRB_IEOT ,pkt_loc, 1);
	   }*/
	if (IS_HARDWARE_CHANNEL(chan))
		MHI_TRB_SET_INFO(TX_TRB_BEI, pkt_loc, 1);
	else
		MHI_TRB_SET_INFO(TX_TRB_BEI, pkt_loc, 0);
	MHI_TRB_SET_INFO(TX_TRB_IEOT, pkt_loc, 1);
	MHI_TRB_SET_INFO(TX_TRB_IEOB, pkt_loc, 0);
	MHI_TRB_SET_INFO(TX_TRB_TYPE, pkt_loc, MHI_PKT_TYPE_TRANSFER);
	MHI_TX_TRB_SET_LEN(TX_TRB_LEN, pkt_loc, buf_len);


	if (MHI_STATE_M0 == mhi_dev_ctxt->mhi_state ||
		MHI_STATE_M1 == mhi_dev_ctxt->mhi_state) {
		mhi_log(MHI_MSG_INFO,
			"Current MHI device state %d\n",
			mhi_dev_ctxt->mhi_state);
		atomic_inc(&mhi_dev_ctxt->mhi_chan_db_order[chan]);
		mhi_acquire_spinlock(&mhi_dev_ctxt->db_write_lock[chan]);
		db_value = mhi_v2p_addr(mhi_dev_ctxt->mhi_ctrl_seg_info,
			(uintptr_t)mhi_dev_ctxt->mhi_local_chan_ctxt[chan].wp);
		MHI_WRITE_DB(mhi_dev_ctxt->channel_db_addr, chan, db_value);
		mhi_release_spinlock(&mhi_dev_ctxt->db_write_lock[chan]);
	} else {
		mhi_log(MHI_MSG_INFO,
			"Current MHI device state %d, not M0 or M1\n",
			mhi_dev_ctxt->mhi_state);
	}
	/* If there are no clients still sending we can trigger our
	 * inactivity timer */
	if (0 == atomic_sub_return(1, &mhi_dev_ctxt->data_pending))
	{
		hrtimer_start(&mhi_dev_ctxt->inactivity_tmr,
			mhi_dev_ctxt->inactivity_timeout,
			HRTIMER_MODE_REL);
		wake_up(&mhi_dev_ctxt->mhi_xfer_stop->event);
	}
	return MHI_STATUS_SUCCESS;
error:
	atomic_dec(&mhi_dev_ctxt->data_pending);
	return ret_val;
}
/**
 * @brief Function used to send a command TRE to the mhi device.
 *
 * @param device [IN ] Specify the mhi dev context to which to send the command
 * @param cmd [IN ] Enum specifying which command to send to device
 * @param chan [in ] Channel number for which this command is intended,
 * not applicable for all commands
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_send_cmd(mhi_device_ctxt *device, MHI_COMMAND cmd, u32 chan)
{
	u64 db_value = 0;
	mhi_cmd_pkt *pkt_loc = NULL;
	MHI_CHAN_STATE from_state = MHI_CHAN_STATE_DISABLED;
	MHI_CHAN_STATE to_state = MHI_CHAN_STATE_DISABLED;
	MHI_PKT_TYPE ring_el_type = MHI_PKT_TYPE_NOOP_CMD;
	osal_mutex *cmd_mutex = NULL;
	osal_mutex *chan_mutex = NULL;

	if (chan >= MHI_MAX_CHANNELS ||
		cmd >= MHI_COMMAND_MAX_NR || NULL == device) {
		mhi_log(MHI_MSG_ERROR,
			"Invalid channel id, received id: 0x%x", chan);
		goto error_general;
	}

	/*If there is a cmd pending a device confirmation, do not send anymore
	  for this channel */
	if (MHI_CMD_PENDING == device->mhi_chan_pend_cmd_ack[chan])
		return MHI_STATUS_CMD_PENDING;

	from_state = device->mhi_ctrl_seg->mhi_cc_list[chan].mhi_chan_state;

	switch (cmd) {
	case MHI_COMMAND_NOOP:
	{
		ring_el_type = MHI_PKT_TYPE_NOOP_CMD;
		break;
	}
	case MHI_COMMAND_RESET_CHAN:
	{
		to_state = MHI_CHAN_STATE_DISABLED;
		MHI_TRB_SET_INFO(EV_TRB_TYPE,
				pkt_loc,
				MHI_PKT_TYPE_RESET_CHAN_CMD);
		break;
	}
	case MHI_COMMAND_START_CHAN:
	{
		switch (from_state) {
		case MHI_CHAN_STATE_ENABLED:
		case MHI_CHAN_STATE_STOP:
			to_state = MHI_CHAN_STATE_RUNNING;
			break;
		default:
			mhi_log(MHI_MSG_ERROR,
				"Invalid state transition for "
				"cmd 0x%x, from_state 0x%x\n",
				cmd, from_state);
			goto error_general;
		}
		ring_el_type = MHI_PKT_TYPE_START_CHAN_CMD;
		break;
	}
	case MHI_COMMAND_STOP_CHAN:
	{
		switch (from_state) {
		case MHI_CHAN_STATE_RUNNING:
		case MHI_CHAN_STATE_SUSPENDED:
			to_state = MHI_CHAN_STATE_STOP;
			break;
		default:
			mhi_log(MHI_MSG_ERROR,
				"Invalid state transition for "
				"cmd 0x%x, from_state 0x%x\n",
					cmd, from_state);
			goto error_general;
		}
		ring_el_type = MHI_PKT_TYPE_STOP_CHAN_CMD;
		break;
	}
	default:
		mhi_log(MHI_MSG_ERROR, "Bad command received\n");
	}

	cmd_mutex = &device->mhi_cmd_mutex_list[PRIMARY_CMD_RING];
	if (MHI_STATUS_SUCCESS != mhi_acquire_mutex(cmd_mutex)) {
		mhi_log(MHI_MSG_ERROR, "Could not acquire cmd mutex\n");
		goto error_cmd_mutex;
	}

	if (MHI_STATUS_SUCCESS != add_element(device->mhi_local_cmd_ctxt,
						(void *)&pkt_loc)) {
		mhi_log(MHI_MSG_ERROR, "Failed to insert element\n");
		goto error_general;
	}
	chan_mutex = &device->mhi_chan_mutex[chan];
	if (MHI_COMMAND_NOOP != cmd) {
		if (MHI_STATUS_SUCCESS != mhi_acquire_mutex(chan_mutex)) {
			mhi_log(MHI_MSG_ERROR,
				"Could not acquire chan mutex: 0x%x",
				chan);
			goto error_general;
		}
		MHI_TRB_SET_INFO(CMD_TRB_TYPE, pkt_loc, ring_el_type);
		MHI_TRB_SET_INFO(CMD_TRB_CHID, pkt_loc, chan);
		device->mhi_local_chan_ctxt[chan].mhi_chan_state = to_state;
		if (MHI_STATUS_SUCCESS != mhi_release_mutex(chan_mutex)) {
			mhi_log(MHI_MSG_ERROR,
				"Could not release chan mutex: 0x%x",
				chan);
			goto error_general;
		}
	}
	db_value = mhi_v2p_addr(device->mhi_ctrl_seg_info,
				(uintptr_t)device->mhi_local_cmd_ctxt->wp);

	device->mhi_chan_pend_cmd_ack[chan] = MHI_CMD_PENDING;
	MHI_WRITE_DB(device->cmd_db_addr, 0, db_value);
	mhi_release_mutex(&device->mhi_cmd_mutex_list[PRIMARY_CMD_RING]);

	return MHI_STATUS_SUCCESS;

error_general:
	mhi_release_mutex(&device->mhi_cmd_mutex_list[PRIMARY_CMD_RING]);
error_cmd_mutex:
	return MHI_STATUS_ERROR;
}

/**
 * @brief Thread which handles inbound data for MHI clients.
 * This thread will invoke thecallback for the mhi clients to
 * inform thme of data availability.
 *
 * The thread monitors the MHI state variable to know if it should
 * continue processing, * or stop.
 *
 * @param ctxt void pointer to a device context
 */
MHI_STATUS parse_xfer_event(mhi_device_ctxt *ctxt, mhi_event_pkt *event)
{
	mhi_device_ctxt *device = (mhi_device_ctxt *)ctxt;
	mhi_result *result = NULL;
	u32 chan = MHI_MAX_CHANNELS;
	u16 xfer_len = 0;
	uintptr_t phy_ev_trb_loc = 0;
	mhi_xfer_pkt *local_ev_trb_loc = NULL;
	mhi_client_handle *client_handle = NULL;
	mhi_xfer_pkt *local_trb_loc = NULL;
	mhi_chan_ctxt *chan_ctxt = NULL;

	if (NULL == device) {
		mhi_log(MHI_MSG_ERROR, "Got bad context, quitting\n");
		return MHI_STATUS_ERROR;
	}
	switch (MHI_EV_READ_CODE(EV_TRB_CODE, event)) {
	case MHI_EVENT_CC_EOT:
	{
		void *trb_data_loc = NULL;
		u32 ieot_flag = 0;
		MHI_STATUS ret_val = 0;

		chan = MHI_EV_READ_CHID(EV_CHID, event);
		xfer_len = MHI_EV_READ_LEN(EV_LEN, event);
		phy_ev_trb_loc = MHI_EV_READ_PTR(EV_PTR, event);

		if (!VALID_CHAN_NR(chan)) {
			mhi_log(MHI_MSG_ERROR, "Bad ring id.\n");
			break;
		}
		chan_ctxt = &device->mhi_ctrl_seg->mhi_cc_list[chan];
		ret_val = validate_xfer_el_addr(chan_ctxt,
						phy_ev_trb_loc);

		if (MHI_STATUS_SUCCESS != ret_val) {
			mhi_log(MHI_MSG_ERROR, "Bad event trb ptr.\n");
			break;
		}

		/* Get the TRB this event points to*/
		local_ev_trb_loc =
			(mhi_xfer_pkt *)mhi_p2v_addr(device->mhi_ctrl_seg_info,
							phy_ev_trb_loc);
		local_trb_loc =
			(mhi_xfer_pkt *)device->mhi_local_chan_ctxt[chan].rp;

		MHI_TRB_GET_INFO(TX_TRB_IEOT, local_trb_loc, ieot_flag);
		trb_data_loc = (void *)(uintptr_t)(local_trb_loc->data_tx_pkt.buffer_ptr);
		client_handle = device->client_handle_list[chan];

		if (NULL != client_handle) {
			result = &device->client_handle_list[chan]->result;
			result->payload_buf = trb_data_loc;
			result->bytes_xferd = xfer_len;
			result->bytes_xferd = MHI_TX_TRB_GET_LEN(TX_TRB_LEN,
								local_trb_loc);
			result->transaction_status = MHI_STATUS_SUCCESS;
			result->user_data =
				device->client_handle_list[chan]->user_data;
		}
		if (chan % 2)
			parse_inbound(device, chan,
					local_ev_trb_loc, xfer_len);
		else
			parse_outbound(device, chan,
					local_ev_trb_loc, xfer_len);
		break;
	} /* CC_EOT */
	default:
		{
			mhi_log(MHI_MSG_ERROR,
				"Unknown TX completion.\n");
			break;
		}
	} /*switch(MHI_EV_READ_CODE(EV_TRB_CODE,event)) */
	return 0;
}

MHI_STATUS recycle_trb_and_ring(mhi_device_ctxt *device,
		mhi_ring *ring,
		MHI_RING_TYPE ring_type,
		u32 ring_index)
{
	MHI_STATUS ret_val = MHI_STATUS_ERROR;
	u64 db_value = 0;
	void *removed_element = NULL;
	void *added_element = NULL;

	/* TODO This will not cover us for ring_index out of
	 * bounds for cmd or event channels */
	if (NULL == device || NULL == ring ||
		ring_type > (MHI_RING_TYPE_MAX - 1) ||
		ring_index > (MHI_MAX_CHANNELS - 1)) {

		mhi_log(MHI_MSG_ERROR, "Bad input params\n");
		return ret_val;
	}
	ret_val = delete_element(ring, &removed_element);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_ERROR, "Could not remove element from ring\n");
		return MHI_STATUS_ERROR;
	}
	ret_val = add_element(ring, &added_element);
	if (MHI_STATUS_SUCCESS != ret_val)
		mhi_log(MHI_MSG_ERROR, "Could not add element to ring\n");
	db_value = mhi_v2p_addr(device->mhi_ctrl_seg_info,
				(uintptr_t)ring->wp);
	if (MHI_STATUS_SUCCESS != ret_val)
		return ret_val;
	if (MHI_RING_TYPE_XFER_RING == ring_type)
	{
		mhi_xfer_pkt *removed_xfer_pkt =
			(mhi_xfer_pkt *)removed_element;
		mhi_xfer_pkt *added_xfer_pkt =
			(mhi_xfer_pkt *)added_element;
		added_xfer_pkt->data_tx_pkt =
				*(mhi_tx_pkt *)removed_xfer_pkt;
	}
	atomic_inc(&device->data_pending);
	if (MHI_STATE_M0 == device->mhi_state ||
	    MHI_STATE_M1 == device->mhi_state) {
		switch (ring_type) {
		case MHI_RING_TYPE_CMD_RING:
			MHI_WRITE_DB(device->cmd_db_addr,
					ring_index, db_value);
			break;
		case MHI_RING_TYPE_EVENT_RING:
			MHI_WRITE_DB(device->event_db_addr,
					ring_index, db_value);
			break;
		case MHI_RING_TYPE_XFER_RING:
		{
			MHI_WRITE_DB(device->channel_db_addr,
					ring_index, db_value);
			break;
		}
		default:
			mhi_log(MHI_MSG_ERROR, "Bad ring type\n");
		}
	}
	if (0 == atomic_sub_return(1, &device->data_pending))
	{
		wake_up(&device->mhi_xfer_stop->event);
	}
	return ret_val;
}
MHI_STATUS mhi_change_chan_state(mhi_device_ctxt *device, u32 chan_id,
				MHI_CHAN_STATE new_state)
{
	osal_mutex *chan_mutex = &device->mhi_chan_mutex[chan_id];
	if (chan_id > (MHI_MAX_CHANNELS - 1) || NULL == device ||
		new_state > MHI_CHAN_STATE_LIMIT) {
		mhi_log(MHI_MSG_ERROR, "Bad input parameters\n");
		return MHI_STATUS_ERROR;
	}

	if (MHI_STATUS_SUCCESS != mhi_acquire_mutex(chan_mutex)) {
		mhi_log(MHI_MSG_ERROR,
			"Failed to get chan 0x%x mutex", chan_id);
		return MHI_STATUS_ERROR;
	}

	/* Set the new state of the channel context */
	device->mhi_ctrl_seg->mhi_cc_list[chan_id].mhi_chan_state =
						MHI_CHAN_STATE_ENABLED;
	if (MHI_STATUS_SUCCESS != mhi_release_mutex(chan_mutex)) {
		mhi_log(MHI_MSG_ERROR,
				"Failed release mutex of chan 0x%x", chan_id);
		return MHI_STATUS_ERROR;
	}

	return MHI_STATUS_SUCCESS;
}

MHI_STATUS parse_cmd_event(mhi_device_ctxt *device, mhi_event_pkt *ev_pkt)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_cmd_pkt *cmd_pkt = NULL;
	if (NULL == ev_pkt)
		/*TODO: Validate this pointer */
		cmd_pkt = (mhi_cmd_pkt *)(uintptr_t)MHI_EV_READ_PTR(EV_PTR,
								ev_pkt);
	mhi_log(MHI_MSG_INFO, "Received CMD completion event\n");
	switch (MHI_EV_READ_CODE(EV_TRB_CODE, ev_pkt)) {
		/* Command completion was successful */
	case MHI_EVENT_CC_SUCCESS:
	{
		switch (MHI_TRB_READ_INFO(CMD_TRB_TYPE, cmd_pkt)) {
		case MHI_PKT_TYPE_NOOP_CMD:
			mhi_log(MHI_MSG_INFO, "Processed NOOP cmd event\n");
			break;
		case MHI_PKT_TYPE_RESET_CHAN_CMD:
			if (MHI_STATUS_SUCCESS != reset_chan_cmd(device,
								ev_pkt))
				mhi_log(MHI_MSG_INFO,
					"Failed to process reset cmd\n");
		break;
		case MHI_PKT_TYPE_STOP_CHAN_CMD:
		{
			u32 chan = MHI_EV_READ_CHID(EV_CHID, ev_pkt);
			mhi_log(MHI_MSG_INFO, "Processed cmd stop event\n");
			if (MHI_STATUS_SUCCESS != ret_val) {
				mhi_log(MHI_MSG_INFO,
						"Failed to set chan state\n");
				return MHI_STATUS_ERROR;
			}
			device->mhi_chan_pend_cmd_ack[chan] =
							MHI_CMD_NOT_PENDING;
			break;
		}
		case MHI_PKT_TYPE_START_CHAN_CMD:
		{
			if (MHI_STATUS_SUCCESS != start_chan_cmd(device,
								ev_pkt))
				mhi_log(MHI_MSG_INFO,
					"Failed to process reset cmd\n");
			break;
		}
		default:
			mhi_log(MHI_MSG_INFO,
				"Bad cmd type 0x%x\n",
				MHI_TRB_READ_INFO(CMD_TRB_TYPE, cmd_pkt));
			break;
		}
		mhi_log(MHI_MSG_INFO, "CMD completion indicated successful\n");
		break;
	}
	default:
		mhi_log(MHI_MSG_INFO, "Unhandled mhi completion code\n");
		break;
	}
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS reset_chan_cmd(mhi_device_ctxt *device, mhi_event_pkt *ev_pkt)
{
	u32 chan = MHI_EV_READ_CHID(EV_CHID, ev_pkt);
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;

	mhi_log(MHI_MSG_INFO, "Processed cmd reset event\n");
	device->mhi_local_chan_ctxt[chan].rp =
				device->mhi_local_chan_ctxt[chan].base;
	device->mhi_local_chan_ctxt[chan].wp =
				device->mhi_local_chan_ctxt[chan].base;
	device->mhi_ctrl_seg->mhi_cc_list[chan].mhi_chan_state =
						MHI_CHAN_STATE_ENABLED;
	device->mhi_chan_pend_cmd_ack[chan] = MHI_CMD_NOT_PENDING;
	if (MHI_STATUS_SUCCESS != mhi_send_cmd(device,
					MHI_COMMAND_START_CHAN, chan))
		mhi_log(MHI_MSG_CRITICAL,
				"Failed to start chan after reset\n");
	return ret_val;
}
MHI_STATUS start_chan_cmd(mhi_device_ctxt *device, mhi_event_pkt *ev_pkt)
{
	u32 chan = MHI_EV_READ_CHID(EV_CHID, ev_pkt);
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	if (!VALID_CHAN_NR(chan))
		mhi_log(MHI_MSG_ERROR, "Bad chan:%d\n", chan);

	mhi_log(MHI_MSG_INFO, "Processed cmd channel start\n");
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_INFO, "Failed to set chan state\n");
		return MHI_STATUS_ERROR;
	}
	if (MHI_CMD_RESET_PENDING == device->mhi_chan_pend_cmd_ack[chan])
		device->mhi_chan_pend_cmd_ack[chan] = MHI_CMD_NOT_PENDING;
	return ret_val;
}
void assert_device_wake()
{}

void mhi_poll_inbound(mhi_client_handle *client_handle,
			uintptr_t *buf, size_t *buf_size)
{
	mhi_tx_pkt *pending_trb = 0;
	mhi_device_ctxt *mhi_dev_ctxt = NULL;
	u32 chan = 0;
	mhi_ring *local_chan_ctxt;

	if (NULL == client_handle || NULL == buf || 0 == buf_size ||
			NULL == client_handle->mhi_dev_ctxt)
		return;

	mhi_dev_ctxt = client_handle->mhi_dev_ctxt;
	chan = client_handle->chan;
	local_chan_ctxt = &mhi_dev_ctxt->mhi_local_chan_ctxt[chan];

	if (atomic_read(&local_chan_ctxt->nr_filled_elements) > 0) {
		pending_trb = (mhi_tx_pkt *)(local_chan_ctxt->rp);
		*buf = (uintptr_t)(pending_trb->buffer_ptr);
		*buf_size = (size_t)MHI_TX_TRB_GET_LEN(TX_TRB_LEN,
						(mhi_xfer_pkt *)pending_trb);
		atomic_dec(&local_chan_ctxt->nr_filled_elements);
	} else {
		*buf = 0;
		*buf_size = 0;
	}
}

MHI_STATUS mhi_client_recycle_trb(mhi_client_handle *client_handle)
{
	u32 chan = client_handle->chan;
	mhi_xfer_pkt *pending_trb = NULL;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_device_ctxt *device = client_handle->mhi_dev_ctxt;
	osal_mutex *chan_mutex  = &device->mhi_chan_mutex[chan];
	mhi_ring *local_ctxt = NULL;
	local_ctxt = &client_handle->mhi_dev_ctxt->mhi_local_chan_ctxt[chan];

	if (MHI_STATUS_SUCCESS != mhi_acquire_mutex(chan_mutex))
		mhi_log(MHI_MSG_ERROR, "Could not acquire mutex\n");

	pending_trb = (mhi_xfer_pkt *)local_ctxt->rp;
	MHI_TX_TRB_SET_LEN(TX_TRB_LEN, pending_trb, TRB_MAX_DATA_SIZE);
	ret_val = recycle_trb_and_ring(client_handle->mhi_dev_ctxt,
			local_ctxt,
			MHI_RING_TYPE_XFER_RING,
			chan);
	if (MHI_STATUS_SUCCESS != mhi_release_mutex(chan_mutex))
		mhi_log(MHI_MSG_ERROR, "Failed to release channel mutex.\n");
	return ret_val;
}

MHI_STATUS validate_xfer_el_addr(mhi_chan_ctxt *ring, uintptr_t addr)
{
	return (addr < (ring->mhi_trb_ring_base_addr) ||
			addr > (ring->mhi_trb_ring_base_addr)
			+ (ring->mhi_trb_ring_len - 1)) ?
		MHI_STATUS_ERROR : MHI_STATUS_SUCCESS;
}
MHI_STATUS validate_ev_el_addr(mhi_ring *ring, uintptr_t addr)
{
	return (addr < (uintptr_t)(ring->base) ||
			addr > ((uintptr_t)(ring->base)
				+ (ring->len - 1))) ?
		MHI_STATUS_ERROR : MHI_STATUS_SUCCESS;
}

MHI_STATUS validate_ring_el_addr(mhi_ring *ring, uintptr_t addr)
{
	return (addr < (uintptr_t)(ring->base) ||
		addr > ((uintptr_t)(ring->base)
			+ (ring->len - 1))) ?
		MHI_STATUS_ERROR : MHI_STATUS_SUCCESS;
}
MHI_STATUS parse_inbound(mhi_device_ctxt *device, u32 chan,
			mhi_xfer_pkt *local_ev_trb_loc,u16 xfer_len)
{
	mhi_client_handle *client_handle = NULL;
	mhi_ring *local_chan_ctxt = NULL;
	mhi_result *result = NULL;

	client_handle = device->client_handle_list[chan];
	local_chan_ctxt = &device->mhi_local_chan_ctxt[chan];
	if (NULL != device->client_handle_list[chan])
		result = &device->client_handle_list[chan]->result;

	/* If a client is registered */
	if (NULL != client_handle) {
		if (IS_SOFTWARE_CHANNEL(chan)) {
				/*Set the length field of the trb to the
				*length reported in the event */
				MHI_TX_TRB_SET_LEN(TX_TRB_LEN,
					local_ev_trb_loc,
					xfer_len);
				atomic_inc(&(local_chan_ctxt->nr_filled_elements));
			/* If a cb is registered for the client, invoke it*/
			if (NULL != (client_handle->client_cbs.mhi_xfer_cb))
				client_handle->client_cbs.mhi_xfer_cb(result);

		} else  {
			/* IN Hardware channel with client
			 * registered, we are done with this TRB*/
			delete_element(local_chan_ctxt, NULL);
		}
	} else  {
		/* A client is not registred for this IN channel */
		if (IS_SOFTWARE_CHANNEL(chan)) {
			/* Set the length field of the trb to the
			 * length reported in the event */
			MHI_TX_TRB_SET_LEN(TX_TRB_LEN,
					local_ev_trb_loc,
					xfer_len);
			atomic_inc(&(local_chan_ctxt->nr_filled_elements));
		} else  {/* Hardware Channel, no client registerered,
				drop data */
			recycle_trb_and_ring(device,
					&device->mhi_local_chan_ctxt[chan],
					MHI_RING_TYPE_XFER_RING,
					chan);
		}
	}
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS parse_outbound(mhi_device_ctxt *device, u32 chan,
			mhi_xfer_pkt *local_ev_trb_loc, u16 xfer_len)
{
	mhi_result *result = NULL;
	mhi_client_handle *client_handle = NULL;

	client_handle = device->client_handle_list[chan];
	if (NULL != client_handle)
		result = &device->client_handle_list[chan]->result;

	if (NULL != client_handle &&
			NULL != (&client_handle->client_cbs.mhi_xfer_cb))
		client_handle->client_cbs.mhi_xfer_cb(result);

	MHI_OSAL_ASSERT(MHI_STATUS_SUCCESS ==
			delete_element(&device->mhi_local_chan_ctxt[chan],
					NULL));
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS probe_clients(mhi_device_ctxt *device)
{
	int ret_val = 0;
	ret_val = rmnet_mhi_probe(device->dev_info->pcie_device);
	if (0 != ret_val)
	{
		mhi_log(MHI_MSG_CRITICAL, "MHI Rmnet Failed to probe.\n");
		goto error;
	}
	ret_val = mhi_shim_probe(device->dev_info->pcie_device);
	if (0 != ret_val)
		mhi_log(MHI_MSG_CRITICAL, "MHI Shim Failed to probe.\n");
error:
	return ret_val;
}
