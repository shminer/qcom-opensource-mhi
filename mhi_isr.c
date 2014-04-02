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
#include "mhi_sys.h"

/* @brief Actual MSI callback running in separate thread */
irqreturn_t irq_cb(int irq_number, void *dev_id)
{
	struct device *mhi_device = ((struct device *)dev_id);
	u32 client_index;
	mhi_device_ctxt *mhi_dev_ctxt =
		*(mhi_device_ctxt **)(mhi_device->platform_data);
	mhi_client_handle *client_handle;
	mhi_client_info_t *client_info;

	if (NULL == mhi_dev_ctxt) {
		mhi_log(MHI_MSG_ERROR, "Failed to get a proper context\n");
		return IRQ_HANDLED;
	}
	mhi_dev_ctxt->msi_counter[IRQ_TO_MSI(mhi_dev_ctxt, irq_number)]++;
	mhi_log(MHI_MSG_VERBOSE,
			"Got MSI 0x%x\n",IRQ_TO_MSI(mhi_dev_ctxt, irq_number));
	switch (IRQ_TO_MSI(mhi_dev_ctxt, irq_number)) {
	case 0:
	case 1:
		atomic_inc(&mhi_dev_ctxt->events_pending);
		wake_up_interruptible(mhi_dev_ctxt->event_handle);
		break;
	case 2:
	{
		mhi_log(MHI_MSG_VERBOSE, "NAPI interrupt received\n");
		client_index =
			mhi_dev_ctxt->alloced_ev_rings[TERTIARY_EVENT_RING];
		client_handle = mhi_dev_ctxt->client_handle_list[client_index];
		client_info = &client_handle->client_info;

		if (NULL != client_handle) {
			(client_handle->result).user_data =
					client_handle->user_data;
			if (NULL != &client_info->mhi_xfer_cb)
				client_info->mhi_xfer_cb(&client_handle->result);
		}
		break;
	}
	}
	return IRQ_HANDLED;
}

int parse_event_thread(void *ctxt)
{
	mhi_device_ctxt *mhi_dev_ctxt = (mhi_device_ctxt *)ctxt;
	u32 i = 0;
	u32 ev_poll_en = 0;
	int ret_val = 0;

	/* Go through all event rings */
	for (;;) {
		ret_val =
			wait_event_interruptible(*mhi_dev_ctxt->event_handle,
			(atomic_read(&mhi_dev_ctxt->events_pending) > 0) ||
			mhi_dev_ctxt->kill_threads);

		if (0 == ret_val) {
			if (mhi_dev_ctxt->kill_threads) {
				mhi_log(MHI_MSG_INFO,
					"Caught exit signal, quitting\n");
				mhi_dev_ctxt->event_thread_state =
					MHI_THREAD_STATE_EXIT;
				return 0;
			}
		}
		atomic_dec(&mhi_dev_ctxt->events_pending);

		for (i = 0; i < EVENT_RINGS_ALLOCATED; ++i) {
			MHI_GET_EVENT_RING_INFO(EVENT_RING_POLLING,
					mhi_dev_ctxt->ev_ring_props[i],
					ev_poll_en)
			if (ev_poll_en) {
				mhi_process_event_ring(mhi_dev_ctxt,
				 mhi_dev_ctxt->alloced_ev_rings[i],
				 EV_EL_PER_RING);
			}
		}
	}
	return 0;
}

MHI_STATUS mhi_process_event_ring(mhi_device_ctxt *mhi_dev_ctxt,
		u32 ev_index,
		u32 event_quota)
{
	mhi_event_pkt *local_rp = NULL;
	mhi_event_pkt *device_rp = NULL;
	mhi_event_pkt event_to_process;
	mhi_event_ctxt *ev_ctxt = NULL;
	mhi_ring *local_ev_ctxt = &mhi_dev_ctxt->mhi_local_event_ctxt[ev_index];

	ev_ctxt = &mhi_dev_ctxt->mhi_ctrl_seg->mhi_ec_list[ev_index];

	device_rp =
		(mhi_event_pkt *)mhi_p2v_addr(mhi_dev_ctxt->mhi_ctrl_seg_info,
						ev_ctxt->mhi_event_read_ptr);
	local_rp = (mhi_event_pkt *)local_ev_ctxt->rp;


	if (MHI_STATUS_SUCCESS != validate_ev_el_addr(local_ev_ctxt,
				(uintptr_t)device_rp))
		mhi_log(MHI_MSG_ERROR,
				"Failed to validate event ring element 0x%p\n",
				device_rp);


	while ((local_rp != device_rp) && (event_quota > 0) &&
			(device_rp != NULL) && (local_rp != NULL)) {
		event_to_process = *local_rp;
		if (MHI_STATUS_SUCCESS != recycle_trb_and_ring(mhi_dev_ctxt,
						local_ev_ctxt,
						MHI_RING_TYPE_EVENT_RING,
						ev_index))
			mhi_log(MHI_MSG_ERROR, "Failed to recycle ev pkt\n");
		switch (MHI_TRB_READ_INFO(EV_TRB_TYPE, (&event_to_process))) {
		case MHI_PKT_TYPE_CMD_COMPLETION_EVENT:
			mhi_log(MHI_MSG_INFO,
					"MHI CCE received ring 0x%x\n",
					ev_index);
			parse_cmd_event(mhi_dev_ctxt,
					&event_to_process);
			break;
		case MHI_PKT_TYPE_TX_EVENT:
			parse_xfer_event(mhi_dev_ctxt, &event_to_process);
			break;
		case MHI_PKT_TYPE_STATE_CHANGE_EVENT:
		{
			MHI_STATE_TRANSITION new_state;
			new_state = MHI_READ_STATE(&event_to_process);
			mhi_log(MHI_MSG_INFO,
					"MHI STE received ring 0x%x\n",
					ev_index);
			mhi_init_state_transition(mhi_dev_ctxt, new_state);
			break;
		}
		case MHI_PKT_TYPE_EE_EVENT:
		{
			MHI_STATE_TRANSITION new_state;
			switch(MHI_READ_EXEC_ENV(&event_to_process)) {
			case MHI_EXEC_ENV_SBL:
				new_state = STATE_TRANSITION_SBL;
				mhi_init_state_transition(mhi_dev_ctxt,
								new_state);
				break;
			case MHI_EXEC_ENV_AMSS:
				new_state = STATE_TRANSITION_AMSS;
				mhi_init_state_transition(mhi_dev_ctxt,
								new_state);
				break;
			}
				break;
		}
		default:
			mhi_log(MHI_MSG_ERROR,
				"Unsupported packet type code 0x%x\n",
				MHI_TRB_READ_INFO(EV_TRB_TYPE,
					&event_to_process));
			break;
		}
		local_rp = (mhi_event_pkt *)local_ev_ctxt->rp;
		device_rp = (mhi_event_pkt *)mhi_p2v_addr(
					mhi_dev_ctxt->mhi_ctrl_seg_info,
					(u64)ev_ctxt->mhi_event_read_ptr);
		--event_quota;
	}
	return MHI_STATUS_SUCCESS;
}

/*
 * @brief Initialize a callback structure and register for interrupt
 */
/* Link the mhi_dev_ctxt context to the msi handle */
mhi_result *mhi_poll(mhi_client_handle *client_handle)
{
	MHI_STATUS ret_val;
	client_handle->result.payload_buf = 0;
	client_handle->result.bytes_xferd = 0;
	ret_val = mhi_process_event_ring(client_handle->mhi_dev_ctxt,
				client_handle->event_ring_index,
				1);
	if (MHI_STATUS_SUCCESS != ret_val)
		mhi_log(MHI_MSG_INFO, "NAPI failed to process event ring\n");
	return &(client_handle->result);
}

void mhi_mask_irq(mhi_client_handle *client_handle)
{
	mhi_log(MHI_MSG_VERBOSE, "Masking MSI 0x%x\n",
		client_handle->msi_vec);
	if (client_handle == NULL)
		return;
	disable_irq_nosync(MSI_TO_IRQ(client_handle->mhi_dev_ctxt,
					client_handle->msi_vec));
}
void mhi_unmask_irq(mhi_client_handle *client_handle)
{
	mhi_log(MHI_MSG_VERBOSE, "Unmasking MSI 0x%x\n",
			client_handle->msi_vec);
	if  (client_handle == NULL)
		return;
	enable_irq(MSI_TO_IRQ(client_handle->mhi_dev_ctxt,
			client_handle->msi_vec));
}
