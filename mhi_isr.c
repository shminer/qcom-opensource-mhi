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
irq_handler_t irq_cb(int msi_number, void *dev_id)
{
	struct device *mhi_device =((struct device *)dev_id);
	mhi_device_ctxt *device = NULL;
	mhi_event_pkt *device_ev_rp;
	mhi_event_pkt *local_ev_rp;
	mhi_client_handle* client_handle;
	MHI_EVENT_POLLING ev_poll_en;
	u32 i = 0;

	device = *(mhi_device_ctxt **)(mhi_device->platform_data);
	if (NULL == device) {
		mhi_log(MHI_MSG_ERROR, "Failed to get a proper context\n");
		return (irq_handler_t)IRQ_HANDLED;
	}
	mhi_log(MHI_MSG_VERBOSE, "Got ISR.\n");

	for (i = 0; i < EVENT_RINGS_ALLOCATED; ++i) {
		MHI_GET_EVENT_RING_INFO(EVENT_RING_POLLING,
					device->ev_ring_props[i],ev_poll_en)
		if (!ev_poll_en) {
			/* We have to know if there is an event pending for the hw inbound chan */
			device_ev_rp = (mhi_event_pkt *)mhi_p2v_addr(device->mhi_ctrl_seg_info,
					(u64)(device->mhi_ctrl_seg->mhi_ec_list[device->alloced_ev_rings[i]].mhi_event_read_ptr));
			local_ev_rp = (mhi_event_pkt *)device->mhi_local_event_ctxt[device->alloced_ev_rings[i]].rp;

			if (device_ev_rp != local_ev_rp) {
				mhi_log(MHI_MSG_INFO,
					"Triggering NAPI.\n");
				mhi_log(MHI_MSG_INFO, "alloced ev rings 0x%x, i 0x%d.\n",device->alloced_ev_rings[i],i);
				client_handle =
					device->client_handle_list[device->alloced_ev_rings[i]];
				mhi_log(MHI_MSG_INFO, "Client handle is %p.\n",client_handle);
				if (NULL != client_handle) {
					(client_handle->result).user_data = client_handle->user_data;
					if(NULL != &client_handle->client_cbs.mhi_xfer_cb)
						client_handle->client_cbs.mhi_xfer_cb(&client_handle->result);
				}
			}
			break; /* We queued NAPI, we are done */
		}
	}
	atomic_inc(&device->events_pending);
	mhi_trigger_event(device->event_handle);
	return (irq_handler_t)IRQ_HANDLED;
}

int parse_event_thread(void *ctxt)
{
	mhi_device_ctxt *device = (mhi_device_ctxt *)ctxt;
	u32 ev_ring_index = 0;
	u32 ev_poll_en = 0;

	/* Go through all event rings */
	for (;;) {
		if (0 == wait_event_interruptible(device->event_handle->event,
					(atomic_read(&device->events_pending) > 0) ||
					device->kill_threads)) {
			if (device->kill_threads) {
				mhi_log(MHI_MSG_INFO, "Caught exit signal, quitting\n");
				device->event_thread_state =
						MHI_THREAD_STATE_EXIT;
				return 0;
			}
		}
		atomic_dec(&device->events_pending);

		mhi_log(MHI_MSG_VERBOSE,
			"Polling index.\n");

		for (ev_ring_index = 0;
			ev_ring_index < EVENT_RINGS_ALLOCATED;
			++ev_ring_index) {

			MHI_GET_EVENT_RING_INFO(EVENT_RING_POLLING,
					device->ev_ring_props[ev_ring_index],
					ev_poll_en)
				if (ev_poll_en){
					mhi_log(MHI_MSG_VERBOSE,
						"Polling index. %d\n",
						device->alloced_ev_rings[ev_ring_index]);
					mhi_process_event_ring(device,
							device->alloced_ev_rings[ev_ring_index],
							EV_EL_PER_RING);
				}
		}
	}
	return 0;
}

MHI_STATUS mhi_process_event_ring(mhi_device_ctxt *device,
					u32 ev_index,
					u32 event_quota)
{
	mhi_event_pkt *local_rp = NULL;
	mhi_event_pkt *device_rp = NULL;
	mhi_event_pkt event_to_process;
	osal_spinlock *event_spinlock = NULL;
	event_spinlock = &device->mhi_ev_spinlock_list[ev_index];

	if (MHI_STATUS_ERROR == mhi_acquire_spinlock(event_spinlock))
	{
		mhi_log(MHI_MSG_ERROR,
			"Failed get event mutex, for ring 0x%x\n",
			ev_index);
		return MHI_STATUS_ERROR;
	}
	device_rp = (mhi_event_pkt *)mhi_p2v_addr(device->mhi_ctrl_seg_info,
			(u64)(device->mhi_ctrl_seg->mhi_ec_list[ev_index].mhi_event_read_ptr));
	local_rp = (mhi_event_pkt *)device->mhi_local_event_ctxt[ev_index].rp;

		mhi_log(MHI_MSG_ERROR,
			"Local rp %p device rp %p\n", local_rp, device_rp);

	if (MHI_STATUS_SUCCESS != validate_ev_el_addr(&device->mhi_local_event_ctxt[ev_index],
				(uintptr_t)device_rp))
		mhi_log(MHI_MSG_ERROR,
			"Failed to validate event ring element 0x%p\n",
			device_rp);

		mhi_log(MHI_MSG_ERROR,
			"Local rp %p device rp %p\n", local_rp, device_rp);
	while ((local_rp != device_rp) && (event_quota > 0) && 
		(device_rp != NULL) && (local_rp != NULL)) {
		event_to_process = *local_rp;
		if (MHI_STATUS_SUCCESS != recycle_trb_and_ring(device,
					&device->mhi_local_event_ctxt[ev_index],
					MHI_RING_TYPE_EVENT_RING,
					ev_index))
			mhi_log(MHI_MSG_ERROR, "Failed to recycle ev pkt\n");
		switch (MHI_TRB_READ_INFO(EV_TRB_TYPE, (&event_to_process))) {
		case MHI_PKT_TYPE_CMD_COMPLETION_EVENT:
		{
			mhi_log(MHI_MSG_INFO,
				"MHI CCE received ring 0x%x\n",
				ev_index);
			parse_cmd_event(device,
					&event_to_process);
			break;
		}
		case MHI_PKT_TYPE_TX_EVENT:
		{
			parse_xfer_event(device,
					&event_to_process);
			break;
		}
		case MHI_PKT_TYPE_STATE_CHANGE_EVENT:
		{
			MHI_STATE_TRANSITION new_state;
			new_state = MHI_READ_STATE(&event_to_process);
			mhi_log(MHI_MSG_INFO,
				"MHI STE received ring 0x%x\n",
				ev_index);
			mhi_init_state_transition(device, new_state);
			break;
		}
		default:
		{
			mhi_log(MHI_MSG_ERROR,
				"Unsupported packet type code 0x%x\n",
					MHI_TRB_READ_INFO(EV_TRB_TYPE,
							&event_to_process));
			break;
		}
		} /*switch(mhi_trb_read_info(EV_TRB_TYPE,ev_pkt)*/

		local_rp = (mhi_event_pkt *)device->mhi_local_event_ctxt[ev_index].rp;
		device_rp = (mhi_event_pkt *)mhi_p2v_addr(device->mhi_ctrl_seg_info,
				(u64)(device->mhi_ctrl_seg->mhi_ec_list[ev_index].mhi_event_read_ptr));
		--event_quota;
	} /* while (local_rp != .. */
	mhi_release_spinlock(event_spinlock);
	return MHI_STATUS_SUCCESS;
}

/*
 * @brief Initialize a callback structure and register for interrupt
 */
/* Link the device context to the msi handle */
mhi_result *mhi_poll(mhi_client_handle *client_handle)
{
	client_handle->result.payload_buf = 0;
	client_handle->result.bytes_xferd = 0;
	if (MHI_STATUS_SUCCESS != mhi_process_event_ring(client_handle->mhi_dev_ctxt,
				client_handle->event_ring_index,
				1))
		mhi_log(MHI_MSG_INFO, "NAPI failed to process event ring\n");
	return &(client_handle->result);
}

void mhi_mask_irq(mhi_client_handle *client_handle)
{
	/* Stubbed until MSI masking is enabled */
}
void mhi_unmask_irq(mhi_client_handle *client_handle)
{
	/* Stubbed until MSI masking is enabled */
}
