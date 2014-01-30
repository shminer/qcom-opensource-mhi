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
#include "mhi_hwio.h"

/**
 * @brief Initialize a full MHI reset procedure. The call first sends the MDM a
 *        reset signal. The RX and CMD threads are then stopped, all data and
 *        ctrl segments are cleared and MMIO is re-initialized.
 *
 * @param device [IN ] Context of the device on which to perform the MHI reset
 *        procedure
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_reset(mhi_device_ctxt *device)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	u32 thread_wait_for_sleep_timeout = 0;
	u32 i = 0;

	if (device == NULL)
		return MHI_STATUS_ERROR;

	for (i = 0; i < MHI_MAX_CHANNELS; ++i) {
		/* Stop all clients from transfering data on the outbound
		 * channels */
		ret_val = mhi_set_state_of_all_channels(device,
							MHI_CHAN_STATE_ERROR);
		if (MHI_STATUS_SUCCESS != ret_val) {
			mhi_log(MHI_MSG_CRITICAL,
			      "Failed to set state of all channels to error\n");
			return ret_val;
		}
	}
	/* At this point no clients can send data,initialize the device reset */
	MHI_REG_WRITE_FIELD(device->mmio_addr, MHICTRL, MHICTRL_MHISTATE_MASK,
			    MHICTRL_MHISTATE_SHIFT, MHI_STATE_RESET);

	mhi_log(MHI_MSG_INFO, "Waiting for RX and CMD threads to stop\n");

	/* 2 Wait for a period of time to stop the RX and CMD threads and clear
	 * their work queue */
	while (MHI_THREAD_STATE_SUSPENDED != device->event_thread_state &&
			0 == thread_wait_for_sleep_timeout) {
		if (0 == thread_wait_for_sleep_timeout) {
			mhi_sleep(MHI_THREAD_SLEEP_TIMEOUT_MS);
			thread_wait_for_sleep_timeout = 1;
		} else {
			mhi_log(MHI_MSG_INFO,
			  "Threads failed to stop within the timeout period\n");
			return MHI_STATUS_ERROR;
		}
	}

	/* We need to clear the work queue of our state change thread as well
	 * to avoid processing any repeated reset requests
	 *
	 * TODO: It may not be necessary to acquire the mutex here, since at
	 *       this point we should be the only thread of execution running
	 *       that could access this work queue */
	mhi_acquire_mutex(device->state_change_work_item_list.q_mutex);
	mhi_init_state_change_thread_work_queue
					 (&device->state_change_work_item_list);
	mhi_release_mutex(device->state_change_work_item_list.q_mutex);

	/* 3. Reset all rings */
	mhi_log(MHI_MSG_INFO, "Resetting all rings..\n");
	mhi_init_contexts(device);

	return ret_val;
}

/**
 * @brief Thread handling all mhi state change transitions`
 *	 This thread processes state transition elements from its work queue.
 *	 Elements are added to the work queue by other threads upon detection of
 *       a possible error condition requring an mhi reset. Elements are also
 *       added to the work queue by the state change thread itself, when
 *       undergoing a multi state transition.
 *
 * @param ctxt mhi device context
 */
int mhi_state_change_thread(void *ctxt)
{
	mhi_device_ctxt *device = (mhi_device_ctxt *)ctxt;
	u32 empty_ring = 1;
	mhi_state_work_item cur_work_item;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;

	mhi_ring *state_change_q = &device->state_change_work_item_list.q_info;

	if (NULL == device) {
		mhi_log(MHI_MSG_ERROR, "Got bad context, quitting\n");
		return -EIO;
	}
	for (;;)	{
		/* 1. Wait for signal from ISR
		 *    mhi_wait_for_event
		 *    (device->state_change_event_handle,0xffffFFFF); */
		wait_event_interruptible(
				device->state_change_event_handle->event,
				state_change_q->rp != state_change_q->wp ||
				device->kill_threads);
		do {
			if (device->kill_threads) {
				mhi_log(MHI_MSG_INFO,
					"Caught exit signal, quitting\n");
				device->state_change_thread_state =
							MHI_THREAD_STATE_EXIT;
				return 0;
			}
			mhi_acquire_mutex
				(device->state_change_work_item_list.q_mutex);
			/* TODO: May need to be removed */
			if (device->state_change_work_item_list.q_info.rp ==
			    device->state_change_work_item_list.q_info.wp) {
				empty_ring = 1;
				mhi_release_mutex
				 (device->state_change_work_item_list.q_mutex);
			} else {
				empty_ring = 0;
				cur_work_item = *(mhi_state_work_item *)(device->state_change_work_item_list.q_info.rp);
				ret_val = delete_element(
				&device->state_change_work_item_list.q_info,
					NULL);
				mhi_release_mutex(
				device->state_change_work_item_list.q_mutex);
				process_stt_work_item(device, &cur_work_item);
				MHI_OSAL_ASSERT(ret_val == MHI_STATUS_SUCCESS);
			}
		} while (!empty_ring);
		empty_ring = 0;
	}
	return 0;
}
/**
 * @brief Reset for a single MHI channel
 *
 * @param device [IN ] context
 * @param chan_id [IN ] channel id of the channel to reset
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_reset_channel(mhi_device_ctxt *device, u32 chan_id)
{
	MHI_STATUS	 ret_val;
	mhi_chan_ctxt *cur_ctxt = NULL;
	mhi_ring *cur_ring = NULL;

	if (chan_id > (MHI_MAX_CHANNELS - 1) || NULL == device) {
		mhi_log(MHI_MSG_ERROR, "Bad input parameters\n");
		return MHI_STATUS_ERROR;
	}

	if (MHI_STATUS_SUCCESS !=
		mhi_acquire_mutex(&device->mhi_chan_mutex[chan_id])) {
		mhi_log(MHI_MSG_ERROR, "Fail to get chan 0x%x mutex", chan_id);
		return MHI_STATUS_ERROR;
	}

	/* We need to reset the channel completley, we will assume that our
	 * base is correct*/
	cur_ctxt = &device->mhi_ctrl_seg->mhi_cc_list[chan_id];
	cur_ring = &device->mhi_local_event_ctxt[chan_id];
	mhi_memset(cur_ring->base, 0, sizeof(char)*cur_ring->len);

	if (IS_HARDWARE_CHANNEL(chan_id%2)) {
		ret_val = mhi_init_chan_ctxt(cur_ctxt,
				mhi_v2p_addr(device->mhi_ctrl_seg_info,
					     (uintptr_t)cur_ring->base),
					     (uintptr_t)cur_ring->base,
					     MAX_NR_TRBS_PER_HARD_CHAN,
					     (chan_id % 2) ? MHI_IN : MHI_OUT,
		      (chan_id % 2) ? SECONDARY_EVENT_RING : PRIMARY_EVENT_RING,
					     cur_ring);
	} else {
		ret_val = mhi_init_chan_ctxt(cur_ctxt,
				mhi_v2p_addr(device->mhi_ctrl_seg_info,
					     (uintptr_t)cur_ring->base),
					     (uintptr_t)cur_ring->base,
					     MAX_NR_TRBS_PER_SOFT_CHAN,
					     (chan_id % 2) ? MHI_IN : MHI_OUT,
					     PRIMARY_EVENT_RING,
					     cur_ring);
	}

	if (MHI_STATUS_SUCCESS != ret_val)
		mhi_log(MHI_MSG_ERROR, "Failed to reset chan ctxt\n");


	if (MHI_STATUS_SUCCESS !=
		mhi_release_mutex(&device->mhi_chan_mutex[chan_id])) {
		return MHI_STATUS_ERROR;
	}
	return ret_val;
}

/**
 * @brief Add a new state transition work item to the state transition
 *        thread work item list.
 *
 * @param device [IN ]	The device context
 * @param new_state	The state we wish to transition to
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_state_transition(mhi_device_ctxt *device,
		MHI_STATE_TRANSITION new_state)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_state_work_item *cur_work_item = NULL;
	s32 nr_avail_work_items = 0;
	mhi_ring *stt_ring = &device->state_change_work_item_list.q_info;

	/* if (NULL == device || new_state >= STATE_TRANSITION_MAX)
	 *      return MHI_STATUS_ERROR; */

	nr_avail_work_items = get_nr_avail_ring_elements(stt_ring);

	if (0 >= nr_avail_work_items) {
		mhi_log(MHI_MSG_CRITICAL, "No Room left on STT work queue\n");
		return MHI_STATUS_ERROR;
	}
	cur_work_item = stt_ring->wp;
	/* TODO Evaluate that state transition is legal */
	cur_work_item->new_state = new_state;
	ret_val = add_element(&device->state_change_work_item_list.q_info,
			      (void **)&cur_work_item);

	MHI_OSAL_ASSERT(MHI_STATUS_SUCCESS == ret_val);
	ret_val = mhi_trigger_event(device->state_change_event_handle);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_ERROR,
			"Failed to signal state transition task\n");
	}
	return ret_val;
}

/**
 * @brief Set the state of all channels. This function is normallu called when
 *        there is a MHI state change. Warning:Calling this function can be
 *        potentially very expensive, a mutex for all channels has to be
 *        acquired to change the state.
 *
 * @param device [IN ] Context
 * @param new_state [IN ] new_state transition
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_set_state_of_all_channels(mhi_device_ctxt *device,
					 MHI_CHAN_STATE new_state)
{
	u32 i = 0;
	if (new_state >= MHI_CHAN_STATE_LIMIT)
		return MHI_STATUS_ERROR;
	/* TODO Validate that state transition is legal */
	for (i = 0; i < MHI_MAX_CHANNELS; ++i) {
		/* mhi_acquire_mutex(&device->mhi_chan_mutex[i]); */
		device->mhi_local_chan_ctxt[i].mhi_chan_state = new_state;
		/* mhi_release_mutex(&device->mhi_chan_mutex[i]); */
	}
	return MHI_STATUS_SUCCESS;
}



MHI_STATUS process_stt_work_item(mhi_device_ctxt  *device,
			mhi_state_work_item *cur_work_item)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;

	mhi_log(MHI_MSG_INFO,"Transitioning to %d\n",
					(int)cur_work_item->new_state);
	switch (cur_work_item->new_state) {
	case STATE_TRANSITION_RESET:
		ret_val = process_RESET_transition(device, cur_work_item);
		break;
	/*MHI is transitioning into READY state, no transfers can occur */
	case STATE_TRANSITION_READY:
		ret_val = process_READY_transition(device, cur_work_item);
		break;
	case STATE_TRANSITION_M0:
		ret_val = process_M0_transition(device, cur_work_item);
		break;
	case STATE_TRANSITION_M1:
		ret_val = process_M1_transition(device, cur_work_item);
		break;
	case STATE_TRANSITION_M2:
		device->mhi_state = MHI_STATE_M2;
		ret_val = MHI_STATUS_SUCCESS;
		break;
	case STATE_TRANSITION_M3:
			ret_val = process_M3_transition(device, cur_work_item);
			break;
	case STATE_TRANSITION_SYS_ERR:
		ret_val = process_SYSERR_transition(device, cur_work_item);
		break;
	case STATE_TRANSITION_BHI:
		ret_val = process_BHI_transition(device, cur_work_item);
		break;
	default:
		mhi_log(MHI_MSG_ERROR, "Unrecongized state change work item :%d\n",
					cur_work_item->new_state);
		break;
	}
	return ret_val;
}
MHI_STATUS process_M0_transition(mhi_device_ctxt *device,
			mhi_state_work_item *cur_work_item)
{
	u32 i = 0;
	u32 device_state_change = 0;
	u32 pcie_word_val = 0;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_log(MHI_MSG_INFO, "Processing M0 state transition\n");

	/* Wait for device transition to M0 */
	device_state_change = 0;
	while (!device_state_change) {
		if (++i >= 10) {
			mhi_log(MHI_MSG_ERROR,
					"Time out! STATUS register not M0.\n");
			ret_val = MHI_STATUS_DEVICE_NOT_READY;
			break;
		} else {
			pcie_word_val = pcie_read(device->mmio_addr,
							MHISTATUS);
			MHI_READ_FIELD(pcie_word_val,
					MHISTATUS_MHISTATE_MASK,
					MHISTATUS_MHISTATE_SHIFT);
			if (pcie_word_val == MHI_STATE_M0) {
				device_state_change = 1;
			} else {
				mhi_log(MHI_MSG_ERROR,
						"STATUS register not M0\n");
				mhi_sleep(1000);
			}
		}
		/* Proxy vote to prevent M3 while we are ringing DBs */
		atomic_inc(&device->data_pending);
		if (device->mhi_state == MHI_STATE_M2)
			device->m2_m0++;
		else if (device->mhi_state == MHI_STATE_M3)
			device->m3_m0++;
		device->mhi_state = MHI_STATE_M0;
		if (0 == device->mhi_initialized) {
			ret_val = mhi_add_elements_to_event_rings(device,
						cur_work_item->new_state);
			if (MHI_STATUS_SUCCESS != ret_val)
				return MHI_STATUS_ERROR;
			device->mhi_initialized = 1;
			ret_val = mhi_set_state_of_all_channels(device,
					MHI_CHAN_STATE_RUNNING);
			if (MHI_STATUS_SUCCESS != ret_val)
				mhi_log(MHI_MSG_CRITICAL,
						"Failed to set local chan state\n");
			ret_val = probe_clients(device);
			if (ret_val != MHI_STATUS_SUCCESS)
				mhi_log(MHI_MSG_CRITICAL,
						"Failed to probe MHI CORE clients.\n");
		} else {
			ring_all_chan_dbs(device);
		}
		if (0 == atomic_sub_return(1, &device->data_pending))
			wake_up(&device->mhi_xfer_stop->event);
	}
	wake_up(&device->M0_event->event);
	return MHI_STATUS_SUCCESS;
}

void ring_all_chan_dbs(mhi_device_ctxt* device)
{
	u32 i = 0;
	u64 db_value;
	u64 rp;
	for (i = 0; i < MHI_MAX_CHANNELS; ++i)
		if (VALID_CHAN_NR(i))
		{
			rp = mhi_v2p_addr(device->mhi_ctrl_seg_info,
				(uintptr_t)device->mhi_local_chan_ctxt[i].rp);
			db_value = mhi_v2p_addr(device->mhi_ctrl_seg_info,
				(uintptr_t)device->mhi_local_chan_ctxt[i].wp);
			if (rp != db_value)
			{
				atomic_set(&device->mhi_chan_db_order[i], 0);
				if (1 == atomic_add_return(1,
						&device->mhi_chan_db_order[i]))

					mhi_acquire_spinlock(&device->db_write_lock[i]);
					db_value = mhi_v2p_addr(device->mhi_ctrl_seg_info,
						(uintptr_t)device->mhi_local_chan_ctxt[i].wp);
					MHI_WRITE_DB(device->channel_db_addr,
							i, db_value);
					mhi_release_spinlock(&device->db_write_lock[i]);
			}
		}
}

MHI_STATUS process_M1_transition(mhi_device_ctxt  *device,
		mhi_state_work_item *cur_work_item)
{
	mhi_log(MHI_MSG_INFO,
			"Processing M1 state transition, previous device state %d\n",
			device->mhi_state);
	mhi_log(MHI_MSG_INFO, "Allowing transition to M2\n");
	device->m0_m1++;
	device->mhi_state = MHI_STATE_M2;
	MHI_REG_WRITE_FIELD(device->mmio_addr, MHICTRL,
			MHICTRL_MHISTATE_MASK,
			MHICTRL_MHISTATE_SHIFT,
			MHI_STATE_M2);
	device->m1_m2++;

	return MHI_STATUS_SUCCESS;
}
MHI_STATUS process_BHI_transition(mhi_device_ctxt *device,
			mhi_state_work_item *cur_work_item)
{
	device->mhi_state = MHI_STATE_BHI;
	mhi_log(MHI_MSG_INFO, "Processing BHI state transition\n");
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS process_READY_transition(mhi_device_ctxt *device,
			mhi_state_work_item *cur_work_item)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	/*TODO: Check if the transition into this state is legal.*/
	mhi_log(MHI_MSG_INFO, "Processing READY state transition\n");

	ret_val = mhi_reset_all_thread_queues(device);

	if (MHI_STATUS_SUCCESS != ret_val)
		mhi_log(MHI_MSG_ERROR,
			"Processing READY state transition\n");

	/* Initialize MMIO */
	if (MHI_STATUS_SUCCESS != mhi_init_mmio(device)) {
		mhi_log(MHI_MSG_ERROR,
			"Failure during MMIO initialization\n");
		return MHI_STATUS_ERROR;
	}
	ret_val = mhi_add_elements_to_event_rings(device,
				cur_work_item->new_state);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_ERROR,
			"Failure during event ring init\n");
		return MHI_STATUS_ERROR;
	}
	MHI_REG_WRITE_FIELD(device->mmio_addr, MHICTRL,
			MHICTRL_MHISTATE_MASK,
			MHICTRL_MHISTATE_SHIFT,
			MHI_STATE_M0);
	return MHI_STATUS_SUCCESS;
}
MHI_STATUS process_RESET_transition(mhi_device_ctxt *device,
			mhi_state_work_item *cur_work_item)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_log(MHI_MSG_INFO, "Processing RESET state transition\n");
	/*TODO: Validate that this state transition is in fact, legal */
	/* The device transitions MHI out of this state,
	 * by letting us know it is READY */
	ret_val = mhi_test_for_device_ready(device);
	/* Poll on the READY bit in MMIO */
	switch (ret_val) {
		case MHI_STATUS_SUCCESS:
			device->mhi_state = MHI_STATE_READY;
			if (MHI_STATUS_SUCCESS != mhi_init_state_transition(device,
						STATE_TRANSITION_READY))

			mhi_log(MHI_MSG_CRITICAL,
			"Failed to initiate 0x%x state trans\n",
			STATE_TRANSITION_READY);
		break;
	case MHI_STATUS_LINK_DOWN:
		mhi_log(MHI_MSG_CRITICAL, "Link down detected\n");
		break;
	case MHI_STATUS_DEVICE_NOT_READY:
		if (MHI_STATUS_SUCCESS != mhi_init_state_transition(device,
					STATE_TRANSITION_RESET))
			mhi_log(MHI_MSG_CRITICAL,
				"Failed to initiate 0x%x state trans\n",
				STATE_TRANSITION_RESET);
		break;
	default:
		mhi_log(MHI_MSG_CRITICAL,
			"Unexpected ret code detected for\n");
		break;
	}
	return ret_val;
}
MHI_STATUS process_SYSERR_transition(mhi_device_ctxt *device,
			mhi_state_work_item *cur_work_item)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_log(MHI_MSG_CRITICAL, "Received SYS ERROR. Resetting MHI\n");
	ret_val = mhi_reset(device);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_CRITICAL, "Failed to reset mhi\n");
		return MHI_STATUS_ERROR;
	}
	device->mhi_state = MHI_STATE_RESET;
	if (MHI_STATUS_SUCCESS != mhi_init_state_transition(device,
				STATE_TRANSITION_RESET))
		mhi_log(MHI_MSG_ERROR,
			"Failed to init state transition to RESET.\n");
	return ret_val;
}

MHI_STATUS process_M3_transition(mhi_device_ctxt *device,
		mhi_state_work_item *cur_work_item)
{
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"M3 state transition received 0x%x\n",
			cur_work_item->new_state);
	device->mhi_state = MHI_STATE_M3;
	device->pending_M3 = 0;
	device->m0_m3++;
	wake_up(&device->M3_event->event);

	return MHI_STATUS_SUCCESS;
}
