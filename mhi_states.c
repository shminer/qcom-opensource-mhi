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

MHI_STATUS mhi_reset(mhi_device_ctxt *mhi_dev_ctxt)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	u32 thread_wait_for_sleep_timeout = 0;
	u32 i = 0;

	if (mhi_dev_ctxt == NULL)
		return MHI_STATUS_ERROR;

	for (i = 0; i < MHI_MAX_CHANNELS; ++i) {
		/* Stop all clients from transfering data on the outbound
		 * channels */
		ret_val = mhi_set_state_of_all_channels(mhi_dev_ctxt,
							MHI_CHAN_STATE_ERROR);
		if (MHI_STATUS_SUCCESS != ret_val) {
			mhi_log(MHI_MSG_CRITICAL,
			"Failed to set state of all channels to error\n");
			return ret_val;
		}
	}
	MHI_REG_WRITE_FIELD(mhi_dev_ctxt->mmio_addr, MHICTRL,
				MHICTRL_MHISTATE_MASK,
				MHICTRL_MHISTATE_SHIFT, MHI_STATE_RESET);

	mhi_log(MHI_MSG_INFO, "Waiting for RX and CMD threads to stop\n");

	while (0 == thread_wait_for_sleep_timeout &&
		MHI_THREAD_STATE_SUSPENDED !=
				mhi_dev_ctxt->event_thread_state) {
		if (0 == thread_wait_for_sleep_timeout) {
			msleep(MHI_THREAD_SLEEP_TIMEOUT_MS);
			thread_wait_for_sleep_timeout = 1;
		} else {
			mhi_log(MHI_MSG_INFO,
			  "Threads failed to stop.\n");
			return MHI_STATUS_ERROR;
		}
	}

	mutex_lock(mhi_dev_ctxt->state_change_work_item_list.q_mutex);
	mhi_init_state_change_thread_work_queue
				 (&mhi_dev_ctxt->state_change_work_item_list);
	mutex_unlock(mhi_dev_ctxt->state_change_work_item_list.q_mutex);

	mhi_log(MHI_MSG_INFO, "Resetting all rings..\n");
	mhi_init_contexts(mhi_dev_ctxt);

	return ret_val;
}

int mhi_state_change_thread(void *ctxt)
{
	mhi_device_ctxt *mhi_dev_ctxt = (mhi_device_ctxt *)ctxt;
	mhi_state_work_item cur_work_item;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	u32 empty_ring = 0;
	mhi_state_work_queue *work_q =
			&mhi_dev_ctxt->state_change_work_item_list;

	mhi_ring *state_change_q = &work_q->q_info;

	if (NULL == mhi_dev_ctxt) {
		mhi_log(MHI_MSG_ERROR, "Got bad context, quitting\n");
		return -EIO;
	}
	for (;;)	{
		wait_event_interruptible(
				*mhi_dev_ctxt->state_change_event_handle,
				state_change_q->rp != state_change_q->wp ||
				mhi_dev_ctxt->kill_threads);
		do {
			if (mhi_dev_ctxt->kill_threads) {
				mhi_log(MHI_MSG_INFO,
					"Caught exit signal, quitting\n");
				mhi_dev_ctxt->state_change_thread_state =
							MHI_THREAD_STATE_EXIT;
				return 0;
			}
			mutex_lock(work_q->q_mutex);
			if (work_q->q_info.rp == work_q->q_info.wp) {
				mutex_unlock(work_q->q_mutex);
				empty_ring = 1;
			} else {
				empty_ring = 0;
				cur_work_item =
				   *(mhi_state_work_item *)(state_change_q->rp);
				ret_val = ctxt_del_element(
				&work_q->q_info,
					NULL);
				mutex_unlock(work_q->q_mutex);
				ret_val =
					process_stt_work_item(mhi_dev_ctxt,
								&cur_work_item);
				MHI_ASSERT(ret_val == MHI_STATUS_SUCCESS);
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
MHI_STATUS mhi_reset_channel(mhi_client_handle *client_handle)
{
	MHI_STATUS ret_val;
	mhi_chan_ctxt *cur_ctxt = NULL;
	mhi_device_ctxt *mhi_dev_ctxt = NULL;
	u32 chan_id = 0;
	mhi_ring *cur_ring = NULL;

	chan_id = client_handle->chan;
	mhi_dev_ctxt = client_handle->mhi_dev_ctxt;

	if (chan_id > (MHI_MAX_CHANNELS - 1) || NULL == mhi_dev_ctxt) {
		mhi_log(MHI_MSG_ERROR, "Bad input parameters\n");
		return MHI_STATUS_ERROR;
	}

	mutex_lock(&mhi_dev_ctxt->mhi_chan_mutex[chan_id]);

	/* We need to reset the channel completley, we will assume that our
	 * base is correct*/
	cur_ctxt = &mhi_dev_ctxt->mhi_ctrl_seg->mhi_cc_list[chan_id];
	cur_ring = &mhi_dev_ctxt->mhi_local_event_ctxt[chan_id];
	memset(cur_ring->base, 0, sizeof(char)*cur_ring->len);

	if (IS_HARDWARE_CHANNEL(chan_id%2)) {
		ret_val = mhi_init_chan_ctxt(cur_ctxt,
				mhi_v2p_addr(mhi_dev_ctxt->mhi_ctrl_seg_info,
					     (uintptr_t)cur_ring->base),
					     (uintptr_t)cur_ring->base,
					     MAX_NR_TRBS_PER_HARD_CHAN,
					     (chan_id % 2) ? MHI_IN : MHI_OUT,
		      (chan_id % 2) ? SECONDARY_EVENT_RING : PRIMARY_EVENT_RING,
					     cur_ring);
	} else {
		ret_val = mhi_init_chan_ctxt(cur_ctxt,
				mhi_v2p_addr(mhi_dev_ctxt->mhi_ctrl_seg_info,
					     (uintptr_t)cur_ring->base),
					     (uintptr_t)cur_ring->base,
					     MAX_NR_TRBS_PER_SOFT_CHAN,
					     (chan_id % 2) ? MHI_IN : MHI_OUT,
					     PRIMARY_EVENT_RING,
					     cur_ring);
	}

	if (MHI_STATUS_SUCCESS != ret_val)
		mhi_log(MHI_MSG_ERROR, "Failed to reset chan ctxt\n");


	mutex_unlock(&mhi_dev_ctxt->mhi_chan_mutex[chan_id]);
	return ret_val;
}

/**
 * @brief Add a new state transition work item to the state transition
 *        thread work item list.
 *
 * @param mhi_dev_ctxt [IN ]	The mhi_dev_ctxt context
 * @param new_state	The state we wish to transition to
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_state_transition(mhi_device_ctxt *mhi_dev_ctxt,
		MHI_STATE_TRANSITION new_state)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_state_work_item *cur_work_item = NULL;
	s32 nr_avail_work_items = 0;
	mhi_ring *stt_ring =
		&mhi_dev_ctxt->state_change_work_item_list.q_info;

	nr_avail_work_items = get_nr_avail_ring_elements(stt_ring);

	if (0 >= nr_avail_work_items) {
		mhi_log(MHI_MSG_CRITICAL, "No Room left on STT work queue\n");
		return MHI_STATUS_ERROR;
	}
	mhi_log(MHI_MSG_VERBOSE,
		"Processing state transition %x\n",
		new_state);
	cur_work_item = stt_ring->wp;
	cur_work_item->new_state = new_state;
	ret_val = ctxt_add_element(stt_ring, (void **)&cur_work_item);
	wmb();
	MHI_ASSERT(MHI_STATUS_SUCCESS == ret_val);
	wake_up_interruptible(mhi_dev_ctxt->state_change_event_handle);
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
MHI_STATUS mhi_set_state_of_all_channels(mhi_device_ctxt *mhi_dev_ctxt,
					 MHI_CHAN_STATE new_state)
{
	u32 i = 0;
	mhi_chan_ctxt *chan_ctxt = NULL;
	if (new_state >= MHI_CHAN_STATE_LIMIT)
		return MHI_STATUS_ERROR;
	for (i = 0; i < MHI_MAX_CHANNELS; ++i) {
		chan_ctxt = &mhi_dev_ctxt->mhi_ctrl_seg->mhi_cc_list[i];
		chan_ctxt->mhi_chan_state = new_state;
	}
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS process_stt_work_item(mhi_device_ctxt  *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;

	mhi_log(MHI_MSG_INFO, "Transitioning to %d\n",
				(int)cur_work_item->new_state);
	switch (cur_work_item->new_state) {
	case STATE_TRANSITION_BHI:
		ret_val = process_BHI_transition(mhi_dev_ctxt, cur_work_item);
		break;
	case STATE_TRANSITION_RESET:
		ret_val = process_RESET_transition(mhi_dev_ctxt, cur_work_item);
		break;
	case STATE_TRANSITION_READY:
		ret_val = process_READY_transition(mhi_dev_ctxt, cur_work_item);
		break;
	case STATE_TRANSITION_SBL:
		ret_val = process_SBL_transition(mhi_dev_ctxt, cur_work_item);
		break;
	case STATE_TRANSITION_AMSS:
		ret_val = process_AMSS_transition(mhi_dev_ctxt, cur_work_item);
		break;
	case STATE_TRANSITION_M0:
		ret_val = process_M0_transition(mhi_dev_ctxt, cur_work_item);
		break;
	case STATE_TRANSITION_M1:
		ret_val = process_M1_transition(mhi_dev_ctxt, cur_work_item);
		break;
	case STATE_TRANSITION_M3:
		ret_val = process_M3_transition(mhi_dev_ctxt, cur_work_item);
		break;
	case STATE_TRANSITION_SYS_ERR:
		ret_val = process_SYSERR_transition(mhi_dev_ctxt,
						   cur_work_item);
		break;
	default:
		mhi_log(MHI_MSG_ERROR,
				"Unrecongized state change work item :%d\n",
				cur_work_item->new_state);
		break;
	}
	return ret_val;
}
MHI_STATUS process_M0_transition(mhi_device_ctxt *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item)
{
	u32 i = 0;
	u32 device_state_change = 0;
	u32 pcie_word_val = 0;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	unsigned long flags;
	mhi_log(MHI_MSG_INFO, "Processing M0 state transition\n");

	/* Wait for mhi_dev_ctxt transition to M0 */
	device_state_change = 0;
	while (!device_state_change) {
		if (++i >= 10) {
			mhi_log(MHI_MSG_ERROR,
					"Time out! STATUS register not M0.\n");
			ret_val = MHI_STATUS_DEVICE_NOT_READY;
			break;
		} else {
			pcie_word_val = pcie_read(mhi_dev_ctxt->mmio_addr,
							MHISTATUS);
			MHI_READ_FIELD(pcie_word_val,
					MHISTATUS_MHISTATE_MASK,
					MHISTATUS_MHISTATE_SHIFT);
			if (pcie_word_val == MHI_STATE_M0) {
				device_state_change = 1;
			} else {
				mhi_log(MHI_MSG_ERROR,
					"STATUS register not M0, is 0x%x retry %d\n",
					i, pcie_word_val);
				msleep(1000);
			}
		}
		/* Proxy vote to prevent M3 while we are ringing DBs */
		if (mhi_dev_ctxt->mhi_state == MHI_STATE_M2)
			mhi_dev_ctxt->m2_m0++;
		else if (mhi_dev_ctxt->mhi_state == MHI_STATE_M3)
			mhi_dev_ctxt->m3_m0++;
		mhi_dev_ctxt->mhi_state = MHI_STATE_M0;
	}
	if (mhi_dev_ctxt->mhi_initialized) {
		/* Bump up the vote for pending data */
		read_lock_irqsave(&mhi_dev_ctxt->xfer_lock, flags);
		atomic_inc(&mhi_dev_ctxt->data_pending);
		gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 1);
		read_unlock_irqrestore(&mhi_dev_ctxt->xfer_lock, flags);

		ring_all_ev_dbs(mhi_dev_ctxt);
		ring_all_chan_dbs(mhi_dev_ctxt);
		ring_all_cmd_dbs(mhi_dev_ctxt);
		atomic_dec(&mhi_dev_ctxt->data_pending);
	}
	if (mhi_dev_ctxt->mhi_initialized &&
	    mhi_dev_ctxt->pending_M3 == 0)
		gpio_direction_output(MHI_DEVICE_WAKE_GPIO, 0);
	wake_up_interruptible(mhi_dev_ctxt->M0_event);
	return MHI_STATUS_SUCCESS;
}

void ring_all_ev_dbs(mhi_device_ctxt *mhi_dev_ctxt)
{
	u32 i;
	u64 db_value = 0;
	u32 event_ring_index;
	mhi_event_ctxt *event_ctxt = NULL;
	mhi_control_seg *mhi_ctrl = NULL;
	spinlock_t *lock = NULL;
	mhi_ctrl = mhi_dev_ctxt->mhi_ctrl_seg;

	for (i = 0; i < EVENT_RINGS_ALLOCATED; ++i) {
		event_ring_index = mhi_dev_ctxt->alloced_ev_rings[i];
		lock = &mhi_dev_ctxt->mhi_ev_spinlock_list[event_ring_index];
		mhi_dev_ctxt->mhi_ev_db_order[event_ring_index] = 0;
		spin_lock(lock);
		event_ctxt = &mhi_ctrl->mhi_ec_list[event_ring_index];
		db_value = mhi_v2p_addr(mhi_dev_ctxt->mhi_ctrl_seg_info,
			(uintptr_t)mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index].wp);

		if (0 == mhi_dev_ctxt->mhi_ev_db_order[event_ring_index]) {
			MHI_WRITE_DB(mhi_dev_ctxt->event_db_addr,
				event_ring_index, db_value);
		}
		mhi_dev_ctxt->mhi_ev_db_order[event_ring_index] = 0;
		spin_unlock(lock);
	}
}
void ring_all_chan_dbs(mhi_device_ctxt *mhi_dev_ctxt)
{
	u32 i = 0;
	u64 db_value = 0;
	u64 rp = 0;
	mhi_ring *local_ctxt = NULL;
	mhi_log(MHI_MSG_VERBOSE, "Ringing chan dbs\n");
	for (i = 0; i < MHI_MAX_CHANNELS; ++i)
		if (VALID_CHAN_NR(i)) {
			local_ctxt = &mhi_dev_ctxt->mhi_local_chan_ctxt[i];
			rp = mhi_v2p_addr(mhi_dev_ctxt->mhi_ctrl_seg_info,
				(uintptr_t)local_ctxt->rp);
			db_value = mhi_v2p_addr(mhi_dev_ctxt->mhi_ctrl_seg_info,
				(uintptr_t)local_ctxt->wp);
			if ((rp != db_value) && (i % 2 == 0))
				conditional_chan_db_write(mhi_dev_ctxt, i);
			else
				conditional_chan_db_write(mhi_dev_ctxt, i);
		}
}

void ring_all_cmd_dbs(mhi_device_ctxt *mhi_dev_ctxt)
{
	struct mutex *cmd_mutex = NULL;
	u64 db_value;
	mhi_log(MHI_MSG_VERBOSE, "Ringing chan dbs\n");
	cmd_mutex = &mhi_dev_ctxt->mhi_cmd_mutex_list[PRIMARY_CMD_RING];
	/* Write the cmd ring */
	mhi_dev_ctxt->cmd_ring_order = 0;
	mutex_lock(cmd_mutex);
	db_value = mhi_v2p_addr(mhi_dev_ctxt->mhi_ctrl_seg_info,
			(uintptr_t)mhi_dev_ctxt->mhi_local_cmd_ctxt[0].wp);
	if (0 == mhi_dev_ctxt->cmd_ring_order)
		MHI_WRITE_DB(mhi_dev_ctxt->cmd_db_addr, 0, db_value);
	mhi_dev_ctxt->cmd_ring_order = 0;
	mutex_unlock(cmd_mutex);
}
void conditional_chan_db_write(mhi_device_ctxt *mhi_dev_ctxt, u32 chan)
{
	u64 db_value;
	mhi_dev_ctxt->mhi_chan_db_order[chan] = 0;
	spin_lock(&mhi_dev_ctxt->db_write_lock[chan]);
	if (0 == mhi_dev_ctxt->mhi_chan_db_order[chan]) {
		db_value = mhi_v2p_addr(mhi_dev_ctxt->mhi_ctrl_seg_info,
			(uintptr_t)mhi_dev_ctxt->mhi_local_chan_ctxt[chan].wp);
		MHI_WRITE_DB(mhi_dev_ctxt->channel_db_addr,
				chan, db_value);
	}
	mhi_dev_ctxt->mhi_chan_db_order[chan] = 0;
	spin_unlock(&mhi_dev_ctxt->db_write_lock[chan]);
}

MHI_STATUS process_M1_transition(mhi_device_ctxt  *mhi_dev_ctxt,
		mhi_state_work_item *cur_work_item)
{
	mhi_log(MHI_MSG_INFO,
			"Processing M1 state transition from state %d\n",
			mhi_dev_ctxt->mhi_state);
	mhi_log(MHI_MSG_INFO, "Allowing transition to M2\n");
	mhi_dev_ctxt->m0_m1++;
	mhi_dev_ctxt->mhi_state = MHI_STATE_M2;
	MHI_REG_WRITE_FIELD(mhi_dev_ctxt->mmio_addr, MHICTRL,
			MHICTRL_MHISTATE_MASK,
			MHICTRL_MHISTATE_SHIFT,
			MHI_STATE_M2);
	mhi_dev_ctxt->m1_m2++;

	return MHI_STATUS_SUCCESS;
}
MHI_STATUS process_BHI_transition(mhi_device_ctxt *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item)
{
	mhi_dev_ctxt->mhi_state = MHI_STATE_BHI;
	mhi_log(MHI_MSG_INFO, "Processing BHI state transition\n");
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS process_READY_transition(mhi_device_ctxt *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_log(MHI_MSG_INFO, "Processing READY state transition\n");

	ret_val = mhi_reset_all_thread_queues(mhi_dev_ctxt);

	if (MHI_STATUS_SUCCESS != ret_val)
		mhi_log(MHI_MSG_ERROR,
			"Failed to reset thread queues\n");

	/* Initialize MMIO */
	if (MHI_STATUS_SUCCESS != mhi_init_mmio(mhi_dev_ctxt)) {
		mhi_log(MHI_MSG_ERROR,
			"Failure during MMIO initialization\n");
		return MHI_STATUS_ERROR;
	}
	ret_val = mhi_add_elements_to_event_rings(mhi_dev_ctxt,
				cur_work_item->new_state);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_ERROR,
			"Failure during event ring init\n");
		return MHI_STATUS_ERROR;
	}
	MHI_REG_WRITE_FIELD(mhi_dev_ctxt->mmio_addr, MHICTRL,
			MHICTRL_MHISTATE_MASK,
			MHICTRL_MHISTATE_SHIFT,
			MHI_STATE_M0);
	return MHI_STATUS_SUCCESS;
}
MHI_STATUS process_RESET_transition(mhi_device_ctxt *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_log(MHI_MSG_INFO, "Processing RESET state transition\n");
	ret_val = mhi_test_for_device_ready(mhi_dev_ctxt);
	switch (ret_val) {
	case MHI_STATUS_SUCCESS:
		mhi_dev_ctxt->mhi_state = MHI_STATE_READY;
		ret_val = mhi_init_state_transition(mhi_dev_ctxt,
					STATE_TRANSITION_READY);
		if (MHI_STATUS_SUCCESS != ret_val)
			mhi_log(MHI_MSG_CRITICAL,
			"Failed to initiate 0x%x state trans\n",
			STATE_TRANSITION_READY);
	break;
	case MHI_STATUS_LINK_DOWN:
		mhi_log(MHI_MSG_CRITICAL, "Link down detected\n");
		break;
	case MHI_STATUS_DEVICE_NOT_READY:
		ret_val = mhi_init_state_transition(mhi_dev_ctxt,
					STATE_TRANSITION_RESET);
		if (MHI_STATUS_SUCCESS != ret_val)
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
MHI_STATUS process_SYSERR_transition(mhi_device_ctxt *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	mhi_log(MHI_MSG_CRITICAL, "Received SYS ERROR. Resetting MHI\n");
	ret_val = mhi_reset(mhi_dev_ctxt);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_CRITICAL, "Failed to reset mhi\n");
		return MHI_STATUS_ERROR;
	}
	mhi_dev_ctxt->mhi_state = MHI_STATE_RESET;
	if (MHI_STATUS_SUCCESS != mhi_init_state_transition(mhi_dev_ctxt,
				STATE_TRANSITION_RESET))
		mhi_log(MHI_MSG_ERROR,
			"Failed to init state transition to RESET.\n");
	return ret_val;
}

MHI_STATUS process_M3_transition(mhi_device_ctxt *mhi_dev_ctxt,
		mhi_state_work_item *cur_work_item)
{
	mhi_log(MHI_MSG_INFO | MHI_DBG_POWER,
			"Processing M3 state transition\n");
	mhi_dev_ctxt->mhi_state = MHI_STATE_M3;
	mhi_dev_ctxt->pending_M3 = 0;
	mhi_dev_ctxt->m0_m3++;
	wake_up_interruptible(mhi_dev_ctxt->M3_event);
	return MHI_STATUS_SUCCESS;
}
MHI_STATUS process_SBL_transition(mhi_device_ctxt *mhi_dev_ctxt,
				mhi_state_work_item *cur_work_item)
{
	MHI_STATUS ret_val;
	u32 chan;
	mhi_chan_ctxt *chan_ctxt;
	mhi_log(MHI_MSG_INFO, "Processing SBL state transition\n");
	for (chan = 0; chan <= MHI_CLIENT_SAHARA_IN; ++chan)
	{
		chan_ctxt =
			&mhi_dev_ctxt->mhi_ctrl_seg->mhi_cc_list[chan];
		if (MHI_CHAN_STATE_ENABLED == chan_ctxt->mhi_chan_state ) {
			ret_val = mhi_send_cmd(mhi_dev_ctxt,
						MHI_COMMAND_START_CHAN,
						chan);
			if (MHI_STATUS_SUCCESS != ret_val) {
				mhi_log(MHI_MSG_VERBOSE,
					"Starting Channel 0x%x \n", chan);
				mhi_log(MHI_MSG_CRITICAL,
					"Failed to start chan0x%x, ret 0x%x\n",
					chan, ret_val);
				return MHI_STATUS_ERROR;
			}
		}
	}
	return MHI_STATUS_SUCCESS;
}
MHI_STATUS process_AMSS_transition(mhi_device_ctxt *mhi_dev_ctxt,
				mhi_state_work_item *cur_work_item)
{
	MHI_STATUS ret_val;
	u32 chan;
	mhi_chan_ctxt *chan_ctxt;
	mhi_log(MHI_MSG_INFO, "Processing AMSS state transition\n");
	for (chan = 0; chan <= MHI_MAX_CHANNELS; ++chan) {
		if (VALID_CHAN_NR(chan)) {
			chan_ctxt =
				&mhi_dev_ctxt->mhi_ctrl_seg->mhi_cc_list[chan];
			if (MHI_CHAN_STATE_ENABLED ==
						chan_ctxt->mhi_chan_state) {
				mhi_log(MHI_MSG_VERBOSE,
					"Starting Channel 0x%x \n", chan);
				ret_val = mhi_send_cmd(mhi_dev_ctxt,
						MHI_COMMAND_START_CHAN,
						chan);
				if (MHI_STATUS_SUCCESS != ret_val) {
					mhi_log(MHI_MSG_CRITICAL,
					"Failed to start chan0x%x,0x%x\n",
					chan, ret_val);
					return MHI_STATUS_ERROR;
				} else {
					atomic_inc(
					&mhi_dev_ctxt->start_cmd_pending_ack);
				}
			}
		}
	}
	wait_event_interruptible(*mhi_dev_ctxt->chan_start_complete,
		atomic_read(&mhi_dev_ctxt->start_cmd_pending_ack) == 0);
	if (0 == mhi_dev_ctxt->mhi_initialized) {
		ret_val = mhi_add_elements_to_event_rings(mhi_dev_ctxt,
					cur_work_item->new_state);
		if (MHI_STATUS_SUCCESS != ret_val)
			return MHI_STATUS_ERROR;
		mhi_dev_ctxt->mhi_initialized = 1;
		ret_val = mhi_set_state_of_all_channels(mhi_dev_ctxt,
				MHI_CHAN_STATE_RUNNING);
		if (MHI_STATUS_SUCCESS != ret_val)
			mhi_log(MHI_MSG_CRITICAL,
				"Failed to set local chan state\n");
		ret_val = probe_clients(mhi_dev_ctxt);
		if (ret_val != MHI_STATUS_SUCCESS)
			mhi_log(MHI_MSG_CRITICAL,
				"Failed to probe MHI CORE clients.\n");
	}
	return MHI_STATUS_SUCCESS;
}
