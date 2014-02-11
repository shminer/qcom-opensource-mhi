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
#include "mhi.h"
#include "mhi_sys.h"
#include "mhi_hwio.h"

MHI_STATUS mhi_clean_init_stage(mhi_device_ctxt *mhi_dev_ctxt,
		MHI_INIT_ERROR_STAGE cleanup_stage)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	switch (cleanup_stage) {
		u32 wait_counter = 0;
	case MHI_INIT_ERROR_STAGE_UNWIND_ALL:
		mhi_dev_ctxt->kill_threads = 1;
		while (MHI_THREAD_STATE_EXIT !=
				mhi_dev_ctxt->event_thread_state &&
				MHI_THREAD_STATE_EXIT !=
				mhi_dev_ctxt->state_change_thread_state &&
				wait_counter <  MHI_MAX_LINK_RETRIES) {
			usleep(1000);
			wait_counter++;
		}
		if (wait_counter == MHI_MAX_LINK_RETRIES)
			mhi_log(MHI_MSG_ERROR, "Failed to stop threads\n");
	case MHI_INIT_ERROR_STAGE_DEVICE_CTRL:
		mhi_freememregion(mhi_dev_ctxt->mhi_ctrl_seg_info);
	case MHI_INIT_ERROR_STAGE_THREAD_QUEUES:
		kfree(mhi_dev_ctxt->state_change_thread_handle);
		kfree(mhi_dev_ctxt->event_thread_handle);
	case MHI_INIT_ERROR_STAGE_THREADS:
		kfree(mhi_dev_ctxt->event_handle);
		kfree(mhi_dev_ctxt->state_change_event_handle);
		kfree(mhi_dev_ctxt->M0_event);
	case MHI_INIT_ERROR_STAGE_EVENTS:
		kfree(mhi_dev_ctxt->mhi_ctrl_seg_info);
	case MHI_INIT_ERROR_STAGE_MEM_ZONES:
		kfree(mhi_dev_ctxt->mhi_cmd_mutex_list);
		kfree(mhi_dev_ctxt->mhi_chan_mutex);
		kfree(mhi_dev_ctxt->mhi_ev_spinlock_list);
	case MHI_INIT_ERROR_STAGE_SYNC:
		kfree(mhi_dev_ctxt);
		break;
	default:
		ret_val = MHI_STATUS_ERROR;
		break;
	}
	return ret_val;
}

/**
 * @brief Main initialization function for a mhi device context
 *	 All threads, events mutexes, mhi specific data structures
 *	 are initialized here
 *
 * @param dev_info [IN ] pcie device information structure to
 which this mhi context belongs
 * @param mhi_device [IN/OUT] reference to a mhi context to be populated
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_device_ctxt(mhi_pcie_dev_info *dev_info,
		mhi_device_ctxt **mhi_device)
{
	if (NULL == dev_info || NULL == mhi_device)
		return MHI_STATUS_ERROR;
	mhi_log(MHI_MSG_VERBOSE, "mhi_init_device_ctxt>Init MHI dev ctxt\n");

	if (MHI_STATUS_SUCCESS != mhi_create_ctxt(mhi_device)) {
		mhi_log(MHI_MSG_ERROR, "Failed to initialize mhi dev ctxt\n");
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != mhi_init_sync(*mhi_device)) {
		mhi_log(MHI_MSG_ERROR, "Failed to initialize mhi sync\n");
		mhi_clean_init_stage(*mhi_device, MHI_INIT_ERROR_STAGE_SYNC);
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != mhi_init_ctrl_zone(dev_info, *mhi_device)) {
		mhi_log(MHI_MSG_ERROR, "Failed to initialize  memory zones\n");
		mhi_clean_init_stage(*mhi_device,
					MHI_INIT_ERROR_STAGE_MEM_ZONES);
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != mhi_init_events(*mhi_device)) {
		mhi_log(MHI_MSG_ERROR, "Failed to initialize mhi events\n");
		mhi_clean_init_stage(*mhi_device, MHI_INIT_ERROR_STAGE_EVENTS);
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != mhi_init_threads(*mhi_device)) {
		mhi_log(MHI_MSG_ERROR, "Failed to initialize mhi threads\n");
		mhi_clean_init_stage(*mhi_device,
					MHI_INIT_ERROR_STAGE_THREADS);
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != mhi_reset_all_thread_queues(*mhi_device)) {
		mhi_log(MHI_MSG_ERROR, "Failed to initialize work queues\n");
		mhi_clean_init_stage(*mhi_device,
					MHI_INIT_ERROR_STAGE_THREAD_QUEUES);
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != mhi_init_device_ctrl(*mhi_device)) {
		mhi_log(MHI_MSG_ERROR, "Failed to initialize ctrl seg\n");
		mhi_clean_init_stage(*mhi_device,
					MHI_INIT_ERROR_STAGE_THREAD_QUEUES);
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != mhi_init_contexts(*mhi_device)) {
		mhi_log(MHI_MSG_ERROR, "Failed initializing contexts\n");
		mhi_clean_init_stage(*mhi_device,
					MHI_INIT_ERROR_STAGE_DEVICE_CTRL);
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != mhi_init_timers(*mhi_device)) {
		mhi_log(MHI_MSG_ERROR, "Failed initializing timers\n");
		mhi_clean_init_stage(*mhi_device,
					MHI_INIT_ERROR_STAGE_DEVICE_CTRL);
		return MHI_STATUS_ERROR;
	}
	if (MHI_STATUS_SUCCESS != mhi_spawn_threads(*mhi_device)) {
		mhi_log(MHI_MSG_ERROR, "mhi_init> Failed to spawn threads\n");
		return MHI_STATUS_ERROR;
	}
	(*mhi_device)->dev_info = dev_info;
	(*mhi_device)->dev_props = dev_info->core;

	return MHI_STATUS_SUCCESS;

}

/**
 * @brief Create the base structure for the mhi context
 *
 * @param mhi_device [IN/OUT] Double pointer to an mhi ctxt struct
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_create_ctxt(mhi_device_ctxt **mhi_device)
{
	*mhi_device = kmalloc(sizeof(mhi_device_ctxt), GFP_KERNEL);
	if (NULL == *mhi_device)
		return MHI_STATUS_ALLOC_ERROR;
	memset(*mhi_device, 0, sizeof(mhi_device_ctxt));
	(*mhi_device)->mhi_state = MHI_STATE_RESET;
	(*mhi_device)->nr_of_cc = MHI_MAX_CHANNELS;
	(*mhi_device)->nr_of_ec = EVENT_RINGS_ALLOCATED;
	(*mhi_device)->nr_of_cmdc = NR_OF_CMD_RINGS;

	(*mhi_device)->alloced_ev_rings[PRIMARY_EVENT_RING] = 0;
	(*mhi_device)->alloced_ev_rings[SECONDARY_EVENT_RING] =
						MHI_CLIENT_IP_HW_0_OUT;
	(*mhi_device)->alloced_ev_rings[TERTIARY_EVENT_RING] =
						MHI_CLIENT_IP_HW_0_IN;

	MHI_SET_EVENT_RING_INFO(EVENT_RING_POLLING,
			(*mhi_device)->ev_ring_props[PRIMARY_EVENT_RING],
			MHI_EVENT_POLLING_ENABLED);
	MHI_SET_EVENT_RING_INFO(EVENT_RING_POLLING,
			(*mhi_device)->ev_ring_props[SECONDARY_EVENT_RING],
			MHI_EVENT_POLLING_ENABLED);
	MHI_SET_EVENT_RING_INFO(EVENT_RING_POLLING,
			(*mhi_device)->ev_ring_props[TERTIARY_EVENT_RING],
			MHI_EVENT_POLLING_DISABLED);
	MHI_SET_EVENT_RING_INFO(EVENT_RING_MSI_VEC,
				(*mhi_device)->ev_ring_props[PRIMARY_EVENT_RING],
				0);
	MHI_SET_EVENT_RING_INFO(EVENT_RING_MSI_VEC,
				(*mhi_device)->ev_ring_props[SECONDARY_EVENT_RING],
				0);
	MHI_SET_EVENT_RING_INFO(EVENT_RING_MSI_VEC,
				(*mhi_device)->ev_ring_props[TERTIARY_EVENT_RING],
				1);
	return MHI_STATUS_SUCCESS;
}
/**
 * @brief Initialize all mutexes and spinlocks used by mhi
 *
 * @param mhi_dev_ctxt [IN ] mhi mhi_dev_ctxt context
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_sync(mhi_device_ctxt *mhi_dev_ctxt)
{
	u32 i = 0;

	mhi_dev_ctxt->mhi_ev_spinlock_list = kmalloc(sizeof(spinlock_t) *
							MHI_MAX_CHANNELS,
							GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->mhi_ev_spinlock_list)
		goto ev_mutex_free;
	mhi_dev_ctxt->mhi_chan_mutex = kmalloc(sizeof(struct mutex) *
						MHI_MAX_CHANNELS, GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->mhi_chan_mutex)
		goto chan_mutex_free;
	mhi_dev_ctxt->mhi_cmd_mutex_list = kmalloc(sizeof(struct mutex) *
						NR_OF_CMD_RINGS, GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->mhi_cmd_mutex_list)
		goto cmd_mutex_free;

	mhi_dev_ctxt->db_write_lock = kmalloc(sizeof(spinlock_t) *
						MHI_MAX_CHANNELS, GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->db_write_lock)
		goto db_write_lock_free;
	for (i = 0; i < mhi_dev_ctxt->nr_of_cc; ++i)
		mutex_init(&mhi_dev_ctxt->mhi_chan_mutex[i]);
	for (i = 0; i < MHI_MAX_CHANNELS; ++i)
		spin_lock_init(&mhi_dev_ctxt->mhi_ev_spinlock_list[i]);
	for (i = 0; i < mhi_dev_ctxt->nr_of_cmdc; ++i)
		mutex_init(&mhi_dev_ctxt->mhi_cmd_mutex_list[i]);
	for (i = 0; i < MHI_MAX_CHANNELS; ++i)
		spin_lock_init(&mhi_dev_ctxt->db_write_lock[i]);
	rwlock_init(&mhi_dev_ctxt->xfer_lock);
	return MHI_STATUS_SUCCESS;

db_write_lock_free:
	kfree(mhi_dev_ctxt->mhi_cmd_mutex_list);
cmd_mutex_free:
	kfree(mhi_dev_ctxt->mhi_chan_mutex);
chan_mutex_free:
	kfree(mhi_dev_ctxt->mhi_ev_spinlock_list);
ev_mutex_free:
	return MHI_STATUS_ALLOC_ERROR;
}

/**
 * @brief Initialize the memory zone structures which hold the virt/phys
 *	 mapping of mhi control and data segments.
 *
 *
 * @param mhi_dev_ctxt [IN ] mhi mhi_dev_ctxt context
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_ctrl_zone(mhi_pcie_dev_info *dev_info,
				mhi_device_ctxt *mhi_dev_ctxt)
{
	mhi_dev_ctxt->mhi_ctrl_seg_info = kmalloc(sizeof(mhi_meminfo),
							GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->mhi_ctrl_seg_info)
		return MHI_STATUS_ALLOC_ERROR;
	mhi_dev_ctxt->mhi_ctrl_seg_info->dev = &dev_info->pcie_device->dev;
	return MHI_STATUS_SUCCESS;
}

/**
 * @brief Initialize all mhi threads
 *
 * @param mhi_dev_ctxt [IN ] mhi mhi_dev_ctxt context
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_threads(mhi_device_ctxt *mhi_dev_ctxt)
{
	mhi_dev_ctxt->event_thread_handle = kmalloc(sizeof(osal_thread),
							GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->event_thread_handle) {
		mhi_log(MHI_MSG_ERROR, "Failed to init rx thread handle");
		return MHI_STATUS_ERROR;
	}
	mhi_dev_ctxt->state_change_thread_handle = kmalloc(sizeof(osal_thread),
								GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->state_change_thread_handle) {
		mhi_log(MHI_MSG_ERROR, "Failed to init STT handle");
		goto error_state_change_thread_handle_alloc;
	}

	return MHI_STATUS_SUCCESS;

error_state_change_thread_handle_alloc:
	kfree(mhi_dev_ctxt->event_thread_handle);
	return MHI_STATUS_ERROR;
}

/**
 * @brief Spawn all the MHI threads
 *
 * @param mhi_dev_ctxt mhi mhi_dev_ctxt context
 *
 * @return MHI_STATUS
 *
 */
MHI_STATUS mhi_spawn_threads(mhi_device_ctxt *mhi_dev_ctxt)
{
	/* Spawn the XFER thread */
	if (MHI_STATUS_SUCCESS != mhi_spawn_thread(mhi_dev_ctxt,
				parse_event_thread,
				mhi_dev_ctxt->event_thread_handle,
				"MHI_EV_THREAD")) {
		mhi_log(MHI_MSG_ERROR, "Failed to start xfer thread");
		return MHI_STATUS_ERROR;
	}

	/* Spawn the MST thread */
	if (MHI_STATUS_SUCCESS != mhi_spawn_thread(mhi_dev_ctxt,
				mhi_state_change_thread,
				mhi_dev_ctxt->state_change_thread_handle,
				"MHI_STATE_THREAD")) {
		mhi_log(MHI_MSG_ERROR, " Failed to start state thread");
		return MHI_STATUS_ERROR;
	}
	return MHI_STATUS_SUCCESS;
}
/**
 * @brief Initialize the event signals for the mhi threads
 *
 * @param mhi_dev_ctxt[IN ] Current mhi_dev_ctxt context
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_events(mhi_device_ctxt *mhi_dev_ctxt)
{

	mhi_dev_ctxt->event_handle = kmalloc(sizeof(wait_queue_head_t),
						GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->event_handle) {
		mhi_log(MHI_MSG_ERROR, "Failed to init event");
		return MHI_STATUS_ERROR;
	}
	mhi_dev_ctxt->state_change_event_handle =
				kmalloc(sizeof(wait_queue_head_t), GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->state_change_event_handle) {
		mhi_log(MHI_MSG_ERROR, "Failed to init event");
		goto error_event_handle_alloc;
	}
	/* Initialize the event which signals M0*/
	mhi_dev_ctxt->M0_event = kmalloc(sizeof(wait_queue_head_t), GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->M0_event) {
		mhi_log(MHI_MSG_ERROR, "Failed to init event");
		goto error_state_change_event_handle;
	}
	/* Initialize the event which signals M0*/
	mhi_dev_ctxt->M3_event = kmalloc(sizeof(wait_queue_head_t), GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->M3_event) {
		mhi_log(MHI_MSG_ERROR, "Failed to init event");
		goto error_M0_event;
	}
	mhi_dev_ctxt->chan_start_complete= kmalloc(sizeof(wait_queue_head_t), GFP_KERNEL);
	if (NULL == mhi_dev_ctxt->chan_start_complete) {
		mhi_log(MHI_MSG_ERROR, "Failed to init event");
		goto error_M3_event;
	}
	/* Initialize the event which starts the event parsing thread */
	init_waitqueue_head(mhi_dev_ctxt->event_handle);
	/* Initialize the event which starts the state change thread */
	init_waitqueue_head(mhi_dev_ctxt->state_change_event_handle);
	/* Initialize the event which triggers clients waiting to send*/
	init_waitqueue_head(mhi_dev_ctxt->M0_event);
	/* Initialize the event which triggers D3hot*/
	init_waitqueue_head(mhi_dev_ctxt->M3_event);
	init_waitqueue_head(mhi_dev_ctxt->chan_start_complete);

	return MHI_STATUS_SUCCESS;
error_M3_event:
	kfree(mhi_dev_ctxt->M3_event);
error_M0_event:
	kfree(mhi_dev_ctxt->M0_event);
error_state_change_event_handle:
	kfree(mhi_dev_ctxt->state_change_event_handle);
error_event_handle_alloc:
	kfree(mhi_dev_ctxt->event_handle);
	return MHI_STATUS_ERROR;
}
/**
 * @brief Initialize the work item list for the state change thread
 *	 NOTE: This function also gets called on MHI reset, but in that case
 *		the mutex would have already been initialized.
 *
 * @param q [IN ]
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_state_change_thread_work_queue(mhi_state_work_queue *q)
{
	q->queue_full_cntr = 0;
	q->q_info.base = q->buf;
	q->q_info.rp = q->buf;
	q->q_info.wp = q->buf;
	q->q_info.len = MHI_WORK_Q_MAX_SIZE * sizeof(mhi_state_work_item);
	q->q_info.el_size = sizeof(mhi_state_work_item);
	q->q_info.overwrite_en = 0;
	if (NULL == q->q_mutex) {
		q->q_mutex = kmalloc(sizeof(struct mutex), GFP_KERNEL);
		if (NULL == q->q_mutex)
			return MHI_STATUS_ALLOC_ERROR;
		mutex_init(q->q_mutex);
	}

	return MHI_STATUS_SUCCESS;
}
/* @brief Initialize the event ring and add event ring
 *	elements for the mhi_dev_ctxt to fill.  Write the event
 *	doorbell, to announce to the mhi_dev_ctxt the new available elements
 */
MHI_STATUS mhi_init_event_ring(mhi_device_ctxt *mhi_dev_ctxt, u32 nr_ev_el,
				u32 event_ring_index)
{
	mhi_event_pkt *ev_pkt = NULL;
	u32 i = 0;
	u64 db_value = 0;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;
	spinlock_t *lock =
		&mhi_dev_ctxt->mhi_ev_spinlock_list[event_ring_index];
	mhi_ring *event_ctxt = NULL;

	if (NULL == mhi_dev_ctxt || 0 == nr_ev_el) {
		mhi_log(MHI_MSG_ERROR, "Bad Input data, quitting\n");
		return MHI_STATUS_ERROR;
	}

	spin_lock(lock);

	mhi_log(MHI_MSG_INFO, "mmio_addr = 0x%lx, mmio_len = 0x%llx\n",
			mhi_dev_ctxt->mmio_addr, mhi_dev_ctxt->mmio_len);
	mhi_log(MHI_MSG_INFO,
			"Initializing event ring %d\n", event_ring_index);

	for (i = 0; i < nr_ev_el - 1; ++i) {
		event_ctxt =
			&mhi_dev_ctxt->mhi_local_event_ctxt[event_ring_index];
		ret_val = ctxt_add_element(event_ctxt, (void *)&ev_pkt);
		if (MHI_STATUS_SUCCESS != ret_val) {
			mhi_log(MHI_MSG_ERROR,
				"Failed to insert el in ev ctxt\n");
			ret_val = MHI_STATUS_ERROR;
			break;
		}
		db_value = mhi_v2p_addr(mhi_dev_ctxt->mhi_ctrl_seg_info,
				(uintptr_t)event_ctxt->wp);
		MHI_WRITE_DB(mhi_dev_ctxt->event_db_addr,
					event_ring_index, db_value);
	}

	spin_unlock(lock);
	return ret_val;
}

/**
 * @brief Allocate a physically contiguous pool of memory for the mhi
 *	 control segment.
 *
 * @param mhi_device [IN ] Context to which to attach the control segment
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_device_ctrl(mhi_device_ctxt *mhi_device)
{
	size_t ctrl_seg_size = 0;
	size_t ctrl_seg_offset = 0;
	u32 i = 0;
	u32 align_len = sizeof(u64)*2;
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;

	if (NULL == mhi_device || NULL == mhi_device->mhi_ctrl_seg_info ||
			NULL == mhi_device->mhi_ctrl_seg_info->dev)
		return MHI_STATUS_ERROR;

	mhi_log(MHI_MSG_INFO, "Allocating control segment.\n");
	ctrl_seg_size += sizeof(mhi_control_seg);
	/* Calculate the size of the control segment needed */
	ctrl_seg_size += align_len - (ctrl_seg_size % align_len);
	for (i = 0; i < MHI_MAX_CHANNELS; ++i) {
		if (IS_HARDWARE_CHANNEL(i))
			ctrl_seg_size += sizeof(mhi_xfer_pkt) *
				(MAX_NR_TRBS_PER_HARD_CHAN + ELEMENT_GAP);
		else if (IS_SOFTWARE_CHANNEL(i))
			ctrl_seg_size += sizeof(mhi_xfer_pkt) *
				(MAX_NR_TRBS_PER_SOFT_CHAN + ELEMENT_GAP);
	}
	ctrl_seg_size += align_len - (ctrl_seg_size % align_len);

	for (i = 0; i < EVENT_RINGS_ALLOCATED; ++i)
		ctrl_seg_size += sizeof(mhi_event_pkt)*
					(EV_EL_PER_RING + ELEMENT_GAP);

	ctrl_seg_size += align_len - (ctrl_seg_size % align_len);
	ret_val = mhi_mallocmemregion(mhi_device->mhi_ctrl_seg_info,
							ctrl_seg_size);
	if (MHI_STATUS_SUCCESS != ret_val)
		return MHI_STATUS_ERROR;
	(mhi_device->mhi_ctrl_seg =
			mhi_get_virt_addr(mhi_device->mhi_ctrl_seg_info));

	if (0 == mhi_device->mhi_ctrl_seg)
		return MHI_STATUS_ALLOC_ERROR;

	/* Set the channel contexts, event contexts and cmd context */
	ctrl_seg_offset = (uintptr_t)mhi_device->mhi_ctrl_seg +
						sizeof(mhi_control_seg);
	ctrl_seg_offset += align_len - (ctrl_seg_offset % align_len);
	/* Set the TRB lists */
	for (i = 0; i < MHI_MAX_CHANNELS; ++i) {
		if (IS_HARDWARE_CHANNEL(i)) {
			mhi_device->mhi_ctrl_seg->xfer_trb_list[i] =
				(mhi_xfer_pkt *)ctrl_seg_offset;
			ctrl_seg_offset += sizeof(mhi_xfer_pkt) *
				(MAX_NR_TRBS_PER_HARD_CHAN + ELEMENT_GAP);

		} else if (IS_SOFTWARE_CHANNEL(i)) {
			mhi_device->mhi_ctrl_seg->xfer_trb_list[i] =
				(mhi_xfer_pkt *)ctrl_seg_offset;
			ctrl_seg_offset += sizeof(mhi_xfer_pkt) *
				(MAX_NR_TRBS_PER_SOFT_CHAN + ELEMENT_GAP);
		}
	}

	ctrl_seg_offset += align_len - (ctrl_seg_offset % align_len);
	for (i = 0; i < EVENT_RINGS_ALLOCATED; ++i) {
		mhi_device->mhi_ctrl_seg->ev_trb_list[i] =
			(mhi_event_pkt *)ctrl_seg_offset;
		ctrl_seg_offset += sizeof(mhi_event_pkt) *
			(EV_EL_PER_RING + ELEMENT_GAP);
	}
	return MHI_STATUS_SUCCESS;
}

/**
 * @brief Initialize the mhi control and data memory segments, specifically,
 *	 all rings, contexts and buffers.
 *	 The two segments are then stitched together.
 *	 This function is called during normal initialization as well as during
 *	 an mhi_reset.
 *
 * @param mhi_device
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_contexts(mhi_device_ctxt *mhi_device)
{
	u32 i = 0;
	mhi_control_seg *mhi_ctrl = mhi_device->mhi_ctrl_seg;
	mhi_event_ctxt *event_ctxt = NULL;
	u32 event_ring_index = 0;
	mhi_xfer_pkt *trb_list = NULL;
	mhi_chan_ctxt *chan_ctxt = NULL;
	mhi_ring *local_event_ctxt = NULL;
	u32 msi_vec = 0;

	for (i = 0; i < EVENT_RINGS_ALLOCATED; ++i) {
		MHI_GET_EVENT_RING_INFO(EVENT_RING_MSI_VEC,
					mhi_device->ev_ring_props[i],
					msi_vec);
		event_ring_index = mhi_device->alloced_ev_rings[i];
		event_ctxt = &mhi_ctrl->mhi_ec_list[event_ring_index];
		local_event_ctxt =
			&mhi_device->mhi_local_event_ctxt[event_ring_index];

		mhi_event_ring_init(event_ctxt,
				mhi_v2p_addr(mhi_device->mhi_ctrl_seg_info,
					(uintptr_t)mhi_ctrl->ev_trb_list[i]),
				(uintptr_t)mhi_ctrl->ev_trb_list[i],
				EV_EL_PER_RING, local_event_ctxt, 0,
				msi_vec);
	}

	/* Init Command Ring */
	mhi_cmd_ring_init(&mhi_ctrl->mhi_cmd_ctxt_list[PRIMARY_CMD_RING],
			mhi_v2p_addr(mhi_device->mhi_ctrl_seg_info,
			(uintptr_t)mhi_ctrl->cmd_trb_list[PRIMARY_CMD_RING]),
			(uintptr_t)mhi_ctrl->cmd_trb_list[PRIMARY_CMD_RING],
			CMD_EL_PER_RING,
			&mhi_device->mhi_local_cmd_ctxt[PRIMARY_CMD_RING]);

	mhi_log(MHI_MSG_INFO, "Initializeing contexts\n");
	/* Initialize Channel Contexts */
	for (i = 0; i < MHI_MAX_CHANNELS; ++i) {
		trb_list = mhi_device->mhi_ctrl_seg->xfer_trb_list[i];
		chan_ctxt = &mhi_device->mhi_ctrl_seg->mhi_cc_list[i];
		if (IS_SOFTWARE_CHANNEL(i)) {
			mhi_init_chan_ctxt(chan_ctxt,
				mhi_v2p_addr(mhi_device->mhi_ctrl_seg_info,
					(uintptr_t)trb_list),
				(uintptr_t)trb_list,
				MAX_NR_TRBS_PER_SOFT_CHAN,
				(i % 2) ? MHI_IN : MHI_OUT,
				0,
				&mhi_device->mhi_local_chan_ctxt[i]);
		} else if (IS_HARDWARE_CHANNEL(i)) {

			mhi_init_chan_ctxt(chan_ctxt,
				mhi_v2p_addr(mhi_device->mhi_ctrl_seg_info,
					(uintptr_t)trb_list),
				(uintptr_t)trb_list,
				MAX_NR_TRBS_PER_HARD_CHAN,
				(i % 2) ? MHI_IN : MHI_OUT,
				i,
				&mhi_device->mhi_local_chan_ctxt[i]);
		}
	}
	mhi_device->mhi_state = MHI_STATE_RESET;

	return MHI_STATUS_SUCCESS;
}

/**
 * @brief Initialize the channel context and shadow context
 *
 * @param cc_list	 Context to initialize
 * @param trb_list_phy	 Physical base address for the TRE ring
 * @param trb_list_virt Virtual base address for the TRE ring
 * @param el_per_ring	 Number of TREs this ring will contain
 * @param chan_type	 Type of channel IN/OUT
 * @param event_ring	 Event ring to be mapped to this channel context
 * @param ring		 Shadow context to be initialized alongside
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_chan_ctxt(mhi_chan_ctxt *cc_list,
		uintptr_t trb_list_phy, uintptr_t trb_list_virt,
		u64 el_per_ring, MHI_CHAN_TYPE chan_type,
		u32 event_ring, mhi_ring *ring)
{
	cc_list->mhi_chan_state = MHI_CHAN_STATE_DISABLED;
	cc_list->mhi_chan_type = chan_type;
	cc_list->mhi_event_ring_index = event_ring;
	cc_list->mhi_trb_ring_base_addr = trb_list_phy;
	cc_list->mhi_trb_ring_len = ((size_t)(el_per_ring)*sizeof(mhi_tx_pkt));
	cc_list->mhi_trb_read_ptr = trb_list_phy;
	cc_list->mhi_trb_write_ptr = trb_list_phy;
	ring->rp = (void *)(trb_list_virt);
	ring->ack_rp = ring->rp;
	ring->wp = (void *)(trb_list_virt);
	ring->base = (void *)(trb_list_virt);
	ring->len = ((size_t)(el_per_ring)*sizeof(mhi_tx_pkt));
	ring->el_size = sizeof(mhi_tx_pkt);
	ring->overwrite_en = 0;
	return MHI_STATUS_SUCCESS;
}

/**
 * @brief Initialize an event ring context
 *
 * @param ev_list	 Event ring context to initialize
 * @param trb_list_phy_addr Pointer to phy mem to the tre list for event ring
 * @param trb_list_virt_addr Pointer to virt mem to the tre list for event ring
 * @param el_per_ring	 Number of event ring elements in this ring
 * @param ring		 Pointer to the shadow context of this event ring
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_event_ring_init(mhi_event_ctxt *ev_list,
		uintptr_t trb_list_phy_addr, uintptr_t trb_list_virt_addr,
		size_t el_per_ring, mhi_ring *ring,
		u32 intmodt_val, u32 msi_vec)
{
	ev_list->mhi_event_er_type = MHI_EVENT_RING_TYPE_VALID;
	ev_list->mhi_msi_vector     = msi_vec;
	ev_list->mhi_event_ring_base_addr = trb_list_phy_addr;
	ev_list->mhi_event_ring_len = el_per_ring*sizeof(mhi_event_pkt);
	ev_list->mhi_event_read_ptr = trb_list_phy_addr;
	ev_list->mhi_event_write_ptr = trb_list_phy_addr;
	ring->wp = (void *)(uintptr_t)trb_list_virt_addr;
	ring->rp = (void *)(uintptr_t)trb_list_virt_addr;
	ring->base = (void *)(uintptr_t)(trb_list_virt_addr);
	ring->len = ((size_t)(el_per_ring)*sizeof(mhi_event_pkt));
	ring->el_size = sizeof(mhi_event_pkt);
	ring->overwrite_en = 0;
	return MHI_STATUS_SUCCESS;
}

/**
 * @brief Initialization of the command ring
 *
 * @param cmd_ctxt		command ring context to initialize
 * @param trb_list_phy_addr	 Pointer to the pysical address of the tre ring
 * @param trb_list_virt_addr	 Pointer to the virtual address of the tre ring
 * @param el_per_ring		Number of elements in this command ring
 * @param ring			Pointer to the shadow command context
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_cmd_ring_init(mhi_cmd_ctxt *cmd_ctxt,
				uintptr_t trb_list_phy_addr,
				uintptr_t trb_list_virt_addr,
				size_t el_per_ring, mhi_ring *ring)
{
	cmd_ctxt->mhi_cmd_ring_base_addr = trb_list_phy_addr;
	cmd_ctxt->mhi_cmd_ring_read_ptr = trb_list_phy_addr;
	cmd_ctxt->mhi_cmd_ring_write_ptr = trb_list_phy_addr;
	cmd_ctxt->mhi_cmd_ring_len = (size_t)el_per_ring*sizeof(mhi_cmd_pkt);
	ring[PRIMARY_CMD_RING].wp = (void *)trb_list_virt_addr;
	ring[PRIMARY_CMD_RING].rp = (void *)trb_list_virt_addr;
	ring[PRIMARY_CMD_RING].base = (void *)trb_list_virt_addr;
	ring[PRIMARY_CMD_RING].len = (size_t)el_per_ring*sizeof(mhi_cmd_pkt);
	ring[PRIMARY_CMD_RING].el_size = sizeof(mhi_cmd_pkt);
	ring[PRIMARY_CMD_RING].overwrite_en = 0;
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS mhi_reset_all_thread_queues(mhi_device_ctxt *mhi_dev_ctxt)
{
	MHI_STATUS ret_val = MHI_STATUS_SUCCESS;

	mhi_init_state_change_thread_work_queue(
				&mhi_dev_ctxt->state_change_work_item_list);
	if (MHI_STATUS_SUCCESS != ret_val) {
		mhi_log(MHI_MSG_ERROR, "Failed to reset STT work queue\n");
		return ret_val;
	}
	return ret_val;
}

MHI_STATUS mhi_init_timers(mhi_device_ctxt *mhi_dev_ctxt)
{
	hrtimer_init(&mhi_dev_ctxt->inactivity_tmr,
			CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);
	mhi_dev_ctxt->inactivity_timeout =
			ktime_set(0, MHI_M1_ENTRY_DELAY_MS * 1E6L);
	mhi_dev_ctxt->inactivity_tmr.function = mhi_initiate_M1;
	return MHI_STATUS_SUCCESS;
}
