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
MHI_DEBUG_LEVEL mhi_msg_lvl = MHI_MSG_CRITICAL;
MHI_DEBUG_CLASS mhi_msg_class = MHI_DBG_DATA | MHI_DBG_POWER;

module_param(mhi_msg_lvl , uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mhi_msg_lvl, "dbg lvl");

module_param(mhi_msg_class , uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mhi_msg_class, "dbg class");

static ssize_t mhi_dbgfs_chan_read(struct file *fp, char __user *buf,
				size_t count, loff_t *offp);
static ssize_t mhi_dbgfs_state_read(struct file *fp, char __user *buf,
				size_t count, loff_t *offp);

static const struct file_operations mhi_dbgfs_chan_fops = {
.read = mhi_dbgfs_chan_read,
.write = NULL,
};

static const struct file_operations mhi_dbgfs_state_fops = {
.read = mhi_dbgfs_state_read,
.write = NULL,
};

static char chan_info[0x1000];
int mhi_init_debugfs(mhi_device_ctxt *mhi_dev_ctxt)
{
	struct dentry *mhi_parent_folder;
	struct dentry *mhi_chan_stats;
	struct dentry *mhi_state_stats;
	mhi_parent_folder = debugfs_create_dir("mhi", NULL);
	if (NULL == mhi_parent_folder) {
		mhi_log(MHI_MSG_INFO, "Failed to create debugfs parent dir.\n");
		return -EIO;
	}
	mhi_chan_stats = debugfs_create_file("mhi_chan_stats",
					0444,
					mhi_parent_folder,
					mhi_dev_ctxt,
					&mhi_dbgfs_chan_fops);
	mhi_state_stats = debugfs_create_file("mhi_state_stats",
					0444,
					mhi_parent_folder,
					mhi_dev_ctxt,
					&mhi_dbgfs_state_fops);
	return 0;
}
static ssize_t mhi_dbgfs_state_read(struct file *fp, char __user *buf,
				size_t count, loff_t *offp)
{
	int amnt_copied = 0;
	mhi_device_ctxt *mhi_dev_ctxt = mhi_devices.device_list[0].mhi_ctxt;
	if (NULL == mhi_dev_ctxt)
		return -EIO;
	usleep(100000);
	amnt_copied =
	scnprintf(chan_info,
			sizeof(chan_info),
			"%s %d %s %d %s %d %s %d %s %d %s %d\n",
			"M0->M1:",
			mhi_dev_ctxt->m0_m1,
			"M0<-M1:",
			mhi_dev_ctxt->m1_m0,
			"M1->M2:",
			mhi_dev_ctxt->m1_m2,
			"M0<-M2:",
			mhi_dev_ctxt->m2_m0,
			"M0->M3:",
			mhi_dev_ctxt->m0_m3,
			"M0<-M3:",
			mhi_dev_ctxt->m3_m0);
	if (amnt_copied < count)
		return amnt_copied - copy_to_user(buf, chan_info, amnt_copied);
	else
		return -ENOMEM;
}

static ssize_t mhi_dbgfs_chan_read(struct file *fp, char __user *buf,
				size_t count, loff_t *offp)
{
	int amnt_copied = 0;
	mhi_chan_ctxt *chan_ctxt;
	mhi_device_ctxt *mhi_dev_ctxt = mhi_devices.device_list[0].mhi_ctxt;
	if (NULL == mhi_dev_ctxt)
		return -EIO;
	*offp = (u32)(*offp) % MHI_MAX_CHANNELS;
	chan_ctxt = &mhi_dev_ctxt->mhi_ctrl_seg->mhi_cc_list[*offp];
	amnt_copied =
	scnprintf(chan_info,
		sizeof(chan_info),
		"%s0x%x %s %d %s %d %s 0x%x %s 0x%llx %s %p %s %p %s %p\n",
		"chan:",
		(unsigned int)*offp,
		"pkts to dev:",
		mhi_dev_ctxt->mhi_chan_cntr[*offp].pkts_to_dev,
		"pkts from dev:",
		mhi_dev_ctxt->mhi_chan_cntr[*offp].pkts_from_dev,
		"chan_state:",
		chan_ctxt->mhi_chan_state,
		"chan_base phy:",
		chan_ctxt->mhi_trb_ring_base_addr,
		"chan_base virt:",
		mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].base,
		"chan_wp virt:",
		mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].wp,
		"chan_rp virt:",
		mhi_dev_ctxt->mhi_local_chan_ctxt[*offp].rp);
	*offp += 1;

	if (amnt_copied < count)
		return amnt_copied -
			copy_to_user(buf, chan_info, amnt_copied);
	else
		return -ENOMEM;
}
inline uintptr_t mhi_p2v_addr(mhi_meminfo *meminfo, uintptr_t pa)
{
	return meminfo->va_aligned + (pa - meminfo->pa_aligned);
}

inline uintptr_t mhi_v2p_addr(mhi_meminfo *meminfo, uintptr_t va)
{
	return meminfo->pa_aligned + (va - meminfo->va_aligned);
}
inline void *mhi_get_virt_addr(mhi_meminfo *meminfo)
{
	return (void *)meminfo->va_aligned;
}

inline void mhi_memcpy(void *to, void *from, size_t size)
{
	memcpy(to, from, size);
}

inline u64 mhi_get_memregion_len(mhi_meminfo *meminfo)
{
	return meminfo->size;
}

MHI_STATUS mhi_mallocmemregion(mhi_meminfo *meminfo, size_t size)
{
	meminfo->va_unaligned = (uintptr_t)dma_alloc_coherent(NULL,
				size,
				(dma_addr_t *)&(meminfo->pa_unaligned),
				GFP_KERNEL);
	meminfo->va_aligned = meminfo->va_unaligned;
	meminfo->pa_aligned = meminfo->pa_unaligned;
	meminfo->size = size;
	if ((meminfo->pa_unaligned + size) >= MHI_DATA_SEG_WINDOW_END_ADDR)
		return MHI_STATUS_ERROR;

	if (0 == meminfo->va_unaligned)
		return MHI_STATUS_ERROR;
	mb();
	return MHI_STATUS_SUCCESS;
}

void mhi_freememregion(mhi_meminfo *meminfo)
{
	mb();
	dma_free_coherent(meminfo->dev,
			meminfo->size,
			(dma_addr_t *)&meminfo->pa_unaligned,
			GFP_KERNEL);

	meminfo->va_aligned = 0;
	meminfo->pa_aligned = 0;
	meminfo->va_unaligned = 0;
	meminfo->pa_unaligned = 0;
	return;
}
MHI_STATUS mhi_spawn_thread(void *ctxt, int(fn)(void *),
			osal_thread *handle, char name[])
{
	handle->thread_handle = kthread_run(fn, ctxt, name);
	if (-ENOMEM == (int)handle->thread_handle)
		return MHI_STATUS_ERROR;
	else
		return MHI_STATUS_SUCCESS;
}

void print_ring(mhi_ring *local_chan_ctxt, u32 ring_id)
{
	u32 i = 0;
	mhi_log(MHI_MSG_VERBOSE,
		"Chan %d, ACK_RP 0x%p RP 0x%p WP 0x%p BASE 0x%p:\n",
		ring_id,
		local_chan_ctxt->ack_rp,
		local_chan_ctxt->rp,
		local_chan_ctxt->wp,
		local_chan_ctxt->base);
	for (i = 0; i < MAX_NR_TRBS_PER_SOFT_CHAN; ++i) {
		mhi_log(MHI_MSG_VERBOSE, "0x%x: TRB: 0x%p ",
			i, &((mhi_tx_pkt *)local_chan_ctxt->base)[i]);
		mhi_log(MHI_MSG_VERBOSE,
			"Buff Ptr = 0x%llx, Buf Len: 0x%x, Buf Flags: 0x%x\n",
			((mhi_tx_pkt *)(local_chan_ctxt->base))[i].buffer_ptr,
			((mhi_tx_pkt *)(local_chan_ctxt->base))[i].buf_len,
			((mhi_tx_pkt *)(local_chan_ctxt->base))[i].info);
	}

}
