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
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/slab.h>

/* MHI Includes */
#include "mhi_shim.h"
#include "shim.h"
#define TRB_MAX_DATA_SIZE 0x1000

SHIM_DBG_LEVEL mhi_shim_msg_lvl = SHIM_DBG_INFO;

static ssize_t mhi_shim_client_read(struct file *file, char __user *buf,
					size_t count, loff_t *offp);
static ssize_t mhi_shim_client_write(struct file *file,
		const char __user *buf, size_t count, loff_t *offp);
static int mhi_shim_client_open(struct inode *mhi_inode, struct file*);
static int mhi_shim_client_release(struct inode *mhi_inode,
					struct file *file_handle);

mhi_shim_ctxt_t mhi_shim_ctxt;

static struct miscdevice mhi_shim_misc_devs[MHI_SOFTWARE_CLIENT_LIMIT];

static const struct file_operations mhi_shim_client_fops = {
read: mhi_shim_client_read,
write : mhi_shim_client_write,
open : mhi_shim_client_open,
release : mhi_shim_client_release,
};

static int mhi_shim_client_open(struct inode *mhi_inode,
				struct file *file_handle)
{
	shim_client_handle_t *shim_client_handle = NULL;
	int ret_val = 0;
	u32 out_chan_nr = iminor(mhi_inode);

	shim_client_handle = &mhi_shim_ctxt.client_handle_list[out_chan_nr];
	memset(shim_client_handle, 0, sizeof(shim_client_handle_t));

	if (NULL == shim_client_handle) {
		ret_val = -ENOMEM;
		goto handle_alloc_err;
	}

	shim_client_handle->out_chan = out_chan_nr;
	shim_client_handle->in_chan = out_chan_nr + 1;
	shim_client_handle->shim_ctxt = &mhi_shim_ctxt;
	ret_val = mhi_shim_open_channel(&shim_client_handle->outbound_handle,
			(MHI_SHIM_CLIENT_CHANNEL)shim_client_handle->out_chan,
			0,
			(mhi_shim_client_cbs_t *)&(mhi_shim_ctxt.client_cbs),
			(void *)shim_client_handle);

	if (MHI_SHIM_STATUS_SUCCESS != ret_val) {
		mhi_shim_log(SHIM_DBG_ERROR,
				"Failed to init out chan client handle\n");
		goto out_handle_err;
	}
	ret_val = mhi_shim_open_channel(&shim_client_handle->inbound_handle,
				shim_client_handle->in_chan,
				0,
				NULL,
				(void *)shim_client_handle);

	if (MHI_SHIM_STATUS_SUCCESS != ret_val) {
		mhi_shim_log(SHIM_DBG_ERROR,
				"Failed to init out chan client handle\n");
		goto in_handle_err;
	}

	/* If this channel was never opened before */
	file_handle->private_data = shim_client_handle;

	return 0;

in_handle_err:
	mhi_shim_close_channel(shim_client_handle->outbound_handle);
out_handle_err:
	vfree(shim_client_handle);
handle_alloc_err:
	return ret_val;
}

static int mhi_shim_client_release(struct inode *mhi_inode,
		struct file *file_handle)
{
	shim_client_handle_t *client_handle = file_handle->private_data;

	if (NULL == client_handle)
		return -EINVAL;
	mhi_shim_close_channel(client_handle->outbound_handle);
	mhi_shim_close_channel(client_handle->inbound_handle);
	return 0;
}

static ssize_t mhi_shim_client_read(struct file *file, char __user *buf,
		size_t uspace_buf_size, loff_t *offp)
{
	shim_client_handle_t *shim_handle = NULL;
	u32 uspace_buf_amount_filled = 0;
	uintptr_t phy_buf = 0;
	mhi_shim_client_handle *client_handle = NULL;
	size_t amount_to_copy = 0;
	int ret_val = 0;
	size_t buf_size = 0;
	struct mutex *mutex;
	u32 chan = 0;


	if (NULL == file || NULL == buf ||
		0 == uspace_buf_size || NULL == file->private_data)
		return -EINVAL;

	shim_handle = file->private_data;
	client_handle = shim_handle->inbound_handle;
	mutex = &mhi_shim_ctxt.client_chan_lock[shim_handle->in_chan];
	chan = shim_handle->in_chan;
	mutex_lock(mutex);
	buf_size = mhi_shim_ctxt.channel_attributes[chan].max_packet_size;
	for (;;) {
		if (0 == *offp) {
			mhi_shim_poll_inbound(client_handle,
						&phy_buf,
						&shim_handle->pending_data);
			if (0 == phy_buf || 0 == shim_handle->pending_data)
				break;
			dma_unmap_single(NULL,
					(dma_addr_t)phy_buf,
					buf_size,
					DMA_BIDIRECTIONAL);
			*offp = (uintptr_t)dma_to_virt(NULL,
					(dma_addr_t)(uintptr_t)phy_buf);
		}
		/* If we can fit the entire pending buffer */
		if (uspace_buf_size >=
			(uspace_buf_amount_filled + shim_handle->pending_data))
			amount_to_copy = shim_handle->pending_data;
		else /* If we can't fit the whole buffer, copy what we can */
			amount_to_copy = uspace_buf_size -
						uspace_buf_amount_filled;


		if (0 != copy_to_user(buf + uspace_buf_amount_filled,
					(void *)(uintptr_t)(*offp),
					amount_to_copy)) {
			ret_val = -EIO;
			goto error;
		}
		shim_handle->pending_data -= amount_to_copy;
		if (0 == shim_handle->pending_data) {
			memset((void *)(uintptr_t)(*offp), 0, buf_size);
			/* No need to free this buffer, re-submit it to mhi */
			dma_map_single(NULL, (void *)(uintptr_t)(*offp),
					buf_size,
					DMA_BIDIRECTIONAL);
			ret_val = mhi_shim_recycle_buffer(client_handle);
			if (MHI_SHIM_STATUS_SUCCESS != ret_val) {
				mhi_shim_log(SHIM_DBG_ERROR,
						"Failed to recycle element\n");
				ret_val = -EIO;
				goto error;
			}
			*offp = 0;
		} else {
			*offp += amount_to_copy;
		}
		uspace_buf_amount_filled += amount_to_copy;

		/* We filled the entire user buffer, we are done*/
		if (uspace_buf_amount_filled == uspace_buf_size)
			break;
	}

	ret_val = uspace_buf_amount_filled;
error:
	mutex_unlock(mutex);
	return ret_val;
}

static ssize_t mhi_shim_client_write(struct file *file,
		const char __user *buf,
		size_t count, loff_t *offp)
{
	shim_client_handle_t *shim_handle = NULL;
	int ret_val = 0;
	u32 chan = 0xFFFFFFFF;

	mhi_shim_log(SHIM_DBG_VERBOSE,
			"Attempting to write 0x%x bytes\n", count);
	if (NULL == file || NULL == buf ||
		0 == count || NULL == file->private_data)
		return -EINVAL;
	else
		shim_handle = (shim_client_handle_t *)file->private_data;
	chan = shim_handle->out_chan;
	mutex_lock(&shim_handle->shim_ctxt->client_chan_lock[chan]);
	ret_val = mhi_shim_send_packet(shim_handle->outbound_handle,
			(void *)buf, count, chan);
	mutex_unlock(&shim_handle->shim_ctxt->client_chan_lock[chan]);
	return ret_val;
}


int mhi_shim_probe(struct pci_dev *dev)
{
	char *dev_node_name = NULL;
	u32 i = 0;
	MHI_SHIM_STATUS ret_val = MHI_SHIM_STATUS_SUCCESS;
	mhi_shim_client_handle init_handle = {0};

	mhi_shim_ctxt.client_cbs.mhi_shim_xfer_cb = shim_write_cb;

	/* Create a device node mapped to channels 0 and 1 for R/W*/
	dev_node_name = vmalloc(MHI_DEV_NODE_NAME_LEN);
	if (NULL == dev_node_name)
		return -EINVAL;
	memset(dev_node_name, '\0', sizeof(MHI_DEV_NODE_NAME_LEN));

	for (i = 0; i < MHI_MAX_SOFTWARE_CHANNELS; ++i)
		mutex_init(&mhi_shim_ctxt.client_chan_lock[i]);

	for (i = 0; i < MHI_SOFTWARE_CLIENT_LIMIT; ++i) {
		snprintf(dev_node_name,
			MHI_DEV_NODE_NAME_LEN,
			"mhi_pipe_%d", i*2);
		/* Map each device node to a pair of channels */
		mhi_shim_misc_devs[i].minor = i * 2;
		mhi_shim_misc_devs[i].name = dev_node_name;
		mhi_shim_misc_devs[i].fops = &mhi_shim_client_fops;
		ret_val = misc_register(&mhi_shim_misc_devs[i]);
		if (0 != ret_val) {
			mhi_shim_log(SHIM_DBG_ERROR,
			"Failed to register miscdev with code %d\n", ret_val);
		}
	}
	vfree(dev_node_name);
	ret_val = shim_init_client_attributes(&mhi_shim_ctxt);
	if (MHI_SHIM_STATUS_SUCCESS != ret_val) {
		mhi_shim_log(SHIM_DBG_ERROR,
				"Failed to init client attributes\n");
		return -EIO;
	}
	/* We need to ensure that buffers are available for the device
	 * as soon as possibe, even before a client has opened his interface
	 * on the host, therefore, add buffers on the inbound channels. */
	for (i = 1; i < MHI_MAX_SOFTWARE_CHANNELS; i += 2) {
		ret_val = mhi_shim_open_channel(&init_handle, i,
						0, NULL, NULL);
		if (MHI_SHIM_STATUS_SUCCESS != ret_val)
			mhi_shim_log(SHIM_DBG_ERROR,
					"Failed to open chan 0x%x\n", i);
		ret_val = mhi_init_inbound(init_handle, i);

		if (MHI_SHIM_STATUS_SUCCESS != ret_val)
			mhi_shim_log(SHIM_DBG_ERROR,
					"Failed to init inbound 0x%x\n", i);
		mhi_shim_close_channel(init_handle);
	}
	return 0;
}
int mhi_shim_remove(struct pci_dev *dev)
{
	return 0;
}
int mhi_shim_send_packet(mhi_shim_client_handle *client_handle,
		void *buf, u32 size, u32 chan)
{
	MHI_SHIM_STATUS ret_val = MHI_SHIM_STATUS_SUCCESS;
	u32 avail_buf_space = 0;
	u32 nr_avail_trbs = 0;
	u32 chain = 0;
	u32 i = 0;
	void *data_loc = NULL;
	uintptr_t memcpy_result = 0;
	u32 data_left_to_insert = 0;
	size_t data_to_insert_now = 0;
	u32 data_inserted_so_far = 0;
	dma_addr_t dma_addr = 0;

	if (NULL == client_handle || NULL == buf || 0 == size)
		return MHI_SHIM_STATUS_ERROR;

	nr_avail_trbs = mhi_shim_get_free_buf_count(client_handle);

	data_left_to_insert = size;

	if (0 == nr_avail_trbs) {
		mhi_shim_log(SHIM_DBG_ERROR, "Channel Full 0x%x\n", size);
		return -ENOMEM;
	}

	for (i = 0; i < nr_avail_trbs; ++i) {
		data_to_insert_now = MIN(data_left_to_insert,
					 TRB_MAX_DATA_SIZE);

		data_loc = kmalloc(data_to_insert_now, GFP_KERNEL);
		if (NULL == data_loc) {
			mhi_shim_log(SHIM_DBG_ERROR,
				 "Failed to allocate memory 0x%x\n", size);
			return -ENOMEM;
		}
		memcpy_result = copy_from_user(data_loc,
						buf + data_inserted_so_far,
						data_to_insert_now);
		if (0 != memcpy_result)
			return data_inserted_so_far;

		dma_addr = dma_map_single(NULL, data_loc, size, DMA_TO_DEVICE);
		if (dma_mapping_error(NULL, dma_addr)) {
			mhi_shim_log(SHIM_DBG_ERROR,
					"Failed to Map DMA 0x%x\n", size);
			return -ENOMEM;
		}

		data_left_to_insert -= data_to_insert_now;
		data_inserted_so_far += data_to_insert_now;

		chain = (data_left_to_insert > 0) ? 1 : 0;
		ret_val = mhi_shim_queue_xfer(client_handle, dma_addr,
						data_to_insert_now, chain);
		if (0 != ret_val)
			break;
		if (0 == data_left_to_insert || 0 == avail_buf_space)
			break;
	}
	return data_inserted_so_far;
}


/**
 * @brief Statically initialize the channel attributes table.
 *	 This table contains information on the nature of the transfer
 *	 on a particular pipe; information which helps us optimize
 *	 the memory allocation layout
 *
 * @param device [IN/OUT] reference to a mhi context to be populated
 *
 * @return MHI_SHIM_STATUS
 */
MHI_SHIM_STATUS shim_init_client_attributes(mhi_shim_ctxt_t *mhi_shim_ctxt)
{
	u32 i = 0;
	u32 nr_trbs = MAX_NR_TRBS_PER_CHAN;
	u32 data_size = TRB_MAX_DATA_SIZE;
	chan_attr *chan_attributes = NULL;
	for (i = 0; i < MHI_MAX_SOFTWARE_CHANNELS; ++i) {
		chan_attributes = &mhi_shim_ctxt->channel_attributes[i];
		chan_attributes->chan_id = i;
		chan_attributes->max_packet_size = data_size;
		chan_attributes->avg_packet_size = data_size;
		chan_attributes->max_nr_packets = nr_trbs;
		chan_attributes->nr_trbs = nr_trbs;
		if (i % 2 == 0)
			chan_attributes->dir = MHI_DIR_OUT;
		else
			chan_attributes->dir = MHI_DIR_IN;
	}
	return MHI_SHIM_STATUS_SUCCESS;
}

MHI_SHIM_STATUS mhi_init_inbound(mhi_shim_client_handle *client_handle,
					MHI_SHIM_CLIENT_CHANNEL chan)
{

	MHI_SHIM_STATUS ret_val = MHI_SHIM_STATUS_SUCCESS;
	u32 i = 0;
	dma_addr_t dma_addr = 0;
	chan_attr *chan_attributes = &mhi_shim_ctxt.channel_attributes[chan];
	void *data_loc = NULL;
	size_t buf_size = chan_attributes->max_packet_size;

	if (NULL == client_handle) {
		mhi_shim_log(SHIM_DBG_ERROR, "Bad Input data, quitting\n");
		return MHI_SHIM_STATUS_ERROR;
	}

	for (i = 0; i < (chan_attributes->nr_trbs - 1); ++i) {
		data_loc = kmalloc(buf_size, GFP_KERNEL);
		dma_addr = dma_map_single(NULL, data_loc,
					buf_size, DMA_BIDIRECTIONAL);
		if (dma_mapping_error(NULL, dma_addr)) {
			mhi_shim_log(SHIM_DBG_ERROR, "Failed to Map DMA\n");
			return -ENOMEM;
		}
		(ret_val = mhi_shim_queue_xfer(client_handle,
						dma_addr, buf_size, 0));
		if (MHI_SHIM_STATUS_SUCCESS != ret_val)
			goto error_insert;
	}
	return ret_val;
error_insert:
	mhi_shim_log(SHIM_DBG_ERROR,
			"Failed insertion for chan 0x%x\n", chan);

	return MHI_SHIM_STATUS_ERROR;
}

void shim_write_cb(mhi_shim_result *result)
{
	dma_unmap_single(NULL,
			(dma_addr_t)(uintptr_t)result->payload_buf,
			result->bytes_xferd,
			DMA_TO_DEVICE);

	kfree(dma_to_virt(NULL, (dma_addr_t)(uintptr_t)result->payload_buf));
}

