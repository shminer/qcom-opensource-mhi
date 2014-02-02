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
#ifndef _MHI_RMNET_PUB_API_
#define _MHI_RMNET_PUB_API_

#include <linux/types.h>
#include <asm/types.h>
#define MHI_MAX_NR_CHANNEL_DESCRIPTORS (9)
typedef void *mhi_rmnet_client_handle;

#define MHI_DMA_MASK       0x3FFFFFFF
#define MHI_MAX_MTU        0xFFFF

typedef enum MHI_RMNET_STATUS_TYPE
{
	MHI_RMNET_STATUS_SUCCESS       = 0x0,
	MHI_RMNET_STATUS_ERROR         = 0x1,
	MHI_RMNET_STATUS_RING_FULL     = 0x3,
	MHI_RMNET_STATUS_reserved = 0xFFFF0000,
}MHI_RMNET_STATUS_TYPE, MHI_RMNET_STATUS;

typedef struct mhi_rmnet_result_t
{
	void* user_data;
	void* payload;
	uint32_t bytes_xferd;
	MHI_RMNET_STATUS transaction_status;
} mhi_rmnet_result;

typedef struct mhi_rmnet_client_info_t
{
	void (*mhi_rmnet_xfer_cb)(mhi_rmnet_result *);
	void (*mhi_rmnet_chan_reset_cb)(void *user_data);
	u32 cb_mod;
}mhi_rmnet_client_info;

/* List of available hardware channels */
typedef enum MHI_RMNET_HW_CLIENT_CHANNEL {
	MHI_RMNET_CLIENT_IP_HW_0_OUT   = 100,
	MHI_RMNET_CLIENT_IP_HW_0_IN    = 101,
	MHI_RMNET_CLIENT_IP_HW_1_OUT   = 102,
	MHI_RMNET_CLIENT_IP_HW_1_IN    = 103,
	MHI_RMNET_CLIENT_IP_HW_2_OUT   = 104,
	MHI_RMNET_CLIENT_IP_HW_2_IN    = 105,
	MHI_RMNET_CLIENT_IP_HW_3_OUT   = 106,
	MHI_RMNET_CLIENT_IP_HW_3_IN    = 107,
	MHI_RMNET_CLIENT_reserved      = 0xFFFF0000,
}MHI_RMNET_HW_CLIENT_CHANNEL;

MHI_RMNET_STATUS mhi_rmnet_open_channel(
		mhi_rmnet_client_handle* mhi_rmnet_handle,
		MHI_RMNET_HW_CLIENT_CHANNEL chan_id, void* user_data,
		mhi_rmnet_client_info* info);

MHI_RMNET_STATUS mhi_rmnet_queue_buffer(
		mhi_rmnet_client_handle mhi_rmnet_handle,
		uintptr_t buf, size_t len);

void mhi_rmnet_close_channel(mhi_rmnet_client_handle mhi_rmnet_handle);
void mhi_rmnet_mask_irq(mhi_rmnet_client_handle mhi_rmnet_handle);
void mhi_rmnet_unmask_irq(mhi_rmnet_client_handle mhi_rmnet_handle);
mhi_rmnet_result* mhi_rmnet_poll(mhi_rmnet_client_handle mhi_rmnet_handle);
MHI_RMNET_STATUS mhi_rmnet_reset_channel(mhi_rmnet_client_handle mhi_rmnet_handle);
uint32_t mhi_rmnet_get_max_buffers(mhi_rmnet_client_handle mhi_rmnet_handle);
uint32_t mhi_rmnet_get_epid(mhi_rmnet_client_handle mhi_rmnet_handle);


#endif
