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
#ifndef _MHI_SHIM_PUB
#define _MHI_SHIM_PUB

#include <linux/types.h>
#include <asm/types.h>
typedef void* mhi_shim_client_handle;

typedef enum MHI_SHIM_CLIENT_CHANNEL
{
	MHI_SHIM_CLIENT_LOOPBACK_OUT = 0,
	MHI_SHIM_CLIENT_LOOPBACK_IN = 1,
	MHI_SHIM_CLIENT_SAHARA_OUT = 2,
	MHI_SHIM_CLIENT_SAHARA_IN = 3,
	MHI_SHIM_CLIENT_DIAG_OUT = 4,
	MHI_SHIM_CLIENT_DIAG_IN = 5,
	MHI_SHIM_CLIENT_SSR_OUT = 6,
	MHI_SHIM_CLIENT_SSR_IN = 7,
	MHI_SHIM_CLIENT_QDSS_OUT = 8,
	MHI_SHIM_CLIENT_QDSS_IN = 9,
	MHI_SHIM_CLIENT_EFS_OUT = 10,
	MHI_SHIM_CLIENT_EFS_IN = 11,
	MHI_SHIM_CLIENT_MBIM_OUT = 12,
	MHI_SHIM_CLIENT_MBIM_IN = 13,
	MHI_SHIM_CLIENT_QMI_OUT = 14,
	MHI_SHIM_CLIENT_QMI_IN = 15,
	MHI_SHIM_CLIENT_IP_CTRL_0_OUT = 16,
	MHI_SHIM_CLIENT_IP_CTRL_0_IN = 17,
	MHI_SHIM_CLIENT_IP_CTRL_1_OUT = 18,
	MHI_SHIM_CLIENT_IP_CTRL_1_IN = 19,
	MHI_SHIM_CLIENT_IP_CTRL_2_OUT = 20,
	MHI_SHIM_CLIENT_IP_CTRL_2_IN = 21,
	MHI_SHIM_CLIENT_IP_CTRL_3_OUT = 22,
	MHI_SHIM_CLIENT_IP_CTRL_3_IN = 23,
	MHI_SHIM_CLIENT_IP_CTRL_4_OUT = 24,
	MHI_SHIM_CLIENT_IP_CTRL_4_IN = 25,
	MHI_SHIM_CLIENT_IP_CTRL_5_OUT = 26,
	MHI_SHIM_CLIENT_IP_CTRL_5_IN = 27,
	MHI_SHIM_CLIENT_IP_CTRL_6_OUT = 28,
	MHI_SHIM_CLIENT_IP_CTRL_6_IN = 29,
	MHI_SHIM_CLIENT_IP_CTRL_7_OUT = 30,
	MHI_SHIM_CLIENT_IP_CTRL_7_IN = 31,
	MHI_SHIM_CLIENT_DUN_OUT = 32,
	MHI_SHIM_CLIENT_DUN_IN = 33,
	MHI_SHIM_CLIENT_IP_SW_0_OUT = 34,
	MHI_SHIM_CLIENT_IP_SW_0_IN = 35,
	MHI_SHIM_CLIENT_IP_SW_1_OUT = 36,
	MHI_SHIM_CLIENT_IP_SW_1_IN = 37,
	MHI_SHIM_CLIENT_IP_SW_2_OUT = 38,
	MHI_SHIM_CLIENT_IP_SW_2_IN = 39,
	MHI_SHIM_CLIENT_IP_SW_3_OUT = 40,
	MHI_SHIM_CLIENT_IP_SW_3_IN = 41,
	MHI_SHIM_CLIENT_CSVT_OUT = 42,
	MHI_SHIM_CLIENT_CSVT_IN = 43,
	MHI_SHIM_CLIENT_SMCT_OUT = 44,
	MHI_SHIM_CLIENT_SMCT_IN = 45,
}MHI_SHIM_CLIENT_CHANNEL;

typedef enum MHI_SHIM_STATUS
{
	MHI_SHIM_STATUS_SUCCESS = 0,
	MHI_SHIM_STATUS_ERROR = 1,
	MHI_SHIM_STATUS_RING_FULL = 3,
	MHI_SHIM_STATUS_RING_EMPTY = 4,
}MHI_SHIM_STATUS;

typedef struct mhi_shim_result
{
	void* user_data; /*<-- Data passed back to the user's callback -->*/
	void* payload_buf; /*<-- Payload for rx call -->*/
	u32 bytes_xferd; /*<-- Actual number of bytes transferred -->*/
	MHI_SHIM_STATUS transaction_status; /*<-- Returned status of the last transaction -->*/
}mhi_shim_result;

typedef struct mhi_shim_client_cbs_t
{
	/*CB to be invoked upon packet transmission rx/tx*/
	void (*mhi_shim_xfer_cb)(mhi_shim_result*);
	/*CB to be invoked upon a channel reset */ 
	void (*mhi_shim_chan_reset_cb)(void* user_data);
}mhi_shim_client_cbs_t;

int mhi_shim_get_free_buf_count(mhi_shim_client_handle client_handle);
void mhi_shim_poll_inbound(mhi_shim_client_handle client_handle, uintptr_t* buf ,size_t* buf_size);

void mhi_shim_close_channel(mhi_shim_client_handle client_handle);
MHI_SHIM_STATUS mhi_shim_open_channel(mhi_shim_client_handle* client_handle,
		MHI_SHIM_CLIENT_CHANNEL chan, s32 device_index,
		mhi_shim_client_cbs_t* RxCb, void* user_data);
MHI_SHIM_STATUS mhi_shim_queue_xfer(mhi_shim_client_handle client_handle, 
		uintptr_t buf, size_t buf_len, u32 chain);

MHI_SHIM_STATUS mhi_shim_recycle_buffer(mhi_shim_client_handle client_handle);

#endif 
