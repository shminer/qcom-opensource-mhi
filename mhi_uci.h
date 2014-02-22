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
#ifndef _H_MHI_SHIM_INTERFACE
#define _H_MHI_SHIM_INTERFACE

#include <msm_mhi.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/pci.h>
#include <linux/vmalloc.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/platform_device.h>

#define MIN(_x, _y)(((_x) < (_y)) ? (_x) : (_y))
#define MHI_DEV_NODE_NAME_LEN 13
#define MHI_MAX_NR_OF_CLIENTS 23
#define MHI_SOFTWARE_CLIENT_START 0
#define MHI_SOFTWARE_CLIENT_LIMIT 23
#define MHI_MAX_SOFTWARE_CHANNELS 46

#define MAX_NR_TRBS_PER_CHAN 10
#define MHI_PCIE_VENDOR_ID 0x17CB
#define MHI_PCIE_DEVICE_ID 0x0300
#define DEVICE_NAME "mhi"

#define CHAN_TO_CLIENT_INDEX(_CHAN_NR) (_CHAN_NR / 2)

typedef enum SHIM_DBG_LEVEL {
	SHIM_DBG_VERBOSE = 0x0,
	SHIM_DBG_INFO = 0x1,
	SHIM_DBG_DBG = 0x2,
	SHIM_DBG_WARNING = 0x3,
	SHIM_DBG_ERROR = 0x4,
	SHIM_DBG_CRITICAL = 0x5,
	SHIM_DBG_reserved = 0x80000000
} SHIM_DBG_LEVEL;

extern SHIM_DBG_LEVEL mhi_uci_msg_lvl;

#define mhi_uci_log(_msg_lvl, _msg, ...) do { \
	if (_msg_lvl >= mhi_uci_msg_lvl) { \
		pr_info("[%s] "_msg, __func__, ##__VA_ARGS__); \
	} \
} while (0)

typedef struct mhi_uci_ctxt_t mhi_uci_ctxt_t;
typedef enum MHI_SHIM_DEBUG_LEVEL {
	MHI_DBG_VERBOSE = 0x0,
	MHI_DBG_INFO = 0x1,
	MHI_DBG_DBG = 0x2,
	MHI_DBG_WARNING = 0x3,
	MHI_DBG_ERROR = 0x4,
	MHI_DBG_CRITICAL = 0x5,
	MHI_DBG_reserved = 0x80000000
} MHI_SHIM_DEBUG_LEVEL;

/* Begin MHI Specification Definition */
typedef enum MHI_CHAN_DIR {
	MHI_DIR_INVALID = 0x0,
	MHI_DIR_OUT = 0x1,
	MHI_DIR_IN = 0x2,
	MHI_DIR__reserved = 0x80000000
} MHI_CHAN_DIR;


typedef struct chan_attr {
	MHI_CLIENT_CHANNEL chan_id;
	size_t max_packet_size;
	size_t avg_packet_size;
	u32 max_nr_packets;
	u32 nr_trbs;
	MHI_CHAN_DIR dir;
} chan_attr;

typedef struct uci_client {
	u32 client_index;
	u32 out_chan;
	u32 in_chan;
	mhi_client_handle* outbound_handle;
	mhi_client_handle* inbound_handle;
	size_t pending_data;
	mhi_uci_ctxt_t *uci_ctxt;
	wait_queue_head_t read_wait_queue;
	atomic_t avail_pkts;
	struct device *dev;
} uci_client;

typedef struct mhi_uci_ctxt_t {
	chan_attr channel_attributes[MHI_MAX_SOFTWARE_CHANNELS];
	uci_client client_handle_list[MHI_SOFTWARE_CLIENT_LIMIT];
	struct mutex client_chan_lock[MHI_MAX_SOFTWARE_CHANNELS];
	mhi_client_info_t client_info;
	dev_t start_ctrl_nr;
	struct cdev cdev[MHI_MAX_SOFTWARE_CHANNELS];
	struct class *mhi_uci_class;
} mhi_uci_ctxt_t;

void uci_xfer_cb(mhi_result *result);
int mhi_uci_send_packet(mhi_client_handle *client_handle, void *buf,
		u32 size, u32 chan);
MHI_STATUS mhi_init_inbound(mhi_client_handle *client_handle,
		MHI_CLIENT_CHANNEL chan);
MHI_STATUS uci_init_client_attributes(mhi_uci_ctxt_t *mhi_uci_ctxt);
int mhi_uci_probe(struct platform_device *dev);
int mhi_uci_remove(struct platform_device *dev);

#endif
