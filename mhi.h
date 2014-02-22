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

#ifndef _H_MHI
#define _H_MHI

#include <msm_mhi.h>
#include "mhi_macros.h"
#include <linux/types.h>
#include <linux/spinlock_types.h>
#include <linux/hrtimer.h>
#include <linux/pm.h>
#include <linux/completion.h>


typedef struct osal_thread osal_thread;
typedef struct mhi_meminfo mhi_meminfo;
typedef struct pci_dev pci_dev;
typedef struct device device;
typedef struct mhi_device_ctxt mhi_device_ctxt;
typedef struct mhi_pcie_devices mhi_pcie_devices;
typedef struct hrtimer hrtimer;
extern mhi_pcie_devices mhi_devices;

typedef enum MHI_DEBUG_CLASS {
	MHI_DBG_DATA = 0x1000,
	MHI_DBG_POWER = 0x2000,
	MHI_DBG_reserved = 0x80000000
} MHI_DEBUG_CLASS;

typedef enum MHI_DEBUG_LEVEL {
	MHI_MSG_VERBOSE = 0x0,
	MHI_MSG_INFO = 0x2,
	MHI_MSG_DBG = 0x4,
	MHI_MSG_WARNING = 0x8,
	MHI_MSG_ERROR = 0x10,
	MHI_MSG_CRITICAL = 0x20,
	MHI_MSG_reserved = 0x80000000
} MHI_DEBUG_LEVEL;

typedef struct pcie_core_info {
	u32 dev_id;
	u32 manufact_id;
	u32 mhi_ver;
	u64 bar0_base;
	u64 bar0_end;
	u64 bar2_base;
	u64 bar2_end;
	u32 device_wake_gpio;
	u32 irq_base;
	u32 max_nr_msis;
} pcie_core_info;

typedef struct bhi_ctxt_t {
	uintptr_t bhi_base;
	void *image_loc;
	dma_addr_t phy_image_loc;
	size_t image_size;
	void *unaligned_image_loc;
} bhi_ctxt_t;

typedef struct mhi_pcie_dev_info {
	pcie_core_info core;
	atomic_t ref_count;
	mhi_device_ctxt *mhi_ctxt;
	pci_dev *pcie_device;
	bhi_ctxt_t bhi_ctxt;
} mhi_pcie_dev_info;

typedef struct mhi_pcie_devices {
	mhi_pcie_dev_info device_list[MHI_MAX_SUPPORTED_DEVICES];
	s32 nr_of_devices;
} mhi_pcie_devices;

typedef enum MHI_CHAN_TYPE {
	MHI_INVALID = 0x0,
	MHI_OUT = 0x1,
	MHI_IN = 0x2,
	MHI_CHAN_TYPE_reserved = 0x80000000
} MHI_CHAN_TYPE;

typedef enum MHI_CHAN_STATE {
	MHI_CHAN_STATE_DISABLED = 0x0,
	MHI_CHAN_STATE_ENABLED = 0x1,
	MHI_CHAN_STATE_RUNNING = 0x2,
	MHI_CHAN_STATE_SUSPENDED = 0x3,
	MHI_CHAN_STATE_STOP = 0x4,
	MHI_CHAN_STATE_ERROR = 0x5,
	MHI_CHAN_STATE_LIMIT = 0x6,
	MHI_CHAN_STATE_reserved = 0x80000000
} MHI_CHAN_STATE;

typedef enum MHI_RING_TYPE {
	MHI_RING_TYPE_CMD_RING = 0x0,
	MHI_RING_TYPE_XFER_RING = 0x1,
	MHI_RING_TYPE_EVENT_RING = 0x2,
	MHI_RING_TYPE_MAX = 0x4,
	MHI_RING_reserved = 0x80000000
} MHI_RING_TYPE;

typedef enum MHI_CHAIN {
	MHI_TRE_CHAIN_OFF = 0x0,
	MHI_TRE_CHAIN_ON = 0x1,
	MHI_TRE_CHAIN_LIMIT = 0x2,
	MHI_TRE_CHAIN_reserved = 0x80000000
} MHI_CHAIN;

typedef enum MHI_STATE {
	MHI_STATE_RESET = 0x0,
	MHI_STATE_READY = 0x1,
	MHI_STATE_M0 = 0x2,
	MHI_STATE_M1 = 0x3,
	MHI_STATE_M2 = 0x4,
	MHI_STATE_M3 = 0x5,
	MHI_STATE_BHI  = 0x7,
	MHI_STATE_LIMIT = 0x8,
	MHI_STATE_reserved = 0x80000000
} MHI_STATE;

#pragma pack(1)
typedef struct mhi_event_ctxt {
	u32 mhi_intmodt;
	u32 mhi_event_er_type;
	u32 mhi_msi_vector;
	u64 mhi_event_ring_base_addr;
	u64 mhi_event_ring_len;
	volatile u64 mhi_event_read_ptr;
	u64 mhi_event_write_ptr;
} mhi_event_ctxt;

typedef struct mhi_chan_ctxt {
	MHI_CHAN_STATE mhi_chan_state;
	MHI_CHAN_TYPE mhi_chan_type;
	u32 mhi_event_ring_index;
	u64 mhi_trb_ring_base_addr;
	u64 mhi_trb_ring_len;
	u64 mhi_trb_read_ptr;
	u64 mhi_trb_write_ptr;
} mhi_chan_ctxt;

typedef struct mhi_cmd_ctxt {
	u32 mhi_cmd_ctxt_reserved1;
	u32 mhi_cmd_ctxt_reserved2;
	u32 mhi_cmd_ctxt_reserved3;
	u64 mhi_cmd_ring_base_addr;
	u64 mhi_cmd_ring_len;
	u64 mhi_cmd_ring_read_ptr;
	u64 mhi_cmd_ring_write_ptr;
} mhi_cmd_ctxt;

#pragma pack()

typedef enum MHI_COMMAND {
	MHI_COMMAND_NOOP = 0x0,
	MHI_COMMAND_RESET_CHAN = 0x1,
	MHI_COMMAND_STOP_CHAN = 0x2,
	MHI_COMMAND_START_CHAN = 0x3,
	MHI_COMMAND_RESUME_CHAN = 0x4,
	MHI_COMMAND_MAX_NR = 0x5,
	MHI_COMMAND_reserved = 0x80000000
} MHI_COMMAND;

typedef enum MHI_PKT_TYPE {
	MHI_PKT_TYPE_RESERVED = 0x0,
	MHI_PKT_TYPE_NOOP_CMD = 0x1,
	MHI_PKT_TYPE_TRANSFER = 0x2,
	MHI_PKT_TYPE_RESET_CHAN_CMD = 0x10,
	MHI_PKT_TYPE_STOP_CHAN_CMD = 0x11,
	MHI_PKT_TYPE_START_CHAN_CMD = 0x12,
	MHI_PKT_TYPE_STATE_CHANGE_EVENT = 0x20,
	MHI_PKT_TYPE_CMD_COMPLETION_EVENT = 0x21,
	MHI_PKT_TYPE_TX_EVENT = 0x22,
	MHI_PKT_TYPE_EE_EVENT = 0x40,
} MHI_PKT_TYPE;

#pragma pack(1)

typedef struct mhi_tx_pkt {
	u64 buffer_ptr;
	u32 buf_len;
	u32 info;
} mhi_tx_pkt;

typedef struct mhi_noop_tx_pkt {
	u64 reserved1;
	u32 reserved2;
	u32 info;
} mhi_noop_tx_pkt;

typedef struct mhi_noop_cmd_pkt {
	u64 reserved1;
	u32 reserved2;
	u32 info;
} mhi_noop_cmd_pkt;

typedef struct mhi_reset_chan_cmd_pkt {
	u32 reserved1;
	u32 reserved2;
	u32 reserved3;
	u32 info;
} mhi_reset_chan_cmd_pkt;

typedef struct mhi_stop_chan_cmd_pkt {
	u32 reserved1;
	u32 reserved2;
	u32 reserved3;
	u32 info;
} mhi_stop_chan_cmd_pkt;
typedef struct mhi_ee_state_change_event {
	u64 reserved1;
	u32 exec_env;
	u32 info;
} mhi_ee_state_change_event;

typedef struct mhi_xfer_event_pkt {
	volatile u64 xfer_ptr;
	volatile u32 xfer_details;
	volatile u32 info;
} mhi_xfer_event_pkt;

typedef struct mhi_cmd_complete_event_pkt {
	u64 ptr;
	u32 code;
	u32 info;
} mhi_cmd_complete_event_pkt;

typedef struct mhi_state_change_event_pkt {
	u64 reserved1;
	u32 state;
	u32 info;
} mhi_state_change_event_pkt;

typedef union mhi_xfer_pkt {
	mhi_tx_pkt data_tx_pkt;
	mhi_noop_tx_pkt noop_tx_pkt;
	mhi_tx_pkt type;
} mhi_xfer_pkt;

typedef union mhi_cmd_pkt {
	mhi_stop_chan_cmd_pkt stop_cmd_pkt;
	mhi_reset_chan_cmd_pkt reset_cmd_pkt;
	mhi_noop_cmd_pkt noop_cmd_pkt;
	mhi_noop_cmd_pkt type;
} mhi_cmd_pkt;

typedef union mhi_event_pkt {
	mhi_xfer_event_pkt xfer_event_pkt;
	mhi_cmd_complete_event_pkt cmd_complete_event_pkt;
	mhi_state_change_event_pkt state_change_event_pkt;
	mhi_ee_state_change_event ee_event_pkt;
	mhi_xfer_event_pkt type;
} mhi_event_pkt;

#pragma pack()
typedef enum MHI_EVENT_CCS {
	MHI_EVENT_CC_INVALID = 0x0,
	MHI_EVENT_CC_SUCCESS = 0x1,
	MHI_EVENT_CC_EOT = 0x2,
	MHI_EVENT_CC_UNDEFINED_ERR = 0x10,
	MHI_EVENT_CC_RING_EL_ERR = 0x11,
} MHI_EVENT_CCS;

typedef enum MHI_THREAD_STATE {
	MHI_THREAD_STATE_SUSPENDED = 0,
	MHI_THREAD_STATE_RUNNING = 1,
	MHI_THREAD_STATE_EXIT = 2,
	MHI_THREAD_STATE_reserved = 0x80000000
} MHI_THREAD_STATE;


typedef struct mhi_ring {
	void *base;
	void *volatile wp;
	void *volatile rp;
	void *volatile ack_rp;
	uintptr_t len;
	uintptr_t el_size;
	u32 overwrite_en;
	atomic_t nr_filled_elements;
} mhi_ring;

typedef enum MHI_CMD_STATUS {
	MHI_CMD_NOT_PENDING = 0x0,
	MHI_CMD_PENDING = 0x1,
	MHI_CMD_RESET_PENDING = 0x2,
	MHI_CMD_RESERVED = 0x80000000
} MHI_CMD_STATUS;

typedef enum MHI_EVENT_RING_TYPE {
	MHI_EVENT_RING_TYPE_INVALID = 0x0,
	MHI_EVENT_RING_TYPE_VALID = 0x1,
	MHI_EVENT_RING_TYPE_reserved = 0x80000000
} MHI_EVENT_RING_TYPE;

typedef enum MHI_INIT_ERROR_STAGE {
	MHI_INIT_ERROR_STAGE_UNWIND_ALL = 0x1,
	MHI_INIT_ERROR_STAGE_DEVICE_CTRL = 0x2,
	MHI_INIT_ERROR_STAGE_THREADS = 0x3,
	MHI_INIT_ERROR_STAGE_EVENTS = 0x4,
	MHI_INIT_ERROR_STAGE_MEM_ZONES = 0x5,
	MHI_INIT_ERROR_STAGE_SYNC = 0x6,
	MHI_INIT_ERROR_STAGE_THREAD_QUEUES = 0x7,
	MHI_INIT_ERROR_STAGE_RESERVED = 0x80000000
} MHI_INIT_ERROR_STAGE;

typedef enum MHI_STATE_TRANSITION {
	STATE_TRANSITION_RESET = 0x0,
	STATE_TRANSITION_READY = 0x1,
	STATE_TRANSITION_M0 = 0x2,
	STATE_TRANSITION_M1 = 0x3,
	STATE_TRANSITION_M2 = 0x4,
	STATE_TRANSITION_M3 = 0x5,
	STATE_TRANSITION_BHI = 0x6,
	STATE_TRANSITION_SBL = 0x7,
	STATE_TRANSITION_AMSS = 0x8,
	STATE_TRANSITION_SYS_ERR = 0xFF,
	STATE_TRANSITION_reserved = 0x80000000
} MHI_STATE_TRANSITION;

typedef enum MHI_EXEC_ENV {
	MHI_EXEC_ENV_SBL = 0x1,
	MHI_EXEC_ENV_AMSS = 0x2,
} MHI_EXEC_ENV;
typedef struct mhi_state_work_item {
	MHI_STATE_TRANSITION new_state;
} mhi_state_work_item;

typedef struct mhi_client_handle {
	mhi_device_ctxt *mhi_dev_ctxt;
	mhi_client_info_t client_info;
	struct completion chan_close_complete;
	void *user_data;
	u32 chan;
	mhi_result result;
	u32 device_index;
	u32 event_ring_index;
	u32 msi_vec;
	u32 cb_mod;
	u32 pkt_count;
} mhi_client_handle;

typedef enum MHI_EVENT_POLLING {
	MHI_EVENT_POLLING_DISABLED = 0x0,
	MHI_EVENT_POLLING_ENABLED = 0x1,
	MHI_EVENT_POLLING_reserved = 0x80000000
} MHI_EVENT_POLLING;

typedef struct mhi_state_work_queue {
	struct mutex *q_mutex;
	mhi_ring q_info;
	u32 queue_full_cntr;
	mhi_state_work_item buf[MHI_WORK_Q_MAX_SIZE];
} mhi_state_work_queue;

typedef struct mhi_control_seg {
	mhi_xfer_pkt *xfer_trb_list[MHI_MAX_CHANNELS];
	mhi_event_pkt *ev_trb_list[EVENT_RINGS_ALLOCATED];
	mhi_cmd_pkt cmd_trb_list[NR_OF_CMD_RINGS][CMD_EL_PER_RING + 1];
	mhi_cmd_ctxt mhi_cmd_ctxt_list[NR_OF_CMD_RINGS];
	mhi_chan_ctxt mhi_cc_list[MHI_MAX_CHANNELS];
	mhi_event_ctxt mhi_ec_list[MHI_MAX_CHANNELS];
	u32 padding;
} mhi_control_seg;

typedef struct mhi_counters {
	u32 pkts_to_dev;
	u32 pkts_from_dev;
	u32 ev_processed;
} mhi_counters;

struct mhi_device_ctxt {
	mhi_pcie_dev_info *dev_info;
	pcie_core_info dev_props;
	volatile u64 channel_db_addr;
	volatile u64 event_db_addr;
	volatile u64 cmd_db_addr;
	mhi_control_seg *mhi_ctrl_seg;
	mhi_meminfo *mhi_ctrl_seg_info;
	u64 nr_of_cc;
	u64 nr_of_ec;
	u64 nr_of_cmdc;
	MHI_STATE mhi_state;
	volatile uintptr_t mmio_addr;
	volatile u64 mmio_len;
	mhi_ring mhi_local_chan_ctxt[MHI_MAX_CHANNELS];
	mhi_ring mhi_local_event_ctxt[MHI_MAX_CHANNELS];
	mhi_ring mhi_local_cmd_ctxt[NR_OF_CMD_RINGS];
	struct mutex *mhi_chan_mutex;
	spinlock_t *mhi_ev_spinlock_list;
	struct mutex *mhi_cmd_mutex_list;
	mhi_client_handle *client_handle_list[MHI_MAX_CHANNELS];
	osal_thread *event_thread_handle;
	osal_thread *state_change_thread_handle;
	wait_queue_head_t *event_handle;
	wait_queue_head_t *state_change_event_handle;
	wait_queue_head_t *M0_event;
	wait_queue_head_t *M3_event;
	wait_queue_head_t *chan_start_complete;
	u32 pending_M3;
	atomic_t mhi_chan_db_order[MHI_MAX_CHANNELS];
	spinlock_t *db_write_lock;

	MHI_THREAD_STATE event_thread_state;
	MHI_THREAD_STATE state_change_thread_state;
	struct platform_device *mhi_uci_dev;
	struct platform_device *mhi_rmnet_dev;

	volatile u32 kill_threads;
	mhi_state_work_queue state_change_work_item_list;
	MHI_CMD_STATUS mhi_chan_pend_cmd_ack[MHI_MAX_CHANNELS];
	atomic_t data_pending;
	u32 mhi_initialized;
	atomic_t events_pending;
	atomic_t start_cmd_pending_ack;
	u32 alloced_ev_rings[EVENT_RINGS_ALLOCATED];
	u32 ev_ring_props[EVENT_RINGS_ALLOCATED];
	u32 hw_intmod_rate;
	u32 outbound_evmod_rate;
	u32 m0_m1;
	u32 m1_m0;
	u32 m1_m2;
	u32 m2_m0;
	u32 m0_m3;
	u32 m3_m0;
	rwlock_t xfer_lock;
	hrtimer inactivity_tmr;
	ktime_t inactivity_timeout;
	mhi_counters mhi_chan_cntr[MHI_MAX_CHANNELS];
	u32 ev_counter[EVENT_RINGS_ALLOCATED];
};

MHI_STATUS mhi_reset_all_thread_queues(mhi_device_ctxt *mhi_dev_ctxt);

MHI_STATUS mhi_add_elements_to_event_rings(mhi_device_ctxt *mhi_dev_ctxt,
					MHI_STATE_TRANSITION new_state);
MHI_STATUS validate_xfer_el_addr(mhi_chan_ctxt *ring, uintptr_t addr);
int get_nr_avail_ring_elements(mhi_ring *ring);
MHI_STATUS get_nr_enclosed_el(mhi_ring *ring, void *loc_1,
					void *loc_2, u32 *nr_el);
MHI_STATUS mhi_init_contexts(mhi_device_ctxt *mhi_dev_ctxt);
MHI_STATUS mhi_init_device_ctrl(mhi_device_ctxt *mhi_dev_ctxt);
MHI_STATUS mhi_init_mmio(mhi_device_ctxt *mhi_dev_ctxt);
MHI_STATUS mhi_init_device_ctxt(mhi_pcie_dev_info *dev_info,
				mhi_device_ctxt **mhi_dev_ctxt);


MHI_STATUS mhi_init_event_ring(mhi_device_ctxt *mhi_dev_ctxt,
		u32 nr_ev_el, u32 event_ring_index);

MHI_STATUS mhi_event_ring_init(mhi_event_ctxt *ev_list, uintptr_t trb_list_phy,
		uintptr_t trb_list_virt,
		size_t el_per_ring, mhi_ring *ring,
		u32 intmodt_val, u32 msi_vec);

/*Mhi Initialization functions */
MHI_STATUS mhi_create_ctxt(mhi_device_ctxt **mhi_dev_ctxt);
MHI_STATUS mhi_clean_init_stage(mhi_device_ctxt *mhi_dev_ctxt,
				MHI_INIT_ERROR_STAGE cleanup_stage);
MHI_STATUS mhi_init_sync(mhi_device_ctxt *mhi_dev_ctxt);
MHI_STATUS mhi_init_ctrl_zone(mhi_pcie_dev_info *dev_info,
				mhi_device_ctxt *mhi_dev_ctxt);

MHI_STATUS mhi_init_threads(mhi_device_ctxt *mhi_dev_ctxt);
MHI_STATUS mhi_init_events(mhi_device_ctxt *mhi_dev_ctxt);

MHI_STATUS mhi_reset(mhi_device_ctxt *mhi_dev_ctxt);
MHI_STATUS mhi_send_cmd(mhi_device_ctxt *dest_device,
			MHI_COMMAND which_cmd, u32 chan);

MHI_STATUS mhi_start(mhi_pcie_dev_info *new_devices, u32 nr_devices);
MHI_STATUS mhi_queue_tx_pkt(mhi_device_ctxt *mhi_dev_ctxt,
				MHI_CLIENT_CHANNEL chan,
				void *payload,
				size_t payload_size);
MHI_STATUS mhi_cmd_ring_init(mhi_cmd_ctxt *cmd_ring_id,
			uintptr_t trb_list_phy, uintptr_t trb_list_virt,
			size_t el_per_ring, mhi_ring *ring);
MHI_STATUS parse_cmd_completion_event(mhi_device_ctxt *mhi_dev_ctxt,
					mhi_event_pkt *ev_pkt);

MHI_STATUS mhi_init_chan_ctxt(mhi_chan_ctxt *cc_list,
		uintptr_t trb_list_phy,
		uintptr_t trb_list_virt,
		u64 el_per_ring,
		MHI_CHAN_TYPE chan_type,
		u32 event_ring,
		mhi_ring *ring);

MHI_STATUS add_element(mhi_ring *ring, void *volatile *rp,
			void *volatile *wp, void **assigned_addr);
MHI_STATUS delete_element(mhi_ring *ring, void *volatile *rp,
			 void *volatile *wp, void **assigned_addr);
MHI_STATUS ctxt_add_element(mhi_ring *ring, void **assigned_addr);
MHI_STATUS ctxt_del_element(mhi_ring *ring, void **assigned_addr);
MHI_STATUS get_element_index(mhi_ring *ring, void *address, uintptr_t *index);
MHI_STATUS get_element_addr(mhi_ring *ring, uintptr_t index, void **address);

MHI_STATUS recycle_trb_and_ring(mhi_device_ctxt *mhi_dev_ctxt, mhi_ring *ring,
		MHI_RING_TYPE ring_type, u32 ring_index);
MHI_STATUS mhi_change_chan_state(mhi_device_ctxt *mhi_dev_ctxt, u32 chan_id,
					MHI_CHAN_STATE new_state);
MHI_STATUS parse_xfer_event(mhi_device_ctxt *ctxt, mhi_event_pkt *event);
MHI_STATUS parse_cmd_event(mhi_device_ctxt *ctxt, mhi_event_pkt *event);
int parse_event_thread(void *ctxt);

MHI_STATUS mhi_init_device(mhi_pcie_dev_info *new_device);
MHI_STATUS mhi_process_event_ring(mhi_device_ctxt *mhi_dev_ctxt,
		u32 ev_ring_nr, u32 quota);

MHI_STATUS mhi_init_outbound(mhi_device_ctxt *mhi_dev_ctxt,
					MHI_CLIENT_CHANNEL);
MHI_STATUS mhi_test_for_device_ready(mhi_device_ctxt *mhi_dev_ctxt);

MHI_STATUS validate_ring_el_addr(mhi_ring *ring, uintptr_t addr);
MHI_STATUS validate_ev_el_addr(mhi_ring *ring, uintptr_t addr);

MHI_STATUS mhi_spawn_threads(mhi_device_ctxt *mhi_dev_ctxt);

void assert_device_wake(void);

MHI_STATUS reset_chan_cmd(mhi_device_ctxt *mhi_dev_ctxt, mhi_cmd_pkt *cmd_pkt);
MHI_STATUS start_chan_cmd(mhi_device_ctxt *mhi_dev_ctxt, mhi_cmd_pkt *cmd_pkt);
MHI_STATUS parse_outbound(mhi_device_ctxt *mhi_dev_ctxt, u32 chan,
			mhi_xfer_pkt *local_ev_trb_loc, u16 xfer_len);
MHI_STATUS parse_inbound(mhi_device_ctxt *mhi_dev_ctxt, u32 chan,
			mhi_xfer_pkt *local_ev_trb_loc, u16 xfer_len);

int mhi_state_change_thread(void *ctxt);
MHI_STATUS mhi_init_state_change_thread_work_queue(mhi_state_work_queue *q);
MHI_STATUS mhi_init_state_transition(mhi_device_ctxt *mhi_dev_ctxt,
					MHI_STATE_TRANSITION new_state);
MHI_STATUS mhi_set_state_of_all_channels(mhi_device_ctxt *mhi_dev_ctxt,
					MHI_CHAN_STATE new_state);
void ring_all_chan_dbs(mhi_device_ctxt *mhi_dev_ctxt);
MHI_STATUS process_stt_work_item(mhi_device_ctxt  *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item);
MHI_STATUS process_M0_transition(mhi_device_ctxt  *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item);
MHI_STATUS process_M1_transition(mhi_device_ctxt  *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item);
MHI_STATUS process_M3_transition(mhi_device_ctxt  *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item);
MHI_STATUS process_READY_transition(mhi_device_ctxt *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item);
MHI_STATUS process_RESET_transition(mhi_device_ctxt *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item);
MHI_STATUS process_SYSERR_transition(mhi_device_ctxt *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item);
MHI_STATUS process_BHI_transition(mhi_device_ctxt *mhi_dev_ctxt,
			mhi_state_work_item *cur_work_item);
MHI_STATUS process_AMSS_transition(mhi_device_ctxt *mhi_dev_ctxt,
				mhi_state_work_item *cur_work_item);
MHI_STATUS process_SBL_transition(mhi_device_ctxt *mhi_dev_ctxt,
				mhi_state_work_item *cur_work_item);
MHI_STATUS mhi_wait_for_link_stability(mhi_device_ctxt *mhi_dev_ctxt);
void conditional_db_write(mhi_device_ctxt *mhi_dev_ctxt, u32 chan);

MHI_STATUS mhi_initiate_M3(mhi_device_ctxt *mhi_dev_ctxt);
MHI_STATUS mhi_initiate_M0(mhi_device_ctxt *mhi_dev_ctxt);
enum hrtimer_restart mhi_initiate_M1(struct hrtimer *timer);
int mhi_suspend(struct pci_dev *dev, pm_message_t state);
int mhi_resume(struct pci_dev *dev);
MHI_STATUS probe_clients(mhi_device_ctxt *mhi_dev_ctxt);
int rmnet_mhi_probe(struct pci_dev *dev);
int mhi_shim_probe(struct pci_dev *dev);
int mhi_init_pcie_device(mhi_pcie_dev_info *mhi_pcie_dev);
int mhi_init_gpios(mhi_pcie_dev_info *mhi_pcie_dev);
int mhi_init_pm_sysfs(struct device *dev);
MHI_STATUS mhi_init_timers(mhi_device_ctxt *mhi_dev_ctxt);
void mhi_remove(struct pci_dev *mhi_device);
int mhi_startup_thread(void *ctxt);
int rmnet_mhi_remove(struct pci_dev *dev);

#endif
