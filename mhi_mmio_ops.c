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
#include "mhi_hwio.h"
#include "mhi.h"
/**
 * @brief Test if the device is ready
 *
 * @param device[IN ] device context
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_test_for_device_ready(mhi_device_ctxt* device)
{
	u32 pcie_word_val = 0;
	u32 expiry_counter;
	mhi_log(MHI_MSG_INFO,"Waiting for MMIO Ready bit to be set\n");

	/* Read MMIO and poll for READY bit to be set */
	pcie_word_val = pcie_read (device->mmio_addr,MHISTATUS);
	MHI_READ_FIELD(pcie_word_val,MHISTATUS_READY_MASK,MHISTATUS_READY_SHIFT);

	if (0xFFFFFFFF == pcie_word_val)
		return MHI_STATUS_LINK_DOWN;
	expiry_counter = 0;
	while (MHI_STATE_READY != pcie_word_val && expiry_counter < 30)
	{

		expiry_counter++;
		mhi_log(MHI_MSG_ERROR, "Device is not ready, sleeping and retrying.\n");
		mhi_sleep(MHI_READY_STATUS_TIMEOUT_MS);
		pcie_word_val = pcie_read (device->mmio_addr,MHISTATUS);
		MHI_READ_FIELD(pcie_word_val,MHISTATUS_READY_MASK,MHISTATUS_READY_SHIFT);
	}

	if (MHI_STATE_READY != pcie_word_val)
	{
		return MHI_STATUS_DEVICE_NOT_READY;
	}
	return MHI_STATUS_SUCCESS;
}
/**
 * @brief Main function to initialize the MMIO on the slave and begin MHI operation.
 *
 * @param device
 *
 * @return MHI_STATUS
 */
MHI_STATUS mhi_init_mmio(mhi_device_ctxt* device)
{
	u64 pcie_bar0_addr = 0;

	u64 pcie_dword_val = 0;
	u32 pcie_word_val = 0;

	u32 i	 = 0;

	mhi_log(MHI_MSG_INFO,"~~~ Initializing MMIO ~~~\n");
	pcie_bar0_addr = device->dev_props.bar0_base;

	device->mmio_addr = pcie_bar0_addr;
	mhi_log(MHI_MSG_INFO,"Bar 0 address is at: 0x%lX\n",
			(device->mmio_addr));

	device->mmio_len = pcie_read(device->mmio_addr,MHIREGLEN);

	if (0 == device->mmio_len)
	{
		mhi_log(MHI_MSG_ERROR,"Received mmio length as zero\n");
		return MHI_STATUS_ERROR;
	}

	mhi_log(MHI_MSG_INFO,"Testing MHI Ver\n");
	device->dev_props.mhi_ver = pcie_read(device->mmio_addr,MHIVER);

	if (MHI_VERSION != device->dev_props.mhi_ver)
	{
		mhi_log(MHI_MSG_CRITICAL,"Bad MMIO version\n");
	}
	/* Enable the channels */
	for (i = 0; i < MHI_MAX_CHANNELS; ++i)
	{
		if ((i >= MHI_CLIENT_RESERVED_1_LOWER && i <= MHI_CLIENT_RESERVED_1_UPPER) ||
				(i >= MHI_CLIENT_IP_HW_1_OUT && i <= MHI_CLIENT_RESERVED_2_UPPER))
		{
			device->mhi_ctrl_seg->mhi_cc_list[i].mhi_chan_state = MHI_CHAN_STATE_DISABLED;
			device->mhi_local_chan_ctxt[i].mhi_chan_state = MHI_CHAN_STATE_DISABLED;
		}
		else
		{
			device->mhi_ctrl_seg->mhi_cc_list[i].mhi_chan_state = MHI_CHAN_STATE_ENABLED;
			device->mhi_local_chan_ctxt[i].mhi_chan_state = MHI_CHAN_STATE_ENABLED;
		}
	}
	mhi_log(MHI_MSG_INFO,"Read back MMIO Ready bit successfully. Moving on..\n");
	mhi_log(MHI_MSG_INFO,"Reading channel doorbell offset\n");

	MHI_REG_READ_FIELD(device->mmio_addr,CHDBOFF,CHDBOFF_CHDBOFF_MASK,
			CHDBOFF_CHDBOFF_SHIFT,device->channel_db_addr);

	mhi_log(MHI_MSG_INFO,"Reading event doorbell offset\n");
	MHI_REG_READ_FIELD(device->mmio_addr,ERDBOFF,ERDBOFF_ERDBOFF_MASK,
			ERDBOFF_ERDBOFF_SHIFT,device->event_db_addr);

	device->channel_db_addr += device->mmio_addr;
	device->event_db_addr += device->mmio_addr;

	mhi_log(MHI_MSG_INFO, "chan_db_addr = %llx,"
			" event_db_address = %llx\n",
			device->channel_db_addr, device->event_db_addr);
	mhi_log(MHI_MSG_INFO,"Setting all MMIO values.\n");

	pcie_dword_val = mhi_v2p_addr(device->mhi_ctrl_seg_info,
			(uintptr_t)device->mhi_ctrl_seg->mhi_cc_list);
	pcie_word_val = HIGH_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr, CCABAP_HIGHER, CCABAP_HIGHER_CCABAP_HIGHER_MASK,
			CCABAP_HIGHER_CCABAP_HIGHER_SHIFT, pcie_word_val);
	pcie_word_val = LOW_WORD(pcie_dword_val);

	MHI_REG_WRITE_FIELD(device->mmio_addr, CCABAP_LOWER, CCABAP_LOWER_CCABAP_LOWER_MASK,
			CCABAP_LOWER_CCABAP_LOWER_SHIFT, pcie_word_val);


	/* Write the Event Context Base Address Register High and Low parts */
	pcie_dword_val = mhi_v2p_addr(device->mhi_ctrl_seg_info,
			(uintptr_t)device->mhi_ctrl_seg->mhi_ec_list);
	pcie_word_val = HIGH_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr, ECABAP_HIGHER, ECABAP_HIGHER_ECABAP_HIGHER_MASK,
			ECABAP_HIGHER_ECABAP_HIGHER_SHIFT,pcie_word_val);
	pcie_word_val = LOW_WORD(pcie_dword_val);

	MHI_REG_WRITE_FIELD(device->mmio_addr, ECABAP_LOWER, ECABAP_LOWER_ECABAP_LOWER_MASK,
			ECABAP_LOWER_ECABAP_LOWER_SHIFT,pcie_word_val);


	/* Write the Command Ring Control Register High and Low parts */
	pcie_dword_val = mhi_v2p_addr(device->mhi_ctrl_seg_info,
			(uintptr_t)device->mhi_ctrl_seg->mhi_cmd_ctxt_list);
	pcie_word_val = HIGH_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr, CRCBAP_HIGHER, CRCBAP_HIGHER_CRCBAP_HIGHER_MASK,
			CRCBAP_HIGHER_CRCBAP_HIGHER_SHIFT,
			pcie_word_val);
	pcie_word_val = LOW_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr, CRCBAP_LOWER, CRCBAP_LOWER_CRCBAP_LOWER_MASK,
			CRCBAP_LOWER_CRCBAP_LOWER_SHIFT,
			pcie_word_val);


	device->cmd_db_addr = device->mmio_addr + CRDB_LOWER;
	/* Set the control segment in the MMIO */
	pcie_dword_val = mhi_v2p_addr(device->mhi_ctrl_seg_info,
				(uintptr_t)device->mhi_ctrl_seg);
	pcie_word_val = HIGH_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr,MHICTRLBASE_HIGHER,
			MHICTRLBASE_HIGHER_MHICTRLBASE_HIGHER_MASK,
			MHICTRLBASE_HIGHER_MHICTRLBASE_HIGHER_SHIFT,
			pcie_word_val);

	pcie_word_val = LOW_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr,MHICTRLBASE_LOWER,
			MHICTRLBASE_LOWER_MHICTRLBASE_LOWER_MASK,
			MHICTRLBASE_LOWER_MHICTRLBASE_LOWER_SHIFT,
			pcie_word_val);

	pcie_dword_val = mhi_v2p_addr(device->mhi_ctrl_seg_info,
				(uintptr_t)device->mhi_ctrl_seg) +
		mhi_get_memregion_len(device->mhi_ctrl_seg_info) - 1;

	pcie_word_val = HIGH_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr, MHICTRLLIMIT_HIGHER,
			MHICTRLLIMIT_HIGHER_MHICTRLLIMIT_HIGHER_MASK,
			MHICTRLLIMIT_HIGHER_MHICTRLLIMIT_HIGHER_SHIFT,
			pcie_word_val);
	pcie_word_val = LOW_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr, MHICTRLLIMIT_LOWER,
			MHICTRLLIMIT_LOWER_MHICTRLLIMIT_LOWER_MASK,
			MHICTRLLIMIT_LOWER_MHICTRLLIMIT_LOWER_SHIFT,
			pcie_word_val);

	/* Set the data segment in the MMIO */
	pcie_dword_val = MHI_DATA_SEG_WINDOW_START_ADDR;
	pcie_word_val = HIGH_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr, MHIDATABASE_HIGHER,
			MHIDATABASE_HIGHER_MHIDATABASE_HIGHER_MASK,
			MHIDATABASE_HIGHER_MHIDATABASE_HIGHER_SHIFT,
			pcie_word_val);

	pcie_word_val = LOW_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr, MHIDATABASE_LOWER,
			MHIDATABASE_LOWER_MHIDATABASE_LOWER_MASK,
			MHIDATABASE_LOWER_MHIDATABASE_LOWER_SHIFT,
			pcie_word_val);

	pcie_dword_val = MHI_DATA_SEG_WINDOW_END_ADDR;

	pcie_word_val = HIGH_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr, MHIDATALIMIT_HIGHER,
			MHIDATALIMIT_HIGHER_MHIDATALIMIT_HIGHER_MASK,
			MHIDATALIMIT_HIGHER_MHIDATALIMIT_HIGHER_SHIFT,
			(pcie_word_val));
	pcie_word_val = LOW_WORD(pcie_dword_val);
	MHI_REG_WRITE_FIELD(device->mmio_addr, MHIDATALIMIT_LOWER,
			MHIDATALIMIT_LOWER_MHIDATALIMIT_LOWER_MASK,
			MHIDATALIMIT_LOWER_MHIDATALIMIT_LOWER_SHIFT,
			(pcie_word_val));

	mhi_log(MHI_MSG_INFO,"Done..\n");
	return MHI_STATUS_SUCCESS;
}

