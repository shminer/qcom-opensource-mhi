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
typedef void* mhi_rmnet_client_handle;

#define MHI_DMA_MASK       0x3FFFFFFF
#define MHI_MAX_MTU        0xFFFF

/* Return codes for MHI */
typedef enum MHI_RMNET_STATUS_TYPE
{
    MHI_RMNET_STATUS_SUCCESS       = 0x0,
    MHI_RMNET_STATUS_ERROR         = 0x1,
    MHI_RMNET_STATUS_RING_FULL     = 0x3,
    MHI_RMNET_STATUS_reserved = 0xFFFF0000,
}MHI_RMNET_STATUS_TYPE, MHI_RMNET_STATUS;

typedef struct mhi_rmnet_result_t
{
    void* user_data;               /*<-- Pointer to registered user data -->*/
    void* payload;                 /*<-- Pointer to payload -->*/
    uint32_t bytes_xferd;            /*<-- Bytes actually transferred -->*/
    MHI_RMNET_STATUS transaction_status; /*<-- Status of transaction -->*/
} mhi_rmnet_result;

/* Struct containing client callbacks to be invoked */
typedef struct mhi_rmnet_client_cbs_t
{
    /*CB to be invoked upon packet transmission rx/tx*/
    void (*mhi_rmnet_xfer_cb)(mhi_rmnet_result*);
    /*CB to be invoked upon a channel reset */    
    void (*mhi_rmnet_chan_reset_cb)(void* user_data);
}mhi_rmnet_client_cbs;

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

/*  @brief Open a channel with MHI
	@param [IN/OUT] mhi_rmnet_handle handle unique to the requested channel
	@param [IN ]    chan_id    channel number to be opened
	@param [IN ]    user_data  context to be passed back to user in the mhi_rmnet_result struct 
						   upon invocation of the client's callback
	@param [IN ]    cb         function pointer to a client's callback 

@return MHI_RMNET_STATUS
*/
MHI_RMNET_STATUS mhi_rmnet_open_channel(mhi_rmnet_client_handle* mhi_rmnet_handle, 
                    MHI_RMNET_HW_CLIENT_CHANNEL chan_id, void* user_data, 
                    mhi_rmnet_client_cbs* cbs);

/*  @brief Queue buffer with MHI for a particular channel, channel number will 
          indicate if buffer will be used for TX or RX paths 
	@param [IN ] mhi_rmnet_handle valid mhi handle, obtained from mhi_rmnet_open_channel
	@param [IN ] buf        pointer to data buffer
	@param [IN ] len        size of data_buffer
	
@return MHI_RMNET_STATUS
*/
MHI_RMNET_STATUS mhi_rmnet_queue_buffer(mhi_rmnet_client_handle mhi_rmnet_handle, uintptr_t buf, size_t len);

/*	@brief Close the channel with MHI, handle will be destroyed
	@param [IN ] mhi_rmnet_handle valid mhi handle, obtained from mhi_rmnet_open_channel

*/
void mhi_rmnet_close_channel(mhi_rmnet_client_handle mhi_rmnet_handle);


/*  @brief Mask the IRQ associated with the mhi client handle 
    @param [IN ] mhi_rmnet_handle valid mhi handle previously obtained from open_channel
*/
void mhi_rmnet_mask_irq(mhi_rmnet_client_handle mhi_rmnet_handle);

/*  @brief Unmask the IRQ associated with the mhi client handle 
    @param [IN ] mhi_rmnet_handle valid mhi handle previously obtained from open_channel
*/
void mhi_rmnet_unmask_irq(mhi_rmnet_client_handle mhi_rmnet_handle);
/*	@brief Poll for buffers on the channel associated with this mhi client handle
	@param [IN ] mhi_rmnet_handle    Valid mhi handle, obtained from mhi_rmnet_open_channel
        
@return mhi_rmnet_result* Pointer to a mhi_rmnet_result struct to describe the
                    retrieved buffer
*/
mhi_rmnet_result* mhi_rmnet_poll(mhi_rmnet_client_handle mhi_rmnet_handle);
/*	@brief Inform MHI core that the to which the handle maps, needs to be reset.
	@param [IN ] mhi_rmnet_handle    Valid mhi handle, obtained from mhi_rmnet_open_channel
        
		@warning This is an asynchronous request to MHI that the channel should be reset.
				 MHI will complete this request and invoke the client's registered callback
				 if available.
				 Only when the callback is invoked, does the client regain control of
				 any submitted buffers.
				 
@return MHI_RMNET_STATUS
*/
MHI_RMNET_STATUS mhi_rmnet_reset_channel(mhi_rmnet_client_handle mhi_rmnet_handle);


/*	@brief Get the maximum number of buffers this mhi channel can support
	@param [IN ] mhi_rmnet_handle    Valid mhi handle, obtained from mhi_rmnet_open_channel
	
@return number of supported descriptors 
*/
uint32_t mhi_rmnet_get_max_buffers(mhi_rmnet_client_handle mhi_rmnet_handle);


/*	@brief Get the ePID for the particular client handle 
	@param [IN ] mhi_rmnet_handle    Valid mhi handle, obtained from mhi_rmnet_open_channel
	
@return number of supported descriptors 
*/
uint32_t mhi_rmnet_get_epid(mhi_rmnet_client_handle mhi_rmnet_handle);

#endif
