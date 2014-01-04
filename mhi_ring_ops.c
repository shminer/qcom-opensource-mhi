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
#include "mhi.h"
#include "mhi_sys.h"

/**
* @brief Returns a pointer to the next free location in
* the ring for the calee to populate
*	 and moves the write pointer of the ring.
*
*@param[in ]	ring_props	pointer to the ring
*@param[in ]	element_size	size of the element to insert
*@param[out]	assigned_addr	address returned to the calee
*				for element population
*@param[in ]	ring	location of local register set for the ring
*/
MHI_STATUS add_element(mhi_ring *ring, void **assigned_addr)
{
	uintptr_t d_wp = 0, d_rp = 0, ring_size = 0;

	if (0 == ring->el_size || NULL == ring
		|| NULL == ring->base || 0 == ring->len) {
		mhi_log(MHI_MSG_ERROR, "Bad input parameters, quitting.\n");
		return MHI_STATUS_ERROR;
	}

	d_wp = ((uintptr_t)ring->wp - (uintptr_t)ring->base)/ring->el_size;
	d_rp = ((uintptr_t)ring->rp - (uintptr_t)ring->base)/ring->el_size;
	ring_size = ring->len / ring->el_size;

	if ((d_wp + 1) % ring_size == d_rp) {
		if (ring->overwrite_en) {
			delete_element(ring, NULL);
		} else {
			mhi_log(MHI_MSG_INFO, "Ring 0x%lX is full\n",
					(uintptr_t)ring->base);
			return MHI_STATUS_RING_FULL;
		}
	}
	if (NULL != assigned_addr)
		*assigned_addr = (char *)ring->wp;
	ring->wp = (void *)(((d_wp + 1) % ring_size) * ring->el_size +
						(uintptr_t)ring->base);
	return MHI_STATUS_SUCCESS;
}
/**
*@brief Moves the read pointer of the transfer ring to
*the next element of the transfer ring,
*	as the element has been consumed by the mhi client.
*
*@param[in ] ring location of local register set for the ring
*@param[out] assigned_addr address returned to the calee with element
*/
MHI_STATUS delete_element(mhi_ring *ring, void **assigned_addr)
{
	uintptr_t d_wp = 0, d_rp = 0, ring_size = 0;

	if (0 == ring->el_size || NULL == ring ||
		NULL == ring->base || 0 == ring->len) {
		mhi_log(MHI_MSG_ERROR, "Bad input parameters, quitting.\n");
		return MHI_STATUS_ERROR;
	}
	d_wp = ((uintptr_t)ring->wp - (uintptr_t)ring->base) / ring->el_size;
	d_rp = ((uintptr_t)ring->rp - (uintptr_t)ring->base) / ring->el_size;
	ring_size = ring->len / ring->el_size;

	if (d_wp == d_rp) {
		mhi_log(MHI_MSG_INFO, "Ring 0x%lx is empty\n",
				(uintptr_t)ring->base);
		return MHI_STATUS_RING_EMPTY;
	}

	if (NULL != assigned_addr)
		*assigned_addr = (void *)ring->rp;

	ring->rp = (void *)(((d_rp + 1) % ring_size) * ring->el_size +
						(uintptr_t)ring->base);
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS get_element_index(mhi_ring *ring, void *address, uintptr_t *index)
{
	/* 1. Check the bounds of this address to ensure it is in the ring.*/
	if (MHI_STATUS_SUCCESS != validate_ring_el_addr(ring,
							(uintptr_t)address))
		return MHI_STATUS_ERROR;
	*index = ((uintptr_t)address - (uintptr_t)ring->base) / ring->el_size;
	return MHI_STATUS_SUCCESS;
}

/**
 * @brief Returns the number of free ring elements.
 *
 * @param[in ]	ring	location of local register set for the ring
 * @param[out]	nr_of_avail_trbs returns the number of free elements
 */
int get_free_trbs(mhi_client_handle *client_handle)
{
	u32 chan = client_handle->chan;
	mhi_device_ctxt *ctxt = client_handle->mhi_dev_ctxt;
	return get_nr_avail_ring_elements(&ctxt->mhi_local_chan_ctxt[chan]);
}
int get_nr_avail_ring_elements(mhi_ring *ring)
{
	uintptr_t d_wp = 0, d_rp = 0, ring_size = 0;

	if (0 == ring->el_size || NULL == ring ||
			NULL == ring->base || 0 == ring->len) {
		mhi_log(MHI_MSG_ERROR, "Bad input parameters, quitting.\n");
		return MHI_STATUS_ERROR;
	}

	d_wp = ((uintptr_t)ring->wp - (uintptr_t)ring->base) / ring->el_size;
	d_rp = ((uintptr_t)ring->rp - (uintptr_t)ring->base) / ring->el_size;
	ring_size = ring->len / ring->el_size;

	if ((d_wp + 1) % ring_size == d_rp) { /* Ring full */

		return 0;
	} else {
		if (d_wp < d_rp)
			return d_rp - d_wp - 1;
		else
			return ring_size - (d_wp - d_rp) - 1;
	}
}
