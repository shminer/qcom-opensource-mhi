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
MHI_DEBUG_LEVEL mhi_msg_lvl = MHI_MSG_VERBOSE;

module_param(mhi_msg_lvl , uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(mhi_msg_lvl, "dbg lvl");

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

MHI_STATUS mhi_trigger_event(osal_event *handle)
{
	if (NULL == handle)
		return MHI_STATUS_ERROR;
	wake_up(&handle->event);
	return MHI_STATUS_SUCCESS;
}
inline void mhi_memcpy(void *to, void *from, size_t size)
{
	memcpy(to, from, size);
}

inline u64 mhi_get_memregion_len(mhi_meminfo *meminfo)
{
	return meminfo->size;
}

MHI_STATUS mhi_init_event(osal_event *handle)
{
	if (NULL == handle)
		return MHI_STATUS_ERROR;
	init_waitqueue_head(&handle->event);

	return MHI_STATUS_SUCCESS;
}
void *mhi_malloc(size_t buf_size)
{
	/* This is a virtual malloc, we don't need physical contiguity */
	return vmalloc(buf_size);
}
void mhi_free(void *data)
{
	vfree(data);
}
void mhi_memset(void *ptr, int value, size_t num)
{
	memset(ptr, value, num);
}
MHI_STATUS mhi_init_mutex(osal_mutex *mutex)
{
	if (NULL == mutex)
		return MHI_STATUS_ERROR;
	mutex_init(&mutex->lock);
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS mhi_release_mutex(osal_mutex *mutex)
{
	if (NULL == mutex)
		return MHI_STATUS_ERROR;
	mutex_unlock(&mutex->lock);
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS mhi_acquire_mutex(osal_mutex *mutex)
{
	if (NULL == mutex)
		return MHI_STATUS_ERROR;
	mutex_lock(&mutex->lock);
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS mhi_init_spinlock(osal_spinlock *spinlock)
{
	if (NULL == spinlock)
		return MHI_STATUS_ERROR;
	spin_lock_init(&spinlock->lock);
	return MHI_STATUS_SUCCESS;
}

MHI_STATUS mhi_acquire_spinlock(osal_spinlock *spinlock)
{
	if (NULL == spinlock)
		return MHI_STATUS_ERROR;
	spin_lock(&spinlock->lock);
	return MHI_STATUS_SUCCESS;
}
MHI_STATUS mhi_release_spinlock(osal_spinlock *spinlock)
{
	if (NULL == spinlock)
		return MHI_STATUS_ERROR;
	spin_unlock(&spinlock->lock);
	return MHI_STATUS_SUCCESS;
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
inline void mhi_sleep(u32 time_ms)
{
	msleep(time_ms);
}
