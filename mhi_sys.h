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

#ifndef _H_OSAL_
#define _H_OSAL_
#include "mhi.h"
#include <linux/mutex.h> /* mutex_t */
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/slab.h> /* kmalloc, kfree */
#include <linux/completion.h> /* completions */
#include <linux/io.h> /*readl_relaxed(), writel_relaxed() */
#include <linux/slab.h> /* GFP - general purpose allocator */
#include <linux/kthread.h>
#include <linux/wait.h> /* wait_event */
#include <linux/dma-mapping.h> /* dma_alloc_coherent */
#include <linux/delay.h>
#include <asm/barrier.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/device.h> /* struct device */
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/jiffies.h>
#include <linux/sysfs.h>
#include <linux/hrtimer.h>
#include <linux/spinlock_types.h>
#include <linux/pm.h>
#include <mach/msm_pcie.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>

extern MHI_DEBUG_LEVEL mhi_msg_lvl;
extern MHI_DEBUG_CLASS mhi_msg_class;

#define MHI_ASSERT(_x) if (!(_x)) {\
	pr_err("ASSERT- Failure in %s:%d/%s()!\n",\
			__FILE__, __LINE__, __func__); \
	panic("ASSERT"); \
}

#define mhi_log(_msg_lvl, _msg, ...) do { \
		if ((_msg_lvl) >= mhi_msg_lvl) \
			pr_info("[%s] " _msg, __func__, ##__VA_ARGS__);\
} while (0)

#define pcie_read(base, offset)	 readl_relaxed((volatile void *) \
				(uintptr_t)(base+offset))
#define pcie_write(base, offset, val) writel_relaxed(val,\
			(volatile void *)(uintptr_t)(base + offset))

irq_handler_t irq_cb(int msi_number, void *dev_id);

struct mhi_meminfo {
	struct device *dev;
	uintptr_t pa_aligned;
	uintptr_t pa_unaligned;
	uintptr_t va_aligned;
	uintptr_t va_unaligned;
	uintptr_t size;
};

struct osal_thread {
	struct task_struct *thread_handle;
	void *data;
	const char *name_fmt;
};

MHI_STATUS mhi_spawn_thread(void *ctxt, int(fn)(void *),
			osal_thread *handle, char[]);

MHI_STATUS mhi_mallocmemregion(mhi_meminfo *meminfo, size_t size);

uintptr_t mhi_get_phy_addr(mhi_meminfo *meminfo);
void *mhi_get_virt_addr(mhi_meminfo *meminfo);
uintptr_t mhi_p2v_addr(mhi_meminfo *meminfo, uintptr_t pa);
uintptr_t mhi_v2p_addr(mhi_meminfo *meminfo, uintptr_t va);
u64 mhi_get_memregion_len(mhi_meminfo *meminfo);
void mhi_freememregion(mhi_meminfo *meminfo);

void print_ring(mhi_ring *local_chan_ctxt, u32 ring_id);
int mhi_init_debugfs(mhi_device_ctxt *mhi_dev_ctxt);
int mhi_probe(struct pci_dev *mhi_device,
		const struct pci_device_id *mhi_device_id);
ssize_t sysfs_init_M3(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t sysfs_init_M0(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t sysfs_init_M1(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t sysfs_get_mhi_state(struct device *dev, struct device_attribute *attr,
			char *buf);

#endif
