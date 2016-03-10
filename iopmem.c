/*
 * IOPMEM Block Device Driver
 * Copyright (c) 2016, PMC-Sierra, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * This driver is heavily based on drivers/block/pmem.c.
 * Copyright (c) 2014, Intel Corporation.
 * Copyright (C) 2007 Nick Piggin
 * Copyright (C) 2007 Novell Inc.
 */

#include <asm/cacheflush.h>
#include <linux/bio.h>
#include <linux/blkdev.h>
#include <linux/fs.h>
#include <linux/hdreg.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/pfn_t.h>
#include <linux/memremap.h>

#define SECTOR_SHIFT		9
#define PAGE_SECTORS_SHIFT	(PAGE_SHIFT - SECTOR_SHIFT)
#define PAGE_SECTORS		(1 << PAGE_SECTORS_SHIFT)

static int vendor_id = 0x11f8;
static int device_id = 0xf117;
static int bar_id    = 4;

module_param(vendor_id, int, S_IRUGO );
MODULE_PARM_DESC(vendor_id, "The PCIe vendor ID to bind this driver too.");
module_param(device_id, int, S_IRUGO );
MODULE_PARM_DESC(device_id, "The PCIe device ID to bind this driver too.");
module_param(bar_id, int, S_IRUGO );
MODULE_PARM_DESC(bar_id, "The Base Address Register to use to the iopmem binding.");

struct iopmem_device {
	struct request_queue	*queue;
	struct gendisk		*disk;
	struct list_head        list;
	struct pci_dev          *pdev;

	/* One contiguous memory region per device */
	phys_addr_t		phys_addr;
	void			*virt_addr;
	size_t			size;
};

static int iopmem_getgeo(struct block_device *bd, struct hd_geometry *geo)
{
	/* some standard values */
	geo->heads = 1 << 6;
	geo->sectors = 1 << 5;
	geo->cylinders = get_capacity(bd->bd_disk) >> 11;
	return 0;
}

/*
 * direct translation from (iopmem,sector) => void*
 * We do not require that sector be page aligned.
 * The return value will point to the beginning of the page containing the
 * given sector, not to the sector itself.
 */
static void *iopmem_lookup_pg_addr(struct iopmem_device *iopmem, sector_t sector)
{
	size_t page_offset = sector >> PAGE_SECTORS_SHIFT;
	size_t offset = page_offset << PAGE_SHIFT;

	BUG_ON(offset >= iopmem->size);
	return iopmem->virt_addr + offset;
}

/* sector must be page aligned */
static pfn_t iopmem_lookup_pfn(struct iopmem_device *iopmem, sector_t sector)
{
	size_t offset = sector << SECTOR_SHIFT;

	return phys_to_pfn_t(iopmem->phys_addr + offset, PFN_DEV | PFN_MAP);
}

/* we can only access the mtr device with full 32-bit word accesses which cannot
 * be gaurantee'd by the regular memcpy */
static void memcpy_from_iopmem(void *dst, const void *src, size_t sz)
{
	u64 *wdst = dst;
	const u64 *wsrc = src;
	u64 tmp;

	while (sz >= sizeof(*wdst)) {
		*wdst++ = *wsrc++;
		sz -= sizeof(*wdst);
	}

	if (!sz) return;

	tmp = *wsrc;
	memcpy(wdst, &tmp, sz);
}

/*
 * sector is not required to be page aligned.
 * n is at most a single page, but could be less.
 */
static void copy_to_iopmem(struct iopmem_device *iopmem, const void *src,
			sector_t sector, size_t n)
{
	u64 *dst;
	unsigned int offset = (sector & (PAGE_SECTORS - 1)) << SECTOR_SHIFT;
	size_t copy;

	BUG_ON(n > PAGE_SIZE);

	copy = min_t(size_t, n, PAGE_SIZE - offset);
	dst = iopmem_lookup_pg_addr(iopmem, sector);
	memcpy(dst + offset, src, copy);

	if (copy < n) {
		src += copy;
		sector += copy >> SECTOR_SHIFT;
		copy = n - copy;
		dst = iopmem_lookup_pg_addr(iopmem, sector);
		memcpy(dst, src, copy);
	}
}

/*
 * sector is not required to be page aligned.
 * n is at most a single page, but could be less.
 */
static void copy_from_iopmem(void *dst, struct iopmem_device *iopmem,
			  sector_t sector, size_t n)
{
	void *src;
	unsigned int offset = (sector & (PAGE_SECTORS - 1)) << SECTOR_SHIFT;
	size_t copy;

	BUG_ON(n > PAGE_SIZE);

	copy = min_t(size_t, n, PAGE_SIZE - offset);
	src = iopmem_lookup_pg_addr(iopmem, sector);

	memcpy_from_iopmem(dst, src + offset, copy);

	if (copy < n) {
		dst += copy;
		sector += copy >> SECTOR_SHIFT;
		copy = n - copy;
		src = iopmem_lookup_pg_addr(iopmem, sector);
		memcpy_from_iopmem(dst, src, copy);
	}
}

static void iopmem_do_bvec(struct iopmem_device *iopmem, struct page *page,
			unsigned int len, unsigned int off, int rw,
			sector_t sector)
{
	void *mem = kmap_atomic(page);

	if (rw == READ) {
		copy_from_iopmem(mem + off, iopmem, sector, len);
		flush_dcache_page(page);
	} else {
		/*
		 * FIXME: Need more involved flushing to ensure that writes to
		 * NVDIMMs are actually durable before returning.
		 */
		flush_dcache_page(page);
		copy_to_iopmem(iopmem, mem + off, sector, len);
	}

	kunmap_atomic(mem);
}

static blk_qc_t iopmem_make_request(struct request_queue *q, struct bio *bio)
{
	struct block_device *bdev = bio->bi_bdev;
	struct iopmem_device *iopmem = bdev->bd_disk->private_data;
	int rw;
	struct bio_vec bvec;
	sector_t sector;
	struct bvec_iter iter;
	int err = 0;

	sector = bio->bi_iter.bi_sector;
	if (bio_end_sector(bio) > get_capacity(bdev->bd_disk)) {
		err = -EIO;
		goto out;
	}

	BUG_ON(bio->bi_rw & REQ_DISCARD);

	rw = bio_rw(bio);
	if (rw == READA)
		rw = READ;

	bio_for_each_segment(bvec, bio, iter) {
		unsigned int len = bvec.bv_len;

		BUG_ON(len > PAGE_SIZE);
		iopmem_do_bvec(iopmem, bvec.bv_page, len,
			    bvec.bv_offset, rw, sector);
		sector += len >> SECTOR_SHIFT;
	}

out:
	bio_endio(bio);
	return BLK_QC_T_NONE;
}

static int iopmem_rw_page(struct block_device *bdev, sector_t sector,
		       struct page *page, int rw)
{
	struct iopmem_device *iopmem = bdev->bd_disk->private_data;

	iopmem_do_bvec(iopmem, page, PAGE_CACHE_SIZE, 0, rw, sector);
	page_endio(page, rw & WRITE, 0);
	return 0;
}

static long iopmem_direct_access(struct block_device *bdev, sector_t sector,
			       void __pmem **kaddr, pfn_t *pfn)
{
	struct iopmem_device *iopmem = bdev->bd_disk->private_data;

	if (!iopmem)
		return -ENODEV;

	*kaddr = iopmem_lookup_pg_addr(iopmem, sector);
	*pfn = iopmem_lookup_pfn(iopmem, sector);

	return iopmem->size - (sector * 512);
}

static const struct block_device_operations iopmem_fops = {
	.owner =		THIS_MODULE,
	.rw_page =		iopmem_rw_page,
	.direct_access =	iopmem_direct_access,
	.getgeo =		iopmem_getgeo,
};

/* Kernel module stuff */
static LIST_HEAD(iopmem_devices);
static int iopmem_major;

static struct iopmem_device *iopmem_alloc(struct pci_dev *pdev, int i)
{
	struct iopmem_device *iopmem;
	struct gendisk *disk;
	int err = 0;

	if (pci_enable_device_mem(pdev) < 0) {
		pr_err("iopmem: unable to enable device!\n");
		goto out;
	}

	iopmem = kzalloc(sizeof(*iopmem), GFP_KERNEL);
	if (unlikely(!iopmem)) {
		err = -ENOMEM;
		goto out_disable_device;
	}


	iopmem->phys_addr = pci_resource_start(pdev, bar_id);
	iopmem->size = pci_resource_end(pdev, bar_id) - iopmem->phys_addr + 1;
	iopmem->pdev = pdev;

	pr_info("iopmem%d: bar space 0x%llx len %lld\n", i+1,
		(unsigned long long) iopmem->phys_addr,
		(unsigned long long) iopmem->size);


	// Should probably call devm_request_mem_region, but nvme driver
	//  has already reserved it.

	iopmem->queue = blk_alloc_queue(GFP_KERNEL);
	if (unlikely(!iopmem->queue)) {
		err = -ENOMEM;
		goto out_free_dev;
	}

	iopmem->virt_addr = devm_memremap_pages(&pdev->dev, &pdev->resource[bar_id],
					      &iopmem->queue->q_usage_counter, NULL,
					      MEMREMAP_WC);

	if (IS_ERR(iopmem->virt_addr)) {
		err = -ENXIO;
		goto out_free_queue;
	}

	blk_queue_make_request(iopmem->queue, iopmem_make_request);
	blk_queue_max_hw_sectors(iopmem->queue, 1024);
	blk_queue_bounce_limit(iopmem->queue, BLK_BOUNCE_ANY);

	disk = alloc_disk(0);
	if (unlikely(!disk)) {
		err = -ENOMEM;
		goto out_free_queue;
	}

	disk->major		= iopmem_major;
	disk->first_minor	= 0;
	disk->fops		= &iopmem_fops;
	disk->private_data	= iopmem;
	disk->driverfs_dev      = &pdev->dev;
	disk->queue		= iopmem->queue;
	disk->flags		= GENHD_FL_EXT_DEVT;
	sprintf(disk->disk_name, "iopmem%d", i+1);
	set_capacity(disk, iopmem->size >> SECTOR_SHIFT);
	iopmem->disk = disk;

	return iopmem;

out_free_queue:
	blk_cleanup_queue(iopmem->queue);
out_free_dev:
	kfree(iopmem);
out_disable_device:
	pci_disable_device(pdev);
out:
	return ERR_PTR(err);
}

static void iopmem_free(struct iopmem_device *iopmem)
{
	put_disk(iopmem->disk);
	blk_cleanup_queue(iopmem->queue);
	kfree(iopmem);
	pci_disable_device(iopmem->pdev);
}

static void iopmem_del_one(struct iopmem_device *iopmem)
{
	list_del(&iopmem->list);
	del_gendisk(iopmem->disk);
	iopmem_free(iopmem);
}

static int __init iopmem_init(void)
{
	int result;
	struct iopmem_device *iopmem, *next;
	int ndevs = 0;
	struct pci_dev *pdev = NULL;

	result = register_blkdev(0, "iopmem");
	if (result < 0)
		return -EIO;
	else
		iopmem_major = result;

	while ((pdev = pci_get_device(vendor_id, device_id, pdev))) {
		iopmem = iopmem_alloc(pdev, ndevs);
		if (IS_ERR(iopmem)) {
			result = PTR_ERR(iopmem);
			goto out_free;
		}
		list_add_tail(&iopmem->list, &iopmem_devices);
		ndevs++;
	}

	if (ndevs == 0) {
		result = -ENODEV;
		goto out_free;
	}

	list_for_each_entry(iopmem, &iopmem_devices, list)
		add_disk(iopmem->disk);

	pr_info("iopmem: module loaded\n");
	return 0;

out_free:
	list_for_each_entry_safe(iopmem, next, &iopmem_devices, list) {
		list_del(&iopmem->list);
		iopmem_free(iopmem);
	}
	unregister_blkdev(iopmem_major, "iopmem");

	return result;
}

static void __exit iopmem_exit(void)
{
	struct iopmem_device *iopmem, *next;

	list_for_each_entry_safe(iopmem, next, &iopmem_devices, list)
		iopmem_del_one(iopmem);

	unregister_blkdev(iopmem_major, "iopmem");
	pr_info("iopmem: module unloaded\n");
}

MODULE_AUTHOR("Logan Gunthorpe <logang@deltatee.com>");
MODULE_LICENSE("GPL");
module_init(iopmem_init);
module_exit(iopmem_exit);
