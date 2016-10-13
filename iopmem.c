/*
 * IOPMEM Block Device Driver
 * Copyright (c) 2016, Microsemi Corporation
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
#include <linux/cdev.h>

#define SECTOR_SHIFT		9
#define PAGE_SECTORS_SHIFT	(PAGE_SHIFT - SECTOR_SHIFT)
#define PAGE_SECTORS		(1 << PAGE_SECTORS_SHIFT)

static struct class *iopmemc_class;
static dev_t char_device_num;
static int max_devices = 16;

static const int BAR_ID = 4;

static struct pci_device_id iopmem_id_table[] = {
	{ PCI_DEVICE(0x11f8, 0xf118) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, iopmem_id_table);

module_param(max_devices, int, S_IRUGO);
MODULE_PARM_DESC(max_devices, "Maximum number of char devices");

struct iopmem_device {
	struct request_queue *queue;
	struct gendisk *disk;
	struct device *dev;

	int instance;

	int cdev_num;
	struct cdev cdev;

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

/* we can only access the iopmem device with full 32-bit word accesses which cannot
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
		flush_dcache_page(page);
		copy_to_iopmem(iopmem, mem + off, sector, len);
	}

	kunmap_atomic(mem);
}

static blk_qc_t iopmem_make_request(struct request_queue *q, struct bio *bio)
{
	struct iopmem_device *iopmem = q->queuedata;
	struct bio_vec bvec;
	struct bvec_iter iter;

	bio_for_each_segment(bvec, bio, iter) {
		iopmem_do_bvec(iopmem, bvec.bv_page, bvec.bv_len,
			    bvec.bv_offset, op_is_write(bio_op(bio)),
			    iter.bi_sector);
	}

	bio_endio(bio);
	return BLK_QC_T_NONE;
}

static int iopmem_rw_page(struct block_device *bdev, sector_t sector,
		       struct page *page, bool is_write)
{
	struct iopmem_device *iopmem = bdev->bd_queue->queuedata;

	iopmem_do_bvec(iopmem, page, PAGE_SIZE, 0, is_write, sector);
	page_endio(page, is_write, 0);
	return 0;
}

static long iopmem_direct_access(struct block_device *bdev, sector_t sector,
			       void **kaddr, pfn_t *pfn, long size)
{
	struct iopmem_device *iopmem = bdev->bd_queue->queuedata;

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

static int iopmemc_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct iopmem_device *iopmem = vma->vm_private_data;
	unsigned long physaddr;
	int error;
	unsigned long vaddr = (unsigned long) vmf->virtual_address;
	pfn_t pfn;

	physaddr = (vaddr - vma->vm_start +
		    (vma->vm_pgoff << PAGE_SHIFT) + iopmem->phys_addr);
	pfn = phys_to_pfn_t(physaddr, PFN_DEV | PFN_MAP);

	if (!pfn_t_valid(pfn))
		return VM_FAULT_SIGBUS;

	if ((error = vm_insert_mixed(vma, vaddr, pfn))) {
		dev_err(iopmem->dev,
			"Unable to insert mixed page into mapping :%d\n",
			error);

		return VM_FAULT_SIGBUS;
	}

	return VM_FAULT_NOPAGE;
}

const struct vm_operations_struct vmops = {
	.fault = iopmemc_fault,
};

static int iopmemc_open(struct inode *inode, struct file *filp)
{
	struct iopmem_device *iopmem;

	iopmem = container_of(inode->i_cdev, struct iopmem_device, cdev);
	filp->private_data = iopmem;

	inode->i_size = iopmem->size;

	return 0;
}

static int iopmemc_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct iopmem_device *iopmem = filp->private_data;

	phys_addr_t end = ((vma->vm_pgoff << PAGE_SHIFT) +
			   vma->vm_end - vma->vm_start);

	if (end > (iopmem->phys_addr + 1))
		return -EINVAL;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_private_data = iopmem;
	vma->vm_ops = &vmops;
	vma->vm_flags |= VM_MIXEDMAP;

	return 0;
}

static const struct file_operations iopmemc_fops = {
	.owner = THIS_MODULE,
	.open = iopmemc_open,
	.mmap = iopmemc_mmap,
};

static DEFINE_IDA(iopmem_instance_ida);
static DEFINE_SPINLOCK(ida_lock);

static int iopmem_set_instance(struct iopmem_device *iopmem)
{
	int instance, error;

	do {
		if (!ida_pre_get(&iopmem_instance_ida, GFP_KERNEL))
			return -ENODEV;

		spin_lock(&ida_lock);
		error = ida_get_new(&iopmem_instance_ida, &instance);
		spin_unlock(&ida_lock);

	} while (error == -EAGAIN);

	if (error)
		return -ENODEV;

	iopmem->instance = instance;
	return 0;
}

static void iopmem_release_instance(struct iopmem_device *iopmem)
{
	spin_lock(&ida_lock);
	ida_remove(&iopmem_instance_ida, iopmem->instance);
	spin_unlock(&ida_lock);
}

static int iopmem_attach_disk(struct iopmem_device *iopmem)
{
	struct gendisk *disk;
	int nid = dev_to_node(iopmem->dev);
	struct request_queue *q = iopmem->queue;

	blk_queue_write_cache(q, true, true);
	blk_queue_make_request(q, iopmem_make_request);
	blk_queue_physical_block_size(q, PAGE_SIZE);
	blk_queue_max_hw_sectors(q, UINT_MAX);
	blk_queue_bounce_limit(q, BLK_BOUNCE_ANY);
	queue_flag_set_unlocked(QUEUE_FLAG_NONROT, q);
	queue_flag_set_unlocked(QUEUE_FLAG_DAX, q);
	q->queuedata = iopmem;

	disk = alloc_disk_node(0, nid);
	if (unlikely(!disk))
		return -ENOMEM;

	disk->fops		= &iopmem_fops;
	disk->queue		= q;
	disk->flags		= GENHD_FL_EXT_DEVT;
	sprintf(disk->disk_name, "iopmem%d", iopmem->instance);
	set_capacity(disk, iopmem->size >> SECTOR_SHIFT);
	iopmem->disk = disk;

	device_add_disk(iopmem->dev, disk);
	revalidate_disk(disk);

	return 0;
}

static void iopmem_detach_disk(struct iopmem_device *iopmem)
{
	del_gendisk(iopmem->disk);
	put_disk(iopmem->disk);
}

static int iopmem_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct iopmem_device *iopmem;
	struct device *dev;
	struct device *char_sys_dev;
	int err = 0;
	int nid = dev_to_node(&pdev->dev);

	if (pci_enable_device_mem(pdev) < 0) {
		dev_err(&pdev->dev, "unable to enable device!\n");
		goto out;
	}

	iopmem = kzalloc(sizeof(*iopmem), GFP_KERNEL);
	if (unlikely(!iopmem)) {
		err = -ENOMEM;
		goto out_disable_device;
	}

	iopmem->phys_addr = pci_resource_start(pdev, BAR_ID);
	iopmem->size = pci_resource_end(pdev, BAR_ID) - iopmem->phys_addr + 1;
	iopmem->dev = dev = get_device(&pdev->dev);
	pci_set_drvdata(pdev, iopmem);

	if ((err = iopmem_set_instance(iopmem)))
		goto out_put_device;

	dev_info(dev, "bar space 0x%llx len %lld\n",
		(unsigned long long) iopmem->phys_addr,
		(unsigned long long) iopmem->size);

	if (!devm_request_mem_region(dev, iopmem->phys_addr,
				     iopmem->size, dev_name(dev))) {
		dev_warn(dev, "could not reserve region [0x%pa:0x%zx]\n",
			 &iopmem->phys_addr, iopmem->size);
		err = -EBUSY;
		goto out_release_instance;
	}

	iopmem->queue = blk_alloc_queue_node(GFP_KERNEL, nid);
	if (!iopmem->queue) {
		err = -ENOMEM;
		goto out_release_instance;
	}

	iopmem->virt_addr = devm_memremap_pages(dev, &pdev->resource[BAR_ID],
				&iopmem->queue->q_usage_counter,
				NULL, MEMREMAP_WC);
	if (IS_ERR(iopmem->virt_addr)) {
		err = -ENXIO;
		goto out_free_queue;
	}

	cdev_init(&iopmem->cdev, &iopmemc_fops);
	iopmem->cdev_num = MKDEV(MAJOR(char_device_num), iopmem->instance);
	if ((err = cdev_add(&iopmem->cdev, iopmem->cdev_num, 1))) {
		dev_err(dev, "failed to create char device!\n");
		goto out_free_queue;
	}

        char_sys_dev = device_create(iopmemc_class, dev,
				     iopmem->cdev_num,
				     iopmem, "iopmemc%d",
				     iopmem->instance);
	if (IS_ERR(char_sys_dev)) {
		err = -EFAULT;
		dev_err(dev, "failed to create iopmemc device!\n");
		goto out_free_chdev;
	}

	if ((err = iopmem_attach_disk(iopmem)))
		goto out_free_chsysdev;

	return 0;

out_free_chsysdev:
	device_destroy(iopmemc_class, iopmem->cdev_num);
out_free_chdev:
	cdev_del(&iopmem->cdev);
out_free_queue:
	blk_cleanup_queue(iopmem->queue);
out_release_instance:
	iopmem_release_instance(iopmem);
out_put_device:
	put_device(&pdev->dev);
	kfree(iopmem);
out_disable_device:
	pci_disable_device(pdev);
out:
	return err;
}

static void iopmem_remove(struct pci_dev *pdev)
{
	struct iopmem_device *iopmem = pci_get_drvdata(pdev);

	blk_set_queue_dying(iopmem->queue);
	iopmem_detach_disk(iopmem);
	blk_cleanup_queue(iopmem->queue);
	device_destroy(iopmemc_class, iopmem->cdev_num);
	cdev_del(&iopmem->cdev);
	iopmem_release_instance(iopmem);
	put_device(iopmem->dev);
	kfree(iopmem);
	pci_disable_device(pdev);
}

static struct pci_driver iopmem_pci_driver = {
	.name = "iopmem",
	.id_table = iopmem_id_table,
	.probe = iopmem_probe,
	.remove = iopmem_remove,
};

static int __init iopmem_init(void)
{
	int result;

	iopmemc_class = class_create(THIS_MODULE, "iopmemc");
	if (IS_ERR(iopmemc_class))
		return PTR_ERR(iopmemc_class);

	if ((result = alloc_chrdev_region(&char_device_num, 0,
					  max_devices, "iopmemc")))
		goto out_destroy_class;

	result = pci_register_driver(&iopmem_pci_driver);
	if (result)
		goto out_unreg_chrdev;

	pr_info("iopmem: module loaded\n");
	return 0;

out_unreg_chrdev:
	unregister_chrdev_region(char_device_num, max_devices);

out_destroy_class:
	class_destroy(iopmemc_class);

	return result;
}

static void __exit iopmem_exit(void)
{
	pci_unregister_driver(&iopmem_pci_driver);
	class_destroy(iopmemc_class);
	unregister_chrdev_region(char_device_num, max_devices);
	pr_info("iopmem: module unloaded\n");
}

MODULE_AUTHOR("Logan Gunthorpe <logang@deltatee.com>");
MODULE_LICENSE("GPL");
module_init(iopmem_init);
module_exit(iopmem_exit);
