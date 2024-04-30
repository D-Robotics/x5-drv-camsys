#include "hobot_vtrace.h"


static vtrace_ctx_s g_ctx = {0};
static vtrace_ctl_ctx_s g_ctl_ctx = {
	.vtrace_on = 1,
	.flowid_mask = VTRACE_ALL,
	.ctxid_mask = VTRACE_ALL,
	.module_mask = VTRACE_ALL,
	.level = LEVEL1,
};
static char *public_type_name[VTRACE_PUBTYPE_MAX] = VTRACE_PUBLIC_NAME;
static char *public_test_name[PUBLIC_TEST_PARAM_NUM] = PUBLIC_TEST_PARAM_NAME;
static char *public_fs_name[PUBLIC_FS_PARAM_NUM] = PUBLIC_FS_PARAM_NAME;
static char *public_fe_name[PUBLIC_FE_PARAM_NUM] = PUBLIC_FE_PARAM_NAME;
static char *public_drop_name[PUBLIC_DROP_PARAM_NUM] = PUBLIC_DROP_PARAM_NAME;
static char *module_name[VNODE_ID_MAX] = VTRACE_MODULE_NAME;

/**
 * vtrace_check_send_valid() - Check whether message needs to be save.
 *
 * Return: 0 means not need, 1 means need to save to buffer.
 */
static int32_t vtrace_check_send_valid(uint32_t module_type, uint32_t flowid,
				       uint32_t ctxid, uint32_t level)
{
	if ((g_ctl_ctx.module_mask && (0x1u << module_type)) == 0)
		return 0;
	if ((g_ctl_ctx.flowid_mask && (0x1u << flowid)) == 0)
		return 0;
	if ((g_ctl_ctx.ctxid_mask && (0x1u << ctxid)) == 0)
		return 0;
	if (g_ctl_ctx.level < level)
		return 0;

	return 1;
}

/**
 * vtrace_register() - Register param info to vtrace.
 *
 * @module_type:  The type of module, max value 11
 * @param_type: The param type index, [0 VTRACE_PUBTYPE_MAX] is used
 * 		for public param, max value VTRACE_PARAM_MAX=32
 * @param_name:  The pointer to the param name list.
 * @param_nums:  The number of param.
 * @level:  Vtrace level.
 *
 * Return: 0 if success, otherwise <0.
 */
int32_t vtrace_register(uint32_t module_type, uint32_t param_type, char **param_name,
			uint32_t param_nums, uint32_t level)
{
	int32_t i;
	vtrace_buf_head_s *buf_head = NULL;
	vtrace_zone_info_s *zone = NULL;

	if (module_type >= VNODE_ID_MAX) {
		vtrace_err("register module type(%d) error\n", module_type);
		return -1;
	}

	buf_head = &g_ctx.buf_head[module_type];
	if (buf_head->is_init[param_type]) {
		vtrace_err("this param type(%d) has been registered\n", param_type);
		return -1;
	}
	buf_head->is_init[param_type] = true;

	zone = (vtrace_zone_info_s *)&buf_head->zone[param_type];
	zone->param_nums = param_nums;
	zone->name = osal_kmalloc(VTRACE_NAME_LEN_MAX * param_nums,
				  OSAL_KMALLOC_KERNEL);
	if (!zone->name) {
		vtrace_err("kmalloc failed\n");
		return -1;
	}
	for (i = 0; i < param_nums; i++) {
		strncpy((char *)(zone->name + i), param_name[i],
			VTRACE_NAME_LEN_MAX-1);
		/* Ensure that the last bit is set to \0 */
		((char *)(zone->name + i))[VTRACE_NAME_LEN_MAX-1] = '\0';
	}
	zone->level = level;

	// vtrace_info("%s register param_type %d done\n",
	// 	    module_name[module_type], param_type);
	return 0;
}
EXPORT_SYMBOL(vtrace_register);

/**
 * vtrace_offres() - Reserve space in the pingpongbuffer.
 *
 * @copyto_off:  Return the offset that current message can use
 * @full_off:  The offset of ping or pong if it's full,
 * 	      just can be used when sflag return 1.
 * @sflag:  Decide whether vtrace swap buffer
 * @len:  The len of current message.
 *
 * Return: true if this message does not need to drop, otherwise false.
 */
static bool vtrace_offres(int32_t *copyto_off, int32_t *full_off,
			  uint32_t *sflag, int32_t len)
{
	uint32_t temp_off, new_off;

	/*
	 * Use cmpxchg() to avoid lock and improve efficiency
	 */
	temp_off = osal_atomic_read(&g_ctx.cur_off);
	do {
		/*
		 * Unlikely, 1. the ping or pong buffer almost reach the max size,
		 * or 2. vtrace is in pop state, that means HAL is using this buffer,
		 * vtrace need to swap the buffer once.
		 */
		if (unlikely(
			(len + temp_off > VTRACE_PING_PONG_SIZE - VTRACE_SAFE_SIZE)
			|| g_ctx.pop_swap)) {
			/*
			 * Something occurred that could not be dealt with, can not
			 * swap the buffer when the buffer that be swap to is be
			 * used by hal, so just drop this message and inc err_cnt.
			 */
			if ((g_ctx.halused_mask | (1 << g_ctx.pp_flag)) == 3) {
				g_ctx.error_cnt++;
				return false;
			}
			/* new offset of buffer is just the incoming message's len */
			new_off = len;
			/* the incoming message should copy to the head of buffer */
			*copyto_off = 0;
			/* full_off is the len of valid data when swap */
			*full_off = temp_off;
			/* distinguish why swap? pop or full */
			if (g_ctx.pop_swap)
				*sflag = 2;
			else
				*sflag = 1;
			/* just swap once */
			g_ctx.pop_swap = 0;
		} else {
			*copyto_off = temp_off;
			new_off = temp_off + len;
		}
	} while(!atomic_try_cmpxchg(&g_ctx.cur_off, &temp_off, new_off));

	return true;
}

/**
 * vtrace_send() - Send message to vtrace.
 * @param:  The pointer of data, vtrace just save this message without data if null
 *
 * Return: 0 if success, otherwise <0.
 */
int32_t vtrace_send(uint32_t module_type, uint32_t param_type,
		    uint32_t *param, uint32_t flow_id, uint32_t frame_id,
		    uint32_t ctx_id, uint32_t chnid)
{
	void *base = NULL;
	char *tmp_base = NULL;
	vtrace_buf_head_s *buf_head = NULL;
	vtrace_zone_info_s *zone = NULL;
	int32_t len = 0, i;
	uint32_t next_off = 0, full_off = 0;
	char tmp_buf[VTRACE_TEMP_BUF_SIZE] = {0};
	struct timespec64 ts1, ts2, ts3;
	static long ti = 0;
	/* use swap_flag instand of next_off to sign whether swap buffer*/
	uint32_t swap_flag = 0;

	if (!g_ctl_ctx.vtrace_on || !g_ctx.hal_on)
		return 0;

	ktime_get_real_ts64(&ts1);

	buf_head = &g_ctx.buf_head[module_type];
	if (module_type >= VNODE_ID_MAX) {
		vtrace_err("register module type(%d) error\n", module_type);
		return -1;
	}
	if (param_type >= VTRACE_PARAM_MAX) {
		vtrace_err("param type(%d) exceed %d\n", param_type, VTRACE_PARAM_MAX);
		return -1;
	}
	if (!buf_head->is_init[param_type]) {
		vtrace_err("param type(%d) has not been registered\n", param_type);
		return -1;
	}

	zone = (vtrace_zone_info_s *)&buf_head->zone[param_type];
	if (!vtrace_check_send_valid(module_type, flow_id, ctx_id, zone->level))
		return 0;

	if (param_type < VTRACE_PUBTYPE_MAX) {
		len += snprintf(&tmp_buf[len], VTRACE_SNP_MAX_SIZE, "[%s %s",
				module_name[module_type], public_type_name[param_type]);
	} else {
		len += snprintf(&tmp_buf[len], VTRACE_SNP_MAX_SIZE, "[%s PrivInfo_%d",
				module_name[module_type], param_type);
	}
	len += snprintf(&tmp_buf[len], VTRACE_SNP_MAX_SIZE,
		" ts:%lld tns:%ld", ts1.tv_sec, ts1.tv_nsec);
	len += snprintf(&tmp_buf[len], VTRACE_SNP_MAX_SIZE,
		" frameid:%d flowid:%d ctxid:%d chnid:%d]",
		frame_id, flow_id, ctx_id, chnid);
	if (param != NULL) {
		for (i = 0; i<zone->param_nums; i++) {
			len += snprintf(&tmp_buf[len], VTRACE_SNP_MAX_SIZE,
				" %s: %d", (char *)(zone->name + i), param[i]);
			/* when message exceed max size it will be truncated */
			if (len > VTRACE_TEMP_BUF_SIZE - VTRACE_SAFE_SIZE) {
				vtrace_warn("buf size too large, truncate message\n");
				break;
			}
		}
	}
	len += snprintf(&tmp_buf[len], VTRACE_SNP_MAX_SIZE, "\n");

	/* get a space in the ping pong buffer */
	if (!vtrace_offres(&next_off, &full_off, &swap_flag, len)) {
		len += snprintf(&tmp_buf[len-1], VTRACE_SNP_MAX_SIZE, "drop info\n");
		vtrace_err("%s", tmp_buf);
		return -1;
	}

	/* if swap_flag equal to non-zero, this means vtrace need swap ping pong buffer */
	if (swap_flag) {
		/* add '\0' to the end of the whole message */
		tmp_base = (char *)(g_ctx.base + g_ctx.pp_flag * VTRACE_PING_PONG_SIZE);
		tmp_base[full_off] = '\0';
		// vtrace_err("swap and wake up, cur pp_flag: %d, fulloff: %d\n", g_ctx.pp_flag, full_off);
		/* swap current ping pong state */
		g_ctx.pp_flag = g_ctx.pp_flag ? 0 : 1;
		/* if swap is called because the buffer is full */
		if (swap_flag == 1) {
			/* message_end and message_valid is the just by hal read */
			g_ctx.message_end = full_off;
			g_ctx.message_valid = 1;
			/* pop state should not wake up */
			wake_up(&g_ctx.irq_wait);
		}
	}

	/* here vtrace know where should copy the incoming to in the pingpong buffer */
	base = g_ctx.base + next_off + g_ctx.pp_flag * VTRACE_PING_PONG_SIZE;
	memcpy(base, tmp_buf, len);
	/* this flag should be set, to tell pop there is incoming message,
	 * otherwhise hal pop will pop the same message
	 */
	g_ctx.new_data_comein = 1;

	// ktime_get_real_ts64(&ts2);
	// ts3 = timespec64_sub(ts2,ts1);
	// ti = ts3.tv_nsec;
	return 0;
}
EXPORT_SYMBOL(vtrace_send);

static int vtrace_open(struct inode *inode, struct file *f)
{
	vtrace_cdev_s *ctx = NULL;

	ctx = container_of(inode->i_cdev, vtrace_cdev_s, cdev);
	f->private_data = ctx;
	/* if hal bin is not runing, vtrace ko should not work */
	g_ctx.hal_on = 1;
	vtrace_info("open done\n");

	return 0;
}

static int vtrace_release(struct inode *inode, struct file *f)
{
	f->private_data = NULL;
	g_ctx.hal_on = 0;
	vtrace_info("release done\n");

	return 0;
}

static int vtrace_mmap(struct file *file, struct vm_area_struct *vma)
{
	vtrace_ctx_s *ctx = &g_ctx;
	int32_t ret;
	uint32_t size = vma->vm_end - vma->vm_start;

	if (size > VTARCE_MEM_SIZE) {
		vtrace_err("mmap size exceed max size, size = %d.\n", size);
		return -1;
	}
	// vma->vm_flags |= VM_IO;
	// vma->vm_flags |= VM_LOCKED;

	ret = remap_vmalloc_range(vma, ctx->base, vma->vm_pgoff);
	if (ret < 0) {
		vtrace_err("mmap vmalloc virtual addr 0x%p fail\n", ctx->base);
		return -1;
	}

	return 0;
}

static bool check_data_valid(void)
{
	/* message_valid come true just because buffer is full, and read is valid */
	return g_ctx.message_valid;
}

static long vtrace_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int32_t ret = 0;
	uint32_t value = 0;
	uint32_t tmp_off;
	vtrace_public_test_s info = {.testinfo = 0};
	static uint32_t frame_id = 0;

	switch (cmd) {
	case VTRACE_READ_START:
		/* wait_event_interruptible_timeout() is not atomic
		 * the risk of check_data_valid() come true in point a is checked
		 * 1. check condition(false)
		 * a
		 * 2. sleep
		 * condition just come true before wake_up()
		 */
		ret = wait_event_interruptible_timeout(g_ctx.irq_wait,
						       check_data_valid(),
						       VTRACE_TIMEOUT);
		if (ret > 1) {
			value = g_ctx.pp_flag;
			value |= g_ctx.message_end << 2;
			g_ctx.halused_mask = 1 << g_ctx.pp_flag;
			// vtrace_err("read message_end: %d\n", g_ctx.message_end);
			if (copy_to_user((void __user *)arg, &value,
					 sizeof(uint32_t))) {
				vtrace_err("failed to copy_from_user\n");
				return -1;
			}
		} else if (ret == 1) {
			vtrace_debug("data valid without wakeup\n");
		} else if (ret == 0) {
			vtrace_debug("wait wake_up timeout\n");
		} else {
			vtrace_err("wait break\n");
		}
		break;
	case VTRACE_READ_END:
		/* clear message_valid flag, halused_mask set to 0 because hal is not using buffer*/
		g_ctx.message_valid = 0;
		g_ctx.halused_mask = 0;
		g_ctx.message_end = 0;
		break;
	case VTRACE_TIMEOUT_POP:
		/* vtrace will pop all valid message whether it is full or not,
		 * another buffer is valid to save message in pop state, so vtrace
		 * should swap buffer at this time
		 */

		/* if no new data send to vtrace, pop is stop */
		if (!g_ctx.new_data_comein)
			return -1;
		else
			g_ctx.new_data_comein = 0;
		/* get current offset to tell vtrace the hall is poping,
		 * read out offset, pp_flag, before turn to in pop
		 */
		tmp_off = osal_atomic_read(&g_ctx.cur_off);
		value = g_ctx.pp_flag;
		g_ctx.halused_mask = 1 << g_ctx.pp_flag;
		// vtrace_err("pop tmp_off: %d\n", tmp_off);
		g_ctx.pop_swap = 1;
		value |= tmp_off << 2;
		if (copy_to_user((void __user *)arg, &value, sizeof(uint32_t))) {
			vtrace_err("failed to copy_from_user\n");
			return -1;
		}
		ret = 0;

		break;
	case VTRACE_TIMEOUT_POP_END:
		/* clear halused_mask flag to end pop state*/
		g_ctx.halused_mask = 0;
		g_ctx.message_end = 0;
		break;
	case VTRACE_MODULE_MASK_SET:
		if (copy_from_user(&g_ctl_ctx.module_mask, (void __user *)arg,
				   sizeof(uint32_t))) {
			vtrace_err("copy_from_user fail at MODULE_MASK_SET\n");
		}
		break;
	case VTRACE_FLOWID_MASK_SET:
		if (copy_from_user(&g_ctl_ctx.flowid_mask, (void __user *)arg,
				   sizeof(uint32_t))) {
			vtrace_err("copy_from_user fail at FLOWID_MASK_SET\n");
		}
		break;
	case VTRACE_CTXID_MASK_SET:
		if (copy_from_user(&g_ctl_ctx.ctxid_mask, (void __user *)arg,
				   sizeof(uint32_t))) {
			vtrace_err("copy_from_user fail at CTXID_MASK_SET\n");
		}
		break;
	case VTRACE_LEVEL_SET:
		if (copy_from_user(&g_ctl_ctx.level, (void __user *)arg,
				   sizeof(uint32_t))) {
			vtrace_err("copy_from_user fail at VLEVEL_SET\n");
		}
		break;
	case VTARCE_MEM_SIZE_GET:
		value = VTARCE_MEM_SIZE;
		if (copy_to_user((void __user *)arg, &value,
				 sizeof(uint32_t))) {
			vtrace_err("copy_to_user fail at MEM_SIZE_GET\n");
			return -1;
		}
		break;
	case VTRACE_TEST_SEND:
		// vtrace_send(ISP_MODULE, VTRACE_PUBLIC_TEST,
		// 	    (uint32_t *)&info, 1, frame_id++, 2, 3);
		break;
	default:
		vtrace_err("ioctl cmd 0x%x is err\n", cmd);
		ret = -1;
		break;
	}

	return ret;
}

static const struct file_operations vtrace_cdev_ops = {
	.owner          = THIS_MODULE,
	.open           = vtrace_open,
	.release        = vtrace_release,
	.mmap		= vtrace_mmap,
	.unlocked_ioctl = vtrace_ioctl,
	.compat_ioctl   = vtrace_ioctl,
};

/* sysfs node functions*/
// static ssize_t frameinfo_show(struct device *dev, struct device_attribute *attr , char *buf)
// {
// 	ssize_t len = 0;
	// int32_t i, j, id = 1;
	// uint32_t value;
	// vtrace_zone_info_s *node = NULL;
	// void *base = NULL;
	// vtrace_frame_info_s info[HBV_MAX] = {0};
	// vtrace_buf_msg_s msg[HBV_MAX] = {0};

	// for (i = 0; i < HBV_MAX; i++) {
	// 	node = &g_ctx.buf_head[i].node[id];
	// 	if (!node->is_init || !node->is_send)
	// 		continue;
	// 	mutex_lock(&node->lock);
	// 	base = node->buf_base + node->zone_size * node->idx_new;
	// 	memcpy((void *)&msg[i], base, sizeof(vtrace_buf_msg_s));
	// 	memcpy((void *)&info[i], base + sizeof(vtrace_buf_msg_s), sizeof(vtrace_frame_info_s));
	// 	mutex_unlock(&node->lock);
	// }

	// len = snprintf(buf, VTRACE_SYSFS_SIZE_MAX, "param_name\tCIM\tISP\tPYM\n");
	// len += snprintf(&buf[len], VTRACE_SYSFS_SIZE_MAX, "frame_id\t");
	// for (i = 0; i < HBV_MAX; i++) {
	// 	node = &g_ctx.buf_head[i].node[id];
	// 	if (node->is_init && node->is_send)
	// 		len += (ssize_t)snprintf(&buf[len], VTRACE_SYSFS_SIZE_MAX, "%d", msg[i].frame_id);
	// 	else
	// 		len += (ssize_t)snprintf(&buf[len], VTRACE_SYSFS_SIZE_MAX, "N/A");
	// 	len += (ssize_t)snprintf(&buf[len], VTRACE_SYSFS_SIZE_MAX, (i==(HBV_MAX-1))?"\n":"\t");
	// }

	// for (i = 0; i < (sizeof(frame_info_name)/sizeof(char *)); i++) {
	// 	len += snprintf(&buf[len], VTRACE_SYSFS_SIZE_MAX, "%s\t", frame_info_name[i]);
	// 	for (j = 0; j < HBV_MAX; j++) {
	// 		node = &g_ctx.buf_head[j].node[id];
	// 		value = *((uint32_t *)&info[j] + i);
	// 		if (node->is_init && node->is_send)
	// 			len += (ssize_t)snprintf(&buf[len], VTRACE_SYSFS_SIZE_MAX, "%d", value);
	// 		else
	// 			len += (ssize_t)snprintf(&buf[len], VTRACE_SYSFS_SIZE_MAX, "N/A");
	// 		len += (ssize_t)snprintf(&buf[len], VTRACE_SYSFS_SIZE_MAX, (j==(HBV_MAX-1))?"\n":"\t");
	// 	}
	// }

// 	return len;
// }
// static DEVICE_ATTR(frameinfo, S_IRUGO, frameinfo_show, NULL);

static ssize_t vtrace_on_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	return snprintf(buf, VTRACE_SYSFS_SIZE_MAX, "%d\n", g_ctl_ctx.vtrace_on);
}

static ssize_t vtrace_on_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t len)
{
	sscanf(buf, "%u", &g_ctl_ctx.vtrace_on);
	return len;
}
static DEVICE_ATTR(vtrace_on, S_IRUGO | S_IWUSR, vtrace_on_show, vtrace_on_store);

static ssize_t flowid_mask_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, VTRACE_SYSFS_SIZE_MAX, "%d\n", g_ctl_ctx.flowid_mask);
}

static ssize_t flowid_mask_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t len)
{
	sscanf(buf, "%u", &g_ctl_ctx.flowid_mask);
	return len;
}
static DEVICE_ATTR(flowid_mask, S_IRUGO | S_IWUSR, flowid_mask_show, flowid_mask_store);

static ssize_t ctxid_mask_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	return snprintf(buf, VTRACE_SYSFS_SIZE_MAX, "%d\n", g_ctl_ctx.ctxid_mask);
}

static ssize_t ctxid_mask_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t len)
{
	sscanf(buf, "%u", &g_ctl_ctx.ctxid_mask);
	return len;
}
static DEVICE_ATTR(ctxid_mask, S_IRUGO | S_IWUSR, ctxid_mask_show, ctxid_mask_store);

static ssize_t module_mask_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	return snprintf(buf, VTRACE_SYSFS_SIZE_MAX, "%d\n", g_ctl_ctx.module_mask);
}

static ssize_t module_mask_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t len)
{
	sscanf(buf, "%u", &g_ctl_ctx.module_mask);
	return len;
}
static DEVICE_ATTR(module_mask, S_IRUGO | S_IWUSR, module_mask_show, module_mask_store);

static ssize_t level_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	return snprintf(buf, VTRACE_SYSFS_SIZE_MAX, "%d\n", g_ctl_ctx.level);
}

static ssize_t level_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t len)
{
	sscanf(buf, "%u", &g_ctl_ctx.level);
	return len;
}
static DEVICE_ATTR(level, S_IRUGO | S_IWUSR, level_show, level_store);

static ssize_t err_cnt_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	return snprintf(buf, VTRACE_SYSFS_SIZE_MAX, "%d\n", g_ctx.error_cnt);
}
static DEVICE_ATTR(err_cnt, S_IRUGO, err_cnt_show, NULL);

static int32_t vtrace_sysfs_create(struct device *dev)
{
	// if (device_create_file(dev, &dev_attr_frameinfo))
	// 	return -ENOMEM;
	if (device_create_file(dev, &dev_attr_vtrace_on))
		return -ENOMEM;
	if (device_create_file(dev, &dev_attr_flowid_mask))
		return -ENOMEM;
	if (device_create_file(dev, &dev_attr_ctxid_mask))
		return -ENOMEM;
	if (device_create_file(dev, &dev_attr_module_mask))
		return -ENOMEM;
	if (device_create_file(dev, &dev_attr_level))
		return -ENOMEM;
	if (device_create_file(dev, &dev_attr_err_cnt))
		return -ENOMEM;

	return 0;
}

static void vtrace_sysfs_remove(struct device *dev)
{
	// device_remove_file(dev, &dev_attr_frameinfo);
	device_remove_file(dev, &dev_attr_vtrace_on);
	device_remove_file(dev, &dev_attr_flowid_mask);
	device_remove_file(dev, &dev_attr_ctxid_mask);
	device_remove_file(dev, &dev_attr_module_mask);
	device_remove_file(dev, &dev_attr_level);
	device_remove_file(dev, &dev_attr_err_cnt);
}

static void vtrace_common_info_register(void)
{
	int32_t i;

	for (i = 0; i < VNODE_ID_MAX; i++) {
		vtrace_register(i, VTRACE_PUBLIC_TEST, public_test_name,
				PUBLIC_TEST_PARAM_NUM, LEVEL0);
		vtrace_register(i, VTRACE_PUBLIC_FS, public_fs_name,
				PUBLIC_FS_PARAM_NUM, LEVEL0);
		vtrace_register(i, VTRACE_PUBLIC_FE, public_fe_name,
				PUBLIC_FS_PARAM_NUM, LEVEL0);
		vtrace_register(i, VTRACE_PUBLIC_DROP, public_drop_name,
				PUBLIC_FS_PARAM_NUM, LEVEL0);
	}
}

static int32_t vtrace_mem_alloc(uint32_t size)
{
	void *ptr = NULL;

	ptr = vmalloc_user(size);
	if (!ptr) {
		vtrace_err("kmalloc failed, size = %d\n", size);
		return -1;
	}
	memset(ptr, 0, size);
	g_ctx.base = ptr;

	SetPageReserved(vmalloc_to_page(ptr));
	SetPageReserved(vmalloc_to_page(ptr+VTRACE_PING_PONG_SIZE));

	return 0;
}

static void vtrace_mem_free(void)
{
	int32_t i, j;
	vtrace_buf_head_s *buf_head = NULL;

	ClearPageReserved(vmalloc_to_page(g_ctx.base));
	ClearPageReserved(vmalloc_to_page(g_ctx.base+VTRACE_PING_PONG_SIZE));
	vfree(g_ctx.base);

	for (i = 0; i < VNODE_ID_MAX; i++) {
		buf_head = &g_ctx.buf_head[i];
		for (j = 0; j < VTRACE_PARAM_MAX; j++) {
			if (buf_head->is_init[j] && buf_head->zone[j].name)
				kfree(buf_head->zone[j].name);
		}
	}
}

static struct class vtrace_class = {
	.name = "hobot_vtrace",
	.owner = THIS_MODULE,
};

static int32_t vtrace_cdev_init(vtrace_cdev_s *vcdev)
{
	int ret = 0;
	struct device *devf = NULL;

	snprintf(vcdev->name, sizeof(vcdev->name), "%s", VTRACE_NAME);
	ret = alloc_chrdev_region(&vcdev->devt, 0, 1, vcdev->name);
	if (!ret) {
		vcdev->major = MAJOR(vcdev->devt);
		vcdev->minor = MINOR(vcdev->devt);
	} else {
		vtrace_err("alloc chrdev region err with ret %d!\n", ret);
		return ret;
	}

	cdev_init(&vcdev->cdev, &vtrace_cdev_ops);
	vcdev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&vcdev->cdev, vcdev->devt, 1);
	if (ret) {
		vtrace_err("add cdev err with return value %d\n",ret);
		goto err1;
	}

	devf = device_create(&vtrace_class, NULL, vcdev->devt, NULL, vcdev->name);
	if (IS_ERR(devf)) {
		vtrace_err("create device fail!\n");
		goto err2;
	}
	vcdev->dev = devf;

	return 0;
err2:
	cdev_del(&vcdev->cdev);
err1:
	unregister_chrdev_region(vcdev->devt, 1);
	return ret;
}

struct dbg_interface_ops vtrace_cb_ops = {
	.vtrace_send = vtrace_send,
};

DECLARE_VIO_CALLBACK_OPS(vtrace_cops, VTRACE_COPS_MAGIC, &vtrace_cb_ops);

static int32_t __init vtrace_init(void)
{
	int32_t ret;

	ret = class_register(&vtrace_class);
	if (ret < 0) {
		vtrace_err("class init failed\n");
		return ret;
	}

	ret = vtrace_cdev_init(&g_ctx.vcdev);
	if (ret != 0) {
		vtrace_err("cdev create failed\n");
		goto err;
	}

	ret = vtrace_sysfs_create(g_ctx.vcdev.dev);
	if (ret < 0) {
		vtrace_err("sysfs create failed\n");
		goto err;
	}

	ret = vtrace_mem_alloc(VTARCE_MEM_SIZE);
	if (ret < 0) {
		vtrace_err("memory alloc failed\n");
		return ret;
	}

	vtrace_common_info_register();

	init_waitqueue_head(&g_ctx.irq_wait);
	g_ctx.pp_flag = 0;
	g_ctx.message_valid = 0;
	g_ctx.halused_mask = 0;
	g_ctx.message_end = 0;
	g_ctx.error_cnt = 0;
	g_ctx.hal_on = 0;
	osal_atomic_set(&g_ctx.cur_off, 0);

	ret = vio_register_callback_ops(&cb_vtrace_cops, DPU_MODULE, COPS_0);	//vtrace tmp using idu module id
	if (ret < 0) {
		vtrace_err("cops register failed\n");
		return ret;
	}

	vtrace_info("Register done\n");
	return 0;
err:
	class_unregister(&vtrace_class);
	return ret;
}
module_init(vtrace_init);

static void __exit vtrace_exit(void)
{
	vtrace_cdev_s *vcdev = &g_ctx.vcdev;

	vtrace_sysfs_remove(vcdev->dev);
	device_destroy(&vtrace_class, vcdev->devt);
	unregister_chrdev_region(vcdev->devt, 1);
	cdev_del(&vcdev->cdev);
	class_unregister(&vtrace_class);
	vio_unregister_callback_ops(DPU_MODULE, COPS_0);	//vtrace tmp using idu module id
	vtrace_mem_free();
	memset(&g_ctx, 0, sizeof(vtrace_ctx_s));

	vtrace_info("Remove done\n");
}
module_exit(vtrace_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Li Ming <ming01.li@horizon.cc>");
MODULE_DESCRIPTION("VTRACE Driver");