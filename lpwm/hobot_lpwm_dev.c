/****************************************************************
 ****                    COPYRIGHT NOTICE                    ****
 ****            Copyright      2023 Horizon Robotics, Inc.  ****
 ****                    All rights reserved.                ****
 ****************************************************************/


#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/consumer.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include "hobot_dev_vin_node.h"
#include "hobot_lpwm_hw_reg.h"
#include "hobot_lpwm_ops.h"
#include "hobot_lpwm_dev.h"

/* LPWM global device struct */
static struct hobot_lpwm_ins *glpwm_chip[LPWM_INUM];

inline struct hobot_lpwm_ins *lpwm_ins_ptr(uint32_t i_id)
{
	return glpwm_chip[i_id];
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief function that hrtimer will call 
 * @param[in] *hrt: The pointer of hrtimer
 * @retval HRTIMER_RESTART: hrtimer will restart itself
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static enum hrtimer_restart swtig_timer_func(struct hrtimer *hrt)
{
	struct hobot_lpwm_ins *lpwm =
		container_of(hrt, struct hobot_lpwm_ins, swtrigger_timer);

	lpwm_sw_trigger(lpwm->base);
	lpwm_debug(lpwm, "SW triggered\n");

	hrtimer_forward_now(hrt, ms_to_ktime(1000));

	return HRTIMER_RESTART;
}

/* probe functions */
/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Reset of peripheral
 * @param[in] *pdev: The pointer of platform_device
 * @retval <0: Error
 * @retval 0: Success
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
// static int32_t lpwm_reset_peri(const struct platform_device *pdev)
// {
// 	struct device_node *node = pdev->dev.of_node;
// 	struct reset_control *lpwm_reset_control = NULL;
// 	int32_t ret = LPWM_RET_OK;

// 	lpwm_reset_control = of_reset_control_array_get_exclusive(node);
// 	if (IS_ERR(lpwm_reset_control)) {
// 		lpwm_err(NULL, "Get reset control fail\n");
// 		return PTR_ERR(lpwm_reset_control);
// 	}

// 	ret = reset_control_assert(lpwm_reset_control);
// 	if (ret != 0) {
// 		goto err_put;
// 	}
// 	osal_msleep(1);
// 	ret = reset_control_deassert(lpwm_reset_control);
// 	if (ret != 0) {
// 		goto err_put;
// 	}
// err_put:
// 	reset_control_put(lpwm_reset_control);
// 	return ret;
// }

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Preinit of lpwm
 * @param[in] *pdev: The pointer of platform_device
 * @param[in] *lpwm: The pointer of lpwm instance
 * @retval <0: Error
 * @retval 0: Success
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t lpwm_preinit(struct platform_device *pdev, struct hobot_lpwm_ins *lpwm)
{
	int32_t ret = LPWM_RET_OK;

	// ret = lpwm_reset_peri(pdev);
	// if (ret != LPWM_RET_OK) {
	// 	lpwm_err(NULL, "Reset failed!\n");
	// 	return ret;
	// }

	lpwm->sclk = devm_clk_get(&pdev->dev, "lpwm_sclk");
	if (IS_ERR(lpwm->sclk)) {
		lpwm_err(NULL, "sclock not found!\n");
		return PTR_ERR(lpwm->sclk);
	}

	lpwm->pclk = devm_clk_get(&pdev->dev, "lpwm_pclk");
	if (IS_ERR(lpwm->pclk)) {
		lpwm_err(NULL, "pclock not found!\n");
		return PTR_ERR(lpwm->pclk);
	}

	ret = clk_prepare_enable(lpwm->sclk);
	if (ret != 0) {
		lpwm_err(NULL, "Enable lpwm sclock fail!\n");
		return ret;
	}

	ret = clk_prepare_enable(lpwm->pclk);
	if (ret != 0) {
		lpwm_err(NULL, "Enable lpwm pclock fail!\n");
		return ret;
	}

	return ret;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Get pinstrl of lpwm
 * @param[in] *dev: The pointer of device
 * @param[in] *lpwm: The pointer of lpwm instance
 * @retval -ENODEV: Failed to get a pinctrl
 * @retval 0: Success
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
// static int32_t lpwm_get_pinctrl(struct device *dev, struct hobot_lpwm_ins *lpwm)
// {
// 	int32_t ret = LPWM_RET_OK;

// 	lpwm->pinctrl = devm_pinctrl_get(dev);
// 	if (IS_ERR(lpwm->pinctrl)) {
// 		lpwm_err(NULL, "Get pinctrl fail!\n");
// 		return PTR_ERR(lpwm->pinctrl);
// 	}

// 	lpwm->pins_default = pinctrl_lookup_state(lpwm->pinctrl, "default");
// 	if (IS_ERR(lpwm->pins_default)) {
// 		lpwm_err(NULL, "Get pinctrl state fail!\n");
// 		return PTR_ERR(lpwm->pins_default);
// 	}

// 	ret = pinctrl_select_state(lpwm->pinctrl, lpwm->pins_default);
// 	if (ret != 0) {
// 		lpwm_err(NULL, "Set pinctrl state fail!\n");
// 		return ret;
// 	}

// 	return ret;
// }

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Lpwm interrupt handler function
 * @param[in] irq: irq
 * @param[in] *data: The pointer of lpwm instance
 * @retval IRQ_HANDLED: Handled
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static irqreturn_t lpwm_irq_handler(int32_t irq, void *data)
{
	struct hobot_lpwm_ins *lpwm = (struct hobot_lpwm_ins *)data;

	lpwm_info(lpwm, "Irq handled\n");
	return IRQ_HANDLED;
}


static const struct pwm_ops pwm_lite_ops = {
	.request = hobot_lpwm_request,
	.free	 = hobot_lpwm_free,
	.apply	 = hobot_lpwm_apply,
	.owner	 = THIS_MODULE,
};

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Platform init of of lpwm.
 * @param[in] *pdev: The pointer of platform_device
 * @param[in] *lpwm: The pointer of lpwm instance
 * @retval <0: Failed
 * @retval 0: Success
 * @data_read None
 * @data_updated glpwm_chip: Global struct of lpwm init
 * @compatibility HW: J6
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t lpwm_platform_init(struct platform_device *pdev, struct hobot_lpwm_ins *lpwm)
{
	int32_t ret = LPWM_RET_OK;
	struct resource *lpwm_res = NULL;
	struct device_node *node = NULL;

	lpwm_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (lpwm_res == NULL) {
		lpwm_err(NULL, "Platform get resource failed!\n");
		return -ENODEV;
	}

	lpwm->base = devm_ioremap_resource(&pdev->dev, lpwm_res);
	if (IS_ERR(lpwm->base)) {
		lpwm_err(NULL, "Ioremap resource failed!\n");
		return PTR_ERR(lpwm->base);
	}

	node = pdev->dev.of_node;
	lpwm->dev_idx = of_alias_get_id(node, "lpwm");
	if (lpwm->dev_idx < 0) {
		lpwm_err(lpwm, "Unable to get alias id, errno %d\n", lpwm->dev_idx);
		return lpwm->dev_idx;
	}
	snprintf(lpwm->name, sizeof(lpwm->name), "%s%d", LPWM_NAME, lpwm->dev_idx);

	// ret = lpwm_get_pinctrl(&pdev->dev, lpwm);
	// if (ret != LPWM_RET_OK)
	// 	return ret;

	lpwm->irq = irq_of_parse_and_map(node, 0);
	if (lpwm->irq == 0) {
		lpwm_err(lpwm, "Irq map failed!\n");
		return -EINVAL;
	}

	ret = devm_request_irq(&pdev->dev, lpwm->irq, lpwm_irq_handler, IRQF_SHARED,
			       lpwm->name, (void *)lpwm);
	if (ret != 0) {
		lpwm_err(lpwm, "Unable to request irq!\n");
		return ret;
	}

	hrtimer_init(&lpwm->swtrigger_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL | HRTIMER_MODE_PINNED);
	lpwm->swtrigger_timer.function = swtig_timer_func;

	osal_mutex_init(&lpwm->con_lock);
	lpwm->utype = INIT_STATE;
	lpwm->chip.dev = &pdev->dev;
	lpwm->chip.ops = &pwm_lite_ops;
	lpwm->chip.npwm = LPWM_CNUM;
	lpwm->chip.base = -1;

	return ret;
}

/* sysfs node attr functions*/
static ssize_t lpwm_interrupt_show(struct device *dev,
				   struct device_attribute *attr , char *buf)
{
	int32_t len;
	uint32_t val;
	struct hobot_lpwm_ins *lpwm = (struct hobot_lpwm_ins *)dev_get_drvdata(dev);

	lpwm_interrupt_read(lpwm->base, &val);

	if(val == 0u) {
		len = snprintf(buf, LPWM_ATTR_MAX_SIZE, "LPWM-%d: Interrupt disable.\n",
			       lpwm->dev_idx);
	} else {
		len = snprintf(buf, LPWM_ATTR_MAX_SIZE, "LPWM-%d: Interrupt enable.\n",
			       lpwm->dev_idx);
	}
	return (ssize_t)len;
}

static ssize_t lpwm_interrupt_store(struct device *dev, struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct hobot_lpwm_ins *lpwm = (struct hobot_lpwm_ins *)dev_get_drvdata(dev);
	uint32_t token;

	(void)sscanf(buf, "%u", &token);
	lpwm_interrupt_config(lpwm->base, token);
	if (token == 0u) {
		lpwm_debug(lpwm, "Interrupt disabled.\n");
	} else {
		lpwm_debug(lpwm, "Interrupt enabled.\n");
	}

	return count;
}
static DEVICE_ATTR_RW(lpwm_interrupt);

static ssize_t lpwm_config_info_show(struct device *dev,
				     struct device_attribute *attr , char *buf)
{
	int32_t len;
	int32_t i;
	struct hobot_lpwm_ins *lpwm = (struct hobot_lpwm_ins *)dev_get_drvdata(dev);
	lpwm_chn_attr_t *config = (lpwm_chn_attr_t *)&lpwm->lpwm_attr;
	char utype[3][12] = {"NONE", "CAMSYS", "BACKLIGHT"};

	len = snprintf(buf, LPWM_ATTR_MAX_SIZE,
		       "core\tsource\toffset\tperiod\tduty_time\tthreshold\tadjust_step\toccupied\n");
	for (i = 0; i < LPWM_CNUM; i++) {
		len += snprintf(&buf[len], LPWM_ATTR_MAX_SIZE,
				"%u\t%d\t%u\t%u\t%u\t%u\t%u\t%s\n",
				i, config[i].trigger_source, config[i].offset,
				config[i].period, config[i].duty_time,
				config[i].threshold, config[i].adjust_step,
				utype[lpwm->utype]);
	}

	return (ssize_t)len;
}
static DEVICE_ATTR_RO(lpwm_config_info);

static ssize_t trigger_source_show(struct device *dev,
				   struct device_attribute *attr , char *buf)
{
	int32_t len;

	len = snprintf(buf, LPWM_ATTR_MAX_SIZE, "0 -- AON RTC PPS  1 -- SGT 0\n");
	len += snprintf(&buf[len], LPWM_ATTR_MAX_SIZE, "2 -- SGT 1        3 -- SGT 2\n");
	len += snprintf(&buf[len], LPWM_ATTR_MAX_SIZE, "4 -- SGT 3        5 -- PAD 0\n");
	len += snprintf(&buf[len], LPWM_ATTR_MAX_SIZE, "6 -- PAD 1        7 -- PAD 2\n");
	len += snprintf(&buf[len], LPWM_ATTR_MAX_SIZE, "8 -- PAD 3        9 -- PCIE ETH\n");
	len += snprintf(&buf[len], LPWM_ATTR_MAX_SIZE, "10-- MCU ETH\n");

	return (ssize_t)len;
}

static ssize_t trigger_source_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int32_t i;
	uint32_t token;
	struct hobot_lpwm_ins *lpwm = (struct hobot_lpwm_ins *)dev_get_drvdata(dev);

	sscanf(buf, "%u", &token);
	if (token >= LPWM_TRIG_SOURCE_MAX) {
		lpwm_err(NULL, "Trigger source should be:\n" \
		"0 -- AON RTC PPS; 1 -- SGT 0\n" \
		"2 -- SGT 1;       3 -- SGT 2\n" \
		"4 -- SGT 3;       5 -- PAD 0\n" \
		"6 -- PAD 1;       7 -- PAD 2\n" \
		"8 -- PAD 3;       9 -- PCIE ETH\n" \
		"10-- MCU ETH");
		return -EINVAL;
	}

	lpwm_trigger_source_config(lpwm, token);
	osal_mutex_lock(&lpwm->con_lock);
	for (i = 0; i < LPWM_CNUM; i++) {
		lpwm->lpwm_attr[i].trigger_source = token;
	}
	osal_mutex_unlock(&lpwm->con_lock);

	return (ssize_t)count;
}
static DEVICE_ATTR_RW(trigger_source);

static int32_t lpwm_sysfs_create(struct platform_device *pdev)
{
	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_lpwm_interrupt.attr))
		return -ENOMEM;
	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_lpwm_config_info.attr))
		return -ENOMEM;
	if (sysfs_create_file(&pdev->dev.kobj, &dev_attr_trigger_source.attr))
		return -ENOMEM;
	return LPWM_RET_OK;
}

static int32_t lpwm_sysfs_remove(struct platform_device *pdev)
{
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_lpwm_interrupt.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_trigger_source.attr);
	sysfs_remove_file(&pdev->dev.kobj, &dev_attr_lpwm_config_info.attr);

	return LPWM_RET_OK;
}

static const struct file_operations lpwm_cdev_ops = {
	.owner          = THIS_MODULE,
	.open           = lpwm_chip_open,
	.release        = lpwm_chip_release,
	.unlocked_ioctl = lpwm_chip_ioctl,
	.compat_ioctl   = lpwm_chip_ioctl,
};

static struct class hobot_lpwm_class = {
	.name = "hobot-lpwm",
	.owner = THIS_MODULE,
};

static int lpwm_chip_cdev_create(struct hobot_lpwm_ins *lpwm)
{
	int ret = LPWM_RET_OK;
	struct device *devf = NULL;
	struct lpwm_chip_cdev *lpwm_cdev = NULL;

	lpwm_cdev = (struct lpwm_chip_cdev *)
		osal_kmalloc(sizeof(struct lpwm_chip_cdev), 0);
	if (!lpwm_cdev) {
		lpwm_err(lpwm, "Kmalloc for lpwm_cdev fail!\n");
		return -ENOMEM;
	}
	memset(lpwm_cdev, 0, sizeof(struct lpwm_chip_cdev));

	lpwm_cdev->lpwm = lpwm;
	lpwm_cdev->dev = lpwm->chip.dev;
	snprintf(lpwm_cdev->name, sizeof(lpwm_cdev->name),
		 "%s%d", LPWM_NAME, lpwm->dev_idx);

	ret = alloc_chrdev_region(&lpwm_cdev->devt, 0, 1, lpwm_cdev->name);
	if (ret < 0) {
		lpwm_err(lpwm, "Alloc chrdev region fail, ret-%d!\n", ret);
		goto free;
	} else {
		lpwm_cdev->major = MAJOR(lpwm_cdev->devt);
		lpwm_cdev->minor = MINOR(lpwm_cdev->devt);
	}

	cdev_init(&lpwm_cdev->cdev, &lpwm_cdev_ops);
	lpwm_cdev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&lpwm_cdev->cdev, lpwm_cdev->devt, 1);
	if (ret) {
		lpwm_err(lpwm, "Add cdev fail, ret-%d\n", ret);
		goto unregister;
	}

	devf = device_create(&hobot_lpwm_class, NULL,
			     lpwm_cdev->devt, NULL, lpwm_cdev->name);
	if (IS_ERR(devf)) {
		lpwm_err(lpwm, "Device create fail!\n");
		ret = PTR_ERR(devf);
		goto del;
	}
	lpwm->lpwm_cdev = lpwm_cdev;

	return ret;
del:
	cdev_del(&lpwm_cdev->cdev);
unregister:
	unregister_chrdev_region(lpwm_cdev->devt, 1);
free:
	osal_kfree(lpwm_cdev);
	return ret;
}

static struct vin_common_ops vin_lpwm_ops = {
	.open = lpwm_open,
	.close = lpwm_close,
	.video_set_attr = lpwm_init,
	.video_get_attr = lpwm_get_attr,
	.video_set_attr_ex = lpwm_change_attr,
	.video_start = lpwm_start,
	.video_stop = lpwm_stop,
	.video_reset = lpwm_reset,
};

static uint64_t get_timestamp_null(uint32_t lpwm_chn)
{
	lpwm_info(NULL, "func null\n");
	return 0;
}

struct cim_interface_ops cim_ops = {
	.cim_get_lpwm_timestamps = get_timestamp_null,
};

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Probe function
 * @param[in] *pdev: The pointer of platform_device
 * @retval <0: Failed
 * @retval 0: Success
 * @data_read None
 * @data_updated glpwm_chip: Global struct init
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_lpwm_probe(struct platform_device *pdev)
{
	int32_t ret = LPWM_RET_OK;
	struct hobot_lpwm_ins *lpwm;

	lpwm = (struct hobot_lpwm_ins *)devm_kzalloc(&pdev->dev,
						     sizeof(struct hobot_lpwm_ins),
						     GFP_KERNEL);
	if (IS_ERR(lpwm)) {
		lpwm_err(NULL, "devm kzalloc failed!\n");
		return -ENOMEM;
	}

	ret = lpwm_preinit(pdev, lpwm);
	if (ret != LPWM_RET_OK) {
		return ret;
	}

	ret = lpwm_platform_init(pdev, lpwm);
	if (ret != LPWM_RET_OK) {
		goto err_disable_clk;
	}

	ret = pwmchip_add(&lpwm->chip);
	if (ret < 0) {
		lpwm_err(lpwm, "Add to Linux pwm chip failed!\n");
		goto err_free_irq;
	}

	ret = lpwm_sysfs_create(pdev);
	if (ret < 0) {
		lpwm_err(lpwm, "Sysfs create failed!\n");
		goto err_free_pwm_chip;
	}

	platform_set_drvdata(pdev, lpwm);

	ret = lpwm_chip_cdev_create(lpwm);
	if (ret != LPWM_RET_OK) {
		goto err_free_pwm_chip;
	}

	lpwm_div_ratio_config(lpwm->base, 24);

	glpwm_chip[lpwm->dev_idx] = lpwm;
	lpwm->cim_cops = vio_get_callback_ops(&cim_ops, VIN_MODULE, COPS_9);
	vin_register_device_node(VIN_LPWM, &vin_lpwm_ops);

	lpwm_info(lpwm, "Probe success\n");

	return ret;

err_free_pwm_chip:
	pwmchip_remove(&lpwm->chip);
err_free_irq:
	devm_free_irq(&pdev->dev, lpwm->irq, lpwm);
err_disable_clk:
	clk_disable_unprepare(lpwm->sclk);
	clk_disable_unprepare(lpwm->pclk);
	osal_kfree(lpwm);
	return ret;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Remove lpwm char dev
 * @param[in] *pdev: The pointer of lpwm device
 * @retval <0: Failed
 * @retval 0: Success
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static void lpwm_chip_cdev_remove(struct hobot_lpwm_ins *lpwm)
{
	struct lpwm_chip_cdev *lpwm_cdev = lpwm->lpwm_cdev;

	device_destroy(&hobot_lpwm_class, lpwm_cdev->devt);
	cdev_del(&lpwm_cdev->cdev);
	unregister_chrdev_region(lpwm_cdev->devt, 1);

	osal_kfree(lpwm_cdev);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Lpwm remove from kernel
 * @param[in] *pdev: The pointer of lpwm device
 * @retval None
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static void lpwm_kernel_remove(struct platform_device *pdev)
{
	struct hobot_lpwm_ins *lpwm = 
		(struct hobot_lpwm_ins *)platform_get_drvdata(pdev);

	if (hrtimer_active(&lpwm->swtrigger_timer))
		hrtimer_cancel(&lpwm->swtrigger_timer);

	lpwm_sysfs_remove(pdev);
	devm_free_irq(&pdev->dev, lpwm->irq, (void *)lpwm);
	pwmchip_remove(&lpwm->chip);
	lpwm_chip_cdev_remove(lpwm);

	clk_disable_unprepare(lpwm->sclk);
	clk_disable_unprepare(lpwm->pclk);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Remove function
 * @param[in] *pdev: The pointer of platform_device
 * @retval <0: Failed
 * @retval 0: Success
 * @data_read None
 * @data_updated glpwm_chip: Global struct pointer set to null
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t hobot_lpwm_remove(struct platform_device *pdev)
{
	int32_t ret = LPWM_RET_OK;
	struct hobot_lpwm_ins *lpwm =
		(struct hobot_lpwm_ins *)platform_get_drvdata(pdev);

	lpwm_kernel_remove(pdev);
	glpwm_chip[lpwm->dev_idx] = NULL;

	lpwm_info(lpwm, "Remove success\n");
	osal_kfree(lpwm);

	return ret;
}

static const struct of_device_id hobot_lpwm_dt_ids[] = {
	{ .compatible = "hobot,hobot-lpwm", },
	{/***NULL****/ }
};
MODULE_DEVICE_TABLE(of, hobot_lpwm_dt_ids);

static struct platform_driver hobot_lpwm_driver = {
	.driver = {
		.name = "hobot-lpwm",
		.of_match_table = hobot_lpwm_dt_ids,
	},
	.probe  = hobot_lpwm_probe,
	.remove = hobot_lpwm_remove,
};

static int32_t __init hobot_lpwm_init(void)
{
	int32_t ret;

	ret = class_register(&hobot_lpwm_class);
	if (ret < 0) {
		lpwm_err(NULL, "class init failed!\n");
		return ret;
	}
	return platform_driver_register(&hobot_lpwm_driver);
}
late_initcall(hobot_lpwm_init);

static void __exit hobot_lpwm_exit(void)
{
	platform_driver_unregister(&hobot_lpwm_driver);
	class_unregister(&hobot_lpwm_class);
}
module_exit(hobot_lpwm_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Li Ming <ming01.li@horizon.cc>");
MODULE_DESCRIPTION("HOBOT LPWM Driver");