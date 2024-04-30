/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include "hobot_lpwm_dev.h"
#include "hobot_lpwm_ops.h"

/* LPWM global pwm device struct */
static struct pwm_device *gpwm_dev[LPWM_ID_MAX];

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Stream on/off of lpwm channel
 * @param[in] *lpwm: The pointer of lpwm instance
 * @param[in] c_id: The id of lpwm channel
 * range: [0, 3];
 * @param[in] enable: On(>0)/off(0)
 * @retval None
 * @data_read None
 * @data_updated glpwm_chip: The enable_cnt of global struct is updated
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static void lpwm_single_channel_stream(struct hobot_lpwm_ins *lpwm, uint32_t c_id, uint32_t enable)
{
	uint32_t trmode;

	osal_mutex_lock(&lpwm->con_lock);
	trmode = lpwm->lpwm_attr[0].trigger_mode;
	osal_mutex_unlock(&lpwm->con_lock);

	if (enable) {
		if (osal_atomic_read(&lpwm->enable_cnt[c_id]) == 0) {
			lpwm_channel_enable_single(lpwm->base, c_id, 1);

			if (trmode == 0 && !hrtimer_active(&lpwm->swtrigger_timer)) {
				hrtimer_start(&lpwm->swtrigger_timer, ms_to_ktime(0), HRTIMER_MODE_REL);
			}
		}
		osal_atomic_inc(&lpwm->enable_cnt[c_id]);
		lpwm_debug(lpwm, "Channel-%d enable, remaining count %d\n",
			   c_id, osal_atomic_read(&lpwm->enable_cnt[c_id]));
	} else {
		if (osal_atomic_read(&lpwm->enable_cnt[c_id]) < 1) {
			lpwm_err(lpwm, "Channel-%d has not been enable\n", c_id);
			return ;
		}
		if (osal_atomic_dec_return(&lpwm->enable_cnt[c_id]) == 0) {
			lpwm_channel_enable_single(lpwm->base, c_id, 0);
		}
		lpwm_debug(lpwm, "Channel-%d disable, remaining count %d\n",
			   c_id, osal_atomic_read(&lpwm->enable_cnt[c_id]));
	}

	return ;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Request a lpwm channel
 * @param[in] *lpwm: The pointer of lpwm instance
 * @param[in] c_id: The id of lpwm channel
 * range: [0, 3];
 * @param[in] type: The type of user to request
 * @retval -ENOENT: The instance has not been probed
 * @retval -EBUSY: The instance has been requested
 * @retval 0: Success
 * @data_read None
 * @data_updated glpwm_chip: The utype and rcount of global struct is updated
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t lpwm_channel_request(struct hobot_lpwm_ins *lpwm, uint32_t c_id, chip_type_t type)
{
	int32_t i, flag = 0;
	int32_t ret = LPWM_RET_OK;

	if ((lpwm->utype != INIT_STATE) && (lpwm->utype != type)) {
		lpwm_err(lpwm, "Camsys/Backlight cannot request the same device! \
			 current utype: %d\n", lpwm->utype);
		return -EBUSY;
	}

	for (i = 0; i < LPWM_CNUM; i++) {
		if (osal_atomic_read(&lpwm->rcount[i])) {
			flag = 1;
			break;
		}
	}
	if (!flag) {
		if (type == BACKLIGHT) {
			/* Default trigger mode in backlight is SW PPS */
			lpwm_trigger_mode_config(lpwm->base, 0);
			osal_mutex_lock(&lpwm->con_lock);
			for (i = 0; i < LPWM_CNUM; i++) {
				lpwm->lpwm_attr[i].trigger_mode = 0;
			}
			osal_mutex_unlock(&lpwm->con_lock);
		}
		lpwm->utype = type;
	}

	osal_atomic_inc(&lpwm->rcount[c_id]);

	if (type == BACKLIGHT) {
		lpwm_debug(lpwm, "Requested by backlight, current cnt: %d.\n",
			   osal_atomic_read(&lpwm->rcount[c_id]));
	} else if (type == CAMSYS) {
		lpwm_debug(lpwm, "Requested by camsys, current cnt: %d.\n",
			   osal_atomic_read(&lpwm->rcount[c_id]));
	}

	return ret;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Free a lpwm channel
 * @param[in] *lpwm: The pointer of lpwm instance
 * @param[in] c_id: The id of lpwm channel
 * @param[in] type: The type of user to free
 * range: [0, 3];
 * @retval -ENOENT: The instance has not been probed
 * @retval 0: Success
 * @data_read None
 * @data_updated glpwm_chip: The rcount and utype of global struct is updated
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t lpwm_channel_free(struct hobot_lpwm_ins *lpwm, uint32_t c_id, chip_type_t type)
{
	int32_t i, flag = 0;
	int32_t ret = LPWM_RET_OK;

	if (osal_atomic_read(&lpwm->rcount[c_id]) == 0) {
		lpwm_err(lpwm, "Channel-%d has not been requested!\n", c_id);
		goto err_out;
	}

	osal_atomic_dec(&lpwm->rcount[c_id]);
	lpwm_debug(lpwm, "Channel free, remaining cnt: %d.\n",
		   osal_atomic_read(&lpwm->rcount[c_id]));

	for (i = 0; i < LPWM_CNUM; i++) {
		if (osal_atomic_read(&lpwm->rcount[i])) {
			flag = 1;
			break;
		}
	}
	if (!flag) {
		lpwm->utype = INIT_STATE;
		if (hrtimer_active(&lpwm->swtrigger_timer))
			hrtimer_cancel(&lpwm->swtrigger_timer);
		// memset(&lpwm->lpwm_attr, 0, sizeof(lpwm_chn_attr_t) * LPWM_CNUM);
	}

err_out:
	return ret;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Check the dynamic param range which will set to lpwm
 * @param[in] *lpwm: The pointer of lpwm instance
 * @param[in] c_id: The id of lpwm channel
 * range: [0, 3];
 * @param[in] *config: The pointer of dynamic lpwm attr
 * @retval -EINVAL: The attr to config out of range
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t lpwm_dynamic_param_check(struct hobot_lpwm_ins *lpwm, uint32_t c_id,
					lpwm_dynamic_fps_t *config, uint32_t sy_on)
{
	if (config->offset >= LPWM_OFFSET_MAX) {
		lpwm_err(lpwm, "Offset value %u set to core-%d should be in [0, 1000000) us!\n",
			 config->offset, c_id);
		return -EINVAL;
	}

	if ((config->duty_time > config->period) ||
	    (config->duty_time > LPWM_HIGH_MAX)) {
		lpwm_err(lpwm, "Duty_time value %u set to core-%d should not exceed 0xFFF and period!\n",
			 config->duty_time, c_id);
		return -EINVAL;
	}

	if ((config->period > LPWM_PERIOD_MAX) ||
	    (config->period < LPWM_PERIOD_MIN)) {
		lpwm_err(lpwm, "Period value %u set to core-%d should be in [2, 1000000) us!\n",
			 config->period, c_id);
		return -EINVAL;
	}

	if (sy_on && config->period <= config->offset) {
		lpwm_err(lpwm, "Offset %u set to core-%d should not larger chan period %u!\n",
			 config->offset, c_id, config->period);
		return -EINVAL;
	}

	if (config->trigger_source >= LPWM_TRIG_SOURCE_MAX) {
		lpwm_err(lpwm, "Trigger source should be:\n" \
		"0 -- AON RTC PPS; 1 -- SGT 0\n" \
		"2 -- SGT 1;       3 -- SGT 2\n" \
		"4 -- SGT 3;       5 -- PAD 0\n" \
		"6 -- PAD 1;       7 -- PAD 2\n" \
		"8 -- PAD 3;       9 -- PCIE ETH\n" \
		"10-- MCU ETH");
		return -EINVAL;
	}

	return LPWM_RET_OK;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Check the param range which will set to lpwm
 * @param[in] i_id: The id of lpwm channel
 * range: [0, 3];
 * @param[in] c_id: The id of lpwm channel
 * range: [0, 3];
 * @param[in] *config: The pointer of lpwm attr
 * @retval -EINVAL: The attr to config out of range
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t lpwm_param_check(struct hobot_lpwm_ins *lpwm, uint32_t c_id,
				lpwm_chn_attr_t* config)
{
	lpwm_dynamic_fps_t dynamic_attr;
	uint32_t sy_on = config->threshold > 0 ? 1 : 0;

	if (config->threshold > LPWM_THRESHOLD_MAX) {
		lpwm_err(lpwm, "Threshold value %u set to core-%d should not exceed 0xFFFF us!\n",
			 config->threshold, c_id);
		return -EINVAL;
	}

	if (config->adjust_step > LPWM_STEP_MAX) {
		lpwm_err(lpwm, "Adjust_step value %u set to core-%d should not exceed 15!\n",
			 config->adjust_step, c_id);
		return -EINVAL;
	}

	dynamic_attr.duty_time = config->duty_time;
	dynamic_attr.offset = config->offset;
	dynamic_attr.period = config->period;
	dynamic_attr.trigger_mode = config->trigger_mode;
	dynamic_attr.trigger_source = config->trigger_source;

	return lpwm_dynamic_param_check(lpwm, c_id, &dynamic_attr, sy_on);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Config threshold of lpwm channel
 * @param[in] *lpwm: The pointer of lpwm instance
 * @param[in] c_id: The id of lpwm channel
 * range: [0, 3];
 * @param[in] *config: The pointer of lpwm attr
 * @retval -EINVAL: The attr to config out of range
 * @data_read None
 * @data_updated glpwm_chip: The lpwm_attr of global struct is updated
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t lpwm_single_channel_config(struct hobot_lpwm_ins *lpwm, uint32_t c_id, lpwm_chn_attr_t* config)
{
	int32_t i, ret = LPWM_RET_OK;

	if (osal_atomic_read(&lpwm->rcount[c_id]) > 1) {
		lpwm_warn(lpwm, "Second config core-%d is invalid\n", c_id);
		return LPWM_RET_OK;
	}

	ret = lpwm_param_check(lpwm, c_id, config);
	if (ret < 0)
		return ret;

	/*
	 * 1. Cfg static ctrl regs
	 * 2. Cfg dynamic ctrl regs
	 */
	lpwm_cfg2_config_single(lpwm->base, c_id, config->threshold, config->adjust_step);
	lpwm_trigger_mode_config(lpwm->base, config->trigger_mode);
	lpwm_trigger_source_config(lpwm->base, config->trigger_source);

	lpwm_offset_config_single(lpwm->base, c_id, config->offset);
	lpwm_cfg1_config_single(lpwm->base, c_id, config->period, config->duty_time);

	osal_mutex_lock(&lpwm->con_lock);
	for (i = 0; i < LPWM_CNUM; i++) {
		lpwm->lpwm_attr[i].trigger_mode = config->trigger_mode > 0 ? 1 : 0;
		lpwm->lpwm_attr[i].trigger_source = config->trigger_source;
	}
	lpwm->lpwm_attr[c_id].offset = config->offset & LPWM_OFFSET_MAX;
	lpwm->lpwm_attr[c_id].duty_time = config->duty_time & LPWM_HIGH_MAX;
	lpwm->lpwm_attr[c_id].period = config->period & LPWM_PERIOD_MAX;
	lpwm->lpwm_attr[c_id].threshold = config->threshold & LPWM_THRESHOLD_MAX;
	lpwm->lpwm_attr[c_id].adjust_step = config->adjust_step & LPWM_STEP_MAX;
	osal_mutex_unlock(&lpwm->con_lock);

	return ret;
}

/*vin node common ops*/
/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Do nothing
 * @param[in] *vctx
 * @retval 0
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int32_t lpwm_open(struct vio_video_ctx *vctx)
{
	int32_t ret = LPWM_RET_OK;

	return ret;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Disable a lpwm and abandon it
 * @param[in] *vctx: [0, 15]
 * @retval -EINVAL: The vctx or the arg wrong
 * @retval -ENOENT: The device cannot be found by id
 * @retval -EPERM: The device is occupied by backlight, or has not been requested
		   Cannot close lpwm when it has not been enabled
 * @data_read glpwm_chip: The global struct to get the address of every reg to config
 * @data_updated glpwm_chip: Memset the attr in this struct
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int32_t lpwm_close(struct vio_video_ctx *vctx)
{
	uint32_t lpwm_channel_id, i_id, c_id;
	struct hobot_lpwm_ins *lpwm = NULL;

	if (!vctx || !vctx->file) {
		lpwm_err(lpwm, "Config param null!\n");
		return -EINVAL;
	}

	lpwm_channel_id = *(uint32_t *)vctx->file;
	i_id = INS_ID(lpwm_channel_id);
	c_id = COR_ID(lpwm_channel_id);

	lpwm_check_and_return(lpwm, i_id, c_id);
	if (!lpwm) {
		lpwm_err(NULL, "Instance %d has not been probed!\n", i_id);
		return -EINVAL;
	}

	lpwm_check_utype(lpwm, CAMSYS);

	lpwm_single_channel_stream(lpwm, c_id, LPWM_STREAM_OFF);

	return lpwm_channel_free(lpwm, c_id, CAMSYS);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Request one channel of lpwm, config the attr to reg, and enable it
 * @param[in] *vctx: [0, 15]
 * @param[in] arg: The pointer of the attr of lpwm to config
 * @retval -EINVAL: The vctx or the arg wrong
 * @retval -EBUSY: Cannot request lpwm when it has been occupied
 * @retval -ENOENT: The device cannot be found by id
 * @data_read glpwm_chip: The global struct to get the address of every reg to config
 * @data_updated glpwm_chip: Updated the attr in the instance
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int32_t lpwm_init(struct vio_video_ctx *vctx, void *attr)
{
	uint32_t lpwm_channel_id, i_id, c_id;
	int32_t ret = LPWM_RET_OK;
	lpwm_chn_attr_t *config = NULL;
	struct hobot_lpwm_ins *lpwm = NULL;

	if (!vctx || !attr || !vctx->file) {
		lpwm_err(lpwm, "Config param null!\n");
		return -EINVAL;
	}

	lpwm_channel_id = *(uint32_t *)vctx->file;
	i_id = INS_ID(lpwm_channel_id);
	c_id = COR_ID(lpwm_channel_id);

	lpwm_check_and_return(lpwm, i_id, c_id);
	if (!lpwm) {
		lpwm_err(NULL, "Instance %d has not been probed!\n", i_id);
		return -EINVAL;
	}

	config = &(((lpwm_attr_t *)attr)->lpwm_chn_attr[c_id]);

	ret = lpwm_channel_request(lpwm, c_id, CAMSYS);
	if (ret < 0)
		return ret;

	ret = lpwm_single_channel_config(lpwm, c_id, config);
	if (ret < 0)
		goto err;

	lpwm_single_channel_stream(lpwm, c_id, LPWM_STREAM_ON);

	return ret;
err:
	lpwm_channel_free(lpwm, c_id, CAMSYS);
	return ret;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Get the attr of lpwm core
 * @param[in] *vctx: [0, 15]
 * @param[in] arg: The pointer of the attr of lpwm to get
 * @retval -EINVAL: The vctx or the arg wrong
 * @retval -ENOENT: The device cannot be found by id
 * @retval -EPERM: The device is occupied by backlight, or has not been requested
 * @data_read glpwm_chip: The global struct to get the attr of lpwm core
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int32_t lpwm_get_attr(struct vio_video_ctx *vctx, void *attr)
{
	uint32_t lpwm_channel_id, i_id, c_id;
	int32_t ret = LPWM_RET_OK;
	lpwm_chn_attr_t *config = NULL;
	struct hobot_lpwm_ins *lpwm = NULL;

	if (!vctx || !attr || !vctx->file) {
		lpwm_err(lpwm, "Config param null!\n");
		return -EINVAL;
	}

	lpwm_channel_id = *(uint32_t *)vctx->file;
	i_id = INS_ID(lpwm_channel_id);
	c_id = COR_ID(lpwm_channel_id);

	lpwm_check_and_return(lpwm, i_id, c_id);
	if (!lpwm) {
		lpwm_err(NULL, "Instance %d has not been probed!\n", i_id);
		return -EINVAL;
	}

	config = &(((lpwm_attr_t *)attr)->lpwm_chn_attr[c_id]);
	osal_mutex_lock(&lpwm->con_lock);
	memcpy(config, &lpwm->lpwm_attr[c_id], sizeof(lpwm_chn_attr_t));
	osal_mutex_unlock(&lpwm->con_lock);
	return ret;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Dynamic change trigger_source, offset, period or duty_time of lpwm,
 * 	   it will work in next period and offset in next pps. it is noteworthy
 * 	   that trigger_source work in whole instance, others just in one channel.
 * @param[in] *vctx: [0, 15]
 * @param[in] arg: The pointer of the attr of lpwm to config
 * @retval -EINVAL: The channel id cannot exceed 15
 * @retval -ENOENT: The device cannot be found by id
 * @retval -EPERM: The device is occupied by backlight, or has not been requested
 * @data_read glpwm_chip: The global struct to get the address of every reg to config
 * @data_updated glpwm_chip: Updated the attr in the instance
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int32_t lpwm_change_attr(struct vio_video_ctx *vctx, void *attr)
{
	uint32_t lpwm_channel_id, i_id, c_id, sy_on;
	int32_t i, ret = LPWM_RET_OK;
	struct hobot_lpwm_ins *lpwm = NULL;
	lpwm_dynamic_fps_t *dynamic_attr = NULL;

	if (!vctx || !attr || !vctx->file) {
		lpwm_err(lpwm, "Config param null!\n");
		return -EINVAL;
	}

	dynamic_attr = &((vin_attr_ex_t *)attr)->dynamic_fps_attr;
	lpwm_channel_id = *(uint32_t *)vctx->file;
	i_id = INS_ID(lpwm_channel_id);
	c_id = COR_ID(lpwm_channel_id);

	lpwm_check_and_return(lpwm, i_id, c_id);
	if (!lpwm) {
		lpwm_err(NULL, "Instance %d has not been probed!\n", i_id);
		return -EINVAL;
	}
	lpwm_check_utype(lpwm, CAMSYS);

	sy_on = lpwm->lpwm_attr[c_id].threshold > 0 ? 1 : 0;
	ret = lpwm_dynamic_param_check(lpwm, c_id, dynamic_attr, sy_on);
	if (ret < 0) {
		return ret;
	}

	lpwm_trigger_source_config(lpwm->base, dynamic_attr->trigger_source);
	lpwm_trigger_mode_config(lpwm->base, dynamic_attr->trigger_mode);
	lpwm_offset_config_single(lpwm->base, c_id, dynamic_attr->offset);
	lpwm_cfg1_config_single(lpwm->base, c_id, dynamic_attr->period, dynamic_attr->duty_time);

	osal_mutex_lock(&lpwm->con_lock);
	for (i = 0; i < LPWM_CNUM; i++) {
		lpwm->lpwm_attr[i].trigger_mode = dynamic_attr->trigger_mode > 0 ? 1 : 0;
		lpwm->lpwm_attr[i].trigger_source = dynamic_attr->trigger_source;
	}
	lpwm->lpwm_attr[c_id].offset = dynamic_attr->offset & LPWM_OFFSET_MAX;
	lpwm->lpwm_attr[c_id].duty_time = dynamic_attr->duty_time & LPWM_HIGH_MAX;
	lpwm->lpwm_attr[c_id].period = dynamic_attr->period & LPWM_PERIOD_MAX;
	osal_mutex_unlock(&lpwm->con_lock);

	return LPWM_RET_OK;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Do nothing
 * @param[in] *vctx
 * @retval 0
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int32_t lpwm_start(struct vio_video_ctx *vctx)
{
	int32_t ret = LPWM_RET_OK;

	return ret;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Do nothing
 * @param[in] *vctx
 * @retval 0
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int32_t lpwm_stop(struct vio_video_ctx *vctx)
{
	int32_t ret = LPWM_RET_OK;

	return ret;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Reset the lpwm hw and memset attr, whether or not it is running
 * @param[in] *vctx [0, 15]
 * @retval -ENOENT: The device cannot be found by id
 * @retval -EPERM: The device is occupied by backlight, or has not been requested
 * @data_read glpwm_chip: The global struct to get the address of RST
 * @data_updated glpwm_chip: Memset the attr of the instance
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int32_t lpwm_reset(struct vio_video_ctx *vctx)
{
	uint32_t lpwm_channel_id, i_id, c_id;
	struct hobot_lpwm_ins *lpwm = NULL;

	if (!vctx || !vctx->file) {
		lpwm_err(lpwm, "Config param null!\n");
		return -EINVAL;
	}

	lpwm_channel_id = *(uint32_t *)vctx->file;
	i_id = INS_ID(lpwm_channel_id);
	c_id = COR_ID(lpwm_channel_id);

	lpwm_check_and_return(lpwm, i_id, c_id);
	if (!lpwm) {
		lpwm_err(NULL, "Instance %d has not been probed!\n", i_id);
		return -EINVAL;
	}

	lpwm_check_utype(lpwm, CAMSYS);

	lpwm_ins_reset(lpwm->base);
	memset(&lpwm->lpwm_attr, 0, sizeof(lpwm_chn_attr_t) * LPWM_CNUM);
	return LPWM_RET_OK;
}

/*Linux PWM interface*/
/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Get the pointer of lpwm instance by pwm_chip
 * @param[in] *chip: The pointer of pwm_chip
 * @retval 0: Success
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
static inline struct hobot_lpwm_ins *to_hobot_lpwm_ins(const struct pwm_chip *chip)
{
	return container_of(chip, struct hobot_lpwm_ins, chip);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Request a lpwm channel
 * @param[in] *chip: The pointer of pwm chip
 * @param[in] *pwm: The pointer of pwm device
 * @retval -EINVAL: The param in is null
 * @retval -EBUSY: Cannot request lpwm when it has been occupied
 * @retval -ENOENT: The device cannot be found by id
 * @data_read glpwm_chip: The global struct to get the address of reg to config
 * @data_updated glpwm_chip: Config the attr in this struct
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_lpwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct hobot_lpwm_ins *lpwm = to_hobot_lpwm_ins(chip);
	if (!lpwm) {
		lpwm_err(NULL, "lpwm null\n");
		return -EFAULT;
	}
	return lpwm_channel_request(lpwm, pwm->hwpwm, BACKLIGHT);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Free a lpwm channel
 * @param[in] *chip: The pointer of pwm chip
 * @param[in] *pwm: The pointer of pwm device
 * @retval None
 * @data_read glpwm_chip: The global struct to get the address of reg to config
 * @data_updated glpwm_chip: Memset the attr in this struct
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
void hobot_lpwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct hobot_lpwm_ins *lpwm = to_hobot_lpwm_ins(chip);
	if (!lpwm) {
		lpwm_err(NULL, "lpwm null\n");
		return ;
	}

	if (lpwm->utype != BACKLIGHT) {
		lpwm_err(lpwm, "Camsys/Backlight cannot handle the same device! \
			 current utype: %d\n", lpwm->utype);
		return ;
	}

	lpwm_channel_free(lpwm, pwm->hwpwm, BACKLIGHT);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Dynamic change duty_time, period of lpwm
 * @param[in] *chip: The pointer of pwm chip
 * @param[in] *pwm: The pointer of pwm device
 * @param[in] *state: The state apply to lpwm
 * @retval -ERANGE: The param out of range
 * @retval -EINVAL: The param in is null
 * @data_read glpwm_chip: The global struct to get the address of reg to config
 * @data_updated glpwm_chip: Updated the attr in the instance
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int32_t hobot_lpwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
				const struct pwm_state *state)
{
	int32_t ret = LPWM_RET_OK;
	uint32_t period, duty, i_id, c_id;
	uint64_t period_t, duty_t;
	struct hobot_lpwm_ins *lpwm = to_hobot_lpwm_ins(chip);

	if (lpwm->utype != BACKLIGHT) {
		lpwm_err(lpwm, "Camsys/Backlight cannot handle the same device! \
			 current utype: %d\n", lpwm->utype);
		return -EFAULT;
	}

	i_id = lpwm->dev_idx;
	c_id = pwm->hwpwm;

	period_t = state->period;
	duty_t = state->duty_cycle;
	lpwm_check_utype(lpwm, BACKLIGHT);

	period = (uint32_t)div64_u64(period_t, (uint32_t)LPWM_CLK_MUL_FACTOR);
	duty = (uint32_t)div64_u64(duty_t, (uint32_t)LPWM_CLK_MUL_FACTOR);

	if (((uint32_t)(period_t % LPWM_CLK_MUL_FACTOR) >= LPWM_CLK_MUL_FACTOR/2) &&
	    ((period + 1) < LPWM_PERIOD_MAX)){
		period += 1;
	}

	if (((uint32_t)(duty_t % LPWM_CLK_MUL_FACTOR) >= LPWM_CLK_MUL_FACTOR/2) &&
	    ((duty + 1) < LPWM_HIGH_MAX)){
		duty += 1;
	}

	if (period == 0 || period > LPWM_PERIOD_MAX) {
		lpwm_err(lpwm, "Channel-%d apply fail, period_ns should in (0, 1,000,000,000]ns!\n", c_id);
		return -ERANGE;
	}

	if (duty > period || duty > LPWM_HIGH_MAX) {
		lpwm_err(lpwm, "Channel-%d apply fail, duty_ns should in [0, 4,095,000|period]ns!\n", c_id);
		return -ERANGE;
	}

	/* X5 IP requires period and duty -1 */
	if (period > 0)
		period -= 1;

	if (duty > 0)
		duty -= 1;

	/* offset should be set to zero */
	lpwm_offset_config_single(lpwm->base, c_id, 0);
	lpwm_cfg1_config_single(lpwm->base, c_id, period, duty);

	osal_mutex_lock(&lpwm->con_lock);
	lpwm->lpwm_attr[c_id].offset = 0;
	lpwm->lpwm_attr[c_id].period = period;
	lpwm->lpwm_attr[c_id].duty_time = duty;
	osal_mutex_unlock(&lpwm->con_lock);

	if (state->enabled) {
		lpwm_channel_enable_single(lpwm->base, c_id, 1);
		lpwm_sw_trigger(lpwm->base);
	} else {
		lpwm_channel_enable_single(lpwm->base, c_id, 0);
	}

	return ret;
}

/*lpwm chr dev ops*/
static inline struct lpwm_chip_cdev *to_lpwm_chip_cdev(const struct cdev *_cdev)
{
	return container_of(_cdev, struct lpwm_chip_cdev, cdev);
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Lpwm char dev open
 * @param[in] *pinode: The pointer of inode
 * @param[in] *pfile: The pointer of file
 * @retval 0:Success
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int lpwm_chip_open(struct inode *pinode, struct file *pfile)
{
	struct lpwm_chip_cdev *lpwm_cdev_p = NULL;

	lpwm_cdev_p = to_lpwm_chip_cdev(pinode->i_cdev);
	pfile->private_data = lpwm_cdev_p;
	lpwm_info(lpwm_cdev_p->lpwm, "Char dev open.\n");

	return LPWM_RET_OK;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Lpwm char dev release
 * @param[in] *pinode: The pointer of inode
 * @param[in] *pfile: The pointer of file
 * @retval 0:Success
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
int lpwm_chip_release(struct inode *pinode, struct file *pfile)
{
	struct lpwm_chip_cdev *lpwm_cdev_p = NULL;

	lpwm_cdev_p = (struct lpwm_chip_cdev *)pfile->private_data;
	pfile->private_data = NULL;
	lpwm_info(lpwm_cdev_p->lpwm, "Char dev release.\n");

	return LPWM_RET_OK;
}

/**
 * @NO{S10E05C01}
 * @ASIL{B}
 * @brief Lpwm char dev ioctl
 * @param[in] *pfile: The pointer of file
 * @param[in] cmd: The control cmd
 * @param[in] args: The pointer of user data
 * @retval 0:Success <0:Failure
 * @data_read None
 * @data_updated None
 * @compatibility HW: J6
 * @compatibility SW: 1.0.0
 * @callgraph
 * @callergraph
 * @design
 */
long lpwm_chip_ioctl(struct file *pfile, unsigned int cmd, unsigned long args)
{
	int ret = LPWM_RET_OK;
	void __user *user_ptr = (void __user *)args;
	struct lpwm_chip_cdev *lpwm_cdev_p = NULL;
	struct hobot_lpwm_ins *lpwm = NULL;
	uint64_t (* get_timestamp_func)(uint32_t) = NULL;
	uint64_t frame_s, frame_e;
	uint32_t channel_id, c_id;
	struct vio_video_ctx vctx;
	vin_attr_ex_t vin_attr;
	struct hobot_lpwm_cdev_config cdev_config;
	struct hobot_lpwm_cdev_chg_config cdev_chg_config;
	struct pwm_device *test_device;
	char pwm_info[24] = {0};
	struct pwm_state state;

	lpwm_cdev_p = pfile->private_data;
	if (lpwm_cdev_p) {
		lpwm = lpwm_cdev_p->lpwm;
	} else {
		lpwm_err(NULL, "Ioctl get struct failed!\n");
		return -EFAULT;
	}
	get_timestamp_func = ((struct cim_interface_ops *)
			(lpwm->cim_cops->cops))->cim_get_lpwm_timestamps;

	switch (cmd) {
	case LPWM_CDEV_INIT_ENABLE:
	{
		if (osal_copy_from_app(&cdev_config, user_ptr, sizeof(cdev_config))) {
			lpwm_err(lpwm, "Copy from user fail\n");
			return -EFAULT;
		}

		vctx.file = (void *)&cdev_config.channel_id;
		ret = lpwm_init(&vctx, (void *)&cdev_config.config);
		if (ret < 0) {
			lpwm_err(lpwm, "Init enable failed\n");
			return -EINVAL;
		}
		lpwm_info(lpwm, "Request init start done\n");
	}
	break;
	case LPWM_CDEV_DEINIT_DISABLE:
	{
		if (osal_copy_from_app(&channel_id, user_ptr, sizeof(uint32_t))) {
			lpwm_err(lpwm, "Copy from user fail\n");
			return -EFAULT;
		}
		vctx.file = (void *)&channel_id;

		ret = lpwm_close(&vctx);
		if (ret < 0) {
			lpwm_err(lpwm, "Close disable failed!\n");
			return -EINVAL;
		}
		lpwm_info(lpwm, "Free deinit stop done\n");
	}
	break;
	case LPWM_CDEV_GET_STATE:
	{
		if (osal_copy_from_app(&cdev_config, user_ptr, sizeof(cdev_config))) {
			lpwm_err(lpwm, "Copy from user fail\n");
			return -EFAULT;
		}

		vctx.file = (void *)&cdev_config.channel_id;
		ret = lpwm_get_attr(&vctx, (void *)&cdev_config.config);
		if (ret < 0) {
			lpwm_err(lpwm, "Get state failed\n");
		}

		if (osal_copy_to_app(user_ptr, &cdev_config, sizeof(cdev_config))) {
			lpwm_err(lpwm, "Copy to user fail\n");
			return -EFAULT;
		}
		lpwm_info(lpwm, "Get state done\n");
	}
	break;
	case LPWM_CDEV_CHANGE_CONFIG:
	{
		if (osal_copy_from_app(&cdev_chg_config, user_ptr, sizeof(cdev_chg_config))) {
			lpwm_err(lpwm, "Copy from user fail\n");
			return -EFAULT;
		}

		memcpy(&vin_attr.dynamic_fps_attr, &cdev_chg_config.config, sizeof(lpwm_dynamic_fps_t));
		vctx.file = (void *)&cdev_chg_config.channel_id;
		ret = lpwm_change_attr(&vctx, (void *)&vin_attr);
		if (ret < 0) {
			lpwm_err(lpwm, "Change config failed\n");
			return -EINVAL;
		}
		lpwm_info(lpwm, "Change config done\n");
	}
	break;
	case LPWM_CDEV_RST:
	{
		channel_id = LPWM_TO_CHANNELID(lpwm->dev_idx, 0);
		vctx.file = (void *)&channel_id;

		ret = lpwm_reset(&vctx);
		if (ret < 0) {
			lpwm_err(lpwm, "Reset failed\n");
			return -EINVAL;
		}
		lpwm_info(lpwm, "Reset success!\n");
	}
	break;
	case LPWM_CDEV_PWM_REQUEST:
	{
		if (osal_copy_from_app(&channel_id, user_ptr, sizeof(uint32_t))) {
			lpwm_err(lpwm, "Copy from user fail\n");
			return -EFAULT;
		}
		if (channel_id >= LPWM_ID_MAX) {
			lpwm_err(lpwm, "Request id exceed 16\n");
			return -EINVAL;
		}

		snprintf(pwm_info, sizeof(pwm_info), "lpwm_test_%d", channel_id);
		test_device = pwm_request(channel_id, pwm_info);
		if (IS_ERR(test_device)) {
			lpwm_err(lpwm, "PWM request fail\n");
			return -ENODEV;
		}

		gpwm_dev[channel_id] = test_device;
	}
	break;
	case LPWM_CDEV_PWM_FREE:
	{
		if (osal_copy_from_app(&channel_id, user_ptr, sizeof(uint32_t))) {
			lpwm_err(lpwm, "Copy from user fail\n");
			return -EFAULT;
		}
		if (channel_id >= LPWM_ID_MAX) {
			lpwm_err(lpwm, "Request id exceed 16\n");
			return -EINVAL;
		}
		if (!gpwm_dev[channel_id]) {
			lpwm_err(lpwm, "gpwm_dev null\n");
			return -EINVAL;
		}
		pwm_free(gpwm_dev[channel_id]);
		gpwm_dev[channel_id] = NULL;
	}
	break;
	case LPWM_CDEV_PWM_APPLY:
	{
		if (osal_copy_from_app(&cdev_config, user_ptr, sizeof(cdev_config))) {
			lpwm_err(lpwm, "Copy from user fail\n");
			return -EFAULT;
		}
		c_id = COR_ID(cdev_config.channel_id);

		state.duty_cycle = cdev_config.config.lpwm_chn_attr[c_id].duty_time;
		state.period = cdev_config.config.lpwm_chn_attr[c_id].period;
		if (cdev_config.config.lpwm_chn_attr[c_id].offset)
			state.enabled = true;
		else
			state.enabled = false;

		if (!gpwm_dev[cdev_config.channel_id]) {
			lpwm_err(lpwm, "gpwm_dev null\n");
			return -EINVAL;
		}

		ret = pwm_apply_state(gpwm_dev[cdev_config.channel_id], &state);
		if (ret < 0) {
			lpwm_err(lpwm, "apply state failed\n");
			return -EINVAL;
		}
		lpwm_info(lpwm, "Apply state done\n");
	}
	break;
	case LPWM_CDEV_SIGNAL_CHECK:
	{
		if (osal_copy_from_app(&channel_id, user_ptr, sizeof(uint32_t))) {
			lpwm_err(lpwm, "Copy from user fail\n");
			return -EFAULT;
		}
		frame_s = get_timestamp_func(channel_id);
		msleep(40);
		frame_e = get_timestamp_func(channel_id);
		if (frame_s != frame_e && frame_s != 0 && frame_e != 0)
			ret = LPWM_RET_OK;
		else
			ret = -EFAULT;
	}
	break;
	default:
	{
		lpwm_err(lpwm, "Recognize cmd error\n");
		ret = -EINVAL;
	}
	break;
	}

	return ret;
}
