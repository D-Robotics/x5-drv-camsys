/*   Copyright (C) 2018 Horizon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 */

/**
 * @file camera_dev.c
 *
 * @NO{S10E02C08}
 * @ASIL{B}
 */

#include "hobot_sensor_ops.h"

#include "camera_i2c.h"
#include "camera_sys_api.h"

static uint32_t chn_index[CAMERA_TOTAL_NUMBER] = {0};

/**
 * @var g_sen_ev_state_names
 * sensor event op state name string array
 */
static const char *g_sen_ev_state_names[] = SENSOR_EV_STATE_NAMES;

/**
 * @var g_sen_pre_state_names
 * sensor pre state name string array
 */
static const char *g_sen_pre_state_names[] = SENSOR_PRE_STATE_NAMES;

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor param init as default
 *
 * @param[in] sen: camera sensor device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void camera_dev_param_init(struct sensor_device_s *sen)
{
	struct sensor_param_s *pa;

	if (sen == NULL)
		return;
	pa = &sen->param;

	/* param clear */
	memset(pa, 0, sizeof(struct sensor_param_s));
	/* param default not 0 */
	pa->frame_debug = SENSOR_PARAM_FRAME_DEBUG_DEFAULT;
	pa->stop_wait_ms = SENSOR_PARAM_STOP_WAIT_MS_DEFAULT;
	pa->ctrl_mode = SENSOR_PARAM_CATL_MODE_DEFAULT;
	pa->ae_event_flag = SENSOR_PARAM_AE_EVENT_FLAG_DEFAULT;
	pa->ctrl_timeout_ms = SENSOR_PARAM_CTRL_TIMEOOUT_MS_DEFAULT;
	pa->ev_timeout_ms = SENSOR_PARAM_EV_TIMEOOUT_MS_DEFAULT;
	pa->ev_retry_max = SENSOR_PARAM_EV_RETRY_MAX_DEFAULT;

	return;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor param reset for bit mask
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] mask: the param bit mask to reset
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void camera_dev_param_reset(struct sensor_device_s *sen, int32_t mask)
{
	int32_t *param;
	int32_t i;

	if ((sen == NULL) || (mask == 0))
		return;
	param = (int32_t *)((void *)(&sen->param));
	for (i = 0; i < (sizeof(struct sensor_param_s)/sizeof(int32_t)); i++) {
		if ((mask & (0x1 << i)) == 0)
			continue;
		param[i] = 0;
	}
	return;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor device operation: open
 *
 * @param[in] sen: camera sensor device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_dev_open(struct sensor_device_s *sen)
{
	struct sensor_user_s *user;
	struct os_dev *dev;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	osal_mutex_lock(&user->open_mutex);
	sen_debug(dev, "open as %u\n", user->open_cnt);
	if (user->open_cnt == 0U) {
		osal_mutex_init(&user->mutex); /* PRQA S 3334 */ /* mutex_init macro */
		osal_waitqueue_init(&user->pre_wq);
		osal_waitqueue_init(&user->evu_wq);
		osal_waitqueue_init(&user->stop_wq);
		osal_waitqueue_init(&user->usr_info_wq);
		camera_dev_param_reset(sen, SENSOR_PARAM_OPEN_RESET_MASK);
		memset(&sen->frame, 0, sizeof(struct sensor_frame_s));
		memset(&sen->fps, 0, sizeof(struct sensor_fps_s));
		user->pre_state = SEN_PRE_STATE_DEFAULT;
		user->ev_state = SEN_EV_STATE_DEFAULT;
		user->data_init = 0U;
		user->init_cnt = 0U;
		user->start_cnt = 0U;
		user->stop_wait = 0U;
		user->devflag = 0U;
	}
	user->open_cnt++;
	osal_mutex_unlock(&user->open_mutex);

	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor device operation: release
 *
 * @param[in] sen: camera sensor device struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_dev_release(struct sensor_device_s *sen)
{

	struct sensor_user_s *user;
	struct os_dev *dev;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	osal_mutex_lock(&user->open_mutex);
	if (user->open_cnt > 0U) {
		user->open_cnt--;
	}
	sen_debug(dev, "close as %u\n", user->open_cnt);
	if (user->open_cnt == 0U) {
		if (sen->camera_param.bus_type == I2C_BUS)
			camera_i2c_release(sen);

		if (user->data_init != 0U) {
			camera_sys_lut_free(sen->port);
			user->data_init = 0;
		}
		user->devflag = 0;
	}
	osal_mutex_unlock(&user->open_mutex);
	wake_up_release_work(sen->port);

	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief SENSOR stream on/off use reg
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] on: 0-off as stop, 1-on as start
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t sensor_stream_reg_do(struct sensor_device_s *sen, int32_t on)
{
	int32_t ret = 0;
	struct sensor_user_s *user;
	struct os_dev *dev;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	// TODO: add reg operation of streaming.

        if (on == 1)
                camera_sys_stream_on(sen->port);
        else if (on == 0)
                camera_sys_stream_off(sen->port);
	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief SENSOR frame end to wait with stop_wq
 *
 * @param[in] sen: camera sensor device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t sensor_frame_end_stop_wait(struct sensor_device_s *sen)
{
	struct sensor_user_s *user;
	struct sensor_frame_s *frame;
	struct sensor_param_s *pa;
	struct os_dev *dev;
	uint64_t ts_in, ts_out;
	uint32_t use_ms;
	int32_t ret;

	if ((sen == NULL) || (sen->param.stop_wait_ms == 0))
		 return -ENODEV;
	user = &sen->user;
	frame = &sen->frame;
	pa = &sen->param;
	dev = &sen->osdev;

	ts_in = osal_time_get_ns();
	user->stop_wait = 1U;
	ret = osal_wait_event_interruptible_timeout(user->stop_wq,
			(frame->event == SENSOR_FRAME_END), pa->stop_wait_ms);
	user->stop_wait = 0U;
	ts_out = osal_time_get_ns();
	use_ms = (uint32_t)((ts_out - ts_in) / 1000000U);
	if (ret > 0) {
		/* wait fe done */
		sen_info(dev, "%s %d fe %dms\n", __func__, frame->fs.count, use_ms);
	} else if (ret == 0) {
		/* wait timeout */
		sen_warn(dev, "%s %d fe %dms timeout\n", __func__, frame->fs.count, use_ms);
	} else {
		/* wait failed */
		sen_err(dev, "%s %d fe %dms failed\n", __func__, frame->fs.count, use_ms);
	}

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief SENSOR frame end to wake up the stop_wq
 *
 * @param[in] sen: camera sensor device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void sensor_frame_end_wake_up(struct sensor_device_s *sen)
{
	struct sensor_user_s *user;
	struct sensor_frame_s *frame;
	struct os_dev *dev;

	if (sen == NULL)
		return;
	user = &sen->user;
	frame = &sen->frame;
	dev = &sen->osdev;

	if (user->stop_wait != 0U) {
		sen_debug(dev, "%s %d fe wake\n", __func__, frame->fs.count);
		osal_wake_up(&user->stop_wq);
	}
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief SENSOR frame event record for debug
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] type: the count to get of frame type
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
uint32_t sensor_frame_count_get(struct sensor_device_s *sen, sensor_frame_type_t type)
{
	struct sensor_frame_s *frame;
	struct sensor_frame_record_s *r = NULL;

	if (sen == NULL)
		return 0U;
	frame = &sen->frame;

	switch (type) {
	case SENSOR_FTYPE_FS:
		r = &frame->fs;
		break;
	case SENSOR_FTYPE_FE:
		r = &frame->fe;
		break;
	case SENSOR_FTYPE_FDONE:
		r = &frame->fs_fe;
		break;
	case SENSOR_FTYPE_2A_UPD:
		r = &frame->update_2a;
		break;
	case SENSOR_FTYPE_2A_START:
		r = &frame->start_2a;
		break;
	case SENSOR_FTYPE_2A_DONE:
		r = &frame->done_2a;
		break;
	case SENSOR_FTYPE_2A_TRIG:
		r = &frame->trig_2a;
		break;
	default:
		break;
	}

	return (r != NULL) ? (r->count) : 0U;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief SENSOR frame event record for debug
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] count: the frame count to record
 * @param[in] ts_ns: the record time with ns count
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void sensor_fps_record(struct sensor_device_s *sen, uint32_t count, uint64_t ts_ns)
{
	struct os_dev *dev = &sen->osdev;
	struct sensor_fps_s *fps = &sen->fps;
	struct sensor_fps_record_s *r = &fps->record[fps->rec_index];
	struct sensor_fps_record_s *rn, *rb;
	struct sensor_param_s *pa = &sen->param;
	uint64_t ts_s = ts_ns / 1000000000UL;

	/* last ts record */
	fps->ts_ns_last = ts_ns;
	/* record in this second */
	if (r->ts_s == ts_s) {
		r->count++;
		return;
	}
	/* first ts record */
	if (fps->ts_ns_first == 0UL)
		fps->ts_ns_first = ts_ns;

	/* to next second */
	fps->rec_index = (fps->rec_index + 1U) % SENSOR_FPS_RECORD_MAX;
	rn = &fps->record[fps->rec_index];
	rn->count = 1U;
	rn->start = count;
	rn->ts_s = ts_s;

	/* target fps record */
	if (fps->fps_target == 0U) {
		if (fps->rec_index != 0U)
			fps->fps_target = r->count;
		return;
	}
	/* need check mismatch? */
	if ((pa->frame_debug & SENSOR_FRAME_DEBUG_FPS_RECMIS) == 0U)
		return;
	/* record if fps mismatch */
	if (r->count != fps->fps_target)  {
		rb = &fps->recmis[fps->mis_index];
		memcpy(rb, r, sizeof(struct sensor_fps_record_s));
		fps->mis_index = (fps->mis_index + 1U) % SENSOR_FPS_RECMIS_MAX;
		fps->mis_count++;
		sen_debug(dev, "%s %d: %d-%d fps %d != %d mismatch\n", __func__,
			fps->mis_count, r->start, count, r->count, fps->fps_target);
	}
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief SENSOR frame event record for debug
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] event: the event to record
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void sensor_frame_event_record(struct sensor_device_s *sen, enum _sensor_frame_event_e event)
{
	struct sensor_frame_s *frame;
	struct sensor_frame_record_s *r;
	struct sensor_param_s *pa;
	struct os_dev *dev;
	uint64_t now = 0UL, ts_ns;
	uint32_t diff_ns;
	int32_t frame_good = 0;

	if (sen == NULL)
		return;
	frame = &sen->frame;
	pa = &sen->param;
	dev = &sen->osdev;

	if ((pa->frame_debug & SENSOR_FRAME_DEBUG_ALL_RECORD) != 0U)
		now = osal_time_get_ns();

	switch (event) {
	case SENSOR_FRAME_START:
		if ((frame->fs.count == 0U) || (frame->event == SENSOR_FRAME_END))
			frame_good = 1;
		frame->event = event;
		/* fs record */
		r = &frame->fs;
		r->count++;
		/* fps record */
		if ((pa->frame_debug & SENSOR_FRAME_DEBUG_FPS_RECORD) != 0U)
			sensor_fps_record(sen,  r->count, now);
		if ((pa->frame_debug & SENSOR_FRAME_DEBUG_TIME_EVENT) == 0)  {
			if (frame_good == 0) {
				r->warn++;
				sen_warn(dev, "%s %d fs duplicate\n", __func__, r->count);
			} else {
				sen_debug(dev, "%s %d fs\n", __func__, r->count);
			}
			break;
		}
		ts_ns = r->ts_ns_last;
		r->ts_ns_last = now;
		if (ts_ns == 0UL) {
			/* 1st fs */
			if (frame_good == 0) {
				r->warn++;
				sen_warn(dev, "%s %d --------- fs duplicate\n", __func__, r->count);
			} else {
				sen_info(dev, "%s %d --------- fs\n", __func__, r->count);
			}
			break;
		}
		diff_ns = r->ts_ns_last - ts_ns;
		r->diff_ns_all += diff_ns;
		if ((r->diff_ns_min == 0U) || (r->diff_ns_min > diff_ns)) {
			r->diff_ns_min = diff_ns;
			r->count_min = r->count;
		}
		if (r->diff_ns_max < diff_ns) {
			r->diff_ns_max = diff_ns;
			r->count_max = r->count;
		}
		if (frame_good == 0) {
			r->warn++;
			sen_warn(dev, "%s %d +%d.%03dms fs duplicate\n", __func__, r->count,
				diff_ns / 1000000U, (diff_ns % 1000000U) / 1000U);
		} else {
			sen_debug(dev, "%s %d +%d.%03dms fs\n", __func__, r->count,
				diff_ns / 1000000U, (diff_ns % 1000000U) / 1000U);
		}
		break;
	case SENSOR_FRAME_END:
		if (frame->event == SENSOR_FRAME_START)
			frame_good = 1;
		frame->event = event;
		/* wake up stop wait if need */
		sensor_frame_end_wake_up(sen);
		/* fe record */
		r = &frame->fe;
		r->count++;
		if ((pa->frame_debug & SENSOR_FRAME_DEBUG_TIME_EVENT) == 0) {
			if (frame_good == 0) {
				r->warn++;
				sen_warn(dev, "%s %d fe duplicate\n", __func__, r->count);
			} else {
				frame->fs_fe.count++;
				sen_debug(dev, "%s %d fe\n", __func__, r->count);
			}
			break;
		}
		ts_ns = r->ts_ns_last;
		r->ts_ns_last = now;
		if (ts_ns == 0UL) {
			/* 1st fe */
			diff_ns = r->ts_ns_last - frame->fs.ts_ns_last;
			if (frame_good == 0) {
				r->warn++;
				sen_warn(dev, "%s %d -%d.%03dms fe duplicate\n", __func__, r->count,
					diff_ns / 1000000U, (diff_ns % 1000000U) / 1000U);
			} else {
				frame->fs_fe.count++;
				sen_info(dev, "%s %d -%d.%03dms fe\n", __func__, r->count,
					diff_ns / 1000000U, (diff_ns % 1000000U) / 1000U);
			}
			break;
		}
		diff_ns = r->ts_ns_last - ts_ns;
		r->diff_ns_all += diff_ns;
		if ((r->diff_ns_min == 0U) || (r->diff_ns_min > diff_ns)) {
			r->diff_ns_min = diff_ns;
			r->count_min = r->count;
		}
		if (r->diff_ns_max < diff_ns) {
			r->diff_ns_max = diff_ns;
			r->count_max = r->count;
		}
		/* fs&fe record */
		if (frame_good == 0) {
			r->warn++;
			sen_warn(dev, "%s %d +%d.%03dms fe duplicate\n", __func__, r->count,
				diff_ns / 1000000U, (diff_ns % 1000000U) / 1000U);
			break;
		}
		r = &frame->fs_fe;
		r->count++;
		r->ts_ns_last = frame->fs.ts_ns_last;
		diff_ns = frame->fe.ts_ns_last - frame->fs.ts_ns_last;
		r->diff_ns_all += diff_ns;
		if ((r->diff_ns_min == 0U) || (r->diff_ns_min > diff_ns)) {
			r->diff_ns_min = diff_ns;
			r->count_min = r->count;
		}
		if (r->diff_ns_max < diff_ns) {
			r->diff_ns_max = diff_ns;
			r->count_max = r->count;
		}
		sen_debug(dev, "%s %d -%d.%03dms fe\n", __func__, r->count,
			diff_ns / 1000000U, (diff_ns % 1000000U) / 1000U);
		break;
	case SENSOR_FRAME_EVENT_MAX:
	default:
		sen_debug(dev, "%s event %d not support\n", __func__, event);
		break;
	}

	return;
}

int32_t camera_isi_sensor_chn_number(void)
{
        uint32_t chn_number = 0;
        uint32_t i = 0;

        for(i = 0; i < CAMERA_TOTAL_NUMBER; i ++) {
                if (chn_index[i] == 1)
                        chn_number ++;
        }

        return chn_number;
}
EXPORT_SYMBOL(camera_isi_sensor_chn_number);

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief SENSOR frame 2a record for debug
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] step: 0-updata, 1-start, 2-done, see sensor_frame_2a_step_e
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void sensor_frame_2a_record(struct sensor_device_s *sen, int32_t step)
{
	struct sensor_frame_s *frame;
	struct sensor_frame_record_s *r = NULL, *f;
	struct sensor_param_s *pa;
	struct os_dev *dev;
	uint64_t now = 0UL, ts_ns;
	uint32_t diff_ns;
	const char *step_desp;

	if (sen == NULL)
		return;
	frame = &sen->frame;
	pa = &sen->param;
	dev = &sen->osdev;

	if ((pa->frame_debug & SENSOR_FRAME_DEBUG_TIME_2A) != 0U)
		now = osal_time_get_ns();

	switch (step) {
	case SENSOR_F2AS_UPDATE:
		step_desp = "update";
		/* 2a update record */
		r = &frame->update_2a;
		r->count++;
		if ((pa->frame_debug & SENSOR_FRAME_DEBUG_TIME_2A) == 0)
			break;
		ts_ns = r->ts_ns_last;
		r->ts_ns_last = now;
		if (ts_ns == 0UL)
			break;
		diff_ns = r->ts_ns_last - ts_ns;
		r->diff_ns_all += diff_ns;
		if ((r->diff_ns_min == 0U) || (r->diff_ns_min > diff_ns)) {
			r->diff_ns_min = diff_ns;
			r->count_min = r->count;
		}
		if (r->diff_ns_max < diff_ns) {
			r->diff_ns_max = diff_ns;
			r->count_max = r->count;
		}
		break;
	case SENSOR_F2AS_TRIGGER:
		step_desp = "trigger";
		/* 2a start record */
		r = &frame->start_2a;
		r->count++;
		if ((pa->frame_debug & SENSOR_FRAME_DEBUG_TIME_2A) == 0)
			break;
		ts_ns = r->ts_ns_last;
		r->ts_ns_last = now;
		if (ts_ns == 0UL)
			break;
		diff_ns = r->ts_ns_last - ts_ns;
		r->diff_ns_all += diff_ns;
		if ((r->diff_ns_min == 0U) || (r->diff_ns_min > diff_ns)) {
			r->diff_ns_min = diff_ns;
			r->count_min = r->count;
		}
		if (r->diff_ns_max < diff_ns) {
			r->diff_ns_max = diff_ns;
			r->count_max = r->count;
		}
		/* fs/fe & start_2a record */
		r = &frame->trig_2a;
		r->count++;
		if (pa->ae_event_flag == SENSOR_FRAME_START)
			f = &frame->fs;
		else if (pa->ae_event_flag == SENSOR_FRAME_END)
			f = &frame->fe;
		else
			break;
		r->ts_ns_last = frame->start_2a.ts_ns_last;
		diff_ns = r->ts_ns_last - f->ts_ns_last;
		r->diff_ns_all += diff_ns;
		if ((r->diff_ns_min == 0U) || (r->diff_ns_min > diff_ns)) {
			r->diff_ns_min = diff_ns;
			r->count_min = r->count;
		}
		if (r->diff_ns_max < diff_ns) {
			r->diff_ns_max = diff_ns;
			r->count_max = r->count;
		}
		break;
	case SENSOR_F2AS_DONE:
		step_desp = "done";
		/* 2a done record */
		r = &frame->done_2a;
		r->count++;
		if ((pa->frame_debug & SENSOR_FRAME_DEBUG_TIME_2A) == 0)
			break;
		ts_ns = r->ts_ns_last;
		r->ts_ns_last = now;
		if (ts_ns == 0UL)
			break;
		diff_ns = r->ts_ns_last - frame->start_2a.ts_ns_last;
		r->diff_ns_all += diff_ns;
		if ((r->diff_ns_min == 0U) || (r->diff_ns_min > diff_ns)) {
			r->diff_ns_min = diff_ns;
			r->count_min = r->count;
		}
		if (r->diff_ns_max < diff_ns) {
			r->diff_ns_max = diff_ns;
			r->count_max = r->count;
		}
		break;
	case SENSOR_F2AS_UPDATE_WARN:
		step_desp = "updata_warn";
		r = &frame->update_2a;
		r->warn++;
		break;
	case SENSOR_F2AS_TRIGGER_WARN:
		step_desp = "trigger_warn";
		r = &frame->trig_2a;
		r->warn++;
		break;
	case SENSOR_F2AS_DONE_WARN:
		step_desp = "done_warn";
		r = &frame->done_2a;
		r->warn++;
		break;
	case SENSOR_F2AS_INVALID:
	default:
		step_desp = "invalid";
		break;
	}

	sen_debug(dev, "%s %d %s\n", __func__,
		(r != NULL) ? r->count : 0U, step_desp);
	return;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief SENSOR frame 2a id check
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] id: the 2a id(fs count) to check
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t sensor_frame_2a_check(struct sensor_device_s *sen, uint32_t id)
{
	uint32_t id_cur, id_exp;
	struct sensor_param_s *pa;

	if (sen == NULL)
		return -ENODEV;
	if (id == 0U)
		return 0;
	pa = &sen->param;

	id_cur = sensor_frame_count_get(sen, SENSOR_FTYPE_FS);
	id_exp = (pa->ae_event_flag == SENSOR_FRAME_START) ? (id + 1U) : id;

	return (id_cur > id_exp) ? (-1) : 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief SENSOR ctrl mode set (only support AUTO/other exchange)
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] ctrl_mode: the ctrl_mode to set, see sensor_ctrl_mode_e
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t sensor_ctrl_mode_set(struct sensor_device_s *sen, int32_t ctrl_mode)
{
	struct sensor_param_s *pa;
	struct os_dev *dev;
	int32_t ret = 0;
	const char *ctrlm_names[] = SENSOR_CTRLM_NAMES;
	int32_t mode = (ctrl_mode >= SENSOR_CTRLM_INVALID) ? SENSOR_CTRLM_AUTO : ctrl_mode;

	if (sen == NULL)
		return -ENODEV;
	pa = &sen->param;
	dev = &sen->osdev;

	if (pa->ctrl_mode != mode) {
		/* only support AUTO->x or x->AUTO */
		if ((mode == SENSOR_CTRLM_AUTO) || (pa->ctrl_mode == SENSOR_CTRLM_AUTO)) {
			pa->ctrl_mode = mode;
			sen_info(dev, "ctrl_mode set to %s\n", ctrlm_names[mode]);
		}  else {
			sen_debug(dev, "ctrl_mode %s to %s ignore\n",
				ctrlm_names[pa->ctrl_mode], ctrlm_names[mode]);
			ret = -EINVAL;
		}
	}

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief SENSOR ctrl mode get
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] ctrl_mode: the ctrl_mode to set,
 *
 * @return >=0:Success(see sensor_ctrl_mode_e), <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t sensor_ctrl_mode_get(struct sensor_device_s *sen)
{
	struct sensor_param_s *pa;
	if (sen == NULL)
		return -ENODEV;
	pa = &sen->param;

	return pa->ctrl_mode;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor updata tuning params when init
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: struct sensor_turning_data
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_tuning_update_param(struct sensor_device_s *sen, unsigned long arg)
{
	int32_t  ret = 0;
	struct sensor_user_s *user;
	sensor_tuning_data_t tuning_data;
	struct os_dev *dev;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	if (arg == 0UL) {
		sen_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}
	if (osal_copy_from_app((void *)&tuning_data, (void __user *)arg,
		sizeof(sensor_tuning_data_t))) {
		sen_err(dev, "%s arg copy error\n", __func__);
		return -ENOMEM;
	}

	osal_mutex_lock(&user->mutex);
	if (tuning_data.bus_type == I2C_BUS) {// i2c
		ret = camera_i2c_open(sen, tuning_data.bus_num,
				tuning_data.sensor_name, tuning_data.sensor_addr);
		if (ret < 0) {
			sen_err(dev, "%s i2c open error %d\n", __func__, ret);
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
	}
	if (user->data_init != 0U) {
		camera_sys_lut_free(sen->port);
		user->data_init = 0U;
	}
	ret = camera_sys_tuning_set(sen->port, &tuning_data);
	if (ret < 0) {
		sen_err(dev, "%s set error %d\n", __func__, ret);
		osal_mutex_unlock(&user->mutex);
		return ret;
	}
	common_init((uint8_t)sen->port, (uint8_t)tuning_data.mode);
	user->data_init = 1;
	sen_info(dev, "%s done\n", __func__);
	chn_index[(uint8_t)sen->port] = 1;
	osal_mutex_unlock(&user->mutex);
	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor set ae share flag
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: ae share flag
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_set_ae_share(struct sensor_device_s *sen, unsigned long arg)
{
	struct sensor_user_s *user;
	struct os_dev *dev;
	uint32_t ae_share_flag = 0U;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	if (arg == 0UL) {
		sen_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}
	if (osal_copy_from_app(&ae_share_flag, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
		sen_err(dev, "%s get data from user failed", __func__);
		return -ENOMEM;
	}
	osal_mutex_lock(&user->mutex);
	sen->param.ae_share_flag = ae_share_flag;

	sen_info(dev, "%s 0x%x done\n", __func__, ae_share_flag);
	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor set input param
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: input param struct
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_set_input_param(struct sensor_device_s *sen, unsigned long arg)
{
	struct sensor_user_s *user;
	struct os_dev *dev;
	sensor_input_param_t input_param;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	if (arg == 0UL) {
		sen_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}
	if (osal_copy_from_app((void *)&input_param, (void __user *)arg,
		sizeof(sensor_input_param_t))) {
		sen_err(dev, "%s arg copy error\n", __func__);
		return -ENOMEM;
	}
	osal_mutex_lock(&user->mutex);
	sen->param.ts_compensate = input_param.ts_compensate;

	sen_info(dev, "%s ts_compensate %d done\n", __func__, sen->param.ts_compensate);
	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor init operation real done
 *
 * @param[in] des: sensor device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void sensor_init_done(struct sensor_device_s *des)
{
	struct sensor_user_s *user = &des->user;

	if (user->ev_state == SEN_EV_STATE_CANCEL)
		user->ev_state = SEN_EV_STATE_DEFAULT;


	return;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief sensor deinit operation real done
 *
 * @param[in] des: sensor device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void sensor_deinit_done(struct sensor_device_s *des)
{

	return;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief sensor event operation cancel (in mutex lock)
 *
 * @param[in] sen: sensor device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void sensor_ev_cancel(struct sensor_device_s *sen)
{
	struct sensor_user_s *user = &sen->user;

	if (user->ev_state != SEN_EV_STATE_DEFAULT) {
		user->ev_state = SEN_EV_STATE_CANCEL;
		osal_wake_up(&user->evu_wq);
		if (user->ev_wait > 0U)
			osal_wake_up(&user->evk_wq);
	}

	return;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief sensor operation user wait (in nutex lock)
 *
 * @param[in] sen: sensor device struct
 * @param[out] ev_info: the opreation info to get
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t sensor_evu_do(struct sensor_device_s *sen, sensor_event_info_t *ev_info)
{
	struct sensor_user_s *user = &sen->user;
	struct os_dev * dev = &sen->osdev;
	int32_t ret;

	if ((user->ev_state == SEN_EV_STATE_DEFAULT) ||
		(user->ev_state == SEN_EV_STATE_FINISH)) {
		sen_info(dev, "%s: %s to %s", __func__,
			g_sen_ev_state_names[user->ev_state],
			g_sen_ev_state_names[SEN_EV_STATE_WAIT]);
		user->ev_state = SEN_EV_STATE_WAIT;
	}
	osal_mutex_unlock(&user->mutex);

	ret = osal_wait_event_interruptible(user->evu_wq,
			((user->ev_state == SEN_EV_STATE_CANCEL) ||
			 (((user->ev_state == SEN_EV_STATE_DOING) ||
			   (user->ev_state == SEN_EV_STATE_DOING_NOWAIT)) &&
			  (user->ev_info != NULL))));
	osal_mutex_lock(&user->mutex);
	if (ret < 0) {
		return ret;
	}
	if (user->ev_state == SEN_EV_STATE_CANCEL)
		return -ESRCH;
	if (user->ev_info == NULL)
		return -EFAULT;

	memcpy(ev_info, user->ev_info, sizeof(sensor_event_info_t));

	return 0;
}

/**
 * @NO{S10E02C11}
 * @ASIL{B}
 * @brief sensor operation user done (in nutex lock)
 *
 * @param[in] sen: sensor device struct
 * @param[in] result: operation result: 0-done, others-error
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t sensor_evu_done(struct sensor_device_s *sen, int32_t result)
{
	uint32_t ret = 0, ev_state;
	struct sensor_user_s *user = &sen->user;
	struct os_dev *dev = &sen->osdev;

	ev_state = user->ev_state;
	user->ev_state = (result == 0) ? SEN_EV_STATE_DONE : SEN_EV_STATE_ERROR;

	if (user->ev_wait == 0U) {
		sen_info(dev, "%s: %s to %s to %s\n", __func__,
			g_sen_ev_state_names[ev_state],
			g_sen_ev_state_names[user->ev_state],
			g_sen_ev_state_names[SEN_EV_STATE_FINISH]);
		user->ev_state = SEN_EV_STATE_FINISH;
		if (ev_state != SEN_EV_STATE_DOING_NOWAIT)
			ret = -EFAULT;
	} else {
		osal_wake_up(&user->evk_wq);
	}

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor init operation request
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_init_req(struct sensor_device_s *sen, unsigned long arg)
{
	int32_t ret = 0;
	struct sensor_user_s *user;
	struct os_dev *dev;
	uint32_t timeout_ms = 0U, wait = 0U;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	if (arg != 0UL) {
		if (osal_copy_from_app(&timeout_ms, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
			sen_err(dev, "%s get data from user failed", __func__);
			return -ENOMEM;
		}
	}
	osal_mutex_lock(&user->mutex);
	if (user->init_cnt == 0U) {
		if (user->pre_state == (uint32_t)SEN_PRE_STATE_DEFAULT) {
			user->pre_state = (uint32_t)SEN_PRE_STATE_INITING;
			sen_info(dev, "%s cmd: %s", __func__,
				g_sen_pre_state_names[user->pre_state]);
		} else if (user->pre_state == (uint32_t)SEN_PRE_STATE_INITING) {
			wait = 1;
		} else {
			sen_info(dev, "%s cmd: %s drop", __func__,
				g_sen_pre_state_names[user->pre_state]);
			ret = -EACCES;
		}
	} else {
		sen_info(dev, "%s cmd: %s drop", __func__,
			g_sen_pre_state_names[user->pre_state]);
			ret = -EACCES;
	}
	osal_mutex_unlock(&user->mutex);
	if (wait != 0U) {
		user->pre_done = (bool)false;
		if (timeout_ms != 0U) {
			osal_wait_event_interruptible_timeout(user->pre_wq,
				user->pre_done, timeout_ms);
		} else {
			osal_wait_event_interruptible(user->pre_wq, user->pre_done);
		}
		osal_mutex_lock(&user->mutex);
		if ((user->init_cnt == 0U) &&
			(user->pre_state == (uint32_t)SEN_PRE_STATE_DEFAULT)) {
			user->pre_state = (uint32_t)SEN_PRE_STATE_INITING;
			sen_info(dev, "%s cmd: wait & %s", __func__,
			       g_sen_pre_state_names[user->pre_state]);
		} else {
			sen_info(dev, "%s cmd: wait & drop", __func__);
			ret = -EACCES;
		}
		osal_mutex_unlock(&user->mutex);
	}

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor init operation result
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_init_result(struct sensor_device_s *sen, unsigned long arg)
{
	int32_t ret = 0;
	struct sensor_user_s *user;
	struct os_dev *dev;
	uint32_t result = 0U, wake = 0U;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	if (arg == 0UL) {
		sen_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}

	if (osal_copy_from_app(&result, (uint32_t *)arg, sizeof(uint32_t)) != 0U) {
		sen_err(dev, "%s get data from user failed", __func__);
		return -EFAULT;
	}
	osal_mutex_lock(&user->mutex);
	if ((user->init_cnt == 0U) &&
		(user->pre_state == (uint32_t)SEN_PRE_STATE_INITING)) {
		if (result != 0U) {
			user->pre_state = (uint32_t)SEN_PRE_STATE_DEFAULT;
		} else {
			sensor_init_done(sen);
			user->pre_state = (uint32_t)SEN_PRE_STATE_INITED;
		}
		wake = 1U;
	}
	if (result == 0U) {
		user->init_cnt ++;
	}
	osal_mutex_unlock(&user->mutex);
	if (wake != 0U) {
		sen_info(dev, "%s cmd: %s wake", __func__,
				(result != 0U) ? "falied" : "done");
		user->pre_done = (bool)true;
		osal_wake_up(&user->pre_wq);
	} else {
		sen_info(dev, "%s cmd: %s drop", __func__,
				(result != 0U) ? "falied" : "done");
	}

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor deinit operation request and done
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_deinit_req(struct sensor_device_s *sen, unsigned long arg)
{
	int32_t ret = 0;
	struct sensor_user_s *user;
	struct os_dev *dev;
	int32_t through = 0;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	if (arg != 0U) {
		if (osal_copy_from_app(&through, (uint32_t *)arg, sizeof(uint32_t)) != 0) {
			sen_err(dev, "%s get data from user failed", __func__);
			return -EFAULT;
		}
	}
	osal_mutex_lock(&user->mutex);
	if (through != 0) {
		user->init_cnt = 0U;
	} else if (user->init_cnt > 0U) {
		user->init_cnt--;
	}
	sen_info(dev, "%s cmd: %u %s", __func__, user->init_cnt,
		(user->init_cnt != 0U) ? "drop" : "real");
	if (user->init_cnt == 0U) {
		if (user->start_cnt > 0U) {
			sensor_streaming_do(sen, 0);
			user->start_cnt = 0U;
		}
		sensor_ev_cancel(sen);
		user->pre_state = SEN_PRE_STATE_DEFAULT;
		if (user->data_init != 0U) {
			camera_sys_lut_free(sen->port);
			common_exit((uint8_t)sen->port);
			user->data_init = 0U;
		}
		sensor_deinit_done(sen);
	} else {
		ret = -EACCES;
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor start as stream on
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_start(struct sensor_device_s *sen, unsigned long arg)
{
	int32_t ret = 0;
	struct sensor_user_s *user;
	struct os_dev *dev;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	osal_mutex_lock(&user->mutex);

	if (user->start_cnt == 0)
		ret = sensor_stream_reg_do(sen, 1);

	if (ret < 0) {
		sen_err(dev, "%s error %d\n", __func__, ret);
	} else {
		if (user->start_cnt == 0)
			sen_info(dev, "%s real done\n", __func__);
		else
			sen_debug(dev, "%s %d done\n", __func__, user->start_cnt);
		user->start_cnt++;
	}

	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor stop as stream off
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_stop(struct sensor_device_s *sen, unsigned long arg)
{
	int32_t ret = 0;
	struct sensor_user_s *user;
	struct os_dev *dev;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	osal_mutex_lock(&user->mutex);

	if (user->start_cnt == 1) {
		/* wait fe & real stop */
		sensor_frame_end_stop_wait(sen);
		ret = sensor_stream_reg_do(sen, 0);
	}

	if (ret < 0) {
		sen_err(dev, "%s error %d\n", __func__, ret);
	} else {
		if (user->start_cnt == 1)
			sen_info(dev, "%s real done\n", __func__);
		else
			sen_debug(dev, "%s %d done\n", __func__, user->start_cnt - 1);
		user->start_cnt--;
	}

	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor event call wait
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_event_get(struct sensor_device_s *sen, unsigned long arg)
{
	int32_t ret = 0;
	struct sensor_user_s *user;
	struct os_dev *dev;
	sensor_event_info_t ev_info;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;


	if (arg == 0UL) {
		sen_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;

	}
	osal_mutex_lock(&user->mutex);

	ret = sensor_evu_do(sen, &ev_info);
	if (ret == 0) {
		if (osal_copy_to_app((void __user *)arg, (void *)&ev_info, sizeof(sensor_event_info_t)) != 0U) {
			sen_err(dev, "%s ev_info to user %p error", __func__, (void __user *)arg);
			ret = -EFAULT;
		}
		sen_debug(dev, "%s ev %d:%d\n", __func__, ev_info.type, ev_info.data);
	} else {
		sen_debug(dev, "%s ev cancel\n", __func__);
	}

	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor event operation done and put status
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_event_put(struct sensor_device_s *sen, unsigned long arg)
{
	int32_t ret = 0, result = 0;
	struct sensor_user_s *user;
	struct os_dev *dev;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	if (arg == 0UL) {
		sen_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}

	if (osal_copy_from_app(&result, (uint32_t *)arg, sizeof(uint32_t)) != 0U) {
		sen_err(dev, "%s get data from user failed", __func__);
		return -EFAULT;
	}
	osal_mutex_lock(&user->mutex);

	ret = sensor_evu_done(sen, result);

	sen_debug(dev, "%s ev %d:%d %s%s\n", __func__,
		(user->ev_info != NULL) ? (user->ev_info->type) : -1,
		(user->ev_info != NULL) ? (user->ev_info->data) : -1,
		(result == 0) ? "done" : "error",
		(ret == 0) ? "" : " failed");

	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera sensor update ae info
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_update_ae_info(struct sensor_device_s *sen, unsigned long arg)
{
	struct sensor_user_s *user;
	struct os_dev *dev;
	uint32_t timeout = 0U;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	if (arg == 0UL) {
		sen_err(dev, "%s arg NULL error\n", __func__);
		return -EINVAL;
	}
	if (osal_copy_from_app((void *)&timeout, (void __user *)arg, sizeof(uint32_t))) {
		sen_err(dev, "%s arg copy error\n", __func__);
		return -ENOMEM;
	}
	osal_mutex_lock(&user->mutex);

	sen_info(dev, "%s done\n", __func__);
	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera set intrinsic param
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_set_intrinsic_param(struct sensor_device_s *sen,
		unsigned long arg)
{
	int32_t ret = 0;
	struct sensor_user_s *user;
	struct os_dev *dev;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;

	if (copy_from_user((void *)&sen->user_info,
		(void __user *)arg, sizeof(struct cam_usr_info_s))) {
		sen_err(dev, "%s port %d error\n", __func__, sen->port);
		ret = -EINVAL;
	}

	user->devflag = 1;
	wake_up_interruptible(&(user->usr_info_wq));
	sen_info(dev, "%s port %d done\n", __func__, sen->port);

	return ret;
}


/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief camera get intrinsic param
 *
 * @param[in] sen: camera sensor device struct
 * @param[in] arg: operation arg
 *
 * @return 0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
int32_t camera_get_intrinsic_param(struct sensor_device_s *sen,
		unsigned long arg)
{
	int32_t ret = 0;
	struct sensor_user_s *user;
	struct os_dev *dev;
	struct sensor_param_s *pa;
	int32_t iparam_timeout_ms;

	if (sen == NULL)
		return -ENODEV;
	user = &sen->user;
	dev = &sen->osdev;
	pa = &sen->param;

	if (user->devflag == 0u) {
		iparam_timeout_ms = pa->iparam_timeout_ms;
		ret = osal_wait_event_interruptible_timeout(user->usr_info_wq,
			user->devflag, iparam_timeout_ms);
		if (user->devflag == 0u) {
			sen_warn(dev, "%s wait %dms for ready ret %d\n",
						__func__, iparam_timeout_ms, ret);
			return -EAGAIN;
		}
	}

	if (copy_to_user((void __user *)arg,
		(void *)&sen->user_info, sizeof(struct cam_usr_info_s))) {
		sen_err(dev, "%s port %d error\n", __func__, sen->port);
		return -EINVAL;
	}

	sen_info(dev, "%s port %d done\n", __func__, sen->port);
	return 0;
}
