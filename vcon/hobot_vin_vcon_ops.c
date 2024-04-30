/*
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/**
 * @file hobot_vin_vcon_ops.c
 *
 * @NO{S10E01C02}
 * @ASIL{B}
 */

#include "hobot_vin_vcon_ops.h"

#include "hobot_vin_common.h"
#include "hobot_dev_vin_node.h"

/**
 * @var g_vcon_dev_type_names
 * covert vcon dev type name string array
 */
const char *g_vcon_dev_type_names[] = VCON_DEV_TYPTE_NAMES;
/**
 * @var g_vcon_attr_names
 * covert vcon attr name string array
 */
const char *g_vcon_attr_names[] = VCON_ATTR_NAMES;
/**
 * @var g_vcon_attr_v_nums
 * covert vcon attr vflag number array
 */
const char g_vcon_attr_v_nums[] = VCON_ATTR_V_NUMS;
/**
 * @var g_vcon
 * covert global vcon device struct
 */
static struct vin_vcon_s g_vcon = {};

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief vcon sub dummy attach: if driver not insmod
 *
 * @param[in] index: the device index
 * @param[in] flow_id: the flow id to used
 * @param[in] add_id: the add id to used
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
static int32_t dummy_attach(int32_t index, int32_t flow_id, int32_t add_id)
{
	vcon_err(NULL, "%s: %d flow%d add:%d\n",
		__func__, index, flow_id, add_id);
	return vcon_sub_dummy_return();
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief vcon sub dummy detach: if driver not insmod
 *
 * @param[in] index: the device index
 * @param[in] flow_id: the flow id to used
 * @param[in] add_id: the add id to used
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
static int32_t dummy_detach(int32_t index, int32_t flow_id, int32_t add_id)
{
	vcon_err(NULL, "%s: %d flow%d add:%d\n",
		__func__, index, flow_id, add_id);
	return vcon_sub_dummy_return();
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief vcon sub dummy start: if driver not insmod
 *
 * @param[in] index: the device index
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
static int32_t dummy_start(int32_t index)
{
	vcon_err(NULL, "%s: %d\n", __func__, index);
	return vcon_sub_dummy_return();
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief vcon sub dummy stop: if driver not insmod
 *
 * @param[in] index: the device index
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
int32_t dummy_stop(int32_t index)
{
	vcon_err(NULL, "%s: %d\n", __func__, index);
	return vcon_sub_dummy_return();
}

/**
 * @NO{S10E02C08}
 * @ASIL{B}
 * @brief vcon sub dummy event: if driver not insmod
 *
 * @param[in] index: the device index
 * @param[in] event_id: the event id
 * @param[in] event_data: the event data
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
int32_t dummy_event(int32_t index, int32_t event_id, void *event_data)
{
	vcon_err(NULL, "%s: %d %d %p\n",
		__func__, index, event_id, event_data);
	return vcon_sub_dummy_return();
}

static struct vcon_sub_ops_s dummy_deserial_ops = {
	.attach = dummy_attach,
	.detach = dummy_detach,
	.start = dummy_start,
	.stop = dummy_stop,
	.event = dummy_event,
	.end_magic = VCON_SUB_OPS_END_MAGIC(VCON_COPT_DESERIAL),
};
static struct vcon_sub_ops_s dummy_sensor_ops = {
	.attach = dummy_attach,
	.detach = dummy_detach,
	.start = dummy_start,
	.stop = dummy_stop,
	.event = dummy_event,
	.end_magic = VCON_SUB_OPS_END_MAGIC(VCON_COPT_SENSOR),
};

#define vcon_sub_op(t, ops) (((struct vcon_sub_ops_s *)(g_vcon.sub_cops[t]->cops))->ops)
#define vcon_sub_op_invalid(t, ops) \
		((g_vcon.sub_cops[t] == NULL) || (g_vcon.sub_cops[t]->cops == NULL) || \
		 (vcon_sub_op(t, ops) == NULL))

/**
 * @def vcon_snr_op
 * covert vcon sub device sensor operation
 */
#define vcon_snr_op(ops, ...) \
	({ \
		int32_t ret; \
		if (vcon_sub_op_invalid(VCON_SENSOR, ops)) \
			ret = -EFAULT; \
		ret = vcon_sub_op(VCON_SENSOR, ops)(__VA_ARGS__); \
		ret; \
       	})

/**
 * @def vcon_des_op
 * covert vcon sub device deserial operation
 */
#define vcon_des_op(ops, ...) \
	({ \
		int32_t ret; \
		if (vcon_sub_op_invalid(VCON_DESERIAL, ops)) \
			ret = -EFAULT; \
		ret = vcon_sub_op(VCON_DESERIAL, ops)(__VA_ARGS__); \
		ret; \
       	})

/**
 * @NO{S10E01C02U}
 * @ASIL{B}
 * @brief get vcon ctx match with mask first
 *
 * @param[in] vcon: vcon device struct
 * @param[in] mask: ctx valid mask
 *
 * @return !NULL:ctx as Success, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct vcon_ctx_s* vcon_ctx_first_from_mask(struct vcon_device_s *vcon, uint32_t mask)
{
	uint32_t i = 0U;

	while ((mask != 0U) && (i < VCON_CTX_MAX_NUM)) {
		if ((mask & (0x1U << i)) != 0U)
			return &vcon->ctx[i];
		mask &= ~(0x1U << i);
		i++;
	}
	return NULL;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon deserial subdev attach to vflow
 *
 * @param[in] vcon: vcon device struct
 * @param[in] iattr: attr struct for attach
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
static int32_t vcon_deserial_attach(struct vcon_device_s *vcon, struct vcon_inter_attr_s *iattr)
{
	struct vcon_user_s *user;
	struct vcon_grp_s *grp;
	struct vcon_ctx_s *ctx;
	struct os_dev *dev;
	int32_t ret = 0;

	if ((vcon == NULL) || (iattr == NULL) || (iattr->ctx_id >= VCON_CTX_MAX_NUM))
		return -EINVAL;

	/* return done if not need deserial */
	if (iattr->deserial_attach == 0)
		return 0;
	user = &vcon->user;
	grp = &vcon->grp;
	dev = &vcon->osdev;

	osal_mutex_lock(&user->mutex);
	if (iattr->attach != 0) {
		/* to attach */
		if ((grp->group_mask != 0) && grp->deserial_attached == 0) {
			/* has attached without deserial */
			ctx = vcon_ctx_first_from_mask(vcon, grp->group_mask);
			vcon_err(dev, "vcon%d has attached 0x%x by vflow%d.ctx%d without deserial error\n",
				vcon->index, grp->group_mask,
				(ctx != NULL) ? ctx->flow_id : -1,
				(ctx != NULL) ? ctx->ctx_id : -1);
			osal_mutex_unlock(&user->mutex);
			return -EBUSY;
		}
		if (grp->deserial_attached != 0U) {
			/* has attached with deserial */
			if (grp->deserial_index != iattr->deserial_index) {
				/* has attached with the different deserial_index */
				ctx = vcon_ctx_first_from_mask(vcon, grp->group_mask);
				vcon_err(dev, "vcon%d has attached 0x%x to des%d by vflow%d.ctx%d error\n",
						vcon->index, grp->group_mask, grp->deserial_index,
						(ctx != NULL) ? ctx->flow_id : -1,
						(ctx != NULL) ? ctx->ctx_id : -1);
				osal_mutex_unlock(&user->mutex);
				return -EACCES;
			}
			/* has attached with the same deserial_index */
			if ((grp->group_mask & (0x1U << iattr->ctx_id)) != 0U) {
				/* has attached with the same ctx */
				ctx = vcon_ctx_first_from_mask(vcon, (0x1U << iattr->ctx_id));
				vcon_info(dev, "vcon%d has attached 0x%x by vflow%d.ctx%d\n",
					vcon->index, grp->group_mask,
					(ctx != NULL) ? ctx->flow_id : -1,
					(ctx != NULL) ? ctx->ctx_id : -1);
				osal_mutex_unlock(&user->mutex);
				return ret;
			}
			/* has attached with the same deserial_index but different ctx */
		}
		/* attach to deserial.link new ctx */
		ret = vcon_des_op(attach, iattr->deserial_index, iattr->flow_id, iattr->deserial_link);
		if (ret < 0) {
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		grp->deserial_attached = 1U;
		grp->deserial_index = iattr->deserial_index;
		grp->group_mask |= (0x1U << iattr->ctx_id);
		grp->deserial_link[iattr->ctx_id] = iattr->deserial_link;
		vcon_info(dev, "flow%d.ctx%d attach deserial%d.link%d done\n",
			iattr->flow_id, iattr->ctx_id, iattr->deserial_index, iattr->deserial_link);
	} else {
		/* to dettach */
		if (grp->deserial_attached == 0U) {
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		if (((grp->group_mask & ~(0x1U << iattr->ctx_id)) == 0U) &&
			(grp->deserial_start_cnt != 0))	{
			vcon_des_op(stop, iattr->deserial_index);
			grp->deserial_start_cnt = 0U;
		}
		if ((grp->group_mask & (0x1U << iattr->ctx_id)) != 0U) {
			ret = vcon_des_op(detach, iattr->deserial_index, iattr->flow_id, grp->deserial_link[iattr->ctx_id]);
			if (ret < 0) {
				osal_mutex_unlock(&user->mutex);
				return ret;
			}
		}
		grp->group_mask &= ~(0x1U << iattr->ctx_id);
		if (grp->group_mask == 0)
			grp->deserial_attached = 0U;
		vcon_info(dev, "flow%d.ctx%d detach deserial%d.link%d done\n",
			iattr->flow_id, iattr->ctx_id, iattr->deserial_index, iattr->deserial_link);
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon deserial start operation
 *
 * @param[in] vcon: vcon device struct
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
static int32_t vcon_deserial_start(struct vcon_device_s *vcon)
{
	struct vcon_user_s *user;
	struct vcon_grp_s *grp;
	struct os_dev *dev;
	int32_t ret = 0;

	if (vcon == NULL)
		return -EINVAL;
	user = &vcon->user;
	grp = &vcon->grp;
	dev = &vcon->osdev;

	osal_mutex_lock(&user->mutex);
	if (grp->deserial_attached == 0U) {
		/* for sensor without deserial */
		osal_mutex_unlock(&user->mutex);
		return 0;
	}
	if (grp->deserial_start_cnt == 0U) {
		ret = vcon_des_op(start, grp->deserial_index);
		if  (ret < 0) {
			osal_mutex_unlock(&user->mutex);
			vcon_err(dev, "deserial%d start error %d\n",
				grp->deserial_index, ret);
			return ret;
		}
		vcon_info(dev, "deserial%d start real done\n",
			grp->deserial_index);
	} else {
		vcon_debug(dev, "deserial%d start %d\n",
			grp->deserial_index, grp->deserial_start_cnt);
	}
	grp->deserial_start_cnt++;
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon deserial stop operation
 *
 * @param[in] vcon: vcon device struct
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
static int32_t vcon_deserial_stop(struct vcon_device_s *vcon)
{
	struct vcon_user_s *user;
	struct vcon_grp_s *grp;
	struct os_dev *dev;
	int32_t ret = 0;

	if (vcon == NULL)
		return -EINVAL;
	user = &vcon->user;
	grp = &vcon->grp;
	dev = &vcon->osdev;

	osal_mutex_lock(&user->mutex);
	if (grp->deserial_attached == 0U) {
		/* for sensor without deserial */
		osal_mutex_unlock(&user->mutex);
		return 0;
	}
	if (grp->deserial_start_cnt == 1U) {
		ret = vcon_des_op(stop, grp->deserial_index);
		if  (ret < 0) {
			vcon_err(dev, "deserial%d stop error %d\n",
				grp->deserial_index, ret);
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		vcon_info(dev, "deserial%d stop real done\n",
			grp->deserial_index);
	} else {
		vcon_debug(dev, "deserial%d stop %d\n",
			grp->deserial_index, grp->deserial_start_cnt - 1);
	}
	if (grp->deserial_start_cnt > 0)
		grp->deserial_start_cnt--;
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon deserial event operation for contex
 *
 * @param[in] vcon: vcon device struct
 * @param[in] event_id: event id
 * @param[in] event_data: event data info
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
static int32_t vcon_deserial_event(struct vcon_device_s *vcon, int32_t event_id, void *event_data)
{
	struct vcon_grp_s *grp;
	int32_t ret;

	if (vcon == NULL)
		return -EINVAL;
	grp = &vcon->grp;

	if (grp->deserial_attached == 0U) {
		return -EPERM;
	}
	ret = vcon_des_op(event, grp->deserial_index, event_id, event_data);

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon sensor subdev attach to vflow
 *
 * @param[in] vcon: vcon device struct
 * @param[in] iattr: attr struct for attach
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
static int32_t vcon_sensor_attach(struct vcon_device_s *vcon, struct vcon_inter_attr_s *iattr)
{
	struct vcon_user_s *user;
	struct vcon_grp_s *grp;
	struct vcon_ctx_s *ctx;
	struct os_dev *dev;
	int32_t ret = 0;

	if ((vcon == NULL) || (iattr == NULL) || (iattr->ctx_id >= VCON_CTX_MAX_NUM))
		return -EINVAL;

	/* return done if not need sensor */
	if (iattr->sensor_attach == 0)
		return 0;
	ctx = &vcon->ctx[iattr->ctx_id];
	user = &vcon->user;
	grp = &vcon->grp;
	dev = &vcon->osdev;

	osal_mutex_lock(&user->mutex);
	if (iattr->attach != 0) {
		/* to attach */
		if (ctx->sensor_attached != 0U) {
			/* has attached ? */
			ret = (ctx->flow_id != iattr->flow_id) ? (-EBUSY) : 0;
			if (ret < 0)
				vcon_err(dev, "vcon%d has attached 0x%x by vflow%d.ctx%d: vflow%d error\n",
					vcon->index, grp->group_mask,
					ctx->flow_id, ctx->ctx_id, iattr->flow_id);
			else
				vcon_info(dev, "vcon%d has attached 0x%x by vflow%d.ctx%d\n",
					vcon->index, grp->group_mask,
					ctx->flow_id, ctx->ctx_id);
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		/* attach to sensor */
		ret = vcon_snr_op(attach, iattr->sensor_index, iattr->flow_id, iattr->ctx_id);
		if (ret < 0) {
			vcon_err(dev, "flow%d.ctx%d attach sensor%d error %d\n",
				iattr->flow_id, iattr->ctx_id, iattr->sensor_index, ret);
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		/* attach done */
		ctx->flow_id = iattr->flow_id;
		ctx->vcon_id = vcon->index;
		ctx->ctx_id = iattr->ctx_id;
		ctx->sensor_attached = 1U;
		ctx->sensor_index = iattr->sensor_index;
		ctx->sensor_start_cnt = 0U;
		g_vcon.ctx_flow[ctx->flow_id] = ctx;
		/* change group here if no deserial */
		if (iattr->deserial_attach != 0)
			grp->group_mask |= (0x1U << iattr->ctx_id);
		vcon_info(dev, "flow%d.ctx%d attach sensor%d done\n",
			iattr->flow_id, iattr->ctx_id, iattr->sensor_index);
	} else {
		/* to dettach */
		if (ctx->sensor_attached == 0U) {
			/* has detached ? */
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		if (ctx->sensor_start_cnt > 0U) {
			/* forget stop? auto stop it */
			vcon_snr_op(stop, ctx->sensor_index);
			ctx->sensor_start_cnt = 0U;
		}
		/* detach from sensor */
		ret = vcon_snr_op(detach, iattr->sensor_index, iattr->flow_id, iattr->ctx_id);
		if (ret < 0) {
			vcon_err(dev, "flow%d.ctx%d detach sensor%d error %d\n",
				iattr->flow_id, iattr->ctx_id, iattr->sensor_index, ret);
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		ctx->sensor_attached = 0U;
		g_vcon.ctx_flow[ctx->flow_id] = NULL;
		/* change group here if no deserial */
		if (iattr->deserial_attach != 0)
			grp->group_mask &= ~(0x1U << iattr->ctx_id);
		vcon_info(dev, "flow%d.ctx%d detach sensor%d done\n",
			iattr->flow_id, iattr->ctx_id, iattr->sensor_index);
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon sensor start operation for contex
 *
 * @param[in] vcon: vcon device struct
 * @param[in] ctx_id: contex index to operation
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
static int32_t vcon_sensor_start(struct vcon_device_s *vcon, uint32_t ctx_id)
{
	struct vcon_user_s *user;
	struct vcon_ctx_s *ctx;
	struct os_dev *dev;
	int32_t ret = 0;

	if ((vcon == NULL)  || (ctx_id >= VCON_CTX_MAX_NUM))
		return -EINVAL;
	ctx = &vcon->ctx[ctx_id];
	user = &vcon->user;
	dev = &vcon->osdev;

	osal_mutex_lock(&user->mutex);
	if (ctx->sensor_attached == 0U) {
		/* sensor must attach before start */
		osal_mutex_unlock(&user->mutex);
		return -EPERM;
	}
	if (ctx->sensor_start_cnt == 0U) {
		ret = vcon_snr_op(start, ctx->sensor_index);
		if  (ret < 0) {
			vcon_err(dev, "sensor%d start error %d\n",
				ctx->sensor_index, ret);
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		vcon_info(dev, "sensor%d start real done\n",
			ctx->sensor_index);
	} else {
		vcon_debug(dev, "sensor%d start %d\n",
			ctx->sensor_index, ctx->sensor_start_cnt);
	}
	ctx->sensor_start_cnt++;
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon sensor stop operation for contex
 *
 * @param[in] vcon: vcon device struct
 * @param[in] ctx_id: contex index to operation
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
static int32_t vcon_sensor_stop(struct vcon_device_s *vcon, uint32_t ctx_id)
{
	struct vcon_user_s *user;
	struct vcon_ctx_s *ctx;
	struct os_dev *dev;
	int32_t ret = 0;

	if ((vcon == NULL)  || (ctx_id >= VCON_CTX_MAX_NUM))
		return -EINVAL;
	ctx = &vcon->ctx[ctx_id];
	user = &vcon->user;
	dev = &vcon->osdev;

	osal_mutex_lock(&user->mutex);
	if (ctx->sensor_attached == 0U) {
		/* sensor must attach before stop */
		osal_mutex_unlock(&user->mutex);
		return -EPERM;
	}
	if (ctx->sensor_start_cnt == 1U) {
		ret = vcon_snr_op(stop, ctx->sensor_index);
		if  (ret < 0) {
			vcon_err(dev, "sensor%d stop error %d\n",
				ctx->sensor_index, ret);
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		vcon_info(dev, "sensor%d stop real done\n",
			ctx->sensor_index);
	} else {
		vcon_debug(dev, "sensor%d stop %d\n",
			ctx->sensor_index, ctx->sensor_start_cnt - 1);
	}
	if (ctx->sensor_start_cnt > 0U)
		ctx->sensor_start_cnt--;
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon sensor event operation for contex
 *
 * @param[in] vcon: vcon device struct
 * @param[in] ctx_id: context index to operation
 * @param[in] event_id: event id
 * @param[in] event_data: event data info
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
static int32_t vcon_sensor_event(struct vcon_device_s *vcon, uint32_t ctx_id, int32_t event_id, void *event_data)
{
	struct vcon_ctx_s *ctx;
	int32_t ret;

	if ((vcon == NULL)  || (ctx_id >= VCON_CTX_MAX_NUM))
		return -EINVAL;
	ctx = &vcon->ctx[ctx_id];

	if (ctx->sensor_attached == 0U) {
		return -EPERM;
	}
	ret = vcon_snr_op(event, ctx->sensor_index, event_id, event_data);

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief get vcon device struct by vcon index
 *
 * @param[in] index: the index of vcon
 *
 * @return !0:vcon device struct pointer, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct vcon_device_s* vcon_get_by_id(int32_t index)
{
	if ((index < 0) || (index >= VCON_DEV_MAX_NUM))
		return NULL;
	return &g_vcon.vcon[index];
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief get vcon device struct by rx index
 *
 * @param[in] index: the rx index of vcon
 *
 * @return !0:vcon device struct pointer, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct vcon_device_s* vcon_get_by_rx(int32_t index)
{
	if ((index < 0) || (index >= VCON_DEV_MAX_NUM))
		return NULL;
	return g_vcon.vcon_rx[index];
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief get vcon device struct by tx index
 *
 * @param[in] index: the tx index of vcon
 *
 * @return !0:vcon device struct pointer, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct vcon_device_s* vcon_get_by_tx(int32_t index)
{
	if ((index < 0) || (index >= VCON_DEV_MAX_NUM))
		return NULL;
	return g_vcon.vcon_tx[index];
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon device index covert from vctx
 *
 * @param[in] vctx: vin contex for vcon
 *
 * @return !0:vcon device struct pointer, NULL:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct vcon_device_s* vcon_get_by_vctx(struct vio_video_ctx *vctx)
{
	uint32_t index;
	struct vcon_device_s *vcon;

	if ((vctx == NULL) || (vctx->device == NULL))
		return NULL;

	index = ((struct j6_vin_node_dev *)(vctx->device))->hw_id;
	vcon = vcon_get_by_rx(index);

	return vcon;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief get the real vcon device struct by current config
 *
 * @param[in] vctx: vin contex for vcon
 *
 * @return !NULL: the real vcon device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct vcon_device_s* vcon_real(struct vcon_device_s *vcon)
{
	struct vcon_attr_s *config = &vcon->config;

	if (((config->attr_valid & VCON_ATTR_V_TYPE) != 0) &&
		(config->vcon_type == VCON_FOLLOW) &&
		(config->vcon_link < VCON_DEV_MAX_NUM))
		return &g_vcon.vcon[config->vcon_link];
	return vcon;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief get the follow vcon device struct by current config
 *
 * @param[in] vctx: vin contex for vcon
 *
 * @return !NULL: the follow vcon device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct vcon_device_s* vcon_follow(struct vcon_device_s *vcon)
{
	struct vcon_attr_s *config = &vcon->config;

	if (((config->attr_valid & VCON_ATTR_V_TYPE) != 0) &&
		(config->vcon_type == VCON_MAIN) &&
		(config->vcon_link < VCON_DEV_MAX_NUM))
		return &g_vcon.vcon[config->vcon_link];
	return NULL;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief get the vcon device struct by flow id if attached
 *
 * @param[in] flow_id: the flow id to match
 * @param[out] pctx: if !NULL to store the ctx struct pointer
 *
 * @return !NULL: the vcon device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static struct vcon_device_s* vcon_by_flow(int32_t flow_id, struct vcon_ctx_s **pctx)
{
	struct vcon_ctx_s *ctx;

	if (flow_id >= VCON_FLOW_MAX_NUM)
		return NULL;
	ctx = g_vcon.ctx_flow[flow_id];
	if (pctx != NULL)
		*pctx = ctx;
	if ((ctx != NULL) && (ctx->vcon_id < VCON_DEV_MAX_NUM))
		return &g_vcon.vcon[ctx->vcon_id];

	return NULL;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief flush the vcon attr config from info struct
 *
 * @param[in] vcon: vcon device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void vcon_config_flush(struct vcon_device_s *vcon)
{
	int32_t v_this = vcon->info.attr_valid;
	int32_t *p_this = &vcon->info.bus_main;
	struct vcon_device_s *vcon_r = vcon_real(vcon);
	struct vcon_device_s *vcon_f = vcon_follow(vcon);
	struct vcon_device_s *vcon_s = NULL;
	int32_t v_to = vcon->config.attr_valid;
	int32_t *p_to = &vcon->config.bus_main;
	int32_t v_from, *p_from;
	int32_t v_real, *p_real;
	int32_t i, j, b, n;

	/* flush to this */
	if ((vcon_r != vcon) && (vcon_r->user.init_cnt > 0)) {
		/* the main vcon has inited: use real config */
		v_real = vcon_r->config.attr_valid;
		p_real = &vcon_r->config.bus_main;
	} else {
		v_real = vcon_r->info.attr_valid;
		p_real = &vcon_r->info.bus_main;
	}
	n = 0;
	for (i = 0; i < ARRAY_SIZE(g_vcon_attr_v_nums); i++) {
		/* loop for valid bit */
		b = (0x1 << i);
		/* form: mipi attr from this vcon, others from real vcon  */
		p_from = ((b & VCON_ATTR_V_MIPI) != 0) ? p_this : p_real;
		v_from = ((b & VCON_ATTR_V_MIPI) != 0) ? v_this : v_real;

		if (((v_to & b) == 0) && (v_from & b) != 0) {
			/* copy from info if config not valid */
			for (j = 0; j < g_vcon_attr_v_nums[i]; j++) {
				p_to[n + j] = p_from[n + j];
			}
			v_to |= b;
		}
		n += g_vcon_attr_v_nums[i];
	}
	vcon->config.attr_valid = v_to;

	/* need to sync? */
	if (vcon_f != NULL) {
		/* this is main: sync to follow */
		vcon_s = vcon_f;
	} else if (vcon != vcon_r) {
		/* this is follow: sync to main */
		vcon_s = vcon_r;
	}
	if (vcon_s == NULL)
		return;

	/* sync to linked */
	v_from = vcon->config.attr_valid;
	p_from = &vcon->config.bus_main;
	v_to = vcon_s->config.attr_valid;
	p_to = &vcon_s->config.bus_main;
	osal_mutex_lock(&vcon_s->user.mutex);
	n = 0;
	for (i = 0; i < ARRAY_SIZE(g_vcon_attr_v_nums); i++) {
		/* loop for valid bit */
		b = (0x1 << i);
		if ((b & (VCON_ATTR_V_MIPI | VCON_ATTR_V_TYPE)) != 0)
			continue;
		if ((v_from & b) != 0) {
			/* sync if valid */
			for (j = 0; j < g_vcon_attr_v_nums[i]; j++) {
				p_to[n + j] = p_from[n + j];
			}
			v_to |= b;
		} else {
			v_to &= ~b;
		}
		n += g_vcon_attr_v_nums[i];
	}
	vcon_s->config.attr_valid = v_to;
	osal_mutex_unlock(&vcon_s->user.mutex);

	return;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief set the vcon attr config from set or info struct
 *
 * @param[in] vcon: vcon device struct
 * @param[in] set: vcon attr to set, NULL init from info
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static void vcon_config_set(struct vcon_device_s *vcon, struct vcon_attr_s *set)
{
	struct vcon_attr_s *config = &vcon->config;
	struct vcon_attr_s *info = &vcon->info;

	if (set == NULL) {
		memset(config, 0, sizeof(struct vcon_attr_s));
		config->attr_valid = VCON_ATTR_V_TYPE;
		config->vcon_type = info->vcon_type;
		config->vcon_link = info->vcon_link;
	} else {
		memcpy(config, set, sizeof(struct vcon_attr_s));
	}

	vcon_config_flush(vcon);
	return;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief attach the tx info to the rx vcon
 *
 * @param[in] vcon: vcon device struct
 * @param[in] iattr: the vcon inter attr to attach
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
static int32_t vcon_tx_attach(struct vcon_device_s *vcon, struct vcon_inter_attr_s *iattr)
{
	struct vcon_device_s *vcon_tx;
	struct vcon_user_s *user;
	struct vcon_grp_s *grp;
	struct os_dev *dev;
	int32_t ret = 0;

	if ((vcon == NULL) || (iattr == NULL) || (iattr->ctx_id >= VCON_CTX_MAX_NUM))
		return -EINVAL;

	/* return done if not need tx */
	if (iattr->tx_attach == 0)
		return 0;
	user = &vcon->user;
	grp = &vcon->grp;
	dev = &vcon->osdev;

	osal_mutex_lock(&user->mutex);
	if (iattr->attach != 0) {
		/* to attach */
		if (vcon->tx_index >= 0) {
			/* has attached with tx */
			if (vcon->tx_index != iattr->tx_index) {
				/* has attached with the different tx_index */
				vcon_err(dev, "vcon%d attach to tx%d but attached tx%d error\n",
					vcon->index, iattr->tx_index, vcon->tx_index);
				ret = -EACCES;
			}
			/* has attached with the same tx_index */
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		/* attach to tx  */
		vcon_tx = vcon_get_by_tx(iattr->tx_index);
		if (vcon_tx == NULL) {
			vcon_err(dev, "vcon%d attach to tx%d invalid error\n",
				vcon->index, iattr->tx_index);
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		vcon->tx_index = iattr->tx_index;
		/* init tx config */
		vcon_config_set(vcon_tx, NULL);
		vcon_info(dev, "flow%d.ctx%d attach tx%d done\n",
			iattr->flow_id, iattr->ctx_id, vcon->tx_index);
	} else {
		/* to dettach */
		if (vcon->tx_index < 0) {
			osal_mutex_unlock(&user->mutex);
			return ret;
		}
		vcon_info(dev, "flow%d.ctx%d detach tx%d done\n",
			iattr->flow_id, iattr->ctx_id, vcon->tx_index);
		vcon->tx_index = VCON_ATTR_INVALID;
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

static void vcon_event_setcb(int32_t set);
/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vin vcon device open
 *
 * @param[in] vctx: vcon port index
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
static int32_t vcon_open(struct vio_video_ctx *vctx)
{
	struct vcon_device_s *vcon = vcon_get_by_vctx(vctx);
	struct vcon_user_s *user;
	struct os_dev *dev;

	if (vcon == NULL)
		return -ENODEV;
	user = &vcon->user;
	dev = &vcon->osdev;

	vcon_event_setcb(1);

	osal_mutex_lock(&user->open_mutex);
	if (user->open_cnt == 0) {
		user->init_cnt = 0;
		memset(&vcon->grp, 0, sizeof(vcon->grp));
		memset(&vcon->ctx, 0, sizeof(vcon->ctx));
		vcon_config_set(vcon, NULL);
		vcon_info(dev, "vcon%d open %d: flow%d ctx%d real\n",
			vcon->index, user->open_cnt, vctx->flow_id, vctx->ctx_id);
	} else {
		vcon_debug(dev, "vcon%d open %d: flow%d ctx%d\n",
			vcon->index, user->open_cnt, vctx->flow_id, vctx->ctx_id);
	}
	user->open_cnt ++;
	osal_mutex_unlock(&user->open_mutex);

	return 0;
}


/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon device close
 *
 * @param[in] vctx: vcon device index
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
static int32_t vcon_close(struct vio_video_ctx *vctx)
{
	struct vcon_device_s *vcon = vcon_get_by_vctx(vctx);
	struct vcon_user_s *user;
	struct vcon_grp_s *grp;
	struct vcon_ctx_s *ctx;
	struct os_dev *dev;
	int32_t i;

	if (vcon == NULL)
		return -ENODEV;
	user = &vcon->user;
	dev = &vcon->osdev;

	osal_mutex_lock(&user->open_mutex);
	if (user->open_cnt == 1) {
		/* auto detach sensor/deserial/tx */
		struct vcon_inter_attr_s iattr = {
			.attach = 0,
			.deserial_attach = 1,
			.sensor_attach = 1,
			.tx_attach = 1,
		};
		for (i = 0; i < VCON_CTX_MAX_NUM; i++) {
			ctx = &(vcon_real(vcon)->ctx[i]);
			if (ctx->sensor_attached) {
				iattr.flow_id = ctx->flow_id;
				iattr.ctx_id = ctx->ctx_id;
				iattr.sensor_index = ctx->sensor_index;
				(void)vcon_sensor_attach(vcon_real(vcon), &iattr);
				grp = &(vcon_real(vcon)->grp);
				if (grp->deserial_attached) {
					vcon_deserial_attach(vcon_real(vcon), &iattr);
				}
			}
		}
		if (vcon_real(vcon)->tx_index >= 0) {
			vcon_tx_attach(vcon_real(vcon), &iattr);
		}
		/* detach done */
		user->init_cnt = 0;
		vcon_info(dev, "vcon%d close %d: flow%d ctx%d real\n",
			vcon->index, user->open_cnt - 1, vctx->flow_id, vctx->ctx_id);
	} else {
		vcon_debug(dev, "vcon%d close %d: flow%d ctx%d\n",
			vcon->index, user->open_cnt -1, vctx->flow_id, vctx->ctx_id);
	}
	if (user->open_cnt > 0)
		user->open_cnt--;
	osal_mutex_unlock(&user->open_mutex);

	return 0;
}

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon set init attr to replace default hw info
 *
 * @param[in] vctx: vcon device index
 * @param[in] arg: struct vcon_attr_s point to set
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
static int32_t vcon_set_init_attr(struct vio_video_ctx *vctx, void *arg)
{
	struct vcon_device_s *vcon = vcon_get_by_vctx(vctx);
	struct vcon_user_s *user;
	struct vcon_grp_s *grp;
	struct os_dev *dev;

	if (vcon == NULL)
		return -ENODEV;
	user = &vcon->user;
	grp = &vcon->grp;
	dev = &vcon->osdev;

	osal_mutex_lock(&user->mutex);
	if ((grp->group_mask != 0) && (arg != NULL)) {
		vcon_info(dev, "set attr after attached ignore\n");
	}
	if ((arg != NULL) || (user->init_cnt > 0))
		vcon_config_set(vcon, (struct vcon_attr_s *)arg);
	if (arg != NULL) {
		user->init_cnt ++;
		vcon_info(dev, "init attr %d done\n", user->init_cnt);
	}
	osal_mutex_unlock(&user->mutex);

	return 0;
}

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon attach operation fucntion
 *
 * @param[in] vctx: vcon device index
 * @param[in] arg: struct vcon_inter_attr_s point to set
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
static int32_t vcon_set_internal_attr(struct vio_video_ctx *vctx, void *arg)
{
	struct vcon_device_s *vcon = vcon_get_by_vctx(vctx);
	struct vcon_inter_attr_s iattr;
	struct os_dev *dev;
	int32_t ret;

	if (vcon == NULL)
		return -ENODEV;
	if (arg == NULL)
		return -EINVAL;
	dev = &vcon->osdev;

	if (vctx->flow_id >= VCON_FLOW_MAX_NUM) {
		vcon_err(dev, "set internal attr flow%d ctx%d invalid\n",
			vctx->flow_id, vctx->ctx_id);
		return -EINVAL;
	}

	/* fill attach vflow info */
	memcpy(&iattr, arg, sizeof(iattr));
	iattr.flow_id = vctx->flow_id;
	iattr.ctx_id = vctx->ctx_id;

	ret = vcon_deserial_attach(vcon_real(vcon), &iattr);
	if ((iattr.attach != 0) && (ret < 0))
		return ret;

	ret = vcon_sensor_attach(vcon_real(vcon), &iattr);
	if ((iattr.attach != 0) && (ret < 0)) {
		iattr.attach = 0;
		vcon_deserial_attach(vcon_real(vcon), &iattr);
		return ret;
	}
	ret = vcon_tx_attach(vcon_real(vcon), &iattr);
	if ((iattr.attach != 0) && (ret < 0)) {
		iattr.attach = 0;
		vcon_sensor_attach(vcon_real(vcon), &iattr);
		vcon_deserial_attach(vcon_real(vcon), &iattr);
		return ret;
	}

	vcon_info(dev, "set internal attr %s done\n",
		(iattr.attach != 0) ? "attach" : "detach");

	return 0;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon current attr fix link as the real phy for follow type
 *
 * @param[in] vcon: vcon device struct
 * @param[out] config: struct vcon_attr_s point to store
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
static int32_t vcon_get_attr_from(struct vcon_device_s *vcon, struct vcon_attr_s *config)
{
	struct os_dev *dev;
	struct vcon_device_s *vcon_r;

	if ((vcon == NULL) || (config == NULL))
		return -EINVAL;
	vcon_r = vcon_real(vcon);
	dev = &vcon->osdev;

	/* copy config */
	memcpy(config, &vcon->config, sizeof(vcon->config));
	/* fix vcon_link for follow */
	if (vcon != vcon_r) {
		if (config->rx_phy_mode != 0)
			config->vcon_link = vcon_r->config.rx_phy_index;
		else if (config->tx_phy_mode != 0)
			config->vcon_link = vcon_r->config.tx_phy_index;

		vcon_info(dev, "get attr %s bus%d %s%d(%d) done\n",
			(config->attr_valid != 0) ? "valid" : "invalid", config->bus_main,
			(config->rx_phy_mode != 0) ? "rx" : ((config->tx_phy_mode != 0) ? "tx" : "no"),
			(config->rx_phy_mode != 0) ? config->rx_phy_index : ((config->tx_phy_mode != 0) ? config->tx_phy_index : -1),
			config->vcon_link);
	} else {
		vcon_info(dev, "get attr %s bus%d %s%d done\n",
			(config->attr_valid != 0) ? "valid" : "invalid", config->bus_main,
			(config->rx_phy_mode != 0) ? "rx" : ((config->tx_phy_mode != 0) ? "tx" : "no"),
			(config->rx_phy_mode != 0) ? config->rx_phy_index : ((config->tx_phy_mode != 0) ? config->tx_phy_index : -1));
	}
	return 0;
}

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon current attr get
 *
 * @param[in] vctx: vcon device index
 * @param[out] arg: struct vcon_attr_s point to get
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
static int32_t vcon_get_attr(struct vio_video_ctx *vctx, void *arg)
{
	struct vcon_device_s *vcon = vcon_get_by_vctx(vctx);
	struct vcon_device_s *vcon_tx;
	struct vcon_attr_s *config;
	struct vcon_user_s *user;
	struct os_dev *dev;
	int32_t ret = 0;

	if (vcon == NULL)
		return -ENODEV;
	if (arg == NULL)
		return -EINVAL;
	user = &vcon->user;
	dev = &vcon->osdev;
	config = (struct vcon_attr_s *)arg;

	osal_mutex_lock(&user->mutex);
	if ((config->tx_phy_mode == 1) && (config->rx_phy_mode == 0)) {
		/* get tx vcon attr */
		vcon_tx = vcon_get_by_tx(vcon_real(vcon)->tx_index);
		if (vcon_tx == NULL) {
			vcon_err(dev, "get tx attr but not attach error\n");
			osal_mutex_unlock(&user->mutex);
			return -EACCES;
		}
		ret = vcon_get_attr_from(vcon_tx, config);
	} else {
		/* get rx vcon attr */
		ret = vcon_get_attr_from(vcon, config);
	}
	osal_mutex_unlock(&user->mutex);

	return ret;
}

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon start call to sub deserial/sensor
 *
 * @param[in] vctx: vcon device index
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
static int32_t vcon_start(struct vio_video_ctx *vctx)
{
	struct vcon_device_s *vcon = vcon_get_by_vctx(vctx);
	int32_t ictx;
	int32_t ret;

	if (vcon == NULL)
		return -ENODEV;
	if (vctx == NULL)
		return -EINVAL;
	ictx = vctx->ctx_id;

	ret = vcon_deserial_start(vcon_real(vcon));
	if (ret < 0)
		return ret;

	ret = vcon_sensor_start(vcon_real(vcon), ictx);
	if (ret < 0) {
		vcon_deserial_stop(vcon_real(vcon));
		return ret;
	}

	return 0;
}

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon stop call to sub deserial/sensor
 *
 * @param[in] vctx: vcon device index
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
static int32_t vcon_stop(struct vio_video_ctx *vctx)
{
	struct vcon_device_s *vcon = vcon_get_by_vctx(vctx);
	int32_t ictx;

	if (vcon == NULL)
		return -ENODEV;
	if (vctx == NULL)
		return -EINVAL;
	ictx = vctx->ctx_id;

	vcon_sensor_stop(vcon_real(vcon), ictx);
	vcon_deserial_stop(vcon_real(vcon));

	return 0;
}

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon error call to sub deserial/sensor
 *
 * @param[in] vctx: vcon device index
 * @param[in] arg: error info struct
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
static int32_t vcon_error_event(struct vio_video_ctx *vctx, void *arg)
{
	struct vcon_device_s *vcon = vcon_get_by_vctx(vctx);
	int32_t ictx;

	if (vcon == NULL)
		return -ENODEV;
	if (vctx == NULL)
		return -EINVAL;
	ictx = vctx->ctx_id;

	vcon_sensor_event(vcon_real(vcon), ictx, VCON_E_ERROR, arg);
	vcon_deserial_event(vcon_real(vcon), VCON_E_ERROR, arg);

	return 0;
}

#if 0
/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon frame sync call to sub deserial/sensor
 *
 * @param[in] vctx: vcon device index
 * @param[in] arg: error info struct
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
static int32_t vcon_frame_event(struct vio_video_ctx *vctx, void *arg)
{
	struct vcon_device_s *vcon = vcon_get_by_vctx(vctx);
	int32_t ictx;

	if (vcon == NULL)
		return -ENODEV;
	if (vctx == NULL)
		return -EINVAL;
	ictx = vctx->ctx_id;

	vcon_sensor_event(vcon_real(vcon), ictx, VCON_E_FRAME, arg);
	vcon_deserial_event(vcon_real(vcon), VCON_E_FRAME, arg);

	return 0;
}
#endif

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vcon reset call to sub deserial/sensor
 *
 * @param[in] vctx: vcon device index
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
static int32_t vcon_reset(struct vio_video_ctx *vctx)
{
	struct vcon_device_s *vcon = vcon_get_by_vctx(vctx);
	int32_t ictx;

	if (vcon == NULL)
		return -ENODEV;
	if (vctx == NULL)
		return -EINVAL;
	ictx = vctx->ctx_id;

	vcon_deserial_event(vcon_real(vcon), VCON_E_RESET, NULL);
	vcon_sensor_event(vcon_real(vcon), ictx, VCON_E_RESET, NULL);

	return 0;
}

/**
 * vin vcon driver ops for vin_node
 */
struct vin_common_ops vcon_ops = {
	.open = vcon_open,
	.close = vcon_close,
	.video_set_attr = vcon_set_init_attr,
        .video_set_internal_attr = vcon_set_internal_attr,
	.video_get_attr = vcon_get_attr,
	.video_start = vcon_start,
	.video_stop = vcon_stop,
	.video_error_callback = vcon_error_event,
	.video_reset = vcon_reset,
};

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vin vcon event from sub drvier callback
 *
 * @param[in] flow_id: flow id of event
 * @param[in] event_id: event id
 * @param[in] event_data: event data
 *
 * @return >=0:Success, <0:Failure
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
static int32_t vcon_event_cb(int32_t flow_id, int32_t event_id, void *event_data)
{
	int32_t ret = 0;
	struct vcon_device_s *vcon, *vcon_tx;
	struct vcon_ctx_s *ctx;
	struct vcon_grp_s *grp;
	struct vcon_attr_s *info;
	struct os_dev *dev;
	int32_t *attr;
	int32_t l = 0, n, i, j, b;
	char *s;

	switch (event_id) {
	case VCON_EVENT_HW_SHOW:
		s = (char *)event_data;
		if (s == NULL) {
			ret = 0;
			break;
		}
		/* flow_id as rx index here */
		vcon = vcon_get_by_rx(flow_id);
		if (vcon == NULL) {
			l += sprintf(&s[l], "%-15s: %d invalid\n", "vcon", flow_id);
			ret = l;
			break;
		}
		info = &vcon->info;
		attr = &info->bus_main;
		l += sprintf(&s[l], "%-15s: %d\n", "vcon", flow_id);
		n = 0;
		for (i = 0; i < ARRAY_SIZE(g_vcon_attr_v_nums); i++) {
			b = (0x1 << i);
			if ((info->attr_valid & b) == 0) {
				n += g_vcon_attr_v_nums[i];
				continue;
			}
			for (j = 0; j < g_vcon_attr_v_nums[i]; j++) {
				if ((attr[n + j] == VCON_ATTR_INVALID) ||
					(((VCON_ATTR_V_GPIO & b) != 0) && (attr[n + j] == VCON_ATTR_INVGPIO)))
					continue;
				l += sprintf(&s[l], "%-15s: %d\n", g_vcon_attr_names[n + j], attr[n + j]);
			}
			n += g_vcon_attr_v_nums[i];
		}
		/* if tx attached? */
		vcon_tx = vcon_get_by_tx(vcon->tx_index);
		if (vcon_tx != NULL) {
			info = &vcon_tx->info;
			attr = &info->bus_main;
			l += sprintf(&s[l], "%-14s%d: %d\n", "vcon_tx", vcon_tx->tx_index, vcon_tx->index);
			n = 0;
			for (i = 0; i < ARRAY_SIZE(g_vcon_attr_v_nums); i++) {
				b = (0x1 << i);
				if ((info->attr_valid & b) == 0) {
					n += g_vcon_attr_v_nums[i];
					continue;
				}
				for (j = 0; j < g_vcon_attr_v_nums[i]; j++) {
					if ((attr[n + j] == VCON_ATTR_INVALID) ||
							(((VCON_ATTR_V_GPIO & b) != 0) && (attr[n + j] == VCON_ATTR_INVGPIO)))
						continue;
					l += sprintf(&s[l], "%-15s: %d\n", g_vcon_attr_names[n + j], attr[n + j]);
				}
				n += g_vcon_attr_v_nums[i];
			}
		}
		ret = l;
		break;
	case VCON_EVENT_ATTR_SHOW:
		s = (char *)event_data;
		if (s == NULL) {
			ret = 0;
			break;
		}
		vcon = vcon_by_flow(flow_id, &ctx);
		if (vcon == NULL) {
			l += sprintf(&s[l], "%-15s: flow%d not found\n", "vcon", flow_id);
			ret = l;
			break;
		}
		info = (vcon->user.open_cnt > 0) ? &vcon->config : &vcon->info;
		attr = &info->bus_main;
		l += sprintf(&s[l], "%-15s: %d ctx%d flow%d %s\n", "vcon",
				vcon->index, ctx->ctx_id, flow_id, (vcon->user.open_cnt > 0) ? "run" : "hw");
		n = 0;
		for (i = 0; i < ARRAY_SIZE(g_vcon_attr_v_nums); i++) {
			b = (0x1 << i);
			if ((info->attr_valid & b) == 0) {
				n += g_vcon_attr_v_nums[i];
				continue;
			}
			for (j = 0; j < g_vcon_attr_v_nums[i]; j++) {
				if ((attr[n + j] == VCON_ATTR_INVALID) ||
					(((VCON_ATTR_V_GPIO & b) != 0) && (attr[n + j] == VCON_ATTR_INVGPIO)))
					continue;
				l += sprintf(&s[l], "%-15s: %d\n", g_vcon_attr_names[n + j], attr[n + j]);
			}
			n += g_vcon_attr_v_nums[i];
		}
		/* if tx attached? */
		vcon_tx = vcon_get_by_tx(vcon->tx_index);
		if (vcon_tx != NULL) {
			l += sprintf(&s[l], "%-15s: %d\n", "bypass_tx", vcon->tx_index);
		}
		ret = l;
		break;
	case VCON_EVENT_INFO_SHOW:
		s = (char *)event_data;
		if (s == NULL) {
			ret = 0;
			break;
		}
		vcon = vcon_by_flow(flow_id, &ctx);
		if (vcon == NULL) {
			l += sprintf(&s[l], "%-15s: flow%d not attach\n", "vcon", flow_id);
			ret = l;
			break;
		}
		grp = &vcon->grp;
		l += sprintf(&s[l], "%-15s: %d ctx%d\n", "vcon", vcon->index, ctx->ctx_id);
		l += sprintf(&s[l], "%-15s: flow%d\n", "attach", flow_id);
		l += sprintf(&s[l], "%-15s: %d\n", "start", ctx->sensor_start_cnt);
		if (grp->deserial_attached) {
			l += sprintf(&s[l], "%-15s: %d\n", "deserial", grp->deserial_index);
			l += sprintf(&s[l], "%-15s: %d\n", "mask", grp->group_mask);
			l += sprintf(&s[l], "%-15s: %d\n", "link", grp->deserial_link[ctx->ctx_id]);
			l += sprintf(&s[l], "%-15s: %d\n", "start", grp->deserial_start_cnt);
		}
		ret = l;
		break;
	default:
		vcon = vcon_by_flow(flow_id, &ctx);
		if (vcon != NULL) {
			dev = &vcon->osdev;
			vcon_err(dev, "vcon%d: ctx%d flow%d event %d not support\n",
				vcon->index, ctx->ctx_id, flow_id, event_id);
			ret = -EINVAL;
		} else {
			ret = -EACCES;
		}
		break;
	}

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vin vcon event cb set to sub drivers
 *
 * @param[in] set: 0-set NULL, 1-set callback
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
static void vcon_event_setcb(int32_t set)
{
	int32_t t;
	for (t = 0; t < VCON_SUBNUM; t++) {
		if (set == 1) {
			if ((g_vcon.setcb_done[t] == 0) && (!vcon_sub_op_invalid(t, setcb))) {
				vcon_sub_op(t, setcb)(vcon_event_cb);
				g_vcon.setcb_done[t] = 1;
			}
		} else {
			if ((g_vcon.setcb_done[t] == 1) && (!vcon_sub_op_invalid(t, setcb))) {
				vcon_sub_op(t, setcb)(NULL);
				g_vcon.setcb_done[t] = 0;
			}
		}
	}
	return;
}

/**
 * @NO{S10E01C02I}
 * @ASIL{B}
 * @brief vin vcon sub drivers setup by vio common ops api
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
static int32_t vcon_subs_ops_setup(void)
{
	int32_t t;
	enum cops_type vcon_sub_cops_types[VCON_SUBNUM] = VCON_SUB_COPTS;
	struct vcon_sub_ops_s *dummy_ops[VCON_SUBNUM] = { &dummy_deserial_ops, &dummy_sensor_ops };
	struct vio_callback_ops *cops;
	struct vcon_sub_ops_s *ops;

	for (t = 0; t < VCON_SUBNUM; t++) {
		cops = vio_get_callback_ops(dummy_ops[t], VCON_COP_MODULE, vcon_sub_cops_types[t]);
		if ((cops != NULL) && (cops->cops != NULL)) {
			ops = (struct vcon_sub_ops_s *)cops->cops;
			if (!(VCON_SUB_OPS_CHECK_STRICT(ops, vcon_sub_cops_types[t]))) {
				if (!(VCON_SUB_OPS_CHECK(ops, vcon_sub_cops_types[t]))) {
					vcon_err(NULL, "[%s] vcon sub type %d ops not match error\n",
							__func__, t);
					cops = NULL;
				} else {
					vcon_info(NULL, "[%s] vcon sub type %d ops maybe not match\n",
					__func__, t);
				}
			}
		}
		if (cops == NULL) {
			while (t > 0) {
				t --;
				g_vcon.sub_cops[t] = NULL;
			}
			return -EFAULT;
		}
		g_vcon.sub_cops[t] = cops;
	}
	vcon_event_setcb(1);

	return 0;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vin vcont init and a device struct by index from platform
 *
 * @param[in] index: vin vcon index
 * @param[in] pvcon: vin vcon device struct point store addr
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
int32_t vcon_device_init_get(int32_t index, struct vcon_device_s **pvcon)
{
	struct vcon_device_s *vcon;

	vcon = vcon_get_by_id(index);
	if (vcon == NULL) {
		vcon_err(NULL, "[%s] vcon %d get invalid\n", __func__, index);
		return -ENOMEM;
	}
	(void)memset((void *)vcon, 0, sizeof(struct vcon_device_s));
	/* prepare: report */
	vcon->index = index;
	vcon->osdev.devno = index;

	osal_mutex_init(&vcon->user.open_mutex);
	osal_mutex_init(&vcon->user.mutex);

	g_vcon.vcon_num ++;
	g_vcon.vcon_mask |= (0x1 << vcon->index);

	if (pvcon != NULL)
		*pvcon = vcon;

	return 0;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vin vcont exit and a device struct and put free
 *
 * @param[in] vcon: vin vcon device struct point
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void vcon_device_exit_put(struct vcon_device_s *vcon)
{
	if (vcon == NULL)
		return;
	if ((g_vcon.vcon_mask & (0x1 << vcon->index)) == 0)
		return;

	if (vcon->rx_index >= 0)
		g_vcon.vcon_rx[vcon->rx_index] = NULL;
	else if (vcon->tx_index >= 0)
		g_vcon.vcon_tx[vcon->tx_index] = NULL;

	g_vcon.vcon_num --;
	g_vcon.vcon_mask &= ~(0x1 << vcon->index);

	return;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon attr config info parse
 *
 * @param[in] vcon: vcon device struct
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
int32_t vcon_device_attr_parse(struct vcon_device_s *vcon)
{
	struct os_dev *dev = &vcon->osdev;
	struct vcon_attr_s *info = &vcon->info;

	vcon->rx_index = VCON_ATTR_INVALID;
	vcon->tx_index = VCON_ATTR_INVALID;

	if ((info->attr_valid & VCON_ATTR_V_MIPI) != 0) {
		if (info->rx_phy_mode > 0) {
			if (info->rx_phy_index >= VCON_DEV_MAX_NUM) {
				vcon_err(dev, "attr rx_phy_index %d over %d error\n",
					info->rx_phy_index, VCON_DEV_MAX_NUM);
				return -ERANGE;
			}
			vcon->rx_index = info->rx_phy_index;
			g_vcon.vcon_rx[vcon->rx_index] = vcon;
		} else if (info->tx_phy_mode > 0) {
			if (info->tx_phy_index >= VCON_DEV_MAX_NUM) {
				vcon_err(dev, "attr tx_phy_index %d over %d error\n",
					info->tx_phy_index, VCON_DEV_MAX_NUM);
				return -ERANGE;
			}
			vcon->tx_index = info->tx_phy_index;
			g_vcon.vcon_tx[vcon->tx_index] = vcon;
		}
	}

	return 0;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon attr config info show
 *
 * @param[in] vcon: vcon device struct
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void vcon_device_attr_show(struct vcon_device_s *vcon)
{
	struct os_dev *dev = &vcon->osdev;
	struct vcon_attr_s *info = &vcon->info;
#if 0
	int i, j, b, n = 0;
	int32_t *attr = &info->bus_main;

	for (i = 0; i < ARRAY_SIZE(g_vcon_attr_v_nums); i++) {
		b = (0x1 << i);
		if ((info->attr_valid & b) == 0)
			continue;
		for (j = 0; j < g_vcon_attr_v_nums[i]; j++) {
			if ((attr[n + j] == VCON_ATTR_INVALID) ||
				(((VCON_ATTR_V_GPIO & (0x1 << j)) != 0) && (attr[n + j] == VCON_ATTR_INVGPIO)))
				continue;
			vcon_info(dev, "%s = %d\n",
				g_vcon_attr_names[n + j], attr[n + j]);
		}
		n += g_vcon_attr_v_nums[i];
	}
#endif
	if (info->attr_valid == 0)
		return;

	vcon_info(dev, "%s %s%d attr 0x%x %s %d probe\n",
		g_vcon_dev_type_names[info->vcon_type],
		(info->rx_phy_mode > 0) ? "rx" : ((info->tx_phy_mode > 0) ? "tx" : "no"),
		(info->rx_phy_mode > 0) ? info->rx_phy_index : ((info->tx_phy_mode > 0) ? info->tx_phy_index : -1),
		vcon->info.attr_valid,
		(info->vcon_type == VCON_FOLLOW) ? "to" : "bus",
		(info->vcon_type == VCON_FOLLOW) ? info->vcon_link : info->bus_main);
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon driver init done: reg ops
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
int32_t vcon_driver_init_done(void)
{
	int32_t ret;

	ret = vcon_subs_ops_setup();
	if (ret < 0)
		return ret;

	vin_register_device_node(VIN_VCON, &vcon_ops);
	vcon_info(NULL, "init %d vcon done\n", g_vcon.vcon_num);

	return ret;
}

/**
 * @NO{S10E01C02}
 * @ASIL{B}
 * @brief vcon driver all exit done: unreg ops
 *
 * @data_read None
 * @data_updated None
 * @compatibility None
 *
 * @callgraph
 * @callergraph
 * @design
 */
void vcon_driver_exit_done(void)
{
	vin_unregister_device_node(VIN_VCON);
	vcon_event_setcb(0);

	vcon_info(NULL, "exit all vcon done\n");
}

