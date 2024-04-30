/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2023 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include "hobot_vin_common.h"
#include "vin_node_config.h"
#include "hobot_vin_node_ops.h"
#include "hobot_dev_vin_node.h"



/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: vin attr check
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] vin_node_attr: sconfiguration parameters
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_attr_check(struct vio_video_ctx *vctx, vin_node_attr_t *vin_node_attr)
{
	uint32_t num;
	cim_attr_t *cim_attr = &vin_node_attr->cim_attr;
	vcon_attr_t *vcon_attr = &vin_node_attr->vcon_attr;
	cim_func_desc_t *func = &cim_attr->func;

	/* vcon attr check */
	if (vcon_attr->bus_main > MAX_VCON_BUS || vcon_attr->bus_main < MIN_VCON_BUS ||
			vcon_attr->bus_second > MAX_VCON_BUS || vcon_attr->bus_second < MIN_VCON_BUS) {
		vio_err("[S%d] unsupport i2c buf main(%d) second(%d)\n", vctx->ctx_id,
				vcon_attr->bus_main, vcon_attr->bus_second);
		return -EINVAL;
	}

	for (num = 0; num < VGPIO_NUM; num++) {
		if (vcon_attr->gpios[num] < MIN_VCON_GPIO || vcon_attr->gpios[num] > MAX_VCON_GPIO) {
			vio_err("[S%d] unsupport gpios[%d]=%d\n",
					vctx->ctx_id, num, vcon_attr->gpios[num]);
			return -EINVAL;
		}
	}

	for (num = 0; num < SENSOR_ERR_PIN_NUM; num++) {
		if (vcon_attr->sensor_err[num] < MIN_VCON_SENSOR_ERR ||
				vcon_attr->sensor_err[num] > MAX_VCON_SENSOR_ERR) {
			vio_err("[S%d] unsupport sensor_err[%d]=%d\n",
					vctx->ctx_id, num, vcon_attr->sensor_err[num]);
			return -EINVAL;
		}
	}

	for (num = 0; num < LPWM_CHN_NUM; num++) {
		if (vcon_attr->lpwm_chn[num] < MIN_LPWM_CHN || vcon_attr->lpwm_chn[num] > MAX_LPWM_CHN) {
			vio_err("[S%d] unsupport lpwm_chn[%d]=%d\n",
					vctx->ctx_id, num, vcon_attr->lpwm_chn[num]);
			return -EINVAL;
		}
	}

	if (vcon_attr->rx_phy_mode < MIN_RX_PHY_MODE || vcon_attr->rx_phy_mode > MAX_RX_PHY_MODE ||
			vcon_attr->rx_phy_index < MIN_RX_PHY_INDEX || vcon_attr->rx_phy_index > MAX_RX_PHY_INDEX ||
			vcon_attr->rx_phy_link < MIN_RX_PHY_LINK || vcon_attr->rx_phy_link > MAX_RX_PHY_LINK) {
		vio_err("[S%d] unsupport attr rx_phy_mode(%d) rx_phy_index(%d) rx_phy_link(%d)\n",
				vctx->ctx_id, vcon_attr->rx_phy_mode, vcon_attr->rx_phy_index,
				vcon_attr->rx_phy_link);
		return -EINVAL;
	}

	if (vcon_attr->tx_phy_mode < MIN_TX_PHY_MODE || vcon_attr->tx_phy_mode > MAX_TX_PHY_MODE ||
			vcon_attr->tx_phy_index < MIN_TX_PHY_INDEX || vcon_attr->tx_phy_index > MAX_TX_PHY_INDEX ||
			vcon_attr->tx_phy_link < MIN_TX_PHY_INDEX || vcon_attr->tx_phy_index > MAX_TX_PHY_INDEX) {
		vio_err("[S%d] unsupport attr tx_phy_mode(%d) tx_phy_index(%d) tx_phy_link(%d)\n",
				vctx->ctx_id, vcon_attr->tx_phy_mode, vcon_attr->tx_phy_index,
				vcon_attr->tx_phy_link);
		return -EINVAL;
	}

	if (vcon_attr->vcon_type < MIN_VCON_TYPE || vcon_attr->vcon_type > MAX_VCON_TYPE ||
			vcon_attr->vcon_link < MIN_VCON_LINK || vcon_attr->vcon_link > MAX_VCON_LINK) {
		vio_err("[S%d] unsupport attr vcon_type(%d) vcon_link(%d)\n", vctx->ctx_id,
				vcon_attr->vcon_type, vcon_attr->vcon_link);
		return -EINVAL;
	}

	/* cim attr check */
	if (cim_attr->mipi_rx > MAX_MIPI_RX_NUM || cim_attr->vc_index > MAX_VC_INDEX ||
			cim_attr->ipi_channel > MAX_IPI_CHANNELS) {
		vio_err("[S%d] unsupport mipi_rx(%d) vc_index(%d) ipi_channel(%d)\n", vctx->ctx_id,
				cim_attr->mipi_rx, cim_attr->vc_index, cim_attr->ipi_channel);
		return -EINVAL;
	}

	if (cim_attr->cim_isp_flyby + cim_attr->cim_pym_flyby > 1) {
		vio_err("[S%d] cim otf attr error isp_flyby(%d) pym_flyby(%d)\n", vctx->ctx_id,
				cim_attr->cim_isp_flyby, cim_attr->cim_pym_flyby);
		return -EINVAL;
	}

	if (func->skip_frame == CIM_HW_SKIP) {
		if ((func->hw_extract_m != 1 && func->hw_extract_n != 1) ||
				func->hw_extract_m > MAX_HW_EXTRACT_FRAME ||
				func->hw_extract_n > MAX_HW_EXTRACT_FRAME) {
			vio_err("[S%d] hw frame extract unsupport m%d n%d\n", vctx->ctx_id,
					func->hw_extract_m, func->hw_extract_n);
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: ichn attr check
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *ichn_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_ichn_attr_check(struct vio_video_ctx *vctx, vin_ichn_attr_t *ichn_attr)
{
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	cim_attr_t *cim_attr;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);/*PRQA S 2810,0497*/
	cim_attr = &subdev->vin_attr.vin_node_attr.cim_attr;

	if (ichn_attr->format < HW_FORMAT_RAW8 && ichn_attr->format > HW_FORMAT_RAW20 && ichn_attr->format != HW_FORMAT_RAW24
			&& ichn_attr->format != HW_FORMAT_YUV422_8BIT && ichn_attr->format != HW_FORMAT_YUV422_10BIT) {
		vio_err("[S%d] unsupport format(0x%x)\n", vctx->ctx_id, ichn_attr->format);
		return -EINVAL;
	}

	if (ichn_attr->width > MAX_CIM_WIDTH || ichn_attr->height > MAX_CIM_HEIGHT) { // TODO
		vio_err("[S%d] unsupport input size w%d h%d\n", vctx->ctx_id,
				ichn_attr->width, ichn_attr->height);
		return -EINVAL;
	}

	return 0;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: ochn attr check
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *ochn_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_ochn_attr_check(struct vio_video_ctx *vctx, vin_ochn_attr_t *ochn_attr)
{
	u32 src_width, src_height, out_width, out_height;
	u32 ochn_id = vctx->id - VNODE_ID_CAP, wstride, vstride;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_node_subdev *subdev, *src_subdev;
	struct vio_subdev *vdev;
	struct vin_roi_attr_s *roi_attr;
	struct vin_emb_attr_s *emb_attr;
	struct vin_basic_attr_s *base_attr;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);
	vin_node_dev = subdev->vin_node_dev;
	src_subdev = &vin_node_dev->src_subdev[subdev->ctx_id];

	src_width = src_subdev->vin_attr.vin_ichn_attr.width;
	src_height = src_subdev->vin_attr.vin_ichn_attr.height;
	roi_attr = &ochn_attr->roi_attr;
	emb_attr = &ochn_attr->emb_attr;
	base_attr = &ochn_attr->vin_basic_attr;

	if (base_attr->format < HW_FORMAT_RAW8 && base_attr->format > HW_FORMAT_RAW20 && base_attr->format != HW_FORMAT_RAW24
			&& base_attr->format != HW_FORMAT_YUV422_8BIT && base_attr->format != HW_FORMAT_YUV422_10BIT) {
		vio_err("[S%d] unsupport format(0x%x)\n", vctx->ctx_id, base_attr->format);
		return -EINVAL;
	}

	if (ochn_attr->roi_en && (roi_attr->roi_x + roi_attr->roi_width > src_width ||
			roi_attr->roi_y + roi_attr->roi_height > src_height)) {
		vio_err("[S%d] ochn%d unsupport roi attr x(%d) y(%d) w(%d) h(%d)\n",
				vctx->ctx_id, ochn_id, roi_attr->roi_x, roi_attr->roi_y,
				roi_attr->roi_width, roi_attr->roi_height);
		return -EINVAL;
	}

	if (ochn_attr->emb_en) {
		if (emb_attr->embeded_height > src_height) {
			vio_err("[S%d] ochn%d emb size error w(%d) h(%d)\n", vctx->ctx_id, ochn_id,
					emb_attr->embeded_width, emb_attr->embeded_height);
			return -EINVAL;
		}
		if (ochn_attr->roi_en && (emb_attr->embeded_height != roi_attr->roi_height ||
					emb_attr->embeded_width != roi_attr->roi_width)) {
			vio_err("[S%d] ochn%d emb size error w(%d) h(%d) roi_w(%d) roi_h(%d)\n",
					vctx->ctx_id, ochn_id,
					emb_attr->embeded_width, emb_attr->embeded_height,
					roi_attr->roi_width, roi_attr->roi_height);
			return -EINVAL;
		}
	}

	/* stride check */
	if (ochn_attr->roi_en && ochn_attr->rawds_en) {
		out_width = roi_attr->roi_width / 2;
		out_height = roi_attr->roi_height / 2;
	} else if (ochn_attr->roi_en) {
		out_width = roi_attr->roi_width;
		out_height = roi_attr->roi_height;
	} else if (ochn_attr->rawds_en) {
		out_width = src_width / 2;
		out_height = src_height / 2;
	} else if (ochn_attr->emb_en) {
		out_width = emb_attr->embeded_width;
		out_height = emb_attr->embeded_height;
	} else {
		out_width = src_width;
		out_height = src_height;
	}
	wstride = vin_get_perline_size(out_width, base_attr->pack_mode, base_attr->format);
	wstride = ALIGN(wstride, CIM_STRIDE_ALIGN);
	vstride = out_height;

	if (base_attr->wstride == 0) {
		base_attr->wstride = wstride;
		vio_info("[S%d] ochn%d auto calc wstride %d\n", vctx->ctx_id, ochn_id, wstride);
	} else if (base_attr->wstride < wstride && (!ochn_attr->emb_en)) {
		vio_err("[S%d] ochn%d unsupport wstride%d\n", vctx->ctx_id, ochn_id,
				base_attr->wstride);
		return -EINVAL;
	}

	if (base_attr->vstride == 0) {
		base_attr->vstride = vstride;
		vio_info("[S%d] ochn%d auto calc vstride %d\n", vctx->ctx_id, ochn_id, vstride);
	} else if (base_attr->vstride < vstride) {
		vio_err("[S%d] ochn%d unsupport vstride%d\n", vctx->ctx_id, ochn_id,
					base_attr->vstride);
		return -EINVAL;
	}

	return 0;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Get vin_node attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] vin_node_attr: sconfiguration parameters
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_get_attr(struct vio_video_ctx *vctx, vin_node_attr_t *vin_node_attr)
{
	s32 ret = 0, lpwm_chn;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_common_ops *vin_ops;
	u8 i;

	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;

	vin_ops = vin_node_dev->vin_ops[VIN_CIM];
	if (vin_ops && vin_ops->video_get_attr) {
		ret = vin_ops->video_get_attr(vctx, &vin_node_attr->cim_attr);
		if (ret < 0) {
			vio_err("[S%d]%s CIM fail\n", vctx->ctx_id, __func__);
			return ret;
		}
	}

	vin_ops = vin_node_dev->vin_ops[VIN_VCON];
	if (vin_ops && vin_ops->video_get_attr) {
		ret = vin_ops->video_get_attr(vctx, &vin_node_attr->vcon_attr);
		if (ret < 0) {
			vio_err("[S%d]%s VCON fail\n", vctx->ctx_id, __func__);
			return ret;
		}
	}

	vin_ops = vin_node_dev->vin_ops[VIN_LPWM];
	if (vin_ops && vin_ops->video_get_attr && vin_node_attr->lpwm_attr.enable) {
		for (i = 0u; i < LPWM_CHN_NUM; i++) {
			lpwm_chn = vin_node_attr->vcon_attr.lpwm_chn[i];
			if (lpwm_chn == -1)
				continue;

			vctx->file = (void*)&lpwm_chn;
			ret = vin_ops->video_get_attr(vctx, &vin_node_attr->lpwm_attr);
			if (ret < 0) {
				vctx->file = NULL;
				vio_err("[S%d]%s LPWM fail\n", vctx->ctx_id, __func__);
				return ret;
			}
			vctx->file = NULL;
		}
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Configuration function of vin_node
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *vin_node_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_set_attr(struct vio_video_ctx *vctx, vin_node_attr_t *vin_node_attr)
{
	s32 ret, lpwm_chn;
	u32 i;
	struct j6_vin_node_dev *vin_node_dev;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct vin_common_ops *vin_ops;
	vcon_attr_t *vcon_attr;
	cim_attr_t *cim_attr;
	lpwm_attr_t *lpwm_attr;
	vcon_attr_t got_vcon_attr = { 0 };

	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	vdev = vctx->vdev;
	vnode = vdev->vnode;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);/*PRQA S 2810,0497*/
	memcpy(&subdev->vin_attr.vin_node_attr,
			(char *)vin_node_attr, sizeof(vin_node_attr_t));
	vcon_attr = &vin_node_attr->vcon_attr;
	cim_attr = &vin_node_attr->cim_attr;
	lpwm_attr = &vin_node_attr->lpwm_attr;

	ret = vin_node_attr_check(vctx, vin_node_attr);
	if (ret != 0)
		return ret;

	vin_ops = vin_node_dev->vin_ops[VIN_VCON];
	if (vctx->id == VNODE_ID_SRC && vin_ops && vin_ops->open && vin_ops->video_set_attr) {
		ret = vin_ops->open(vctx);
		if (ret < 0) {
			vio_err("[S%d]%s VCON open fail\n", vctx->ctx_id, __func__);
			goto err;
		}
		osal_set_bit(VCON_OPEN, &subdev->state);

		if (vcon_attr->attr_valid)
			ret = vin_ops->video_set_attr(vctx, vcon_attr);
		else
			ret = vin_ops->video_set_attr(vctx, NULL);

		if (ret < 0) {
			vio_err("[S%d]%s VCON set attr fail\n", vctx->ctx_id, __func__);
			goto vcon_init_err;
		}
	}

	vin_ops = vin_node_dev->vin_ops[VIN_CIM];
	if (vin_ops && vin_ops->video_set_attr) {
		ret = vin_ops->video_set_attr(vctx, cim_attr);
		if (ret < 0) {
			vio_err("[S%d]%s CIM fail\n", vctx->ctx_id, __func__);
			goto vcon_init_err;
		}
		osal_set_bit(CIM_INIT, &subdev->state);
	}

	vin_ops = vin_node_dev->vin_ops[VIN_VCON];
	if (vctx->id == VNODE_ID_SRC && vin_ops && vin_ops->video_get_attr) {
		ret = vin_ops->video_get_attr(vctx, &got_vcon_attr);
		if (ret < 0) {
			vio_err("[S%d]%s VCON get attr fail\n", vctx->ctx_id, __func__);
			goto vcon_init_err;
		}
		memcpy(vcon_attr, &got_vcon_attr, sizeof(struct vcon_attr_s));
		vin_ops = vin_node_dev->vin_ops[VIN_LPWM];

		for (i = 0; i < LPWM_CHN_NUM; i++) {
			lpwm_chn = got_vcon_attr.lpwm_chn[i];
			if (lpwm_chn == -1)
				continue;
			if (!vin_ops->open || !vin_ops->video_set_attr)
				break;
			if (!lpwm_attr->enable)
				break;

			vctx->file = (void*)&lpwm_chn;
			ret = vin_ops->open(vctx);
			if (ret < 0) {
				vio_err("[S%d]%s LPWM open fail\n", vctx->ctx_id, __func__);
				goto vcon_init_err;
			}
			osal_set_bit(LPWM_CHN0_OPEN + i, &subdev->state);

			ret = vin_ops->video_set_attr(vctx, lpwm_attr);
			if (ret < 0) {
				vio_err("[S%d]%s LPWM set attr fail\n", vctx->ctx_id, __func__);
				goto lpwm_init_err;
			}
			vctx->file = NULL;
			osal_set_bit(LPWM_CHN0_INIT + i, &subdev->state);
			osal_set_bit(LPWM_INIT, &subdev->state);
		}
	}

	vio_info("[C%d] %s ipi %d done\n", vctx->ctx_id, __func__, vin_node_attr->cim_attr.vc_index);
	return ret;

lpwm_init_err:
	vin_ops = vin_node_dev->vin_ops[VIN_LPWM];
	if (vctx->id == VNODE_ID_SRC && vin_ops && vin_ops->close) {
		for (i = 0; i < LPWM_CHN_NUM; i++) {
			lpwm_chn = got_vcon_attr.lpwm_chn[i];
			if (lpwm_chn == -1)
				continue;
			if (osal_test_and_clear_bit(LPWM_CHN0_OPEN + i, &subdev->state)) {
				vctx->file = (void*)&lpwm_chn;
				vin_ops->close(vctx);
			}
		}
	}
	vctx->file = NULL;

vcon_init_err:
	vin_ops = vin_node_dev->vin_ops[VIN_VCON];
	if (vctx->id == VNODE_ID_SRC && vin_ops && vin_ops->close) {
		vin_ops->close(vctx);
		osal_clear_bit(VCON_OPEN, &subdev->state);
	}
err:
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Configuration function of vin_node extend attr
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *vin_attr_ex: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_set_attr_ex(struct vio_video_ctx *vctx, vin_attr_ex_t *vin_attr_ex)
{
	s32 ret, i;
	struct j6_vin_node_dev *vin_node_dev;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	vin_attr_ex_type_e attr_type;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	attr_type = vin_attr_ex->ex_attr_type;

	for (i = VIN_CIM; i < VIN_MAX_DEVICES; i++) {
		if (i == VIN_VCON)
			continue;
		if ((NULL == (vin_node_dev->vin_ops[i])) &&
				(NULL == (vin_node_dev->vin_ops[i])->video_set_attr_ex))
			return -1;
	}
	switch (attr_type) {
		case VIN_STATIC_CIM_ATTR:
		case VIN_DYNAMIC_FPS_CTRL:
			ret = (vin_node_dev->vin_ops[VIN_CIM])->video_set_attr_ex(vctx, vin_attr_ex);
			if (ret < 0) {
				vio_err("[S%d] %s cim_set_attr_ex fail\n", vnode->flow_id, __func__);
			}
			break;
		case VIN_STATIC_MIPI_ATTR:
		case VIN_DYNAMIC_IPI_RESET:
		case VIN_DYNAMIC_BYPASS_ENABLE:
                case VIN_STATIC_MCLK_ATTR:
			ret = (vin_node_dev->vin_ops[VIN_MIPI])->video_set_attr_ex(vctx, vin_attr_ex);
			if (ret < 0) {
				vio_err("[S%d] %s mipi_set_attr_ex fail\n", vnode->flow_id, __func__);
			}
			break;
		case VIN_DYNAMIC_LPWM_FPS:
		case VIN_DYNAMIC_LPWM_TRIGGER_SOURCE:
			ret = (vin_node_dev->vin_ops[VIN_LPWM])->video_set_attr_ex(vctx, vin_attr_ex);
			if (ret < 0) {
				vio_err("[S%d] %s lpwm_change_fps_attr_ex fail\n", vnode->flow_id, __func__);
			}
			break;
		default:
			vio_err("[S%d] %s invalid attr_ex type\n", vnode->flow_id, __func__);
			ret = -EINVAL;
			break;
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set vin_node internal_attr
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *mipi_attr: mipi configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_set_internal_attr(struct vio_video_ctx *vctx, void *attr)
{
	s32 ret = 0;
	struct j6_vin_node_dev *vin_node_dev;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct vin_common_ops *vin_ops;
	vin_inter_attr_t *inter_attr = (vin_inter_attr_t *)attr;
        mipi_attr_t *mipi_inter_attr = &inter_attr->mipi_inter_attr;
        vcon_inter_attr_t *vcon_inter_attr = &inter_attr->vcon_inter_attr;
	s32 vcon_attach = 0;

	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	vdev = vctx->vdev;
	vnode = vdev->vnode;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);

	vin_ops = vin_node_dev->vin_ops[VIN_VCON];
	if ((vin_ops) && (vin_ops->video_set_internal_attr) && (vcon_inter_attr->attr_valid)) {
		ret = vin_ops->video_set_internal_attr(vctx, vcon_inter_attr);
		if ((ret < 0) && (vcon_inter_attr->attach != 0)) {
			vio_err("[S%d] %s set iner attr fail\n", vnode->flow_id, __func__);
			return ret;
		}
		vcon_attach = vcon_inter_attr->attach;
		osal_set_bit(VCON_INIT, &subdev->state);
	}

	vin_ops = vin_node_dev->vin_ops[VIN_MIPI];
	if ((vin_ops) && (vin_ops->open) && (vin_ops->close) && (vin_ops->video_set_internal_attr)
	    && (mipi_inter_attr->attr_valid)) {
		if (mipi_inter_attr->attach) {
			ret = vin_ops->open(vctx);
			if (ret < 0) {
				vio_err("[S%d] %s mipi open fail\n", vnode->flow_id, __func__);
				goto internal_error_vcon;
			}
			osal_set_bit(MIPI_OPEN, &subdev->state);
			ret = vin_ops->video_set_internal_attr(vctx, &inter_attr->mipi_inter_attr);
			if (ret < 0) {
				vio_err("[S%d] %s mipi_set_attr fail\n", vnode->flow_id, __func__);
				goto internal_error_mipi;
			}
		} else {
			vin_ops->video_set_internal_attr(vctx, &inter_attr->mipi_inter_attr);
			if (osal_test_and_clear_bit(MIPI_OPEN, &subdev->state))
				vin_ops->close(vctx);
		}
		osal_set_bit(MIPI_INIT, &subdev->state);
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;

internal_error_mipi:
	vin_ops = vin_node_dev->vin_ops[VIN_MIPI];
	vin_ops->close(vctx);
internal_error_vcon:
	if (vcon_attach) {
		vcon_inter_attr->attach = 0;
		vin_ops = vin_node_dev->vin_ops[VIN_VCON];
		vin_ops->video_set_internal_attr(vctx, vcon_inter_attr);
	}

	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set vin_node output channel attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *ochn_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_set_ochn_attr(struct vio_video_ctx *vctx, vin_ochn_attr_t *ochn_attr)
{
	s32 ret;
	u32 ochn_id;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_common_ops *vin_ops;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	subdev = container_of(vdev, struct vin_node_subdev, vdev); /*PRQA S 2810,0497*/
	ochn_id = vctx->id - VNODE_ID_CAP;

	ret = vin_node_ochn_attr_check(vctx, ochn_attr);
	if (ret != 0)
		return ret;

	memcpy(&subdev->vin_attr.vin_ochn_attr[ochn_id],
			ochn_attr, sizeof(vin_ochn_attr_t));

	vin_ops = vin_node_dev->vin_ops[VIN_CIM];
	if (vin_ops && vin_ops->video_set_ochn_attr) {
		ret = vin_ops->video_set_ochn_attr(vctx, ochn_attr);
		if (ret < 0) {
			vio_err("[C%d] %s cim_set_ochn_basic_attr fail",
					vnode->ctx_id, __func__);
			return ret;
		}
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set vin_node input channel attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *ichn_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_set_ichn_attr(struct vio_video_ctx *vctx, vin_ichn_attr_t *ichn_attr)
{
	s32 ret;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_common_ops *vin_ops;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);/*PRQA S 2810,0497*/

	ret = vin_node_ichn_attr_check(vctx, ichn_attr);
	if (ret != 0)
		return ret;

	memcpy(&subdev->vin_attr.vin_ichn_attr, ichn_attr, sizeof(vin_ichn_attr_t));
	vio_info("[S%d] %s begin format %d width %d \n",
			vnode->flow_id, __func__,
			subdev->vin_attr.vin_ichn_attr.format,
			subdev->vin_attr.vin_ichn_attr.width);

	vin_ops = vin_node_dev->vin_ops[VIN_CIM];
	if (vin_ops && vin_ops->video_set_ichn_attr) {
		ret = vin_ops->video_set_ichn_attr(vctx, ichn_attr);
		if (ret < 0) {
			vio_err("[C%d] %s cim_set_ichn_basic_attr fail",
					vnode->ctx_id, __func__);
			return ret;
		}
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief:  Set vin_node output channel buffer attributes
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] *ochn_buff_attr: User configuration parameters
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_set_ochn_buff_attr(struct vio_video_ctx *vctx,
		vin_ochn_buff_attr_t *ochn_buff_attr)
{
	s32 ret = 0;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct j6_vin_node_dev *vin_node_dev;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

static void vin_node_get_plane(u32 format, u32 pack_mode, u32 hdr_mode, struct vbuf_group_info *group_attr)
{
	if ((format == HW_FORMAT_RAW12) || (format == HW_FORMAT_RAW10)
			|| (format == HW_FORMAT_RAW8) || (format == HW_FORMAT_RAW14)
			|| (format == HW_FORMAT_RAW16) || (format == HW_FORMAT_RAW20)) {
		if (hdr_mode == DOL_2) {
			group_attr->info[0].buf_attr.planecount = 2;
			group_attr->info[0].buf_attr.format = MEM_PIX_FMT_RAW12;
		} else {
			group_attr->info[0].buf_attr.planecount = 1;
			group_attr->info[0].buf_attr.format = vio_hw_format_cov_hbmem_format(format);
		}
	} else if ((format == HW_FORMAT_YUV422_10BIT) ||(format == HW_FORMAT_YUV422_8BIT)) {
		if (pack_mode == DOUBLE_PLANE) {
			group_attr->info[0].buf_attr.planecount = 2;
			group_attr->info[0].buf_attr.format = MEM_PIX_FMT_NV16;
		} else if (pack_mode == SINGLE_PLANE) {
			group_attr->info[0].buf_attr.planecount = 1;
			group_attr->info[0].buf_attr.format = MEM_PIX_FMT_YUYV422;
		}
	} else if ((format == HW_FORMAT_YUV420_SHIFT_8BIT) || (format == HW_FORMAT_YUV420_SHIFT_10BIT) ||
		(format == HW_FORMAT_YUV420_LEG_8BIT) || (format == HW_FORMAT_YUV420_8BIT) ||
		(format == HW_FORMAT_YUV420_10BIT)) {
		group_attr->info[0].buf_attr.planecount = 2;
		group_attr->info[0].buf_attr.format = vio_hw_format_cov_hbmem_format(format);
	} else {
		vio_err("error format %d\n", format);
	}
}

s32 vin_node_bind_check(struct vio_subdev *vdev, struct vio_subdev *remote_vdev, u8 online)
{
	struct vin_node_subdev *subdev;
	s32 ret = 0;
	vin_attr_t *vin_attr;
	cim_attr_t *cim_attr;
	vin_ochn_attr_t *vin_ochn_attr;
	u32 id;

	subdev = container_of(vdev, struct vin_node_subdev, vdev);/*PRQA S 2810,0497*/
	vin_attr = &subdev->vin_attr;
	cim_attr = &vin_attr->vin_node_attr.cim_attr;
	id = vdev->vctx[0]->id;

	if (online) {
		if (id != VNODE_ID_CAP || (cim_attr->cim_isp_flyby | cim_attr->cim_pym_flyby) == 0) {
			vio_err("[V%d] unsuport otf bind\n", id);
			ret = -1;
		}
	} else {
		vin_ochn_attr = &vin_attr->vin_ochn_attr[id - VNODE_ID_CAP];
		if (id == VNODE_ID_CAP && vin_ochn_attr->ddr_en == 0) {
			vio_err("[V%d] offline bind but ddr not enable\n", id);
			ret = -1;
		} else if (id == VNODE_ID_CAP + VIN_EMB && vin_ochn_attr->emb_en == 0) {
			vio_err("[V%d] emb offline bind but emb not enable\n", id);
			ret = -1;
		} else if (id == VNODE_ID_CAP + VIN_ROI && vin_ochn_attr->roi_en == 0) {
			vio_err("[V%d] roi offline bind but roi not enable\n", id);
			ret = -1;
		}
	}

	return ret;
}

void vin_node_set_ochn_bind_param(struct vio_video_ctx *vctx)
{
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_node_subdev *src_subdev;
	struct vin_node_subdev *emb_subdev;
	struct vin_node_subdev *roi_subdev;
	vin_attr_t *vin_attr;
	vin_ichn_attr_t *vin_ichn_attr;
	struct chn_attr *chn_attr;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);/*PRQA S 2810,0497*/
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	chn_attr = &vdev->chn_attr;

	if (vctx->id == VNODE_ID_CAP) {
		vin_attr = &subdev->vin_attr;
		src_subdev = &vin_node_dev->src_subdev[vctx->ctx_id];
		vin_ichn_attr = &src_subdev->vin_attr.vin_ichn_attr;
		chn_attr->format = vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.format;
		chn_attr->slot_id = vin_attr->vin_node_attr.cim_attr.vc_index;
		if (vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_en == 1 &&
				vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].rawds_en == 1) {
			chn_attr->width = vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_attr.roi_width / 2;
			chn_attr->height = vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_attr.roi_height / 2;
		} else if (vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_en == 1) {
			chn_attr->width = vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_attr.roi_width;
			chn_attr->height = vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_attr.roi_height;
		} else if (vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].rawds_en == 1) {
			chn_attr->width = vin_ichn_attr->width / 2;
			chn_attr->height = vin_ichn_attr->height / 2;
		} else {
			chn_attr->width = vin_ichn_attr->width;
			chn_attr->height = vin_ichn_attr->height;
		}
		chn_attr->wstride = vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.wstride;
		chn_attr->vstride = vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.vstride;
	}

	if (vctx->id == VNODE_ID_CAP + VIN_EMB) {  //EMB
		emb_subdev = &vin_node_dev->emb_subdev[vctx->ctx_id];
		vin_attr = &emb_subdev->vin_attr;
		chn_attr->format = vin_attr->vin_ochn_attr[VIN_EMB].vin_basic_attr.format;
		chn_attr->slot_id = vin_attr->vin_node_attr.cim_attr.vc_index;
		if (vin_attr->vin_ochn_attr[VIN_EMB].emb_en) {
			chn_attr->width = vin_attr->vin_ochn_attr[VIN_EMB].emb_attr.embeded_width;
			chn_attr->height = vin_attr->vin_ochn_attr[VIN_EMB].emb_attr.embeded_height;
			chn_attr->wstride = vin_attr->vin_ochn_attr[VIN_EMB].vin_basic_attr.wstride;
			chn_attr->vstride = vin_attr->vin_ochn_attr[VIN_EMB].vin_basic_attr.vstride;
		}
	}

	if (vctx->id == VNODE_ID_CAP + VIN_ROI) {  //ROI
		roi_subdev = &vin_node_dev->roi_subdev[vctx->ctx_id];
		vin_attr = &roi_subdev->vin_attr;
		chn_attr->format = vin_attr->vin_ochn_attr[VIN_ROI].vin_basic_attr.format;
		chn_attr->slot_id = vin_attr->vin_node_attr.cim_attr.vc_index;
		if (vin_attr->vin_ochn_attr[VIN_ROI].roi_en) {
			chn_attr->width = vin_attr->vin_ochn_attr[VIN_ROI].roi_attr.roi_width;
			chn_attr->height = vin_attr->vin_ochn_attr[VIN_ROI].roi_attr.roi_height;
			chn_attr->wstride = vin_attr->vin_ochn_attr[VIN_ROI].vin_basic_attr.wstride;
			chn_attr->vstride = vin_attr->vin_ochn_attr[VIN_ROI].vin_basic_attr.vstride;
		}
	}

	chn_attr->format = vio_hw_format_cov_hbmem_format(chn_attr->format);
	vio_info("[S%d][V%d] vin_node chn_attr format(%d) slod_id(%d) width(%d) height(%d) wstride(%d) vstride(%d)\n",
			vctx->ctx_id, vctx->id, chn_attr->format, chn_attr->slot_id,
			chn_attr->width, chn_attr->height, chn_attr->wstride, chn_attr->vstride);
}
/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Obtain the attributes of the buffer request
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] group_attr: buff attribute
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_reqbufs(struct vio_video_ctx *vctx,
		struct vbuf_group_info *group_attr)
{
	s32 ret = 0;
	u32 format, pack_mode, hdr_mode;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_node_subdev *src_subdev;
	struct vin_node_subdev *emb_subdev;
	struct vin_node_subdev *roi_subdev;
	vin_attr_t *vin_attr;
	vin_ichn_attr_t *vin_ichn_attr;

	vdev = vctx->vdev;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);/*PRQA S 2810,0497*/
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	if (vctx->id == VNODE_ID_SRC) {
		group_attr->bit_map = 1;
		group_attr->is_contig = 1;
		vin_attr = &subdev->vin_attr;
		src_subdev = &vin_node_dev->src_subdev[vctx->ctx_id];
		format = vin_attr->vin_ichn_attr.format;
		vin_ichn_attr = &src_subdev->vin_attr.vin_ichn_attr;
		group_attr->info[0].buf_attr.width = vin_ichn_attr->width;
		group_attr->info[0].buf_attr.height = vin_ichn_attr->height;
		group_attr->info[0].buf_attr.wstride =
			vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.wstride;
		group_attr->info[0].buf_attr.vstride =
			vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.vstride;
		pack_mode = vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.pack_mode;
		hdr_mode = vin_attr->vin_node_attr.cim_attr.func.hdr_mode;
		vin_node_get_plane(format, pack_mode, hdr_mode, group_attr);
		group_attr->is_alloc = 0;
	}

	if (vctx->id == VNODE_ID_CAP) {
		vin_attr = &subdev->vin_attr;
		src_subdev = &vin_node_dev->src_subdev[vctx->ctx_id];
		vin_ichn_attr = &src_subdev->vin_attr.vin_ichn_attr;
		format = vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.format;
		pack_mode = vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.pack_mode;
		hdr_mode = vin_attr->vin_node_attr.cim_attr.func.hdr_mode;
		if (vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].ddr_en) {
			group_attr->bit_map |= 1;
			group_attr->is_contig = 1;
			if (vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_en == 1 &&
					vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].rawds_en == 1) {
				group_attr->info[0].buf_attr.width =
					vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_attr.roi_width / 2;
				group_attr->info[0].buf_attr.height =
					vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_attr.roi_height / 2;
			} else if (vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_en == 1) {
				group_attr->info[0].buf_attr.width =
					vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_attr.roi_width;
				group_attr->info[0].buf_attr.height =
					vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].roi_attr.roi_height;
			} else if (vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].rawds_en == 1) {
				group_attr->info[0].buf_attr.width = vin_ichn_attr->width / 2;
				group_attr->info[0].buf_attr.height = vin_ichn_attr->height / 2;
			} else {
				group_attr->info[0].buf_attr.width = vin_ichn_attr->width;
				group_attr->info[0].buf_attr.height = vin_ichn_attr->height;
			}
			group_attr->info[0].buf_attr.wstride =
				vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.wstride;
			group_attr->info[0].buf_attr.vstride =
				vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.vstride;
			vio_info("VNODE_ID_CAP wstride %d\n",
					vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.wstride);
			vio_info("VNODE_ID_CAP vstride %d\n",
					vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.vstride);
		}
		vin_node_get_plane(format, pack_mode, hdr_mode, group_attr);
		vio_info("vstride %d  planecount %d \n",
				vin_attr->vin_ochn_attr[VIN_MAIN_FRAME].vin_basic_attr.vstride,
				group_attr->info[0].buf_attr.planecount);
	}

	if (vctx->id == VNODE_ID_CAP + VIN_EMB) {    //EMB
		emb_subdev = &vin_node_dev->emb_subdev[vctx->ctx_id];
		vin_attr = &emb_subdev->vin_attr;
		format = vin_attr->vin_ochn_attr[VIN_EMB].vin_basic_attr.format;
		pack_mode = vin_attr->vin_ochn_attr[VIN_EMB].vin_basic_attr.pack_mode;
		hdr_mode = vin_attr->vin_node_attr.cim_attr.func.hdr_mode;
		if (vin_attr->vin_ochn_attr[VIN_EMB].emb_en) {
			group_attr->bit_map |= 1;
			group_attr->is_contig = 1;
			group_attr->info[0].buf_attr.width = vin_attr->vin_ochn_attr[VIN_EMB].emb_attr.embeded_width;
			group_attr->info[0].buf_attr.height = vin_attr->vin_ochn_attr[VIN_EMB].emb_attr.embeded_height;
			group_attr->info[0].buf_attr.wstride = vin_attr->vin_ochn_attr[VIN_EMB].vin_basic_attr.wstride;
			group_attr->info[0].buf_attr.vstride = vin_attr->vin_ochn_attr[VIN_EMB].vin_basic_attr.vstride;
			vin_node_get_plane(format, pack_mode, hdr_mode, group_attr);
		}
		vio_info("EMB wstride %d\n", vin_attr->vin_ochn_attr[VIN_EMB].vin_basic_attr.wstride);
		vio_info("EMB vstride %d\n", vin_attr->vin_ochn_attr[VIN_EMB].vin_basic_attr.vstride);
	}

	if (vctx->id == VNODE_ID_CAP + VIN_ROI) {  //ROI
		roi_subdev = &vin_node_dev->roi_subdev[vctx->ctx_id];
		vin_attr = &roi_subdev->vin_attr;
		format = vin_attr->vin_ochn_attr[VIN_ROI].vin_basic_attr.format;
		pack_mode = vin_attr->vin_ochn_attr[VIN_ROI].vin_basic_attr.pack_mode;
		hdr_mode = vin_attr->vin_node_attr.cim_attr.func.hdr_mode;
		if (vin_attr->vin_ochn_attr[VIN_ROI].roi_en) {
			group_attr->bit_map |= 1;
			group_attr->is_contig = 1;
			group_attr->info[0].buf_attr.width = vin_attr->vin_ochn_attr[VIN_ROI].roi_attr.roi_width;
			group_attr->info[0].buf_attr.height = vin_attr->vin_ochn_attr[VIN_ROI].roi_attr.roi_height;
			group_attr->info[0].buf_attr.wstride = vin_attr->vin_ochn_attr[VIN_ROI].vin_basic_attr.wstride;
			group_attr->info[0].buf_attr.vstride = vin_attr->vin_ochn_attr[VIN_ROI].vin_basic_attr.vstride;
			vin_node_get_plane(format, pack_mode, hdr_mode, group_attr);
		}
		vio_info("VIN_ROI wstride %d\n", vin_attr->vin_ochn_attr[VIN_ROI].vin_basic_attr.wstride);
		vio_info("VIN_ROI vstride %d\n", vin_attr->vin_ochn_attr[VIN_ROI].vin_basic_attr.vstride);
	}
	vio_info("[S%d] %s done bit_map 0x%x planecount %d \n",
			vin_node_dev->flow_id, __func__, group_attr->bit_map,
			group_attr->info[0].buf_attr.planecount);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief:Start vin_node from working
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_start(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_common_ops *vin_ops;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;

	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	vdev = vctx->vdev;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);

	vin_ops = vin_node_dev->vin_ops[VIN_CIM];
	if (vin_ops && vin_ops->video_start && osal_test_bit(CIM_INIT, &subdev->state)) {
		ret = vin_ops->video_start(vctx);
		if (ret < 0) {
			vio_err("%s cim start fail\n", __func__);
			goto err;
		}
		osal_set_bit(CIM_START, &subdev->state);
	}

	vin_ops = vin_node_dev->vin_ops[VIN_LPWM];
	if (vin_ops && vin_ops->video_start && osal_test_bit(LPWM_INIT, &subdev->state)) {
		ret = vin_ops->video_start(vctx);
		if (ret < 0) {
			vio_err("%s lpwm start fail\n", __func__);
			goto lpwm_fail;
		}
		osal_set_bit(LPWM_START, &subdev->state);
	}

	vin_ops = vin_node_dev->vin_ops[VIN_VCON];
	if (vin_ops && vin_ops->video_start && osal_test_bit(VCON_INIT, &subdev->state)) {
		ret = vin_ops->video_start(vctx);
		if (ret < 0) {
			vio_err("%s vcon start fail\n", __func__);
			goto vcon_fail;
		}
		osal_set_bit(VCON_START, &subdev->state);
	}

	vin_ops = vin_node_dev->vin_ops[VIN_MIPI];
	if (vin_ops && vin_ops->video_start && osal_test_bit(MIPI_INIT, &subdev->state)) {
		ret = vin_ops->video_start(vctx);
		if (ret < 0) {
			vio_err("%s vcon start fail\n", __func__);
			goto mipi_fail;
		}
		osal_set_bit(MIPI_START, &subdev->state);
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;

mipi_fail:
	vin_ops = vin_node_dev->vin_ops[VIN_VCON];
	if (osal_test_and_clear_bit(VCON_START, &subdev->state))
		vin_ops->video_stop(vctx);
vcon_fail:
	vin_ops = vin_node_dev->vin_ops[VIN_LPWM];
	if (osal_test_and_clear_bit(LPWM_START, &subdev->state))
		vin_ops->video_stop(vctx);
lpwm_fail:
	vin_ops = vin_node_dev->vin_ops[VIN_CIM];
	if (osal_test_and_clear_bit(CIM_START, &subdev->state))
		vin_ops->video_stop(vctx);
err:
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Stop vin_node from working
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_stop(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_common_ops *vin_ops;
	struct vio_subdev *vdev;
	struct vin_node_subdev *subdev;

	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	vdev = vctx->vdev;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);

	vin_ops = vin_node_dev->vin_ops[VIN_VCON];
	if (vin_ops && vin_ops->video_stop && osal_test_bit(VCON_START, &subdev->state)) {
		ret = vin_ops->video_stop(vctx);
		if (ret < 0)
			vio_err("%s vcon stop fail\n", __func__);
		else
			osal_clear_bit(VCON_START, &subdev->state);
	}

	vin_ops = vin_node_dev->vin_ops[VIN_LPWM];
	if (vin_ops && vin_ops->video_stop && osal_test_bit(LPWM_START, &subdev->state)) {
		ret = vin_ops->video_stop(vctx);
		if (ret < 0)
			vio_err("%s lpwm stop fail\n", __func__);
		else
			osal_clear_bit(LPWM_START, &subdev->state);
	}

	vin_ops = vin_node_dev->vin_ops[VIN_MIPI];
	if (vin_ops && vin_ops->video_stop && osal_test_bit(MIPI_START, &subdev->state)) {
		ret = vin_ops->video_stop(vctx);
		if (ret < 0)
			vio_err("%s mipi stop fail\n", __func__);
		else
			osal_clear_bit(MIPI_START, &subdev->state);
	}

	vin_ops = vin_node_dev->vin_ops[VIN_CIM];
	if (vin_ops && vin_ops->video_stop && osal_test_bit(CIM_START, &subdev->state)) {
		ret = vin_ops->video_stop(vctx);
		if (ret < 0)
			vio_err("%s cim stop fail\n", __func__);
		else
			osal_clear_bit(CIM_START, &subdev->state);
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Open the calling function of the vin_node device node
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_open(struct vio_video_ctx *vctx)
{
	s32 ret = 0;
	u32 hw_id;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_common_ops *vin_ops;

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	hw_id = vin_node_dev->hw_id;

	vin_ops = vin_node_dev->vin_ops[VIN_CIM];
	if (vin_ops && vin_ops->open) {
		ret = vin_ops->open(vctx);
		if (ret < 0) {
			vio_err("[S%d]%s VIN_CIM fail\n", vctx->ctx_id, __func__);
			return ret;
		}
	}

	vio_info("[S%d]%s done\n", vctx->ctx_id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Close the calling function of the vin_node device node
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_close(struct vio_video_ctx *vctx)
{
	s32 ret = 0, i;
	u32 hw_id, lpwm_chn;
	struct vio_node *vnode;
	struct vio_subdev *vdev;
	struct j6_vin_node_dev *vin_node_dev;
	struct vin_common_ops *vin_ops;
	struct vin_node_subdev *subdev;
	vcon_attr_t got_vcon_attr = { 0 };

	vdev = vctx->vdev;
	vnode = vdev->vnode;
	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	hw_id = vin_node_dev->hw_id;
	subdev = container_of(vdev, struct vin_node_subdev, vdev);
	if (subdev == NULL) {
		vio_err("[S%d]%s vin_node_subdev is NULL\n", vctx->ctx_id, __func__);
		return -EFAULT;
	}

	vin_ops = vin_node_dev->vin_ops[VIN_CIM];
	if (vin_ops && vin_ops->close) {
		ret = vin_ops->close(vctx);
		if (ret < 0) {
			vio_err("[S%d]%s cim close fail\n", vctx->ctx_id, __func__);
		}
	}

	vin_ops = vin_node_dev->vin_ops[VIN_MIPI];
	if (vctx->id == VNODE_ID_SRC && vin_ops && vin_ops->close &&
			osal_test_and_clear_bit(MIPI_OPEN, &subdev->state)) {
		ret = vin_ops->close(vctx);
		if (ret < 0) {
			vio_err("[S%d]%s mipi close fail\n", vctx->ctx_id, __func__);
		}
	}

	vin_ops = vin_node_dev->vin_ops[VIN_VCON];
	if (vctx->id == VNODE_ID_SRC && vin_ops && vin_ops->video_get_attr) {
		ret = vin_ops->video_get_attr(vctx, &got_vcon_attr);
		if (ret < 0) {
			vio_err("[S%d] %s get vcon attr fail\n", vctx->ctx_id, __func__);
		}
	}

	vin_ops = vin_node_dev->vin_ops[VIN_LPWM];
	if (vctx->id == VNODE_ID_SRC && vin_ops && vin_ops->close) {
		for (i = 0; i < LPWM_CHN_NUM; i++) {
			lpwm_chn = got_vcon_attr.lpwm_chn[i];
			if (lpwm_chn == -1 || !osal_test_and_clear_bit(LPWM_CHN0_OPEN + i, &subdev->state))
				continue;

			vctx->file = (void*)&lpwm_chn;
			ret = vin_ops->close(vctx);
			if (ret < 0) {
				vio_err("[S%d]%s LPWM chn %dclose fail\n",
						vctx->ctx_id, __func__, lpwm_chn);
			}
		}
	}

	vin_ops = vin_node_dev->vin_ops[VIN_VCON];
	if (vctx->id == VNODE_ID_SRC && vin_ops && vin_ops->close
			&& osal_test_and_clear_bit(VCON_OPEN, &subdev->state)) {
		ret = vin_ops->close(vctx);
		if (ret < 0) {
			vio_err("[S%d]%s vcon close fail\n", vctx->ctx_id, __func__);
		}
	}

	subdev->state = 0;

	vio_info("[S%d][V%d] %s done\n", vctx->ctx_id, vctx->id, __func__);
	return ret;
}

/**
 * @NO{S10E01C01}
 * @ASIL{B}
 * @brief: Set ctrl function
 * @retval 0: success
 * @retval <0: fail
 * @param[in] *vctx: vio_video_ctx
 * @param[in] cmd: ctrl commod
 * @param[in] arg: arg
 * @param[out] None
 * @data_read None
 * @data_updated None
 * @compatibility None
 * @callgraph
 * @callergraph
 * @design
 */
s32 vin_node_set_ctrl(struct vio_video_ctx *vctx, u32 cmd, unsigned long arg)
{
	s32 ret = -1;
	struct vin_common_ops *vin_ops;
	struct j6_vin_node_dev *vin_node_dev;

	vin_node_dev = (struct j6_vin_node_dev *)vctx->device;
	vin_ops = vin_node_dev->vin_ops[VIN_CIM];

	if (vin_ops && vin_ops->video_s_ctrl) {
		ret = vin_ops->video_s_ctrl(vctx, cmd, arg);
		if (ret < 0) {
			vio_err("[S%d]%s cim set ctrl fail\n", vctx->ctx_id, __func__);
			return ret;
		}
	}

	vio_info("[S%d][V%d] %s done\n", vctx->ctx_id, vctx->id, __func__);
	return ret;
}
