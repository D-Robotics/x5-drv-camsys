// SPDX-License-Identifier: GPL-2.0-only
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/reset.h>

#include "csi_uapi.h"

#include "csi.h"
#include "csi_regs.h"

static const struct csi_irq_reg csi_err_regs[] = {
	{
		CSI_ERR_PHY_FATAL,
		CSI_INT_MAIN_ST_PHY_FATAL, CSI_INT_ST_PHY_FATAL,
		CSI_INT_MSK_PHY_FATAL, CSI_INT_FORCE_PHY_FATAL,
		0x0007000fU, /* ignore err_deskew */
		{
			[0] = "phy_errsotsynchs_0", [1] = "phy_errsotsynchs_1",
			[2] = "phy_errsotsynchs_2", [3] = "phy_errsotsynchs_3",
			[4] = "phy_errsotsynchs_4", [5] = "phy_errsotsynchs_5",
			[6] = "phy_errsotsynchs_6", [7] = "phy_errsotsynchs_7",
			[8] = "err_deskew",
			[9 ... 15] = NULL,
			[16] = "phy_rxinvalidcodehs_0",
			[17] = "phy_rxinvalidcodehs_1",
			[18] = "phy_rxinvalidcodehs_2",
			[19 ... 31] = NULL,
		}
	},
	{
		CSI_ERR_PKT_FATAL,
		CSI_INT_MAIN_ST_PKT_FATAL, CSI_INT_ST_PKT_FATAL,
		CSI_INT_MSK_PKT_FATAL, CSI_INT_FORCE_PKT_FATAL,
		0x00000003U,
		{
			[0] = "err_ecc_double",
			[1] = "shorter_payload",
			[2 ... 31] = NULL,
		}
	},
	{
		CSI_ERR_BNDRY_FRAME_FATAL,
		CSI_INT_MAIN_ST_BNDRY_FRAME_FATAL, CSI_INT_ST_BNDRY_FRAME_FATAL,
		CSI_INT_MSK_BNDRY_FRAME_FATAL, CSI_INT_FORCE_BNDRY_FRAME_FATAL,
		0x0000ffffU,
		{
			[0] = "err_f_bndry_match_vc0", [1] = "err_f_bndry_match_vc1",
			[2] = "err_f_bndry_match_vc2", [3] = "err_f_bndry_match_vc3",
		  	[4] = "err_f_bndry_match_vc4", [5] = "err_f_bndry_match_vc5",
		  	[6] = "err_f_bndry_match_vc6", [7] = "err_f_bndry_match_vc7",
		  	[8] = "err_f_bndry_match_vc8", [9] = "err_f_bndry_match_vc9",
		  	[10] = "err_f_bndry_match_vc10", [11] = "err_f_bndry_match_vc11",
		  	[12] = "err_f_bndry_match_vc12", [13] = "err_f_bndry_match_vc13",
		  	[14] = "err_f_bndry_match_vc14", [15] = "err_f_bndry_match_vc15",
		  	[16] = "err_f_bndry_match_vc16", [17] = "err_f_bndry_match_vc17",
		  	[18] = "err_f_bndry_match_vc18", [19] = "err_f_bndry_match_vc19",
		  	[20] = "err_f_bndry_match_vc20", [21] = "err_f_bndry_match_vc21",
		  	[22] = "err_f_bndry_match_vc22", [23] = "err_f_bndry_match_vc23",
		  	[24] = "err_f_bndry_match_vc24", [25] = "err_f_bndry_match_vc25",
		  	[26] = "err_f_bndry_match_vc26", [27] = "err_f_bndry_match_vc27",
		  	[28] = "err_f_bndry_match_vc28", [29] = "err_f_bndry_match_vc29",
		  	[30] = "err_f_bndry_match_vc30", [31] = "err_f_bndry_match_vc31",
		}
	},
	{
		CSI_ERR_SEQ_FRAME_FATAL,
		CSI_INT_MAIN_ST_SEQ_FRAME_FATAL, CSI_INT_ST_SEQ_FRAME_FATAL,
		CSI_INT_MSK_SEQ_FRAME_FATAL, CSI_INT_FORCE_SEQ_FRAME_FATAL,
		0x0000ffffU,
		{
			[0] = "err_f_seq_vc0", [1] = "err_f_seq_vc1",
			[2] = "err_f_seq_vc2", [3] = "err_f_seq_vc3",
			[4] = "err_f_seq_vc4", [5] = "err_f_seq_vc5",
			[6] = "err_f_seq_vc6", [7] = "err_f_seq_vc7",
			[8] = "err_f_seq_vc8", [9] = "err_f_seq_vc9",
			[10] = "err_f_seq_vc10", [11] = "err_f_seq_vc11",
			[12] = "err_f_seq_vc12", [13] = "err_f_seq_vc13",
			[14] = "err_f_seq_vc14", [15] = "err_f_seq_vc15",
			[16] = "err_f_seq_vc16", [17] = "err_f_seq_vc17",
			[18] = "err_f_seq_vc18", [19] = "err_f_seq_vc19",
			[20] = "err_f_seq_vc20", [21] = "err_f_seq_vc21",
			[22] = "err_f_seq_vc22", [23] = "err_f_seq_vc23",
			[24] = "err_f_seq_vc24", [25] = "err_f_seq_vc25",
			[26] = "err_f_seq_vc26", [27] = "err_f_seq_vc27",
			[28] = "err_f_seq_vc28", [29] = "err_f_seq_vc29",
			[30] = "err_f_seq_vc30", [31] = "err_f_seq_vc31",
		}
	},
	{
		CSI_ERR_CRC_FRAME_FATAL,
		CSI_INT_MAIN_ST_CRC_FRAME_FATAL, CSI_INT_ST_CRC_FRAME_FATAL,
		CSI_INT_MSK_CRC_FRAME_FATAL, CSI_INT_FORCE_CRC_FRAME_FATAL,
		0x0000ffffU,
		{
			[0] = "err_frame_data_vc0", [1] = "err_frame_data_vc1",
			[2] = "err_frame_data_vc2", [3] = "err_frame_data_vc3",
			[4] = "err_frame_data_vc4", [5] = "err_frame_data_vc5",
			[6] = "err_frame_data_vc6", [7] = "err_frame_data_vc7",
			[8] = "err_frame_data_vc8", [9] = "err_frame_data_vc9",
			[10] = "err_frame_data_vc10", [11] = "err_frame_data_vc11",
			[12] = "err_frame_data_vc12", [13] = "err_frame_data_vc13",
			[14] = "err_frame_data_vc14", [15] = "err_frame_data_vc15",
			[16] = "err_frame_data_vc16", [17] = "err_frame_data_vc17",
			[18] = "err_frame_data_vc18", [19] = "err_frame_data_vc19",
			[20] = "err_frame_data_vc20", [21] = "err_frame_data_vc21",
			[22] = "err_frame_data_vc22", [23] = "err_frame_data_vc23",
			[24] = "err_frame_data_vc24", [25] = "err_frame_data_vc25",
			[26] = "err_frame_data_vc26", [27] = "err_frame_data_vc27",
			[28] = "err_frame_data_vc28", [29] = "err_frame_data_vc29",
			[30] = "err_frame_data_vc30", [31] = "err_frame_data_vc31",
		}
	},
	{
		CSI_ERR_PLD_CRC_FATAL,
		CSI_INT_MAIN_ST_PLD_CRC_FATAL, CSI_INT_ST_PLD_CRC_FATAL,
		CSI_INT_MSK_PLD_CRC_FATAL, CSI_INT_FORCE_PLD_CRC_FATAL,
		0x0000ffffU,
		{
			[0] = "err_crc_vc0", [1] = "err_crc_vc1",
			[2] = "err_crc_vc2", [3] = "err_crc_vc3",
			[4] = "err_crc_vc4", [5] = "err_crc_vc5",
			[6] = "err_crc_vc6", [7] = "err_crc_vc7",
			[8] = "err_crc_vc8", [9] = "err_crc_vc9",
			[10] = "err_crc_vc10", [11] = "err_crc_vc11",
			[12] = "err_crc_vc12", [13] = "err_crc_vc13",
			[14] = "err_crc_vc14", [15] = "err_crc_vc15",
			[16] = "err_crc_vc16", [17] = "err_crc_vc17",
			[18] = "err_crc_vc18", [19] = "err_crc_vc19",
			[20] = "err_crc_vc20", [21] = "err_crc_vc21",
			[22] = "err_crc_vc22", [23] = "err_crc_vc23",
			[24] = "err_crc_vc24", [25] = "err_crc_vc25",
			[26] = "err_crc_vc26", [27] = "err_crc_vc27",
			[28] = "err_crc_vc28", [29] = "err_crc_vc29",
			[30] = "err_crc_vc30", [31] = "err_crc_vc31",
		}
	},
	{
		CSI_ERR_DATA_ID,
		CSI_INT_MAIN_ST_DATA_ID, CSI_INT_ST_DATA_ID,
		CSI_INT_MSK_DATA_ID, CSI_INT_FORCE_DATA_ID,
		0x0000ffffU,
		{
			[0] = "err_id_vc0", [1] = "err_id_vc1",
			[2] = "err_id_vc2", [3] = "err_id_vc3",
			[4] = "err_id_vc4", [5] = "err_id_vc5",
			[6] = "err_id_vc6", [7] = "err_id_vc7",
			[8] = "err_id_vc8", [9] = "err_id_vc9",
			[10] = "err_id_vc10", [11] = "err_id_vc11",
			[12] = "err_id_vc12", [13] = "err_id_vc13",
			[14] = "err_id_vc14", [15] = "err_id_vc15",
			[16] = "err_id_vc16", [17] = "err_id_vc17",
			[18] = "err_id_vc18", [19] = "err_id_vc19",
			[20] = "err_id_vc20", [21] = "err_id_vc21",
			[22] = "err_id_vc22", [23] = "err_id_vc23",
			[24] = "err_id_vc24", [25] = "err_id_vc25",
			[26] = "err_id_vc26", [27] = "err_id_vc27",
			[28] = "err_id_vc28", [29] = "err_id_vc29",
			[30] = "err_id_vc30", [31] = "err_id_vc31",
		}
	},
	{
		CSI_ERR_ECC_CORRECTED,
		CSI_INT_MAIN_ST_ECC_CORRECTED, CSI_INT_ST_ECC_CORRECTED,
		CSI_INT_MSK_ECC_CORRECTED, CSI_INT_FORCE_ECC_CORRECTED,
		0x0000ffffU,
		{
			[0] = "err_ecc_corrected0", [1] = "err_ecc_corrected1",
			[2] = "err_ecc_corrected2", [3] = "err_ecc_corrected3",
			[4] = "err_ecc_corrected4", [5] = "err_ecc_corrected5",
			[6] = "err_ecc_corrected6", [7] = "err_ecc_corrected7",
			[8] = "err_ecc_corrected8", [9] = "err_ecc_corrected9",
			[10] = "err_ecc_corrected10", [11] = "err_ecc_corrected11",
			[12] = "err_ecc_corrected12", [13] = "err_ecc_corrected13",
			[14] = "err_ecc_corrected14", [15] = "err_ecc_corrected15",
			[16] = "err_ecc_corrected16", [17] = "err_ecc_corrected17",
			[18] = "err_ecc_corrected18", [19] = "err_ecc_corrected19",
			[20] = "err_ecc_corrected20", [21] = "err_ecc_corrected21",
			[22] = "err_ecc_corrected22", [23] = "err_ecc_corrected23",
			[24] = "err_ecc_corrected24", [25] = "err_ecc_corrected25",
			[26] = "err_ecc_corrected26", [27] = "err_ecc_corrected27",
			[28] = "err_ecc_corrected28", [29] = "err_ecc_corrected29",
			[30] = "err_ecc_corrected30", [31] = "err_ecc_corrected31",
		}
	},
	{
		CSI_ERR_PHY,
		CSI_INT_MAIN_ST_PHY, CSI_INT_ST_PHY,
		CSI_INT_MSK_PHY, CSI_INT_FORCE_PHY,
		0x000f000fU,
		{
			[0] = "phy_errsoths_0", [1] = "phy_errsoths_1",
			[2] = "phy_errsoths_2", [3] = "phy_errsoths_3",
			[4] = "phy_errsoths_4", [5] = "phy_errsoths_5",
			[6] = "phy_errsoths_6", [7] = "phy_errsoths_7",
			[8 ... 15] = NULL,
			[16] = "phy_erresc_0", [17] = "phy_erresc_1",
			[18] = "phy_erresc_2", [19] = "phy_erresc_3",
			[20] = "phy_erresc_4", [21] = "phy_erresc_5",
			[22] = "phy_erresc_6", [23] = "phy_erresc_7",
			[24 ... 31] = NULL,
		}
	},
	{
		CSI_ERR_LINE,
		CSI_INT_MAIN_ST_LINE, CSI_INT_ST_LINE,
		CSI_INT_MSK_LINE, CSI_INT_FORCE_LINE,
		0x00ff00ffU,
		{
			[0] = "err_l_bndry_match_di0", [1] = "err_l_bndry_match_di1",
			[2] = "err_l_bndry_match_di2", [3] = "err_l_bndry_match_di3",
			[4] = "err_l_bndry_match_di4", [5] = "err_l_bndry_match_di5",
			[6] = "err_l_bndry_match_di6", [7] = "err_l_bndry_match_di7",
			[8 ... 15] = NULL,
			[16] = "err_l_seq_di0", [17] = "err_l_seq_di1",
			[18] = "err_l_seq_di2", [19] = "err_l_seq_di3",
			[20] = "err_l_seq_di4", [21] = "err_l_seq_di5",
			[22] = "err_l_seq_di6", [23] = "err_l_seq_di7",
			[24 ... 31] = NULL,
		}
	},
	{
		CSI_ERR_IPI1,
		CSI_INT_MAIN_ST_IPI1_8_FATAL(1), CSI_INT_ST_IPI1_4_FATAL(1),
		CSI_INT_MSK_IPI1_4_FATAL(1), CSI_INT_FORCE_IPI1_4_FATAL(1),
		0x0000003fU,
		{
			[0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
			[2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
			[4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
			[6 ... 31] = NULL,
		}
	},
	{
		CSI_ERR_IPI2,
		CSI_INT_MAIN_ST_IPI1_8_FATAL(2), CSI_INT_ST_IPI1_4_FATAL(2),
		CSI_INT_MSK_IPI1_4_FATAL(2), CSI_INT_FORCE_IPI1_4_FATAL(2),
		0x0000003fU,
		{
			[0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
			[2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
			[4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
			[6 ... 31] = NULL,
		}
	},
	{
		CSI_ERR_IPI3,
		CSI_INT_MAIN_ST_IPI1_8_FATAL(3), CSI_INT_ST_IPI1_4_FATAL(3),
		CSI_INT_MSK_IPI1_4_FATAL(3), CSI_INT_FORCE_IPI1_4_FATAL(3),
		0x0000003fU,
		{
			[0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
			[2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
			[4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
			[6 ... 31] = NULL,
		}
	},
	{
		CSI_ERR_IPI4,
		CSI_INT_MAIN_ST_IPI1_8_FATAL(4), CSI_INT_ST_IPI1_4_FATAL(4),
		CSI_INT_MSK_IPI1_4_FATAL(4), CSI_INT_FORCE_IPI1_4_FATAL(4),
		0x0000003fU,
		{
			[0] = "pixel_if_fifo_underflow", [1] = "pixel_if_fifo_overflow",
			[2] = "pixel_if_frame_sync_err", [3] = "pixel_if_fifo_nempty_fs",
			[4] = "pixel_if_hline_err", [5] = "int_event_fifo_overflow",
			[6 ... 31] = NULL,
		}
	}
};

static u32 handle_set_vc_cfg(struct csi_device *csi, struct csi_msg *m)
{
	if (m->inst >= csi->num_insts - 1)
		return -EINVAL;

	return csi_ipi_set_vc_cfg(csi, m->inst, &m->vc_cfg);
}

static s32 handle_set_format(struct csi_device *csi, struct csi_msg *msg)
{
	struct cam_format f;
	struct csi_ipi_base_cfg ipi_cfg;

	f.width = msg->fmt.width;
	f.height = msg->fmt.height;
	f.format = msg->fmt.format;
	ipi_cfg.id = msg->inst;
	ipi_cfg.adv_val = CSI_IPI_ADV_FEAT_EVSELPROG | CSI_IPI_ADV_FEAT_EN_VIDEO
			| CSI_IPI_ADV_FEAT_EN_EBD | CSI_IPI_ADV_FEAT_MODE_LEGACY;
	ipi_cfg.cut_through = true;
	ipi_cfg.mem_auto_flush = true;
	csi_ipi_init(csi, &ipi_cfg, &f);
	csi->subirq_func = NULL;
	csi->irq_done_func = NULL;
	csi_irq_enable(csi);
	return 0;
}

static s32 handle_set_state(struct csi_device *csi, struct csi_msg *msg)
{
	if (msg->state)
		csi_ipi_start(csi, msg->inst);
	else
		csi_ipi_stop(csi, msg->inst);
	return 0;
}

static s32 handle_reset_control(struct csi_device *csi, struct csi_msg *msg)
{
	if (csi->rst) {
		reset_control_assert(csi->rst);
		udelay(2);
		reset_control_deassert(csi->rst);
	}
	return 0;
}

s32 csi_msg_handler(void *msg, u32 len, void *arg)
{
	struct csi_device *csi = (struct csi_device *)arg;
	struct csi_msg *m = (struct csi_msg *)msg;
	s32 rc = 0;

	if (!csi || !msg || !len)
		return -EINVAL;

	if (csi->is_native)
		return 0;
	switch (m->id) {
	case CSI_MSG_SET_VC_CFG:
		rc = handle_set_vc_cfg(csi, m);
		break;
	case CSI_MSG_SET_LANES:
		csi_set_lanes(csi, m->num_lanes, 0);
		break;
	case CSI_MSG_SET_LANE_RATE:
		rc = csi_set_lane_rate(csi, m->lane_rate);
		break;
	case CSI_MSG_SET_TPG_MODE:
		/* ppipg clk should be enable before RST release if tpg needed */
		csi_set_tpg_mode(csi, m->tpg_cfg.tpg_en, m->tpg_cfg.pkt2pkt_time,
				 0, 0, DATA_TYPE_RGB888);
		break;
	case CSI_MSG_SET_BYPASS:
		csi->is_bypass = m->bypass ? true : false;
		break;
	case CAM_MSG_SET_FORMAT:
		rc = handle_set_format(csi, m);
		break;
	case CAM_MSG_SET_STATE:
		rc = handle_set_state(csi, m);
		break;
	case CAM_MSG_RESET_CONTROL:
		rc = handle_reset_control(csi, m);
		break;
	default:
		return -EINVAL;
	}
	return rc;
}

void csi_irq_enable(struct csi_device *csi)
{
	int i;
	u32 val;

	(void)csi_read(csi, CSI_INT_ST_MAIN);
	for (i = 0; i < ARRAY_SIZE(csi_err_regs); i++) {
		(void)csi_read(csi, csi_err_regs[i].reg_st);
		val = csi_read(csi, csi_err_regs[i].reg_mask);
		val |= csi_err_regs[i].err_mask;
		csi_write(csi, csi_err_regs[i].reg_mask, val);
	}
}

void csi_irq_disable(struct csi_device *csi)
{
	int i;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(csi_err_regs); i++) {
		val = csi_read(csi, csi_err_regs[i].reg_mask);
		val &= ~(csi_err_regs[i].err_mask);
		csi_write(csi, csi_err_regs[i].reg_mask, val);
	}
}

int csi_irq_disable_mask(struct csi_device *csi, u32 mask)
{
	int i, has_msk = 0;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(csi_err_regs); i++) {
		val = csi_read(csi, csi_err_regs[i].reg_mask);
		if (mask & (1 << csi_err_regs[i].id)) {
			val &= ~(csi_err_regs[i].err_mask);
			csi_write(csi, csi_err_regs[i].reg_mask, val);
		} else {
			if (val)
				has_msk = 1;
		}
	}
	return has_msk;
}

irqreturn_t csi_irq_handler(int irq, void *arg)
{
	struct csi_device *csi = (struct csi_device *)arg;
	u32 val, sub_val, mask, irq_main_st, should_reset = 0;
	int i;

	val = csi_read(csi, CSI_INT_ST_MAIN);
	irq_main_st = val;
	for (i = 0; i < ARRAY_SIZE(csi_err_regs); i++) {
		mask = csi_err_regs[i].st_mask;
		if ((val & mask) == 0)
			continue;
		sub_val = csi_read(csi, csi_err_regs[i].reg_st) & csi_err_regs[i].err_mask;
		if ((csi_err_regs[i].id >= CSI_ERR_IPI1) && (csi_err_regs[i].id <= CSI_ERR_IPI4)) {
			if (sub_val & CSI_INT_ST_IPI_OVERFLOW)
				should_reset = 1;
		}
		if (csi->subirq_func)
			csi->subirq_func(csi, sub_val, &csi_err_regs[i], should_reset);

		val &= ~mask;
		if (val == 0)
			break;
	}
	if (csi->irq_done_func)
		csi->irq_done_func(csi, irq_main_st);

	return IRQ_HANDLED;
}
