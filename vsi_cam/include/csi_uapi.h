/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _UAPI_LINUX_CSI_H_
#define _UAPI_LINUX_CSI_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#include "cam_uapi.h"

#define CSI_UID(n) cam_fourcc('c', 's', 'i', (n) + '0')

#define CSI_MSG_SET_VC_CFG    (0x1)
#define CSI_MSG_SET_LANES     (0x2)
#define CSI_MSG_SET_LANE_RATE (0x3)
#define CSI_MSG_SET_TPG_MODE  (0x4)
#define CSI_MSG_SET_BYPASS    (0x5)

#define CSI_IPI_ADV_FEAT_EVSELPROG   BIT(16)
#define CSI_IPI_ADV_FEAT_EN_VIDEO    BIT(17)
#define CSI_IPI_ADV_FEAT_EN_LS       BIT(18)
#define CSI_IPI_ADV_FEAT_EN_NULL     BIT(19)
#define CSI_IPI_ADV_FEAT_EN_BLANKING BIT(20)
#define CSI_IPI_ADV_FEAT_EN_EBD      BIT(21)
#define CSI_IPI_ADV_FEAT_MODE_LEGACY BIT(24)

enum csi_vc_id {
	CSI_VC0,
	CSI_VC1,
	CSI_VC2,
	CSI_VC3,
};

enum csi_vc_mode {
	CSI_VC_OFF,
	CSI_VC_CAMERA,
	CSI_VC_CTRL,
};

struct csi_vc_cfg {
	__u32 vc_id;
	__u32 vc_mode;
	__u32 vc_ebd_en;
};

struct csi_tpg_cfg {
	__u32 tpg_en;
	__u32 pkt2pkt_time;
};

struct csi_msg {
	__u32 id;
	__u32 inst;
	union {
		struct csi_vc_cfg vc_cfg;
		struct csi_tpg_cfg tpg_cfg;
		struct cam_format fmt;
		__u32 num_lanes;
		__u32 lane_rate;
		__u32 bypass;
		__u32 state; /* enum cam_state */
		struct cam_clk clk;
	};
};

#endif /* _UAPI_LINUX_CSI_H_ */
