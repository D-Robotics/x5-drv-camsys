/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/**
 * @file hobot_vin_vcon_ops.h
 *
 * @NO{S10E01C02}
 * @ASIL{B}
 */

#ifndef __HOBOT_VIN_VCON_OPS_H__
#define __HOBOT_VIN_VCON_OPS_H__ /* PRQA S 0603 */ /* header file macro */

#include "hobot_vin_vcon_osal.h"
#include "hobot_vin_vcon_sub.h"
#include "vin_node_config.h"

/**
 * @def VCON_DEV_MAX_NUM
 * vcon device nax number
 */
#define	VIN_VCON_DNAME		"vin_vcon"
/**
 * @def VCON_DEV_MAX_NUM
 * vcon device nax number
 */
#ifdef CONFIG_HOBOT_VIN_VCON_DEV_NUM
#define VCON_DEV_MAX_NUM	CONFIG_HOBOT_VIN_VCON_DEV_NUM
#else
#define VCON_DEV_MAX_NUM	(8)
#endif
/**
 * @def VCON_CTX_MAX_NUM
 * vcon contex nax number
 */
#define VCON_CTX_MAX_NUM	(4)
/**
 * @def VCON_FLOW_MAX_NUM
 * vcon contex nax number
 */
#define VCON_FLOW_MAX_NUM	(VCON_DEV_MAX_NUM * VCON_CTX_MAX_NUM)

/**
 * @def VCON_ATTR_V_BUS_MAIN
 * vcon attr valid bit mask: bus_main
 */
#define VCON_ATTR_V_BUS_MAIN	(0x1 << 0)
/**
 * @def VCON_ATTR_V_BUS_SEC
 * vcon attr valid bit mask: bus_sec
 */
#define VCON_ATTR_V_BUS_SEC	(0x1 << 1)
/**
 * @def VCON_ATTR_V_GPIO_POC
 * vcon attr valid bit mask: gpio_poc
 */
#define VCON_ATTR_V_GPIO_POC	(0x1 << 2)
/**
 * @def VCON_ATTR_V_GPIO_DES
 * vcon attr valid bit mask: gpio_des
 */
#define VCON_ATTR_V_GPIO_DES	(0x1 << 3)
/**
 * @def VCON_ATTR_V_GPIO_SER
 * vcon attr valid bit mask: gpio_ser
 */
#define VCON_ATTR_V_GPIO_SER	(0x1 << 4)
/**
 * @def VCON_ATTR_V_GPIO_OTH
 * vcon attr valid bit mask: gpio_oth
 */
#define VCON_ATTR_V_GPIO_OTH	(0x1 << 5)
/**
 * @def VCON_ATTR_V_SENSOR_ERR
 * vcon attr valid bit mask: sensor_err
 */
#define VCON_ATTR_V_SENSOR_ERR	(0x1 << 6)
/**
 * @def VCON_ATTR_V_LPWM
 * vcon attr valid bit mask: lpwm index
 */
#define VCON_ATTR_V_LPWM	(0x1 << 7)
/**
 * @def VCON_ATTR_V_MIPI_RX
 * vcon attr valid bit mask: mipi rx index
 */
#define VCON_ATTR_V_MIPI_RX	(0x1 << 8)
/**
 * @def VCON_ATTR_V_MIPI_TX
 * vcon attr valid bit mask: mipi tx index
 */
#define VCON_ATTR_V_MIPI_TX	(0x1 << 9)
/**
 * @def VCON_ATTR_V_TYPE
 * vcon attr valid bit mask: type
 */
#define VCON_ATTR_V_TYPE	(0x1 << 10)
/**
 * @def VCON_ATTR_V_ALL
 * vcon attr valid bit mask: all bits valid
 */
#define VCON_ATTR_V_ALL		((0x1 << 11) - 1)
/**
 * @def VCON_ATTR_V_BUS
 * vcon attr valid bit mask: bus type: bus_main, bus_sec
 */
#define VCON_ATTR_V_BUS		(VCON_ATTR_V_BUS_MAIN | VCON_ATTR_V_BUS_SEC)
/**
 * @def VCON_ATTR_V_GPIO
 * vcon attr valid bit mask: gpio type: gpio_xx
 */
#define VCON_ATTR_V_GPIO	(VCON_ATTR_V_GPIO_POC | VCON_ATTR_V_GPIO_DES | \
				 VCON_ATTR_V_GPIO_SER | VCON_ATTR_V_GPIO_OTH)
/**
 * @def VCON_ATTR_V_MIPI
 * vcon attr valid bit mask: mipi type: rx, tx
 */
#define VCON_ATTR_V_MIPI	(VCON_ATTR_V_MIPI_RX | VCON_ATTR_V_MIPI_TX)

/**
 * @def VCON_ATTR_INVALID
 * vcon attr invalid value
 */
#define VCON_ATTR_INVALID	(-1)
/**
 * @def VCON_ATTR_INVGPIO
 * vcon gpio attr invalid value
 */
#define VCON_ATTR_INVGPIO	(0)

/**
 * @def VCON_SENSOR_ERR_MAX
 * vcon attr number max of sensor_err
 */
#define VCON_SENSOR_ERR_MAX	(4)
/**
 * @def VCON_LPWM_CHN_MAX
 * vcon attr number max of lpwm channel
 */
#define VCON_LPWM_CHN_MAX	(4)

/**
 * @enum vcon_dev_type_e
 * vcon device type enum
 * @NO{S10E01C02}
 */
enum vcon_dev_type_e {
	VCON_SELF,
	VCON_MAIN,
	VCON_FOLLOW,
	VCON_INVALID,
};

/**
 * @def VCON_DEV_TYPTE_NAMES
 * vcon dev type name string array
 */
#define VCON_DEV_TYPTE_NAMES { \
	"self", \
	"main", \
	"follow", \
	"invalid", \
}

/**
 * @enum vcon_event_type_e
 * vcon event type enum
 * @NO{S10E01C02}
 */
enum vcon_event_type_e {
	VCON_E_FRAME,
	VCON_E_ERROR,
	VCON_E_RESET,
	VCON_E_INVALID,
};

/**
 * @enum vcon_rx_phy_mode_e
 * vcon attr rx_phy_mode enum
 * @NO{S10E01C02}
 */
enum vcon_rx_phy_mode_e {
	VCON_RX_NOTUSE,
	VCON_RX_DPHY,
	VCON_RX_CPHY,
	VCON_RX_INVALID,
};

/**
 * @enum vcon_tx_phy_mode_e
 * vcon attr tx_phy_mode enum
 * @NO{S10E01C02}
 */
enum vcon_tx_phy_mode_e {
	VCON_TX_NOTUSE,
	VCON_TX_CSI,
	VCON_TX_DSI,
	VCON_TX_INVALID,
};

#ifndef VIN_NODE_CONFIG_H
/**
 * @enum vcon_gpio_e
 * vcon gpio type enum
 * @NO{S10E01C02}
 */
enum vcon_gpio_e {
	/* poc gpios */
	VGPIO_POC_PWREN,
	VGPIO_POC_EN,
	VGPIO_POC_INT,
	VGPIO_POC_ENC0,
	VGPIO_POC_ENC1,
	VGPIO_POC_ENC2,
	VGPIO_POC_ENC3,
	VGPIO_POC_NC,
	/* des gpios */
	VGPIO_DES_PWREN,
	VGPIO_DES_PWDN,
	VGPIO_DES_LOCK,
	VGPIO_DES_ERRB,
	VGPIO_DES_CFG0,
	VGPIO_DES_CFG1,
	VGPIO_DES_ERROR0,
	VGPIO_DES_ERROR1,
	VGPIO_DES_ERROR2,
	VGPIO_DES_ERROR3,
	VGPIO_DES_NC1,
	VGPIO_DES_NC2,
	/* ser gpios */
	VGPIO_SER_PWREN,
	VGPIO_SER_PWDN,
	VGPIO_SER_LOCK,
	VGPIO_SER_ERRB,
	VGPIO_SER_CFG0,
	VGPIO_SER_CFG1,
	VGPIO_SER_CFG2,
	VGPIO_SER_NC,
	/* oth gpios */
	VGPIO_OTH_0,
	VGPIO_OTH_1,
	VGPIO_OTH_2,
	VGPIO_OTH_3,
	/* local info */
	VGPIO_NUM,
	VGPIO_POC_BASE = VGPIO_POC_PWREN,
	VGPIO_POC_NUM = VGPIO_DES_PWREN - VGPIO_POC_PWREN,
	VGPIO_DES_BASE = VGPIO_DES_PWREN,
	VGPIO_DES_NUM = VGPIO_SER_PWREN - VGPIO_DES_PWREN,
	VGPIO_SER_BASE = VGPIO_SER_PWREN,
	VGPIO_SER_NUM = VGPIO_OTH_0 - VGPIO_SER_PWREN,
	VGPIO_OTH_BASE = VGPIO_OTH_0,
	VGPIO_OTH_NUM = VGPIO_NUM - VGPIO_OTH_0,
};
#endif

/**
 * @def VCON_GPIO_NAMES_POC
 * vcon poc gpio name string array
 */
#define VCON_GPIO_NAMES_POC  \
	/* poc gpios */ \
	"poc_pwren", \
	"poc_en", \
	"poc_int", \
	"poc_enc0", \
	"poc_enc1", \
	"poc_enc2", \
	"poc_enc3", \
	"poc_nc"

/**
 * @def VCON_GPIO_NAMES_DES
 * vcon des gpio name string array
 */
#define VCON_GPIO_NAMES_DES  \
	/* des gpios */ \
	"des_pwren", \
	"des_pwdn", \
	"des_lock", \
	"des_errb", \
	"des_cfg0", \
	"des_cfg1", \
	"des_error0", \
	"des_error1", \
	"des_error2", \
	"des_error3", \
	"des_nc1", \
	"des_nc2"

/**
 * @def VCON_GPIO_NAMES_SER
 * vcon ser gpio name string array
 */
#define VCON_GPIO_NAMES_SER  \
	/* ser gpios */ \
	"ser_pwren", \
	"ser_pwdn", \
	"ser_lock", \
	"ser_errb", \
	"ser_cfg0", \
	"ser_cfg1", \
	"ser_cfg2", \
	"ser_nc"

/**
 * @def VCON_GPIO_NAMES_OTH
 * vcon oth gpio name string array
 */
#define VCON_GPIO_NAMES_OTH  \
	/* oth gpios */ \
	"oth_0", \
	"oth_1", \
	"oth_2", \
	"oth_3"

/**
 * @def VCON_ATTR_NAMES
 * vcon attr name string array
 */
#define VCON_ATTR_NAMES { \
	"bus_main", \
	"bus_second", \
	VCON_GPIO_NAMES_POC, \
	VCON_GPIO_NAMES_DES, \
	VCON_GPIO_NAMES_SER, \
	VCON_GPIO_NAMES_OTH, \
	"sensor_err0", "sensor_err1", "sensor_err2", "sensor_err3", \
	"lpwm_chn0", "lpwm_chn1", "lpwm_chn2", "lpwm_chn3", \
	"rx_phy_mode", "rx_phy_index", "rx_phy_link", \
	"tx_phy_mode", "tx_phy_index", "tx_phy_link", \
	"vcon_type", "vcon_link", \
}

/**
 * @def VCON_ATTR_V_NUMS
 * vcon attr vflag number array
 */
#define VCON_ATTR_V_NUMS { \
	1, 1, \
	VGPIO_POC_NUM, \
	VGPIO_DES_NUM, \
	VGPIO_SER_NUM, \
	VGPIO_OTH_NUM, \
	4, 4, 3, 3, 2, \
}

/**
 * @struct vcon_user_s
 * vcon device user struct for operation
 * @NO{S10E01C02}
 */
struct vcon_user_s {
	osal_mutex_t open_mutex;
	osal_mutex_t mutex;
	uint32_t     open_cnt;
	uint32_t     init_cnt;
};

/**
 * @struct vcon_grp_s
 * vcon device group struct for deserial operation
 * @NO{S10E01C02}
 */
struct vcon_grp_s {
	uint32_t     group_mask;
	uint32_t     deserial_attached;
	uint32_t     deserial_index;
	uint32_t     deserial_start_cnt;
	uint32_t     deserial_link[VCON_CTX_MAX_NUM];
};

/**
 * @struct vcon_ctx_s
 * vcon description struct of vin contex attached
 * @NO{S10E01C02}
 */
struct vcon_ctx_s {
	uint32_t          flow_id;
	uint32_t          vcon_id;
	uint32_t          ctx_id;
	uint32_t          sensor_attached;
	uint32_t          sensor_index;
	uint32_t          sensor_start_cnt;
};

typedef struct vcon_device_s vcon_device_t;
/**
 * @struct vcon_device_s
 * vcon device struct
 * @NO{S10E01C02}
 */
struct vcon_device_s {
	int32_t            index;
	int32_t            rx_index;
	int32_t            tx_index;
	struct os_dev      osdev;

	struct vcon_attr_s info;
	struct vcon_attr_s config;
	struct vcon_user_s user;
	struct vcon_grp_s grp;
	struct vcon_ctx_s ctx[VCON_CTX_MAX_NUM];
}; // vcon设备结构体.

/**
 * @struct vin_vcon_s
 * global operation struct vin vcon driver
 * @NO{S10E01C02}
 */
typedef struct vin_vcon_s {
	struct vcon_device_s vcon[VCON_DEV_MAX_NUM];
	struct vcon_device_s *vcon_rx[VCON_DEV_MAX_NUM];
	struct vcon_device_s *vcon_tx[VCON_DEV_MAX_NUM];
	struct vcon_ctx_s *ctx_flow[VCON_FLOW_MAX_NUM];
	int32_t vcon_num;
	int32_t vcon_mask;
	int32_t setcb_done[VCON_SUBNUM];
	struct vio_callback_ops *sub_cops[VCON_SUBNUM];
} vin_vcon_t;

/* inter apis */
extern int32_t vcon_device_init_get(int32_t index, struct vcon_device_s **pvcon);
extern void vcon_device_exit_put(struct vcon_device_s *vcon);
extern int32_t vcon_device_attr_parse(struct vcon_device_s *vcon);
extern void vcon_device_attr_show(struct vcon_device_s *vcon);
extern int32_t vcon_driver_init_done(void);
extern void vcon_driver_exit_done(void);

/* depend apis */
extern int32_t vcon_sub_dummy_return(void);

#endif // __HOBOT_VIN_VCON_OPS_H__
