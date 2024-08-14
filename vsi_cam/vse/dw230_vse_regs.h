/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _DW230_VSE_REGS_H_
#define _DW230_VSE_REGS_H_

#define VSE_CTRL                        (0x0304)

/* MI IRQ */
#define VSE_FE_MIS                      (0x0114)
#define VSE_FE_ICR                      (0x0118)
#define VSE_MI0_BUS_CFG                 (0x0d20)
#define VSE_MI_IMSC1                    (0x0d44)
#define VSE_MI_ISR                      (0x0d48)
#define VSE_MI_MIS                      (0x0d50)
#define VSE_MI_MIS1                     (0x0d54)
#define VSE_MI_ICR                      (0x0d58)
#define VSE_MI_ICR1                     (0x0d5c)

/* MI buffer related */
#define VSE_MIn_BASE(n) \
({ \
	typeof(n) _n = (n); \
	(_n < 3 ? (_n * 0xa0 + 0xb10) : (_n * 0xa0 + 0xf30)); \
})
#define VSE_MIn_CTRL(__base)            (__base)
#define VSE_MIn_Y_ADDR(__base)          ((__base) + 0x14)
#define VSE_MIn_Y_SIZE(__base)          ((__base) + 0x18)
#define VSE_MIn_Y_OFFS(__base)          ((__base) + 0x1c)
#define VSE_MIn_CB_ADDR(__base)         ((__base) + 0x30)
#define VSE_MIn_CB_SIZE(__base)         ((__base) + 0x34)
#define VSE_MIn_CB_OFFS(__base)         ((__base) + 0x38)
#define VSE_MIn_CR_ADDR(__base)         ((__base) + 0x3c)
#define VSE_MIn_CR_SIZE(__base)         ((__base) + 0x40)
#define VSE_MIn_CR_OFFS(__base)         ((__base) + 0x44)

/* OSD buffer related */
#define VSE_OSDn_BASE(n)                (n == 0 ? 0x1400: (n == 1 ? 0x148c:\
                                        (n == 2 ? 0x1518: (n == 3 ? 0x15a4:\
                                        (n == 4 ? 0x1630:0x16bc)))))
#define VSE_OSDn_CTRL(__base)           ((__base) + 0x00)
#define VSE_OSDn_ROI0_ADDR(__base)      ((__base) + 0x10)
#define VSE_OSDn_ROI1_ADDR(__base)      ((__base) + 0x20)
#define VSE_OSDn_ROI2_ADDR(__base)      ((__base) + 0x30)
#define VSE_OSDn_ROI3_ADDR(__base)      ((__base) + 0x40)

#define VSE_HISTn_BASE(n)               (n == 0 ? 0x1800: (n == 1 ? 0x1900:\
                                        (n == 2 ? 0x1a00: (n == 3 ? 0x1b00:\
                                        (n == 4 ? 0x1c00:0x1d00)))))
#define VSE_HISTn_ROIn_RANGE01(__base, roi)    ((__base) + 0x90 + (roi) *0x8)
#define VSE_HISTn_ROIn_RANGE23(__base, roi)    ((__base) + 0x90 + (roi) *0x8 + 0x4)


#endif /* _DW230_VSE_REGS_H_ */
