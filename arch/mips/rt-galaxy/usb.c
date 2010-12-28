/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/delay.h>

#include <rt-galaxy-soc.h>
#include <rt-galaxy-io.h>

#define USBPHY_30                   0xe0
#define USBPHY_XCVR_AUTOK(v)        ((v & 0x1) << 7)
#define USBPHY_XCVR_SC(v)           ((v & 0xf) << 3)
#define USBPHY_XCVR_CP(v)           ((v & 0x7) << 0)
#define USBPHY_31                   0xe1
#define USBPHY_XCVR_CALL_HOST(v)    ((v & 0x1) << 7)
#define USBPHY_XCVR_ZERES_SEL(v)    ((v & 0x1) << 6)
#define USBPHY_XCVR_ZO_EN(v)        ((v & 0x1) << 5)
#define USBPHY_XCVR_SD(v)           ((v & 0x3) << 3)
#define USBPHY_XCVR_SR(v)           ((v & 0x7) << 0)
#define USBPHY_32                   0xe2
#define USBPHY_SEN(v)               ((v & 0xf) << 4)
#define USBPHY_SH(v)                ((v & 0xf) << 0)
#define USBPHY_33                   0xe3
#define USBPHY_HSXMPTPDEN(v)        ((v & 0x1) << 7)
#define USBPHY_XCVR_CAL(v)          ((v & 0x1) << 6)
#define USBPHY_XCVR_DB(v)           ((v & 0x7) << 3)
#define USBPHY_XCVR_DR(v)           ((v & 0x7) << 0)
#define USBPHY_34                   0xe4
#define USBPHY_XCVR_TS(v)           ((v & 0x7) << 3)
/* USBPHY_34 : Venus, Neptune */
#define USBPHY_TPA_EN(v)            ((v & 0x1) << 7)
#define USBPHY_TPB_EN(v)            ((v & 0x1) << 6)
#define USBPHY_XCVR_SE(v)           ((v & 0x7) << 0)
/* USBPHY_34 : Mars */
#define USBPHY_DBG_EN(v)            ((v & 0x1) << 7)
#define USBPHY_35                   0xe5
#define USBPHY_XCVR_SRC(v)          ((v & 0x7) << 2)
#define USBPHY_HSTESTEN(v)          ((v & 0x1) << 1)
#define USBPHY_XCVR_NSQDLY(v)       ((v & 0x1) << 0)
/* USBPHY_35 : Venus, Neptune */
#define USBPHY_TPC_EN(v)            ((v & 0x1) << 7)
#define USBPHY_XCVR_SP(v)           ((v & 0x3) << 5)
#define USBPHY_HSTESTEN(v)          ((v & 0x1) << 1)
/* USBPHY_35 : Mars */
#define USBPHY_SE0_LVL(v)           ((v & 0x1) << 7)
#define USBPHY_FORCE_XTL_ON(v)      ((v & 0x1) << 1)
#define USBPHY_36                   0xe6
#define USBPHY_XCVR_SENH(v)         ((v & 0xf) << 4)
#define USBPHY_XCVR_ADJR(v)         ((v & 0xf) << 0)
#define USBPHY_37                   0xe7
#define USBPHY_LDO(v)               ((v & 0x7) << 3)
#define USBPHY_LDO_TN(v)            ((v & 0x3) << 1)
#define USBPHY_PLL_TEST_EN(v)       ((v & 0x1) << 0)
#define USBPHY_38                   0xf0
#define USBPHY_DBNC_EN(v)           ((v & 0x1) << 7)
#define USBPHY_DISCON_EN(v)         ((v & 0x1) << 6)
#define USBPHY_EN_ERR_UNDERRUN(v)   ((v & 0x1) << 5)
#define USBPHY_LATE_DLLEN(v)        ((v & 0x1) << 4)
#define USBPHY_INTG(v)              ((v & 0x1) << 3)
#define USBPHY_SOP_KK(v)            ((v & 0x1) << 2)
#define USBPHY_SLB_INNER(v)         ((v & 0x1) << 1)
#define USBPHY_SLB_EN(v)            ((v & 0x1) << 0)
#define USBPHY_39                   0xf1
#define USBPHY_UTMI_POS_OUT(v)      ((v & 0x1) << 7)
#define USBPHY_SLB_RST(v)           ((v & 0x1) << 6)
#define USBPHY_SLB_SEL(v)           ((v & 0x3) << 4)
#define USBPHY_AUTO_SEL(v)          ((v & 0x1) << 3)
#define USBPHY_TX_DELAY(v)          ((v & 0x3) << 1)
#define USBPHY_SLB_FS(v)            ((v & 0x1) << 0)
#define USBPHY_3A                   0xf2
#define USBPHY_FORCE_XCVRSEL(v)     ((v & 0x1) << 7)
#define USBPHY_XCVRSEL_MODE(v)      ((v & 0x3) << 5)
#define USBPHY_FORCE_TERMSEL(v)     ((v & 0x1) << 4)
#define USBPHY_TERMSEL_MODE(v)      ((v & 0x1) << 3)
#define USBPHY_FORCE_OPMODE(v)      ((v & 0x1) << 2)
#define USBPHY_OPMODE_MODE(v)       ((v & 0x3) << 0)
#define USBPHY_3B                   0xf3
#define USBPHY_FORCE_SERIAL(v)      ((v & 0x1) << 7)
#define USBPHY_SERIAL_MODE(v)       ((v & 0x1) << 6)
#define USBPHY_FORCE_SUSPENDM(v)    ((v & 0x1) << 5)
#define USBPHY_SUSPENDM_MODE(v)     ((v & 0x1) << 4)
#define USBPHY_FORCE_TXSE0(v)       ((v & 0x1) << 3)
#define USBPHY_TXSE0_MODE(v)        ((v & 0x1) << 2)
#define USBPHY_FORCE_TXENABLE_N(v)  ((v & 0x1) << 1)
#define USBPHY_TXENABLE_N_MODE(v)   ((v & 0x1) << 0)

#define WAIT_FOR_BUSY() while(rtgalaxy_reg_readl(RTGALAXY_USB_EHCI_INSNREG05) & RTGALAXY_USB_EHCI_INSNREG05_VBUSY_MASK);

static void rtgalaxy_usbphy_init(uint8_t port)
{
    uint32_t raddr = (port == 1) ? RTGALAXY_USB_HOST1_VSTATUS : RTGALAXY_USB_HOST2_VSTATUS;
    uint32_t rdata;

    WAIT_FOR_BUSY();

    rdata  = rtgalaxy_reg_readl(RTGALAXY_USB_EHCI_INSNREG05);
    rdata &= ~RTGALAXY_USB_EHCI_INSNREG05_VPORT_MASK;
    rdata |= RTGALAXY_USB_EHCI_INSNREG05_VPORT(port);
    rtgalaxy_reg_writel(RTGALAXY_USB_EHCI_INSNREG05, rdata);
    WAIT_FOR_BUSY();

    rdata  = rtgalaxy_reg_readl(RTGALAXY_USB_EHCI_INSNREG05);
    rdata &= ~(RTGALAXY_USB_EHCI_INSNREG05_VLOADM_MASK | RTGALAXY_USB_EHCI_INSNREG05_VCTRL_MASK);
    rdata |= RTGALAXY_USB_EHCI_INSNREG05_VLOADM(1);
    rdata |= RTGALAXY_USB_EHCI_INSNREG05_VCTRL(0);
    rtgalaxy_reg_writel(RTGALAXY_USB_EHCI_INSNREG05, rdata);
    WAIT_FOR_BUSY();

    rtgalaxy_reg_writel(raddr, 0);
    WAIT_FOR_BUSY();
}

static void rtgalaxy_usbphy_setreg(uint8_t port, uint8_t addr, uint8_t data)
{
    uint32_t raddr = (port == 1) ? RTGALAXY_USB_HOST1_VSTATUS : RTGALAXY_USB_HOST2_VSTATUS;
    uint32_t rdata;

    rtgalaxy_reg_writel(raddr, data);
    WAIT_FOR_BUSY();

    rdata  = rtgalaxy_reg_readl(RTGALAXY_USB_EHCI_INSNREG05);
    rdata &= ~(RTGALAXY_USB_EHCI_INSNREG05_VLOADM_MASK | RTGALAXY_USB_EHCI_INSNREG05_VPORT_MASK | RTGALAXY_USB_EHCI_INSNREG05_VCTRL_MASK);
    rdata |= RTGALAXY_USB_EHCI_INSNREG05_VPORT(port);
    rdata |= RTGALAXY_USB_EHCI_INSNREG05_VLOADM(1);
    rdata |= RTGALAXY_USB_EHCI_INSNREG05_VCTRL(addr);
    rtgalaxy_reg_writel(RTGALAXY_USB_EHCI_INSNREG05, rdata);
    WAIT_FOR_BUSY();

    rdata &= ~RTGALAXY_USB_EHCI_INSNREG05_VLOADM_MASK;
    rtgalaxy_reg_writel(RTGALAXY_USB_EHCI_INSNREG05, rdata);
    WAIT_FOR_BUSY();

    rdata &= ~RTGALAXY_USB_EHCI_INSNREG05_VCTRL_MASK;
    rdata |= RTGALAXY_USB_EHCI_INSNREG05_VLOADM(1);
    rdata |= RTGALAXY_USB_EHCI_INSNREG05_VCTRL(addr >> 4);
    rtgalaxy_reg_writel(RTGALAXY_USB_EHCI_INSNREG05, rdata);
    WAIT_FOR_BUSY();

    rdata &= ~RTGALAXY_USB_EHCI_INSNREG05_VLOADM_MASK;
    rtgalaxy_reg_writel(RTGALAXY_USB_EHCI_INSNREG05, rdata);
    WAIT_FOR_BUSY();
}

void rtgalaxy_usbphy_setup(void)
{
    int nports, n;
  u32 data;

	nports = rtgalaxy_is_mars_soc() ? 2 : 1;

  rtgalaxy_reg_writel(RTGALAXY_USB_EHCI_INSNREG03, 0x00000001);
  rtgalaxy_reg_writel(RTGALAXY_USB_EHCI_INSNREG01, 0x01000040);

	if (rtgalaxy_is_mars_soc()) {
		if (rtgalaxy_info.chip_rev == 0xa0) {
			rtgalaxy_reg_writel(0x4000a081, RTGALAXY_USB_HOST1_USBIP_INPUT);
			rtgalaxy_reg_writel(0x40002000, RTGALAXY_USB_HOST2_USBIP_INPUT);
		} else {
			rtgalaxy_reg_writel(0x4400a081, RTGALAXY_USB_HOST1_USBIP_INPUT);
			rtgalaxy_reg_writel(0x44002000, RTGALAXY_USB_HOST2_USBIP_INPUT);
		}

		data  = rtgalaxy_reg_readl(RTGALAXY_USB_HOST_VERSION);
		data |= RTGALAXY_USB_HOST_VERSION_NO_USE_DONE;
		rtgalaxy_reg_writel(data, RTGALAXY_USB_HOST_VERSION);
	}

	for(n=1; n <= nports; n++) {
      rtgalaxy_usbphy_init(n);

      rtgalaxy_usbphy_setreg(n, USBPHY_30, 
                             USBPHY_XCVR_AUTOK(0x1) | USBPHY_XCVR_SC(0x3) | USBPHY_XCVR_CP(0x1));
      rtgalaxy_usbphy_setreg(n, USBPHY_31, 
                             USBPHY_XCVR_CALL_HOST(0x1) | USBPHY_XCVR_ZERES_SEL(0x0) | USBPHY_XCVR_ZO_EN(0x1) |
                             USBPHY_XCVR_SD(0x1) | USBPHY_XCVR_SR(0x4));

      rtgalaxy_usbphy_setreg(n, USBPHY_32, USBPHY_SEN(0x8) | USBPHY_SH(0x5));

      data = (rtgalaxy_is_venus_soc()) ? 0x1 : 0x7;
      rtgalaxy_usbphy_setreg(n, USBPHY_33, USBPHY_HSXMPTPDEN(0x1) | USBPHY_XCVR_CAL(0x1) | USBPHY_XCVR_CAL(0x4) | data);

      if (rtgalaxy_is_mars_soc()) {
          data = USBPHY_DBG_EN(0x0) | USBPHY_XCVR_TS(0x0);
      } else {
          data = USBPHY_TPA_EN(0x0) | USBPHY_TPB_EN(0x0) | USBPHY_XCVR_TS(0x0) | USBPHY_XCVR_SE(0x2);
      }
      rtgalaxy_usbphy_setreg(n, USBPHY_34, data);

      data = USBPHY_XCVR_SRC(0x7) | USBPHY_HSTESTEN(0x0) | USBPHY_XCVR_NSQDLY(0x1);
      if (rtgalaxy_is_mars_soc()) {
          data |= USBPHY_SE0_LVL(0x1);
      } else {
          data = USBPHY_TPC_EN(0x0) | USBPHY_XCVR_SP(0x0);
      }
      rtgalaxy_usbphy_setreg(n, USBPHY_35, data);
      rtgalaxy_usbphy_setreg(n, USBPHY_36, USBPHY_XCVR_SENH(0xd) | USBPHY_XCVR_ADJR(0x8));

      if (rtgalaxy_is_mars_soc())
          rtgalaxy_usbphy_setreg(n, USBPHY_37, USBPHY_LDO(0x0) | USBPHY_LDO_TN(0x3) | USBPHY_PLL_TEST_EN(0x0));
          
      rtgalaxy_usbphy_setreg(n, USBPHY_38, 
                             USBPHY_DBNC_EN(0x1) | USBPHY_DISCON_EN(0x1) | USBPHY_EN_ERR_UNDERRUN(0x1) | 
                             USBPHY_LATE_DLLEN(0x1) | USBPHY_INTG(0x1) | USBPHY_SOP_KK(0x1) | 
                             USBPHY_SLB_INNER(0x0) | USBPHY_SLB_EN(0));

      if (rtgalaxy_is_mars_soc()) {
          rtgalaxy_usbphy_setreg(n, USBPHY_39, 
                                 USBPHY_UTMI_POS_OUT(0x1) | USBPHY_SLB_RST(0x0) | USBPHY_SLB_SEL(0x0) |
                                 USBPHY_AUTO_SEL(0x1) | USBPHY_TX_DELAY(0x1) | USBPHY_SLB_FS(0x0));
          rtgalaxy_usbphy_setreg(n, USBPHY_3A, 
                                 USBPHY_FORCE_XCVRSEL(0x0) | USBPHY_XCVRSEL_MODE(0x0) | USBPHY_FORCE_TERMSEL(0x0) |
                                 USBPHY_TERMSEL_MODE(0x0) | USBPHY_FORCE_OPMODE(0x0) | USBPHY_OPMODE_MODE(0x0));
          rtgalaxy_usbphy_setreg(n, USBPHY_3B, 
                                 USBPHY_FORCE_SERIAL(0x0) | USBPHY_SERIAL_MODE(0x0) | 
                                 USBPHY_FORCE_SUSPENDM(0x0) | USBPHY_SUSPENDM_MODE(0x1) |
                                 USBPHY_FORCE_TXSE0(0x0) | USBPHY_TXSE0_MODE(0x0) | 
                                 USBPHY_FORCE_TXENABLE_N(0x0) | USBPHY_TXENABLE_N_MODE(0x1));
      }
  }

  rtgalaxy_reg_writel(RTGALAXY_USB_OHCI_RH_DESC_A, 0x02001001);
  rtgalaxy_reg_writel(RTGALAXY_USB_OHCI_RH_STATUS, 0x00008001);
  rtgalaxy_reg_writel(RTGALAXY_USB_EHCI_PORTSC_0, 0x00000000);
  mdelay(10);
  rtgalaxy_reg_writel(RTGALAXY_USB_EHCI_PORTSC_0, 0x00001000);
}
