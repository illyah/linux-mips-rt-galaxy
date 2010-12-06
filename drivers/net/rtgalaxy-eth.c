/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DRV_NAME		"rtgalaxy-eth"
#define DRV_VERSION		"0.1"
#define DRV_RELDATE		"Dec 5, 2010"

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/compiler.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/gfp.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/cache.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include <rt-galaxy-io.h>
#include <rt-galaxy-soc.h>

/* VLAN tagging feature enable/disable */
#if defined(CONFIG_VLAN_8021Q) || defined(CONFIG_VLAN_8021Q_MODULE)
#define RGE_VLAN_TAG_USED 1
#define RGE_VLAN_TX_TAG(tx_desc,vlan_tag_value) \
	do { (tx_desc)->opts2 = vlan_tag_value; } while (0)
#else
#define RGE_VLAN_TAG_USED 0
#define RGE_VLAN_TX_TAG(tx_desc,vlan_tag_value) \
	do { (tx_desc)->opts2 = 0; } while (0)
#endif

/* These identify the driver base version and may not be removed. */
static char version[] =
    DRV_NAME ": 10/100 Ethernet driver v" DRV_VERSION " (" DRV_RELDATE ")\n";

MODULE_AUTHOR("shmprtd <shmprtd@googlemail.com>");
MODULE_DESCRIPTION("Realtek Galaxy SoC 10/100 Ethernet driver");
MODULE_VERSION(DRV_VERSION);
MODULE_LICENSE("GPL");

static int debug = -1;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "rtgalaxy-eth: bitmapped message enable number");

/* Maximum number of multicast addresses to filter (vs. Rx-all-multicast).
   The RTL chips use a 64 element hash table based on the Ethernet CRC.  */
static int multicast_filter_limit = 32;
module_param(multicast_filter_limit, int, 0);
MODULE_PARM_DESC(multicast_filter_limit,
		 "rtgalaxy-eth: maximum number of filtered multicast addresses");

#define RGE_DEF_MSG_ENABLE	(NETIF_MSG_DRV		| \
				 NETIF_MSG_PROBE 	| \
				 NETIF_MSG_LINK)
#define RGE_NUM_STATS		14	/* struct cp_dma_stats, plus one */
#define RGE_STATS_SIZE		64	/* size in bytes of DMA stats block */
#define RGE_REGS_SIZE		(0xff + 1)
#define RGE_REGS_VER		1	/* version 1 */
#define RGE_RX_RING_SIZE	64
#define RGE_TX_RING_SIZE	32

#define DESC_ALIGN		0x100

#define RGE_RXRING_BYTES	((sizeof(struct rge_desc) * (RGE_RX_RING_SIZE+1)) + DESC_ALIGN)
#define RGE_TXRING_BYTES	((sizeof(struct rge_desc) * (RGE_TX_RING_SIZE+1)) + DESC_ALIGN)

#define NEXT_TX(N)		(((N) + 1) & (RGE_TX_RING_SIZE - 1))
#define NEXT_RX(N)		(((N) + 1) & (RGE_RX_RING_SIZE - 1))
#define TX_BUFFS_AVAIL(PRIV)						\
	(((PRIV)->tx_tail <= (PRIV)->tx_head) ?				\
	  (PRIV)->tx_tail + (RGE_TX_RING_SIZE - 1) - (PRIV)->tx_head :	\
	  (PRIV)->tx_tail - (PRIV)->tx_head - 1)

#define SKB_PRIV_MAPPING		0
#define SKB_PRIV_FRAG			1
#define SKB_PRIVATE(skb,n)		((u32*)skb->cb)[n]

#define PKT_BUF_SZ		2048	/* Size of each temporary Rx buffer. */
#define RGE_INTERNAL_PHY	1

/* The following settings are log_2(bytes)-4:  0 == 16 bytes .. 6==1024, 7==end of packet. */
#define RX_FIFO_THRESH		5	/* Rx buffer level before first PCI xfer.  */
#define RX_DMA_BURST		4	/* Maximum PCI burst, '4' is 256 */
#define TX_DMA_BURST		6	/* Maximum PCI burst, '6' is 1024 */
#define TX_EARLY_THRESH		256	/* Early Tx threshold, in bytes */

/* Time in jiffies before concluding the transmitter is hung. */
#define TX_TIMEOUT		(6*HZ)

/* hardware minimum and maximum for a single frame's data payload */
#define RGE_MIN_MTU		60
#define RGE_MAX_MTU		4096

enum {
	MAC0 = 0x00,
	MAR0 = 0x08,
	StatTxOkCnt = 0x10,
	StatRxOkCnt = 0x12,
	StatTxError = 0x14,
	StatRxError = 0x16,
	StatRxMissed = 0x18,
	StatFae = 0x1a,
	StatTx1Col = 0x1c,
	StatTxMCol = 0x1e,
	StatRxOkPhy = 0x20,
	StatRxOkBroadcast = 0x22,
	StatRxOkMulticast = 0x24,
	StatTxAbort = 0x26,
	StatTxUnderrun = 0x28,
	TRSR = 0x34,
	Cmd = 0x3b,
	IntrMask = 0x3c,
	IntrStatus = 0x3e,
	TxConfig = 0x40,
	RxConfig = 0x44,
	MiiStatus = 0x58,
	MiiAccess = 0x5c,
	Tx1FDP = 0x0100,
	Tx1CDO = 0x0104,
	Tx1CPO = 0x0108,
	Tx1PSA = 0x010a,
	Tx1CPA = 0x010c,
	Tx1LastCDO = 0x0110,
	Tx1PageCnt = 0x0112,
	Tx1ScratchDes = 0x0150,
	Tx2FDP = 0x0180,
	Tx2CDO = 0x0184,
	Tx2CPO = 0x0188,
	Tx2PSA = 0x018a,
	Tx2CPA = 0x018c,
	Tx2LastCDO = 0x0190,
	Tx2PageCnt = 0x0192,
	Tx2ScratchDes = 0x01a0,
	RxFDP = 0x01f0,
	RxCDO = 0x01f4,
	RxRingSize = 0x01f6,
	RxCPO = 0x01f8,
	RxPSA = 0x01fa,
	RxCPA = 0x01fc,
	RxPLen = 0x0200,
	RxPFDP = 0x0204,
	RxPageCnt = 0x0208,
	RxScratchDes = 0x0210,
	RxCpuEthernetDesNum = 0x0230,
	RxCpuEthernetDesWrap = 0x0231,
	RxPseDesThres = 0x0232,
	IoCmd = 0x0234,

	/*
	 * IoCmd flags
	 */
	TxOwn = (1 << 5),
	RxOwn = (1 << 4),
	RxMii = (1 << 3),
	TxMii = (1 << 2),
	TxFnL = (1 << 1),
	TxFnH = (1 << 0),

	/*
	 * Cmd flags
	 */

	CmdChecksum = (1 << 1),
	CmdReset = (1 << 0),

	/*
	 * Phy flags
	 */

	TxFCE = (1 << 7),
	RxFCE = (1 << 6),
	ForceTx = (1 << 5),
	Speed100 = (1 << 3),
	Link = (1 << 2),
	TxPF = (1 << 1),
	RxPF = (1 << 0),

	/*
	 * Tx/Rx Status flags
	 */
	DescOwn = (1 << 31),	/* Descriptor is owned by NIC */
	RingEnd = (1 << 30),	/* End of descriptor ring */
	FirstFrag = (1 << 29),	/* First segment of a packet */
	LastFrag = (1 << 28),	/* Final segment of a packet */
	TxError = (1 << 23),	/* Tx error summary */
	RxError = (1 << 20),	/* Rx error summary */
	IPCS = (1 << 18),	/* Calculate IP checksum */
	UDPCS = (1 << 17),	/* Calculate UDP/IP checksum */
	TCPCS = (1 << 16),	/* Calculate TCP/IP checksum */
	TxVlanTag = (1 << 17),	/* Add VLAN tag */
	RxVlanTagged = (1 << 16),	/* Rx VLAN tag available */
	IPFail = (1 << 15),	/* IP checksum failed */
	UDPFail = (1 << 14),	/* UDP/IP checksum failed */
	TCPFail = (1 << 13),	/* TCP/IP checksum failed */
	NormalTxPoll = (1 << 6),	/* One or more normal Tx packets to send */
	PID1 = (1 << 17),	/* 2 protocol id bits:  0==non-IP, */
	PID0 = (1 << 16),	/* 1==UDP/IP, 2==TCP/IP, 3==IP */
	RxProtoTCP = 1,
	RxProtoUDP = 2,
	RxProtoIP = 3,
	TxFIFOUnder = (1 << 25),	/* Tx FIFO underrun */
	TxOWC = (1 << 22),	/* Tx Out-of-window collision */
	TxLinkFail = (1 << 21),	/* Link failed during Tx of packet */
	TxMaxCol = (1 << 20),	/* Tx aborted due to excessive collisions */
	TxColCntShift = 16,	/* Shift, to get 4-bit Tx collision cnt */
	TxColCntMask = 0x01 | 0x02 | 0x04 | 0x08,	/* 4-bit collision count */
	RxErrFrame = (1 << 27),	/* Rx frame alignment error */
	RxMcast = (1 << 26),	/* Rx multicast packet rcv'd */
	RxErrCRC = (1 << 18),	/* Rx CRC error */
	RxErrRunt = (1 << 19),	/* Rx error, packet < 64 bytes */
	RWT = (1 << 21),	/* Rx  */
	E8023 = (1 << 22),	/* Receive Ethernet 802.3 packet  */
	TxCRC = (1 << 23),

	RxVlanOn = (1 << 2),	/* Rx VLAN de-tagging enable */
	RxChkSum = (1 << 1),	/* Rx checksum offload enable */

	/*
	 * Threshold flags
	 */
	ThresholdVal = 16,
	RingSize16 = 0,
	RingSize32 = 1,
	RingSize64 = 2,
	Loopback = (3 << 8),

	AcceptErr = (1 << 5),
	AcceptRunt = (1 << 4),
	AcceptBroadcast = (1 << 3),
	AcceptMulticast = (1 << 2),
	AcceptMyPhys = (1 << 1),
	AcceptAllPhys = (1 << 0),

	AcceptAll =
	    AcceptBroadcast | AcceptMulticast | AcceptMyPhys | AcceptAllPhys |
	    AcceptErr | AcceptRunt,
	AcceptNoBroad =
	    AcceptMulticast | AcceptMyPhys | AcceptAllPhys | AcceptErr |
	    AcceptRunt,
	AcceptNoMulti = AcceptMyPhys | AcceptAllPhys | AcceptErr | AcceptRunt,
	NoErrAccept = AcceptBroadcast | AcceptMulticast | AcceptMyPhys,
	NoErrPromiscAccept =
	    AcceptBroadcast | AcceptMulticast | AcceptMyPhys | AcceptAllPhys,

	/*
	 * IntrStatus flags
	 */
	RuntErr = (1 << 19),
	SwInt = (1 << 10),
	TxEmpty = (1 << 9),
	LinkChg = (1 << 8),
	TxErr = (1 << 7),
	TxOK = (1 << 6),
	RxEmpty = (1 << 5),
	RxFIFOOvr = (1 << 4),
	RxErr = (1 << 2),
	RxOK = (1 << 0),

	/*
	 * IoCmd flags
	 */
#ifdef CONFIG_RTGALAXY_ETH_RX_MIT
	RxMIT = 4,
#else
	RxMIT = 1,
#endif
	RxTimer = 1,
	RxFifo = 2,
	TxFifo = 1,
	TxMIT = 4,
	TxPoll = (1 << 0),
	CmdConfig =
	    0x3c | (RxMIT << 8) | (RxFifo << 11) | (RxTimer << 13) | (TxMIT <<
								      16) |
	    (TxFifo << 19),

	/*
	 * MiiAccess flags
	 */
	MiiRegDone = (1 << 31),
	MiiRegWrite = (1 << 31),
	MiiRegRead = (0 << 31),
	MiiPhyAddrShift = 26,
	MiiRegAddrShift = 16,
	MiiDataShift = 0,

	rge_norx_intr_mask = LinkChg | TxOK | TxErr | TxEmpty,
	rge_rx_intr_mask = LinkChg | RxOK | RxErr | RxEmpty | RxFIFOOvr,
	rge_intr_mask = rge_rx_intr_mask | rge_norx_intr_mask,
};

struct rge_desc {
	u32 opts1;
	u32 addr;
	u32 opts2;
	u32 opts3;
};

struct rge_extra_stats {
	unsigned long rx_frags;
};

struct rge_private {
	struct resource *resource;
	int irq;
	void __iomem *base;

	void __iomem *regs;
	struct net_device *dev;
	spinlock_t lock;
	u32 msg_enable;

	struct napi_struct napi;
	struct platform_device *pdev;

	u32 rx_config;
	u32 rx_offset;
	u16 rgecmd;

	struct rge_extra_stats rge_stats;

	unsigned rx_head ____cacheline_aligned;
	unsigned rx_tail;
	struct rge_desc *rx_ring;
	struct sk_buff *rx_skb[RGE_RX_RING_SIZE];

	unsigned tx_head ____cacheline_aligned;
	unsigned tx_tail;
	struct rge_desc *tx_ring;
	struct sk_buff *tx_skb[RGE_TX_RING_SIZE];

	unsigned char *rxdesc_buf;
	dma_addr_t ring_dma;

	unsigned rx_buf_sz;
#if RGE_VLAN_TAG_USED
	struct vlan_group *vlgrp;
#endif

	struct mii_if_info mii_if;
};

#define rger8(reg)	rtgalaxy_readb(priv->regs + (reg))
#define rger16(reg)	rtgalaxy_readw(priv->regs + (reg))
#define rger32(reg)	rtgalaxy_readl(priv->regs + (reg))
#define rgew8(reg,val)	rtgalaxy_writeb((val), priv->regs + (reg))
#define rgew16(reg,val)	rtgalaxy_writew((val), priv->regs + (reg))
#define rgew32(reg,val)	rtgalaxy_writel((val), priv->regs + (reg))
#define rgew8_f(reg,val) do {				\
	rtgalaxy_writeb((val), priv->regs + (reg));	\
	rtgalaxy_readb(priv->regs + (reg));		\
	} while (0)
#define rgew16_f(reg,val) do {				\
	rtgalaxy_writew((val), priv->regs + (reg));	\
	rtgalaxy_readw(priv->regs + (reg));		\
	} while (0)
#define rgew32_f(reg,val) do {				\
	rtgalaxy_writel((val), priv->regs + (reg));	\
	rtgalaxy_readl(priv->regs + (reg));		\
	} while (0)

static void __rge_set_rx_mode(struct net_device *dev);
static void rge_tx(struct rge_private *priv);
static void rge_clean_rings(struct rge_private *priv);
#ifdef CONFIG_NET_POLL_CONTROLLER
static void rge_poll_controller(struct net_device *dev);
#endif
static void rge_init_rings_index(struct rge_private *priv);
static int rge_init_rings(struct rge_private *priv);
static inline void rge_start_hw(struct rge_private *priv);
static int rge_mdio_read(struct net_device *dev, int phy_id, int location);
static void rge_mdio_write(struct net_device *dev, int phy_id, int location,
			   int value);

static struct {
	const char str[ETH_GSTRING_LEN];
} ethtool_stats_keys[] = {
	{
	"tx_ok"}, {
	"rx_ok"}, {
	"tx_err"}, {
	"rx_err"}, {
	"rx_fifo"}, {
	"frame_align"}, {
	"tx_ok_1col"}, {
	"tx_ok_mcol"}, {
	"rx_ok_phys"}, {
	"rx_ok_bcast"}, {
	"rx_ok_mcast"}, {
	"tx_abort"}, {
	"tx_underrun"}, {
"rx_frags"},};

#if RGE_VLAN_TAG_USED
static void rge_vlan_rx_register(struct net_device *dev, struct vlan_group *grp)
{
	struct rge_private *priv = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	priv->vlgrp = grp;
	if (grp)
		priv->rgecmd |= RxVlanOn;
	else
		priv->rgecmd &= ~RxVlanOn;

	rgew16(CpCmd, priv->rgecmd);
	spin_unlock_irqrestore(&priv->lock, flags);
}
#endif /* RGE_VLAN_TAG_USED */

static inline void rge_set_rxbufsize(struct rge_private *priv)
{
	unsigned int mtu = priv->dev->mtu;

	if (mtu > ETH_DATA_LEN)
		/* MTU + ethernet header + FCS + optional VLAN tag */
		priv->rx_buf_sz = mtu + ETH_HLEN + 8;
	else
		priv->rx_buf_sz = PKT_BUF_SZ;
}

static inline void rge_rx_skb(struct rge_private *priv, struct sk_buff *skb,
			      struct rge_desc *desc)
{
	skb->dev = priv->dev;
	skb->protocol = eth_type_trans(skb, priv->dev);

	priv->dev->stats.rx_packets++;
	priv->dev->stats.rx_bytes += skb->len;

#if RGE_VLAN_TAG_USED
	if (priv->vlgrp && (desc->opts2 & RxVlanTagged)) {
		vlan_hwaccel_receive_skb(skb, priv->vlgrp,
					 b16_to_cpu(desc->opts2 & 0xffff));
	} else
#endif
		netif_receive_skb(skb);
}

static void rge_rx_err_acct(struct rge_private *priv, unsigned rx_tail,
			    u32 status, u32 len)
{
	netif_dbg(priv, rx_err, priv->dev,
		  "rx err, slot %d status 0x%x len %d\n", rx_tail, status, len);
	priv->dev->stats.rx_errors++;
	if (status & RxErrFrame)
		priv->dev->stats.rx_frame_errors++;
	if (status & RxErrCRC)
		priv->dev->stats.rx_crc_errors++;
	if (status & RxErrRunt)
		priv->dev->stats.rx_length_errors++;
	if ((status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag))
		priv->dev->stats.rx_length_errors++;
}

static inline unsigned int rge_rx_csum_ok(u32 status)
{
	unsigned int protocol = (status >> 16) & 0x3;

	if (!protocol ||
	    ((protocol == RxProtoTCP) && !(status & TCPFail)) ||
	    ((protocol == RxProtoUDP) && !(status & UDPFail)) ||
	    ((protocol == RxProtoIP) && !(status & IPFail)))
		return 1;
	else
		return 0;
}

static int rge_rx_poll(struct napi_struct *napi, int budget)
{
	struct rge_private *priv = container_of(napi, struct rge_private, napi);
	struct net_device *dev = priv->dev;
	unsigned int rx_tail = priv->rx_tail;
	int rx;

rx_status_loop:
	rx = 0;
	rgew16(IntrStatus, rge_rx_intr_mask);

	while (1) {
		u32 status, len;
		dma_addr_t mapping;
		struct sk_buff *skb, *new_skb;
		struct rge_desc *desc;
		const unsigned buflen = priv->rx_buf_sz + priv->rx_offset;

		skb = priv->rx_skb[rx_tail];
		BUG_ON(!skb);

		desc = &priv->rx_ring[rx_tail];

		status = desc->opts1;
		if (status & DescOwn)
			break;

		len = (status & 0x0fff) - 4;
		mapping = SKB_PRIVATE(skb, SKB_PRIV_MAPPING);

		if ((status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag)) {
			/* we don't support incoming fragmented frames.
			 * instead, we attempt to ensure that the
			 * pre-allocated RX skbs are properly sized such
			 * that RX fragments are never encountered
			 */

			rge_rx_err_acct(priv, rx_tail, status, len);
			dev->stats.rx_dropped++;
			priv->rge_stats.rx_frags++;
			goto rx_next;
		}

		if (status & RxError) {
			rge_rx_err_acct(priv, rx_tail, status, len);
			goto rx_next;
		}

		netif_dbg(priv, rx_status, dev,
			  "rx slot %d status 0x%x len %d\n", rx_tail, status,
			  len);

		dma_cache_inv((u32) skb->tail, len);

		new_skb = netdev_alloc_skb(dev, buflen);
		if (!new_skb) {
			dev->stats.rx_dropped++;
			goto rx_next;
		}

		if ((u32) new_skb->data & 0x3) {
			netif_err(priv, rx_status, dev,
				  "rx slot %d data misalignment 0x%x\n",
				  rx_tail, (u32) new_skb->data);
			dev->stats.rx_dropped++;
			goto rx_next;
		}

		skb_reserve(new_skb, priv->rx_offset);
		new_skb->dev = priv->dev;

		skb->ip_summed = CHECKSUM_UNNECESSARY;
		skb_put(skb, len);

		dma_cache_inv((u32) new_skb->tail, priv->rx_buf_sz);

		mapping = CPHYSADDR(new_skb->tail);
		SKB_PRIVATE(new_skb, SKB_PRIV_MAPPING) = mapping;

		priv->rx_skb[rx_tail] = new_skb;

		rge_rx_skb(priv, skb, desc);
		rx++;

rx_next:
		priv->rx_ring[rx_tail].opts2 = 0;
		priv->rx_ring[rx_tail].addr = mapping;

		if (rx_tail == (RGE_RX_RING_SIZE - 1))
			desc->opts1 = DescOwn | RingEnd | priv->rx_buf_sz;
		else
			desc->opts1 = DescOwn | priv->rx_buf_sz;
		rx_tail = NEXT_RX(rx_tail);

		if (rx >= budget)
			break;
	}

	priv->rx_tail = rx_tail;

	/* if we did not reach work limit, then we're done with
	 * this round of polling
	 */
	if (rx < budget) {
		unsigned long flags;

		if (rger16(IntrStatus) & rge_rx_intr_mask)
			goto rx_status_loop;

		spin_lock_irqsave(&priv->lock, flags);
		__napi_complete(napi);
		rgew16_f(IntrMask, rge_intr_mask);
		spin_unlock_irqrestore(&priv->lock, flags);
	}

	return rx;
}

static irqreturn_t rge_interrupt(int irq, void *dev_instance)
{
	struct net_device *dev = dev_instance;
	struct rge_private *priv;
	u16 status;

	if (unlikely(dev == NULL))
		return IRQ_NONE;
	priv = netdev_priv(dev);

	status = rger16(IntrStatus);
	if (!status || (status == 0xFFFF))
		return IRQ_NONE;

	netif_dbg(priv, intr, dev, "intr, status %04x cmd %02x\n",
		  status, rger8(Cmd));

	rgew16(IntrStatus, status & ~rge_rx_intr_mask);

	spin_lock(&priv->lock);

	/* close possible race's with dev_close */
	if (unlikely(!netif_running(dev))) {
		rgew16(IntrMask, 0);
		spin_unlock(&priv->lock);
		return IRQ_HANDLED;
	}

	if (status & RxFIFOOvr) {
		local_irq_disable();

		rgew16_f(IntrMask, 0);

		rgew32(IoCmd, 0);
		rgew16_f(IntrStatus, 0xffff);
		rge_clean_rings(priv);
		rge_init_rings(priv);

		rgew8(RxCpuEthernetDesNum, RGE_RX_RING_SIZE - 1);
		rgew16(RxCDO, 0);

		rgew16_f(IntrMask, rge_intr_mask);
		rge_start_hw(priv);

		local_irq_enable();

		return IRQ_HANDLED;
	}

	if (status & (RxOK | RxErr | RxEmpty)) {
		if (napi_schedule_prep(&priv->napi)) {
			rgew16_f(IntrMask, rge_norx_intr_mask);
			__napi_schedule(&priv->napi);
		}
	}

	if (status & (TxOK | TxErr | TxEmpty | SwInt)) {
		rge_tx(priv);
	}

	if (status & LinkChg) {
		mii_check_media(&priv->mii_if, netif_msg_link(priv), false);
		rgew16(IntrStatus, LinkChg);
	}

	spin_unlock(&priv->lock);

	return IRQ_HANDLED;
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling receive - used by netconsole and other diagnostic tools
 * to allow network i/o with interrupts disabled.
 */
static void rge_poll_controller(struct net_device *dev)
{
	disable_irq(dev->irq);
	rge_interrupt(dev->irq, dev);
	enable_irq(dev->irq);
}
#endif

static void rge_tx(struct rge_private *priv)
{
	unsigned tx_head = priv->tx_head;
	unsigned tx_tail = priv->tx_tail;

	while (tx_tail != tx_head) {
		struct rge_desc *txd = priv->tx_ring + tx_tail;
		struct sk_buff *skb;
		u32 status;

		rmb();
		status = txd->opts1;
		if (status & DescOwn)
			break;

		skb = priv->tx_skb[tx_tail];
		BUG_ON(!skb);

		if (status & LastFrag) {
			if (status & (TxError | TxFIFOUnder)) {
				netif_dbg(priv, tx_err, priv->dev,
					  "tx err, status 0x%x\n", status);
				priv->dev->stats.tx_errors++;
				if (status & TxOWC)
					priv->dev->stats.tx_window_errors++;
				if (status & TxMaxCol)
					priv->dev->stats.tx_aborted_errors++;
				if (status & TxLinkFail)
					priv->dev->stats.tx_carrier_errors++;
				if (status & TxFIFOUnder)
					priv->dev->stats.tx_fifo_errors++;
			} else {
				priv->dev->stats.collisions +=
				    ((status >> TxColCntShift) & TxColCntMask);
				priv->dev->stats.tx_packets++;
				priv->dev->stats.tx_bytes += skb->len;
				netif_dbg(priv, tx_done, priv->dev,
					  "tx done, slot %d\n", tx_tail);
			}
			dev_kfree_skb_irq(skb);
		}

		priv->tx_skb[tx_tail] = NULL;

		tx_tail = NEXT_TX(tx_tail);
	}

	priv->tx_tail = tx_tail;

	if (TX_BUFFS_AVAIL(priv) > (MAX_SKB_FRAGS + 1))
		netif_wake_queue(priv->dev);
}

static netdev_tx_t rge_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct rge_private *priv = netdev_priv(dev);
	unsigned entry;
	u32 eor;
	unsigned long intr_flags;
#if RGE_VLAN_TAG_USED
	u32 vlan_tag = 0;
#endif

	spin_lock_irqsave(&priv->lock, intr_flags);

	/* This is a hard error, log it. */
	if (TX_BUFFS_AVAIL(priv) <= (skb_shinfo(skb)->nr_frags + 1)) {
		netif_stop_queue(dev);
		spin_unlock_irqrestore(&priv->lock, intr_flags);
		netdev_err(dev, "BUG! Tx Ring full when queue awake!\n");
		return NETDEV_TX_BUSY;
	}
#if RGE_VLAN_TAG_USED
	if (vlan_tx_tag_present(skb))
		vlan_tag = TxVlanTag | vlan_tx_tag_get(skb);
#endif

	dma_cache_wback((u32) skb->data, skb->len);

	entry = priv->tx_head;
	eor = (entry == (RGE_TX_RING_SIZE - 1)) ? RingEnd : 0;

	if (skb_shinfo(skb)->nr_frags == 0) {
		struct rge_desc *txd = &priv->tx_ring[entry];
		u32 len;
		dma_addr_t mapping;

		len = skb->len;
		mapping = CPHYSADDR(skb->data);
		RGE_VLAN_TX_TAG(txd, vlan_tag);
		txd->addr = mapping;

		wmb();
		txd->opts1 = eor | len | DescOwn | FirstFrag | LastFrag | TxCRC;
		wmb();

		SKB_PRIVATE(skb, SKB_PRIV_MAPPING) = mapping;
		SKB_PRIVATE(skb, SKB_PRIV_FRAG) = 0;
		priv->tx_skb[entry] = skb;
		entry = NEXT_TX(entry);
	} else {
		struct rge_desc *txd;
		u32 first_len, first_eor;
		dma_addr_t first_mapping;
		int frag, first_entry = entry;

		/* We must give this initial chunk to the device last.
		 * Otherwise we could race with the device.
		 */
		first_eor = eor;
		first_len = skb_headlen(skb);
		first_mapping = CPHYSADDR(skb->data);
		SKB_PRIVATE(skb, SKB_PRIV_MAPPING) = first_mapping;
		SKB_PRIVATE(skb, SKB_PRIV_FRAG) = 1;
		priv->tx_skb[entry] = skb;
		entry = NEXT_TX(entry);

		for (frag = 0; frag < skb_shinfo(skb)->nr_frags; frag++) {
			skb_frag_t *this_frag = &skb_shinfo(skb)->frags[frag];
			u32 len;
			u32 ctrl;
			dma_addr_t mapping;

			len = this_frag->size;
			mapping = CPHYSADDR(this_frag->page_offset);
			eor = (entry == (RGE_TX_RING_SIZE - 1)) ? RingEnd : 0;

			ctrl = eor | len | DescOwn | TxCRC;

			if (frag == skb_shinfo(skb)->nr_frags - 1)
				ctrl |= LastFrag;

			txd = &priv->tx_ring[entry];
			RGE_VLAN_TX_TAG(txd, vlan_tag);
			txd->addr = mapping;
			wmb();

			txd->opts1 = ctrl;
			wmb();

			SKB_PRIVATE(skb, SKB_PRIV_MAPPING) = mapping;
			SKB_PRIVATE(skb, SKB_PRIV_FRAG) += 2;
			priv->tx_skb[entry] = skb;
			entry = NEXT_TX(entry);
		}

		txd = &priv->tx_ring[first_entry];
		RGE_VLAN_TX_TAG(txd, vlan_tag);
		txd->addr = first_mapping;
		wmb();

		eor = (entry == (RGE_TX_RING_SIZE - 1)) ? RingEnd : 0;
		txd->opts1 = eor | first_len | FirstFrag | DescOwn | TxCRC;
		wmb();
	}
	priv->tx_head = entry;
	netif_dbg(priv, tx_queued, priv->dev, "tx queued, slot %d, skblen %d\n",
		  entry, skb->len);
	if (TX_BUFFS_AVAIL(priv) <= (MAX_SKB_FRAGS + 1))
		netif_stop_queue(dev);

	spin_unlock_irqrestore(&priv->lock, intr_flags);

	rgew32(IoCmd, CmdConfig | TxPoll);
	dev->trans_start = jiffies;

	return NETDEV_TX_OK;
}

/* Set or clear the multicast filter for this adaptor.
   This routine is not state sensitive and need not be SMP locked. */

static void __rge_set_rx_mode(struct net_device *dev)
{
	struct rge_private *priv = netdev_priv(dev);
	u32 mc_filter[2];	/* Multicast hash filter */
	int rx_mode;
	u32 tmp;

	/* Note: do not reorder, GCC is clever about common statements. */
	if (dev->flags & IFF_PROMISC) {
		/* Unconditionally log net taps. */
		rx_mode =
		    AcceptBroadcast | AcceptMulticast | AcceptMyPhys |
		    AcceptAllPhys;
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else if ((netdev_mc_count(dev) > multicast_filter_limit) ||
		   (dev->flags & IFF_ALLMULTI)) {
		/* Too many to filter perfectly -- accept all multicasts. */
		rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
		mc_filter[1] = mc_filter[0] = 0xffffffff;
	} else {
		struct netdev_hw_addr *ha;
		rx_mode = AcceptBroadcast | AcceptMyPhys;
		mc_filter[1] = mc_filter[0] = 0;
		netdev_for_each_mc_addr(ha, dev) {
			int bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 26;

			mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
			rx_mode |= AcceptMulticast;
		}
	}

	/* We can safely update without stopping the chip. */
	tmp = priv->rx_config | rx_mode;
	if (priv->rx_config != tmp) {
		rgew32_f(RxConfig, tmp);
		priv->rx_config = tmp;
	}
	rgew32_f(MAR0 + 0, __cpu_to_be32(mc_filter[0]));
	rgew32_f(MAR0 + 4, __cpu_to_be32(mc_filter[1]));
}

static void rge_set_rx_mode(struct net_device *dev)
{
	unsigned long flags;
	struct rge_private *priv = netdev_priv(dev);

	spin_lock_irqsave(&priv->lock, flags);
	__rge_set_rx_mode(dev);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static void __rge_get_stats(struct rge_private *priv)
{
	priv->dev->stats.rx_missed_errors += rger16(StatRxMissed);
	rgew16(StatRxMissed, 0);
}

static struct net_device_stats *rge_get_stats(struct net_device *dev)
{
	struct rge_private *priv = netdev_priv(dev);
	unsigned long flags;

	/* The chip only need report frame silently dropped. */
	spin_lock_irqsave(&priv->lock, flags);
	if (netif_running(dev) && netif_device_present(dev))
		__rge_get_stats(priv);
	spin_unlock_irqrestore(&priv->lock, flags);

	return &dev->stats;
}

static void rge_reset_hw(struct rge_private *priv)
{
	unsigned work = 1000;

	rgew8(Cmd, CmdReset);

	while (work--) {
		if (!(rger8(Cmd) & CmdReset)) {
			rgew8(Cmd, CmdChecksum);
			return;
		}
		schedule_timeout_uninterruptible(10);
	}

	netdev_err(priv->dev, "hardware reset timeout\n");
}

static void rge_stop_hw(struct rge_private *priv)
{
	rgew16_f(IntrStatus, 0xffff);
	rgew16_f(IntrMask, 0);
	rgew32(IoCmd, 0);

	rge_reset_hw(priv);
	rgew16_f(IntrStatus, 0xffff);

	priv->rx_config = 0;
	priv->rx_tail = 0;
	priv->tx_head = priv->tx_tail = 0;
}

static inline void rge_start_hw(struct rge_private *priv)
{
	rgew32(IoCmd, CmdConfig);
}

static void rge_init_hw(struct rge_private *priv)
{
	struct net_device *dev = priv->dev;
	int phy_id = priv->mii_if.phy_id;
	u8 status, timeout;
	u32 *hwaddr;

	timeout = 10;
	while (timeout--
	       && ((rge_mdio_read(dev, phy_id, 1) & 0xffdb) != 0x7849)) {
		mdelay(100);
	}

	if (timeout < 0) {
		netdev_err(dev, "Timeout waiting for MiiAccess\n");
		return;
	}

	rge_reset_hw(priv);

	rge_mdio_write(dev, phy_id, 0, 0x8000);
	rge_mdio_write(dev, phy_id, 0, 0x1000);
	rge_mdio_write(dev, phy_id, 4, 0x05e1);

	rgew16(IntrStatus, 0xffff);
	rgew16(IntrMask, rge_intr_mask);

	rgew32(RxFDP, (u32) priv->rx_ring);
	rgew16(RxCDO, 0);

	rgew32(Tx1FDP, (u32) priv->tx_ring);
	rgew16(Tx1CDO, 0);

	rgew32(Tx2FDP, 0);
	rgew16(Tx2CDO, 0);

	rgew32(TxConfig, 0x00000c00);
	rgew8(RxPseDesThres, ThresholdVal);
	rgew8(RxCpuEthernetDesNum, RGE_RX_RING_SIZE - 1);
	rgew8(RxRingSize, RingSize64);

	status = rger8(MiiStatus);
	status |= TxFCE | RxFCE | ForceTx;
	rgew8(MiiStatus, status);

	hwaddr = (u32 *) (dev->dev_addr + 0);
	rgew32(MAC0 + 0, __cpu_to_be32(hwaddr[0]));
	rgew32(MAC0 + 4, __cpu_to_be32(hwaddr[1]));

	rge_start_hw(priv);
	__rge_set_rx_mode(dev);
}

static int rge_refill_rx(struct rge_private *priv)
{
	struct net_device *dev = priv->dev;
	unsigned i;

	for (i = 0; i < RGE_RX_RING_SIZE; i++) {
		struct sk_buff *skb;

		skb = netdev_alloc_skb(dev, priv->rx_buf_sz + priv->rx_offset);
		if (!skb)
			goto err_out;

		skb->dev = dev;
		skb_reserve(skb, priv->rx_offset);

		dma_cache_inv((u32) skb->data, priv->rx_buf_sz);
		SKB_PRIVATE(skb, SKB_PRIV_MAPPING) = CPHYSADDR(skb->data);
		SKB_PRIVATE(skb, SKB_PRIV_FRAG) = 0;
		priv->rx_skb[i] = skb;

		priv->rx_ring[i].opts2 = 0;
		priv->rx_ring[i].addr = SKB_PRIVATE(skb, SKB_PRIV_MAPPING);
		priv->rx_ring[i].opts1 = DescOwn | priv->rx_buf_sz;
		if (i == (RGE_RX_RING_SIZE - 1))
			priv->rx_ring[i].opts1 |= RingEnd;
	}

	return 0;

err_out:
	rge_clean_rings(priv);
	return -ENOMEM;
}

static void rge_init_rings_index(struct rge_private *priv)
{
	priv->rx_tail = 0;
	priv->tx_head = priv->tx_tail = 0;
}

static int rge_init_rings(struct rge_private *priv)
{
	memset(priv->tx_ring, 0, sizeof(struct rge_desc) * RGE_TX_RING_SIZE);
	priv->tx_ring[RGE_TX_RING_SIZE - 1].opts1 = RingEnd;

	memset(priv->rx_ring, 0, sizeof(struct rge_desc) * RGE_RX_RING_SIZE);

	rge_init_rings_index(priv);

	return rge_refill_rx(priv);
}

static int rge_alloc_rings(struct rge_private *priv)
{
	void *mem;

	mem = dma_alloc_coherent(&priv->pdev->dev,
				 RGE_RXRING_BYTES + RGE_TXRING_BYTES,
				 &priv->ring_dma, GFP_ATOMIC);

	priv->rxdesc_buf = NULL;
	if (!mem) {
		netdev_err(priv->dev, "Cannot allocate dma memory\n");
		return -ENOMEM;
	}

	priv->rxdesc_buf = mem;
	memset(mem, 0, RGE_RXRING_BYTES + RGE_TXRING_BYTES);

	mem = (void *)((u32) (mem + DESC_ALIGN - 1) & ~(DESC_ALIGN - 1));

	priv->rx_ring = (struct rge_desc *)KSEG1ADDR(mem);
	priv->tx_ring = &priv->rx_ring[RGE_RX_RING_SIZE];

	return rge_init_rings(priv);
}

static void rge_clean_rings(struct rge_private *priv)
{
	unsigned i;

	memset(priv->rx_ring, 0, sizeof(struct rge_desc) * RGE_RX_RING_SIZE);
	memset(priv->tx_ring, 0, sizeof(struct rge_desc) * RGE_TX_RING_SIZE);

	for (i = 0; i < RGE_RX_RING_SIZE; i++) {
		if (priv->rx_skb[i]) {
			dev_kfree_skb(priv->rx_skb[i]);
		}
	}

	for (i = 0; i < RGE_TX_RING_SIZE; i++) {
		if (priv->tx_skb[i]) {
			struct sk_buff *skb = priv->tx_skb[i];
			dev_kfree_skb(skb);
			priv->dev->stats.tx_dropped++;
		}
	}

	memset(priv->rx_skb, 0, sizeof(struct sk_buff *) * RGE_RX_RING_SIZE);
	memset(priv->tx_skb, 0, sizeof(struct sk_buff *) * RGE_TX_RING_SIZE);
}

static void rge_free_rings(struct rge_private *priv)
{
	rge_clean_rings(priv);

	if (priv->rxdesc_buf) {
		dma_free_coherent(&priv->pdev->dev,
				  RGE_RXRING_BYTES + RGE_TXRING_BYTES,
				  priv->rxdesc_buf, priv->ring_dma);
	}

	priv->rxdesc_buf = NULL;
	priv->rx_ring = NULL;
	priv->tx_ring = NULL;
}

static int rge_open(struct net_device *dev)
{
	struct rge_private *priv = netdev_priv(dev);
	int rc;

	netif_dbg(priv, ifup, dev, "enabling interface\n");

	rc = rge_alloc_rings(priv);
	if (rc)
		return rc;

	napi_enable(&priv->napi);

	rge_init_hw(priv);

	rc = request_irq(dev->irq, rge_interrupt, IRQF_SHARED, dev->name, dev);
	if (rc)
		goto err_out_hw;

	netif_carrier_off(dev);
	mii_check_media(&priv->mii_if, netif_msg_link(priv), true);
	netif_start_queue(dev);

	return 0;

err_out_hw:
	napi_disable(&priv->napi);
	rge_stop_hw(priv);
	rge_free_rings(priv);
	return rc;
}

static int rge_close(struct net_device *dev)
{
	struct rge_private *priv = netdev_priv(dev);
	unsigned long flags;

	napi_disable(&priv->napi);

	netif_dbg(priv, ifdown, dev, "disabling interface\n");

	spin_lock_irqsave(&priv->lock, flags);

	netif_stop_queue(dev);
	netif_carrier_off(dev);

	rge_stop_hw(priv);

	spin_unlock_irqrestore(&priv->lock, flags);

	free_irq(dev->irq, dev);

	rge_free_rings(priv);
	return 0;
}

static void rge_tx_timeout(struct net_device *dev)
{
	struct rge_private *priv = netdev_priv(dev);
	unsigned long flags;
	int rc;

	netdev_warn(dev, "Transmit timeout, status %2x %4x %4x\n",
		    rger8(Cmd), rger16(IntrStatus), rger16(IntrMask));

	spin_lock_irqsave(&priv->lock, flags);

	rge_stop_hw(priv);
	rge_clean_rings(priv);
	rc = rge_init_rings(priv);
	rge_start_hw(priv);

	netif_wake_queue(dev);

	spin_unlock_irqrestore(&priv->lock, flags);
}

#ifdef BROKEN
static int rge_change_mtu(struct net_device *dev, int new_mtu)
{
	struct rge_private *priv = netdev_priv(dev);
	int rc;
	unsigned long flags;

	/* check for invalid MTU, according to hardware limits */
	if (new_mtu < RGE_MIN_MTU || new_mtu > RGE_MAX_MTU)
		return -EINVAL;

	/* if network interface not up, no need for complexity */
	if (!netif_running(dev)) {
		dev->mtu = new_mtu;
		rge_set_rxbufsize(priv);	/* set new rx buf size */
		return 0;
	}

	spin_lock_irqsave(&priv->lock, flags);

	rge_stop_hw(priv);	/* stop h/w and free rings */
	rge_clean_rings(priv);

	dev->mtu = new_mtu;
	rge_set_rxbufsize(priv);	/* set new rx buf size */

	rc = rge_init_rings(priv);	/* realloc and restart h/w */
	rge_start_hw(priv);

	spin_unlock_irqrestore(&priv->lock, flags);

	return rc;
}
#endif /* BROKEN */

static char mii_to_rge_map[8] = {
	1,			/* BasicModeCtrl */
	1,			/* BasicModeStatus */
	0,
	0,
	1,			/* NWayAdvert */
	1,			/* NWayLPAR */
	1,			/* NWayExpansion */
	0
};

static int rge_mdio_read(struct net_device *dev, int phy_id, int location)
{
	struct rge_private *priv = netdev_priv(dev);

	if (location < 8 && mii_to_rge_map[location]) {
		rgew32(MiiAccess,
		       MiiRegRead + (phy_id << MiiPhyAddrShift) +
		       (location << MiiRegAddrShift));
		while ((rger32(MiiAccess) & MiiRegDone) == 0)
			mdelay(10);
		return rger32(MiiAccess) & 0xffff;
	}
	return 0;
}

static void rge_mdio_write(struct net_device *dev, int phy_id, int location,
			   int value)
{
	struct rge_private *priv = netdev_priv(dev);

	if (location < 8 && mii_to_rge_map[location]) {
		rgew32(MiiAccess, MiiRegWrite +
		       (phy_id << MiiRegAddrShift) +
		       (location << MiiRegAddrShift) + (value << MiiDataShift));
		while ((rger32(MiiAccess) & MiiRegDone) == 0)
			mdelay(10);
	}
}

static void rge_get_drvinfo(struct net_device *dev,
			    struct ethtool_drvinfo *info)
{
	strcpy(info->driver, DRV_NAME);
	strcpy(info->version, DRV_VERSION);
}

static int rge_get_regs_len(struct net_device *dev)
{
	return RGE_REGS_SIZE;
}

static int rge_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return RGE_NUM_STATS;
	default:
		return -EOPNOTSUPP;
	}
}

static int rge_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct rge_private *priv = netdev_priv(dev);
	int rc;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	rc = mii_ethtool_gset(&priv->mii_if, cmd);
	spin_unlock_irqrestore(&priv->lock, flags);

	return rc;
}

static int rge_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct rge_private *priv = netdev_priv(dev);
	int rc;
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	rc = mii_ethtool_sset(&priv->mii_if, cmd);
	spin_unlock_irqrestore(&priv->lock, flags);

	return rc;
}

static int rge_nway_reset(struct net_device *dev)
{
	struct rge_private *priv = netdev_priv(dev);
	return mii_nway_restart(&priv->mii_if);
}

static u32 rge_get_msglevel(struct net_device *dev)
{
	struct rge_private *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void rge_set_msglevel(struct net_device *dev, u32 value)
{
	struct rge_private *priv = netdev_priv(dev);
	priv->msg_enable = value;
}

static u32 rge_get_rx_csum(struct net_device *dev)
{
	struct rge_private *priv = netdev_priv(dev);
	return (rger16(Cmd) & RxChkSum) ? 1 : 0;
}

static int rge_set_rx_csum(struct net_device *dev, u32 data)
{
	struct rge_private *priv = netdev_priv(dev);
	u16 cmd = priv->rgecmd, newcmd;

	newcmd = cmd;

	if (data)
		newcmd |= RxChkSum;
	else
		newcmd &= ~RxChkSum;

	if (newcmd != cmd) {
		unsigned long flags;

		spin_lock_irqsave(&priv->lock, flags);
		priv->rgecmd = newcmd;
		rgew16_f(Cmd, newcmd);
		spin_unlock_irqrestore(&priv->lock, flags);
	}

	return 0;
}

static void rge_get_regs(struct net_device *dev, struct ethtool_regs *regs,
			 void *p)
{
	struct rge_private *priv = netdev_priv(dev);
	unsigned long flags;

	if (regs->len < RGE_REGS_SIZE)
		return /* -EINVAL */ ;

	regs->version = RGE_REGS_VER;

	spin_lock_irqsave(&priv->lock, flags);
	memcpy(p, priv->regs, RGE_REGS_SIZE);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static void rge_get_strings(struct net_device *dev, u32 stringset, u8 * buf)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(buf, &ethtool_stats_keys, sizeof(ethtool_stats_keys));
		break;
	default:
		BUG();
		break;
	}
}

static void rge_get_ethtool_stats(struct net_device *dev,
				  struct ethtool_stats *estats, u64 * tmp_stats)
{
	struct rge_private *priv = netdev_priv(dev);
	int i;

	i = 0;
	tmp_stats[i++] = rger16(StatTxOkCnt);
	tmp_stats[i++] = rger16(StatRxOkCnt);
	tmp_stats[i++] = rger16(StatTxError);
	tmp_stats[i++] = rger16(StatRxError);
	tmp_stats[i++] = rger16(StatRxMissed);
	tmp_stats[i++] = rger16(StatFae);
	tmp_stats[i++] = rger16(StatTx1Col);
	tmp_stats[i++] = rger16(StatTxMCol);
	tmp_stats[i++] = rger16(StatRxOkPhy);
	tmp_stats[i++] = rger16(StatRxOkBroadcast);
	tmp_stats[i++] = rger16(StatRxOkMulticast);
	tmp_stats[i++] = rger16(StatTxAbort);
	tmp_stats[i++] = rger16(StatTxUnderrun);
	tmp_stats[i++] = priv->rge_stats.rx_frags;
	BUG_ON(i != RGE_NUM_STATS);
}

static const struct ethtool_ops rge_ethtool_ops = {
	.get_drvinfo = rge_get_drvinfo,
	.get_regs_len = rge_get_regs_len,
	.get_sset_count = rge_get_sset_count,
	.get_settings = rge_get_settings,
	.set_settings = rge_set_settings,
	.nway_reset = rge_nway_reset,
	.get_link = ethtool_op_get_link,
	.get_msglevel = rge_get_msglevel,
	.set_msglevel = rge_set_msglevel,
	.get_rx_csum = rge_get_rx_csum,
	.set_rx_csum = rge_set_rx_csum,
	.set_tx_csum = ethtool_op_set_tx_csum,
	.set_sg = ethtool_op_set_sg,
	.set_tso = ethtool_op_set_tso,
	.get_regs = rge_get_regs,
	.get_strings = rge_get_strings,
	.get_ethtool_stats = rge_get_ethtool_stats,
};

static int rge_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct rge_private *priv = netdev_priv(dev);
	int rc;
	unsigned long flags;

	if (!netif_running(dev))
		return -EINVAL;

	spin_lock_irqsave(&priv->lock, flags);
	rc = generic_mii_ioctl(&priv->mii_if, if_mii(rq), cmd, NULL);
	spin_unlock_irqrestore(&priv->lock, flags);
	return rc;
}

static int rge_set_mac_address(struct net_device *dev, void *p)
{
	struct rge_private *priv = netdev_priv(dev);
	struct sockaddr *addr = p;
	u32 *hwaddr;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	memcpy(dev->dev_addr, addr->sa_data, 6);

	spin_lock_irq(&priv->lock);

	hwaddr = (u32 *) (dev->dev_addr + 0);
	rgew32(MAC0 + 0, __cpu_to_be32(hwaddr[0]));
	rgew32(MAC0 + 4, __cpu_to_be32(hwaddr[1]));

	spin_unlock_irq(&priv->lock);

	return 0;
}

static const struct net_device_ops rge_netdev_ops = {
	.ndo_open = rge_open,
	.ndo_stop = rge_close,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = rge_set_mac_address,
	.ndo_set_multicast_list = rge_set_rx_mode,
	.ndo_get_stats = rge_get_stats,
	.ndo_do_ioctl = rge_ioctl,
	.ndo_start_xmit = rge_start_xmit,
	.ndo_tx_timeout = rge_tx_timeout,

#if RGE_VLAN_TAG_USED
	.ndo_vlan_rx_register = rge_vlan_rx_register,
#endif

#ifdef BROKEN
	.ndo_change_mtu = rge_change_mtu,
#endif

#ifdef CONFIG_NET_POLL_CONTROLLER
	.ndo_poll_controller = rge_poll_controller,
#endif
};

static int __init rge_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct rge_private *priv;
	int err;

	pr_info("%s", version);

	dev = alloc_etherdev(sizeof(struct rge_private));
	if (!dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, dev);
	priv = netdev_priv(dev);
	priv->pdev = pdev;
	priv->dev = dev;
	spin_lock_init(&priv->lock);
	priv->mii_if.dev = dev;
	priv->mii_if.mdio_read = rge_mdio_read;
	priv->mii_if.mdio_write = rge_mdio_write;
	priv->mii_if.phy_id = RGE_INTERNAL_PHY;
	priv->mii_if.phy_id_mask = 0x1f;
	priv->mii_if.reg_num_mask = 0x1f;
	rge_set_rxbufsize(priv);

	priv->msg_enable = (debug < 0 ? RGE_DEF_MSG_ENABLE : debug);
	priv->rx_offset = 2;

	err = -ENODEV;
	if (rtgalaxy_is_mars_soc()) {
		priv->rx_offset = 0;

		if ((rtgalaxy_soc_readl(RTGALAXY_SOC_CLOCK_ENABLE) &
		     RTGALAXY_SOC_CLOCK_ENABLE_ETH) == 0) {
			netdev_err(dev,
				   "Realtek Mars ethernet clock disabled\n");
			goto err_clock;
		}
	}

	err = -ENXIO;
	priv->resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!priv->resource) {
		netdev_err(dev, "No io memory resource defined\n");
		goto err_resource;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0) {
		netdev_err(dev, "No irq resource defined\n");
		goto err_resource;
	}
	dev->irq = priv->irq;

	err = -ENOMEM;
	priv->base = ioremap(priv->resource->start,
			     resource_size(priv->resource));
	if (!priv->base) {
		netdev_err(dev, "Unable to map io memory\n");
		goto err_map;
	}
	dev->base_addr = (unsigned long)priv->base;
	priv->regs = priv->base;

	rge_stop_hw(priv);

	/* platform_data contains pointer to mac address passed from bootloader */
	memcpy(dev->dev_addr, pdev->dev.platform_data, 6);
	dev->dev_addr[6] = dev->dev_addr[7] = 0;

	dev->netdev_ops = &rge_netdev_ops;
	netif_napi_add(dev, &priv->napi, rge_rx_poll, 8);
	dev->ethtool_ops = &rge_ethtool_ops;
	dev->watchdog_timeo = TX_TIMEOUT;
	dev->features |= NETIF_F_HIGHDMA;

#if RGE_VLAN_TAG_USED
	dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
#endif

	err = register_netdev(dev);
	if (err)
		goto err_register;

	netdev_info(dev, "Realtek Galaxy at 0x%lx, %pM, IRQ %d\n",
		    dev->base_addr, dev->dev_addr, dev->irq);

	return 0;

err_register:
	iounmap(priv->base);
err_resource:
err_map:
err_clock:
	free_netdev(dev);
	return err;
}

static int __exit rge_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct rge_private *priv = netdev_priv(dev);

	iounmap(priv->base);
	priv->base = NULL;
	unregister_netdev(dev);
	flush_scheduled_work();
	free_netdev(dev);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int rge_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct net_device *dev = platform_get_drvdata(pdev);

	if (!netif_running(dev))
		return 0;

	// rge_close(dev); 

	return 0;
}

static int rge_resume(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);

	if (!netif_running(dev))
		return 0;

	// rge_open(dev); 

	return 0;
}

#else
#define rge_suspend	NULL
#define rge_resume	NULL
#endif

static struct platform_driver rge_driver = {
	.probe = rge_probe,
	.remove = rge_remove,
	.suspend = rge_suspend,
	.resume = rge_resume,
	.driver = {
		   .name = "rtgalaxy-eth",
		   },
};

static int __init rge_init(void)
{
	if (soc_is_rtgalaxy())
		return platform_driver_register(&rge_driver);

	return -ENODEV;
}

static void __exit rge_exit(void)
{
	platform_driver_unregister(&rge_driver);
}

module_init(rge_init);
module_exit(rge_exit);
