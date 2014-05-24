/* RealtekRTL8100Linux-102400.c -- Code shared with the linux driver code.
 *
 * Copyright (c) 2014 Laura Müller <laura-mueller@uni-duesseldorf.de>
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Driver for Realtek RTL8100x PCIe fast ethernet controllers.
 *
 * This driver is based on Realtek's r8101 Linux driver (1.024.00).
 */

#include "RealtekRTL8100.h"

/*
 ################################################################################
 #
 # r8101 is the Linux device driver released for Realtek Fast Ethernet
 # controllers with PCI-Express interface.
 #
 # Copyright(c) 2013 Realtek Semiconductor Corp. All rights reserved.
 #
 # This program is free software; you can redistribute it and/or modify it
 # under the terms of the GNU General Public License as published by the Free
 # Software Foundation; either version 2 of the License, or (at your option)
 # any later version.
 #
 # This program is distributed in the hope that it will be useful, but WITHOUT
 # ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 # FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 # more details.
 #
 # You should have received a copy of the GNU General Public License along with
 # this program; if not, see <http://www.gnu.org/licenses/>.
 #
 # Author:
 # Realtek NIC software team <nicfae@realtek.com>
 # No. 2, Innovation Road II, Hsinchu Science Park, Hsinchu 300, Taiwan
 #
 ################################################################################
 */

/************************************************************************************
 *  This product is covered by one or more of the following patents:
 *  US6,570,884, US6,115,776, and US6,327,625.
 ***********************************************************************************/

/*
 This driver is modified from r8169.c in Linux kernel 2.6.18
 */

#if DISABLED_CODE

#include <linux/module.h>
#include <linux/version.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/if_vlan.h>
#include <linux/crc32.h>
#include <linux/interrupt.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/init.h>
#include <linux/rtnetlink.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#endif//LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>

#include "r8101.h"
#include "rtl_eeprom.h"
#include "rtl_ethtool.h"
#include "rtltool.h"

static int eee_enable = 0 ;
module_param(eee_enable, int, S_IRUGO);

#endif /* DISABLED_CODE */

/* Maximum events (Rx packets, etc.) to handle at each interrupt. */
static const int max_interrupt_work = 20;

/* Maximum number of multicast addresses to filter (vs. Rx-all-multicast).
 The RTL chips use a 64 element hash table based on the Ethernet CRC. */
static const int multicast_filter_limit = 32;

#define _R(NAME,MAC,MASK) \
{ .name = NAME, .mcfg = MAC, .RxConfigMask = MASK }

const struct RTLChipInfo rtl_chip_info[] = {
    _R("RTL8101E", CFG_METHOD_1, 0xff7e1880),
    _R("RTL8101E", CFG_METHOD_2, 0xff7e1880),
    _R("RTL8101E", CFG_METHOD_3, 0xff7e1880),
    _R("RTL8102E", CFG_METHOD_4, 0xff7e1880),
    _R("RTL8102E", CFG_METHOD_5, 0xff7e1880),
    _R("RTL8103E", CFG_METHOD_6, 0xff7e1880),
    _R("RTL8103E", CFG_METHOD_7, 0xff7e1880),
    _R("RTL8103E", CFG_METHOD_8, 0xff7e1880),
    _R("RTL8401E", CFG_METHOD_9, 0xff7e1880),
    _R("RTL8105E", CFG_METHOD_10, 0xff7e1880),
    _R("RTL8105E", CFG_METHOD_11, 0xff7e1880),
    _R("RTL8105E", CFG_METHOD_12, 0xff7e1880),
    _R("RTL8105E", CFG_METHOD_13, 0xff7e1880),
    _R("RTL8402", CFG_METHOD_14, 0xff7e1880),
    _R("RTL8106E", CFG_METHOD_15, 0xff7e1880),
    _R("RTL8106E", CFG_METHOD_16, 0xff7e1880),
    _R("RTL8106EUS", CFG_METHOD_17, 0xff7e1880)
};
#undef _R

#if DISABLED_CODE

static struct pci_device_id rtl8101_pci_tbl[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_REALTEK,	0x8136), },
    {0,},
};

MODULE_DEVICE_TABLE(pci, rtl8101_pci_tbl);

static int rx_copybreak = 200;
static int timer_count = 0x2600;
static int use_dac;
static struct {
    u32 msg_enable;
} debug = { -1 };

static unsigned short speed = SPEED_100;
static int duplex = DUPLEX_FULL;
static int autoneg = AUTONEG_ENABLE;
#ifdef CONFIG_ASPM
static int aspm = 1;
#else
static int aspm = 0;
#endif
#ifdef ENABLE_S5WOL
static int s5wol = 1;
#else
static int s5wol = 0;
#endif


MODULE_AUTHOR("Realtek and the Linux r8101 crew <netdev@vger.kernel.org>");
MODULE_DESCRIPTION("RealTek RTL-8101 Fast Ethernet driver");

module_param(speed, ushort, 0);
MODULE_PARM_DESC(speed, "force phy operation. Deprecated by ethtool (8).");

module_param(duplex, int, 0);
MODULE_PARM_DESC(duplex, "force phy operation. Deprecated by ethtool (8).");

module_param(autoneg, int, 0);
MODULE_PARM_DESC(autoneg, "force phy operation. Deprecated by ethtool (8).");

module_param(aspm, int, 0);
MODULE_PARM_DESC(aspm, "Enable ASPM.");

module_param(s5wol, int, 0);
MODULE_PARM_DESC(s5wol, "Enable Shutdown Wake On Lan.");

module_param(rx_copybreak, int, 0);
MODULE_PARM_DESC(rx_copybreak, "Copy breakpoint for copy-only-tiny-frames");

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
module_param_named(debug, debug.msg_enable, int, 0);
MODULE_PARM_DESC(debug, "Debug verbosity level (0=none, ..., 16=all)");
#endif//LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)

MODULE_LICENSE("GPL");

MODULE_VERSION(RTL8101_VERSION);

static void rtl8101_dsm(struct net_device *dev, int dev_state);

static void rtl8101_esd_timer(unsigned long __opaque);

static void rtl8101_hw_phy_config(struct net_device *dev);

static void rtl8101_wait_for_quiescence(struct net_device *dev);

static void rtl8101_tx_clear(struct rtl8101_private *tp);
static void rtl8101_rx_clear(struct rtl8101_private *tp);

static void rtl8101_link_timer(unsigned long __opaque);
static void rtl8101_aspm_fix1(struct net_device *dev);

static int rtl8101_open(struct net_device *dev);
static int rtl8101_start_xmit(struct sk_buff *skb, struct net_device *dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t rtl8101_interrupt(int irq, void *dev_instance, struct pt_regs *regs);
#else
static irqreturn_t rtl8101_interrupt(int irq, void *dev_instance);
#endif
static int rtl8101_init_ring(struct net_device *dev);
static void rtl8101_hw_start(struct net_device *dev);
static int rtl8101_close(struct net_device *dev);
static void rtl8101_set_rx_mode(struct net_device *dev);
static void rtl8101_tx_timeout(struct net_device *dev);
static struct net_device_stats *rtl8101_get_stats(struct net_device *dev);
static int rtl8101_rx_interrupt(struct net_device *, struct rtl8101_private *, void __iomem *, u32 budget);
static int rtl8101_change_mtu(struct net_device *dev, int new_mtu);
static void rtl8101_down(struct net_device *dev);

static int rtl8101_set_speed(struct net_device *dev, u8 autoneg, u16 speed, u8 duplex);
static int rtl8101_set_mac_address(struct net_device *dev, void *p);
void rtl8101_rar_set(struct rtl8101_private *tp, uint8_t *addr);
static void rtl8101_desc_addr_fill(struct rtl8101_private *);
static void rtl8101_tx_desc_init(struct rtl8101_private *tp);
static void rtl8101_rx_desc_init(struct rtl8101_private *tp);

static void rtl8101_hw_reset(struct net_device *dev);

static void rtl8101_phy_power_down (struct net_device *dev);

#endif  /* DISABLED_CODE */

static void rtl8101_phy_power_up (struct net_device *dev);

#if DISABLED_CODE

#ifdef CONFIG_R8101_NAPI
static int rtl8101_poll(napi_ptr napi, napi_budget budget);
#endif

static const unsigned int rtl8101_rx_config =
(Reserved2_data << Reserved2_shift) | (RX_DMA_BURST << RxCfgDMAShift);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,3)
/* copied from linux kernel 2.6.20 include/linux/netdev.h */
#define	NETDEV_ALIGN		32
#define	NETDEV_ALIGN_CONST	(NETDEV_ALIGN - 1)

static inline void *netdev_priv(struct net_device *dev)
{
    return (char *)dev + ((sizeof(struct net_device)
                           + NETDEV_ALIGN_CONST)
                          & ~NETDEV_ALIGN_CONST);
}
#endif	//LINUX_VERSION_CODE < KERNEL_VERSION(2,6,3)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)
/* copied from linux kernel 2.6.20 include/linux/netdevice.h */
static inline u32 netif_msg_init(int debug_value, int default_msg_enable_bits)
{
    /* use default */
    if (debug_value < 0 || debug_value >= (sizeof(u32) * 8))
        return default_msg_enable_bits;
    if (debug_value == 0)	/* no output */
        return 0;
    /* set low N bits */
    return (1 << debug_value) - 1;
}

#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)
static inline void eth_copy_and_sum (struct sk_buff *dest,
                                     const unsigned char *src,
                                     int len, int base)
{
    memcpy (dest->data, src, len);
}
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)
/* copied from linux kernel 2.6.20 /include/linux/time.h */
/* Parameters used to convert the timespec values: */
#define MSEC_PER_SEC	1000L

/* copied from linux kernel 2.6.20 /include/linux/jiffies.h */
/*
 * Change timeval to jiffies, trying to avoid the
 * most obvious overflows..
 *
 * And some not so obvious.
 *
 * Note that we don't want to return MAX_LONG, because
 * for various timeout reasons we often end up having
 * to wait "jiffies+1" in order to guarantee that we wait
 * at _least_ "jiffies" - so "jiffies+1" had better still
 * be positive.
 */
#define MAX_JIFFY_OFFSET ((~0UL >> 1)-1)

/*
 * Convert jiffies to milliseconds and back.
 *
 * Avoid unnecessary multiplications/divisions in the
 * two most common HZ cases:
 */
static inline unsigned int jiffies_to_msecs(const unsigned long j)
{
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
    return (MSEC_PER_SEC / HZ) * j;
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
    return (j + (HZ / MSEC_PER_SEC) - 1)/(HZ / MSEC_PER_SEC);
#else
    return (j * MSEC_PER_SEC) / HZ;
#endif
}

static inline unsigned long msecs_to_jiffies(const unsigned int m)
{
    if (m > jiffies_to_msecs(MAX_JIFFY_OFFSET))
        return MAX_JIFFY_OFFSET;
#if HZ <= MSEC_PER_SEC && !(MSEC_PER_SEC % HZ)
    return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
#elif HZ > MSEC_PER_SEC && !(HZ % MSEC_PER_SEC)
    return m * (HZ / MSEC_PER_SEC);
#else
    return (m * HZ + MSEC_PER_SEC - 1) / MSEC_PER_SEC;
#endif
}
#endif	//LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
//for linux kernel 2.6.10 and earlier.

/* copied from linux kernel 2.6.12.6 /include/linux/pm.h */
typedef int __bitwise pci_power_t;

/* copied from linux kernel 2.6.12.6 /include/linux/pci.h */
typedef u32 __bitwise pm_message_t;

#define PCI_D0	((pci_power_t __force) 0)
#define PCI_D1	((pci_power_t __force) 1)
#define PCI_D2	((pci_power_t __force) 2)
#define PCI_D3hot	((pci_power_t __force) 3)
#define PCI_D3cold	((pci_power_t __force) 4)
#define PCI_POWER_ERROR	((pci_power_t __force) -1)

/* copied from linux kernel 2.6.12.6 /drivers/pci/pci.c */
/**
 * pci_choose_state - Choose the power state of a PCI device
 * @dev: PCI device to be suspended
 * @state: target sleep state for the whole system. This is the value
 *	that is passed to suspend() function.
 *
 * Returns PCI power state suitable for given device and given system
 * message.
 */

pci_power_t pci_choose_state(struct pci_dev *dev, pm_message_t state)
{
    if (!pci_find_capability(dev, PCI_CAP_ID_PM))
        return PCI_D0;
    
    switch (state) {
        case 0:
            return PCI_D0;
        case 3:
            return PCI_D3hot;
        default:
            printk("They asked me for state %d\n", state);
            //		BUG();
    }
    return PCI_D0;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
//+porting on 2.6.8.1 and earlier
/**
 * msleep_interruptible - sleep waiting for waitqueue interruptions
 * @msecs: Time in milliseconds to sleep for
 */
unsigned long msleep_interruptible(unsigned int msecs)
{
    unsigned long timeout = msecs_to_jiffies(msecs);
    
    while (timeout && !signal_pending(current)) {
        set_current_state(TASK_INTERRUPTIBLE);
        timeout = schedule_timeout(timeout);
    }
    return jiffies_to_msecs(timeout);
}

/* copied from linux kernel 2.6.20 include/linux/mii.h */
#undef if_mii
#define if_mii _kc_if_mii
static inline struct mii_ioctl_data *if_mii(struct ifreq *rq) {
    return (struct mii_ioctl_data *) &rq->ifr_ifru;
}
#endif	//LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)

#endif  /* DISABLED_CODE */

static inline u16 map_phy_ocp_addr(u16 PageNum, u8 RegNum)
{
    u16 OcpPageNum = 0;
    u8 OcpRegNum = 0;
    u16 OcpPhyAddress = 0;
    
    if( PageNum == 0 ) {
        OcpPageNum = OCP_STD_PHY_BASE_PAGE + ( RegNum / 8 );
        OcpRegNum = 0x10 + ( RegNum % 8 );
    } else {
        OcpPageNum = PageNum;
        OcpRegNum = RegNum;
    }
    
    OcpPageNum <<= 4;
    
    if( OcpRegNum < 16 ) {
        OcpPhyAddress = 0;
    } else {
        OcpRegNum -= 16;
        OcpRegNum <<= 1;
        
        OcpPhyAddress = OcpPageNum + OcpRegNum;
    }
    
    
    return OcpPhyAddress;
}

void mdio_write(struct rtl8101_private *tp, u32 RegAddr, u32 value)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int i;
    
    if (RegAddr == 0x1F) {
        tp->cur_page = value;
    }
    
    if (tp->mcfg == CFG_METHOD_17) {
        u32 data32;
        u16 ocp_addr;
        
        if (RegAddr == 0x1F) {
            return;
        }
        ocp_addr = map_phy_ocp_addr(tp->cur_page, RegAddr);
        
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        WARN_ON_ONCE(ocp_addr % 2);
#endif
        data32 = ocp_addr/2;
        data32 <<= OCPR_Addr_Reg_shift;
        data32 |= OCPR_Write | value;
        
        RTL_W32(PHYOCP, data32);
        for (i = 0; i < 100; i++) {
            udelay(1);
            
            if (!(RTL_R32(PHYOCP) & OCPR_Flag))
                break;
        }
    } else {
        RTL_W32(PHYAR, PHYAR_Write |
                (RegAddr & PHYAR_Reg_Mask) << PHYAR_Reg_shift |
                (value & PHYAR_Data_Mask));
        
        for (i = 0; i < 10; i++) {
            udelay(100);
            
            /* Check if the RTL8101 has completed writing to the specified MII register */
            if (!(RTL_R32(PHYAR) & PHYAR_Flag)) {
                udelay(20);
                break;
            }
        }
    }
}

u32 mdio_read(struct rtl8101_private *tp, u32 RegAddr)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int i, value = 0;
    
    if (tp->mcfg == CFG_METHOD_17) {
        u32 data32;
        u16 ocp_addr;
        
        ocp_addr = map_phy_ocp_addr(tp->cur_page, RegAddr);
        
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
        WARN_ON_ONCE(ocp_addr % 2);
#endif
        data32 = ocp_addr/2;
        data32 <<= OCPR_Addr_Reg_shift;
        
        RTL_W32(PHYOCP, data32);
        for (i = 0; i < 100; i++) {
            udelay(1);
            
            if (RTL_R32(PHYOCP) & OCPR_Flag)
                break;
        }
        value = RTL_R32(PHYOCP) & OCPDR_Data_Mask;
    } else {
        RTL_W32(PHYAR,
                PHYAR_Read | (RegAddr & PHYAR_Reg_Mask) << PHYAR_Reg_shift);
        
        for (i = 0; i < 10; i++) {
            udelay(100);
            
            /* Check if the RTL8101 has completed retrieving data from the specified MII register */
            if (RTL_R32(PHYAR) & PHYAR_Flag) {
                value = RTL_R32(PHYAR) & PHYAR_Data_Mask;
                udelay(20);
                break;
            }
        }
    }
    
    return value;
}

void mac_ocp_write(struct rtl8101_private *tp, u16 reg_addr, u16 value)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 data32;
    
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
    WARN_ON_ONCE(reg_addr % 2);
#endif
    
    data32 = reg_addr/2;
    data32 <<= OCPR_Addr_Reg_shift;
    data32 += value;
    data32 |= OCPR_Write;
    
    RTL_W32(MACOCP, data32);
}

u16 mac_ocp_read(struct rtl8101_private *tp, u16 reg_addr)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 data32;
    u16 data16 = 0;
    
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,18)
    WARN_ON_ONCE(reg_addr % 2);
#endif
    
    data32 = reg_addr/2;
    data32 <<= OCPR_Addr_Reg_shift;
    
    RTL_W32(MACOCP, data32);
    data16 = (u16)RTL_R32(MACOCP);
    
    return data16;
}


static void
rtl8101_phyio_write(void __iomem *ioaddr,
                    int RegAddr,
                    int value)
{
    int i;
    
    RTL_W32(PHYIO, PHYIO_Write |
            (RegAddr & PHYIO_Reg_Mask) << PHYIO_Reg_shift |
            (value & PHYIO_Data_Mask));
    
    for (i = 0; i < 10; i++) {
        udelay(100);
        
        /* Check if the RTL8101 has completed writing to the specified MII register */
        if (!(RTL_R32(PHYIO) & PHYIO_Flag))
            break;
    }
    
    udelay(100);
}

#if 0
static int
rtl8101_phyio_read(void __iomem *ioaddr,
                   int RegAddr)
{
    int i, value = -1;
    
    RTL_W32(PHYIO,
            PHYIO_Read | (RegAddr & PHYIO_Reg_Mask) << PHYIO_Reg_shift);
    
    for (i = 0; i < 10; i++) {
        udelay(100);
        
        /* Check if the RTL8101 has completed retrieving data from the specified MII register */
        if (RTL_R32(PHYIO) & PHYIO_Flag) {
            value = (int) (RTL_R32(PHYIO) & PHYIO_Data_Mask);
            break;
        }
    }
    
    udelay(100);
    
    return value;
}
#endif

void rtl8101_ephy_write(void __iomem *ioaddr, u32 RegAddr, u32 value)
{
    int i;
    
    RTL_W32(EPHYAR,
            EPHYAR_Write |
            (RegAddr & EPHYAR_Reg_Mask) << EPHYAR_Reg_shift |
            (value & EPHYAR_Data_Mask));
    
    for (i = 0; i < 10; i++) {
        udelay(100);
        
        /* Check if the RTL8101 has completed EPHY write */
        if (!(RTL_R32(EPHYAR) & EPHYAR_Flag))
            break;
    }
    
    udelay(20);
}

u16 rtl8101_ephy_read(void __iomem *ioaddr, u32 RegAddr)
{
    int i;
    u16 value = 0xffff;
    
    RTL_W32(EPHYAR,
            EPHYAR_Read | (RegAddr & EPHYAR_Reg_Mask) << EPHYAR_Reg_shift);
    
    for (i = 0; i < 10; i++) {
        udelay(100);
        
        /* Check if the RTL8101 has completed EPHY read */
        if (RTL_R32(EPHYAR) & EPHYAR_Flag) {
            value = (u16) (RTL_R32(EPHYAR) & EPHYAR_Data_Mask);
            break;
        }
    }
    
    udelay(20);
    
    return value;
}

static void
rtl8101_csi_write(struct rtl8101_private *tp,
                  int addr,
                  int value)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 cmd;
    int i;
    
    RTL_W32(CSIDR, value);
    cmd = CSIAR_Write | CSIAR_ByteEn << CSIAR_ByteEn_shift | (addr & CSIAR_Addr_Mask);
    if (tp->mcfg == CFG_METHOD_14)
        cmd |= 0x00020000;
    RTL_W32(CSIAR, cmd);
    
    for (i = 0; i < 10; i++) {
        udelay(100);
        
        /* Check if the RTL8101 has completed CSI write */
        if (!(RTL_R32(CSIAR) & CSIAR_Flag))
            break;
    }
}

static int
rtl8101_csi_read(struct rtl8101_private *tp,
                 int addr)
{
    void __iomem *ioaddr = tp->mmio_addr;
    u32 cmd;
    int i, value = -1;
    
    cmd = CSIAR_Read | CSIAR_ByteEn << CSIAR_ByteEn_shift | (addr & CSIAR_Addr_Mask);
    
    if (tp->mcfg == CFG_METHOD_14)
        cmd |= 0x00020000;
    
    RTL_W32(CSIAR, cmd);
    
    for (i = 0; i < 10; i++) {
        udelay(100);
        
        /* Check if the RTL8101 has completed CSI read */
        if (RTL_R32(CSIAR) & CSIAR_Flag) {
            value = (int)RTL_R32(CSIDR);
            break;
        }
    }
    return value;
}

u32 rtl8101_eri_read(void __iomem *ioaddr, int addr, int len, int type)
{
    int i, val_shift, shift = 0;
    u32 value1 = 0, value2 = 0, mask;
    
    if (len > 4 || len <= 0)
        return -1;
    
    while (len > 0) {
        val_shift = addr % ERIAR_Addr_Align;
        addr = addr & ~0x3;
        
        RTL_W32(ERIAR,
                ERIAR_Read |
                type << ERIAR_Type_shift |
                ERIAR_ByteEn << ERIAR_ByteEn_shift |
                addr);
        
        for (i = 0; i < 10; i++) {
            udelay(100);
            
            /* Check if the RTL8101 has completed ERI read */
            if (RTL_R32(ERIAR) & ERIAR_Flag)
                break;
        }
        
        if (len == 1)		mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
        else if (len == 2)	mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        else if (len == 3)	mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        else			mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        
        value1 = RTL_R32(ERIDR) & mask;
        value2 |= (value1 >> val_shift * 8) << shift * 8;
        
        if (len <= 4 - val_shift)
            len = 0;
        else {
            len -= (4 - val_shift);
            shift = 4 - val_shift;
            addr += 4;
        }
    }
    
    return value2;
}

int rtl8101_eri_write(void __iomem *ioaddr, int addr, int len, u32 value, int type)
{
    
    int i, val_shift, shift = 0;
    u32 value1 = 0, mask;
    
    if (len > 4 || len <= 0)
        return -1;
    
    while (len > 0) {
        val_shift = addr % ERIAR_Addr_Align;
        addr = addr & ~0x3;
        
        if (len == 1)		mask = (0xFF << (val_shift * 8)) & 0xFFFFFFFF;
        else if (len == 2)	mask = (0xFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        else if (len == 3)	mask = (0xFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        else			mask = (0xFFFFFFFF << (val_shift * 8)) & 0xFFFFFFFF;
        
        value1 = rtl8101_eri_read(ioaddr, addr, 4, type) & ~mask;
        value1 |= ((value << val_shift * 8) >> shift * 8);
        
        RTL_W32(ERIDR, value1);
        RTL_W32(ERIAR,
                ERIAR_Write |
                type << ERIAR_Type_shift |
                ERIAR_ByteEn << ERIAR_ByteEn_shift |
                addr);
        
        for (i = 0; i < 10; i++) {
            udelay(100);
            
            /* Check if the RTL8101 has completed ERI write */
            if (!(RTL_R32(ERIAR) & ERIAR_Flag))
                break;
        }
        
        if (len <= 4 - val_shift)
            len = 0;
        else {
            len -= (4 - val_shift);
            shift = 4 - val_shift;
            addr += 4;
        }
    }
    
    return 0;
}

#if 1

static int rtl8101_enable_EEE(struct rtl8101_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int	ret;
    //unsigned long flags;
    __u16	data;
    
    ret = 0;
    switch (tp->mcfg) {
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
            rtl8101_eri_write(ioaddr, 0x1B0, 2, 0xED03, ERIAR_ExGMAC);
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1F, 0x0004);
            if (RTL_R8(0xEF) & 0x02) {
                mdio_write(tp, 0x10, 0x731F);
                mdio_write(tp, 0x19, 0x7630);
            } else {
                mdio_write(tp, 0x10, 0x711F);
                mdio_write(tp, 0x19, 0x7030);
            }
            mdio_write(tp, 0x1A, 0x1506);
            mdio_write(tp, 0x1B, 0x0551);
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0002);
            mdio_write(tp, 0x0D, 0x0000);
            
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0003);
            mdio_write(tp, 0x0E, 0x0015);
            mdio_write(tp, 0x0D, 0x4003);
            mdio_write(tp, 0x0E, 0x0002);
            mdio_write(tp, 0x0D, 0x0000);
            
            mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            break;
            
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
            rtl8101_eri_write(ioaddr, 0x1B0, 2, 0xED03, ERIAR_ExGMAC);
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1F, 0x0004);
            mdio_write(tp, 0x10, 0x731F);
            mdio_write(tp, 0x19, 0x7630);
            mdio_write(tp, 0x1A, 0x1506);
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0002);
            mdio_write(tp, 0x0D, 0x0000);
            
            mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            break;
            
        case CFG_METHOD_17:
            spin_lock_irqsave(&tp->phy_lock, flags);
            data = rtl8101_eri_read(ioaddr, 0x1B0, 4, ERIAR_ExGMAC);
            data |= BIT_1 | BIT_0;
            rtl8101_eri_write(ioaddr, 0x1B0, 4, data, ERIAR_ExGMAC);
            mdio_write(tp, 0x1F, 0x0A43);
            data = mdio_read(tp, 0x11);
            mdio_write(tp, 0x11, data | BIT_4);
            mdio_write(tp, 0x1F, 0x0A5D);
            mdio_write(tp, 0x10, 0x0006);
            mdio_write(tp, 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            break;
            
        default:
            //dev_printk(KERN_DEBUG, &tp->pci_dev->dev, "Not Support EEE\n");
            ret = -EOPNOTSUPP;
            break;
    }
    
    /*Advanced EEE*/
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            data = rtl8101_eri_read(ioaddr, 0x1EA, 1, ERIAR_ExGMAC);
            data |= 0xFA;
            rtl8101_eri_write(ioaddr, 0x1EA, 1, data, ERIAR_ExGMAC);
            
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1F, 0x0A43);
            data = mdio_read(tp, 0x10);
            if (data & BIT_10) {
                mdio_write(tp, 0x1F, 0x0A42);
                data = mdio_read(tp, 0x16);
                data &= ~(BIT_1);
                mdio_write(tp, 0x16, data);
            } else {
                mdio_write(tp, 0x1F, 0x0A42);
                data = mdio_read(tp, 0x16);
                data |= BIT_1;
                mdio_write(tp, 0x16, data);
            }
            mdio_write(tp, 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            break;
    }
    
    
    return ret;
}

int rtl8101_disable_EEE(struct rtl8101_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    int	ret;
    //unsigned long flags;
    __u16	data;
    
    ret = 0;
    switch (tp->mcfg) {
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
            rtl8101_eri_write(ioaddr, 0x1B0, 2, 0, ERIAR_ExGMAC);
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1F, 0x0004);
            mdio_write(tp, 0x10, 0x401F);
            mdio_write(tp, 0x19, 0x7030);
            
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0000);
            mdio_write(tp, 0x0D, 0x0000);
            
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0003);
            mdio_write(tp, 0x0E, 0x0015);
            mdio_write(tp, 0x0D, 0x4003);
            mdio_write(tp, 0x0E, 0x0000);
            mdio_write(tp, 0x0D, 0x0000);
            
            mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            break;
            
        case CFG_METHOD_14:
            rtl8101_eri_write(ioaddr, 0x1B0, 2, 0, ERIAR_ExGMAC);
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1F, 0x0004);
            mdio_write(tp, 0x10, 0x401F);
            mdio_write(tp, 0x19, 0x7030);
            
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0000);
            mdio_write(tp, 0x0D, 0x0000);
            
            mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            break;
            
        case CFG_METHOD_15:
        case CFG_METHOD_16:
            rtl8101_eri_write(ioaddr, 0x1B0, 2, 0, ERIAR_ExGMAC);
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1F, 0x0004);
            mdio_write(tp, 0x10, 0xC07F);
            mdio_write(tp, 0x19, 0x7030);
            mdio_write(tp, 0x1F, 0x0000);
            
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x0D, 0x0007);
            mdio_write(tp, 0x0E, 0x003C);
            mdio_write(tp, 0x0D, 0x4007);
            mdio_write(tp, 0x0E, 0x0000);
            mdio_write(tp, 0x0D, 0x0000);
            
            mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            break;
            
        case CFG_METHOD_17:
            spin_lock_irqsave(&tp->phy_lock, flags);
            data = rtl8101_eri_read(ioaddr, 0x1B0, 4, ERIAR_ExGMAC);
            data &= ~(BIT_1 | BIT_0);
            rtl8101_eri_write(ioaddr, 0x1B0, 4, data, ERIAR_ExGMAC);
            mdio_write(tp, 0x1F, 0x0A43);
            data = mdio_read(tp, 0x11);
            mdio_write(tp, 0x11, data & ~BIT_4);
            mdio_write(tp, 0x1F, 0x0A5D);
            mdio_write(tp, 0x10, 0x0000);
            mdio_write(tp, 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            break;
            
        default:
            //dev_printk(KERN_DEBUG, &tp->pci_dev->dev, "Not Support EEE\n");
            ret = -EOPNOTSUPP;
            break;
    }
    
    /*Advanced EEE*/
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            rtl8101_eri_write(ioaddr, 0x1EA, 1, 0x00, ERIAR_ExGMAC);
            
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1F, 0x0A42);
            data = mdio_read(tp, 0x16);
            data &= ~(BIT_1);
            mdio_write(tp, 0x16, data);
            mdio_write(tp, 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            break;
    }
    
    return ret;
}

#endif

static void
rtl8101_enable_rxdvgate(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            RTL_W8(0xF2, RTL_R8(0xF2) | BIT_3);
            mdelay(2);
            break;
    }
}

void rtl8101_disable_rxdvgate(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            RTL_W8(0xF2, RTL_R8(0xF2) & ~BIT_3);
            mdelay(2);
            break;
    }
}

static void
rtl8101_wait_txrx_fifo_empty(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i;
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            for (i = 0; i < 10; i++) {
                udelay(100);
                if (RTL_R32(TxConfig) & BIT_11)
                    break;
            }
            
            if(RTL_R8(ChipCmd) & (CmdTxEnb | CmdRxEnb)) {
                mdelay(1);
                RTL_W8(ChipCmd, RTL_R8(ChipCmd) & ~(CmdTxEnb | CmdRxEnb));
            }
            
            for (i = 0; i < 10; i++) {
                udelay(100);
                if ((RTL_R8(MCUCmd_reg) & (Txfifo_empty | Rxfifo_empty)) == (Txfifo_empty | Rxfifo_empty))
                    break;
                
            }
            break;
    }
}

#if DISABLED_CODE

static void
rtl8101_irq_mask_and_ack(void __iomem *ioaddr)
{
    RTL_W16(IntrMask, 0x0000);
    RTL_W16(IntrStatus, RTL_R16(IntrStatus));
}

#endif /* DISABLED_CODE */

void rtl8101_nic_reset(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i;
    
    RTL_W32(RxConfig, (RX_DMA_BURST << RxCfgDMAShift));
    
    rtl8101_enable_rxdvgate(dev);
    
    rtl8101_wait_txrx_fifo_empty(dev);
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            RTL_W8(ChipCmd, StopReq | CmdRxEnb | CmdTxEnb);
            udelay(100);
            break;
        case CFG_METHOD_14:
        case CFG_METHOD_17:
            mdelay(2);
            break;
        default:
            mdelay(10);
            break;
    }
    
    /* Soft reset the chip. */
    RTL_W8(ChipCmd, CmdReset);
    
    /* Check that the chip has finished the reset. */
    for (i = 100; i > 0; i--) {
        if ((RTL_R8(ChipCmd) & CmdReset) == 0)
            break;
        udelay(100);
    }
}

#if DISABLED_CODE

static void
rtl8101_hw_reset(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    
    /* Disable interrupts */
    rtl8101_irq_mask_and_ack(ioaddr);
    
    rtl8101_nic_reset(dev);
}

#endif /* DISABLED_CODE */

unsigned int rtl8101_xmii_reset_pending(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    //unsigned long flags;
    unsigned int retval;
    
    spin_lock_irqsave(&tp->phy_lock, flags);
    mdio_write(tp, 0x1f, 0x0000);
    retval = mdio_read(tp, MII_BMCR) & BMCR_RESET;
    spin_unlock_irqrestore(&tp->phy_lock, flags);
    
    return retval;
}

unsigned int rtl8101_xmii_link_ok(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned int retval;
    
    retval = (RTL_R8(PHYstatus) & LinkStatus) ? 1 : 0;
    
    return retval;
}

void rtl8101_xmii_reset_enable(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    //unsigned long flags;
    int i;
    
    spin_lock_irqsave(&tp->phy_lock, flags);
    mdio_write(tp, 0x1f, 0x0000);
    mdio_write(tp, MII_BMCR, mdio_read(tp, MII_BMCR) | BMCR_RESET);
    for (i = 0; i < 2500; i++) {
        if (!(mdio_read(tp, MII_BMSR) & BMCR_RESET))
            break;
        
        mdelay(1);
    }
    spin_unlock_irqrestore(&tp->phy_lock, flags);
}

void set_offset70F(struct rtl8101_private *tp, u8 setting)
{
    u32 csi_tmp;
    u32 temp = (u32)setting;
    temp = temp << 24;
    /*set PCI configuration space offset 0x70F to setting*/
    /*When the register offset of PCI configuration space larger than 0xff, use CSI to access it.*/
    
    csi_tmp = rtl8101_csi_read(tp, 0x70c) & 0x00ffffff;
    rtl8101_csi_write(tp, 0x70c, csi_tmp | temp);
}

#if DISABLED_CODE

static void
set_offset79(struct rtl8101_private *tp, u8 setting)
{
    //Set PCI configuration space offset 0x79 to setting
    
    struct pci_dev *pdev = tp->pci_dev;
    u8 device_control;
    
    pci_read_config_byte(pdev, 0x79, &device_control);
    device_control &= ~0x70;
    device_control |= setting;
    pci_write_config_byte(pdev, 0x79, device_control);
    
}

static void
rtl8101_init_ring_indexes(struct rtl8101_private *tp)
{
    tp->dirty_tx = 0;
    tp->dirty_rx = 0;
    tp->cur_tx = 0;
    tp->cur_rx = 0;
}

static void
rtl8101_check_link_status(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int link_status_on;
    u32 data32;
    unsigned long flags;
    
    link_status_on = tp->link_ok(dev);
    
    if (netif_carrier_ok(dev) != link_status_on) {
        
        if (link_status_on) {
            if (tp->mcfg == CFG_METHOD_5 || tp->mcfg == CFG_METHOD_6 ||
                tp->mcfg == CFG_METHOD_7 || tp->mcfg == CFG_METHOD_8)
                set_offset70F(tp, 0x3F);
            
            if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
                tp->mcfg == CFG_METHOD_13) {
                if ((RTL_R8(PHYstatus) & FullDup) == 0 && eee_enable == 1)
                    rtl8101_disable_EEE(tp);
                
                if (RTL_R8(PHYstatus) & _10bps) {
                    rtl8101_eri_write(ioaddr, 0x1D0, 2, 0x4D02, ERIAR_ExGMAC);
                    rtl8101_eri_write(ioaddr, 0x1DC, 2, 0x0060, ERIAR_ExGMAC);
                    
                    rtl8101_eri_write(ioaddr, 0x1B0, 2, 0, ERIAR_ExGMAC);
                    spin_lock_irqsave(&tp->phy_lock, flags);
                    mdio_write( tp, 0x1F, 0x0004);
                    data32 = mdio_read( tp, 0x10);
                    data32 |= 0x0400;
                    data32 &= ~0x0800;
                    mdio_write(tp, 0x10, data32);
                    mdio_write(tp, 0x1F, 0x0000);
                    spin_unlock_irqrestore(&tp->phy_lock, flags);
                } else {
                    rtl8101_eri_write(ioaddr, 0x1D0, 2, 0, ERIAR_ExGMAC);
                    if ( eee_enable == 1 && (RTL_R8(0xEF) & BIT_0) == 0)
                        rtl8101_eri_write(ioaddr, 0x1B0, 2, 0xED03, ERIAR_ExGMAC);
                }
            } else if (tp->mcfg == CFG_METHOD_14 || tp->mcfg == CFG_METHOD_15 ||
                       tp->mcfg == CFG_METHOD_16) {
                if (RTL_R8(PHYstatus) & _10bps) {
                    rtl8101_eri_write(ioaddr, 0x1D0, 2, 0x4d02, ERIAR_ExGMAC);
                    rtl8101_eri_write(ioaddr, 0x1DC, 2, 0x0060, ERIAR_ExGMAC);
                } else {
                    rtl8101_eri_write(ioaddr, 0x1D0, 2, 0, ERIAR_ExGMAC);
                }
            }
            
            rtl8101_hw_start(dev);
            
            netif_carrier_on(dev);
            
            if (netif_msg_ifup(tp))
                printk(KERN_INFO PFX "%s: link up\n", dev->name);
        } else {
            if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
                tp->mcfg == CFG_METHOD_13) {
                spin_lock_irqsave(&tp->phy_lock, flags);
                mdio_write( tp, 0x1F, 0x0004);
                data32 = mdio_read( tp, 0x10);
                data32 &= ~0x0C00;
                mdio_write(tp, 0x1F, 0x0000);
                spin_unlock_irqrestore(&tp->phy_lock, flags);
            }
            if (netif_msg_ifdown(tp))
                printk(KERN_INFO PFX "%s: link down\n", dev->name);
            netif_carrier_off(dev);
            
            netif_stop_queue(dev);
            
            rtl8101_hw_reset(dev);
            
            rtl8101_tx_clear(tp);
            
            rtl8101_init_ring_indexes(tp);
            
            rtl8101_set_speed(dev, tp->autoneg, tp->speed, tp->duplex);
            
            if (tp->mcfg == CFG_METHOD_5 || tp->mcfg == CFG_METHOD_6 ||
                tp->mcfg == CFG_METHOD_7 || tp->mcfg == CFG_METHOD_8)
                set_offset70F(tp, 0x17);
        }
    }
    switch (tp->mcfg) {
        case CFG_METHOD_4:
            rtl8101_aspm_fix1(dev);
            break;
    }
}

#endif /* DISABLED_CODE */

static void
rtl8101_wait_ll_share_fifo_ready(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i;
    
    for (i = 0; i < 10; i++) {
        udelay(100);
        if (RTL_R16(0xD2) & BIT_9)
            break;
    }
}

void rtl8101_hw_d3_para(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    //unsigned long flags;
    
    switch (tp->mcfg) {
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
            RTL_W8(Cfg9346, Cfg9346_Unlock);
            RTL_W8(Config5, RTL_R8(Config5) & ~BIT_0);
            RTL_W8(Config2, RTL_R8(Config2) & ~BIT_7);
            RTL_W8(Cfg9346, Cfg9346_Lock);
            RTL_W8(0xF1, RTL_R8(0xF1) & ~BIT_7);
            break;
    }
    
    if ((tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
         tp->mcfg == CFG_METHOD_13) && (eee_enable == 1))
        rtl8101_disable_EEE(tp);
    
    if (tp->mcfg == CFG_METHOD_17) {
        u32 csi_tmp;
        
        csi_tmp = rtl8101_eri_read(ioaddr, 0x3F2, 2, ERIAR_ExGMAC);
        csi_tmp &= ~(BIT_0 | BIT_1);
        rtl8101_eri_write(ioaddr, 0x3F2, 2, csi_tmp, ERIAR_ExGMAC);
        
        csi_tmp = rtl8101_eri_read(ioaddr, 0x1E2, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_2;
        rtl8101_eri_write(ioaddr, 0x1E2, 1, csi_tmp, ERIAR_ExGMAC);
        
        rtl8101_eri_write(ioaddr, 0x2F8, 2, 0x0064, ERIAR_ExGMAC);
        
        /*disable ocp phy power saving*/
        spin_lock_irqsave(&tp->phy_lock, flags);
        mdio_write(tp, 0x1F, 0x0C41);
        mdio_write(tp, 0x13, 0x0000);
        mdio_write(tp, 0x13, 0x0500);
        mdio_write(tp, 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }
    
    if (tp->bios_setting & BIT_28) {
        if (tp->mcfg == CFG_METHOD_13) {
            if (!(RTL_R8(0xEF) & BIT_2)) {
                u32 gphy_val;
                
                spin_lock_irqsave(&tp->phy_lock, flags);
                mdio_write(tp, 0x1F, 0x0000);
                mdio_write(tp, 0x04, 0x0061);
                mdio_write(tp, 0x00, 0x1200);
                mdio_write(tp, 0x18, 0x0310);
                mdelay(20);
                mdio_write(tp, 0x1F, 0x0005);
                gphy_val = mdio_read(tp, 0x1a);
                gphy_val |= BIT_8 | BIT_0;
                mdio_write(tp, 0x1a, gphy_val);
                mdelay(30);
                mdio_write(tp, 0x1f, 0x0000);
                mdio_write(tp, 0x18, 0x8310);
                spin_unlock_irqrestore(&tp->phy_lock, flags);
            }
        }
    }
    
    rtl8101_disable_rxdvgate(dev);
}

#if DISABLED_CODE

static void
rtl8101_powerdown_pll(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    
    if (tp->wol_enabled == WOL_ENABLED) {
        int auto_nego;
        u16 val;
        unsigned long flags;
        
        if (tp->mcfg == CFG_METHOD_17) {
            RTL_W8(Cfg9346, Cfg9346_Unlock);
            RTL_W8(Config2, RTL_R8(Config2) | PMSTS_En);
            RTL_W8(Cfg9346, Cfg9346_Lock);
        }
        spin_lock_irqsave(&tp->phy_lock, flags);
        mdio_write(tp, 0x1F, 0x0000);
        auto_nego = mdio_read(tp, MII_ADVERTISE);
        auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL
                       | ADVERTISE_100HALF | ADVERTISE_100FULL);
        
        val = mdio_read(tp, MII_LPA);
        
        if (val & (LPA_10HALF | LPA_10FULL))
            auto_nego |= (ADVERTISE_10HALF | ADVERTISE_10FULL);
        else
            auto_nego |= (ADVERTISE_100FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_10FULL);
        
        mdio_write(tp, MII_ADVERTISE, auto_nego);
        mdio_write(tp, MII_BMCR, BMCR_RESET | BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        
        switch (tp->mcfg) {
            case CFG_METHOD_1:
            case CFG_METHOD_2:
            case CFG_METHOD_3:
            case CFG_METHOD_4:
            case CFG_METHOD_5:
            case CFG_METHOD_6:
            case CFG_METHOD_7:
            case CFG_METHOD_8:
            case CFG_METHOD_9:
                break;
            default:
                RTL_W32(RxConfig, RTL_R32(RxConfig) | AcceptBroadcast | AcceptMulticast | AcceptMyPhys);
                break;
        }
        
        return;
    }
    
    rtl8101_phy_power_down(dev);
    
    switch (tp->mcfg) {
        case CFG_METHOD_6:
        case CFG_METHOD_9:
            RTL_W8(DBG_reg, RTL_R8(DBG_reg) | BIT_3);
            RTL_W8(PMCH, RTL_R8(PMCH) & ~BIT_7);
            break;
            
        case CFG_METHOD_8:
            pci_write_config_byte(tp->pci_dev, 0x81, 0);
            RTL_W8(PMCH, RTL_R8(PMCH) & ~BIT_7);
            break;
        case CFG_METHOD_7:
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
            RTL_W8(PMCH, RTL_R8(PMCH) & ~BIT_7);
            break;
        default:
            break;
    }
}

#endif  /* DISABLED_CODE */

void rtl8101_powerup_pll(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    
    switch (tp->mcfg) {
        case CFG_METHOD_6:
        case CFG_METHOD_9:
            RTL_W8(PMCH, RTL_R8(PMCH) | BIT_7);
            RTL_W8(DBG_reg, RTL_R8(DBG_reg) & ~BIT_3);
            break;
        case CFG_METHOD_7:
        case CFG_METHOD_8:
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
            RTL_W8(PMCH, RTL_R8(PMCH) | BIT_7);
            break;
    }
    
    rtl8101_phy_power_up(dev);
}

#if DISABLED_CODE

static void
rtl8101_link_option(int idx,
                    u8 *aut,
                    u16 *spd,
                    u8 *dup)
{
    if ((*spd != SPEED_100) && (*spd != SPEED_10))
        *spd = SPEED_100;
    
    if ((*dup != DUPLEX_FULL) && (*dup != DUPLEX_HALF))
        *dup = DUPLEX_FULL;
    
    if ((*aut != AUTONEG_ENABLE) && (*aut != AUTONEG_DISABLE))
        *aut = AUTONEG_ENABLE;
}

static void
rtl8101_get_wol(struct net_device *dev,
                struct ethtool_wolinfo *wol)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u8 options;
    u32 csi_tmp;
    //unsigned long flags;
    
    wol->wolopts = 0;
    
#define WAKE_ANY (WAKE_PHY | WAKE_MAGIC | WAKE_UCAST | WAKE_BCAST | WAKE_MCAST)
    wol->supported = WAKE_ANY;
    
    spin_lock_irqsave(&tp->lock, flags);
    
    options = RTL_R8(Config1);
    if (!(options & PMEnable))
        goto out_unlock;
    
    options = RTL_R8(Config3);
    if (options & LinkUp)
        wol->wolopts |= WAKE_PHY;
    
    switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_17:
            csi_tmp = rtl8101_eri_read(ioaddr, 0xDE, 1, ERIAR_ExGMAC);
            if (csi_tmp & BIT_0)
                wol->wolopts |= WAKE_MAGIC;
            break;
        default:
            if (options & MagicPacket)
                wol->wolopts |= WAKE_MAGIC;
            break;
    }
    
    options = RTL_R8(Config5);
    if (options & UWF)
        wol->wolopts |= WAKE_UCAST;
    if (options & BWF)
        wol->wolopts |= WAKE_BCAST;
    if (options & MWF)
        wol->wolopts |= WAKE_MCAST;
    
out_unlock:
    spin_unlock_irqrestore(&tp->lock, flags);
}

static int
rtl8101_set_wol(struct net_device *dev,
                struct ethtool_wolinfo *wol)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int i,tmp;
    u32 csi_tmp;
    static struct {
        u32 opt;
        u16 reg;
        u8  mask;
    } cfg[] = {
        { WAKE_ANY,   Config1, PMEnable },
        { WAKE_PHY,   Config3, LinkUp },
        { WAKE_UCAST, Config5, UWF },
        { WAKE_BCAST, Config5, BWF },
        { WAKE_MCAST, Config5, MWF },
        { WAKE_ANY,   Config5, LanWake },
        { WAKE_MAGIC, Config3, MagicPacket }
    };
    //unsigned long flags;
    
    spin_lock_irqsave(&tp->lock, flags);
    
    RTL_W8(Cfg9346, Cfg9346_Unlock);
    
    switch (tp->mcfg) {
        case CFG_METHOD_14:
        case CFG_METHOD_17:
            tmp = ARRAY_SIZE(cfg) - 1;
            
            csi_tmp = rtl8101_eri_read(ioaddr, 0xDE, 1, ERIAR_ExGMAC);
            if (wol->wolopts & WAKE_MAGIC)
                csi_tmp |= BIT_0;
            else
                csi_tmp &= ~BIT_0;
            rtl8101_eri_write(ioaddr, 0xDE, 1, csi_tmp, ERIAR_ExGMAC);
            break;
        default:
            tmp = ARRAY_SIZE(cfg);
            break;
    }
    
    for (i = 0; i < tmp; i++) {
        u8 options = RTL_R8(cfg[i].reg) & ~cfg[i].mask;
        if (wol->wolopts & cfg[i].opt)
            options |= cfg[i].mask;
        RTL_W8(cfg[i].reg, options);
    }
    
    RTL_W8(Cfg9346, Cfg9346_Lock);
    
    tp->wol_enabled = (wol->wolopts) ? WOL_ENABLED : WOL_DISABLED;
    
    spin_unlock_irqrestore(&tp->lock, flags);
    
    return 0;
}

static void
rtl8101_get_drvinfo(struct net_device *dev,
                    struct ethtool_drvinfo *info)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    
    strcpy(info->driver, MODULENAME);
    strcpy(info->version, RTL8101_VERSION);
    strcpy(info->bus_info, pci_name(tp->pci_dev));
}

static int
rtl8101_get_regs_len(struct net_device *dev)
{
    return R8101_REGS_SIZE;
}

#endif  /* DISABLED_CODE */

int rtl8101_set_speed_xmii(struct net_device *dev, u8 autoneg, u16 speed, u8 duplex)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    int auto_nego = 0;
    int bmcr_true_force = 0;
    //unsigned long flags;
    
    if ((speed != SPEED_100) &&
        (speed != SPEED_10)) {
        speed = SPEED_100;
        duplex = DUPLEX_FULL;
    }
    
    auto_nego = mdio_read(tp, MII_ADVERTISE);
    
    auto_nego &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL | ADVERTISE_PAUSE_CAP | ADVERTISE_PAUSE_ASYM);
    
    if (autoneg == AUTONEG_ENABLE) {
        /*n-way force*/
        if ((speed == SPEED_10) && (duplex == DUPLEX_HALF)) {
            auto_nego |= ADVERTISE_10HALF;
        } else if ((speed == SPEED_10) && (duplex == DUPLEX_FULL)) {
            auto_nego |= ADVERTISE_10HALF |
            ADVERTISE_10FULL;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_HALF)) {
            auto_nego |= ADVERTISE_100HALF |
            ADVERTISE_10HALF |
            ADVERTISE_10FULL;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL)) {
            auto_nego |= ADVERTISE_100HALF |
            ADVERTISE_100FULL |
            ADVERTISE_10HALF |
            ADVERTISE_10FULL;
        }
        
        //flow contorol
        auto_nego |= ADVERTISE_PAUSE_CAP|ADVERTISE_PAUSE_ASYM;
        
        if ((tp->mcfg == CFG_METHOD_4) || (tp->mcfg == CFG_METHOD_5) ||
            (tp->mcfg == CFG_METHOD_6) || (tp->mcfg == CFG_METHOD_7) ||
            (tp->mcfg == CFG_METHOD_8) || (tp->mcfg == CFG_METHOD_9)) {
            auto_nego &= ~(ADVERTISE_PAUSE_CAP|ADVERTISE_PAUSE_ASYM);
        }
        
        if ((tp->mcfg == CFG_METHOD_10) || (tp->mcfg == CFG_METHOD_11) ||
            (tp->mcfg == CFG_METHOD_12) || (tp->mcfg == CFG_METHOD_13) ||
            (tp->mcfg == CFG_METHOD_14) || (tp->mcfg == CFG_METHOD_15) ||
            (tp->mcfg == CFG_METHOD_16)) {
            if (eee_enable == 1)
                auto_nego &= ~(ADVERTISE_PAUSE_CAP|ADVERTISE_PAUSE_ASYM);
        }
        
        tp->phy_auto_nego_reg = auto_nego;
        
        if ((tp->mcfg == CFG_METHOD_4) ||
            (tp->mcfg == CFG_METHOD_5)) {
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1f, 0x0000);
            mdio_write(tp, MII_BMCR, BMCR_RESET);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            udelay(100);
            rtl8101_hw_phy_config(dev);
        } else if (((tp->mcfg == CFG_METHOD_1) ||
                    (tp->mcfg == CFG_METHOD_2) ||
                    (tp->mcfg == CFG_METHOD_3)) &&
                   (speed == SPEED_10)) {
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1f, 0x0000);
            mdio_write(tp, MII_BMCR, BMCR_RESET);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            rtl8101_hw_phy_config(dev);
        }
        
        spin_lock_irqsave(&tp->phy_lock, flags);
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, MII_ADVERTISE, auto_nego);
        if (tp->mcfg == CFG_METHOD_10)
            mdio_write(tp, MII_BMCR, BMCR_RESET | BMCR_ANENABLE | BMCR_ANRESTART);
        else
            mdio_write(tp, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
        mdelay(20);
    } else {
        /*true force*/
#ifndef BMCR_SPEED100
#define BMCR_SPEED100	0x0040
#endif
        
#ifndef BMCR_SPEED10
#define BMCR_SPEED10	0x0000
#endif
        if ((speed == SPEED_10) && (duplex == DUPLEX_HALF)) {
            bmcr_true_force = BMCR_SPEED10;
        } else if ((speed == SPEED_10) && (duplex == DUPLEX_FULL)) {
            bmcr_true_force = BMCR_SPEED10 |
            BMCR_FULLDPLX;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_HALF)) {
            bmcr_true_force = BMCR_SPEED100;
        } else if ((speed == SPEED_100) && (duplex == DUPLEX_FULL)) {
            bmcr_true_force = BMCR_SPEED100 |
            BMCR_FULLDPLX;
        }
        
        spin_lock_irqsave(&tp->phy_lock, flags);
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, MII_BMCR, bmcr_true_force);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }
    
    tp->autoneg = autoneg;
    tp->speed = speed;
    tp->duplex = duplex;
    
    return 0;
}

int rtl8101_set_speed(struct net_device *dev,
                  u8 autoneg,
                  u16 speed,
                  u8 duplex)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    int ret;
    
    ret = tp->set_speed(dev, autoneg, speed, duplex);
    
    return ret;
}

#if DISABLED_CODE

static int
rtl8101_set_settings(struct net_device *dev,
                     struct ethtool_cmd *cmd)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    int ret;
    
    spin_lock_irqsave(&tp->lock, flags);
    ret = rtl8101_set_speed(dev, cmd->autoneg, cmd->speed, cmd->duplex);
    spin_unlock_irqrestore(&tp->lock, flags);
    
    return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static u32
rtl8101_get_tx_csum(struct net_device *dev)
{
    return (dev->features & NETIF_F_IP_CSUM) != 0;
}

static u32
rtl8101_get_rx_csum(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    
    return tp->cp_cmd & RxChkSum;
}

static int
rtl8101_set_tx_csum(struct net_device *dev,
                    u32 data)
{
    if (data)
        dev->features |= NETIF_F_IP_CSUM;
    else
        dev->features &= ~NETIF_F_IP_CSUM;
    
    return 0;
}

static int
rtl8101_set_rx_csum(struct net_device *dev,
                    u32 data)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;
    
    spin_lock_irqsave(&tp->lock, flags);
    
    if (data)
        tp->cp_cmd |= RxChkSum;
    else
        tp->cp_cmd &= ~RxChkSum;
    
    RTL_W16(CPlusCmd, tp->cp_cmd);
    
    spin_unlock_irqrestore(&tp->lock, flags);
    
    return 0;
}
#endif

#ifdef CONFIG_R8101_VLAN

static inline u32
rtl8101_tx_vlan_tag(struct rtl8101_private *tp,
                    struct sk_buff *skb)
{
    u32 tag;
    
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    tag = (tp->vlgrp && vlan_tx_tag_present(skb)) ?
    TxVlanTag | swab16(vlan_tx_tag_get(skb)) : 0x00;
#else
    tag = (vlan_tx_tag_present(skb)) ?
    TxVlanTag | swab16(vlan_tx_tag_get(skb)) : 0x00;
#endif
    
    return tag;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)

static void
rtl8101_vlan_rx_register(struct net_device *dev,
                         struct vlan_group *grp)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;
    
    spin_lock_irqsave(&tp->lock, flags);
    tp->vlgrp = grp;
    if (tp->vlgrp)
        tp->cp_cmd |= RxVlan;
    else
        tp->cp_cmd &= ~RxVlan;
    RTL_W16(CPlusCmd, tp->cp_cmd);
    spin_unlock_irqrestore(&tp->lock, flags);
}

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
static void
rtl8101_vlan_rx_kill_vid(struct net_device *dev,
                         unsigned short vid)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    
    spin_lock_irqsave(&tp->lock, flags);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
    if (tp->vlgrp)
        tp->vlgrp->vlan_devices[vid] = NULL;
#else
    vlan_group_set_device(tp->vlgrp, vid, NULL);
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,21)
    spin_unlock_irqrestore(&tp->lock, flags);
}
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)

static int
rtl8101_rx_vlan_skb(struct rtl8101_private *tp,
                    struct RxDesc *desc,
                    struct sk_buff *skb)
{
    u32 opts2 = le32_to_cpu(desc->opts2);
    int ret = -1;
    
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    if (tp->vlgrp && (opts2 & RxVlanTag)) {
        rtl8101_rx_hwaccel_skb(skb, tp->vlgrp, swab16(opts2 & 0xffff));
        ret = 0;
    }
#else
    if (opts2 & RxVlanTag)
        __vlan_hwaccel_put_tag(skb, swab16(opts2 & 0xffff));
#endif
    
    desc->opts2 = 0;
    return ret;
}

#else /* !CONFIG_R8101_VLAN */

static inline u32
rtl8101_tx_vlan_tag(struct rtl8101_private *tp,
                    struct sk_buff *skb)
{
    return 0;
}

static int
rtl8101_rx_vlan_skb(struct rtl8101_private *tp,
                    struct RxDesc *desc,
                    struct sk_buff *skb)
{
    return -1;
}

#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,0,0)

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static u32 rtl8101_fix_features(struct net_device *dev, u32 features)
#else
static netdev_features_t rtl8101_fix_features(struct net_device *dev,
                                              netdev_features_t features)
#endif
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    
    spin_lock_irqsave(&tp->lock, flags);
    if (dev->mtu > ETH_DATA_LEN)
        features &= ~NETIF_F_ALL_TSO;
    spin_unlock_irqrestore(&tp->lock, flags);
    
    return features;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static int rtl8101_hw_set_features(struct net_device *dev, u32 features)
#else
static int rtl8101_hw_set_features(struct net_device *dev,
                                   netdev_features_t features)
#endif
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    
    if (features & NETIF_F_RXCSUM)
        tp->cp_cmd |= RxChkSum;
    else
        tp->cp_cmd &= ~RxChkSum;
    
    if (dev->features & NETIF_F_HW_VLAN_RX)
        tp->cp_cmd |= RxVlan;
    else
        tp->cp_cmd &= ~RxVlan;
    
    RTL_W16(CPlusCmd, tp->cp_cmd);
    RTL_R16(CPlusCmd);
    
    return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
static int rtl8101_set_features(struct net_device *dev, u32 features)
#else
static int rtl8101_set_features(struct net_device *dev,
                                netdev_features_t features)
#endif
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    
    spin_lock_irqsave(&tp->lock, flags);
    
    rtl8101_hw_set_features(dev, features);
    
    spin_unlock_irqrestore(&tp->lock, flags);
    
    return 0;
}

#endif

#endif  /* DISABLED_CODE */

void rtl8101_gset_xmii(struct net_device *dev, struct ethtool_cmd *cmd)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u8 status;
    //unsigned long flags;
    
    cmd->supported = SUPPORTED_10baseT_Half |
    SUPPORTED_10baseT_Full |
    SUPPORTED_100baseT_Half |
    SUPPORTED_100baseT_Full |
    SUPPORTED_Autoneg |
    SUPPORTED_TP;
    
    spin_lock_irqsave(&tp->phy_lock, flags);
    cmd->autoneg = (mdio_read(tp, MII_BMCR) & BMCR_ANENABLE) ? 1 : 0;
    spin_unlock_irqrestore(&tp->phy_lock, flags);
    cmd->advertising = ADVERTISED_TP | ADVERTISED_Autoneg;
    
    if (tp->phy_auto_nego_reg & ADVERTISE_10HALF)
        cmd->advertising |= ADVERTISED_10baseT_Half;
    
    if (tp->phy_auto_nego_reg & ADVERTISE_10FULL)
        cmd->advertising |= ADVERTISED_10baseT_Half |
        ADVERTISED_10baseT_Full;
    
    if (tp->phy_auto_nego_reg & ADVERTISE_100HALF)
        cmd->advertising |= ADVERTISED_10baseT_Half |
        ADVERTISED_10baseT_Full |
        ADVERTISED_100baseT_Half;
    
    if (tp->phy_auto_nego_reg & ADVERTISE_100FULL)
        cmd->advertising |= ADVERTISED_10baseT_Half |
        ADVERTISED_10baseT_Full |
        ADVERTISED_100baseT_Half |
        ADVERTISED_100baseT_Full;
    
    status = RTL_R8(PHYstatus);
    
    if (status & _100bps)
        cmd->speed = SPEED_100;
    else if (status & _10bps)
        cmd->speed = SPEED_10;
    
    if (status & TxFlowCtrl)
        cmd->advertising |= ADVERTISED_Asym_Pause;
    
    if (status & RxFlowCtrl)
        cmd->advertising |= ADVERTISED_Pause;
    
    cmd->duplex = (status & FullDup) ? DUPLEX_FULL : DUPLEX_HALF;
}

#if DISABLED_CODE

static int
rtl8101_get_settings(struct net_device *dev,
                     struct ethtool_cmd *cmd)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    //unsigned long flags;
    
    spin_lock_irqsave(&tp->lock, flags);
    
    tp->get_settings(dev, cmd);
    
    spin_unlock_irqrestore(&tp->lock, flags);
    return 0;
}

static void
rtl8101_get_regs(struct net_device *dev,
                 struct ethtool_regs *regs,
                 void *p)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    
    if (regs->len > R8101_REGS_SIZE)
        regs->len = R8101_REGS_SIZE;
    
    spin_lock_irqsave(&tp->lock, flags);
    memcpy_fromio(p, tp->mmio_addr, regs->len);
    spin_unlock_irqrestore(&tp->lock, flags);
}

static u32
rtl8101_get_msglevel(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    
    return tp->msg_enable;
}

static void
rtl8101_set_msglevel(struct net_device *dev,
                     u32 value)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    
    tp->msg_enable = value;
}

static const char rtl8101_gstrings[][ETH_GSTRING_LEN] = {
    "tx_packets",
    "rx_packets",
    "tx_errors",
    "rx_errors",
    "rx_missed",
    "align_errors",
    "tx_single_collisions",
    "tx_multi_collisions",
    "unicast",
    "broadcast",
    "multicast",
    "tx_aborted",
    "tx_underrun",
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
static int rtl8101_get_stats_count(struct net_device *dev)
{
    return ARRAY_SIZE(rtl8101_gstrings);
}
#else
static int rtl8101_get_sset_count(struct net_device *dev, int sset)
{
    switch (sset) {
        case ETH_SS_STATS:
            return ARRAY_SIZE(rtl8101_gstrings);
        default:
            return -EOPNOTSUPP;
    }
}
#endif

static void
rtl8101_get_ethtool_stats(struct net_device *dev,
                          struct ethtool_stats *stats,
                          u64 *data)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    struct rtl8101_counters *counters;
    dma_addr_t paddr;
    u32 cmd;
    
    ASSERT_RTNL();
    
    counters = tp->tally_vaddr;
    paddr = tp->tally_paddr;
    if (!counters)
        return;
    
    RTL_W32(CounterAddrHigh, (u64)paddr >> 32);
    cmd = (u64)paddr & DMA_BIT_MASK(32);
    RTL_W32(CounterAddrLow, cmd);
    RTL_W32(CounterAddrLow, cmd | CounterDump);
    
    while (RTL_R32(CounterAddrLow) & CounterDump) {
        if (msleep_interruptible(1))
            break;
    }
    
    data[0]	= le64_to_cpu(counters->tx_packets);
    data[1] = le64_to_cpu(counters->rx_packets);
    data[2] = le64_to_cpu(counters->tx_errors);
    data[3] = le32_to_cpu(counters->rx_errors);
    data[4] = le16_to_cpu(counters->rx_missed);
    data[5] = le16_to_cpu(counters->align_errors);
    data[6] = le32_to_cpu(counters->tx_one_collision);
    data[7] = le32_to_cpu(counters->tx_multi_collision);
    data[8] = le64_to_cpu(counters->rx_unicast);
    data[9] = le64_to_cpu(counters->rx_broadcast);
    data[10] = le32_to_cpu(counters->rx_multicast);
    data[11] = le16_to_cpu(counters->tx_aborted);
    data[12] = le16_to_cpu(counters->tx_underun);
}

static void
rtl8101_get_strings(struct net_device *dev,
                    u32 stringset,
                    u8 *data)
{
    switch (stringset) {
        case ETH_SS_STATS:
            memcpy(data, *rtl8101_gstrings, sizeof(rtl8101_gstrings));
            break;
    }
}

#undef ethtool_op_get_link
#define ethtool_op_get_link _kc_ethtool_op_get_link
u32 _kc_ethtool_op_get_link(struct net_device *dev)
{
    return netif_carrier_ok(dev) ? 1 : 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
#undef ethtool_op_get_sg
#define ethtool_op_get_sg _kc_ethtool_op_get_sg
u32 _kc_ethtool_op_get_sg(struct net_device *dev)
{
#ifdef NETIF_F_SG
    return (dev->features & NETIF_F_SG) != 0;
#else
    return 0;
#endif
}

#undef ethtool_op_set_sg
#define ethtool_op_set_sg _kc_ethtool_op_set_sg
int _kc_ethtool_op_set_sg(struct net_device *dev, u32 data)
{
#ifdef NETIF_F_SG
    if (data)
        dev->features |= NETIF_F_SG;
    else
        dev->features &= ~NETIF_F_SG;
#endif
    
    return 0;
}
#endif

static struct ethtool_ops rtl8101_ethtool_ops = {
    .get_drvinfo		= rtl8101_get_drvinfo,
    .get_regs_len		= rtl8101_get_regs_len,
    .get_link		= ethtool_op_get_link,
    .get_settings		= rtl8101_get_settings,
    .set_settings		= rtl8101_set_settings,
    .get_msglevel		= rtl8101_get_msglevel,
    .set_msglevel		= rtl8101_set_msglevel,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
    .get_rx_csum		= rtl8101_get_rx_csum,
    .set_rx_csum		= rtl8101_set_rx_csum,
    .get_tx_csum		= rtl8101_get_tx_csum,
    .set_tx_csum		= rtl8101_set_tx_csum,
    .get_sg			= ethtool_op_get_sg,
    .set_sg			= ethtool_op_set_sg,
#ifdef NETIF_F_TSO
    .get_tso		= ethtool_op_get_tso,
    .set_tso		= ethtool_op_set_tso,
#endif
#endif
    .get_regs		= rtl8101_get_regs,
    .get_wol		= rtl8101_get_wol,
    .set_wol		= rtl8101_set_wol,
    .get_strings		= rtl8101_get_strings,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
    .get_stats_count    = rtl8101_get_stats_count,
#else
    .get_sset_count     = rtl8101_get_sset_count,
#endif
    .get_ethtool_stats	= rtl8101_get_ethtool_stats,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
#ifdef ETHTOOL_GPERMADDR
    .get_perm_addr		= ethtool_op_get_perm_addr,
#endif
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)
};

#endif  /* DISABLED_CODE */

void rtl8101_get_mac_version(struct rtl8101_private *tp, void __iomem *ioaddr)
{
    u32 reg,val32;
    u32 ICVerID;
    
    val32 = RTL_R32(TxConfig);
    reg = val32 & 0x7c800000;
    ICVerID = val32 & 0x00700000;
    
    tp->mcfg = CFG_METHOD_UNKNOWN;
    switch (reg) {
        case 0x34000000:
            if (ICVerID == 0x00000000)
                tp->mcfg = CFG_METHOD_1;
            else if (ICVerID == 0x00200000)
                tp->mcfg = CFG_METHOD_2;
            else if (ICVerID == 0x00300000)
                tp->mcfg = CFG_METHOD_3;
            else
                tp->mcfg = CFG_METHOD_3;
            break;
        case 0x34800000:
        case 0x24800000:
            if (ICVerID == 0x00100000)
                tp->mcfg = CFG_METHOD_4;
            else if (ICVerID == 0x00200000)
                tp->mcfg = CFG_METHOD_5;
            else if (ICVerID == 0x00400000)
                tp->mcfg = CFG_METHOD_6;
            else if (ICVerID == 0x00500000)
                tp->mcfg = CFG_METHOD_7;
            else if (ICVerID == 0x00600000)
                tp->mcfg = CFG_METHOD_8;
            else
                tp->mcfg = CFG_METHOD_8;
            break;
        case 0x24000000:
            tp->mcfg = CFG_METHOD_9;
            break;
        case 0x2C000000:
            tp->mcfg = CFG_METHOD_10;
            break;
        case 0x40800000:
            if (ICVerID == 0x00100000)
                tp->mcfg = CFG_METHOD_11;
            else if (ICVerID == 0x00200000)
                tp->mcfg = CFG_METHOD_12;
            else if (ICVerID == 0x00300000)
                tp->mcfg = CFG_METHOD_13;
            else if (ICVerID == 0x00400000)
                tp->mcfg = CFG_METHOD_13;
            break;
        case 0x44000000:
            tp->mcfg = CFG_METHOD_14;
            break;
        case 0x44800000:
            if (ICVerID == 0x00000000)
                tp->mcfg = CFG_METHOD_15;
            else if (ICVerID == 0x00100000)
                tp->mcfg = CFG_METHOD_16;
            break;
        case 0x50800000:
            if (ICVerID == 0x00100000) {
                tp->mcfg = CFG_METHOD_17;
            } else {
                tp->mcfg = CFG_METHOD_17;
            }
            break;
        default:
            printk("unknown chip version (%x)\n",reg);
            break;
    }
}

#if DISABLED_CODE

static void
rtl8101_print_mac_version(struct rtl8101_private *tp)
{
    int i;
    for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--) {
        if (tp->mcfg == rtl_chip_info[i].mcfg) {
            dprintk("Realtek PCIe FE Family Controller mcfg = %04d\n",
                    rtl_chip_info[i].mcfg);
            return;
        }
    }
    
    dprintk("mac_version == Unknown\n");
}

static void
rtl8101_tally_counter_addr_fill(struct rtl8101_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    
    if (!tp->tally_paddr)
        return;
    
    RTL_W32(CounterAddrHigh, (u64)tp->tally_paddr >> 32);
    RTL_W32(CounterAddrLow, (u64)tp->tally_paddr & (DMA_BIT_MASK(32)));
}

static void
rtl8101_tally_counter_clear(struct rtl8101_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    
    if (tp->mcfg == CFG_METHOD_1 || tp->mcfg == CFG_METHOD_2 ||
        tp->mcfg == CFG_METHOD_3 )
        return;
    
    if (!tp->tally_paddr)
        return;
    
    RTL_W32(CounterAddrHigh, (u64)tp->tally_paddr >> 32);
    RTL_W32(CounterAddrLow, (u64)tp->tally_paddr & (DMA_BIT_MASK(32) | BIT_0));
}

#endif  /* DISABLED_CODE */

void rtl8101_exit_oob(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u16 data16;
    
    rtl8101_enable_rxdvgate(dev);
    
    rtl8101_wait_txrx_fifo_empty(dev);
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            RTL_W8(MCUCmd_reg, RTL_R8(MCUCmd_reg) & ~Now_is_oob);
            
            data16 = mac_ocp_read(tp, 0xE8DE) & ~BIT_14;
            mac_ocp_write(tp, 0xE8DE, data16);
            rtl8101_wait_ll_share_fifo_ready(dev);
            
            data16 = mac_ocp_read(tp, 0xE8DE) | BIT_15;
            mac_ocp_write(tp, 0xE8DE, data16);
            
            rtl8101_wait_ll_share_fifo_ready(dev);
            break;
    }
}

static void
rtl8101_hw_mac_mcu_config(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned int i;
    
    if (tp->mcfg == CFG_METHOD_17) {
        u16 rtl8101_phy_value[]= {
            0xE008, 0xE01B, 0xE02E, 0xE044, 0xE046,
            0xE048, 0xE04A, 0xE04C, 0x49D2, 0xF10D,
            0x766C, 0x49E2, 0xF00A, 0x1EC0, 0x8EE1,
            0xC60A, 0x77C0, 0x4870, 0x9FC0, 0x1EA0,
            0xC707, 0x8EE1, 0x9D6C, 0xC603, 0xBE00,
            0xB416, 0x0076, 0xE86C, 0xC513, 0x64A0,
            0x49C1, 0xF00A, 0x1CEA, 0x2242, 0x0402,
            0xC50B, 0x9CA2, 0x1C11, 0x9CA0, 0xC506,
            0xBD00, 0x7444, 0xC502, 0xBD00, 0x0A30,
            0x0A46, 0xE434, 0xE096, 0x49D9, 0xF00F,
            0xC512, 0x74A0, 0x48C8, 0x48CA, 0x9CA0,
            0xC50F, 0x1B00, 0x9BA0, 0x1B1C, 0x483F,
            0x9BA2, 0x1B04, 0xC5F0, 0x9BA0, 0xC602,
            0xBE00, 0x03DE, 0xE434, 0xE096, 0xE860,
            0xDE20, 0xC602, 0xBE00, 0x0000, 0xC602,
            0xBE00, 0x0000, 0xC602, 0xBE00, 0x0000,
            0xC602, 0xBE00, 0x0000, 0xC602, 0xBE00,
            0x0000
        };
        mac_ocp_write(tp, 0xFC28, 0x0000);
        mac_ocp_write(tp, 0xFC2A, 0x0000);
        mac_ocp_write(tp, 0xFC2C, 0x0000);
        mac_ocp_write(tp, 0xFC2E, 0x0000);
        mac_ocp_write(tp, 0xFC30, 0x0000);
        mac_ocp_write(tp, 0xFC32, 0x0000);
        mac_ocp_write(tp, 0xFC34, 0x0000);
        mac_ocp_write(tp, 0xFC36, 0x0000);
        mdelay(3);
        mac_ocp_write(tp, 0xFC26, 0x0000);
        for (i = 0; i < ARRAY_SIZE(rtl8101_phy_value); i++)
            mac_ocp_write(tp, 0xF800+i*2, rtl8101_phy_value[i]);
        mac_ocp_write(tp, 0xFC26, 0x8000);
        mac_ocp_write(tp, 0xFC2A, 0x0A2F);
        mac_ocp_write(tp, 0xFC2C, 0x0297);
    }
}

void rtl8101_hw_init(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    //unsigned long flags;
    
    switch (tp->mcfg) {
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
            RTL_W8(Cfg9346, Cfg9346_Unlock);
            RTL_W8(Config5, RTL_R8(Config5) & ~BIT_0);
            RTL_W8(Config2, RTL_R8(Config2) & ~BIT_7);
            RTL_W8(Cfg9346, Cfg9346_Lock);
            RTL_W8(0xF1, RTL_R8(0xF1) & ~BIT_7);
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            RTL_W8(Cfg9346, Cfg9346_Unlock);
            RTL_W8(Config5, RTL_R8(Config5) & ~BIT_0);
            RTL_W8(Config2, RTL_R8(Config2) & ~BIT_7);
            RTL_W8(Cfg9346, Cfg9346_Lock);
            RTL_W8(0xF1, RTL_R8(0xF1) & ~BIT_7);
            break;
    }
    
    if (tp->mcfg == CFG_METHOD_10)
        RTL_W8(0xF3, RTL_R8(0xF3) | BIT_2);
    
    rtl8101_hw_mac_mcu_config(dev);
    
    /*disable ocp phy power saving*/
    if (tp->mcfg == CFG_METHOD_17 ) {
        spin_lock_irqsave(&tp->phy_lock, flags);
        mdio_write(tp, 0x1F, 0x0C41);
        mdio_write(tp, 0x13, 0x0000);
        mdio_write(tp, 0x13, 0x0500);
        mdio_write(tp, 0x1F, 0x0000);
        spin_unlock_irqrestore(&tp->phy_lock, flags);
    }
}

void rtl8101_hw_ephy_config(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u16 ephy_data;
    
    if (tp->mcfg == CFG_METHOD_4) {
        rtl8101_ephy_write(ioaddr, 0x03, 0xc2f9);
    } else if (tp->mcfg == CFG_METHOD_5) {
        rtl8101_ephy_write(ioaddr, 0x01, 0x6FE5);
        rtl8101_ephy_write(ioaddr, 0x03, 0xD7D9);
    } else if (tp->mcfg == CFG_METHOD_6) {
        rtl8101_ephy_write(ioaddr, 0x06, 0xAF35);
    } else if (tp->mcfg == CFG_METHOD_7) {
        rtl8101_ephy_write(ioaddr, 0x19, 0xEC90);
        rtl8101_ephy_write(ioaddr, 0x01, 0x6FE5);
        rtl8101_ephy_write(ioaddr, 0x03, 0x05D9);
        rtl8101_ephy_write(ioaddr, 0x06, 0xAF35);
    } else if (tp->mcfg == CFG_METHOD_8) {
        rtl8101_ephy_write(ioaddr, 0x01, 0x6FE5);
        rtl8101_ephy_write(ioaddr, 0x03, 0x05D9);
        rtl8101_ephy_write(ioaddr, 0x06, 0xAF35);
        rtl8101_ephy_write(ioaddr, 0x19, 0xECFA);
    } else if (tp->mcfg == CFG_METHOD_9) {
        rtl8101_ephy_write(ioaddr, 0x06, 0xAF25);
        rtl8101_ephy_write(ioaddr, 0x07, 0x8E68);
    } else if (tp->mcfg == CFG_METHOD_10) {
        ephy_data = rtl8101_ephy_read(ioaddr, 0x00) & ~0x0200;
        ephy_data |= 0x0100;
        rtl8101_ephy_write(ioaddr, 0x00, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x00);
        ephy_data |= 0x0004;
        rtl8101_ephy_write(ioaddr, 0x00, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x06) & ~0x0002;
        ephy_data |= 0x0001;
        rtl8101_ephy_write(ioaddr, 0x06, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x06);
        ephy_data |= 0x0030;
        rtl8101_ephy_write(ioaddr, 0x06, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x07);
        ephy_data |= 0x2000;
        rtl8101_ephy_write(ioaddr, 0x07, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x00);
        ephy_data |= 0x0020;
        rtl8101_ephy_write(ioaddr, 0x00, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x03) & ~0x5800;
        ephy_data |= 0x2000;
        rtl8101_ephy_write(ioaddr, 0x03, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x03);
        ephy_data |= 0x0001;
        rtl8101_ephy_write(ioaddr, 0x03, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x01) & ~0x0800;
        ephy_data |= 0x1000;
        rtl8101_ephy_write(ioaddr, 0x01, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x07);
        ephy_data |= 0x4000;
        rtl8101_ephy_write(ioaddr, 0x07, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x1E);
        ephy_data |= 0x2000;
        rtl8101_ephy_write(ioaddr, 0x1E, ephy_data);
        
        rtl8101_ephy_write(ioaddr, 0x19, 0xFE6C);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x0A);
        ephy_data |= 0x0040;
        rtl8101_ephy_write(ioaddr, 0x0A, ephy_data);
        
    } else if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
               tp->mcfg == CFG_METHOD_13) {
        ephy_data = rtl8101_ephy_read(ioaddr, 0x07);
        ephy_data |= 0x4000;
        rtl8101_ephy_write(ioaddr, 0x07, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x19);
        ephy_data |= 0x0200;
        rtl8101_ephy_write(ioaddr, 0x19, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x19);
        ephy_data |= 0x0020;
        rtl8101_ephy_write(ioaddr, 0x19, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x1E);
        ephy_data |= 0x2000;
        rtl8101_ephy_write(ioaddr, 0x1E, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x03);
        ephy_data |= 0x0001;
        rtl8101_ephy_write(ioaddr, 0x03, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x19);
        ephy_data |= 0x0100;
        rtl8101_ephy_write(ioaddr, 0x19, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x19);
        ephy_data |= 0x0004;
        rtl8101_ephy_write(ioaddr, 0x19, ephy_data);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x0A);
        ephy_data |= 0x0020;
        rtl8101_ephy_write(ioaddr, 0x0A, ephy_data);
        
        if (tp->mcfg == CFG_METHOD_11) {
            RTL_W8(Config5, RTL_R8(Config5) & ~BIT_0);
        } else if (tp->mcfg == CFG_METHOD_12 ||
                   tp->mcfg == CFG_METHOD_13) {
            ephy_data = rtl8101_ephy_read(ioaddr, 0x1E);
            ephy_data |= 0x8000;
            rtl8101_ephy_write(ioaddr, 0x1E, ephy_data);
        }
    } else if (tp->mcfg == CFG_METHOD_14) {
        rtl8101_ephy_write(ioaddr, 0x19, 0xff64);
    } else if (tp->mcfg == CFG_METHOD_17) {
        ephy_data = rtl8101_ephy_read(ioaddr, 0x00);
        ephy_data |= BIT_3;
        rtl8101_ephy_write(ioaddr, 0x00, ephy_data);
        ephy_data = rtl8101_ephy_read(ioaddr, 0x0C);
        ephy_data &= ~(BIT_13 | BIT_12 | BIT_11 | BIT_9 | BIT_8 | BIT_7 | BIT_6 | BIT_5 | BIT_4);
        ephy_data |= BIT_10;
        rtl8101_ephy_write(ioaddr, 0x0C, ephy_data);
        
        rtl8101_ephy_write(ioaddr, 0x19, 0xFC00);
        rtl8101_ephy_write(ioaddr, 0x1E, 0x20EB);
        
        ephy_data = rtl8101_ephy_read(ioaddr, 0x06);
        ephy_data |= BIT_4;
        rtl8101_ephy_write(ioaddr, 0x06, ephy_data);
    }
}

static int
rtl8101_check_hw_phy_mcu_code_ver(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    int ram_code_ver_match = 0;
    u16 sw_ram_code_ver = 0xFFFF;
    u16 hw_ram_code_ver = 0;
    
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_17;
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            mdio_write(tp, 0x1F, 0x0A43);
            mdio_write(tp, 0x13, 0x801E);
            hw_ram_code_ver = mdio_read(tp, 0x14);
            mdio_write(tp, 0x1F, 0x0000);
            break;
    }
    
    if( hw_ram_code_ver == sw_ram_code_ver)
        ram_code_ver_match = 1;
    
    return ram_code_ver_match;
}

static void
rtl8101_write_hw_phy_mcu_code_ver(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    u16 sw_ram_code_ver = 0xFFFF;
    
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            sw_ram_code_ver = NIC_RAMCODE_VERSION_CFG_METHOD_17;
            break;
    }
    
    switch (tp->mcfg) {
        case CFG_METHOD_17:
            mdio_write(tp, 0x1F, 0x0A43);
            mdio_write(tp, 0x13, 0x801E);
            mdio_write(tp, 0x14, sw_ram_code_ver);
            mdio_write(tp, 0x1F, 0x0000);
            break;
    }
}

static void
rtl8101_init_hw_phy_mcu(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned int gphy_val,i;
    u32 csi_tmp;
    
    if(rtl8101_check_hw_phy_mcu_code_ver(dev))
        return;
    
    if (tp->mcfg == CFG_METHOD_10) {
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x00, 0x1800);
        mdio_write(tp, 0x1f, 0x0007);
        mdio_write(tp, 0x1e, 0x0023);
        mdio_write(tp, 0x17, 0x0117);
        mdio_write(tp, 0x1f, 0x0007);
        mdio_write(tp, 0x1E, 0x002C);
        mdio_write(tp, 0x1B, 0x5000);
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x16, 0x4104);
        for (i = 0; i < 200; i++) {
            udelay(100);
            gphy_val = mdio_read(tp, 0x1E);
            gphy_val &= 0x03FF;
            if (gphy_val==0x000C)
                break;
        }
        mdio_write(tp, 0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            udelay(100);
            gphy_val = mdio_read(tp, 0x07);
            if ((gphy_val & BIT_5)==0)
                break;
        }
        gphy_val = mdio_read(tp, 0x07);
        if (gphy_val & BIT_5) {
            mdio_write(tp, 0x1f, 0x0007);
            mdio_write(tp, 0x1e, 0x00a1);
            mdio_write(tp, 0x17, 0x1000);
            mdio_write(tp, 0x17, 0x0000);
            mdio_write(tp, 0x17, 0x2000);
            mdio_write(tp, 0x1e, 0x002f);
            mdio_write(tp, 0x18, 0x9bfb);
            mdio_write(tp, 0x1f, 0x0005);
            mdio_write(tp, 0x07, 0x0000);
            mdio_write(tp, 0x1f, 0x0000);
        }
        mdio_write(tp, 0x1f, 0x0005);
        mdio_write(tp, 0x05, 0xfff6);
        mdio_write(tp, 0x06, 0x0080);
        gphy_val = mdio_read(tp, 0x00);
        gphy_val &= ~(BIT_7);
        mdio_write(tp, 0x00, gphy_val);
        mdio_write(tp, 0x1f, 0x0002);
        gphy_val = mdio_read(tp, 0x08);
        gphy_val &= ~(BIT_7);
        mdio_write(tp, 0x08, gphy_val);
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x1f, 0x0007);
        mdio_write(tp, 0x1e, 0x0023);
        mdio_write(tp, 0x16, 0x0306);
        mdio_write(tp, 0x16, 0x0307);
        mdio_write(tp, 0x15, 0x000e);
        mdio_write(tp, 0x19, 0x000a);
        mdio_write(tp, 0x15, 0x0010);
        mdio_write(tp, 0x19, 0x0008);
        mdio_write(tp, 0x15, 0x0018);
        mdio_write(tp, 0x19, 0x4801);
        mdio_write(tp, 0x15, 0x0019);
        mdio_write(tp, 0x19, 0x6801);
        mdio_write(tp, 0x15, 0x001a);
        mdio_write(tp, 0x19, 0x66a1);
        mdio_write(tp, 0x15, 0x001f);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0020);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0021);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0022);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0023);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0024);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0025);
        mdio_write(tp, 0x19, 0x64a1);
        mdio_write(tp, 0x15, 0x0026);
        mdio_write(tp, 0x19, 0x40ea);
        mdio_write(tp, 0x15, 0x0027);
        mdio_write(tp, 0x19, 0x4503);
        mdio_write(tp, 0x15, 0x0028);
        mdio_write(tp, 0x19, 0x9f00);
        mdio_write(tp, 0x15, 0x0029);
        mdio_write(tp, 0x19, 0xa631);
        mdio_write(tp, 0x15, 0x002a);
        mdio_write(tp, 0x19, 0x9717);
        mdio_write(tp, 0x15, 0x002b);
        mdio_write(tp, 0x19, 0x302c);
        mdio_write(tp, 0x15, 0x002c);
        mdio_write(tp, 0x19, 0x4802);
        mdio_write(tp, 0x15, 0x002d);
        mdio_write(tp, 0x19, 0x58da);
        mdio_write(tp, 0x15, 0x002e);
        mdio_write(tp, 0x19, 0x400d);
        mdio_write(tp, 0x15, 0x002f);
        mdio_write(tp, 0x19, 0x4488);
        mdio_write(tp, 0x15, 0x0030);
        mdio_write(tp, 0x19, 0x9e00);
        mdio_write(tp, 0x15, 0x0031);
        mdio_write(tp, 0x19, 0x63c8);
        mdio_write(tp, 0x15, 0x0032);
        mdio_write(tp, 0x19, 0x6481);
        mdio_write(tp, 0x15, 0x0033);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0034);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0035);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0036);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0037);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0038);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0039);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x003a);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x003b);
        mdio_write(tp, 0x19, 0x63e8);
        mdio_write(tp, 0x15, 0x003c);
        mdio_write(tp, 0x19, 0x7d00);
        mdio_write(tp, 0x15, 0x003d);
        mdio_write(tp, 0x19, 0x59d4);
        mdio_write(tp, 0x15, 0x003e);
        mdio_write(tp, 0x19, 0x63f8);
        mdio_write(tp, 0x15, 0x0040);
        mdio_write(tp, 0x19, 0x64a1);
        mdio_write(tp, 0x15, 0x0041);
        mdio_write(tp, 0x19, 0x30de);
        mdio_write(tp, 0x15, 0x0044);
        mdio_write(tp, 0x19, 0x480f);
        mdio_write(tp, 0x15, 0x0045);
        mdio_write(tp, 0x19, 0x6800);
        mdio_write(tp, 0x15, 0x0046);
        mdio_write(tp, 0x19, 0x6680);
        mdio_write(tp, 0x15, 0x0047);
        mdio_write(tp, 0x19, 0x7c10);
        mdio_write(tp, 0x15, 0x0048);
        mdio_write(tp, 0x19, 0x63c8);
        mdio_write(tp, 0x15, 0x0049);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x004a);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x004b);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x004c);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x004d);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x004e);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x004f);
        mdio_write(tp, 0x19, 0x40ea);
        mdio_write(tp, 0x15, 0x0050);
        mdio_write(tp, 0x19, 0x4503);
        mdio_write(tp, 0x15, 0x0051);
        mdio_write(tp, 0x19, 0x58ca);
        mdio_write(tp, 0x15, 0x0052);
        mdio_write(tp, 0x19, 0x63c8);
        mdio_write(tp, 0x15, 0x0053);
        mdio_write(tp, 0x19, 0x63d8);
        mdio_write(tp, 0x15, 0x0054);
        mdio_write(tp, 0x19, 0x66a0);
        mdio_write(tp, 0x15, 0x0055);
        mdio_write(tp, 0x19, 0x9f00);
        mdio_write(tp, 0x15, 0x0056);
        mdio_write(tp, 0x19, 0x3000);
        mdio_write(tp, 0x15, 0x006E);
        mdio_write(tp, 0x19, 0x9afa);
        mdio_write(tp, 0x15, 0x00a1);
        mdio_write(tp, 0x19, 0x3044);
        mdio_write(tp, 0x15, 0x00ab);
        mdio_write(tp, 0x19, 0x5820);
        mdio_write(tp, 0x15, 0x00ac);
        mdio_write(tp, 0x19, 0x5e04);
        mdio_write(tp, 0x15, 0x00ad);
        mdio_write(tp, 0x19, 0xb60c);
        mdio_write(tp, 0x15, 0x00af);
        mdio_write(tp, 0x19, 0x000a);
        mdio_write(tp, 0x15, 0x00b2);
        mdio_write(tp, 0x19, 0x30b9);
        mdio_write(tp, 0x15, 0x00b9);
        mdio_write(tp, 0x19, 0x4408);
        mdio_write(tp, 0x15, 0x00ba);
        mdio_write(tp, 0x19, 0x480b);
        mdio_write(tp, 0x15, 0x00bb);
        mdio_write(tp, 0x19, 0x5e00);
        mdio_write(tp, 0x15, 0x00bc);
        mdio_write(tp, 0x19, 0x405f);
        mdio_write(tp, 0x15, 0x00bd);
        mdio_write(tp, 0x19, 0x4448);
        mdio_write(tp, 0x15, 0x00be);
        mdio_write(tp, 0x19, 0x4020);
        mdio_write(tp, 0x15, 0x00bf);
        mdio_write(tp, 0x19, 0x4468);
        mdio_write(tp, 0x15, 0x00c0);
        mdio_write(tp, 0x19, 0x9c02);
        mdio_write(tp, 0x15, 0x00c1);
        mdio_write(tp, 0x19, 0x58a0);
        mdio_write(tp, 0x15, 0x00c2);
        mdio_write(tp, 0x19, 0xb605);
        mdio_write(tp, 0x15, 0x00c3);
        mdio_write(tp, 0x19, 0xc0d3);
        mdio_write(tp, 0x15, 0x00c4);
        mdio_write(tp, 0x19, 0x00e6);
        mdio_write(tp, 0x15, 0x00c5);
        mdio_write(tp, 0x19, 0xdaec);
        mdio_write(tp, 0x15, 0x00c6);
        mdio_write(tp, 0x19, 0x00fa);
        mdio_write(tp, 0x15, 0x00c7);
        mdio_write(tp, 0x19, 0x9df9);
        mdio_write(tp, 0x15, 0x00c8);
        mdio_write(tp, 0x19, 0x307a);
        mdio_write(tp, 0x15, 0x0112);
        mdio_write(tp, 0x19, 0x6421);
        mdio_write(tp, 0x15, 0x0113);
        mdio_write(tp, 0x19, 0x7c08);
        mdio_write(tp, 0x15, 0x0114);
        mdio_write(tp, 0x19, 0x63f0);
        mdio_write(tp, 0x15, 0x0115);
        mdio_write(tp, 0x19, 0x4003);
        mdio_write(tp, 0x15, 0x0116);
        mdio_write(tp, 0x19, 0x4418);
        mdio_write(tp, 0x15, 0x0117);
        mdio_write(tp, 0x19, 0x9b00);
        mdio_write(tp, 0x15, 0x0118);
        mdio_write(tp, 0x19, 0x6461);
        mdio_write(tp, 0x15, 0x0119);
        mdio_write(tp, 0x19, 0x64e1);
        mdio_write(tp, 0x15, 0x011a);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0150);
        mdio_write(tp, 0x19, 0x6461);
        mdio_write(tp, 0x15, 0x0151);
        mdio_write(tp, 0x19, 0x4003);
        mdio_write(tp, 0x15, 0x0152);
        mdio_write(tp, 0x19, 0x4540);
        mdio_write(tp, 0x15, 0x0153);
        mdio_write(tp, 0x19, 0x9f00);
        mdio_write(tp, 0x15, 0x0155);
        mdio_write(tp, 0x19, 0x6421);
        mdio_write(tp, 0x15, 0x0156);
        mdio_write(tp, 0x19, 0x64a1);
        mdio_write(tp, 0x15, 0x021e);
        mdio_write(tp, 0x19, 0x5410);
        mdio_write(tp, 0x15, 0x0225);
        mdio_write(tp, 0x19, 0x5400);
        mdio_write(tp, 0x15, 0x023D);
        mdio_write(tp, 0x19, 0x4050);
        mdio_write(tp, 0x15, 0x0295);
        mdio_write(tp, 0x19, 0x6c08);
        mdio_write(tp, 0x15, 0x02bd);
        mdio_write(tp, 0x19, 0xa523);
        mdio_write(tp, 0x15, 0x02be);
        mdio_write(tp, 0x19, 0x32ca);
        mdio_write(tp, 0x15, 0x02ca);
        mdio_write(tp, 0x19, 0x48b3);
        mdio_write(tp, 0x15, 0x02cb);
        mdio_write(tp, 0x19, 0x4020);
        mdio_write(tp, 0x15, 0x02cc);
        mdio_write(tp, 0x19, 0x4823);
        mdio_write(tp, 0x15, 0x02cd);
        mdio_write(tp, 0x19, 0x4510);
        mdio_write(tp, 0x15, 0x02ce);
        mdio_write(tp, 0x19, 0xb63a);
        mdio_write(tp, 0x15, 0x02cf);
        mdio_write(tp, 0x19, 0x7dc8);
        mdio_write(tp, 0x15, 0x02d6);
        mdio_write(tp, 0x19, 0x9bf8);
        mdio_write(tp, 0x15, 0x02d8);
        mdio_write(tp, 0x19, 0x85f6);
        mdio_write(tp, 0x15, 0x02d9);
        mdio_write(tp, 0x19, 0x32e0);
        mdio_write(tp, 0x15, 0x02e0);
        mdio_write(tp, 0x19, 0x4834);
        mdio_write(tp, 0x15, 0x02e1);
        mdio_write(tp, 0x19, 0x6c08);
        mdio_write(tp, 0x15, 0x02e2);
        mdio_write(tp, 0x19, 0x4020);
        mdio_write(tp, 0x15, 0x02e3);
        mdio_write(tp, 0x19, 0x4824);
        mdio_write(tp, 0x15, 0x02e4);
        mdio_write(tp, 0x19, 0x4520);
        mdio_write(tp, 0x15, 0x02e5);
        mdio_write(tp, 0x19, 0x4008);
        mdio_write(tp, 0x15, 0x02e6);
        mdio_write(tp, 0x19, 0x4560);
        mdio_write(tp, 0x15, 0x02e7);
        mdio_write(tp, 0x19, 0x9d04);
        mdio_write(tp, 0x15, 0x02e8);
        mdio_write(tp, 0x19, 0x48c4);
        mdio_write(tp, 0x15, 0x02e9);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x02ea);
        mdio_write(tp, 0x19, 0x4844);
        mdio_write(tp, 0x15, 0x02eb);
        mdio_write(tp, 0x19, 0x7dc8);
        mdio_write(tp, 0x15, 0x02f0);
        mdio_write(tp, 0x19, 0x9cf7);
        mdio_write(tp, 0x15, 0x02f1);
        mdio_write(tp, 0x19, 0xdf94);
        mdio_write(tp, 0x15, 0x02f2);
        mdio_write(tp, 0x19, 0x0002);
        mdio_write(tp, 0x15, 0x02f3);
        mdio_write(tp, 0x19, 0x6810);
        mdio_write(tp, 0x15, 0x02f4);
        mdio_write(tp, 0x19, 0xb614);
        mdio_write(tp, 0x15, 0x02f5);
        mdio_write(tp, 0x19, 0xc42b);
        mdio_write(tp, 0x15, 0x02f6);
        mdio_write(tp, 0x19, 0x00d4);
        mdio_write(tp, 0x15, 0x02f7);
        mdio_write(tp, 0x19, 0xc455);
        mdio_write(tp, 0x15, 0x02f8);
        mdio_write(tp, 0x19, 0x0093);
        mdio_write(tp, 0x15, 0x02f9);
        mdio_write(tp, 0x19, 0x92ee);
        mdio_write(tp, 0x15, 0x02fa);
        mdio_write(tp, 0x19, 0xefed);
        mdio_write(tp, 0x15, 0x02fb);
        mdio_write(tp, 0x19, 0x3312);
        mdio_write(tp, 0x15, 0x0312);
        mdio_write(tp, 0x19, 0x49b5);
        mdio_write(tp, 0x15, 0x0313);
        mdio_write(tp, 0x19, 0x7d00);
        mdio_write(tp, 0x15, 0x0314);
        mdio_write(tp, 0x19, 0x4d00);
        mdio_write(tp, 0x15, 0x0315);
        mdio_write(tp, 0x19, 0x6810);
        mdio_write(tp, 0x15, 0x031e);
        mdio_write(tp, 0x19, 0x404f);
        mdio_write(tp, 0x15, 0x031f);
        mdio_write(tp, 0x19, 0x44c8);
        mdio_write(tp, 0x15, 0x0320);
        mdio_write(tp, 0x19, 0xd64f);
        mdio_write(tp, 0x15, 0x0321);
        mdio_write(tp, 0x19, 0x00e7);
        mdio_write(tp, 0x15, 0x0322);
        mdio_write(tp, 0x19, 0x7c08);
        mdio_write(tp, 0x15, 0x0323);
        mdio_write(tp, 0x19, 0x8203);
        mdio_write(tp, 0x15, 0x0324);
        mdio_write(tp, 0x19, 0x4d48);
        mdio_write(tp, 0x15, 0x0325);
        mdio_write(tp, 0x19, 0x3327);
        mdio_write(tp, 0x15, 0x0326);
        mdio_write(tp, 0x19, 0x4d40);
        mdio_write(tp, 0x15, 0x0327);
        mdio_write(tp, 0x19, 0xc8d7);
        mdio_write(tp, 0x15, 0x0328);
        mdio_write(tp, 0x19, 0x0003);
        mdio_write(tp, 0x15, 0x0329);
        mdio_write(tp, 0x19, 0x7c20);
        mdio_write(tp, 0x15, 0x032a);
        mdio_write(tp, 0x19, 0x4c20);
        mdio_write(tp, 0x15, 0x032b);
        mdio_write(tp, 0x19, 0xc8ed);
        mdio_write(tp, 0x15, 0x032c);
        mdio_write(tp, 0x19, 0x00f4);
        mdio_write(tp, 0x15, 0x032d);
        mdio_write(tp, 0x19, 0x82b3);
        mdio_write(tp, 0x15, 0x032e);
        mdio_write(tp, 0x19, 0xd11d);
        mdio_write(tp, 0x15, 0x032f);
        mdio_write(tp, 0x19, 0x00b1);
        mdio_write(tp, 0x15, 0x0330);
        mdio_write(tp, 0x19, 0xde18);
        mdio_write(tp, 0x15, 0x0331);
        mdio_write(tp, 0x19, 0x0008);
        mdio_write(tp, 0x15, 0x0332);
        mdio_write(tp, 0x19, 0x91ee);
        mdio_write(tp, 0x15, 0x0333);
        mdio_write(tp, 0x19, 0x3339);
        mdio_write(tp, 0x15, 0x033a);
        mdio_write(tp, 0x19, 0x4064);
        mdio_write(tp, 0x15, 0x0340);
        mdio_write(tp, 0x19, 0x9e06);
        mdio_write(tp, 0x15, 0x0341);
        mdio_write(tp, 0x19, 0x7c08);
        mdio_write(tp, 0x15, 0x0342);
        mdio_write(tp, 0x19, 0x8203);
        mdio_write(tp, 0x15, 0x0343);
        mdio_write(tp, 0x19, 0x4d48);
        mdio_write(tp, 0x15, 0x0344);
        mdio_write(tp, 0x19, 0x3346);
        mdio_write(tp, 0x15, 0x0345);
        mdio_write(tp, 0x19, 0x4d40);
        mdio_write(tp, 0x15, 0x0346);
        mdio_write(tp, 0x19, 0xd11d);
        mdio_write(tp, 0x15, 0x0347);
        mdio_write(tp, 0x19, 0x0099);
        mdio_write(tp, 0x15, 0x0348);
        mdio_write(tp, 0x19, 0xbb17);
        mdio_write(tp, 0x15, 0x0349);
        mdio_write(tp, 0x19, 0x8102);
        mdio_write(tp, 0x15, 0x034a);
        mdio_write(tp, 0x19, 0x334d);
        mdio_write(tp, 0x15, 0x034b);
        mdio_write(tp, 0x19, 0xa22c);
        mdio_write(tp, 0x15, 0x034c);
        mdio_write(tp, 0x19, 0x3397);
        mdio_write(tp, 0x15, 0x034d);
        mdio_write(tp, 0x19, 0x91f2);
        mdio_write(tp, 0x15, 0x034e);
        mdio_write(tp, 0x19, 0xc218);
        mdio_write(tp, 0x15, 0x034f);
        mdio_write(tp, 0x19, 0x00f0);
        mdio_write(tp, 0x15, 0x0350);
        mdio_write(tp, 0x19, 0x3397);
        mdio_write(tp, 0x15, 0x0351);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0364);
        mdio_write(tp, 0x19, 0xbc05);
        mdio_write(tp, 0x15, 0x0367);
        mdio_write(tp, 0x19, 0xa1fc);
        mdio_write(tp, 0x15, 0x0368);
        mdio_write(tp, 0x19, 0x3377);
        mdio_write(tp, 0x15, 0x0369);
        mdio_write(tp, 0x19, 0x328b);
        mdio_write(tp, 0x15, 0x036a);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x15, 0x0377);
        mdio_write(tp, 0x19, 0x4b97);
        mdio_write(tp, 0x15, 0x0378);
        mdio_write(tp, 0x19, 0x6818);
        mdio_write(tp, 0x15, 0x0379);
        mdio_write(tp, 0x19, 0x4b07);
        mdio_write(tp, 0x15, 0x037a);
        mdio_write(tp, 0x19, 0x40ac);
        mdio_write(tp, 0x15, 0x037b);
        mdio_write(tp, 0x19, 0x4445);
        mdio_write(tp, 0x15, 0x037c);
        mdio_write(tp, 0x19, 0x404e);
        mdio_write(tp, 0x15, 0x037d);
        mdio_write(tp, 0x19, 0x4461);
        mdio_write(tp, 0x15, 0x037e);
        mdio_write(tp, 0x19, 0x9c09);
        mdio_write(tp, 0x15, 0x037f);
        mdio_write(tp, 0x19, 0x63da);
        mdio_write(tp, 0x15, 0x0380);
        mdio_write(tp, 0x19, 0x5440);
        mdio_write(tp, 0x15, 0x0381);
        mdio_write(tp, 0x19, 0x4b98);
        mdio_write(tp, 0x15, 0x0382);
        mdio_write(tp, 0x19, 0x7c60);
        mdio_write(tp, 0x15, 0x0383);
        mdio_write(tp, 0x19, 0x4c00);
        mdio_write(tp, 0x15, 0x0384);
        mdio_write(tp, 0x19, 0x4b08);
        mdio_write(tp, 0x15, 0x0385);
        mdio_write(tp, 0x19, 0x63d8);
        mdio_write(tp, 0x15, 0x0386);
        mdio_write(tp, 0x19, 0x338d);
        mdio_write(tp, 0x15, 0x0387);
        mdio_write(tp, 0x19, 0xd64f);
        mdio_write(tp, 0x15, 0x0388);
        mdio_write(tp, 0x19, 0x0080);
        mdio_write(tp, 0x15, 0x0389);
        mdio_write(tp, 0x19, 0x820c);
        mdio_write(tp, 0x15, 0x038a);
        mdio_write(tp, 0x19, 0xa10b);
        mdio_write(tp, 0x15, 0x038b);
        mdio_write(tp, 0x19, 0x9df3);
        mdio_write(tp, 0x15, 0x038c);
        mdio_write(tp, 0x19, 0x3395);
        mdio_write(tp, 0x15, 0x038d);
        mdio_write(tp, 0x19, 0xd64f);
        mdio_write(tp, 0x15, 0x038e);
        mdio_write(tp, 0x19, 0x00f9);
        mdio_write(tp, 0x15, 0x038f);
        mdio_write(tp, 0x19, 0xc017);
        mdio_write(tp, 0x15, 0x0390);
        mdio_write(tp, 0x19, 0x0005);
        mdio_write(tp, 0x15, 0x0391);
        mdio_write(tp, 0x19, 0x6c0b);
        mdio_write(tp, 0x15, 0x0392);
        mdio_write(tp, 0x19, 0xa103);
        mdio_write(tp, 0x15, 0x0393);
        mdio_write(tp, 0x19, 0x6c08);
        mdio_write(tp, 0x15, 0x0394);
        mdio_write(tp, 0x19, 0x9df9);
        mdio_write(tp, 0x15, 0x0395);
        mdio_write(tp, 0x19, 0x6c08);
        mdio_write(tp, 0x15, 0x0396);
        mdio_write(tp, 0x19, 0x3397);
        mdio_write(tp, 0x15, 0x0399);
        mdio_write(tp, 0x19, 0x6810);
        mdio_write(tp, 0x15, 0x03a4);
        mdio_write(tp, 0x19, 0x7c08);
        mdio_write(tp, 0x15, 0x03a5);
        mdio_write(tp, 0x19, 0x8203);
        mdio_write(tp, 0x15, 0x03a6);
        mdio_write(tp, 0x19, 0x4d08);
        mdio_write(tp, 0x15, 0x03a7);
        mdio_write(tp, 0x19, 0x33a9);
        mdio_write(tp, 0x15, 0x03a8);
        mdio_write(tp, 0x19, 0x4d00);
        mdio_write(tp, 0x15, 0x03a9);
        mdio_write(tp, 0x19, 0x9bfa);
        mdio_write(tp, 0x15, 0x03aa);
        mdio_write(tp, 0x19, 0x33b6);
        mdio_write(tp, 0x15, 0x03bb);
        mdio_write(tp, 0x19, 0x4056);
        mdio_write(tp, 0x15, 0x03bc);
        mdio_write(tp, 0x19, 0x44e9);
        mdio_write(tp, 0x15, 0x03bd);
        mdio_write(tp, 0x19, 0x4054);
        mdio_write(tp, 0x15, 0x03be);
        mdio_write(tp, 0x19, 0x44f8);
        mdio_write(tp, 0x15, 0x03bf);
        mdio_write(tp, 0x19, 0xd64f);
        mdio_write(tp, 0x15, 0x03c0);
        mdio_write(tp, 0x19, 0x0037);
        mdio_write(tp, 0x15, 0x03c1);
        mdio_write(tp, 0x19, 0xbd37);
        mdio_write(tp, 0x15, 0x03c2);
        mdio_write(tp, 0x19, 0x9cfd);
        mdio_write(tp, 0x15, 0x03c3);
        mdio_write(tp, 0x19, 0xc639);
        mdio_write(tp, 0x15, 0x03c4);
        mdio_write(tp, 0x19, 0x0011);
        mdio_write(tp, 0x15, 0x03c5);
        mdio_write(tp, 0x19, 0x9b03);
        mdio_write(tp, 0x15, 0x03c6);
        mdio_write(tp, 0x19, 0x7c01);
        mdio_write(tp, 0x15, 0x03c7);
        mdio_write(tp, 0x19, 0x4c01);
        mdio_write(tp, 0x15, 0x03c8);
        mdio_write(tp, 0x19, 0x9e03);
        mdio_write(tp, 0x15, 0x03c9);
        mdio_write(tp, 0x19, 0x7c20);
        mdio_write(tp, 0x15, 0x03ca);
        mdio_write(tp, 0x19, 0x4c20);
        mdio_write(tp, 0x15, 0x03cb);
        mdio_write(tp, 0x19, 0x9af4);
        mdio_write(tp, 0x15, 0x03cc);
        mdio_write(tp, 0x19, 0x7c12);
        mdio_write(tp, 0x15, 0x03cd);
        mdio_write(tp, 0x19, 0x4c52);
        mdio_write(tp, 0x15, 0x03ce);
        mdio_write(tp, 0x19, 0x4470);
        mdio_write(tp, 0x15, 0x03cf);
        mdio_write(tp, 0x19, 0x7c12);
        mdio_write(tp, 0x15, 0x03d0);
        mdio_write(tp, 0x19, 0x4c40);
        mdio_write(tp, 0x15, 0x03d1);
        mdio_write(tp, 0x19, 0x33bf);
        mdio_write(tp, 0x15, 0x03d6);
        mdio_write(tp, 0x19, 0x4047);
        mdio_write(tp, 0x15, 0x03d7);
        mdio_write(tp, 0x19, 0x4469);
        mdio_write(tp, 0x15, 0x03d8);
        mdio_write(tp, 0x19, 0x492b);
        mdio_write(tp, 0x15, 0x03d9);
        mdio_write(tp, 0x19, 0x4479);
        mdio_write(tp, 0x15, 0x03da);
        mdio_write(tp, 0x19, 0x7c09);
        mdio_write(tp, 0x15, 0x03db);
        mdio_write(tp, 0x19, 0x8203);
        mdio_write(tp, 0x15, 0x03dc);
        mdio_write(tp, 0x19, 0x4d48);
        mdio_write(tp, 0x15, 0x03dd);
        mdio_write(tp, 0x19, 0x33df);
        mdio_write(tp, 0x15, 0x03de);
        mdio_write(tp, 0x19, 0x4d40);
        mdio_write(tp, 0x15, 0x03df);
        mdio_write(tp, 0x19, 0xd64f);
        mdio_write(tp, 0x15, 0x03e0);
        mdio_write(tp, 0x19, 0x0017);
        mdio_write(tp, 0x15, 0x03e1);
        mdio_write(tp, 0x19, 0xbd17);
        mdio_write(tp, 0x15, 0x03e2);
        mdio_write(tp, 0x19, 0x9b03);
        mdio_write(tp, 0x15, 0x03e3);
        mdio_write(tp, 0x19, 0x7c20);
        mdio_write(tp, 0x15, 0x03e4);
        mdio_write(tp, 0x19, 0x4c20);
        mdio_write(tp, 0x15, 0x03e5);
        mdio_write(tp, 0x19, 0x88f5);
        mdio_write(tp, 0x15, 0x03e6);
        mdio_write(tp, 0x19, 0xc428);
        mdio_write(tp, 0x15, 0x03e7);
        mdio_write(tp, 0x19, 0x0008);
        mdio_write(tp, 0x15, 0x03e8);
        mdio_write(tp, 0x19, 0x9af2);
        mdio_write(tp, 0x15, 0x03e9);
        mdio_write(tp, 0x19, 0x7c12);
        mdio_write(tp, 0x15, 0x03ea);
        mdio_write(tp, 0x19, 0x4c52);
        mdio_write(tp, 0x15, 0x03eb);
        mdio_write(tp, 0x19, 0x4470);
        mdio_write(tp, 0x15, 0x03ec);
        mdio_write(tp, 0x19, 0x7c12);
        mdio_write(tp, 0x15, 0x03ed);
        mdio_write(tp, 0x19, 0x4c40);
        mdio_write(tp, 0x15, 0x03ee);
        mdio_write(tp, 0x19, 0x33da);
        mdio_write(tp, 0x15, 0x03ef);
        mdio_write(tp, 0x19, 0x3312);
        mdio_write(tp, 0x16, 0x0306);
        mdio_write(tp, 0x16, 0x0300);
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x17, 0x2179);
        mdio_write(tp, 0x1f, 0x0007);
        mdio_write(tp, 0x1e, 0x0040);
        mdio_write(tp, 0x18, 0x0645);
        mdio_write(tp, 0x19, 0xe200);
        mdio_write(tp, 0x18, 0x0655);
        mdio_write(tp, 0x19, 0x9000);
        mdio_write(tp, 0x18, 0x0d05);
        mdio_write(tp, 0x19, 0xbe00);
        mdio_write(tp, 0x18, 0x0d15);
        mdio_write(tp, 0x19, 0xd300);
        mdio_write(tp, 0x18, 0x0d25);
        mdio_write(tp, 0x19, 0xfe00);
        mdio_write(tp, 0x18, 0x0d35);
        mdio_write(tp, 0x19, 0x4000);
        mdio_write(tp, 0x18, 0x0d45);
        mdio_write(tp, 0x19, 0x7f00);
        mdio_write(tp, 0x18, 0x0d55);
        mdio_write(tp, 0x19, 0x1000);
        mdio_write(tp, 0x18, 0x0d65);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x0d75);
        mdio_write(tp, 0x19, 0x8200);
        mdio_write(tp, 0x18, 0x0d85);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x0d95);
        mdio_write(tp, 0x19, 0x7000);
        mdio_write(tp, 0x18, 0x0da5);
        mdio_write(tp, 0x19, 0x0f00);
        mdio_write(tp, 0x18, 0x0db5);
        mdio_write(tp, 0x19, 0x0100);
        mdio_write(tp, 0x18, 0x0dc5);
        mdio_write(tp, 0x19, 0x9b00);
        mdio_write(tp, 0x18, 0x0dd5);
        mdio_write(tp, 0x19, 0x7f00);
        mdio_write(tp, 0x18, 0x0de5);
        mdio_write(tp, 0x19, 0xe000);
        mdio_write(tp, 0x18, 0x0df5);
        mdio_write(tp, 0x19, 0xef00);
        mdio_write(tp, 0x18, 0x16d5);
        mdio_write(tp, 0x19, 0xe200);
        mdio_write(tp, 0x18, 0x16e5);
        mdio_write(tp, 0x19, 0xab00);
        mdio_write(tp, 0x18, 0x2904);
        mdio_write(tp, 0x19, 0x4000);
        mdio_write(tp, 0x18, 0x2914);
        mdio_write(tp, 0x19, 0x7f00);
        mdio_write(tp, 0x18, 0x2924);
        mdio_write(tp, 0x19, 0x0100);
        mdio_write(tp, 0x18, 0x2934);
        mdio_write(tp, 0x19, 0x2000);
        mdio_write(tp, 0x18, 0x2944);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x2954);
        mdio_write(tp, 0x19, 0x4600);
        mdio_write(tp, 0x18, 0x2964);
        mdio_write(tp, 0x19, 0xfc00);
        mdio_write(tp, 0x18, 0x2974);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x2984);
        mdio_write(tp, 0x19, 0x5000);
        mdio_write(tp, 0x18, 0x2994);
        mdio_write(tp, 0x19, 0x9d00);
        mdio_write(tp, 0x18, 0x29a4);
        mdio_write(tp, 0x19, 0xff00);
        mdio_write(tp, 0x18, 0x29b4);
        mdio_write(tp, 0x19, 0x4000);
        mdio_write(tp, 0x18, 0x29c4);
        mdio_write(tp, 0x19, 0x7f00);
        mdio_write(tp, 0x18, 0x29d4);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x29e4);
        mdio_write(tp, 0x19, 0x2000);
        mdio_write(tp, 0x18, 0x29f4);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x2a04);
        mdio_write(tp, 0x19, 0xe600);
        mdio_write(tp, 0x18, 0x2a14);
        mdio_write(tp, 0x19, 0xff00);
        mdio_write(tp, 0x18, 0x2a24);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x2a34);
        mdio_write(tp, 0x19, 0x5000);
        mdio_write(tp, 0x18, 0x2a44);
        mdio_write(tp, 0x19, 0x8500);
        mdio_write(tp, 0x18, 0x2a54);
        mdio_write(tp, 0x19, 0x7f00);
        mdio_write(tp, 0x18, 0x2a64);
        mdio_write(tp, 0x19, 0xac00);
        mdio_write(tp, 0x18, 0x2a74);
        mdio_write(tp, 0x19, 0x0800);
        mdio_write(tp, 0x18, 0x2a84);
        mdio_write(tp, 0x19, 0xfc00);
        mdio_write(tp, 0x18, 0x2a94);
        mdio_write(tp, 0x19, 0xe000);
        mdio_write(tp, 0x18, 0x2aa4);
        mdio_write(tp, 0x19, 0x7400);
        mdio_write(tp, 0x18, 0x2ab4);
        mdio_write(tp, 0x19, 0x4000);
        mdio_write(tp, 0x18, 0x2ac4);
        mdio_write(tp, 0x19, 0x7f00);
        mdio_write(tp, 0x18, 0x2ad4);
        mdio_write(tp, 0x19, 0x0100);
        mdio_write(tp, 0x18, 0x2ae4);
        mdio_write(tp, 0x19, 0xff00);
        mdio_write(tp, 0x18, 0x2af4);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x2b04);
        mdio_write(tp, 0x19, 0x4400);
        mdio_write(tp, 0x18, 0x2b14);
        mdio_write(tp, 0x19, 0xfc00);
        mdio_write(tp, 0x18, 0x2b24);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x2b34);
        mdio_write(tp, 0x19, 0x4000);
        mdio_write(tp, 0x18, 0x2b44);
        mdio_write(tp, 0x19, 0x9d00);
        mdio_write(tp, 0x18, 0x2b54);
        mdio_write(tp, 0x19, 0xff00);
        mdio_write(tp, 0x18, 0x2b64);
        mdio_write(tp, 0x19, 0x4000);
        mdio_write(tp, 0x18, 0x2b74);
        mdio_write(tp, 0x19, 0x7f00);
        mdio_write(tp, 0x18, 0x2b84);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x2b94);
        mdio_write(tp, 0x19, 0xff00);
        mdio_write(tp, 0x18, 0x2ba4);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x2bb4);
        mdio_write(tp, 0x19, 0xfc00);
        mdio_write(tp, 0x18, 0x2bc4);
        mdio_write(tp, 0x19, 0xff00);
        mdio_write(tp, 0x18, 0x2bd4);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x2be4);
        mdio_write(tp, 0x19, 0x4000);
        mdio_write(tp, 0x18, 0x2bf4);
        mdio_write(tp, 0x19, 0x8900);
        mdio_write(tp, 0x18, 0x2c04);
        mdio_write(tp, 0x19, 0x8300);
        mdio_write(tp, 0x18, 0x2c14);
        mdio_write(tp, 0x19, 0xe000);
        mdio_write(tp, 0x18, 0x2c24);
        mdio_write(tp, 0x19, 0x0000);
        mdio_write(tp, 0x18, 0x2c34);
        mdio_write(tp, 0x19, 0xac00);
        mdio_write(tp, 0x18, 0x2c44);
        mdio_write(tp, 0x19, 0x0800);
        mdio_write(tp, 0x18, 0x2c54);
        mdio_write(tp, 0x19, 0xfa00);
        mdio_write(tp, 0x18, 0x2c64);
        mdio_write(tp, 0x19, 0xe100);
        mdio_write(tp, 0x18, 0x2c74);
        mdio_write(tp, 0x19, 0x7f00);
        mdio_write(tp, 0x18, 0x0001);
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x17, 0x2100);
        mdio_write(tp, 0x1f, 0x0005);
        mdio_write(tp, 0x05, 0xfff6);
        mdio_write(tp, 0x06, 0x0080);
        mdio_write(tp, 0x05, 0x8000);
        mdio_write(tp, 0x06, 0xd480);
        mdio_write(tp, 0x06, 0xc1e4);
        mdio_write(tp, 0x06, 0x8b9a);
        mdio_write(tp, 0x06, 0xe58b);
        mdio_write(tp, 0x06, 0x9bee);
        mdio_write(tp, 0x06, 0x8b83);
        mdio_write(tp, 0x06, 0x41bf);
        mdio_write(tp, 0x06, 0x8b88);
        mdio_write(tp, 0x06, 0xec00);
        mdio_write(tp, 0x06, 0x19a9);
        mdio_write(tp, 0x06, 0x8b90);
        mdio_write(tp, 0x06, 0xf9ee);
        mdio_write(tp, 0x06, 0xfff6);
        mdio_write(tp, 0x06, 0x00ee);
        mdio_write(tp, 0x06, 0xfff7);
        mdio_write(tp, 0x06, 0xffe0);
        mdio_write(tp, 0x06, 0xe140);
        mdio_write(tp, 0x06, 0xe1e1);
        mdio_write(tp, 0x06, 0x41f7);
        mdio_write(tp, 0x06, 0x2ff6);
        mdio_write(tp, 0x06, 0x28e4);
        mdio_write(tp, 0x06, 0xe140);
        mdio_write(tp, 0x06, 0xe5e1);
        mdio_write(tp, 0x06, 0x41f7);
        mdio_write(tp, 0x06, 0x0002);
        mdio_write(tp, 0x06, 0x020c);
        mdio_write(tp, 0x06, 0x0202);
        mdio_write(tp, 0x06, 0x1d02);
        mdio_write(tp, 0x06, 0x0230);
        mdio_write(tp, 0x06, 0x0202);
        mdio_write(tp, 0x06, 0x4002);
        mdio_write(tp, 0x06, 0x028b);
        mdio_write(tp, 0x06, 0x0280);
        mdio_write(tp, 0x06, 0x6c02);
        mdio_write(tp, 0x06, 0x8085);
        mdio_write(tp, 0x06, 0xe08b);
        mdio_write(tp, 0x06, 0x88e1);
        mdio_write(tp, 0x06, 0x8b89);
        mdio_write(tp, 0x06, 0x1e01);
        mdio_write(tp, 0x06, 0xe18b);
        mdio_write(tp, 0x06, 0x8a1e);
        mdio_write(tp, 0x06, 0x01e1);
        mdio_write(tp, 0x06, 0x8b8b);
        mdio_write(tp, 0x06, 0x1e01);
        mdio_write(tp, 0x06, 0xe18b);
        mdio_write(tp, 0x06, 0x8c1e);
        mdio_write(tp, 0x06, 0x01e1);
        mdio_write(tp, 0x06, 0x8b8d);
        mdio_write(tp, 0x06, 0x1e01);
        mdio_write(tp, 0x06, 0xe18b);
        mdio_write(tp, 0x06, 0x8e1e);
        mdio_write(tp, 0x06, 0x01a0);
        mdio_write(tp, 0x06, 0x00c7);
        mdio_write(tp, 0x06, 0xaec3);
        mdio_write(tp, 0x06, 0xf8e0);
        mdio_write(tp, 0x06, 0x8b8d);
        mdio_write(tp, 0x06, 0xad20);
        mdio_write(tp, 0x06, 0x10ee);
        mdio_write(tp, 0x06, 0x8b8d);
        mdio_write(tp, 0x06, 0x0002);
        mdio_write(tp, 0x06, 0x1310);
        mdio_write(tp, 0x06, 0x0280);
        mdio_write(tp, 0x06, 0xc602);
        mdio_write(tp, 0x06, 0x1f0c);
        mdio_write(tp, 0x06, 0x0227);
        mdio_write(tp, 0x06, 0x49fc);
        mdio_write(tp, 0x06, 0x04f8);
        mdio_write(tp, 0x06, 0xe08b);
        mdio_write(tp, 0x06, 0x8ead);
        mdio_write(tp, 0x06, 0x200b);
        mdio_write(tp, 0x06, 0xf620);
        mdio_write(tp, 0x06, 0xe48b);
        mdio_write(tp, 0x06, 0x8e02);
        mdio_write(tp, 0x06, 0x852d);
        mdio_write(tp, 0x06, 0x021b);
        mdio_write(tp, 0x06, 0x67ad);
        mdio_write(tp, 0x06, 0x2211);
        mdio_write(tp, 0x06, 0xf622);
        mdio_write(tp, 0x06, 0xe48b);
        mdio_write(tp, 0x06, 0x8e02);
        mdio_write(tp, 0x06, 0x2ba5);
        mdio_write(tp, 0x06, 0x022a);
        mdio_write(tp, 0x06, 0x2402);
        mdio_write(tp, 0x06, 0x82e5);
        mdio_write(tp, 0x06, 0x022a);
        mdio_write(tp, 0x06, 0xf0ad);
        mdio_write(tp, 0x06, 0x2511);
        mdio_write(tp, 0x06, 0xf625);
        mdio_write(tp, 0x06, 0xe48b);
        mdio_write(tp, 0x06, 0x8e02);
        mdio_write(tp, 0x06, 0x8445);
        mdio_write(tp, 0x06, 0x0204);
        mdio_write(tp, 0x06, 0x0302);
        mdio_write(tp, 0x06, 0x19cc);
        mdio_write(tp, 0x06, 0x022b);
        mdio_write(tp, 0x06, 0x5bfc);
        mdio_write(tp, 0x06, 0x04ee);
        mdio_write(tp, 0x06, 0x8b8d);
        mdio_write(tp, 0x06, 0x0105);
        mdio_write(tp, 0x06, 0xf8f9);
        mdio_write(tp, 0x06, 0xfae0);
        mdio_write(tp, 0x06, 0x8b81);
        mdio_write(tp, 0x06, 0xac26);
        mdio_write(tp, 0x06, 0x08e0);
        mdio_write(tp, 0x06, 0x8b81);
        mdio_write(tp, 0x06, 0xac21);
        mdio_write(tp, 0x06, 0x02ae);
        mdio_write(tp, 0x06, 0x6bee);
        mdio_write(tp, 0x06, 0xe0ea);
        mdio_write(tp, 0x06, 0x00ee);
        mdio_write(tp, 0x06, 0xe0eb);
        mdio_write(tp, 0x06, 0x00e2);
        mdio_write(tp, 0x06, 0xe07c);
        mdio_write(tp, 0x06, 0xe3e0);
        mdio_write(tp, 0x06, 0x7da5);
        mdio_write(tp, 0x06, 0x1111);
        mdio_write(tp, 0x06, 0x15d2);
        mdio_write(tp, 0x06, 0x60d6);
        mdio_write(tp, 0x06, 0x6666);
        mdio_write(tp, 0x06, 0x0207);
        mdio_write(tp, 0x06, 0x6cd2);
        mdio_write(tp, 0x06, 0xa0d6);
        mdio_write(tp, 0x06, 0xaaaa);
        mdio_write(tp, 0x06, 0x0207);
        mdio_write(tp, 0x06, 0x6c02);
        mdio_write(tp, 0x06, 0x201d);
        mdio_write(tp, 0x06, 0xae44);
        mdio_write(tp, 0x06, 0xa566);
        mdio_write(tp, 0x06, 0x6602);
        mdio_write(tp, 0x06, 0xae38);
        mdio_write(tp, 0x06, 0xa5aa);
        mdio_write(tp, 0x06, 0xaa02);
        mdio_write(tp, 0x06, 0xae32);
        mdio_write(tp, 0x06, 0xeee0);
        mdio_write(tp, 0x06, 0xea04);
        mdio_write(tp, 0x06, 0xeee0);
        mdio_write(tp, 0x06, 0xeb06);
        mdio_write(tp, 0x06, 0xe2e0);
        mdio_write(tp, 0x06, 0x7ce3);
        mdio_write(tp, 0x06, 0xe07d);
        mdio_write(tp, 0x06, 0xe0e0);
        mdio_write(tp, 0x06, 0x38e1);
        mdio_write(tp, 0x06, 0xe039);
        mdio_write(tp, 0x06, 0xad2e);
        mdio_write(tp, 0x06, 0x21ad);
        mdio_write(tp, 0x06, 0x3f13);
        mdio_write(tp, 0x06, 0xe0e4);
        mdio_write(tp, 0x06, 0x14e1);
        mdio_write(tp, 0x06, 0xe415);
        mdio_write(tp, 0x06, 0x6880);
        mdio_write(tp, 0x06, 0xe4e4);
        mdio_write(tp, 0x06, 0x14e5);
        mdio_write(tp, 0x06, 0xe415);
        mdio_write(tp, 0x06, 0x0220);
        mdio_write(tp, 0x06, 0x1dae);
        mdio_write(tp, 0x06, 0x0bac);
        mdio_write(tp, 0x06, 0x3e02);
        mdio_write(tp, 0x06, 0xae06);
        mdio_write(tp, 0x06, 0x0281);
        mdio_write(tp, 0x06, 0x4602);
        mdio_write(tp, 0x06, 0x2057);
        mdio_write(tp, 0x06, 0xfefd);
        mdio_write(tp, 0x06, 0xfc04);
        mdio_write(tp, 0x06, 0xf8e0);
        mdio_write(tp, 0x06, 0x8b81);
        mdio_write(tp, 0x06, 0xad26);
        mdio_write(tp, 0x06, 0x0302);
        mdio_write(tp, 0x06, 0x20a7);
        mdio_write(tp, 0x06, 0xe08b);
        mdio_write(tp, 0x06, 0x81ad);
        mdio_write(tp, 0x06, 0x2109);
        mdio_write(tp, 0x06, 0xe08b);
        mdio_write(tp, 0x06, 0x2eac);
        mdio_write(tp, 0x06, 0x2003);
        mdio_write(tp, 0x06, 0x0281);
        mdio_write(tp, 0x06, 0x61fc);
        mdio_write(tp, 0x06, 0x04f8);
        mdio_write(tp, 0x06, 0xe08b);
        mdio_write(tp, 0x06, 0x81ac);
        mdio_write(tp, 0x06, 0x2505);
        mdio_write(tp, 0x06, 0x0222);
        mdio_write(tp, 0x06, 0xaeae);
        mdio_write(tp, 0x06, 0x0302);
        mdio_write(tp, 0x06, 0x8172);
        mdio_write(tp, 0x06, 0xfc04);
        mdio_write(tp, 0x06, 0xf8f9);
        mdio_write(tp, 0x06, 0xfaef);
        mdio_write(tp, 0x06, 0x69fa);
        mdio_write(tp, 0x06, 0xe086);
        mdio_write(tp, 0x06, 0x20a0);
        mdio_write(tp, 0x06, 0x8016);
        mdio_write(tp, 0x06, 0xe086);
        mdio_write(tp, 0x06, 0x21e1);
        mdio_write(tp, 0x06, 0x8b33);
        mdio_write(tp, 0x06, 0x1b10);
        mdio_write(tp, 0x06, 0x9e06);
        mdio_write(tp, 0x06, 0x0223);
        mdio_write(tp, 0x06, 0x91af);
        mdio_write(tp, 0x06, 0x8252);
        mdio_write(tp, 0x06, 0xee86);
        mdio_write(tp, 0x06, 0x2081);
        mdio_write(tp, 0x06, 0xaee4);
        mdio_write(tp, 0x06, 0xa081);
        mdio_write(tp, 0x06, 0x1402);
        mdio_write(tp, 0x06, 0x2399);
        mdio_write(tp, 0x06, 0xbf25);
        mdio_write(tp, 0x06, 0xcc02);
        mdio_write(tp, 0x06, 0x2d21);
        mdio_write(tp, 0x06, 0xee86);
        mdio_write(tp, 0x06, 0x2100);
        mdio_write(tp, 0x06, 0xee86);
        mdio_write(tp, 0x06, 0x2082);
        mdio_write(tp, 0x06, 0xaf82);
        mdio_write(tp, 0x06, 0x52a0);
        mdio_write(tp, 0x06, 0x8232);
        mdio_write(tp, 0x06, 0xe086);
        mdio_write(tp, 0x06, 0x21e1);
        mdio_write(tp, 0x06, 0x8b32);
        mdio_write(tp, 0x06, 0x1b10);
        mdio_write(tp, 0x06, 0x9e06);
        mdio_write(tp, 0x06, 0x0223);
        mdio_write(tp, 0x06, 0x91af);
        mdio_write(tp, 0x06, 0x8252);
        mdio_write(tp, 0x06, 0xee86);
        mdio_write(tp, 0x06, 0x2100);
        mdio_write(tp, 0x06, 0xd000);
        mdio_write(tp, 0x06, 0x0282);
        mdio_write(tp, 0x06, 0x5910);
        mdio_write(tp, 0x06, 0xa004);
        mdio_write(tp, 0x06, 0xf9e0);
        mdio_write(tp, 0x06, 0x861f);
        mdio_write(tp, 0x06, 0xa000);
        mdio_write(tp, 0x06, 0x07ee);
        mdio_write(tp, 0x06, 0x8620);
        mdio_write(tp, 0x06, 0x83af);
        mdio_write(tp, 0x06, 0x8178);
        mdio_write(tp, 0x06, 0x0224);
        mdio_write(tp, 0x06, 0x0102);
        mdio_write(tp, 0x06, 0x2399);
        mdio_write(tp, 0x06, 0xae72);
        mdio_write(tp, 0x06, 0xa083);
        mdio_write(tp, 0x06, 0x4b1f);
        mdio_write(tp, 0x06, 0x55d0);
        mdio_write(tp, 0x06, 0x04bf);
        mdio_write(tp, 0x06, 0x8615);
        mdio_write(tp, 0x06, 0x1a90);
        mdio_write(tp, 0x06, 0x0c54);
        mdio_write(tp, 0x06, 0xd91e);
        mdio_write(tp, 0x06, 0x31b0);
        mdio_write(tp, 0x06, 0xf4e0);
        mdio_write(tp, 0x06, 0xe022);
        mdio_write(tp, 0x06, 0xe1e0);
        mdio_write(tp, 0x06, 0x23ad);
        mdio_write(tp, 0x06, 0x2e0c);
        mdio_write(tp, 0x06, 0xef02);
        mdio_write(tp, 0x06, 0xef12);
        mdio_write(tp, 0x06, 0x0e44);
        mdio_write(tp, 0x06, 0xef23);
        mdio_write(tp, 0x06, 0x0e54);
        mdio_write(tp, 0x06, 0xef21);
        mdio_write(tp, 0x06, 0xe6e4);
        mdio_write(tp, 0x06, 0x2ae7);
        mdio_write(tp, 0x06, 0xe42b);
        mdio_write(tp, 0x06, 0xe2e4);
        mdio_write(tp, 0x06, 0x28e3);
        mdio_write(tp, 0x06, 0xe429);
        mdio_write(tp, 0x06, 0x6d20);
        mdio_write(tp, 0x06, 0x00e6);
        mdio_write(tp, 0x06, 0xe428);
        mdio_write(tp, 0x06, 0xe7e4);
        mdio_write(tp, 0x06, 0x29bf);
        mdio_write(tp, 0x06, 0x25ca);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0x21ee);
        mdio_write(tp, 0x06, 0x8620);
        mdio_write(tp, 0x06, 0x84ee);
        mdio_write(tp, 0x06, 0x8621);
        mdio_write(tp, 0x06, 0x00af);
        mdio_write(tp, 0x06, 0x8178);
        mdio_write(tp, 0x06, 0xa084);
        mdio_write(tp, 0x06, 0x19e0);
        mdio_write(tp, 0x06, 0x8621);
        mdio_write(tp, 0x06, 0xe18b);
        mdio_write(tp, 0x06, 0x341b);
        mdio_write(tp, 0x06, 0x109e);
        mdio_write(tp, 0x06, 0x0602);
        mdio_write(tp, 0x06, 0x2391);
        mdio_write(tp, 0x06, 0xaf82);
        mdio_write(tp, 0x06, 0x5202);
        mdio_write(tp, 0x06, 0x241f);
        mdio_write(tp, 0x06, 0xee86);
        mdio_write(tp, 0x06, 0x2085);
        mdio_write(tp, 0x06, 0xae08);
        mdio_write(tp, 0x06, 0xa085);
        mdio_write(tp, 0x06, 0x02ae);
        mdio_write(tp, 0x06, 0x0302);
        mdio_write(tp, 0x06, 0x2442);
        mdio_write(tp, 0x06, 0xfeef);
        mdio_write(tp, 0x06, 0x96fe);
        mdio_write(tp, 0x06, 0xfdfc);
        mdio_write(tp, 0x06, 0x04f8);
        mdio_write(tp, 0x06, 0xf9fa);
        mdio_write(tp, 0x06, 0xef69);
        mdio_write(tp, 0x06, 0xfad1);
        mdio_write(tp, 0x06, 0x801f);
        mdio_write(tp, 0x06, 0x66e2);
        mdio_write(tp, 0x06, 0xe0ea);
        mdio_write(tp, 0x06, 0xe3e0);
        mdio_write(tp, 0x06, 0xeb5a);
        mdio_write(tp, 0x06, 0xf81e);
        mdio_write(tp, 0x06, 0x20e6);
        mdio_write(tp, 0x06, 0xe0ea);
        mdio_write(tp, 0x06, 0xe5e0);
        mdio_write(tp, 0x06, 0xebd3);
        mdio_write(tp, 0x06, 0x05b3);
        mdio_write(tp, 0x06, 0xfee2);
        mdio_write(tp, 0x06, 0xe07c);
        mdio_write(tp, 0x06, 0xe3e0);
        mdio_write(tp, 0x06, 0x7dad);
        mdio_write(tp, 0x06, 0x3703);
        mdio_write(tp, 0x06, 0x7dff);
        mdio_write(tp, 0x06, 0xff0d);
        mdio_write(tp, 0x06, 0x581c);
        mdio_write(tp, 0x06, 0x55f8);
        mdio_write(tp, 0x06, 0xef46);
        mdio_write(tp, 0x06, 0x0282);
        mdio_write(tp, 0x06, 0xc7ef);
        mdio_write(tp, 0x06, 0x65ef);
        mdio_write(tp, 0x06, 0x54fc);
        mdio_write(tp, 0x06, 0xac30);
        mdio_write(tp, 0x06, 0x2b11);
        mdio_write(tp, 0x06, 0xa188);
        mdio_write(tp, 0x06, 0xcabf);
        mdio_write(tp, 0x06, 0x860e);
        mdio_write(tp, 0x06, 0xef10);
        mdio_write(tp, 0x06, 0x0c11);
        mdio_write(tp, 0x06, 0x1a91);
        mdio_write(tp, 0x06, 0xda19);
        mdio_write(tp, 0x06, 0xdbf8);
        mdio_write(tp, 0x06, 0xef46);
        mdio_write(tp, 0x06, 0x021e);
        mdio_write(tp, 0x06, 0x17ef);
        mdio_write(tp, 0x06, 0x54fc);
        mdio_write(tp, 0x06, 0xad30);
        mdio_write(tp, 0x06, 0x0fef);
        mdio_write(tp, 0x06, 0x5689);
        mdio_write(tp, 0x06, 0xde19);
        mdio_write(tp, 0x06, 0xdfe2);
        mdio_write(tp, 0x06, 0x861f);
        mdio_write(tp, 0x06, 0xbf86);
        mdio_write(tp, 0x06, 0x161a);
        mdio_write(tp, 0x06, 0x90de);
        mdio_write(tp, 0x06, 0xfeef);
        mdio_write(tp, 0x06, 0x96fe);
        mdio_write(tp, 0x06, 0xfdfc);
        mdio_write(tp, 0x06, 0x04ac);
        mdio_write(tp, 0x06, 0x2707);
        mdio_write(tp, 0x06, 0xac37);
        mdio_write(tp, 0x06, 0x071a);
        mdio_write(tp, 0x06, 0x54ae);
        mdio_write(tp, 0x06, 0x11ac);
        mdio_write(tp, 0x06, 0x3707);
        mdio_write(tp, 0x06, 0xae00);
        mdio_write(tp, 0x06, 0x1a54);
        mdio_write(tp, 0x06, 0xac37);
        mdio_write(tp, 0x06, 0x07d0);
        mdio_write(tp, 0x06, 0x01d5);
        mdio_write(tp, 0x06, 0xffff);
        mdio_write(tp, 0x06, 0xae02);
        mdio_write(tp, 0x06, 0xd000);
        mdio_write(tp, 0x06, 0x04f8);
        mdio_write(tp, 0x06, 0xe08b);
        mdio_write(tp, 0x06, 0x83ad);
        mdio_write(tp, 0x06, 0x2444);
        mdio_write(tp, 0x06, 0xe0e0);
        mdio_write(tp, 0x06, 0x22e1);
        mdio_write(tp, 0x06, 0xe023);
        mdio_write(tp, 0x06, 0xad22);
        mdio_write(tp, 0x06, 0x3be0);
        mdio_write(tp, 0x06, 0x8abe);
        mdio_write(tp, 0x06, 0xa000);
        mdio_write(tp, 0x06, 0x0502);
        mdio_write(tp, 0x06, 0x28de);
        mdio_write(tp, 0x06, 0xae42);
        mdio_write(tp, 0x06, 0xa001);
        mdio_write(tp, 0x06, 0x0502);
        mdio_write(tp, 0x06, 0x28f1);
        mdio_write(tp, 0x06, 0xae3a);
        mdio_write(tp, 0x06, 0xa002);
        mdio_write(tp, 0x06, 0x0502);
        mdio_write(tp, 0x06, 0x8344);
        mdio_write(tp, 0x06, 0xae32);
        mdio_write(tp, 0x06, 0xa003);
        mdio_write(tp, 0x06, 0x0502);
        mdio_write(tp, 0x06, 0x299a);
        mdio_write(tp, 0x06, 0xae2a);
        mdio_write(tp, 0x06, 0xa004);
        mdio_write(tp, 0x06, 0x0502);
        mdio_write(tp, 0x06, 0x29ae);
        mdio_write(tp, 0x06, 0xae22);
        mdio_write(tp, 0x06, 0xa005);
        mdio_write(tp, 0x06, 0x0502);
        mdio_write(tp, 0x06, 0x29d7);
        mdio_write(tp, 0x06, 0xae1a);
        mdio_write(tp, 0x06, 0xa006);
        mdio_write(tp, 0x06, 0x0502);
        mdio_write(tp, 0x06, 0x29fe);
        mdio_write(tp, 0x06, 0xae12);
        mdio_write(tp, 0x06, 0xee8a);
        mdio_write(tp, 0x06, 0xc000);
        mdio_write(tp, 0x06, 0xee8a);
        mdio_write(tp, 0x06, 0xc100);
        mdio_write(tp, 0x06, 0xee8a);
        mdio_write(tp, 0x06, 0xc600);
        mdio_write(tp, 0x06, 0xee8a);
        mdio_write(tp, 0x06, 0xbe00);
        mdio_write(tp, 0x06, 0xae00);
        mdio_write(tp, 0x06, 0xfc04);
        mdio_write(tp, 0x06, 0xf802);
        mdio_write(tp, 0x06, 0x2a67);
        mdio_write(tp, 0x06, 0xe0e0);
        mdio_write(tp, 0x06, 0x22e1);
        mdio_write(tp, 0x06, 0xe023);
        mdio_write(tp, 0x06, 0x0d06);
        mdio_write(tp, 0x06, 0x5803);
        mdio_write(tp, 0x06, 0xa002);
        mdio_write(tp, 0x06, 0x02ae);
        mdio_write(tp, 0x06, 0x2da0);
        mdio_write(tp, 0x06, 0x0102);
        mdio_write(tp, 0x06, 0xae2d);
        mdio_write(tp, 0x06, 0xa000);
        mdio_write(tp, 0x06, 0x4de0);
        mdio_write(tp, 0x06, 0xe200);
        mdio_write(tp, 0x06, 0xe1e2);
        mdio_write(tp, 0x06, 0x01ad);
        mdio_write(tp, 0x06, 0x2444);
        mdio_write(tp, 0x06, 0xe08a);
        mdio_write(tp, 0x06, 0xc2e4);
        mdio_write(tp, 0x06, 0x8ac4);
        mdio_write(tp, 0x06, 0xe08a);
        mdio_write(tp, 0x06, 0xc3e4);
        mdio_write(tp, 0x06, 0x8ac5);
        mdio_write(tp, 0x06, 0xee8a);
        mdio_write(tp, 0x06, 0xbe03);
        mdio_write(tp, 0x06, 0xe08b);
        mdio_write(tp, 0x06, 0x83ad);
        mdio_write(tp, 0x06, 0x253a);
        mdio_write(tp, 0x06, 0xee8a);
        mdio_write(tp, 0x06, 0xbe05);
        mdio_write(tp, 0x06, 0xae34);
        mdio_write(tp, 0x06, 0xe08a);
        mdio_write(tp, 0x06, 0xceae);
        mdio_write(tp, 0x06, 0x03e0);
        mdio_write(tp, 0x06, 0x8acf);
        mdio_write(tp, 0x06, 0xe18a);
        mdio_write(tp, 0x06, 0xc249);
        mdio_write(tp, 0x06, 0x05e5);
        mdio_write(tp, 0x06, 0x8ac4);
        mdio_write(tp, 0x06, 0xe18a);
        mdio_write(tp, 0x06, 0xc349);
        mdio_write(tp, 0x06, 0x05e5);
        mdio_write(tp, 0x06, 0x8ac5);
        mdio_write(tp, 0x06, 0xee8a);
        mdio_write(tp, 0x06, 0xbe05);
        mdio_write(tp, 0x06, 0x022a);
        mdio_write(tp, 0x06, 0xb6ac);
        mdio_write(tp, 0x06, 0x2012);
        mdio_write(tp, 0x06, 0x0283);
        mdio_write(tp, 0x06, 0xbaac);
        mdio_write(tp, 0x06, 0x200c);
        mdio_write(tp, 0x06, 0xee8a);
        mdio_write(tp, 0x06, 0xc100);
        mdio_write(tp, 0x06, 0xee8a);
        mdio_write(tp, 0x06, 0xc600);
        mdio_write(tp, 0x06, 0xee8a);
        mdio_write(tp, 0x06, 0xbe02);
        mdio_write(tp, 0x06, 0xfc04);
        mdio_write(tp, 0x06, 0xd000);
        mdio_write(tp, 0x06, 0x0283);
        mdio_write(tp, 0x06, 0xcc59);
        mdio_write(tp, 0x06, 0x0f39);
        mdio_write(tp, 0x06, 0x02aa);
        mdio_write(tp, 0x06, 0x04d0);
        mdio_write(tp, 0x06, 0x01ae);
        mdio_write(tp, 0x06, 0x02d0);
        mdio_write(tp, 0x06, 0x0004);
        mdio_write(tp, 0x06, 0xf9fa);
        mdio_write(tp, 0x06, 0xe2e2);
        mdio_write(tp, 0x06, 0xd2e3);
        mdio_write(tp, 0x06, 0xe2d3);
        mdio_write(tp, 0x06, 0xf95a);
        mdio_write(tp, 0x06, 0xf7e6);
        mdio_write(tp, 0x06, 0xe2d2);
        mdio_write(tp, 0x06, 0xe7e2);
        mdio_write(tp, 0x06, 0xd3e2);
        mdio_write(tp, 0x06, 0xe02c);
        mdio_write(tp, 0x06, 0xe3e0);
        mdio_write(tp, 0x06, 0x2df9);
        mdio_write(tp, 0x06, 0x5be0);
        mdio_write(tp, 0x06, 0x1e30);
        mdio_write(tp, 0x06, 0xe6e0);
        mdio_write(tp, 0x06, 0x2ce7);
        mdio_write(tp, 0x06, 0xe02d);
        mdio_write(tp, 0x06, 0xe2e2);
        mdio_write(tp, 0x06, 0xcce3);
        mdio_write(tp, 0x06, 0xe2cd);
        mdio_write(tp, 0x06, 0xf95a);
        mdio_write(tp, 0x06, 0x0f6a);
        mdio_write(tp, 0x06, 0x50e6);
        mdio_write(tp, 0x06, 0xe2cc);
        mdio_write(tp, 0x06, 0xe7e2);
        mdio_write(tp, 0x06, 0xcde0);
        mdio_write(tp, 0x06, 0xe03c);
        mdio_write(tp, 0x06, 0xe1e0);
        mdio_write(tp, 0x06, 0x3def);
        mdio_write(tp, 0x06, 0x64fd);
        mdio_write(tp, 0x06, 0xe0e2);
        mdio_write(tp, 0x06, 0xcce1);
        mdio_write(tp, 0x06, 0xe2cd);
        mdio_write(tp, 0x06, 0x580f);
        mdio_write(tp, 0x06, 0x5af0);
        mdio_write(tp, 0x06, 0x1e02);
        mdio_write(tp, 0x06, 0xe4e2);
        mdio_write(tp, 0x06, 0xcce5);
        mdio_write(tp, 0x06, 0xe2cd);
        mdio_write(tp, 0x06, 0xfde0);
        mdio_write(tp, 0x06, 0xe02c);
        mdio_write(tp, 0x06, 0xe1e0);
        mdio_write(tp, 0x06, 0x2d59);
        mdio_write(tp, 0x06, 0xe05b);
        mdio_write(tp, 0x06, 0x1f1e);
        mdio_write(tp, 0x06, 0x13e4);
        mdio_write(tp, 0x06, 0xe02c);
        mdio_write(tp, 0x06, 0xe5e0);
        mdio_write(tp, 0x06, 0x2dfd);
        mdio_write(tp, 0x06, 0xe0e2);
        mdio_write(tp, 0x06, 0xd2e1);
        mdio_write(tp, 0x06, 0xe2d3);
        mdio_write(tp, 0x06, 0x58f7);
        mdio_write(tp, 0x06, 0x5a08);
        mdio_write(tp, 0x06, 0x1e02);
        mdio_write(tp, 0x06, 0xe4e2);
        mdio_write(tp, 0x06, 0xd2e5);
        mdio_write(tp, 0x06, 0xe2d3);
        mdio_write(tp, 0x06, 0xef46);
        mdio_write(tp, 0x06, 0xfefd);
        mdio_write(tp, 0x06, 0x04f8);
        mdio_write(tp, 0x06, 0xf9fa);
        mdio_write(tp, 0x06, 0xef69);
        mdio_write(tp, 0x06, 0xe0e0);
        mdio_write(tp, 0x06, 0x22e1);
        mdio_write(tp, 0x06, 0xe023);
        mdio_write(tp, 0x06, 0x58c4);
        mdio_write(tp, 0x06, 0xe18b);
        mdio_write(tp, 0x06, 0x6e1f);
        mdio_write(tp, 0x06, 0x109e);
        mdio_write(tp, 0x06, 0x58e4);
        mdio_write(tp, 0x06, 0x8b6e);
        mdio_write(tp, 0x06, 0xad22);
        mdio_write(tp, 0x06, 0x22ac);
        mdio_write(tp, 0x06, 0x2755);
        mdio_write(tp, 0x06, 0xac26);
        mdio_write(tp, 0x06, 0x02ae);
        mdio_write(tp, 0x06, 0x1ad1);
        mdio_write(tp, 0x06, 0x06bf);
        mdio_write(tp, 0x06, 0x3bba);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1d1);
        mdio_write(tp, 0x06, 0x07bf);
        mdio_write(tp, 0x06, 0x3bbd);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1d1);
        mdio_write(tp, 0x06, 0x07bf);
        mdio_write(tp, 0x06, 0x3bc0);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1ae);
        mdio_write(tp, 0x06, 0x30d1);
        mdio_write(tp, 0x06, 0x03bf);
        mdio_write(tp, 0x06, 0x3bc3);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1d1);
        mdio_write(tp, 0x06, 0x00bf);
        mdio_write(tp, 0x06, 0x3bc6);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1d1);
        mdio_write(tp, 0x06, 0x00bf);
        mdio_write(tp, 0x06, 0x84e9);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1d1);
        mdio_write(tp, 0x06, 0x0fbf);
        mdio_write(tp, 0x06, 0x3bba);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1d1);
        mdio_write(tp, 0x06, 0x01bf);
        mdio_write(tp, 0x06, 0x3bbd);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1d1);
        mdio_write(tp, 0x06, 0x01bf);
        mdio_write(tp, 0x06, 0x3bc0);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1ef);
        mdio_write(tp, 0x06, 0x96fe);
        mdio_write(tp, 0x06, 0xfdfc);
        mdio_write(tp, 0x06, 0x04d1);
        mdio_write(tp, 0x06, 0x00bf);
        mdio_write(tp, 0x06, 0x3bc3);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1d0);
        mdio_write(tp, 0x06, 0x1102);
        mdio_write(tp, 0x06, 0x2bfb);
        mdio_write(tp, 0x06, 0x5903);
        mdio_write(tp, 0x06, 0xef01);
        mdio_write(tp, 0x06, 0xd100);
        mdio_write(tp, 0x06, 0xa000);
        mdio_write(tp, 0x06, 0x02d1);
        mdio_write(tp, 0x06, 0x01bf);
        mdio_write(tp, 0x06, 0x3bc6);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1d1);
        mdio_write(tp, 0x06, 0x11ad);
        mdio_write(tp, 0x06, 0x2002);
        mdio_write(tp, 0x06, 0x0c11);
        mdio_write(tp, 0x06, 0xad21);
        mdio_write(tp, 0x06, 0x020c);
        mdio_write(tp, 0x06, 0x12bf);
        mdio_write(tp, 0x06, 0x84e9);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1ae);
        mdio_write(tp, 0x06, 0xc870);
        mdio_write(tp, 0x06, 0xe426);
        mdio_write(tp, 0x06, 0x0284);
        mdio_write(tp, 0x06, 0xf005);
        mdio_write(tp, 0x06, 0xf8fa);
        mdio_write(tp, 0x06, 0xef69);
        mdio_write(tp, 0x06, 0xe0e2);
        mdio_write(tp, 0x06, 0xfee1);
        mdio_write(tp, 0x06, 0xe2ff);
        mdio_write(tp, 0x06, 0xad2d);
        mdio_write(tp, 0x06, 0x1ae0);
        mdio_write(tp, 0x06, 0xe14e);
        mdio_write(tp, 0x06, 0xe1e1);
        mdio_write(tp, 0x06, 0x4fac);
        mdio_write(tp, 0x06, 0x2d22);
        mdio_write(tp, 0x06, 0xf603);
        mdio_write(tp, 0x06, 0x0203);
        mdio_write(tp, 0x06, 0x3bf7);
        mdio_write(tp, 0x06, 0x03f7);
        mdio_write(tp, 0x06, 0x06bf);
        mdio_write(tp, 0x06, 0x8561);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0x21ae);
        mdio_write(tp, 0x06, 0x11e0);
        mdio_write(tp, 0x06, 0xe14e);
        mdio_write(tp, 0x06, 0xe1e1);
        mdio_write(tp, 0x06, 0x4fad);
        mdio_write(tp, 0x06, 0x2d08);
        mdio_write(tp, 0x06, 0xbf85);
        mdio_write(tp, 0x06, 0x6c02);
        mdio_write(tp, 0x06, 0x2d21);
        mdio_write(tp, 0x06, 0xf606);
        mdio_write(tp, 0x06, 0xef96);
        mdio_write(tp, 0x06, 0xfefc);
        mdio_write(tp, 0x06, 0x04f8);
        mdio_write(tp, 0x06, 0xfaef);
        mdio_write(tp, 0x06, 0x69e0);
        mdio_write(tp, 0x06, 0xe000);
        mdio_write(tp, 0x06, 0xe1e0);
        mdio_write(tp, 0x06, 0x01ad);
        mdio_write(tp, 0x06, 0x271f);
        mdio_write(tp, 0x06, 0xd101);
        mdio_write(tp, 0x06, 0xbf85);
        mdio_write(tp, 0x06, 0x5e02);
        mdio_write(tp, 0x06, 0x2dc1);
        mdio_write(tp, 0x06, 0xe0e0);
        mdio_write(tp, 0x06, 0x20e1);
        mdio_write(tp, 0x06, 0xe021);
        mdio_write(tp, 0x06, 0xad20);
        mdio_write(tp, 0x06, 0x0ed1);
        mdio_write(tp, 0x06, 0x00bf);
        mdio_write(tp, 0x06, 0x855e);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0xc1bf);
        mdio_write(tp, 0x06, 0x3b96);
        mdio_write(tp, 0x06, 0x022d);
        mdio_write(tp, 0x06, 0x21ef);
        mdio_write(tp, 0x06, 0x96fe);
        mdio_write(tp, 0x06, 0xfc04);
        mdio_write(tp, 0x06, 0x00e2);
        mdio_write(tp, 0x06, 0x34a7);
        mdio_write(tp, 0x06, 0x25e5);
        mdio_write(tp, 0x06, 0x0a1d);
        mdio_write(tp, 0x06, 0xe50a);
        mdio_write(tp, 0x06, 0x2ce5);
        mdio_write(tp, 0x06, 0x0a6d);
        mdio_write(tp, 0x06, 0xe50a);
        mdio_write(tp, 0x06, 0x1de5);
        mdio_write(tp, 0x06, 0x0a1c);
        mdio_write(tp, 0x06, 0xe50a);
        mdio_write(tp, 0x06, 0x2da7);
        mdio_write(tp, 0x06, 0x5500);
        mdio_write(tp, 0x05, 0x8b94);
        mdio_write(tp, 0x06, 0x84ec);
        gphy_val = mdio_read(tp, 0x01);
        gphy_val |= BIT_0;
        mdio_write(tp, 0x01, gphy_val);
        mdio_write(tp, 0x00, 0x0005);
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x1f, 0x0005);
        for (i = 0; i < 200; i++) {
            udelay(100);
            gphy_val = mdio_read(tp, 0x00);
            if (gphy_val & BIT_7)
                break;
        }
        mdio_write(tp, 0x1f, 0x0007);
        mdio_write(tp, 0x1e, 0x0023);
        mdio_write(tp, 0x17, 0x0116);
        mdio_write(tp, 0x1f, 0x0007);
        mdio_write(tp, 0x1e, 0x0028);
        mdio_write(tp, 0x15, 0x0010);
        mdio_write(tp, 0x1f, 0x0007);
        mdio_write(tp, 0x1e, 0x0020);
        mdio_write(tp, 0x15, 0x0100);
        mdio_write(tp, 0x1f, 0x0007);
        mdio_write(tp, 0x1e, 0x0041);
        mdio_write(tp, 0x15, 0x0802);
        mdio_write(tp, 0x16, 0x2185);
        mdio_write(tp, 0x1f, 0x0000);
    } else if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
               tp->mcfg == CFG_METHOD_13) {
        mdio_write(tp, 0x1F, 0x0000);
        mdio_write(tp, 0x18, 0x0310);
        mdio_write(tp, 0x1F, 0x0000);
        
        mdelay(20);
        
        mdio_write(tp, 0x1f, 0x0004);
        mdio_write(tp, 0x1f, 0x0004);
        mdio_write(tp, 0x19, 0x7070);
        mdio_write(tp, 0x1c, 0x0600);
        mdio_write(tp, 0x1d, 0x9700);
        mdio_write(tp, 0x1d, 0x7d00);
        mdio_write(tp, 0x1d, 0x6900);
        mdio_write(tp, 0x1d, 0x7d00);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x4899);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x8000);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x4007);
        mdio_write(tp, 0x1d, 0x4400);
        mdio_write(tp, 0x1d, 0x4800);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x6736);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x571f);
        mdio_write(tp, 0x1d, 0x5ffb);
        mdio_write(tp, 0x1d, 0xaa03);
        mdio_write(tp, 0x1d, 0x5b58);
        mdio_write(tp, 0x1d, 0x301e);
        mdio_write(tp, 0x1d, 0x5b64);
        mdio_write(tp, 0x1d, 0xa6fc);
        mdio_write(tp, 0x1d, 0xdcdb);
        mdio_write(tp, 0x1d, 0x0014);
        mdio_write(tp, 0x1d, 0xd9a9);
        mdio_write(tp, 0x1d, 0x0013);
        mdio_write(tp, 0x1d, 0xd16b);
        mdio_write(tp, 0x1d, 0x0011);
        mdio_write(tp, 0x1d, 0xb40e);
        mdio_write(tp, 0x1d, 0xd06b);
        mdio_write(tp, 0x1d, 0x000c);
        mdio_write(tp, 0x1d, 0xb206);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c00);
        mdio_write(tp, 0x1d, 0x301a);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5801);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c04);
        mdio_write(tp, 0x1d, 0x301e);
        mdio_write(tp, 0x1d, 0x314d);
        mdio_write(tp, 0x1d, 0x31f0);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4c20);
        mdio_write(tp, 0x1d, 0x6004);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x4833);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c08);
        mdio_write(tp, 0x1d, 0x8300);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x6600);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xb90c);
        mdio_write(tp, 0x1d, 0x30d3);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4de0);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x6736);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x300b);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4c60);
        mdio_write(tp, 0x1d, 0x6803);
        mdio_write(tp, 0x1d, 0x6520);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xaf03);
        mdio_write(tp, 0x1d, 0x6015);
        mdio_write(tp, 0x1d, 0x3059);
        mdio_write(tp, 0x1d, 0x6017);
        mdio_write(tp, 0x1d, 0x57e0);
        mdio_write(tp, 0x1d, 0x580c);
        mdio_write(tp, 0x1d, 0x588c);
        mdio_write(tp, 0x1d, 0x7ffc);
        mdio_write(tp, 0x1d, 0x5fa3);
        mdio_write(tp, 0x1d, 0x4827);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c10);
        mdio_write(tp, 0x1d, 0x8400);
        mdio_write(tp, 0x1d, 0x7c30);
        mdio_write(tp, 0x1d, 0x6020);
        mdio_write(tp, 0x1d, 0x48bf);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0xad09);
        mdio_write(tp, 0x1d, 0x7c03);
        mdio_write(tp, 0x1d, 0x5c03);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x4400);
        mdio_write(tp, 0x1d, 0xad2c);
        mdio_write(tp, 0x1d, 0xd6cf);
        mdio_write(tp, 0x1d, 0x0002);
        mdio_write(tp, 0x1d, 0x80f4);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4c80);
        mdio_write(tp, 0x1d, 0x7c20);
        mdio_write(tp, 0x1d, 0x5c20);
        mdio_write(tp, 0x1d, 0x481e);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c02);
        mdio_write(tp, 0x1d, 0xad0a);
        mdio_write(tp, 0x1d, 0x7c03);
        mdio_write(tp, 0x1d, 0x5c03);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x4400);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x8d02);
        mdio_write(tp, 0x1d, 0x4401);
        mdio_write(tp, 0x1d, 0x81f4);
        mdio_write(tp, 0x1d, 0x3114);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d00);
        mdio_write(tp, 0x1d, 0x4832);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c10);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0xa4b7);
        mdio_write(tp, 0x1d, 0xd9b3);
        mdio_write(tp, 0x1d, 0xfffe);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d20);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6200);
        mdio_write(tp, 0x1d, 0x3045);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d40);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x4401);
        mdio_write(tp, 0x1d, 0x5210);
        mdio_write(tp, 0x1d, 0x4833);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x4c08);
        mdio_write(tp, 0x1d, 0x8300);
        mdio_write(tp, 0x1d, 0x5f80);
        mdio_write(tp, 0x1d, 0x55e0);
        mdio_write(tp, 0x1d, 0xc06f);
        mdio_write(tp, 0x1d, 0x0005);
        mdio_write(tp, 0x1d, 0xd9b3);
        mdio_write(tp, 0x1d, 0xfffd);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x6040);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d60);
        mdio_write(tp, 0x1d, 0x57e0);
        mdio_write(tp, 0x1d, 0x4814);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x4c04);
        mdio_write(tp, 0x1d, 0x8200);
        mdio_write(tp, 0x1d, 0x7c03);
        mdio_write(tp, 0x1d, 0x5c03);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xad02);
        mdio_write(tp, 0x1d, 0x4400);
        mdio_write(tp, 0x1d, 0xc0e9);
        mdio_write(tp, 0x1d, 0x0003);
        mdio_write(tp, 0x1d, 0xadd8);
        mdio_write(tp, 0x1d, 0x30c6);
        mdio_write(tp, 0x1d, 0x3078);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4dc0);
        mdio_write(tp, 0x1d, 0x6730);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xd09d);
        mdio_write(tp, 0x1d, 0x0002);
        mdio_write(tp, 0x1d, 0xb4fe);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d80);
        mdio_write(tp, 0x1d, 0x6802);
        mdio_write(tp, 0x1d, 0x6600);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x486c);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x9503);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6200);
        mdio_write(tp, 0x1d, 0x571f);
        mdio_write(tp, 0x1d, 0x5fbb);
        mdio_write(tp, 0x1d, 0xaa03);
        mdio_write(tp, 0x1d, 0x5b58);
        mdio_write(tp, 0x1d, 0x30e9);
        mdio_write(tp, 0x1d, 0x5b64);
        mdio_write(tp, 0x1d, 0xcdab);
        mdio_write(tp, 0x1d, 0xff5b);
        mdio_write(tp, 0x1d, 0xcd8d);
        mdio_write(tp, 0x1d, 0xff59);
        mdio_write(tp, 0x1d, 0xd96b);
        mdio_write(tp, 0x1d, 0xff57);
        mdio_write(tp, 0x1d, 0xd0a0);
        mdio_write(tp, 0x1d, 0xffdb);
        mdio_write(tp, 0x1d, 0xcba0);
        mdio_write(tp, 0x1d, 0x0003);
        mdio_write(tp, 0x1d, 0x80f0);
        mdio_write(tp, 0x1d, 0x30f6);
        mdio_write(tp, 0x1d, 0x3109);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ce0);
        mdio_write(tp, 0x1d, 0x7d30);
        mdio_write(tp, 0x1d, 0x6530);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7ce0);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x4832);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c08);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6008);
        mdio_write(tp, 0x1d, 0x8300);
        mdio_write(tp, 0x1d, 0xb902);
        mdio_write(tp, 0x1d, 0x30d3);
        mdio_write(tp, 0x1d, 0x308f);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4da0);
        mdio_write(tp, 0x1d, 0x57a0);
        mdio_write(tp, 0x1d, 0x590c);
        mdio_write(tp, 0x1d, 0x5fa2);
        mdio_write(tp, 0x1d, 0xcba4);
        mdio_write(tp, 0x1d, 0x0005);
        mdio_write(tp, 0x1d, 0xcd8d);
        mdio_write(tp, 0x1d, 0x0003);
        mdio_write(tp, 0x1d, 0x80fc);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ca0);
        mdio_write(tp, 0x1d, 0xb603);
        mdio_write(tp, 0x1d, 0x7c10);
        mdio_write(tp, 0x1d, 0x6010);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x541f);
        mdio_write(tp, 0x1d, 0x7ffc);
        mdio_write(tp, 0x1d, 0x5fb3);
        mdio_write(tp, 0x1d, 0x9403);
        mdio_write(tp, 0x1d, 0x7c03);
        mdio_write(tp, 0x1d, 0x5c03);
        mdio_write(tp, 0x1d, 0xaa05);
        mdio_write(tp, 0x1d, 0x7c80);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x5b58);
        mdio_write(tp, 0x1d, 0x3128);
        mdio_write(tp, 0x1d, 0x7c80);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x5b64);
        mdio_write(tp, 0x1d, 0x4827);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c10);
        mdio_write(tp, 0x1d, 0x8400);
        mdio_write(tp, 0x1d, 0x7c10);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x4824);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c04);
        mdio_write(tp, 0x1d, 0x8200);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4cc0);
        mdio_write(tp, 0x1d, 0x7d00);
        mdio_write(tp, 0x1d, 0x6400);
        mdio_write(tp, 0x1d, 0x7ffc);
        mdio_write(tp, 0x1d, 0x5fbb);
        mdio_write(tp, 0x1d, 0x4824);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c04);
        mdio_write(tp, 0x1d, 0x8200);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6a00);
        mdio_write(tp, 0x1d, 0x4824);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c04);
        mdio_write(tp, 0x1d, 0x8200);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x30f6);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e00);
        mdio_write(tp, 0x1d, 0x4007);
        mdio_write(tp, 0x1d, 0x4400);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x6736);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x570f);
        mdio_write(tp, 0x1d, 0x5fff);
        mdio_write(tp, 0x1d, 0xaa03);
        mdio_write(tp, 0x1d, 0x585b);
        mdio_write(tp, 0x1d, 0x315c);
        mdio_write(tp, 0x1d, 0x5867);
        mdio_write(tp, 0x1d, 0x9402);
        mdio_write(tp, 0x1d, 0x6200);
        mdio_write(tp, 0x1d, 0xcda3);
        mdio_write(tp, 0x1d, 0x009d);
        mdio_write(tp, 0x1d, 0xcd85);
        mdio_write(tp, 0x1d, 0x009b);
        mdio_write(tp, 0x1d, 0xd96b);
        mdio_write(tp, 0x1d, 0x0099);
        mdio_write(tp, 0x1d, 0x96e9);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x6736);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e20);
        mdio_write(tp, 0x1d, 0x96e4);
        mdio_write(tp, 0x1d, 0x8b04);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x5008);
        mdio_write(tp, 0x1d, 0xab03);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x5000);
        mdio_write(tp, 0x1d, 0x6801);
        mdio_write(tp, 0x1d, 0x6776);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xdb7c);
        mdio_write(tp, 0x1d, 0xfff0);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7fe1);
        mdio_write(tp, 0x1d, 0x4e40);
        mdio_write(tp, 0x1d, 0x4837);
        mdio_write(tp, 0x1d, 0x4418);
        mdio_write(tp, 0x1d, 0x41c7);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e40);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x8fc9);
        mdio_write(tp, 0x1d, 0xd2a0);
        mdio_write(tp, 0x1d, 0x004a);
        mdio_write(tp, 0x1d, 0x9203);
        mdio_write(tp, 0x1d, 0xa041);
        mdio_write(tp, 0x1d, 0x3184);
        mdio_write(tp, 0x1d, 0x7fe1);
        mdio_write(tp, 0x1d, 0x4e60);
        mdio_write(tp, 0x1d, 0x489c);
        mdio_write(tp, 0x1d, 0x4628);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e60);
        mdio_write(tp, 0x1d, 0x7e28);
        mdio_write(tp, 0x1d, 0x4628);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c00);
        mdio_write(tp, 0x1d, 0x41e8);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x8fb0);
        mdio_write(tp, 0x1d, 0xb241);
        mdio_write(tp, 0x1d, 0xa02a);
        mdio_write(tp, 0x1d, 0x319d);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ea0);
        mdio_write(tp, 0x1d, 0x7c02);
        mdio_write(tp, 0x1d, 0x4402);
        mdio_write(tp, 0x1d, 0x4448);
        mdio_write(tp, 0x1d, 0x4894);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c03);
        mdio_write(tp, 0x1d, 0x4824);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c07);
        mdio_write(tp, 0x1d, 0x41ef);
        mdio_write(tp, 0x1d, 0x41ff);
        mdio_write(tp, 0x1d, 0x4891);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c07);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c17);
        mdio_write(tp, 0x1d, 0x8400);
        mdio_write(tp, 0x1d, 0x8ef8);
        mdio_write(tp, 0x1d, 0x41c7);
        mdio_write(tp, 0x1d, 0x8f95);
        mdio_write(tp, 0x1d, 0x92d5);
        mdio_write(tp, 0x1d, 0xa10f);
        mdio_write(tp, 0x1d, 0xd480);
        mdio_write(tp, 0x1d, 0x0008);
        mdio_write(tp, 0x1d, 0xd580);
        mdio_write(tp, 0x1d, 0xffb9);
        mdio_write(tp, 0x1d, 0xa202);
        mdio_write(tp, 0x1d, 0x31b8);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x4404);
        mdio_write(tp, 0x1d, 0x31b8);
        mdio_write(tp, 0x1d, 0xd484);
        mdio_write(tp, 0x1d, 0xfff3);
        mdio_write(tp, 0x1d, 0xd484);
        mdio_write(tp, 0x1d, 0xfff1);
        mdio_write(tp, 0x1d, 0x314d);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ee0);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x4488);
        mdio_write(tp, 0x1d, 0x41cf);
        mdio_write(tp, 0x1d, 0x314d);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ec0);
        mdio_write(tp, 0x1d, 0x48f3);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c09);
        mdio_write(tp, 0x1d, 0x4508);
        mdio_write(tp, 0x1d, 0x41c7);
        mdio_write(tp, 0x1d, 0x8f24);
        mdio_write(tp, 0x1d, 0xd218);
        mdio_write(tp, 0x1d, 0x0022);
        mdio_write(tp, 0x1d, 0xd2a4);
        mdio_write(tp, 0x1d, 0xff9f);
        mdio_write(tp, 0x1d, 0x31d9);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e80);
        mdio_write(tp, 0x1d, 0x4832);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c11);
        mdio_write(tp, 0x1d, 0x4428);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5440);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5801);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c04);
        mdio_write(tp, 0x1d, 0x41e8);
        mdio_write(tp, 0x1d, 0xa4b3);
        mdio_write(tp, 0x1d, 0x31ee);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x6736);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x570f);
        mdio_write(tp, 0x1d, 0x5fff);
        mdio_write(tp, 0x1d, 0xaa03);
        mdio_write(tp, 0x1d, 0x585b);
        mdio_write(tp, 0x1d, 0x31fa);
        mdio_write(tp, 0x1d, 0x5867);
        mdio_write(tp, 0x1d, 0xbcf6);
        mdio_write(tp, 0x1d, 0x300b);
        mdio_write(tp, 0x1d, 0x300b);
        mdio_write(tp, 0x1d, 0x314d);
        mdio_write(tp, 0x1f, 0x0004);
        mdio_write(tp, 0x1c, 0x0200);
        mdio_write(tp, 0x19, 0x7030);
        mdio_write(tp, 0x1f, 0x0000);
        
        
    } else if (tp->mcfg == CFG_METHOD_14) {
        mdio_write(tp, 0x1F, 0x0000);
        mdio_write(tp, 0x18, 0x0310);
        mdio_write(tp, 0x1F, 0x0000);
        
        mdelay(20);
        
        mdio_write(tp, 0x1f, 0x0004);
        mdio_write(tp, 0x1f, 0x0004);
        mdio_write(tp, 0x19, 0x7070);
        mdio_write(tp, 0x1c, 0x0600);
        mdio_write(tp, 0x1d, 0x9700);
        mdio_write(tp, 0x1d, 0x7d00);
        mdio_write(tp, 0x1d, 0x6900);
        mdio_write(tp, 0x1d, 0x7d00);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x4899);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x8000);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x4007);
        mdio_write(tp, 0x1d, 0x4400);
        mdio_write(tp, 0x1d, 0x4800);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x6736);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x571f);
        mdio_write(tp, 0x1d, 0x5ffb);
        mdio_write(tp, 0x1d, 0xaa03);
        mdio_write(tp, 0x1d, 0x5b58);
        mdio_write(tp, 0x1d, 0x301e);
        mdio_write(tp, 0x1d, 0x5b64);
        mdio_write(tp, 0x1d, 0xa6fc);
        mdio_write(tp, 0x1d, 0xdcdb);
        mdio_write(tp, 0x1d, 0x0015);
        mdio_write(tp, 0x1d, 0xb915);
        mdio_write(tp, 0x1d, 0xb511);
        mdio_write(tp, 0x1d, 0xd16b);
        mdio_write(tp, 0x1d, 0x000f);
        mdio_write(tp, 0x1d, 0xb40f);
        mdio_write(tp, 0x1d, 0xd06b);
        mdio_write(tp, 0x1d, 0x000d);
        mdio_write(tp, 0x1d, 0xb206);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c00);
        mdio_write(tp, 0x1d, 0x301a);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5801);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c04);
        mdio_write(tp, 0x1d, 0x301e);
        mdio_write(tp, 0x1d, 0x3079);
        mdio_write(tp, 0x1d, 0x30f1);
        mdio_write(tp, 0x1d, 0x3199);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4c60);
        mdio_write(tp, 0x1d, 0x6803);
        mdio_write(tp, 0x1d, 0x6420);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xaf03);
        mdio_write(tp, 0x1d, 0x6015);
        mdio_write(tp, 0x1d, 0x3040);
        mdio_write(tp, 0x1d, 0x6017);
        mdio_write(tp, 0x1d, 0x57e0);
        mdio_write(tp, 0x1d, 0x580c);
        mdio_write(tp, 0x1d, 0x588c);
        mdio_write(tp, 0x1d, 0x5fa3);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x4827);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c10);
        mdio_write(tp, 0x1d, 0x8400);
        mdio_write(tp, 0x1d, 0x7c30);
        mdio_write(tp, 0x1d, 0x6020);
        mdio_write(tp, 0x1d, 0x48bf);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0xd6cf);
        mdio_write(tp, 0x1d, 0x0002);
        mdio_write(tp, 0x1d, 0x80fe);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4c80);
        mdio_write(tp, 0x1d, 0x7c20);
        mdio_write(tp, 0x1d, 0x5c20);
        mdio_write(tp, 0x1d, 0x481e);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c02);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x81ff);
        mdio_write(tp, 0x1d, 0x30ba);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d00);
        mdio_write(tp, 0x1d, 0x4832);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c10);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0xa4cc);
        mdio_write(tp, 0x1d, 0xd9b3);
        mdio_write(tp, 0x1d, 0xfffe);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d20);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6200);
        mdio_write(tp, 0x1d, 0x300b);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4dc0);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xd09d);
        mdio_write(tp, 0x1d, 0x0002);
        mdio_write(tp, 0x1d, 0xb4fe);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d80);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x6004);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x6802);
        mdio_write(tp, 0x1d, 0x6720);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x486c);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x9503);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6200);
        mdio_write(tp, 0x1d, 0x571f);
        mdio_write(tp, 0x1d, 0x5fbb);
        mdio_write(tp, 0x1d, 0xaa03);
        mdio_write(tp, 0x1d, 0x5b58);
        mdio_write(tp, 0x1d, 0x3092);
        mdio_write(tp, 0x1d, 0x5b64);
        mdio_write(tp, 0x1d, 0xcdab);
        mdio_write(tp, 0x1d, 0xff78);
        mdio_write(tp, 0x1d, 0xcd8d);
        mdio_write(tp, 0x1d, 0xff76);
        mdio_write(tp, 0x1d, 0xd96b);
        mdio_write(tp, 0x1d, 0xff74);
        mdio_write(tp, 0x1d, 0xd0a0);
        mdio_write(tp, 0x1d, 0xffd9);
        mdio_write(tp, 0x1d, 0xcba0);
        mdio_write(tp, 0x1d, 0x0003);
        mdio_write(tp, 0x1d, 0x80f0);
        mdio_write(tp, 0x1d, 0x309f);
        mdio_write(tp, 0x1d, 0x30ac);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ce0);
        mdio_write(tp, 0x1d, 0x4832);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c08);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6008);
        mdio_write(tp, 0x1d, 0x8300);
        mdio_write(tp, 0x1d, 0xb902);
        mdio_write(tp, 0x1d, 0x3079);
        mdio_write(tp, 0x1d, 0x3061);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4da0);
        mdio_write(tp, 0x1d, 0x6400);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x57a0);
        mdio_write(tp, 0x1d, 0x590c);
        mdio_write(tp, 0x1d, 0x5fa3);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xcba4);
        mdio_write(tp, 0x1d, 0x0004);
        mdio_write(tp, 0x1d, 0xcd8d);
        mdio_write(tp, 0x1d, 0x0002);
        mdio_write(tp, 0x1d, 0x80fc);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ca0);
        mdio_write(tp, 0x1d, 0xb603);
        mdio_write(tp, 0x1d, 0x7c10);
        mdio_write(tp, 0x1d, 0x6010);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x541f);
        mdio_write(tp, 0x1d, 0x5fb3);
        mdio_write(tp, 0x1d, 0xaa05);
        mdio_write(tp, 0x1d, 0x7c80);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x5b58);
        mdio_write(tp, 0x1d, 0x30ca);
        mdio_write(tp, 0x1d, 0x7c80);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x5b64);
        mdio_write(tp, 0x1d, 0x4824);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c04);
        mdio_write(tp, 0x1d, 0x8200);
        mdio_write(tp, 0x1d, 0x4827);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c10);
        mdio_write(tp, 0x1d, 0x8400);
        mdio_write(tp, 0x1d, 0x7c10);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4cc0);
        mdio_write(tp, 0x1d, 0x5fbb);
        mdio_write(tp, 0x1d, 0x4824);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c04);
        mdio_write(tp, 0x1d, 0x8200);
        mdio_write(tp, 0x1d, 0x7ce0);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x6720);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6a00);
        mdio_write(tp, 0x1d, 0x4824);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c04);
        mdio_write(tp, 0x1d, 0x8200);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x309f);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e00);
        mdio_write(tp, 0x1d, 0x4007);
        mdio_write(tp, 0x1d, 0x4400);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x6736);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x570f);
        mdio_write(tp, 0x1d, 0x5fff);
        mdio_write(tp, 0x1d, 0xaa03);
        mdio_write(tp, 0x1d, 0x585b);
        mdio_write(tp, 0x1d, 0x3100);
        mdio_write(tp, 0x1d, 0x5867);
        mdio_write(tp, 0x1d, 0x9403);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6200);
        mdio_write(tp, 0x1d, 0xcda3);
        mdio_write(tp, 0x1d, 0x002d);
        mdio_write(tp, 0x1d, 0xcd85);
        mdio_write(tp, 0x1d, 0x002b);
        mdio_write(tp, 0x1d, 0xd96b);
        mdio_write(tp, 0x1d, 0x0029);
        mdio_write(tp, 0x1d, 0x9629);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x6736);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x9624);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e20);
        mdio_write(tp, 0x1d, 0x8b04);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x5008);
        mdio_write(tp, 0x1d, 0xab03);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x5000);
        mdio_write(tp, 0x1d, 0x6801);
        mdio_write(tp, 0x1d, 0x6776);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xdb7c);
        mdio_write(tp, 0x1d, 0xffee);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7fe1);
        mdio_write(tp, 0x1d, 0x4e40);
        mdio_write(tp, 0x1d, 0x4837);
        mdio_write(tp, 0x1d, 0x4418);
        mdio_write(tp, 0x1d, 0x41c7);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e40);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x8f07);
        mdio_write(tp, 0x1d, 0xd2a0);
        mdio_write(tp, 0x1d, 0x004c);
        mdio_write(tp, 0x1d, 0x9205);
        mdio_write(tp, 0x1d, 0xa043);
        mdio_write(tp, 0x1d, 0x312b);
        mdio_write(tp, 0x1d, 0x300b);
        mdio_write(tp, 0x1d, 0x30f1);
        mdio_write(tp, 0x1d, 0x7fe1);
        mdio_write(tp, 0x1d, 0x4e60);
        mdio_write(tp, 0x1d, 0x489c);
        mdio_write(tp, 0x1d, 0x4628);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e60);
        mdio_write(tp, 0x1d, 0x7e28);
        mdio_write(tp, 0x1d, 0x4628);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c00);
        mdio_write(tp, 0x1d, 0x41e8);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x8fec);
        mdio_write(tp, 0x1d, 0xb241);
        mdio_write(tp, 0x1d, 0xa02a);
        mdio_write(tp, 0x1d, 0x3146);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ea0);
        mdio_write(tp, 0x1d, 0x7c02);
        mdio_write(tp, 0x1d, 0x4402);
        mdio_write(tp, 0x1d, 0x4448);
        mdio_write(tp, 0x1d, 0x4894);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c03);
        mdio_write(tp, 0x1d, 0x4824);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c07);
        mdio_write(tp, 0x1d, 0x41ef);
        mdio_write(tp, 0x1d, 0x41ff);
        mdio_write(tp, 0x1d, 0x4891);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c07);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c17);
        mdio_write(tp, 0x1d, 0x8400);
        mdio_write(tp, 0x1d, 0x8ef8);
        mdio_write(tp, 0x1d, 0x41c7);
        mdio_write(tp, 0x1d, 0x8fd1);
        mdio_write(tp, 0x1d, 0x92d5);
        mdio_write(tp, 0x1d, 0xa10f);
        mdio_write(tp, 0x1d, 0xd480);
        mdio_write(tp, 0x1d, 0x0008);
        mdio_write(tp, 0x1d, 0xd580);
        mdio_write(tp, 0x1d, 0xffb7);
        mdio_write(tp, 0x1d, 0xa202);
        mdio_write(tp, 0x1d, 0x3161);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x4404);
        mdio_write(tp, 0x1d, 0x3161);
        mdio_write(tp, 0x1d, 0xd484);
        mdio_write(tp, 0x1d, 0xfff3);
        mdio_write(tp, 0x1d, 0xd484);
        mdio_write(tp, 0x1d, 0xfff1);
        mdio_write(tp, 0x1d, 0x30f1);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ee0);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x4488);
        mdio_write(tp, 0x1d, 0x41cf);
        mdio_write(tp, 0x1d, 0x30f1);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ec0);
        mdio_write(tp, 0x1d, 0x48f3);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c09);
        mdio_write(tp, 0x1d, 0x4508);
        mdio_write(tp, 0x1d, 0x41c7);
        mdio_write(tp, 0x1d, 0x8fb0);
        mdio_write(tp, 0x1d, 0xd218);
        mdio_write(tp, 0x1d, 0xffae);
        mdio_write(tp, 0x1d, 0xd2a4);
        mdio_write(tp, 0x1d, 0xff9d);
        mdio_write(tp, 0x1d, 0x3182);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e80);
        mdio_write(tp, 0x1d, 0x4832);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c11);
        mdio_write(tp, 0x1d, 0x4428);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5440);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5801);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c04);
        mdio_write(tp, 0x1d, 0x41e8);
        mdio_write(tp, 0x1d, 0xa4b3);
        mdio_write(tp, 0x1d, 0x3197);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4f20);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x6736);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x570f);
        mdio_write(tp, 0x1d, 0x5fff);
        mdio_write(tp, 0x1d, 0xaa03);
        mdio_write(tp, 0x1d, 0x585b);
        mdio_write(tp, 0x1d, 0x31a5);
        mdio_write(tp, 0x1d, 0x5867);
        mdio_write(tp, 0x1d, 0xbcf4);
        mdio_write(tp, 0x1d, 0x300b);
        mdio_write(tp, 0x1f, 0x0004);
        mdio_write(tp, 0x1c, 0x0200);
        mdio_write(tp, 0x19, 0x7030);
        mdio_write(tp, 0x1f, 0x0000);
    } else if (tp->mcfg == CFG_METHOD_16) {
        mdio_write(tp, 0x1F, 0x0000);
        mdio_write(tp, 0x18, 0x0310);
        
        mdelay(20);
        
        mdio_write(tp, 0x1f, 0x0004);
        mdio_write(tp, 0x1f, 0x0004);
        mdio_write(tp, 0x19, 0x7070);
        mdio_write(tp, 0x1c, 0x0600);
        mdio_write(tp, 0x1d, 0x9700);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x4007);
        mdio_write(tp, 0x1d, 0x4400);
        mdio_write(tp, 0x1d, 0x4800);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x673e);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x571f);
        mdio_write(tp, 0x1d, 0x5ffb);
        mdio_write(tp, 0x1d, 0xaa04);
        mdio_write(tp, 0x1d, 0x5b58);
        mdio_write(tp, 0x1d, 0x6100);
        mdio_write(tp, 0x1d, 0x3016);
        mdio_write(tp, 0x1d, 0x5b64);
        mdio_write(tp, 0x1d, 0x6080);
        mdio_write(tp, 0x1d, 0xa6fa);
        mdio_write(tp, 0x1d, 0xdcdb);
        mdio_write(tp, 0x1d, 0x0015);
        mdio_write(tp, 0x1d, 0xb915);
        mdio_write(tp, 0x1d, 0xb511);
        mdio_write(tp, 0x1d, 0xd16b);
        mdio_write(tp, 0x1d, 0x000f);
        mdio_write(tp, 0x1d, 0xb40f);
        mdio_write(tp, 0x1d, 0xd06b);
        mdio_write(tp, 0x1d, 0x000d);
        mdio_write(tp, 0x1d, 0xb206);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c00);
        mdio_write(tp, 0x1d, 0x3010);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5801);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c04);
        mdio_write(tp, 0x1d, 0x3016);
        mdio_write(tp, 0x1d, 0x307e);
        mdio_write(tp, 0x1d, 0x30f4);
        mdio_write(tp, 0x1d, 0x319f);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4c60);
        mdio_write(tp, 0x1d, 0x6803);
        mdio_write(tp, 0x1d, 0x7d00);
        mdio_write(tp, 0x1d, 0x6900);
        mdio_write(tp, 0x1d, 0x6520);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xaf03);
        mdio_write(tp, 0x1d, 0x6115);
        mdio_write(tp, 0x1d, 0x303a);
        mdio_write(tp, 0x1d, 0x6097);
        mdio_write(tp, 0x1d, 0x57e0);
        mdio_write(tp, 0x1d, 0x580c);
        mdio_write(tp, 0x1d, 0x588c);
        mdio_write(tp, 0x1d, 0x5f80);
        mdio_write(tp, 0x1d, 0x4827);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c10);
        mdio_write(tp, 0x1d, 0x8400);
        mdio_write(tp, 0x1d, 0x7c30);
        mdio_write(tp, 0x1d, 0x6020);
        mdio_write(tp, 0x1d, 0x48bf);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0xb802);
        mdio_write(tp, 0x1d, 0x3053);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6808);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7c10);
        mdio_write(tp, 0x1d, 0x6810);
        mdio_write(tp, 0x1d, 0xd6cf);
        mdio_write(tp, 0x1d, 0x0002);
        mdio_write(tp, 0x1d, 0x80fe);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4c80);
        mdio_write(tp, 0x1d, 0x7c10);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7c23);
        mdio_write(tp, 0x1d, 0x5c23);
        mdio_write(tp, 0x1d, 0x481e);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c02);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x81ff);
        mdio_write(tp, 0x1d, 0x30c1);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d00);
        mdio_write(tp, 0x1d, 0x4832);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c10);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0xa4bd);
        mdio_write(tp, 0x1d, 0xd9b3);
        mdio_write(tp, 0x1d, 0x00fe);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d20);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6200);
        mdio_write(tp, 0x1d, 0x3001);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4dc0);
        mdio_write(tp, 0x1d, 0xd09d);
        mdio_write(tp, 0x1d, 0x0002);
        mdio_write(tp, 0x1d, 0xb4fe);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4d80);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x6004);
        mdio_write(tp, 0x1d, 0x6802);
        mdio_write(tp, 0x1d, 0x6728);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x486c);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x9503);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6200);
        mdio_write(tp, 0x1d, 0x571f);
        mdio_write(tp, 0x1d, 0x5fbb);
        mdio_write(tp, 0x1d, 0xaa05);
        mdio_write(tp, 0x1d, 0x5b58);
        mdio_write(tp, 0x1d, 0x7d80);
        mdio_write(tp, 0x1d, 0x6100);
        mdio_write(tp, 0x1d, 0x309a);
        mdio_write(tp, 0x1d, 0x5b64);
        mdio_write(tp, 0x1d, 0x7d80);
        mdio_write(tp, 0x1d, 0x6080);
        mdio_write(tp, 0x1d, 0xcdab);
        mdio_write(tp, 0x1d, 0x0058);
        mdio_write(tp, 0x1d, 0xcd8d);
        mdio_write(tp, 0x1d, 0x0056);
        mdio_write(tp, 0x1d, 0xd96b);
        mdio_write(tp, 0x1d, 0x0054);
        mdio_write(tp, 0x1d, 0xd0a0);
        mdio_write(tp, 0x1d, 0x00d8);
        mdio_write(tp, 0x1d, 0xcba0);
        mdio_write(tp, 0x1d, 0x0003);
        mdio_write(tp, 0x1d, 0x80ec);
        mdio_write(tp, 0x1d, 0x30a7);
        mdio_write(tp, 0x1d, 0x30b4);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ce0);
        mdio_write(tp, 0x1d, 0x4832);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c08);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6008);
        mdio_write(tp, 0x1d, 0x8300);
        mdio_write(tp, 0x1d, 0xb902);
        mdio_write(tp, 0x1d, 0x307e);
        mdio_write(tp, 0x1d, 0x3068);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4da0);
        mdio_write(tp, 0x1d, 0x6628);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x56a0);
        mdio_write(tp, 0x1d, 0x590c);
        mdio_write(tp, 0x1d, 0x5fa0);
        mdio_write(tp, 0x1d, 0xcba4);
        mdio_write(tp, 0x1d, 0x0004);
        mdio_write(tp, 0x1d, 0xcd8d);
        mdio_write(tp, 0x1d, 0x0002);
        mdio_write(tp, 0x1d, 0x80fc);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ca0);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x6408);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7d00);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0xb603);
        mdio_write(tp, 0x1d, 0x7c10);
        mdio_write(tp, 0x1d, 0x6010);
        mdio_write(tp, 0x1d, 0x7d1f);
        mdio_write(tp, 0x1d, 0x551f);
        mdio_write(tp, 0x1d, 0x5fb3);
        mdio_write(tp, 0x1d, 0xaa05);
        mdio_write(tp, 0x1d, 0x7c80);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x5b58);
        mdio_write(tp, 0x1d, 0x30d7);
        mdio_write(tp, 0x1d, 0x7c80);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x5b64);
        mdio_write(tp, 0x1d, 0x4827);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c10);
        mdio_write(tp, 0x1d, 0x8400);
        mdio_write(tp, 0x1d, 0x7c10);
        mdio_write(tp, 0x1d, 0x6000);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4cc0);
        mdio_write(tp, 0x1d, 0x7d00);
        mdio_write(tp, 0x1d, 0x6400);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x5fbb);
        mdio_write(tp, 0x1d, 0x4824);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c00);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c04);
        mdio_write(tp, 0x1d, 0x8200);
        mdio_write(tp, 0x1d, 0x7ce0);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x7d00);
        mdio_write(tp, 0x1d, 0x6500);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x30a7);
        mdio_write(tp, 0x1d, 0x3001);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e00);
        mdio_write(tp, 0x1d, 0x4007);
        mdio_write(tp, 0x1d, 0x4400);
        mdio_write(tp, 0x1d, 0x5310);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x673e);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x570f);
        mdio_write(tp, 0x1d, 0x5fff);
        mdio_write(tp, 0x1d, 0xaa05);
        mdio_write(tp, 0x1d, 0x585b);
        mdio_write(tp, 0x1d, 0x7d80);
        mdio_write(tp, 0x1d, 0x6100);
        mdio_write(tp, 0x1d, 0x3107);
        mdio_write(tp, 0x1d, 0x5867);
        mdio_write(tp, 0x1d, 0x7d80);
        mdio_write(tp, 0x1d, 0x6080);
        mdio_write(tp, 0x1d, 0x9403);
        mdio_write(tp, 0x1d, 0x7e00);
        mdio_write(tp, 0x1d, 0x6200);
        mdio_write(tp, 0x1d, 0xcda3);
        mdio_write(tp, 0x1d, 0x00e8);
        mdio_write(tp, 0x1d, 0xcd85);
        mdio_write(tp, 0x1d, 0x00e6);
        mdio_write(tp, 0x1d, 0xd96b);
        mdio_write(tp, 0x1d, 0x00e4);
        mdio_write(tp, 0x1d, 0x96e4);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x673e);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e20);
        mdio_write(tp, 0x1d, 0x96dd);
        mdio_write(tp, 0x1d, 0x8b04);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x5008);
        mdio_write(tp, 0x1d, 0xab03);
        mdio_write(tp, 0x1d, 0x7c08);
        mdio_write(tp, 0x1d, 0x5000);
        mdio_write(tp, 0x1d, 0x6801);
        mdio_write(tp, 0x1d, 0x677e);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0xdb7c);
        mdio_write(tp, 0x1d, 0x00ee);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x7fe1);
        mdio_write(tp, 0x1d, 0x4e40);
        mdio_write(tp, 0x1d, 0x4837);
        mdio_write(tp, 0x1d, 0x4418);
        mdio_write(tp, 0x1d, 0x41c7);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e40);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x8fc2);
        mdio_write(tp, 0x1d, 0xd2a0);
        mdio_write(tp, 0x1d, 0x004b);
        mdio_write(tp, 0x1d, 0x9204);
        mdio_write(tp, 0x1d, 0xa042);
        mdio_write(tp, 0x1d, 0x3132);
        mdio_write(tp, 0x1d, 0x30f4);
        mdio_write(tp, 0x1d, 0x7fe1);
        mdio_write(tp, 0x1d, 0x4e60);
        mdio_write(tp, 0x1d, 0x489c);
        mdio_write(tp, 0x1d, 0x4628);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e60);
        mdio_write(tp, 0x1d, 0x7e28);
        mdio_write(tp, 0x1d, 0x4628);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5800);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c00);
        mdio_write(tp, 0x1d, 0x41e8);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x8fa8);
        mdio_write(tp, 0x1d, 0xb241);
        mdio_write(tp, 0x1d, 0xa02a);
        mdio_write(tp, 0x1d, 0x314c);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ea0);
        mdio_write(tp, 0x1d, 0x7c02);
        mdio_write(tp, 0x1d, 0x4402);
        mdio_write(tp, 0x1d, 0x4448);
        mdio_write(tp, 0x1d, 0x4894);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c03);
        mdio_write(tp, 0x1d, 0x4824);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c07);
        mdio_write(tp, 0x1d, 0x41ef);
        mdio_write(tp, 0x1d, 0x41ff);
        mdio_write(tp, 0x1d, 0x4891);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c07);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c17);
        mdio_write(tp, 0x1d, 0x8400);
        mdio_write(tp, 0x1d, 0x8ef8);
        mdio_write(tp, 0x1d, 0x41c7);
        mdio_write(tp, 0x1d, 0x8f8d);
        mdio_write(tp, 0x1d, 0x92d5);
        mdio_write(tp, 0x1d, 0xa10f);
        mdio_write(tp, 0x1d, 0xd480);
        mdio_write(tp, 0x1d, 0x0008);
        mdio_write(tp, 0x1d, 0xd580);
        mdio_write(tp, 0x1d, 0x00b8);
        mdio_write(tp, 0x1d, 0xa202);
        mdio_write(tp, 0x1d, 0x3167);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x4404);
        mdio_write(tp, 0x1d, 0x3167);
        mdio_write(tp, 0x1d, 0xd484);
        mdio_write(tp, 0x1d, 0x00f3);
        mdio_write(tp, 0x1d, 0xd484);
        mdio_write(tp, 0x1d, 0x00f1);
        mdio_write(tp, 0x1d, 0x30f4);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ee0);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5400);
        mdio_write(tp, 0x1d, 0x4488);
        mdio_write(tp, 0x1d, 0x41cf);
        mdio_write(tp, 0x1d, 0x30f4);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4ec0);
        mdio_write(tp, 0x1d, 0x48f3);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c09);
        mdio_write(tp, 0x1d, 0x4508);
        mdio_write(tp, 0x1d, 0x41c7);
        mdio_write(tp, 0x1d, 0x8fb0);
        mdio_write(tp, 0x1d, 0xd218);
        mdio_write(tp, 0x1d, 0x00ae);
        mdio_write(tp, 0x1d, 0xd2a4);
        mdio_write(tp, 0x1d, 0x009e);
        mdio_write(tp, 0x1d, 0x3188);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4e80);
        mdio_write(tp, 0x1d, 0x4832);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c01);
        mdio_write(tp, 0x1d, 0x7c1f);
        mdio_write(tp, 0x1d, 0x4c11);
        mdio_write(tp, 0x1d, 0x4428);
        mdio_write(tp, 0x1d, 0x7c40);
        mdio_write(tp, 0x1d, 0x5440);
        mdio_write(tp, 0x1d, 0x7c01);
        mdio_write(tp, 0x1d, 0x5801);
        mdio_write(tp, 0x1d, 0x7c04);
        mdio_write(tp, 0x1d, 0x5c04);
        mdio_write(tp, 0x1d, 0x41e8);
        mdio_write(tp, 0x1d, 0xa4b3);
        mdio_write(tp, 0x1d, 0x319d);
        mdio_write(tp, 0x1d, 0x7fe0);
        mdio_write(tp, 0x1d, 0x4f20);
        mdio_write(tp, 0x1d, 0x6800);
        mdio_write(tp, 0x1d, 0x673e);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x0000);
        mdio_write(tp, 0x1d, 0x570f);
        mdio_write(tp, 0x1d, 0x5fff);
        mdio_write(tp, 0x1d, 0xaa04);
        mdio_write(tp, 0x1d, 0x585b);
        mdio_write(tp, 0x1d, 0x6100);
        mdio_write(tp, 0x1d, 0x31ad);
        mdio_write(tp, 0x1d, 0x5867);
        mdio_write(tp, 0x1d, 0x6080);
        mdio_write(tp, 0x1d, 0xbcf2);
        mdio_write(tp, 0x1d, 0x3001);
        mdio_write(tp, 0x1f, 0x0004);
        mdio_write(tp, 0x1c, 0x0200);
        mdio_write(tp, 0x19, 0x7030);
        mdio_write(tp, 0x1f, 0x0000);
    } else if (tp->mcfg == CFG_METHOD_17) {
        mdio_write(tp, 0x1f, 0x0B82);
        csi_tmp = mdio_read(tp, 0x10);
        csi_tmp |= BIT_4;
        mdio_write(tp, 0x10, csi_tmp);
        mdio_write(tp, 0x1f, 0x0B80);
        i = 0;
        do {
            csi_tmp = mdio_read(tp, 0x10);
            csi_tmp &= 0x0040;
            udelay(50);
            udelay(50);
            i++;
        } while(csi_tmp != 0x0040 && i <1000);
        mdio_write(tp, 0x1f, 0x0A43);
        mdio_write(tp, 0x13, 0x8146);
        mdio_write(tp, 0x14, 0x0300);
        mdio_write(tp, 0x13, 0xB82E);
        mdio_write(tp, 0x14, 0x0001);
        mdio_write(tp, 0x1F, 0x0A43);
        mdio_write(tp, 0x13, 0xb820);
        mdio_write(tp, 0x14, 0x0290);
        mdio_write(tp, 0x13, 0xa012);
        mdio_write(tp, 0x14, 0x0000);
        mdio_write(tp, 0x13, 0xa014);
        mdio_write(tp, 0x14, 0x2c04);
        mdio_write(tp, 0x14, 0x2c07);
        mdio_write(tp, 0x14, 0x2c07);
        mdio_write(tp, 0x14, 0x2c07);
        mdio_write(tp, 0x14, 0xa304);
        mdio_write(tp, 0x14, 0xa301);
        mdio_write(tp, 0x14, 0x207e);
        mdio_write(tp, 0x13, 0xa01a);
        mdio_write(tp, 0x14, 0x0000);
        mdio_write(tp, 0x13, 0xa006);
        mdio_write(tp, 0x14, 0x0fff);
        mdio_write(tp, 0x13, 0xa004);
        mdio_write(tp, 0x14, 0x0fff);
        mdio_write(tp, 0x13, 0xa002);
        mdio_write(tp, 0x14, 0x0fff);
        mdio_write(tp, 0x13, 0xa000);
        mdio_write(tp, 0x14, 0x107c);
        mdio_write(tp, 0x13, 0xb820);
        mdio_write(tp, 0x14, 0x0210);
        mdio_write(tp, 0x1F, 0x0A43);
        mdio_write(tp, 0x13, 0x0000);
        mdio_write(tp, 0x14, 0x0000);
        mdio_write(tp, 0x1f, 0x0B82);
        csi_tmp = mdio_read(tp, 0x17);
        csi_tmp &= ~(BIT_0);
        mdio_write(tp, 0x17, csi_tmp);
        mdio_write(tp, 0x1f, 0x0A43);
        mdio_write(tp, 0x13, 0x8146);
        mdio_write(tp, 0x14, 0x0000);
        mdio_write(tp, 0x1f, 0x0B82);
        csi_tmp = mdio_read(tp, 0x10);
        csi_tmp &= ~(BIT_4);
        mdio_write(tp, 0x10, csi_tmp);
    }
    
    rtl8101_write_hw_phy_mcu_code_ver(dev);
    
    mdio_write(tp, 0x1F, 0x0000);
}

void rtl8101_hw_phy_config(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    //unsigned long flags;
    __u32	gphy_val;
    
    tp->phy_reset_enable(dev);
    
    spin_lock_irqsave(&tp->phy_lock, flags);
    
    rtl8101_init_hw_phy_mcu(dev);
    if (tp->mcfg == CFG_METHOD_4) {
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x11, mdio_read(tp, 0x11) | 0x1000);
        mdio_write(tp, 0x19, mdio_read(tp, 0x19) | 0x2000);
        mdio_write(tp, 0x10, mdio_read(tp, 0x10) | 0x8000);
        
        mdio_write(tp, 0x1f, 0x0003);
        mdio_write(tp, 0x08, 0x441D);
        mdio_write(tp, 0x01, 0x9100);
        mdio_write(tp, 0x1f, 0x0000);
    } else if (tp->mcfg == CFG_METHOD_5) {
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x11, mdio_read(tp, 0x11) | 0x1000);
        mdio_write(tp, 0x19, mdio_read(tp, 0x19) | 0x2000);
        mdio_write(tp, 0x10, mdio_read(tp, 0x10) | 0x8000);
        
        mdio_write(tp, 0x1f, 0x0003);
        mdio_write(tp, 0x08, 0x441D);
        mdio_write(tp, 0x01, 0x9100);
        mdio_write(tp, 0x1f, 0x0000);
    } else if (tp->mcfg == CFG_METHOD_6) {
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x11, mdio_read(tp, 0x11) | 0x1000);
        mdio_write(tp, 0x19, mdio_read(tp, 0x19) | 0x2000);
        mdio_write(tp, 0x10, mdio_read(tp, 0x10) | 0x8000);
        
        mdio_write(tp, 0x1f, 0x0003);
        mdio_write(tp, 0x08, 0x441D);
        mdio_write(tp, 0x1f, 0x0000);
    } else if (tp->mcfg == CFG_METHOD_7 || tp->mcfg == CFG_METHOD_8) {
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x11, mdio_read(tp, 0x11) | 0x1000);
        mdio_write(tp, 0x19, mdio_read(tp, 0x19) | 0x2000);
        mdio_write(tp, 0x10, mdio_read(tp, 0x10) | 0x8000);
    } else if (tp->mcfg == CFG_METHOD_9) {
        mdio_write(tp, 0x1F, 0x0000);
        mdio_write(tp, 0x11, mdio_read(tp, 0x11) | BIT_12);
        mdio_write(tp, 0x1F, 0x0002);
        mdio_write(tp, 0x0F, mdio_read(tp, 0x0F) | BIT_0 | BIT_1);
        mdio_write(tp, 0x1F, 0x0000);
        
        mdio_write(tp, 0x1F, 0x0000);
        rtl8101_phyio_write(ioaddr, 0x0E, 0x0068);
        rtl8101_phyio_write(ioaddr, 0x0E, 0x0069);
        rtl8101_phyio_write(ioaddr, 0x0E, 0x006A);
        rtl8101_phyio_write(ioaddr, 0x0E, 0x006B);
        rtl8101_phyio_write(ioaddr, 0x0E, 0x006C);
    } else if (tp->mcfg == CFG_METHOD_10) {
        mdio_write(tp, 0x1F, 0x0007);
        mdio_write(tp, 0x1E, 0x0023);
        mdio_write(tp, 0x17, 0x0116);
        mdio_write(tp, 0x1F, 0x0000);
        
        mdio_write(tp, 0x1f, 0x0005);
        mdio_write(tp, 0x05, 0x8b80);
        mdio_write(tp, 0x06, 0xc896);
        mdio_write(tp, 0x1f, 0x0000);
        
        mdio_write(tp, 0x1F, 0x0001);
        mdio_write(tp, 0x0B, 0x8C60);
        mdio_write(tp, 0x07, 0x2872);
        mdio_write(tp, 0x1C, 0xEFFF);
        mdio_write(tp, 0x1F, 0x0003);
        mdio_write(tp, 0x14, 0x94B0);
        mdio_write(tp, 0x1F, 0x0000);
        
        mdio_write(tp, 0x1F, 0x0002);
        gphy_val = mdio_read(tp, 0x08) & 0x00FF;
        mdio_write(tp, 0x08, gphy_val | 0x8000);
        
        mdio_write(tp, 0x1F, 0x0007);
        mdio_write(tp, 0x1E, 0x002D);
        gphy_val = mdio_read(tp, 0x18);
        mdio_write(tp, 0x18, gphy_val | 0x0010);
        mdio_write(tp, 0x1F, 0x0000);
        gphy_val = mdio_read(tp, 0x14);
        mdio_write(tp, 0x14, gphy_val | 0x8000);
        
        mdio_write(tp, 0x1F, 0x0002);
        mdio_write(tp, 0x00, 0x080B);
        mdio_write(tp, 0x0B, 0x09D7);
        mdio_write(tp, 0x1f, 0x0000);
        mdio_write(tp, 0x15, 0x1006);
        
        mdio_write(tp, 0x1F, 0x0003);
        mdio_write(tp, 0x19, 0x7F46);
        mdio_write(tp, 0x1F, 0x0005);
        mdio_write(tp, 0x05, 0x8AD2);
        mdio_write(tp, 0x06, 0x6810);
        mdio_write(tp, 0x05, 0x8AD4);
        mdio_write(tp, 0x06, 0x8002);
        mdio_write(tp, 0x05, 0x8ADE);
        mdio_write(tp, 0x06, 0x8025);
        mdio_write(tp, 0x1F, 0x0000);
    } else if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
               tp->mcfg == CFG_METHOD_13) {
        if (RTL_R8(0xEF) & 0x08) {
            mdio_write(tp, 0x1F, 0x0005);
            mdio_write(tp, 0x1A, 0x0004);
            mdio_write(tp, 0x1F, 0x0000);
        } else {
            mdio_write(tp, 0x1F, 0x0005);
            mdio_write(tp, 0x1A, 0x0000);
            mdio_write(tp, 0x1F, 0x0000);
        }
        
        if (RTL_R8(0xEF) & 0x010) {
            mdio_write(tp, 0x1F, 0x0004);
            mdio_write(tp, 0x1C, 0x0000);
            mdio_write(tp, 0x1F, 0x0000);
        } else {
            mdio_write(tp, 0x1F, 0x0004);
            mdio_write(tp, 0x1C, 0x0200);
            mdio_write(tp, 0x1F, 0x0000);
        }
        
        mdio_write(tp, 0x1F, 0x0001);
        mdio_write(tp, 0x15, 0x7701);
        mdio_write(tp, 0x1F, 0x0000);
        
        mdio_write(tp, 0x1F, 0x0000);
        gphy_val = mdio_read(tp, 0x1A);
        mdio_write(tp, 0x08, gphy_val & ~BIT_14);
        
        if(tp->aspm) {
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x18, 0x8310);
            mdio_write(tp, 0x1F, 0x0000);
        }
    } else if (tp->mcfg == CFG_METHOD_14) {
        if(tp->aspm) {
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x18, 0x8310);
            mdio_write(tp, 0x1F, 0x0000);
        }
    } else if (tp->mcfg == CFG_METHOD_15 || tp->mcfg == CFG_METHOD_16) {
        mdio_write(tp, 0x1F, 0x0001);
        mdio_write(tp, 0x11, 0x83BA);
        mdio_write(tp, 0x1F, 0x0000);
        
        if(tp->aspm) {
            mdio_write(tp, 0x1F, 0x0000);
            mdio_write(tp, 0x18, 0x8310);
            mdio_write(tp, 0x1F, 0x0000);
        }
    } else if (tp->mcfg == CFG_METHOD_17) {
        mdio_write(tp, 0x1F, 0x0BCC);
        mdio_write(tp, 0x14, mdio_read(tp, 0x14) & ~BIT_8);
        mdio_write(tp, 0x1F, 0x0A44);
        mdio_write(tp, 0x11, mdio_read(tp, 0x11) | BIT_7);
        mdio_write(tp, 0x11, mdio_read(tp, 0x11) | BIT_6);
        mdio_write(tp, 0x1F, 0x0A43);
        mdio_write(tp, 0x13, 0x8084);
        mdio_write(tp, 0x14, mdio_read(tp, 0x14) & ~(BIT_14 | BIT_13));
        mdio_write(tp, 0x10, mdio_read(tp, 0x10) | BIT_12);
        mdio_write(tp, 0x10, mdio_read(tp, 0x10) | BIT_1);
        mdio_write(tp, 0x10, mdio_read(tp, 0x10) | BIT_0);
        
        mdio_write(tp, 0x1F, 0x0A43);
        mdio_write(tp, 0x13, 0x8012);
        mdio_write(tp, 0x14, mdio_read(tp, 0x14) | BIT_15);
        
        mdio_write(tp, 0x1F, 0x0BCE);
        mdio_write(tp, 0x12, 0x8860);
        
        mdio_write(tp, 0x1F, 0x0A43);
        mdio_write(tp, 0x13, 0x80F3);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x8B00);
        mdio_write(tp, 0x13, 0x80F0);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x3A00);
        mdio_write(tp, 0x13, 0x80EF);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x0500);
        mdio_write(tp, 0x13, 0x80F6);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x6E00);
        mdio_write(tp, 0x13, 0x80EC);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x6800);
        mdio_write(tp, 0x13, 0x80ED);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x7C00);
        mdio_write(tp, 0x13, 0x80F2);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0xF400);
        mdio_write(tp, 0x13, 0x80F4);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x8500);
        mdio_write(tp, 0x1F, 0x0A43);
        mdio_write(tp, 0x13, 0x8110);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0xA800);
        mdio_write(tp, 0x13, 0x810F);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x1D00);
        mdio_write(tp, 0x13, 0x8111);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0xF500);
        mdio_write(tp, 0x13, 0x8113);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x6100);
        mdio_write(tp, 0x13, 0x8115);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x9200);
        mdio_write(tp, 0x13, 0x810E);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x0400);
        mdio_write(tp, 0x13, 0x810C);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x7C00);
        mdio_write(tp, 0x13, 0x810B);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x5A00);
        mdio_write(tp, 0x1F, 0x0A43);
        mdio_write(tp, 0x13, 0x80D1);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0xFF00);
        mdio_write(tp, 0x13, 0x80CD);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x9E00);
        mdio_write(tp, 0x13, 0x80D3);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x0E00);
        mdio_write(tp, 0x13, 0x80D5);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0xCA00);
        mdio_write(tp, 0x13, 0x80D7);
        mdio_write(tp, 0x14, (mdio_read(tp, 0x14) & ~0xFF00) | 0x8400);
        
        if (tp->aspm) {
            mdio_write(tp, 0x1F, 0x0A43);
            mdio_write(tp, 0x10, mdio_read(tp, 0x10) | BIT_2);
        }
    }
    
    /*ocp phy power saving*/
    if (tp->mcfg == CFG_METHOD_17) {
        if (tp->aspm) {
            mdio_write(tp, 0x1F, 0x0C41);
            mdio_write(tp, 0x13, 0x0000);
            mdio_write(tp, 0x13, 0x0050);
            mdio_write(tp, 0x1F, 0x0000);
        }
    }
    
    mdio_write(tp, 0x1F, 0x0000);
    
    spin_unlock_irqrestore(&tp->phy_lock, flags);
    
    if (eee_enable == 1)
        rtl8101_enable_EEE(tp);
    else
        rtl8101_disable_EEE(tp);
}

#if DISABLED_CODE

static inline void rtl8101_delete_link_timer(struct net_device *dev, struct timer_list *timer)
{
    del_timer_sync(timer);
}

static inline void rtl8101_request_link_timer(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct timer_list *timer = &tp->link_timer;
    
    init_timer(timer);
    timer->expires = jiffies + RTL8101_LINK_TIMEOUT;
    timer->data = (unsigned long)(dev);
    timer->function = rtl8101_link_timer;
    add_timer(timer);
}

static inline void rtl8101_delete_esd_timer(struct net_device *dev, struct timer_list *timer)
{
    del_timer_sync(timer);
}

static inline void rtl8101_request_esd_timer(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct timer_list *timer = &tp->esd_timer;
    
    init_timer(timer);
    timer->expires = jiffies + RTL8101_ESD_TIMEOUT;
    timer->data = (unsigned long)(dev);
    timer->function = rtl8101_esd_timer;
    add_timer(timer);
}

#ifdef CONFIG_NET_POLL_CONTROLLER
/*
 * Polling 'interrupt' - used by things like netconsole to send skbs
 * without having to re-enable interrupts. It's not called while
 * the interrupt routine is executing.
 */
static void
rtl8101_netpoll(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    
    disable_irq(pdev->irq);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
    rtl8101_interrupt(pdev->irq, dev, NULL);
#else
    rtl8101_interrupt(pdev->irq, dev);
#endif
    enable_irq(pdev->irq);
}
#endif

#endif  /* DISABLED_CODE */

void rtl8101_get_bios_setting(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    
    tp->bios_setting = 0;
    switch (tp->mcfg) {
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
            tp->bios_setting = RTL_R32(0x8c);
            break;
    }
}

void rtl8101_set_bios_setting(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    
    switch (tp->mcfg) {
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
            RTL_W32(0x8C, tp->bios_setting);
            break;
    }
}

#if DISABLED_CODE

static void
rtl8101_release_board(struct pci_dev *pdev,
                      struct net_device *dev,
                      void __iomem *ioaddr)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    
    rtl8101_set_bios_setting(dev);
    rtl8101_rar_set(tp, tp->org_mac_addr);
    tp->wol_enabled = WOL_DISABLED;
    
    rtl8101_phy_power_down(dev);
    
    iounmap(ioaddr);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    free_netdev(dev);
}

static int
rtl8101_get_mac_address(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u16 mac_addr[3];
    int i;
    
    
    if (tp->mcfg == CFG_METHOD_14 ||
        tp->mcfg == CFG_METHOD_17) {
        *(u32*)&mac_addr[0] = rtl8101_eri_read(ioaddr, 0xE0, 4, ERIAR_ExGMAC);
        *(u16*)&mac_addr[2] = rtl8101_eri_read(ioaddr, 0xE4, 2, ERIAR_ExGMAC);
    } else {
        if (tp->eeprom_type != EEPROM_TYPE_NONE) {
            /* Get MAC address from EEPROM */
            mac_addr[0] = rtl_eeprom_read_sc(tp, 7);
            mac_addr[1] = rtl_eeprom_read_sc(tp, 8);
            mac_addr[2] = rtl_eeprom_read_sc(tp, 9);
            RTL_W8(Cfg9346, Cfg9346_Unlock);
            RTL_W32(MAC0, (mac_addr[1] << 16) | mac_addr[0]);
            RTL_W16(MAC4, mac_addr[2]);
            RTL_W8(Cfg9346, Cfg9346_Lock);
        }
    }
    
    for (i = 0; i < MAC_ADDR_LEN; i++) {
        dev->dev_addr[i] = RTL_R8(MAC0 + i);
        tp->org_mac_addr[i] = dev->dev_addr[i]; /* keep the original MAC address */
    }
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
    memcpy(dev->perm_addr, dev->dev_addr, dev->addr_len);
#endif
    //  memcpy(dev->dev_addr, dev->dev_addr, dev->addr_len);
    
    return 0;
}

/**
 * rtl8101_set_mac_address - Change the Ethernet Address of the NIC
 * @dev: network interface device structure
 * @p:   pointer to an address structure
 *
 * Return 0 on success, negative on failure
 **/
static int
rtl8101_set_mac_address(struct net_device *dev,
                        void *p)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct sockaddr *addr = p;
    
    if (!is_valid_ether_addr(addr->sa_data))
        return -EADDRNOTAVAIL;
    
    memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);
    
    rtl8101_rar_set(tp, dev->dev_addr);
    
    return 0;
}

#endif  /* DISABLED_CODE */

/******************************************************************************
 * rtl8101_rar_set - Puts an ethernet address into a receive address register.
 *
 * tp - The private data structure for driver
 * addr - Address to put into receive address register
 *****************************************************************************/
void
rtl8101_rar_set(struct rtl8101_private *tp,
                uint8_t *addr)
{
    void __iomem *ioaddr = tp->mmio_addr;
    uint32_t rar_low = 0;
    uint32_t rar_high = 0;
    
    rar_low = ((uint32_t) addr[0] |
               ((uint32_t) addr[1] << 8) |
               ((uint32_t) addr[2] << 16) |
               ((uint32_t) addr[3] << 24));
    
    rar_high = ((uint32_t) addr[4] |
                ((uint32_t) addr[5] << 8));
    
    RTL_W8(Cfg9346, Cfg9346_Unlock);
    RTL_W32(MAC0, rar_low);
    RTL_W32(MAC4, rar_high);
    RTL_W8(Cfg9346, Cfg9346_Lock);
}

#if DISABLED_CODE

#ifdef ETHTOOL_OPS_COMPAT
static int ethtool_get_settings(struct net_device *dev, void *useraddr)
{
    struct ethtool_cmd cmd = { ETHTOOL_GSET };
    int err;
    
    if (!ethtool_ops->get_settings)
        return -EOPNOTSUPP;
    
    err = ethtool_ops->get_settings(dev, &cmd);
    if (err < 0)
        return err;
    
    if (copy_to_user(useraddr, &cmd, sizeof(cmd)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_settings(struct net_device *dev, void *useraddr)
{
    struct ethtool_cmd cmd;
    
    if (!ethtool_ops->set_settings)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&cmd, useraddr, sizeof(cmd)))
        return -EFAULT;
    
    return ethtool_ops->set_settings(dev, &cmd);
}

static int ethtool_get_drvinfo(struct net_device *dev, void *useraddr)
{
    struct ethtool_drvinfo info;
    struct ethtool_ops *ops = ethtool_ops;
    
    if (!ops->get_drvinfo)
        return -EOPNOTSUPP;
    
    memset(&info, 0, sizeof(info));
    info.cmd = ETHTOOL_GDRVINFO;
    ops->get_drvinfo(dev, &info);
    
    if (ops->self_test_count)
        info.testinfo_len = ops->self_test_count(dev);
    if (ops->get_stats_count)
        info.n_stats = ops->get_stats_count(dev);
    if (ops->get_regs_len)
        info.regdump_len = ops->get_regs_len(dev);
    if (ops->get_eeprom_len)
        info.eedump_len = ops->get_eeprom_len(dev);
    
    if (copy_to_user(useraddr, &info, sizeof(info)))
        return -EFAULT;
    return 0;
}

static int ethtool_get_regs(struct net_device *dev, char *useraddr)
{
    struct ethtool_regs regs;
    struct ethtool_ops *ops = ethtool_ops;
    void *regbuf;
    int reglen, ret;
    
    if (!ops->get_regs || !ops->get_regs_len)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&regs, useraddr, sizeof(regs)))
        return -EFAULT;
    
    reglen = ops->get_regs_len(dev);
    if (regs.len > reglen)
        regs.len = reglen;
    
    regbuf = kmalloc(reglen, GFP_USER);
    if (!regbuf)
        return -ENOMEM;
    
    ops->get_regs(dev, &regs, regbuf);
    
    ret = -EFAULT;
    if (copy_to_user(useraddr, &regs, sizeof(regs)))
        goto out;
    useraddr += offsetof(struct ethtool_regs, data);
    if (copy_to_user(useraddr, regbuf, reglen))
        goto out;
    ret = 0;
    
out:
    kfree(regbuf);
    return ret;
}

static int ethtool_get_wol(struct net_device *dev, char *useraddr)
{
    struct ethtool_wolinfo wol = { ETHTOOL_GWOL };
    
    if (!ethtool_ops->get_wol)
        return -EOPNOTSUPP;
    
    ethtool_ops->get_wol(dev, &wol);
    
    if (copy_to_user(useraddr, &wol, sizeof(wol)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_wol(struct net_device *dev, char *useraddr)
{
    struct ethtool_wolinfo wol;
    
    if (!ethtool_ops->set_wol)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&wol, useraddr, sizeof(wol)))
        return -EFAULT;
    
    return ethtool_ops->set_wol(dev, &wol);
}

static int ethtool_get_msglevel(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GMSGLVL };
    
    if (!ethtool_ops->get_msglevel)
        return -EOPNOTSUPP;
    
    edata.data = ethtool_ops->get_msglevel(dev);
    
    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_msglevel(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata;
    
    if (!ethtool_ops->set_msglevel)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&edata, useraddr, sizeof(edata)))
        return -EFAULT;
    
    ethtool_ops->set_msglevel(dev, edata.data);
    return 0;
}

static int ethtool_nway_reset(struct net_device *dev)
{
    if (!ethtool_ops->nway_reset)
        return -EOPNOTSUPP;
    
    return ethtool_ops->nway_reset(dev);
}

static int ethtool_get_link(struct net_device *dev, void *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GLINK };
    
    if (!ethtool_ops->get_link)
        return -EOPNOTSUPP;
    
    edata.data = ethtool_ops->get_link(dev);
    
    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_get_eeprom(struct net_device *dev, void *useraddr)
{
    struct ethtool_eeprom eeprom;
    struct ethtool_ops *ops = ethtool_ops;
    u8 *data;
    int ret;
    
    if (!ops->get_eeprom || !ops->get_eeprom_len)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&eeprom, useraddr, sizeof(eeprom)))
        return -EFAULT;
    
    /* Check for wrap and zero */
    if (eeprom.offset + eeprom.len <= eeprom.offset)
        return -EINVAL;
    
    /* Check for exceeding total eeprom len */
    if (eeprom.offset + eeprom.len > ops->get_eeprom_len(dev))
        return -EINVAL;
    
    data = kmalloc(eeprom.len, GFP_USER);
    if (!data)
        return -ENOMEM;
    
    ret = -EFAULT;
    if (copy_from_user(data, useraddr + sizeof(eeprom), eeprom.len))
        goto out;
    
    ret = ops->get_eeprom(dev, &eeprom, data);
    if (ret)
        goto out;
    
    ret = -EFAULT;
    if (copy_to_user(useraddr, &eeprom, sizeof(eeprom)))
        goto out;
    if (copy_to_user(useraddr + sizeof(eeprom), data, eeprom.len))
        goto out;
    ret = 0;
    
out:
    kfree(data);
    return ret;
}

static int ethtool_set_eeprom(struct net_device *dev, void *useraddr)
{
    struct ethtool_eeprom eeprom;
    struct ethtool_ops *ops = ethtool_ops;
    u8 *data;
    int ret;
    
    if (!ops->set_eeprom || !ops->get_eeprom_len)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&eeprom, useraddr, sizeof(eeprom)))
        return -EFAULT;
    
    /* Check for wrap and zero */
    if (eeprom.offset + eeprom.len <= eeprom.offset)
        return -EINVAL;
    
    /* Check for exceeding total eeprom len */
    if (eeprom.offset + eeprom.len > ops->get_eeprom_len(dev))
        return -EINVAL;
    
    data = kmalloc(eeprom.len, GFP_USER);
    if (!data)
        return -ENOMEM;
    
    ret = -EFAULT;
    if (copy_from_user(data, useraddr + sizeof(eeprom), eeprom.len))
        goto out;
    
    ret = ops->set_eeprom(dev, &eeprom, data);
    if (ret)
        goto out;
    
    if (copy_to_user(useraddr + sizeof(eeprom), data, eeprom.len))
        ret = -EFAULT;
    
out:
    kfree(data);
    return ret;
}

static int ethtool_get_coalesce(struct net_device *dev, void *useraddr)
{
    struct ethtool_coalesce coalesce = { ETHTOOL_GCOALESCE };
    
    if (!ethtool_ops->get_coalesce)
        return -EOPNOTSUPP;
    
    ethtool_ops->get_coalesce(dev, &coalesce);
    
    if (copy_to_user(useraddr, &coalesce, sizeof(coalesce)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_coalesce(struct net_device *dev, void *useraddr)
{
    struct ethtool_coalesce coalesce;
    
    if (!ethtool_ops->get_coalesce)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&coalesce, useraddr, sizeof(coalesce)))
        return -EFAULT;
    
    return ethtool_ops->set_coalesce(dev, &coalesce);
}

static int ethtool_get_ringparam(struct net_device *dev, void *useraddr)
{
    struct ethtool_ringparam ringparam = { ETHTOOL_GRINGPARAM };
    
    if (!ethtool_ops->get_ringparam)
        return -EOPNOTSUPP;
    
    ethtool_ops->get_ringparam(dev, &ringparam);
    
    if (copy_to_user(useraddr, &ringparam, sizeof(ringparam)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_ringparam(struct net_device *dev, void *useraddr)
{
    struct ethtool_ringparam ringparam;
    
    if (!ethtool_ops->get_ringparam)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&ringparam, useraddr, sizeof(ringparam)))
        return -EFAULT;
    
    return ethtool_ops->set_ringparam(dev, &ringparam);
}

static int ethtool_get_pauseparam(struct net_device *dev, void *useraddr)
{
    struct ethtool_pauseparam pauseparam = { ETHTOOL_GPAUSEPARAM };
    
    if (!ethtool_ops->get_pauseparam)
        return -EOPNOTSUPP;
    
    ethtool_ops->get_pauseparam(dev, &pauseparam);
    
    if (copy_to_user(useraddr, &pauseparam, sizeof(pauseparam)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_pauseparam(struct net_device *dev, void *useraddr)
{
    struct ethtool_pauseparam pauseparam;
    
    if (!ethtool_ops->get_pauseparam)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&pauseparam, useraddr, sizeof(pauseparam)))
        return -EFAULT;
    
    return ethtool_ops->set_pauseparam(dev, &pauseparam);
}

static int ethtool_get_rx_csum(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GRXCSUM };
    
    if (!ethtool_ops->get_rx_csum)
        return -EOPNOTSUPP;
    
    edata.data = ethtool_ops->get_rx_csum(dev);
    
    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_rx_csum(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata;
    
    if (!ethtool_ops->set_rx_csum)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&edata, useraddr, sizeof(edata)))
        return -EFAULT;
    
    ethtool_ops->set_rx_csum(dev, edata.data);
    return 0;
}

static int ethtool_get_tx_csum(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GTXCSUM };
    
    if (!ethtool_ops->get_tx_csum)
        return -EOPNOTSUPP;
    
    edata.data = ethtool_ops->get_tx_csum(dev);
    
    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_tx_csum(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata;
    
    if (!ethtool_ops->set_tx_csum)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&edata, useraddr, sizeof(edata)))
        return -EFAULT;
    
    return ethtool_ops->set_tx_csum(dev, edata.data);
}

static int ethtool_get_sg(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GSG };
    
    if (!ethtool_ops->get_sg)
        return -EOPNOTSUPP;
    
    edata.data = ethtool_ops->get_sg(dev);
    
    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_sg(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata;
    
    if (!ethtool_ops->set_sg)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&edata, useraddr, sizeof(edata)))
        return -EFAULT;
    
    return ethtool_ops->set_sg(dev, edata.data);
}

static int ethtool_get_tso(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata = { ETHTOOL_GTSO };
    
    if (!ethtool_ops->get_tso)
        return -EOPNOTSUPP;
    
    edata.data = ethtool_ops->get_tso(dev);
    
    if (copy_to_user(useraddr, &edata, sizeof(edata)))
        return -EFAULT;
    return 0;
}

static int ethtool_set_tso(struct net_device *dev, char *useraddr)
{
    struct ethtool_value edata;
    
    if (!ethtool_ops->set_tso)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&edata, useraddr, sizeof(edata)))
        return -EFAULT;
    
    return ethtool_ops->set_tso(dev, edata.data);
}

static int ethtool_self_test(struct net_device *dev, char *useraddr)
{
    struct ethtool_test test;
    struct ethtool_ops *ops = ethtool_ops;
    u64 *data;
    int ret;
    
    if (!ops->self_test || !ops->self_test_count)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&test, useraddr, sizeof(test)))
        return -EFAULT;
    
    test.len = ops->self_test_count(dev);
    data = kmalloc(test.len * sizeof(u64), GFP_USER);
    if (!data)
        return -ENOMEM;
    
    ops->self_test(dev, &test, data);
    
    ret = -EFAULT;
    if (copy_to_user(useraddr, &test, sizeof(test)))
        goto out;
    useraddr += sizeof(test);
    if (copy_to_user(useraddr, data, test.len * sizeof(u64)))
        goto out;
    ret = 0;
    
out:
    kfree(data);
    return ret;
}

static int ethtool_get_strings(struct net_device *dev, void *useraddr)
{
    struct ethtool_gstrings gstrings;
    struct ethtool_ops *ops = ethtool_ops;
    u8 *data;
    int ret;
    
    if (!ops->get_strings)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&gstrings, useraddr, sizeof(gstrings)))
        return -EFAULT;
    
    switch (gstrings.string_set) {
        case ETH_SS_TEST:
            if (!ops->self_test_count)
                return -EOPNOTSUPP;
            gstrings.len = ops->self_test_count(dev);
            break;
        case ETH_SS_STATS:
            if (!ops->get_stats_count)
                return -EOPNOTSUPP;
            gstrings.len = ops->get_stats_count(dev);
            break;
        default:
            return -EINVAL;
    }
    
    data = kmalloc(gstrings.len * ETH_GSTRING_LEN, GFP_USER);
    if (!data)
        return -ENOMEM;
    
    ops->get_strings(dev, gstrings.string_set, data);
    
    ret = -EFAULT;
    if (copy_to_user(useraddr, &gstrings, sizeof(gstrings)))
        goto out;
    useraddr += sizeof(gstrings);
    if (copy_to_user(useraddr, data, gstrings.len * ETH_GSTRING_LEN))
        goto out;
    ret = 0;
    
out:
    kfree(data);
    return ret;
}

static int ethtool_phys_id(struct net_device *dev, void *useraddr)
{
    struct ethtool_value id;
    
    if (!ethtool_ops->phys_id)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&id, useraddr, sizeof(id)))
        return -EFAULT;
    
    return ethtool_ops->phys_id(dev, id.data);
}

static int ethtool_get_stats(struct net_device *dev, void *useraddr)
{
    struct ethtool_stats stats;
    struct ethtool_ops *ops = ethtool_ops;
    u64 *data;
    int ret;
    
    if (!ops->get_ethtool_stats || !ops->get_stats_count)
        return -EOPNOTSUPP;
    
    if (copy_from_user(&stats, useraddr, sizeof(stats)))
        return -EFAULT;
    
    stats.n_stats = ops->get_stats_count(dev);
    data = kmalloc(stats.n_stats * sizeof(u64), GFP_USER);
    if (!data)
        return -ENOMEM;
    
    ops->get_ethtool_stats(dev, &stats, data);
    
    ret = -EFAULT;
    if (copy_to_user(useraddr, &stats, sizeof(stats)))
        goto out;
    useraddr += sizeof(stats);
    if (copy_to_user(useraddr, data, stats.n_stats * sizeof(u64)))
        goto out;
    ret = 0;
    
out:
    kfree(data);
    return ret;
}

static int ethtool_ioctl(struct ifreq *ifr)
{
    struct net_device *dev = __dev_get_by_name(ifr->ifr_name);
    void *useraddr = (void *) ifr->ifr_data;
    u32 ethcmd;
    
    /*
     * XXX: This can be pushed down into the ethtool_* handlers that
     * need it.  Keep existing behaviour for the moment.
     */
    if (!capable(CAP_NET_ADMIN))
        return -EPERM;
    
    if (!dev || !netif_device_present(dev))
        return -ENODEV;
    
    if (copy_from_user(&ethcmd, useraddr, sizeof (ethcmd)))
        return -EFAULT;
    
    switch (ethcmd) {
        case ETHTOOL_GSET:
            return ethtool_get_settings(dev, useraddr);
        case ETHTOOL_SSET:
            return ethtool_set_settings(dev, useraddr);
        case ETHTOOL_GDRVINFO:
            return ethtool_get_drvinfo(dev, useraddr);
        case ETHTOOL_GREGS:
            return ethtool_get_regs(dev, useraddr);
        case ETHTOOL_GWOL:
            return ethtool_get_wol(dev, useraddr);
        case ETHTOOL_SWOL:
            return ethtool_set_wol(dev, useraddr);
        case ETHTOOL_GMSGLVL:
            return ethtool_get_msglevel(dev, useraddr);
        case ETHTOOL_SMSGLVL:
            return ethtool_set_msglevel(dev, useraddr);
        case ETHTOOL_NWAY_RST:
            return ethtool_nway_reset(dev);
        case ETHTOOL_GLINK:
            return ethtool_get_link(dev, useraddr);
        case ETHTOOL_GEEPROM:
            return ethtool_get_eeprom(dev, useraddr);
        case ETHTOOL_SEEPROM:
            return ethtool_set_eeprom(dev, useraddr);
        case ETHTOOL_GCOALESCE:
            return ethtool_get_coalesce(dev, useraddr);
        case ETHTOOL_SCOALESCE:
            return ethtool_set_coalesce(dev, useraddr);
        case ETHTOOL_GRINGPARAM:
            return ethtool_get_ringparam(dev, useraddr);
        case ETHTOOL_SRINGPARAM:
            return ethtool_set_ringparam(dev, useraddr);
        case ETHTOOL_GPAUSEPARAM:
            return ethtool_get_pauseparam(dev, useraddr);
        case ETHTOOL_SPAUSEPARAM:
            return ethtool_set_pauseparam(dev, useraddr);
        case ETHTOOL_GRXCSUM:
            return ethtool_get_rx_csum(dev, useraddr);
        case ETHTOOL_SRXCSUM:
            return ethtool_set_rx_csum(dev, useraddr);
        case ETHTOOL_GTXCSUM:
            return ethtool_get_tx_csum(dev, useraddr);
        case ETHTOOL_STXCSUM:
            return ethtool_set_tx_csum(dev, useraddr);
        case ETHTOOL_GSG:
            return ethtool_get_sg(dev, useraddr);
        case ETHTOOL_SSG:
            return ethtool_set_sg(dev, useraddr);
        case ETHTOOL_GTSO:
            return ethtool_get_tso(dev, useraddr);
        case ETHTOOL_STSO:
            return ethtool_set_tso(dev, useraddr);
        case ETHTOOL_TEST:
            return ethtool_self_test(dev, useraddr);
        case ETHTOOL_GSTRINGS:
            return ethtool_get_strings(dev, useraddr);
        case ETHTOOL_PHYS_ID:
            return ethtool_phys_id(dev, useraddr);
        case ETHTOOL_GSTATS:
            return ethtool_get_stats(dev, useraddr);
        default:
            return -EOPNOTSUPP;
    }
    
    return -EOPNOTSUPP;
}
#endif //ETHTOOL_OPS_COMPAT


static int
rtl8101_do_ioctl(struct net_device *dev,
                 struct ifreq *ifr,
                 int cmd)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct mii_ioctl_data *data = if_mii(ifr);
    unsigned long flags;
    
    if (!netif_running(dev))
        return -ENODEV;
    
    switch (cmd) {
        case SIOCGMIIPHY:
            data->phy_id = 32; /* Internal PHY */
            return 0;
            
        case SIOCGMIIREG:
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1f, 0x0000);
            data->val_out = mdio_read(tp, data->reg_num & 0x1f);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            return 0;
            
        case SIOCSMIIREG:
            if (!capable(CAP_NET_ADMIN))
                return -EPERM;
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1f, 0x0000);
            mdio_write(tp, data->reg_num & 0x1f, data->val_in);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
            return 0;
#ifdef ETHTOOL_OPS_COMPAT
        case SIOCETHTOOL:
            return ethtool_ioctl(ifr);
#endif
            
        case SIOCRTLTOOL:
            return rtltool_ioctl(tp, ifr);
            
        default:
            return -EOPNOTSUPP;
    }
    
    return -EOPNOTSUPP;
}

#endif  /* DISABLED_CODE */

void rtl8101_phy_power_down (struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    //unsigned long flags;
    
    spin_lock_irqsave(&tp->phy_lock, flags);
    mdio_write(tp, 0x1f, 0x0000);
    if (tp->mcfg == CFG_METHOD_10)
        mdio_write(tp, MII_BMCR, BMCR_ANENABLE|BMCR_PDOWN);
    else
        mdio_write(tp, MII_BMCR, BMCR_PDOWN);
    
    spin_unlock_irqrestore(&tp->phy_lock, flags);
}

static void
rtl8101_phy_power_up (struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    //unsigned long flags;
    
    spin_lock_irqsave(&tp->phy_lock, flags);
    mdio_write(tp, 0x1f, 0x0000);
    mdio_write(tp, MII_BMCR, BMCR_ANENABLE);
    spin_unlock_irqrestore(&tp->phy_lock, flags);
}

#if DISABLED_CODE

static int __devinit
rtl8101_init_board(struct pci_dev *pdev,
                   struct net_device **dev_out,
                   void __iomem **ioaddr_out)
{
    void __iomem *ioaddr;
    struct net_device *dev;
    struct rtl8101_private *tp;
    int rc = -ENOMEM, i, acpi_idle_state = 0, pm_cap;
    
    assert(ioaddr_out != NULL);
    
    /* dev zeroed in alloc_etherdev */
    dev = alloc_etherdev(sizeof (*tp));
    if (dev == NULL) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_drv(&debug))
            dev_err(&pdev->dev, "unable to alloc new ethernet\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        goto err_out;
    }
    
    SET_MODULE_OWNER(dev);
    SET_NETDEV_DEV(dev, &pdev->dev);
    tp = netdev_priv(dev);
    tp->dev = dev;
    tp->pci_dev = pdev;
    tp->msg_enable = netif_msg_init(debug.msg_enable, R8101_MSG_DEFAULT);
    
    /* enable device (incl. PCI PM wakeup and hotplug setup) */
    rc = pci_enable_device(pdev);
    if (rc < 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev, "enable failure\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        goto err_out_free_dev;
    }
    
    rc = pci_set_mwi(pdev);
    if (rc < 0)
        goto err_out_disable;
    
    /* save power state before pci_enable_device overwrites it */
    pm_cap = pci_find_capability(pdev, PCI_CAP_ID_PM);
    if (pm_cap) {
        u16 pwr_command;
        
        pci_read_config_word(pdev, pm_cap + PCI_PM_CTRL, &pwr_command);
        acpi_idle_state = pwr_command & PCI_PM_CTRL_STATE_MASK;
    } else {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp)) {
            dev_err(&pdev->dev, "PowerManagement capability not found.\n");
        }
#else
        printk("PowerManagement capability not found.\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    }
    
    /* make sure PCI base addr 1 is MMIO */
    if (!(pci_resource_flags(pdev, 2) & IORESOURCE_MEM)) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev,
                    "region #1 not an MMIO resource, aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        rc = -ENODEV;
        goto err_out_mwi;
    }
    /* check for weird/broken PCI region reporting */
    if (pci_resource_len(pdev, 2) < R8101_REGS_SIZE) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev,
                    "Invalid PCI region size(s), aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        rc = -ENODEV;
        goto err_out_mwi;
    }
    
    rc = pci_request_regions(pdev, MODULENAME);
    if (rc < 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev, "could not request regions.\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        goto err_out_mwi;
    }
    
    if ((sizeof(dma_addr_t) > 4) &&
        !pci_set_dma_mask(pdev, DMA_BIT_MASK(64)) && use_dac) {
        tp->cp_cmd |= PCIDAC;
        dev->features |= NETIF_F_HIGHDMA;
    } else {
        rc = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
        if (rc < 0) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
            if (netif_msg_probe(tp))
                dev_err(&pdev->dev, "DMA configuration failed.\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
            goto err_out_free_res;
        }
    }
    
    pci_set_master(pdev);
    
    /* ioremap MMIO region */
    ioaddr = ioremap(pci_resource_start(pdev, 2), R8101_REGS_SIZE);
    if (ioaddr == NULL) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp))
            dev_err(&pdev->dev, "cannot remap MMIO, aborting\n");
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        rc = -EIO;
        goto err_out_free_res;
    }
    
    pci_write_config_dword(pdev, 0x30, 0);
    
    /* Unneeded ? Don't mess with Mrs. Murphy. */
    rtl8101_irq_mask_and_ack(ioaddr);
    
    /* Soft reset the chip. */
    RTL_W8(ChipCmd, CmdReset);
    
    /* Check that the chip has finished the reset. */
    for (i = 1000; i > 0; i--) {
        if ((RTL_R8(ChipCmd) & CmdReset) == 0)
            break;
        udelay(10);
    }
    
    /* Identify chip attached to board */
    rtl8101_get_mac_version(tp, ioaddr);
    
    rtl8101_print_mac_version(tp);
    
    for (i = ARRAY_SIZE(rtl_chip_info) - 1; i >= 0; i--) {
        if (tp->mcfg == rtl_chip_info[i].mcfg)
            break;
    }
    
    if (i < 0) {
        /* Unknown chip: assume array element #0, original RTL-8101 */
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        if (netif_msg_probe(tp)) {
            dev_printk(KERN_DEBUG, &pdev->dev, "unknown chip version, assuming %s\n", rtl_chip_info[0].name);
        }
#else
        printk("Realtek unknown chip version, assuming %s\n", rtl_chip_info[0].name);
#endif //LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
        i++;
    }
    
    tp->chipset = i;
    
    *ioaddr_out = ioaddr;
    *dev_out = dev;
out:
    return rc;
    
err_out_free_res:
    pci_release_regions(pdev);
    
err_out_mwi:
    pci_clear_mwi(pdev);
    
err_out_disable:
    pci_disable_device(pdev);
    
err_out_free_dev:
    free_netdev(dev);
err_out:
    *ioaddr_out = NULL;
    *dev_out = NULL;
    goto out;
}

static void
rtl8101_esd_timer(unsigned long __opaque)
{
    struct net_device *dev = (struct net_device *)__opaque;
    struct rtl8101_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    struct timer_list *timer = &tp->esd_timer;
    unsigned long timeout = RTL8101_ESD_TIMEOUT;
    unsigned long flags;
    u8 cmd;
    u8 cls;
    u16 io_base_l;
    u16 io_base_h;
    u16 mem_base_l;
    u16 mem_base_h;
    u8 ilr;
    u16 resv_0x20_l;
    u16 resv_0x20_h;
    u16 resv_0x24_l;
    u16 resv_0x24_h;
    
    spin_lock_irqsave(&tp->lock, flags);
    
    tp->esd_flag = 0;
    
    pci_read_config_byte(pdev, PCI_COMMAND, &cmd);
    if (cmd != tp->pci_cfg_space.cmd) {
        pci_write_config_byte(pdev, PCI_COMMAND, tp->pci_cfg_space.cmd);
        tp->esd_flag |= 0x0001;
    }
    
    pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &cls);
    if (cls != tp->pci_cfg_space.cls) {
        pci_write_config_byte(pdev, PCI_CACHE_LINE_SIZE, tp->pci_cfg_space.cls);
        tp->esd_flag |= 0x0002;
    }
    
    pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &io_base_l);
    if (io_base_l != tp->pci_cfg_space.io_base_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_0, tp->pci_cfg_space.io_base_l);
        tp->esd_flag |= 0x0004;
    }
    
    pci_read_config_word(pdev, PCI_BASE_ADDRESS_0 + 2, &io_base_h);
    if (io_base_h != tp->pci_cfg_space.io_base_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_0 + 2, tp->pci_cfg_space.io_base_h);
        
        /* Put carrier down when fnf2 disable network */
        netif_carrier_off(dev);
        
        tp->esd_flag |= 0x0008;
    }
    
    pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &mem_base_l);
    if (mem_base_l != tp->pci_cfg_space.mem_base_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_2, tp->pci_cfg_space.mem_base_l);
        tp->esd_flag |= 0x0010;
    }
    
    pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &mem_base_h);
    if (mem_base_h != tp->pci_cfg_space.mem_base_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, tp->pci_cfg_space.mem_base_h);
        tp->esd_flag |= 0x0020;
    }
    
    pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &ilr);
    if (ilr != tp->pci_cfg_space.ilr) {
        pci_write_config_byte(pdev, PCI_INTERRUPT_LINE, tp->pci_cfg_space.ilr);
        tp->esd_flag |= 0x0040;
    }
    
    pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &resv_0x20_l);
    if (resv_0x20_l != tp->pci_cfg_space.resv_0x20_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_4, tp->pci_cfg_space.resv_0x20_l);
        tp->esd_flag |= 0x0080;
    }
    
    pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &resv_0x20_h);
    if (resv_0x20_h != tp->pci_cfg_space.resv_0x20_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, tp->pci_cfg_space.resv_0x20_h);
        tp->esd_flag |= 0x0100;
    }
    
    pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &resv_0x24_l);
    if (resv_0x24_l != tp->pci_cfg_space.resv_0x24_l) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_5, tp->pci_cfg_space.resv_0x24_l);
        tp->esd_flag |= 0x0200;
    }
    
    pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &resv_0x24_h);
    if (resv_0x24_h != tp->pci_cfg_space.resv_0x24_h) {
        pci_write_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, tp->pci_cfg_space.resv_0x24_h);
        tp->esd_flag |= 0x0400;
    }
    
    if (tp->esd_flag != 0) {
        netif_stop_queue(dev);
        netif_carrier_off(dev);
        rtl8101_hw_reset(dev);
        rtl8101_tx_clear(tp);
        rtl8101_init_ring_indexes(tp);
        rtl8101_hw_init(dev);
        rtl8101_powerup_pll(dev);
        rtl8101_hw_ephy_config(dev);
        rtl8101_hw_phy_config(dev);
        rtl8101_hw_start(dev);
        rtl8101_set_speed(dev, tp->autoneg, tp->speed, tp->duplex);
        tp->esd_flag = 0;
    }
    spin_unlock_irqrestore(&tp->lock, flags);
    
    mod_timer(timer, jiffies + timeout);
}

static void
rtl8101_link_timer(unsigned long __opaque)
{
    struct net_device *dev = (struct net_device *)__opaque;
    struct rtl8101_private *tp = netdev_priv(dev);
    struct timer_list *timer = &tp->link_timer;
    unsigned long flags;
    
    spin_lock_irqsave(&tp->lock, flags);
    rtl8101_check_link_status(dev);
    spin_unlock_irqrestore(&tp->lock, flags);
    
    mod_timer(timer, jiffies + RTL8101_LINK_TIMEOUT);
}

static unsigned rtl8101_try_msi(struct pci_dev *pdev, void __iomem *ioaddr)
{
    unsigned msi = 0;
    
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
    if (pci_enable_msi(pdev))
        dev_info(&pdev->dev, "no MSI. Back to INTx.\n");
    else
        msi |= RTL_FEATURE_MSI;
#endif
    
    return msi;
}

static void rtl8101_disable_msi(struct pci_dev *pdev, struct rtl8101_private *tp)
{
    if (tp->features & RTL_FEATURE_MSI) {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13)
        pci_disable_msi(pdev);
#endif
        tp->features &= ~RTL_FEATURE_MSI;
    }
}

#endif  /* DISABLED_CODE */

void rtl8101_aspm_fix1(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int data;
    
    data = rtl8101_csi_read(tp, 0x110);
    
    if ((data & (1 << 7)) && (data & (1 << 8))) {
        rtl8101_ephy_write(ioaddr, 0x01, 0x2e65);
        rtl8101_ephy_write(ioaddr, 0x01, 0x6e65);
    }
}

#if DISABLED_CODE

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,29)
static const struct net_device_ops rtl8101_netdev_ops = {
    .ndo_open		= rtl8101_open,
    .ndo_stop		= rtl8101_close,
    .ndo_get_stats		= rtl8101_get_stats,
    .ndo_start_xmit		= rtl8101_start_xmit,
    .ndo_tx_timeout		= rtl8101_tx_timeout,
    .ndo_change_mtu		= rtl8101_change_mtu,
    .ndo_set_mac_address	= rtl8101_set_mac_address,
    .ndo_do_ioctl		= rtl8101_do_ioctl,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0)
    .ndo_set_multicast_list	= rtl8101_set_rx_mode,
#else
    .ndo_set_rx_mode	= rtl8101_set_rx_mode,
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
#ifdef CONFIG_R8101_VLAN
    dev->vlan_rx_register	= rtl8101_vlan_rx_register,
#endif
#else
    .ndo_fix_features	= rtl8101_fix_features,
    .ndo_set_features	= rtl8101_set_features,
#endif
#ifdef CONFIG_NET_POLL_CONTROLLER
    .ndo_poll_controller	= rtl8101_netpoll,
#endif
};
#endif

static int __devinit
rtl8101_init_one(struct pci_dev *pdev,
                 const struct pci_device_id *ent)
{
    struct net_device *dev = NULL;
    struct rtl8101_private *tp;
    void __iomem *ioaddr = NULL;
    static int board_idx = -1;
    u8 autoneg, duplex;
    u16 speed;
    int rc;
    
    assert(pdev != NULL);
    assert(ent != NULL);
    
    board_idx++;
    
    if (netif_msg_drv(&debug)) {
        printk(KERN_INFO "%s Fast Ethernet driver %s loaded\n",
               MODULENAME, RTL8101_VERSION);
    }
    
    rc = rtl8101_init_board(pdev, &dev, &ioaddr);
    if (rc)
        return rc;
    
    tp = netdev_priv(dev);
    assert(ioaddr != NULL);
    
    tp->mmio_addr = ioaddr;
    tp->set_speed = rtl8101_set_speed_xmii;
    tp->get_settings = rtl8101_gset_xmii;
    tp->phy_reset_enable = rtl8101_xmii_reset_enable;
    tp->phy_reset_pending = rtl8101_xmii_reset_pending;
    tp->link_ok = rtl8101_xmii_link_ok;
    
    tp->features |= rtl8101_try_msi(pdev, ioaddr);
    
    RTL_NET_DEVICE_OPS(rtl8101_netdev_ops);
    
    SET_ETHTOOL_OPS(dev, &rtl8101_ethtool_ops);
    
    dev->watchdog_timeo = RTL8101_TX_TIMEOUT;
    dev->irq = pdev->irq;
    dev->base_addr = (unsigned long) ioaddr;
    
#ifdef CONFIG_R8101_NAPI
    RTL_NAPI_CONFIG(dev, tp, rtl8101_poll, R8101_NAPI_WEIGHT);
#endif
    
#ifdef CONFIG_R8101_VLAN
    dev->features |= NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
    dev->vlan_rx_kill_vid = rtl8101_vlan_rx_kill_vid;
#endif//LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
#endif
    
    dev->features |= NETIF_F_IP_CSUM;
    tp->cp_cmd |= RTL_R16(CPlusCmd);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    tp->cp_cmd |= RxChkSum;
#else
    dev->features |= NETIF_F_RXCSUM | NETIF_F_SG;
    dev->hw_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
    NETIF_F_RXCSUM | NETIF_F_HW_VLAN_TX | NETIF_F_HW_VLAN_RX;
    dev->vlan_features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_TSO |
    NETIF_F_HIGHDMA;
#endif
    
    switch (tp->mcfg) {
        case CFG_METHOD_1:
        case CFG_METHOD_2:
        case CFG_METHOD_3:
            tp->intr_mask = RxDescUnavail | TxDescUnavail | TxOK | RxOK | SWInt;
            break;
        default:
            tp->intr_mask = RxDescUnavail | TxOK | RxOK | SWInt;
            break;
    }
    
    spin_lock_init(&tp->lock);
    
    spin_lock_init(&tp->phy_lock);
    
    rtl8101_get_bios_setting(dev);
    
    rtl8101_exit_oob(dev);
    
    rtl8101_hw_init(dev);
    
    rtl8101_hw_reset(dev);
    
    /* Get production from EEPROM */
    if (tp->mcfg == CFG_METHOD_17 && (mac_ocp_read(tp, 0xDC00) & BIT_3))
        tp->eeprom_type = EEPROM_TYPE_NONE;
    else
        rtl_eeprom_type(tp);
    
    if (tp->eeprom_type == EEPROM_TYPE_93C46 || tp->eeprom_type == EEPROM_TYPE_93C56)
        rtl_set_eeprom_sel_low(ioaddr);
    
    rtl8101_get_mac_address(dev);
    
    pci_set_drvdata(pdev, dev);
    
    if (netif_msg_probe(tp)) {
        printk(KERN_INFO "%s: %s at 0x%lx, "
               "%2.2x:%2.2x:%2.2x:%2.2x:%2.2x:%2.2x, "
               "IRQ %d\n",
               dev->name,
               rtl_chip_info[ent->driver_data].name,
               dev->base_addr,
               dev->dev_addr[0], dev->dev_addr[1],
               dev->dev_addr[2], dev->dev_addr[3],
               dev->dev_addr[4], dev->dev_addr[5], dev->irq);
    }
    
    rtl8101_link_option(board_idx, &autoneg, &speed, &duplex);
    
    rc = register_netdev(dev);
    if (rc) {
        rtl8101_release_board(pdev, dev, ioaddr);
        return rc;
    }
    
    printk(KERN_INFO "%s: This product is covered by one or more of the following patents: US6,570,884, US6,115,776, and US6,327,625.\n", MODULENAME);
    
    if (netif_msg_probe(tp)) {
        printk(KERN_DEBUG "%s: Identified chip type is '%s'.\n",
               dev->name, rtl_chip_info[tp->chipset].name);
    }
    
    netif_carrier_off(dev);
    
    printk("%s", GPL_CLAIM);
    
    return 0;
}

static void __devexit
rtl8101_remove_one(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8101_private *tp = netdev_priv(dev);
    
    assert(dev != NULL);
    assert(tp != NULL);
    
    flush_scheduled_work();
    
    unregister_netdev(dev);
    
    rtl8101_disable_msi(pdev, tp);
    rtl8101_release_board(pdev, dev, tp->mmio_addr);
    pci_set_drvdata(pdev, NULL);
}

static void rtl8101_set_rxbufsize(struct rtl8101_private *tp,
                                  struct net_device *dev)
{
    switch (tp->mcfg) {
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
            tp->rx_buf_sz = 0x05F3;
            break;
        default:
            tp->rx_buf_sz = 0x05EF;
            break;
    }
}

static int rtl8101_open(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    int retval;
    
    retval = -ENOMEM;
    
    rtl8101_set_rxbufsize(tp, dev);
    
    /*
     * Rx and Tx desscriptors needs 256 bytes alignment.
     * pci_alloc_consistent provides more.
     */
    tp->TxDescArray = pci_alloc_consistent(pdev, R8101_TX_RING_BYTES,
                                           &tp->TxPhyAddr);
    if (!tp->TxDescArray)
        goto out;
    
    tp->RxDescArray = pci_alloc_consistent(pdev, R8101_RX_RING_BYTES,
                                           &tp->RxPhyAddr);
    if (!tp->RxDescArray)
        goto err_free_tx;
    
    tp->tally_vaddr = pci_alloc_consistent(pdev, sizeof(*tp->tally_vaddr), &tp->tally_paddr);
    if (!tp->tally_vaddr)
        goto err_free_rx;
    
    retval = rtl8101_init_ring(dev);
    if (retval < 0)
        goto err_free_counters;
    
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    INIT_WORK(&tp->task, NULL, dev);
#else
    INIT_DELAYED_WORK(&tp->task, NULL);
#endif
    
#ifdef	CONFIG_R8101_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    RTL_NAPI_ENABLE(dev, &tp->napi);
#endif
#endif
    
    rtl8101_exit_oob(dev);
    
    rtl8101_tally_counter_clear(tp);
    
    rtl8101_hw_init(dev);
    
    rtl8101_hw_reset(dev);
    
    rtl8101_powerup_pll(dev);
    
    rtl8101_hw_ephy_config(dev);
    
    rtl8101_hw_phy_config(dev);
    
    rtl8101_hw_start(dev);
    
    rtl8101_dsm(dev, DSM_IF_UP);
    
    rtl8101_set_speed(dev, autoneg, speed, duplex);
    
    retval = request_irq(dev->irq, rtl8101_interrupt, (tp->features & RTL_FEATURE_MSI) ? 0 : SA_SHIRQ, dev->name, dev);
    
    if (retval < 0)
        goto err_free_counters;
    
    if (tp->esd_flag == 0)
        rtl8101_request_esd_timer(dev);
    
    rtl8101_request_link_timer(dev);
out:
    return retval;
    
err_free_counters:
    pci_free_consistent(pdev, sizeof(*tp->tally_vaddr), tp->tally_vaddr, tp->tally_paddr);
    
    tp->tally_vaddr = NULL;
err_free_rx:
    pci_free_consistent(pdev, R8101_RX_RING_BYTES, tp->RxDescArray,
                        tp->RxPhyAddr);
    tp->RxDescArray = NULL;
err_free_tx:
    pci_free_consistent(pdev, R8101_TX_RING_BYTES, tp->TxDescArray,
                        tp->TxPhyAddr);
    tp->TxDescArray = NULL;
    goto out;
}

#endif  /* DISABLED_CODE */

void rtl8101_dsm(struct net_device *dev, int dev_state)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    
    switch (dev_state) {
        case DSM_MAC_INIT:
            if ((tp->mcfg == CFG_METHOD_4) ||
                (tp->mcfg == CFG_METHOD_5) ||
                (tp->mcfg == CFG_METHOD_6)) {
                if (RTL_R8(MACDBG) & 0x80) {
                    mdio_write(tp, 0x1f, 0x0000);
                    mdio_write(tp, 0x11, mdio_read(tp, 0x11) & ~(1 << 12));
                    RTL_W8(GPIO, RTL_R8(GPIO) | GPIO_en);
                } else {
                    RTL_W8(GPIO, RTL_R8(GPIO) & ~GPIO_en);
                }
            }
            
            break;
        case DSM_NIC_GOTO_D3:
        case DSM_IF_DOWN:
            if (RTL_R8(MACDBG) & 0x80) {
                if ((tp->mcfg == CFG_METHOD_4) || (tp->mcfg == CFG_METHOD_5)) {
                    RTL_W8(GPIO, RTL_R8(GPIO) | GPIO_en);
                    mdio_write(tp, 0x11, mdio_read(tp, 0x11) | (1 << 12));
                } else if (tp->mcfg == CFG_METHOD_6) {
                    RTL_W8(GPIO, RTL_R8(GPIO) & ~GPIO_en);
                }
            }
            break;
        case DSM_NIC_RESUME_D3:
        case DSM_IF_UP:
            if (RTL_R8(MACDBG) & 0x80) {
                if ((tp->mcfg == CFG_METHOD_4) || (tp->mcfg == CFG_METHOD_5)) {
                    RTL_W8(GPIO, RTL_R8(GPIO) & ~GPIO_en);
                } else if (tp->mcfg == CFG_METHOD_6) {
                    RTL_W8(GPIO, RTL_R8(GPIO) | GPIO_en);
                }
            }
            
            break;
    }
    
}

#if DISABLED_CODE

static void
rtl8101_hw_set_rx_packet_filter(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    u32 mc_filter[2];	/* Multicast hash filter */
    int i, j, rx_mode;
    u32 tmp = 0;
    
    if (dev->flags & IFF_PROMISC) {
        /* Unconditionally log net taps. */
        if (netif_msg_link(tp)) {
            printk(KERN_NOTICE "%s: Promiscuous mode enabled.\n",
                   dev->name);
        }
        rx_mode =
        AcceptBroadcast | AcceptMulticast | AcceptMyPhys |
        AcceptAllPhys;
        mc_filter[1] = mc_filter[0] = 0xffffffff;
    } else if ((netdev_mc_count(dev) > multicast_filter_limit)
               || (dev->flags & IFF_ALLMULTI)) {
        /* Too many to filter perfectly -- accept all multicasts. */
        rx_mode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
        mc_filter[1] = mc_filter[0] = 0xffffffff;
    } else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
        struct dev_mc_list *mclist;
        rx_mode = AcceptBroadcast | AcceptMyPhys;
        mc_filter[1] = mc_filter[0] = 0;
        for (i = 0, mclist = dev->mc_list; mclist && i < dev->mc_count;
             i++, mclist = mclist->next) {
            int bit_nr = ether_crc(ETH_ALEN, mclist->dmi_addr) >> 26;
            mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
            rx_mode |= AcceptMulticast;
        }
#else
        struct netdev_hw_addr *ha;
        
        rx_mode = AcceptBroadcast | AcceptMyPhys;
        mc_filter[1] = mc_filter[0] = 0;
        netdev_for_each_mc_addr(ha, dev) {
            int bit_nr = ether_crc(ETH_ALEN, ha->addr) >> 26;
            mc_filter[bit_nr >> 5] |= 1 << (bit_nr & 31);
            rx_mode |= AcceptMulticast;
        }
#endif
    }
    
    tmp = rtl8101_rx_config | rx_mode |
    (RTL_R32(RxConfig) & rtl_chip_info[tp->chipset].RxConfigMask);
    
    for (i = 0; i < 2; i++) {
        u32 mask = 0x000000ff;
        u32 tmp1 = 0;
        u32 tmp2 = 0;
        int x = 0;
        int y = 0;
        
        for (j = 0; j < 4; j++) {
            tmp1 = mc_filter[i] & mask;
            x = 32 - (8 + 16 * j);
            y = x - 2 * x;
            
            if (x > 0)
                tmp2 = tmp2 | (tmp1 << x);
            else
                tmp2 = tmp2 | (tmp1 >> y);
            
            mask = mask << 8;
        }
        mc_filter[i] = tmp2;
    }
    
    RTL_W32(RxConfig, tmp);
    RTL_W32(MAR0 + 0, mc_filter[1]);
    RTL_W32(MAR0 + 4, mc_filter[0]);
}

static void
rtl8101_set_rx_mode(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    
    spin_lock_irqsave(&tp->lock, flags);
    
    rtl8101_hw_set_rx_packet_filter(dev);
    
    spin_unlock_irqrestore(&tp->lock, flags);
}

static void
rtl8101_hw_start(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    struct pci_dev *pdev = tp->pci_dev;
    u8 link_control, options1, options2;
    u32 csi_tmp;
    unsigned long flags;
    
    netif_stop_queue(dev);
    
    RTL_W32(RxConfig, (RX_DMA_BURST << RxCfgDMAShift));
    
    rtl8101_hw_reset(dev);
    
    RTL_W8(Cfg9346, Cfg9346_Unlock);
    switch (tp->mcfg) {
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
            RTL_W8(Config5, RTL_R8(Config5) & ~BIT_0);
            RTL_W8(Config2, RTL_R8(Config2) & ~BIT_7);
            RTL_W8(0xF1, RTL_R8(0xF1) & ~BIT_7);
            break;
    }
    RTL_W8(MTPS, Reserved1_data);
    
    /* Set DMA burst size and Interframe Gap Time */
    RTL_W32(TxConfig, (TX_DMA_BURST << TxDMAShift) |
            (InterFrameGap << TxInterFrameGapShift));
    
    tp->cp_cmd &= 0x2063;
    
    RTL_W16(IntrMitigate, 0x0000);
    
    rtl8101_tally_counter_addr_fill(tp);
    
    rtl8101_desc_addr_fill(tp);
    
    if (tp->mcfg == CFG_METHOD_4) {
        set_offset70F(tp, 0x17);
        set_offset79(tp, 0x50);
        
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);
            
            RTL_W8(DBG_reg, 0x98);
            RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
            RTL_W8(Config4, RTL_R8(Config4) | BIT_2);
            
            pci_write_config_byte(pdev, 0x81, 1);
        }
        
        RTL_W8(Config1, 0x0f);
        
        RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_5) {
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);
            
            RTL_W8(DBG_reg, 0x98);
            RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
            RTL_W8(Config4, RTL_R8(Config4) | BIT_2);
            
            pci_write_config_byte(pdev, 0x81, 1);
        }
        
        set_offset79(tp, 0x50);
        
        RTL_W8(Config1, 0x0f);
        
        RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_6) {
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);
            
            RTL_W8(DBG_reg, 0x98);
            RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
            RTL_W8(Config4, RTL_R8(Config4) | BIT_2);
            
            pci_write_config_byte(pdev, 0x81, 1);
        }
        
        set_offset79(tp, 0x50);
        
        //		RTL_W8(Config1, 0xDF);
        
        RTL_W8(0xF4, 0x01);
        
        RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_7) {
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);
            
            RTL_W8(DBG_reg, 0x98);
            RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
            RTL_W8(Config4, RTL_R8(Config4) | BIT_2);
            
            pci_write_config_byte(pdev, 0x81, 1);
        }
        
        set_offset79(tp, 0x50);
        
        //		RTL_W8(Config1, (RTL_R8(Config1)&0xC0)|0x1F);
        
        RTL_W8(0xF4, 0x01);
        
        RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);
        
        RTL_W8(0xF5, RTL_R8(0xF5) | BIT_2);
    } else if (tp->mcfg == CFG_METHOD_8) {
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);
            
            RTL_W8(DBG_reg, 0x98);
            RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
            RTL_W8(Config4, RTL_R8(Config4) | BIT_2);
            RTL_W8(0xF4, RTL_R8(0xF4) | BIT_3);
            RTL_W8(0xF5, RTL_R8(0xF5) | BIT_2);
            
            pci_write_config_byte(pdev, 0x81, 1);
            
            if (rtl8101_ephy_read(ioaddr, 0x10)==0x0008) {
                rtl8101_ephy_write(ioaddr, 0x10, 0x000C);
            }
        }
        
        pci_read_config_byte(pdev, 0x80, &link_control);
        if (link_control & 3)
            rtl8101_ephy_write(ioaddr, 0x02, 0x011F);
        
        set_offset79(tp, 0x50);
        
        //		RTL_W8(Config1, (RTL_R8(Config1)&0xC0)|0x1F);
        
        RTL_W8(0xF4, RTL_R8(0xF4) | BIT_0);
        
        RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_9) {
        pci_read_config_byte(pdev, 0x81, &link_control);
        if (link_control == 1) {
            pci_write_config_byte(pdev, 0x81, 0);
            
            RTL_W8(DBG_reg, 0x98);
            RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
            RTL_W8(Config4, RTL_R8(Config4) | BIT_2);
            
            pci_write_config_byte(pdev, 0x81, 1);
        }
        
        set_offset79(tp, 0x50);
        
        //		RTL_W8(Config1, 0xDF);
        
        RTL_W8(0xF4, 0x01);
        
        RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_10) {
        set_offset70F(tp, 0x27);
        set_offset79(tp, 0x50);
        
        /* tx checksum offload enable */
        dev->features |= NETIF_F_IP_CSUM;
        
        RTL_W8(0xF3, RTL_R8(0xF3) | BIT_5);
        RTL_W8(0xF3, RTL_R8(0xF3) & ~BIT_5);
        
        RTL_W8(0xD0, RTL_R8(0xD0) | BIT_7 | BIT_6);
        
        RTL_W8(0xF1, RTL_R8(0xF1) | BIT_6 | BIT_5 | BIT_4 | BIT_2 | BIT_1);
        
        if (aspm)
            RTL_W8(0xF1, RTL_R8(0xF1) | BIT_7);
        
        RTL_W8(Config5, (RTL_R8(Config5)&~0x08) | BIT_0);
        RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
        
        RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);
    } else if (tp->mcfg == CFG_METHOD_11 || tp->mcfg == CFG_METHOD_12 ||
               tp->mcfg == CFG_METHOD_13) {
        u8	pci_config;
        
        tp->cp_cmd &= 0x2063;
        
        /* tx checksum offload enable */
        dev->features |= NETIF_F_IP_CSUM;
        
        pci_read_config_byte(pdev, 0x80, &pci_config);
        if (pci_config & 0x03) {
            RTL_W8(Config5, RTL_R8(Config5) | BIT_0);
            RTL_W8(0xF2, RTL_R8(0xF2) | BIT_7);
            if (aspm)
                RTL_W8(0xF1, RTL_R8(0xF1) | BIT_7);
            RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
        }
        
        RTL_W8(0xF1, RTL_R8(0xF1) | BIT_5 | BIT_3);
        RTL_W8(0xF2, RTL_R8(0xF2) & ~BIT_0);
        RTL_W8(0xD3, RTL_R8(0xD3) | BIT_3 | BIT_2);
        RTL_W8(0xD0, RTL_R8(0xD0) | BIT_6);
        RTL_W16(0xE0, RTL_R16(0xE0) & ~0xDF9C);
        
        if (tp->mcfg == CFG_METHOD_11)
            RTL_W8(Config5, RTL_R8(Config5) & ~BIT_0);
    } else if (tp->mcfg == CFG_METHOD_14) {
        set_offset70F(tp, 0x27);
        set_offset79(tp, 0x50);
        
        rtl8101_eri_write(ioaddr, 0xC8, 4, 0x00000002, ERIAR_ExGMAC);
        rtl8101_eri_write(ioaddr, 0xE8, 4, 0x00000006, ERIAR_ExGMAC);
        RTL_W32(TxConfig, RTL_R32(TxConfig) | BIT_7);
        RTL_W8(0xD3, RTL_R8(0xD3) & ~BIT_7);
        csi_tmp = rtl8101_eri_read(ioaddr, 0xDC, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_0;
        rtl8101_eri_write( ioaddr, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
        csi_tmp |= BIT_0;
        rtl8101_eri_write( ioaddr, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
        
        rtl8101_ephy_write(ioaddr, 0x19, 0xff64);
        
        RTL_W8(Config5, RTL_R8(Config5) | BIT_0);
        RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
        
        rtl8101_eri_write(ioaddr, 0xC0, 2, 0x00000000, ERIAR_ExGMAC);
        rtl8101_eri_write(ioaddr, 0xB8, 2, 0x00000000, ERIAR_ExGMAC);
        rtl8101_eri_write(ioaddr, 0xD5, 1, 0x0000000E, ERIAR_ExGMAC);
    } else if (tp->mcfg == CFG_METHOD_15 || tp->mcfg == CFG_METHOD_16) {
        u8	pci_config;
        
        tp->cp_cmd &= 0x2063;
        
        /* tx checksum offload enable */
        dev->features |= NETIF_F_IP_CSUM;
        
        pci_read_config_byte(pdev, 0x80, &pci_config);
        if (pci_config & 0x03) {
            RTL_W8(Config5, RTL_R8(Config5) | BIT_0);
            RTL_W8(0xF2, RTL_R8(0xF2) | BIT_7);
            if (aspm)
                RTL_W8(0xF1, RTL_R8(0xF1) | BIT_7);
            RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
        }
        
        RTL_W8(0xF1, RTL_R8(0xF1) | BIT_5 | BIT_3);
        RTL_W8(0xF2, RTL_R8(0xF2) & ~BIT_0);
        RTL_W8(0xD3, RTL_R8(0xD3) | BIT_3 | BIT_2);
        RTL_W8(0xD0, RTL_R8(0xD0) & ~BIT_6);
        RTL_W16(0xE0, RTL_R16(0xE0) & ~0xDF9C);
    } else if (tp->mcfg == CFG_METHOD_17) {
        u8 data8;
        
        set_offset70F(tp, 0x17);
        set_offset79(tp, 0x50);
        
        rtl8101_eri_write(ioaddr, 0xC8, 4, 0x00080002, ERIAR_ExGMAC);
        rtl8101_eri_write(ioaddr, 0xCC, 1, 0x38, ERIAR_ExGMAC);
        rtl8101_eri_write(ioaddr, 0xD0, 1, 0x48, ERIAR_ExGMAC);
        rtl8101_eri_write(ioaddr, 0xE8, 4, 0x00100006, ERIAR_ExGMAC);
        
        RTL_W32(TxConfig, RTL_R32(TxConfig) | BIT_7);
        
        csi_tmp = rtl8101_eri_read(ioaddr, 0xDC, 1, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_0;
        rtl8101_eri_write(ioaddr, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
        csi_tmp |= BIT_0;
        rtl8101_eri_write(ioaddr, 0xDC, 1, csi_tmp, ERIAR_ExGMAC);
        
        RTL_W8(Config3, RTL_R8(Config3) & ~Beacon_en);
        
        tp->cp_cmd = RTL_R16(CPlusCmd) &
        ~(EnableBist | Macdbgo_oe | Force_halfdup |
          Force_rxflow_en | Force_txflow_en |
          Cxpl_dbg_sel | ASF | PktCntrDisable |
          Macdbgo_sel);
        
        RTL_W8(0x1B, RTL_R8(0x1B) & ~0x07);
        
        RTL_W8(TDFNR, 0x4);
        
        if (aspm)
            RTL_W8(0xF1, RTL_R8(0xF1) | BIT_7);
        
        /* tx checksum offload enable */
        dev->features |= NETIF_F_IP_CSUM;
        
        RTL_W8(0xD0, RTL_R8(0xD0) | BIT_6);
        RTL_W8(0xF2, RTL_R8(0xF2) | BIT_6);
        
        RTL_W8(0xD0, RTL_R8(0xD0) | BIT_7);
        
        rtl8101_eri_write(ioaddr, 0xC0, 2, 0x0000, ERIAR_ExGMAC);
        rtl8101_eri_write(ioaddr, 0xB8, 4, 0x00000000, ERIAR_ExGMAC);
        
        rtl8101_eri_write(ioaddr, 0x5F0, 2, 0x4f87, ERIAR_ExGMAC);
        
        csi_tmp = rtl8101_eri_read(ioaddr, 0xD4, 4, ERIAR_ExGMAC);
        csi_tmp  |= ( BIT_7 | BIT_8 | BIT_9 | BIT_10 | BIT_11 | BIT_12 );
        rtl8101_eri_write(ioaddr, 0xD4, 4, csi_tmp, ERIAR_ExGMAC);
        
        csi_tmp = rtl8101_eri_read(ioaddr, 0x1B0, 4, ERIAR_ExGMAC);
        csi_tmp &= ~BIT_12;
        rtl8101_eri_write(ioaddr, 0x1B0, 4, csi_tmp, ERIAR_ExGMAC);
        
        csi_tmp = rtl8101_eri_read(ioaddr, 0x2FC, 1, ERIAR_ExGMAC);
        csi_tmp &= ~(BIT_0 | BIT_1 | BIT_2);
        csi_tmp |= BIT_0;
        rtl8101_eri_write(ioaddr, 0x2FC, 1, csi_tmp, ERIAR_ExGMAC);
        
        csi_tmp = rtl8101_eri_read(ioaddr, 0x1D0, 1, ERIAR_ExGMAC);
        csi_tmp |= BIT_1;
        rtl8101_eri_write(ioaddr, 0x1D0, 1, csi_tmp, ERIAR_ExGMAC);
        
        if (aspm) {
            csi_tmp = rtl8101_eri_read(ioaddr, 0x3F2, 2, ERIAR_ExGMAC);
            csi_tmp &= ~( BIT_8 | BIT_9  | BIT_10 | BIT_11  | BIT_12  | BIT_13  | BIT_14 | BIT_15 );
            csi_tmp |= ( BIT_9 | BIT_10 | BIT_13  | BIT_14 | BIT_15 );
            rtl8101_eri_write(ioaddr, 0x3F2, 2, csi_tmp, ERIAR_ExGMAC);
            csi_tmp = rtl8101_eri_read(ioaddr, 0x3F5, 1, ERIAR_ExGMAC);
            csi_tmp |= BIT_6 | BIT_7;
            rtl8101_eri_write(ioaddr, 0x3F5, 1, csi_tmp, ERIAR_ExGMAC);
            mac_ocp_write(tp, 0xE02C, 0x1880);
            mac_ocp_write(tp, 0xE02E, 0x4880);
            rtl8101_eri_write(ioaddr, 0x2E8, 2, 0x9003, ERIAR_ExGMAC);
            rtl8101_eri_write(ioaddr, 0x2EA, 2, 0x9003, ERIAR_ExGMAC);
            rtl8101_eri_write(ioaddr, 0x2EC, 2, 0x9003, ERIAR_ExGMAC);
            rtl8101_eri_write(ioaddr, 0x2E2, 2, 0x883C, ERIAR_ExGMAC);
            rtl8101_eri_write(ioaddr, 0x2E4, 2, 0x8C12, ERIAR_ExGMAC);
            rtl8101_eri_write(ioaddr, 0x2E6, 2, 0x9003, ERIAR_ExGMAC);
            csi_tmp = rtl8101_eri_read(ioaddr, 0x3FA, 2, ERIAR_ExGMAC);
            csi_tmp |= BIT_14;
            rtl8101_eri_write(ioaddr, 0x3FA, 2, csi_tmp, ERIAR_ExGMAC);
            csi_tmp = rtl8101_eri_read(ioaddr, 0x3F2, 2, ERIAR_ExGMAC);
            csi_tmp &= ~(BIT_0 | BIT_1);
            csi_tmp |= BIT_0;
            pci_read_config_byte(pdev, 0x99, &data8);
            if (!(data8 & (BIT_5 | BIT_6)))
                csi_tmp &= ~(BIT_1);
            if (!(data8 & BIT_2))
                csi_tmp &= ~(BIT_0 );
            
            rtl8101_eri_write(ioaddr, 0x3F2, 2, csi_tmp, ERIAR_ExGMAC);
            
            pci_read_config_byte(pdev, 0x180, &data8);
            if (data8 & (BIT_0|BIT_1)) {
                csi_tmp = rtl8101_eri_read(ioaddr, 0x1E2, 1, ERIAR_ExGMAC);
                csi_tmp |= BIT_2;
                rtl8101_eri_write(ioaddr, 0x1E2, 1, csi_tmp, ERIAR_ExGMAC);
            } else {
                csi_tmp = rtl8101_eri_read(ioaddr, 0x1E2, 1, ERIAR_ExGMAC);
                csi_tmp &= ~BIT_2;
                rtl8101_eri_write(ioaddr, 0x1E2, 1, csi_tmp, ERIAR_ExGMAC);
            }
        }
    }
    
    if ((tp->mcfg == CFG_METHOD_1) ||
        (tp->mcfg == CFG_METHOD_2) ||
        (tp->mcfg == CFG_METHOD_3)) {
        /* csum offload command for RTL8101E */
        tp->tx_tcp_csum_cmd = TxIPCS | TxTCPCS;
        tp->tx_udp_csum_cmd = TxIPCS | TxUDPCS;
        tp->tx_ip_csum_cmd = TxIPCS;
    } else {
        /* csum offload command for RTL8102E */
        tp->tx_tcp_csum_cmd = TxIPCS_C | TxTCPCS_C;
        tp->tx_udp_csum_cmd = TxIPCS_C | TxUDPCS_C;
        tp->tx_ip_csum_cmd = TxIPCS_C;
    }
    
    //other hw parameretrs
    if (tp->mcfg == CFG_METHOD_17)
        rtl8101_eri_write(ioaddr, 0x2F8, 2, 0x1D8F, ERIAR_ExGMAC);
    
    if (tp->bios_setting & BIT_28) {
        if (tp->mcfg == CFG_METHOD_13) {
            if (RTL_R8(0xEF) & BIT_2) {
                u32 gphy_val;
                
                spin_lock_irqsave(&tp->phy_lock, flags);
                mdio_write(tp, 0x1F, 0x0001);
                gphy_val = mdio_read(tp, 0x1B);
                gphy_val |= BIT_2;
                mdio_write(tp, 0x1B, gphy_val);
                mdio_write(tp, 0x1F, 0x0000);
                spin_unlock_irqrestore(&tp->phy_lock, flags);
            }
        }
        
        if (tp->mcfg == CFG_METHOD_14) {
            u32 gphy_val;
            
            spin_lock_irqsave(&tp->phy_lock, flags);
            mdio_write(tp, 0x1F, 0x0001);
            gphy_val = mdio_read(tp, 0x13);
            gphy_val |= BIT_15;
            mdio_write(tp, 0x13, gphy_val);
            mdio_write(tp, 0x1F, 0x0000);
            spin_unlock_irqrestore(&tp->phy_lock, flags);
        }
    }
    
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,0,0)
    RTL_W16(CPlusCmd, tp->cp_cmd);
#else
    rtl8101_hw_set_features(dev, dev->features);
#endif
    
    switch (tp->mcfg) {
        case CFG_METHOD_17: {
            int timeout;
            for (timeout = 0; timeout < 10; timeout++) {
                if ((rtl8101_eri_read(ioaddr, 0x1AE, 2, ERIAR_ExGMAC) & BIT_13)==0)
                    break;
                mdelay(1);
            }
        }
            break;
    }
    
    RTL_W16(RxMaxSize, tp->rx_buf_sz);
    
    rtl8101_disable_rxdvgate(dev);
    
    if (!tp->pci_cfg_is_read) {
        pci_read_config_byte(pdev, PCI_COMMAND, &tp->pci_cfg_space.cmd);
        pci_read_config_byte(pdev, PCI_CACHE_LINE_SIZE, &tp->pci_cfg_space.cls);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_0, &tp->pci_cfg_space.io_base_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_0 + 2, &tp->pci_cfg_space.io_base_h);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2, &tp->pci_cfg_space.mem_base_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_2 + 2, &tp->pci_cfg_space.mem_base_h);
        pci_read_config_byte(pdev, PCI_INTERRUPT_LINE, &tp->pci_cfg_space.ilr);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4, &tp->pci_cfg_space.resv_0x20_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_4 + 2, &tp->pci_cfg_space.resv_0x20_h);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5, &tp->pci_cfg_space.resv_0x24_l);
        pci_read_config_word(pdev, PCI_BASE_ADDRESS_5 + 2, &tp->pci_cfg_space.resv_0x24_h);
        
        tp->pci_cfg_is_read = 1;
    }
    
    rtl8101_dsm(dev, DSM_MAC_INIT);
    
    options1 = RTL_R8(Config3);
    options2 = RTL_R8(Config5);
    if ((options1 & (LinkUp | MagicPacket)) || (options2 & (UWF | BWF | MWF)))
        tp->wol_enabled = WOL_ENABLED;
    else
        tp->wol_enabled = WOL_DISABLED;
    
    /* Set Rx packet filter */
    rtl8101_hw_set_rx_packet_filter(dev);
    
    switch (tp->mcfg) {
        case CFG_METHOD_10:
        case CFG_METHOD_11:
        case CFG_METHOD_12:
        case CFG_METHOD_13:
        case CFG_METHOD_14:
        case CFG_METHOD_15:
        case CFG_METHOD_16:
        case CFG_METHOD_17:
            if (aspm) {
                RTL_W8(Config5, RTL_R8(Config5) | BIT_0);
                RTL_W8(Config2, RTL_R8(Config2) | BIT_7);
            } else {
                RTL_W8(Config5, RTL_R8(Config5) & ~BIT_0);
                RTL_W8(Config2, RTL_R8(Config2) & ~BIT_7);
            }
            break;
    }
    
    RTL_W8(Cfg9346, Cfg9346_Lock);
    
    RTL_W8(ChipCmd, CmdTxEnb | CmdRxEnb);
    
    /* Enable all known interrupts by setting the interrupt mask. */
    RTL_W16(IntrMask, tp->intr_mask);
    netif_start_queue(dev);
    
    
    udelay(10);
}

static int
rtl8101_change_mtu(struct net_device *dev,
                   int new_mtu)
{
    int ret = 0;
    
    if (new_mtu < ETH_ZLEN || new_mtu > ETH_DATA_LEN)
        return -EINVAL;
    
    dev->mtu = new_mtu;
    
    return ret;
}

static inline void
rtl8101_make_unusable_by_asic(struct RxDesc *desc)
{
    desc->addr = 0x0badbadbadbadbadull;
    desc->opts1 &= ~cpu_to_le32(DescOwn | RsvdMask);
}

static void
rtl8101_free_rx_skb(struct rtl8101_private *tp,
                    struct sk_buff **sk_buff,
                    struct RxDesc *desc)
{
    struct pci_dev *pdev = tp->pci_dev;
    
    pci_unmap_single(pdev, le64_to_cpu(desc->addr), tp->rx_buf_sz,
                     PCI_DMA_FROMDEVICE);
    dev_kfree_skb(*sk_buff);
    *sk_buff = NULL;
    rtl8101_make_unusable_by_asic(desc);
}

static inline void
rtl8101_mark_to_asic(struct RxDesc *desc,
                     u32 rx_buf_sz)
{
    u32 eor = le32_to_cpu(desc->opts1) & RingEnd;
    
    desc->opts1 = cpu_to_le32(DescOwn | eor | rx_buf_sz);
}

static inline void
rtl8101_map_to_asic(struct RxDesc *desc,
                    dma_addr_t mapping,
                    u32 rx_buf_sz)
{
    desc->addr = cpu_to_le64(mapping);
    wmb();
    rtl8101_mark_to_asic(desc, rx_buf_sz);
}

static int
rtl8101_alloc_rx_skb(struct pci_dev *pdev,
                     struct sk_buff **sk_buff,
                     struct RxDesc *desc,
                     int rx_buf_sz)
{
    struct sk_buff *skb;
    dma_addr_t mapping;
    int ret = 0;
    
    skb = dev_alloc_skb(rx_buf_sz + RTK_RX_ALIGN);
    if (!skb)
        goto err_out;
    
    skb_reserve(skb, RTK_RX_ALIGN);
    *sk_buff = skb;
    
    mapping = pci_map_single(pdev, skb->data, rx_buf_sz,
                             PCI_DMA_FROMDEVICE);
    
    rtl8101_map_to_asic(desc, mapping, rx_buf_sz);
    
out:
    return ret;
    
err_out:
    ret = -ENOMEM;
    rtl8101_make_unusable_by_asic(desc);
    goto out;
}

static void
rtl8101_rx_clear(struct rtl8101_private *tp)
{
    int i;
    
    for (i = 0; i < NUM_RX_DESC; i++) {
        if (tp->Rx_skbuff[i]) {
            rtl8101_free_rx_skb(tp, tp->Rx_skbuff + i,
                                tp->RxDescArray + i);
        }
    }
}

static u32
rtl8101_rx_fill(struct rtl8101_private *tp,
                struct net_device *dev,
                u32 start, u32 end)
{
    u32 cur;
    
    for (cur = start; end - cur > 0; cur++) {
        int ret, i = cur % NUM_RX_DESC;
        
        if (tp->Rx_skbuff[i])
            continue;
        
        ret = rtl8101_alloc_rx_skb(tp->pci_dev, tp->Rx_skbuff + i,
                                   tp->RxDescArray + i, tp->rx_buf_sz);
        if (ret < 0)
            break;
    }
    return cur - start;
}

static inline void
rtl8101_mark_as_last_descriptor(struct RxDesc *desc)
{
    desc->opts1 |= cpu_to_le32(RingEnd);
}

static void
rtl8101_desc_addr_fill(struct rtl8101_private *tp)
{
    void __iomem *ioaddr = tp->mmio_addr;
    
    if (!tp->TxPhyAddr || !tp->RxPhyAddr)
        return;
    
    RTL_W32(TxDescStartAddrLow, ((u64) tp->TxPhyAddr & DMA_BIT_MASK(32)));
    RTL_W32(TxDescStartAddrHigh, ((u64) tp->TxPhyAddr >> 32));
    RTL_W32(RxDescAddrLow, ((u64) tp->RxPhyAddr & DMA_BIT_MASK(32)));
    RTL_W32(RxDescAddrHigh, ((u64) tp->RxPhyAddr >> 32));
}

static void
rtl8101_tx_desc_init(struct rtl8101_private *tp)
{
    int i = 0;
    
    memset(tp->TxDescArray, 0x0, NUM_TX_DESC * sizeof(struct TxDesc));
    
    for (i = 0; i < NUM_TX_DESC; i++) {
        if (i == (NUM_TX_DESC - 1))
            tp->TxDescArray[i].opts1 = cpu_to_le32(RingEnd);
    }
}

static void
rtl8101_rx_desc_init(struct rtl8101_private *tp)
{
    memset(tp->RxDescArray, 0x0, NUM_RX_DESC * sizeof(struct RxDesc));
}

static int
rtl8101_init_ring(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    
    rtl8101_init_ring_indexes(tp);
    
    memset(tp->tx_skb, 0x0, NUM_TX_DESC * sizeof(struct ring_info));
    memset(tp->Rx_skbuff, 0x0, NUM_RX_DESC * sizeof(struct sk_buff *));
    
    rtl8101_tx_desc_init(tp);
    rtl8101_rx_desc_init(tp);
    
    if (rtl8101_rx_fill(tp, dev, 0, NUM_RX_DESC) != NUM_RX_DESC)
        goto err_out;
    
    rtl8101_mark_as_last_descriptor(tp->RxDescArray + NUM_RX_DESC - 1);
    
    return 0;
    
err_out:
    rtl8101_rx_clear(tp);
    return -ENOMEM;
}

static void
rtl8101_unmap_tx_skb(struct pci_dev *pdev,
                     struct ring_info *tx_skb,
                     struct TxDesc *desc)
{
    unsigned int len = tx_skb->len;
    
    pci_unmap_single(pdev, le64_to_cpu(desc->addr), len, PCI_DMA_TODEVICE);
    desc->opts1 = 0x00;
    desc->opts2 = 0x00;
    desc->addr = 0x00;
    tx_skb->len = 0;
}

static void
rtl8101_tx_clear(struct rtl8101_private *tp)
{
    unsigned int i;
    struct net_device *dev = tp->dev;
    
    for (i = tp->dirty_tx; i < tp->dirty_tx + NUM_TX_DESC; i++) {
        unsigned int entry = i % NUM_TX_DESC;
        struct ring_info *tx_skb = tp->tx_skb + entry;
        unsigned int len = tx_skb->len;
        
        if (len) {
            struct sk_buff *skb = tx_skb->skb;
            
            rtl8101_unmap_tx_skb(tp->pci_dev, tx_skb,
                                 tp->TxDescArray + entry);
            if (skb) {
                dev_kfree_skb(skb);
                tx_skb->skb = NULL;
            }
            RTLDEV->stats.tx_dropped++;
        }
    }
    tp->cur_tx = tp->dirty_tx = 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8101_schedule_work(struct net_device *dev, void (*task)(void *))
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    struct rtl8101_private *tp = netdev_priv(dev);
    
    PREPARE_WORK(&tp->task, task, dev);
    schedule_delayed_work(&tp->task, 4);
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
}
#else
static void rtl8101_schedule_work(struct net_device *dev, work_func_t task)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    
    PREPARE_DELAYED_WORK(&tp->task, task);
    schedule_delayed_work(&tp->task, 4);
}
#endif

static void
rtl8101_wait_for_quiescence(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    
    synchronize_irq(dev->irq);
    
    /* Wait for any pending NAPI task to complete */
#ifdef CONFIG_R8101_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    RTL_NAPI_DISABLE(dev, &tp->napi);
#endif//LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#endif
    
    rtl8101_irq_mask_and_ack(ioaddr);
    
#ifdef CONFIG_R8101_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    RTL_NAPI_ENABLE(dev, &tp->napi);
#endif//LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
#endif
}

#if 0
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8101_reinit_task(void *_data)
#else
static void rtl8101_reinit_task(struct work_struct *work)
#endif
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
    struct net_device *dev = _data;
#else
    struct rtl8101_private *tp =
    container_of(work, struct rtl8101_private, task.work);
    struct net_device *dev = tp->dev;
#endif
    int ret;
    
    if (netif_running(dev)) {
        rtl8101_wait_for_quiescence(dev);
        rtl8101_close(dev);
    }
    
    ret = rtl8101_open(dev);
    if (unlikely(ret < 0)) {
        if (net_ratelimit()) {
            struct rtl8101_private *tp = netdev_priv(dev);
            
            if (netif_msg_drv(tp)) {
                printk(PFX KERN_ERR
                       "%s: reinit failure (status = %d)."
                       " Rescheduling.\n", dev->name, ret);
            }
        }
        rtl8101_schedule_work(dev, rtl8101_reinit_task);
    }
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void rtl8101_reset_task(void *_data)
{
    struct net_device *dev = _data;
    struct rtl8101_private *tp = netdev_priv(dev);
#else
static void rtl8101_reset_task(struct work_struct *work)
{
    struct rtl8101_private *tp =
    container_of(work, struct rtl8101_private, task.work);
    struct net_device *dev = tp->dev;
#endif
    unsigned long flags;
    
    if (!netif_running(dev))
        return;
    
    rtl8101_wait_for_quiescence(dev);
    
    rtl8101_rx_interrupt(dev, tp, tp->mmio_addr, ~(u32)0);
    
    spin_lock_irqsave(&tp->lock, flags);
    
    rtl8101_tx_clear(tp);
    
    if (tp->dirty_rx == tp->cur_rx) {
        rtl8101_init_ring_indexes(tp);
        rtl8101_hw_start(dev);
        rtl8101_set_speed(dev, tp->autoneg, tp->speed, tp->duplex);
        netif_wake_queue(dev);
        spin_unlock_irqrestore(&tp->lock, flags);
    } else {
        spin_unlock_irqrestore(&tp->lock, flags);
        if (net_ratelimit()) {
            struct rtl8101_private *tp = netdev_priv(dev);
            
            if (netif_msg_intr(tp)) {
                printk(PFX KERN_EMERG
                       "%s: Rx buffers shortage\n", dev->name);
            }
        }
        rtl8101_schedule_work(dev, rtl8101_reset_task);
    }
}

static void
rtl8101_tx_timeout(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    
    spin_lock_irqsave(&tp->lock, flags);
    rtl8101_hw_reset(dev);
    spin_unlock_irqrestore(&tp->lock, flags);
    
    /* Let's wait a bit while any (async) irq lands on */
    rtl8101_schedule_work(dev, rtl8101_reset_task);
}

static int
rtl8101_xmit_frags(struct rtl8101_private *tp,
                   struct sk_buff *skb,
                   u32 opts1,
                   u32 opts2)
{
    struct skb_shared_info *info = skb_shinfo(skb);
    unsigned int cur_frag, entry;
    struct TxDesc *txd = NULL;
    
    entry = tp->cur_tx;
    for (cur_frag = 0; cur_frag < info->nr_frags; cur_frag++) {
        skb_frag_t *frag = info->frags + cur_frag;
        dma_addr_t mapping;
        u32 status, len;
        void *addr;
        
        entry = (entry + 1) % NUM_TX_DESC;
        
        txd = tp->TxDescArray + entry;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)
        len = frag->size;
        addr = ((void *) page_address(frag->page)) + frag->page_offset;
#else
        len = skb_frag_size(frag);
        addr = skb_frag_address(frag);
#endif
        mapping = pci_map_single(tp->pci_dev, addr, len, PCI_DMA_TODEVICE);
        
        /* anti gcc 2.95.3 bugware (sic) */
        status = opts1 | len | (RingEnd * !((entry + 1) % NUM_TX_DESC));
        
        txd->addr = cpu_to_le64(mapping);
        
        tp->tx_skb[entry].len = len;
        
        txd->opts1 = cpu_to_le32(status);
        txd->opts2 = cpu_to_le32(opts2);
    }
    
    if (cur_frag) {
        tp->tx_skb[entry].skb = skb;
        wmb();
        txd->opts1 |= cpu_to_le32(LastFrag);
    }
    
    return cur_frag;
}

static inline u32
rtl8101_tx_csum(struct sk_buff *skb,
                struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
    const struct iphdr *ip = skb->nh.iph;
#else
    const struct iphdr *ip = ip_hdr(skb);
#endif
    
    if (skb->ip_summed == CHECKSUM_PARTIAL) {
        if (ip->protocol == IPPROTO_TCP)
            return tp->tx_tcp_csum_cmd;
        else if (ip->protocol == IPPROTO_UDP)
            return tp->tx_udp_csum_cmd;
        else if (ip->protocol == IPPROTO_IP)
            return tp->tx_ip_csum_cmd;
        
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
        WARN_ON(1);	/* we need a WARN() */
#endif
    }
    
    return 0;
}

static int
rtl8101_start_xmit(struct sk_buff *skb,
                   struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned int frags, entry = tp->cur_tx % NUM_TX_DESC;
    struct TxDesc *txd = tp->TxDescArray + entry;
    void __iomem *ioaddr = tp->mmio_addr;
    dma_addr_t mapping;
    u32 len;
    u32 opts1;
    u32 opts2;
    int ret = NETDEV_TX_OK;
    unsigned long flags, large_send;
    
    spin_lock_irqsave(&tp->lock, flags);
    
    if (unlikely(TX_BUFFS_AVAIL(tp) < skb_shinfo(skb)->nr_frags)) {
        if (netif_msg_drv(tp)) {
            printk(KERN_ERR
                   "%s: BUG! Tx Ring full when queue awake!\n",
                   dev->name);
        }
        goto err_stop;
    }
    
    if (unlikely(le32_to_cpu(txd->opts1) & DescOwn))
        goto err_stop;
    
    opts1 = DescOwn;
    opts2 = rtl8101_tx_vlan_tag(tp, skb);
    
    large_send = 0;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    if (dev->features & NETIF_F_TSO) {
        u32 mss = skb_is_gso(skb);
        
        /* TCP Segmentation Offload (or TCP Large Send) */
        if (mss) {
            if ((tp->mcfg == CFG_METHOD_1) ||
                (tp->mcfg == CFG_METHOD_2) ||
                (tp->mcfg == CFG_METHOD_3)) {
                opts1 |= LargeSend | ((mss & MSSMask) << 16);
            } else {
                opts1 |= LargeSend;
                opts2 |= (mss & MSSMask) << 18;
            }
            large_send = 1;
        }
    }
#endif //LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    
    if (large_send == 0) {
        if (dev->features & NETIF_F_IP_CSUM) {
            if ((tp->mcfg == CFG_METHOD_1) ||
                (tp->mcfg == CFG_METHOD_2) ||
                (tp->mcfg == CFG_METHOD_3))
                opts1 |= rtl8101_tx_csum(skb, dev);
            else
                opts2 |= rtl8101_tx_csum(skb, dev);
        }
    }
    
    frags = rtl8101_xmit_frags(tp, skb, opts1, opts2);
    if (frags) {
        len = skb_headlen(skb);
        opts1 |= FirstFrag;
    } else {
        len = skb->len;
        
        opts1 |= FirstFrag | LastFrag;
        
        tp->tx_skb[entry].skb = skb;
    }
    
    opts1 |= len | (RingEnd * !((entry + 1) % NUM_TX_DESC));
    mapping = pci_map_single(tp->pci_dev, skb->data, len, PCI_DMA_TODEVICE);
    tp->tx_skb[entry].len = len;
    txd->addr = cpu_to_le64(mapping);
    txd->opts2 = cpu_to_le32(opts2);
    txd->opts1 = cpu_to_le32(opts1&~DescOwn);
    wmb();
    txd->opts1 = cpu_to_le32(opts1);
    
    dev->trans_start = jiffies;
    
    tp->cur_tx += frags + 1;
    
    wmb();
    
    RTL_W8(TxPoll, NPQ);    /* set polling bit */
    
    if (TX_BUFFS_AVAIL(tp) < MAX_SKB_FRAGS) {
        netif_stop_queue(dev);
        rtl_rmb();
        if (TX_BUFFS_AVAIL(tp) >= MAX_SKB_FRAGS)
            netif_wake_queue(dev);
    }
    
    spin_unlock_irqrestore(&tp->lock, flags);
    
out:
    return ret;
    
err_stop:
    netif_stop_queue(dev);
    ret = NETDEV_TX_BUSY;
    RTLDEV->stats.tx_dropped++;
    
    spin_unlock_irqrestore(&tp->lock, flags);
    goto out;
}

static void
rtl8101_tx_interrupt(struct net_device *dev,
                     struct rtl8101_private *tp,
                     void __iomem *ioaddr)
{
    unsigned int dirty_tx, tx_left;
    
    assert(dev != NULL);
    assert(tp != NULL);
    assert(ioaddr != NULL);
    
    dirty_tx = tp->dirty_tx;
    rtl_rmb();
    tx_left = tp->cur_tx - dirty_tx;
    
    while (tx_left > 0) {
        unsigned int entry = dirty_tx % NUM_TX_DESC;
        struct ring_info *tx_skb = tp->tx_skb + entry;
        u32 len = tx_skb->len;
        u32 status;
        
        rmb();
        status = le32_to_cpu(tp->TxDescArray[entry].opts1);
        if (status & DescOwn)
            break;
        
        RTLDEV->stats.tx_bytes += len;
        RTLDEV->stats.tx_packets++;
        
        rtl8101_unmap_tx_skb(tp->pci_dev, tx_skb, tp->TxDescArray + entry);
        
        if (status & LastFrag) {
            dev_kfree_skb_irq(tx_skb->skb);
            tx_skb->skb = NULL;
        }
        dirty_tx++;
        tx_left--;
    }
    
    if (tp->dirty_tx != dirty_tx) {
        tp->dirty_tx = dirty_tx;
        rtl_wmb();
        if (netif_queue_stopped(dev) &&
            (TX_BUFFS_AVAIL(tp) >= MAX_SKB_FRAGS)) {
            netif_wake_queue(dev);
        }
        rtl_wmb();
        if (tp->cur_tx != dirty_tx)
            RTL_W8(TxPoll, NPQ);
    }
}

static inline int
rtl8101_fragmented_frame(u32 status)
{
    return (status & (FirstFrag | LastFrag)) != (FirstFrag | LastFrag);
}

static inline void
rtl8101_rx_csum(struct rtl8101_private *tp,
                struct sk_buff *skb,
                struct RxDesc *desc)
{
    u32 opts1 = le32_to_cpu(desc->opts1);
    u32 opts2 = le32_to_cpu(desc->opts2);
    u32 status = opts1 & RxProtoMask;
    
    if ((tp->mcfg == CFG_METHOD_1) ||
        (tp->mcfg == CFG_METHOD_2) ||
        (tp->mcfg == CFG_METHOD_3)) {
        /* rx csum offload for RTL8101E */
        if (((status == RxProtoTCP) && !(opts1 & RxTCPF)) ||
            ((status == RxProtoUDP) && !(opts1 & RxUDPF)) ||
            ((status == RxProtoIP) && !(opts1 & RxIPF)))
            skb->ip_summed = CHECKSUM_UNNECESSARY;
        else
            skb->ip_summed = CHECKSUM_NONE;
    } else {
        /* rx csum offload for RTL8102E */
        if (((status == RxTCPT) && !(opts1 & RxTCPF)) ||
            ((status == RxUDPT) && !(opts1 & RxUDPF)) ||
            ((status == 0) && (opts2 & RxV4F) && !(opts1 & RxIPF)))
            skb->ip_summed = CHECKSUM_UNNECESSARY;
        else
            skb->ip_summed = CHECKSUM_NONE;
    }
    
}

static inline int
rtl8101_try_rx_copy(struct sk_buff **sk_buff,
                    int pkt_size,
                    struct RxDesc *desc,
                    int rx_buf_sz)
{
    int ret = -1;
    
    if (pkt_size < rx_copybreak) {
        struct sk_buff *skb;
        
        skb = dev_alloc_skb(pkt_size + NET_IP_ALIGN);
        if (skb) {
            skb_reserve(skb, NET_IP_ALIGN);
            eth_copy_and_sum(skb, sk_buff[0]->data, pkt_size, 0);
            *sk_buff = skb;
            ret = 0;
        }
    }
    return ret;
}

static inline void
rtl8101_rx_skb(struct rtl8101_private *tp,
               struct sk_buff *skb)
{
#ifdef CONFIG_R8101_NAPI
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
    netif_receive_skb(skb);
#else
    napi_gro_receive(&tp->napi, skb);
#endif
#else
    netif_rx(skb);
#endif
}

static int
rtl8101_rx_interrupt(struct net_device *dev,
                     struct rtl8101_private *tp,
                     void __iomem *ioaddr, u32 budget)
{
    unsigned int cur_rx, rx_left;
    unsigned int delta, count = 0;
    u32 rx_quota = RTL_RX_QUOTA(dev, budget);
    
    assert(dev != NULL);
    assert(tp != NULL);
    assert(ioaddr != NULL);
    
    cur_rx = tp->cur_rx;
    rx_left = NUM_RX_DESC + tp->dirty_rx - cur_rx;
    rx_left = rtl8101_rx_quota(rx_left, (u32) rx_quota);
    
    if ((tp->RxDescArray == NULL) || (tp->Rx_skbuff == NULL))
        goto rx_out;
    
    for (; rx_left > 0; rx_left--, cur_rx++) {
        unsigned int entry = cur_rx % NUM_RX_DESC;
        struct RxDesc *desc = tp->RxDescArray + entry;
        u32 status;
        
        rmb();
        status = le32_to_cpu(desc->opts1);
        
        if (status & DescOwn)
            break;
        if (unlikely(status & RxRES)) {
            if (netif_msg_rx_err(tp)) {
                printk(KERN_INFO
                       "%s: Rx ERROR. status = %08x\n",
                       dev->name, status);
            }
            RTLDEV->stats.rx_errors++;
            
            if (status & (RxRWT | RxRUNT))
                RTLDEV->stats.rx_length_errors++;
            if (status & RxCRC)
                RTLDEV->stats.rx_crc_errors++;
            rtl8101_mark_to_asic(desc, tp->rx_buf_sz);
        } else {
            struct sk_buff *skb = tp->Rx_skbuff[entry];
            int pkt_size = (status & 0x00003FFF) - 4;
            
            /*
             * The driver does not support incoming fragmented
             * frames. They are seen as a symptom of over-mtu
             * sized frames.
             */
            if (unlikely(rtl8101_fragmented_frame(status))) {
                RTLDEV->stats.rx_dropped++;
                RTLDEV->stats.rx_length_errors++;
                rtl8101_mark_to_asic(desc, tp->rx_buf_sz);
                continue;
            }
            
            rtl8101_rx_csum(tp, skb, desc);
            
            pci_unmap_single(tp->pci_dev,
                             le64_to_cpu(desc->addr), tp->rx_buf_sz,
                             PCI_DMA_FROMDEVICE);
            
            if (rtl8101_try_rx_copy(&skb, pkt_size, desc,
                                    tp->rx_buf_sz)) {
                tp->Rx_skbuff[entry] = NULL;
            } else {
                dma_addr_t mapping;
                
                mapping = pci_map_single(tp->pci_dev, tp->Rx_skbuff[entry]->data, tp->rx_buf_sz,
                                         PCI_DMA_FROMDEVICE);
                rtl8101_map_to_asic(desc, mapping, tp->rx_buf_sz);
            }
            
            skb->dev = dev;
            skb_put(skb, pkt_size);
            skb->protocol = eth_type_trans(skb, dev);
            
            if (rtl8101_rx_vlan_skb(tp, desc, skb) < 0)
                rtl8101_rx_skb(tp, skb);
            
            dev->last_rx = jiffies;
            RTLDEV->stats.rx_bytes += pkt_size;
            RTLDEV->stats.rx_packets++;
        }
    }
    
    count = cur_rx - tp->cur_rx;
    tp->cur_rx = cur_rx;
    
    delta = rtl8101_rx_fill(tp, dev, tp->dirty_rx, tp->cur_rx);
    if (!delta && count && netif_msg_intr(tp))
        printk(KERN_INFO "%s: no Rx buffer allocated\n", dev->name);
    tp->dirty_rx += delta;
    
    /*
     * FIXME: until there is periodic timer to try and refill the ring,
     * a temporary shortage may definitely kill the Rx process.
     * - disable the asic to try and avoid an overflow and kick it again
     *   after refill ?
     * - how do others driver handle this condition (Uh oh...).
     */
    if ((tp->dirty_rx + NUM_RX_DESC == tp->cur_rx) && netif_msg_intr(tp))
        printk(KERN_EMERG "%s: Rx buffers exhausted\n", dev->name);
    
rx_out:
    
    return count;
}

static inline void
rtl8101_switch_to_timer_interrupt(struct rtl8101_private *tp, void __iomem *ioaddr)
{
    RTL_W32(TCTR, timer_count);
    RTL_W32(TimeIntr, timer_count);
    RTL_W16(IntrMask, PCSTimeout);
}

static inline void
rtl8101_switch_to_hw_interrupt(struct rtl8101_private *tp, void __iomem *ioaddr)
{
    RTL_W16(TimeIntr, 0x0000);
    RTL_W16(IntrMask, tp->intr_mask);
}

/* The interrupt handler does all of the Rx thread work and cleans up after the Tx thread. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
static irqreturn_t rtl8101_interrupt(int irq, void *dev_instance, struct pt_regs *regs)
#else
static irqreturn_t rtl8101_interrupt(int irq, void *dev_instance)
#endif
{
    struct net_device *dev = (struct net_device *) dev_instance;
    struct rtl8101_private *tp = netdev_priv(dev);
    void __iomem *ioaddr = tp->mmio_addr;
    int status;
    int handled = 0;
    
    do {
        status = RTL_R16(IntrStatus);
        
        /* hotplug/major error/no more work/shared irq */
        if ((status == 0xFFFF) || !status)
            break;
        
        if (!(status & (tp->intr_mask | PCSTimeout)))
            break;
        
        if (unlikely(!netif_running(dev))) {
            break;
        }
        
        handled = 1;
        
        RTL_W16(IntrMask, 0x0000);
        
        RTL_W16(IntrStatus, status);
        
#ifdef CONFIG_R8101_NAPI
        if (status & tp->intr_mask) {
            if (likely(RTL_NETIF_RX_SCHEDULE_PREP(dev, &tp->napi)))
                __RTL_NETIF_RX_SCHEDULE(dev, &tp->napi);
            else if (netif_msg_intr(tp))
                printk(KERN_INFO "%s: interrupt %04x in poll\n",
                       dev->name, status);
        } else
            rtl8101_switch_to_hw_interrupt(tp, ioaddr);
#else
        if (status & tp->intr_mask) {
            rtl8101_rx_interrupt(dev, tp, tp->mmio_addr, ~(u32)0);
            rtl8101_tx_interrupt(dev, tp, ioaddr);
            
            rtl8101_switch_to_timer_interrupt(tp, ioaddr);
        } else
            rtl8101_switch_to_hw_interrupt(tp, ioaddr);
#endif
    } while (false);
    
    return IRQ_RETVAL(handled);
}

#ifdef CONFIG_R8101_NAPI
static int rtl8101_poll(napi_ptr napi, napi_budget budget)
{
    struct rtl8101_private *tp = RTL_GET_PRIV(napi, struct rtl8101_private);
    void __iomem *ioaddr = tp->mmio_addr;
    RTL_GET_NETDEV(tp)
    unsigned int work_to_do = RTL_NAPI_QUOTA(budget, dev);
    unsigned int work_done;
    unsigned long flags;
    
    work_done = rtl8101_rx_interrupt(dev, tp, ioaddr, (u32) budget);
    spin_lock_irqsave(&tp->lock, flags);
    rtl8101_tx_interrupt(dev, tp, ioaddr);
    spin_unlock_irqrestore(&tp->lock, flags);
    
    RTL_NAPI_QUOTA_UPDATE(dev, work_done, budget);
    
    if (work_done < work_to_do) {
        RTL_NETIF_RX_COMPLETE(dev, napi);
        /*
         * 20040426: the barrier is not strictly required but the
         * behavior of the irq handler could be less predictable
         * without it. Btw, the lack of flush for the posted pci
         * write is safe - FR
         */
        smp_wmb();
        rtl8101_switch_to_timer_interrupt(tp, ioaddr);
    }
    
    return RTL_NAPI_RETURN_VALUE;
}
#endif//CONFIG_R8101_NAPI

static void
rtl8101_down(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    unsigned long flags;
    
    rtl8101_delete_link_timer(dev, &tp->link_timer);
    
    rtl8101_delete_esd_timer(dev, &tp->esd_timer);
    
#ifdef CONFIG_R8101_NAPI
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,23)
    napi_disable(&tp->napi);
#endif
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,23)) && (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0))
    netif_poll_disable(dev);
#endif
#endif
    
    netif_stop_queue(dev);
    
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
    /* Give a racing hard_start_xmit a few cycles to complete. */
    synchronize_sched();  /* FIXME: should this be synchronize_irq()? */
#endif
    
    netif_carrier_off(dev);
    
    spin_lock_irqsave(&tp->lock, flags);
    
    rtl8101_dsm(dev, DSM_IF_DOWN);
    
    rtl8101_hw_reset(dev);
    
    spin_unlock_irqrestore(&tp->lock, flags);
    
    synchronize_irq(dev->irq);
    
    spin_lock_irqsave(&tp->lock, flags);
    
    rtl8101_tx_clear(tp);
    
    rtl8101_rx_clear(tp);
    
    spin_unlock_irqrestore(&tp->lock, flags);
}

static int rtl8101_close(struct net_device *dev)
{
    struct rtl8101_private *tp = netdev_priv(dev);
    struct pci_dev *pdev = tp->pci_dev;
    void __iomem *ioaddr = tp->mmio_addr;
    u8 options;
    
    options = RTL_R8(Config1);
    if (((tp->mcfg == CFG_METHOD_4) || (tp->mcfg == CFG_METHOD_5)) &&
        !(options & PMEnable)) {
        RTL_W8(Config4, RTL_R8(Config4) | (1 << 0));
        RTL_W8(DBG_reg, RTL_R8(DBG_reg) | (1 << 3));
        RTL_W8(PMCH, RTL_R8(PMCH) & !(1 << 7));
        RTL_W8(CPlusCmd, RTL_R8(CPlusCmd) | (1 << 1));
    }
    
    if (tp->TxDescArray!=NULL && tp->RxDescArray!=NULL) {
        rtl8101_down(dev);
        
        rtl8101_hw_d3_para(dev);
        
        rtl8101_powerdown_pll(dev);
        
        free_irq(dev->irq, dev);
        
        pci_free_consistent(pdev, R8101_RX_RING_BYTES, tp->RxDescArray, tp->RxPhyAddr);
        
        pci_free_consistent(pdev, R8101_TX_RING_BYTES, tp->TxDescArray, tp->TxPhyAddr);
        
        tp->TxDescArray = NULL;
        tp->RxDescArray = NULL;
        
        pci_free_consistent(pdev, sizeof(*tp->tally_vaddr), tp->tally_vaddr, tp->tally_paddr);
        
        tp->tally_vaddr = NULL;
    }
    
    return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
static void rtl8101_shutdown(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8101_private *tp = netdev_priv(dev);
    
    rtl8101_set_bios_setting(dev);
    rtl8101_rar_set(tp, tp->org_mac_addr);
    
    if (s5wol == 0)
        tp->wol_enabled = WOL_DISABLED;
    
    /*
     if (s5wol) {
     void __iomem *ioaddr = tp->mmio_addr;
     u32 csi_tmp;
     
     RTL_W8(Cfg9346, Cfg9346_Unlock);
     switch (tp->mcfg) {
     case CFG_METHOD_14:
     case CFG_METHOD_17:
     csi_tmp = rtl8101_eri_read(ioaddr, 0xDE, 1, ERIAR_ExGMAC);
     csi_tmp |= BIT_0;
     rtl8101_eri_write(ioaddr, 0xDE, 1, csi_tmp, ERIAR_ExGMAC);
     break;
     default:
     RTL_W8(Config3, RTL_R8(Config3) | MagicPacket);
     break;
     }
     RTL_W8(Config5, RTL_R8(Config5) | LanWake);
     RTL_W8(Cfg9346, Cfg9346_Lock);
     
     tp->wol_enabled = WOL_ENABLED;
     }
     */
    
    rtl8101_close(dev);
    rtl8101_disable_msi(pdev, tp);
}
#endif

/**
 *  rtl8101_get_stats - Get rtl8101 read/write statistics
 *  @dev: The Ethernet Device to get statistics for
 *
 *  Get TX/RX statistics for rtl8101
 */
static struct net_device_stats *
rtl8101_get_stats(struct net_device *dev) {
    struct rtl8101_private *tp = netdev_priv(dev);
    //	void __iomem *ioaddr = tp->mmio_addr;
    unsigned long flags;
    
    if (netif_running(dev)) {
        spin_lock_irqsave(&tp->lock, flags);
        //		tp->stats.rx_missed_errors += RTL_R32(RxMissed);
        //		RTL_W32(RxMissed, 0);
        spin_unlock_irqrestore(&tp->lock, flags);
    }
    
    return &RTLDEV->stats;
}

#ifdef CONFIG_PM

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
static int
rtl8101_suspend(struct pci_dev *pdev,
                u32 state)
#else
static int
rtl8101_suspend(struct pci_dev *pdev,
                pm_message_t state)
#endif
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8101_private *tp = netdev_priv(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    u32 pci_pm_state = pci_choose_state(pdev, state);
#endif
    unsigned long flags;
    
    
    if (!netif_running(dev))
        goto out;
    
    rtl8101_delete_esd_timer(dev, &tp->esd_timer);
    
    rtl8101_delete_link_timer(dev, &tp->link_timer);
    
    netif_stop_queue(dev);
    
    netif_carrier_off(dev);
    
    rtl8101_dsm(dev, DSM_NIC_GOTO_D3);
    
    netif_device_detach(dev);
    
    spin_lock_irqsave(&tp->lock, flags);
    
    rtl8101_hw_reset(dev);
    
    rtl8101_hw_d3_para(dev);
    
    rtl8101_powerdown_pll(dev);
    
    spin_unlock_irqrestore(&tp->lock, flags);
    
out:
    
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    pci_save_state(pdev, &pci_pm_state);
#else
    pci_save_state(pdev);
#endif
    pci_enable_wake(pdev, pci_choose_state(pdev, state), tp->wol_enabled);
    pci_set_power_state(pdev, pci_choose_state(pdev, state));
    
    return 0;
}

static int
rtl8101_resume(struct pci_dev *pdev)
{
    struct net_device *dev = pci_get_drvdata(pdev);
    struct rtl8101_private *tp = netdev_priv(dev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    u32 pci_pm_state = PCI_D0;
#endif
    
    pci_set_power_state(pdev, PCI_D0);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
    pci_restore_state(pdev, &pci_pm_state);
#else
    pci_restore_state(pdev);
#endif
    pci_enable_wake(pdev, PCI_D0, 0);
    
    /* restore last modified mac address */
    rtl8101_rar_set(tp, dev->dev_addr);
    
    if (!netif_running(dev))
        goto out;
    
    rtl8101_exit_oob(dev);
    
    rtl8101_dsm(dev, DSM_NIC_RESUME_D3);
    
    rtl8101_hw_init(dev);
    
    rtl8101_powerup_pll(dev);
    
    rtl8101_hw_ephy_config(dev);
    
    rtl8101_hw_phy_config(dev);
    
    rtl8101_schedule_work(dev, rtl8101_reset_task);
    
    netif_device_attach(dev);
    
out:
    return 0;
}

#endif /* CONFIG_PM */

static struct pci_driver rtl8101_pci_driver = {
    .name		= MODULENAME,
    .id_table	= rtl8101_pci_tbl,
    .probe		= rtl8101_init_one,
    .remove		= __devexit_p(rtl8101_remove_one),
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,11)
    .shutdown	= rtl8101_shutdown,
#endif
#ifdef CONFIG_PM
    .suspend	= rtl8101_suspend,
    .resume		= rtl8101_resume,
#endif
};

static int __init
rtl8101_init_module(void)
{
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,0)
    return pci_register_driver(&rtl8101_pci_driver);
#else
    return pci_module_init(&rtl8101_pci_driver);
#endif
}

static void __exit
rtl8101_cleanup_module(void)
{
    pci_unregister_driver(&rtl8101_pci_driver);
}

module_init(rtl8101_init_module);
module_exit(rtl8101_cleanup_module);

#endif  /* DISABLED_CODE */

#pragma mark --- EEPROM routines ---

//-------------------------------------------------------------------
//rtl_eeprom_type():
//  tell the eeprom type
//return value:
//  0: the eeprom type is 93C46
//  1: the eeprom type is 93C56 or 93C66
//-------------------------------------------------------------------
void rtl_eeprom_type(struct rtl8101_private *tp)
{
    void __iomem *ioaddr=tp->mmio_addr;
    u16 magic = 0;
    
    if (tp->mcfg == CFG_METHOD_UNKNOWN)
        goto out_no_eeprom;
    
    if(RTL_R8(0xD2)&0x04) {
        //not support
        //tp->eeprom_type = EEPROM_TWSI;
        //tp->eeprom_len = 256;
        goto out_no_eeprom;
    } else if(RTL_R32(RxConfig) & RxCfg_9356SEL) {
        tp->eeprom_type = EEPROM_TYPE_93C56;
        tp->eeprom_len = 256;
    } else {
        tp->eeprom_type = EEPROM_TYPE_93C46;
        tp->eeprom_len = 128;
    }
    
    magic = rtl_eeprom_read_sc(tp, 0);
    
out_no_eeprom:
    if ((magic != 0x8129) && (magic != 0x8128)) {
        tp->eeprom_type = EEPROM_TYPE_NONE;
        tp->eeprom_len = 0;
    }
}

void rtl_eeprom_cleanup(void __iomem *ioaddr)
{
    u8 x;
    
    x = RTL_R8(Cfg9346);
    x &= ~(Cfg9346_EEDI | Cfg9346_EECS);
    
    RTL_W8(Cfg9346, x);
    
    rtl_raise_clock(&x, ioaddr);
    rtl_lower_clock(&x, ioaddr);
}

int rtl_eeprom_cmd_done(void __iomem *ioaddr)
{
    u8 x;
    int i;
    
    rtl_stand_by(ioaddr);
    
    for (i = 0; i < 50000; i++) {
        x = RTL_R8(Cfg9346);
        
        if (x & Cfg9346_EEDO) {
            udelay(RTL_CLOCK_RATE * 2 * 3);
            return 0;
        }
        udelay(1);
    }
    
    return -1;
}

//-------------------------------------------------------------------
//rtl_eeprom_read_sc():
//  read one word from eeprom
//-------------------------------------------------------------------
u16 rtl_eeprom_read_sc(struct rtl8101_private *tp, u16 reg)
{
    void __iomem *ioaddr=tp->mmio_addr;
    int addr_sz = 6;
    u8 x;
    u16 data;
    
    if(tp->eeprom_type == EEPROM_TYPE_NONE) {
        return -1;
    }
    
    if (tp->eeprom_type==EEPROM_TYPE_93C46)
        addr_sz = 6;
    else if (tp->eeprom_type==EEPROM_TYPE_93C56)
        addr_sz = 8;
    
    x = Cfg9346_EEM1 | Cfg9346_EECS;
    RTL_W8(Cfg9346, x);
    
    rtl_shift_out_bits(RTL_EEPROM_READ_OPCODE, 3, ioaddr);
    rtl_shift_out_bits(reg, addr_sz, ioaddr);
    
    data = rtl_shift_in_bits(ioaddr);
    
    rtl_eeprom_cleanup(ioaddr);
    
    RTL_W8(Cfg9346, 0);
    
    return data;
}

//-------------------------------------------------------------------
//rtl_eeprom_write_sc():
//  write one word to a specific address in the eeprom
//-------------------------------------------------------------------
void rtl_eeprom_write_sc(struct rtl8101_private *tp, u16 reg, u16 data)
{
    void __iomem *ioaddr=tp->mmio_addr;
    u8 x;
    int addr_sz = 6;
    int w_dummy_addr = 4;
    
    if(tp->eeprom_type == EEPROM_TYPE_NONE) {
        return ;
    }
    
    if (tp->eeprom_type==EEPROM_TYPE_93C46) {
        addr_sz = 6;
        w_dummy_addr = 4;
    } else if (tp->eeprom_type==EEPROM_TYPE_93C56) {
        addr_sz = 8;
        w_dummy_addr = 6;
    }
    
    x = Cfg9346_EEM1 | Cfg9346_EECS;
    RTL_W8(Cfg9346, x);
    
    rtl_shift_out_bits(RTL_EEPROM_EWEN_OPCODE, 5, ioaddr);
    rtl_shift_out_bits(reg, w_dummy_addr, ioaddr);
    rtl_stand_by(ioaddr);
    
    rtl_shift_out_bits(RTL_EEPROM_ERASE_OPCODE, 3, ioaddr);
    rtl_shift_out_bits(reg, addr_sz, ioaddr);
    if (rtl_eeprom_cmd_done(ioaddr) < 0) {
        return;
    }
    rtl_stand_by(ioaddr);
    
    rtl_shift_out_bits(RTL_EEPROM_WRITE_OPCODE, 3, ioaddr);
    rtl_shift_out_bits(reg, addr_sz, ioaddr);
    rtl_shift_out_bits(data, 16, ioaddr);
    if (rtl_eeprom_cmd_done(ioaddr) < 0) {
        return;
    }
    rtl_stand_by(ioaddr);
    
    rtl_shift_out_bits(RTL_EEPROM_EWDS_OPCODE, 5, ioaddr);
    rtl_shift_out_bits(reg, w_dummy_addr, ioaddr);
    
    rtl_eeprom_cleanup(ioaddr);
    RTL_W8(Cfg9346, 0);
}

void rtl_raise_clock(u8 *x, void __iomem *ioaddr)
{
    *x = *x | Cfg9346_EESK;
    RTL_W8(Cfg9346, *x);
    udelay(RTL_CLOCK_RATE);
}

void rtl_lower_clock(u8 *x, void __iomem *ioaddr)
{
    
    *x = *x & ~Cfg9346_EESK;
    RTL_W8(Cfg9346, *x);
    udelay(RTL_CLOCK_RATE);
}

void rtl_shift_out_bits(int data, int count, void __iomem *ioaddr)
{
    u8 x;
    int  mask;
    
    mask = 0x01 << (count - 1);
    x = RTL_R8(Cfg9346);
    x &= ~(Cfg9346_EEDI | Cfg9346_EEDO);
    
    do {
        if (data & mask)
            x |= Cfg9346_EEDI;
        else
            x &= ~Cfg9346_EEDI;
        
        RTL_W8(Cfg9346, x);
        udelay(RTL_CLOCK_RATE);
        rtl_raise_clock(&x, ioaddr);
        rtl_lower_clock(&x, ioaddr);
        mask = mask >> 1;
    } while(mask);
    
    x &= ~Cfg9346_EEDI;
    RTL_W8(Cfg9346, x);
}

u16 rtl_shift_in_bits(void __iomem *ioaddr)
{
    u8 x;
    u16 d, i;
    
    x = RTL_R8(Cfg9346);
    x &= ~(Cfg9346_EEDI | Cfg9346_EEDO);
    
    d = 0;
    
    for (i = 0; i < 16; i++) {
        d = d << 1;
        rtl_raise_clock(&x, ioaddr);
        
        x = RTL_R8(Cfg9346);
        x &= ~Cfg9346_EEDI;
        
        if (x & Cfg9346_EEDO)
            d |= 1;
        
        rtl_lower_clock(&x, ioaddr);
    }
    
    return d;
}

void rtl_stand_by(void __iomem *ioaddr)
{
    u8 x;
    
    x = RTL_R8(Cfg9346);
    x &= ~(Cfg9346_EECS | Cfg9346_EESK);
    RTL_W8(Cfg9346, x);
    udelay(RTL_CLOCK_RATE);
    
    x |= Cfg9346_EECS;
    RTL_W8(Cfg9346, x);
}

void rtl_set_eeprom_sel_low(void __iomem *ioaddr)
{
    RTL_W8(Cfg9346, Cfg9346_EEM1);
    RTL_W8(Cfg9346, Cfg9346_EEM1 | Cfg9346_EESK);
    
    udelay(20);
    
    RTL_W8(Cfg9346, Cfg9346_EEM1);
}

