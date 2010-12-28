/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/platform_device.h>

#define RTGALAXY_OHCI_HCD_NAME "rtgalaxy-ohci"

extern void rtgalaxy_usbphy_setup(void);

static void ohci_rtgalaxy_hw_start(void)
{
	rtgalaxy_usbphy_setup();
}

static void ohci_rtgalaxy_hw_stop(void)
{
}

static int __devinit ohci_rtgalaxy_start(struct usb_hcd *hcd)
{
    struct ohci_hcd *ohci = hcd_to_ohci(hcd);
    int ret;

    ret = ohci_init(ohci);

    if (ret < 0)
        return ret;

    ret = ohci_run(ohci);
    
    if (ret < 0) {
        ohci_err(ohci, "can't start %s", hcd->self.bus_name);
        ohci_stop(hcd);
        return ret;
    }
    return 0;
}

static const struct hc_driver ohci_hcd_rtgalaxy_driver = {
    .description		= hcd_name,
    .product_desc		= "Realtek Galaxy OHCI",
    .hcd_priv_size		= sizeof(struct ohci_hcd),

    /*
     * generic hardware linkage
     */
    .irq =			ohci_irq,
    .flags =		HCD_USB11 | HCD_MEMORY,

    /*
     * basic lifecycle operations
     */
    .start =		ohci_rtgalaxy_start,
    .stop =			ohci_stop,
    .shutdown =		ohci_shutdown,
    
    /*
     * managing i/o requests and associated device resources
     */
    .urb_enqueue =		ohci_urb_enqueue,
    .urb_dequeue =		ohci_urb_dequeue,
    .endpoint_disable =	ohci_endpoint_disable,
    
    /*
     * scheduling support
     */
    .get_frame_number =	ohci_get_frame,
    
    /*
     * root hub support
     */
    .hub_status_data =	ohci_hub_status_data,
    .hub_control =		ohci_hub_control,
    
    .start_port_reset =	ohci_start_port_reset,
};

static int ohci_rtgalaxy_drv_probe(struct platform_device *pdev)
{
    struct usb_hcd *hcd;
    struct ohci_hcd *ohci;
    struct resource *res_mem;
    int irq;
    int ret;

    if (usb_disabled())
        return -ENODEV;

    irq = platform_get_irq(pdev, 0);
    if (irq < 0) {
        dev_err(&pdev->dev, "No irq assigned\n");
        return -ENODEV;
    }

    res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res_mem == NULL) {
        dev_err(&pdev->dev, "No register space assigned\n");
        return -ENODEV;
    }

    hcd = usb_create_hcd(&ohci_hcd_rtgalaxy_driver, &pdev->dev, "rtgalaxy");
    if (!hcd)
        return -ENOMEM;

    hcd->rsrc_start = res_mem->start;
    hcd->rsrc_len = res_mem->end - res_mem->start + 1;

    if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,	RTGALAXY_OHCI_HCD_NAME)) {
        dev_err(&pdev->dev, "request_mem_region failed\n");
        ret = -EBUSY;
        goto err1;
    }

    hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
    if (!hcd->regs) {
        dev_err(&pdev->dev, "ioremap failed\n");
        ret = -ENOMEM;
        goto err2;
    }

    ohci_rtgalaxy_hw_start();

    ohci = hcd_to_ohci(hcd);
    ohci_hcd_init(ohci);

    ret = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
    if (ret) {
        dev_dbg(&pdev->dev, "failed to add hcd with err %d\n", ret);
        goto err3;
    }

    platform_set_drvdata(pdev, hcd);

    return 0;

err3:
    ohci_rtgalaxy_hw_stop();

    iounmap(hcd->regs);
err2:
    release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
    usb_put_hcd(hcd);
    return ret;
}

static int ohci_rtgalaxy_drv_remove(struct platform_device *pdev)
{
    struct usb_hcd *hcd = platform_get_drvdata(pdev);

    usb_remove_hcd(hcd);
    
    ohci_rtgalaxy_hw_stop();
    iounmap(hcd->regs);
    release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
    usb_put_hcd(hcd);

    platform_set_drvdata(pdev, NULL);

    return 0;
}

static struct platform_driver ohci_rtgalaxy_driver = {
    .probe		= ohci_rtgalaxy_drv_probe,
    .remove		= ohci_rtgalaxy_drv_remove,
    .shutdown	= usb_hcd_platform_shutdown,
    .driver = {
        .name	= RTGALAXY_OHCI_HCD_NAME,
        .owner	= THIS_MODULE,
    }
};

MODULE_ALIAS("platform:" RTGALAXY_OHCI_HCD_NAME);
