/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 */

#include <linux/platform_device.h>

#define RTGALAXY_EHCI_HCD_NAME "rtgalaxy-ehci"

extern void rtgalaxy_usbphy_setup(void);

static void ehci_rtgalaxy_hw_start(void)
{
	rtgalaxy_usbphy_setup();
}

static void ehci_rtgalaxy_hw_stop(void)
{
}

static const struct hc_driver ehci_hcd_rtgalaxy_driver = {
    .description		= hcd_name,
    .product_desc		= "Realtek Galaxy EHCI",
    .hcd_priv_size		= sizeof(struct ehci_hcd),

    /*
     * generic hardware linkage
     */
    .irq =			ehci_irq,
    .flags =		HCD_USB2 | HCD_MEMORY,

    /*
     * basic lifecycle operations
     */
    .reset =    ehci_init,
    .start =		ehci_run,
    .stop =			ehci_stop,
    .shutdown =		ehci_shutdown,
    
    /*
     * managing i/o requests and associated device resources
     */
    .urb_enqueue =		ehci_urb_enqueue,
    .urb_dequeue =		ehci_urb_dequeue,
    .endpoint_disable =	ehci_endpoint_disable,
    .endpoint_reset =	  ehci_endpoint_reset,
    
    /*
     * scheduling support
     */
    .get_frame_number =	ehci_get_frame,
    
    /*
     * root hub support
     */
    .hub_status_data =	ehci_hub_status_data,
    .hub_control =		ehci_hub_control,
    .bus_suspend =	ehci_bus_suspend,
    .bus_resume =		ehci_bus_resume,
    .relinquish_port =		ehci_relinquish_port,
    .port_handed_over =		ehci_port_handed_over,
    
    .clear_tt_buffer_complete =	ehci_clear_tt_buffer_complete,
};

static int ehci_rtgalaxy_drv_probe(struct platform_device *pdev)
{
    struct usb_hcd *hcd;
    struct ehci_hcd *ehci;
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

    hcd = usb_create_hcd(&ehci_hcd_rtgalaxy_driver, &pdev->dev, "rtgalaxy");
    if (!hcd)
        return -ENOMEM;

    hcd->rsrc_start = res_mem->start;
    hcd->rsrc_len = res_mem->end - res_mem->start + 1;

    if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,	RTGALAXY_EHCI_HCD_NAME)) {
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

    ehci_rtgalaxy_hw_start();

    ehci = hcd_to_ehci(hcd);
    ehci->caps = hcd->regs;
    ehci->regs = hcd->regs +
        HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));

    ret = usb_add_hcd(hcd, irq, IRQF_DISABLED | IRQF_SHARED);
    if (ret) {
        dev_dbg(&pdev->dev, "failed to add hcd with err %d\n", ret);
        goto err3;
    }

    /* root ports should always stay powered */
    ehci_port_power(ehci, 1);

    platform_set_drvdata(pdev, hcd);

    return 0;

err3:
    ehci_rtgalaxy_hw_stop();

    iounmap(hcd->regs);
err2:
    release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
err1:
    usb_put_hcd(hcd);
    return ret;
}

static int ehci_rtgalaxy_drv_remove(struct platform_device *pdev)
{
    struct usb_hcd *hcd = platform_get_drvdata(pdev);

    usb_remove_hcd(hcd);
    
    ehci_rtgalaxy_hw_stop();
    iounmap(hcd->regs);
    release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
    usb_put_hcd(hcd);

    platform_set_drvdata(pdev, NULL);

    return 0;
}

static struct platform_driver ehci_rtgalaxy_driver = {
    .probe		= ehci_rtgalaxy_drv_probe,
    .remove		= ehci_rtgalaxy_drv_remove,
    .shutdown	= usb_hcd_platform_shutdown,
    .driver = {
        .name	= RTGALAXY_EHCI_HCD_NAME,
        .owner	= THIS_MODULE,
    }
};

MODULE_ALIAS("platform:" RTGALAXY_EHCI_HCD_NAME);
