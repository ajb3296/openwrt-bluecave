// SPDX-License-Identifier: GPL-2.0-only
/*
 * Intel/Lantiq xRX500 (GRX350/GRX500) Serial Shift Output (SSO) LED driver.
 *
 * Drives the external shift-register LED cascade (up to 32 outputs) via the
 * xRX500 SSO register block at 0x16d00000.
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define XRX500_SSO_CON0		0x2b0
#define XRX500_SSO_CON1		0x2b4
#define XRX500_SSO_CPU0		0x2b8
#define XRX500_SSO_CPU1		0x2bc
#define XRX500_SSO_AR		0x2c0

#define XRX500_SSO_CON_SWU	BIT(31)
#define XRX500_SSO_GROUP_MASK	0x0f

struct xrx500_sso {
	void __iomem *base;
	spinlock_t lock;
	u32 shadow;
};

struct xrx500_sso_led {
	struct led_classdev cdev;
	struct xrx500_sso *sso;
	u32 pin;
	bool active_low;
};

static void xrx500_sso_update(struct xrx500_sso *sso)
{
	u32 con0;

	writel(sso->shadow, sso->base + XRX500_SSO_CPU0);
	con0 = readl(sso->base + XRX500_SSO_CON0);
	writel(con0 | XRX500_SSO_CON_SWU, sso->base + XRX500_SSO_CON0);
}

static void xrx500_sso_led_set(struct led_classdev *cdev,
			       enum led_brightness val)
{
	struct xrx500_sso_led *led =
		container_of(cdev, struct xrx500_sso_led, cdev);
	struct xrx500_sso *sso = led->sso;
	unsigned long flags;
	bool on = (val != LED_OFF);

	if (led->active_low)
		on = !on;

	spin_lock_irqsave(&sso->lock, flags);
	if (on)
		sso->shadow |= BIT(led->pin);
	else
		sso->shadow &= ~BIT(led->pin);
	xrx500_sso_update(sso);
	spin_unlock_irqrestore(&sso->lock, flags);
}

static int xrx500_sso_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *child;
	struct xrx500_sso *sso;
	u32 groups = 0x3;
	u32 con1;
	int count = 0;

	sso = devm_kzalloc(dev, sizeof(*sso), GFP_KERNEL);
	if (!sso)
		return -ENOMEM;

	sso->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(sso->base))
		return PTR_ERR(sso->base);

	spin_lock_init(&sso->lock);
	of_property_read_u32(np, "lantiq,groups", &groups);

	/* Keep bootloader shift clock/pinctrl; enable requested LED groups */
	writel(0, sso->base + XRX500_SSO_AR);
	con1 = readl(sso->base + XRX500_SSO_CON1);
	con1 = (con1 & ~XRX500_SSO_GROUP_MASK) | (groups & XRX500_SSO_GROUP_MASK);
	writel(con1, sso->base + XRX500_SSO_CON1);
	sso->shadow = readl(sso->base + XRX500_SSO_CPU0);

	for_each_available_child_of_node(np, child) {
		struct xrx500_sso_led *led;
		struct led_init_data init_data = { .fwnode = of_fwnode_handle(child) };
		const char *state = NULL;
		u32 pin;
		int ret;

		if (of_property_read_u32(child, "reg", &pin) || pin >= 32)
			continue;

		led = devm_kzalloc(dev, sizeof(*led), GFP_KERNEL);
		if (!led) {
			of_node_put(child);
			return -ENOMEM;
		}

		led->sso = sso;
		led->pin = pin;
		led->active_low = of_property_read_bool(child, "active-low");
		led->cdev.max_brightness = LED_ON;
		led->cdev.brightness_set = xrx500_sso_led_set;

		if (!of_property_read_string(child, "default-state", &state)) {
			if (!strcmp(state, "on"))
				xrx500_sso_led_set(&led->cdev, LED_ON);
			else if (!strcmp(state, "off"))
				xrx500_sso_led_set(&led->cdev, LED_OFF);
		}

		ret = devm_led_classdev_register_ext(dev, &led->cdev, &init_data);
		if (ret) {
			dev_warn(dev, "failed to register SSO LED pin %u: %d\n",
				 pin, ret);
			continue;
		}
		count++;
	}

	dev_info(dev, "xRX500 SSO LED controller ready (%d LEDs, groups=0x%x)\n",
		 count, groups);
	return 0;
}

static const struct of_device_id xrx500_sso_of_match[] = {
	{ .compatible = "lantiq,gpio-sso-xrx500" },
	{ .compatible = "lantiq,xrx500-sso-leds" },
	{ }
};
MODULE_DEVICE_TABLE(of, xrx500_sso_of_match);

static struct platform_driver xrx500_sso_driver = {
	.probe = xrx500_sso_probe,
	.driver = {
		.name = "leds-xrx500-sso",
		.of_match_table = xrx500_sso_of_match,
	},
};
module_platform_driver(xrx500_sso_driver);

MODULE_DESCRIPTION("Intel/Lantiq xRX500 Serial Shift Output LED driver");
MODULE_LICENSE("GPL");
