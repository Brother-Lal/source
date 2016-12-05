/*
 *  TP-LINK WR1043 V4 support
 *
 *  Copyright (C) 2015-2016 P. Wassi <p.wassi at gmx.at>
 *  Copyright (C) 2016 Matthias Schiffer <mschiffer@universe-factory.net>
 *  Copyright (C) 2016 Andreas Ziegler <github@andreas-ziegler.de>
 *  Copyright (C) 2016 Ludwig Thomeczek <ledesrc@wxorx.net>
 *
 *  Derived from: mach-dir-869-a1.c
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */


#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/ath9k_platform.h>

#include <asm/mach-ath79/ath79.h>
#include <asm/mach-ath79/irq.h>
#include <asm/mach-ath79/ar71xx_regs.h>

#include <linux/platform_data/phy-at803x.h>
#include <linux/ar8216_platform.h>

#include "common.h"
#include "dev-ap9x-pci.h"
#include "dev-eth.h"
#include "dev-gpio-buttons.h"
#include "dev-leds-gpio.h"
#include "dev-m25p80.h"
#include "dev-wmac.h"
#include "dev-usb.h"
#include "machtypes.h"
#include "nvram.h"

#define TL_WR1043_V4_GPIO_BTN_RESET		2
#define TL_WR1043_V4_GPIO_BTN_RFKILL	5

// 1:1 from dir869 TODO
#define TL_WR1043_V4_GPIO_SWITCH_MODE	8
#define TL_WR1043_V4_GPIO_ENABLE_SWITCH	11

#define TL_WR1043_V4_GPIO_LED_WLAN		19
#define TL_WR1043_V4_GPIO_LED_USB			7
#define TL_WR1043_V4_GPIO_LED_WPS			1
#define TL_WR1043_V4_GPIO_LED_SYSTEM	6

#define TL_WR1043_V4_GPIO_USB_POWER		8

#define TL_WR1043_V4_KEYS_POLL_INTERVAL	20 /* msecs */
#define TL_WR1043_V4_KEYS_DEBOUNCE_INTERVAL	(3 * TL_WR1043_V4_KEYS_POLL_INTERVAL)

// TODO
#define TL_WR1043_V4_DEVDATA_ADDR		0x1f050000
#define TL_WR1043_V4_DEVDATA_SIZE		0x10000

#define TL_WR1043_V4_EEPROM_ADDR		0x1fff0000
#define TL_WR1043_V4_WMAC_CALDATA_OFFSET	0x1000
#define TL_WR1043_V4_PCI_CALDATA_OFFSET	0x5000

// taken from 1043v2 start
static const char *wr1043nd_v4_part_probes[] = {
	"tp-link",
	NULL,
};

static struct flash_platform_data wr1043nd_v4_flash_data = {
	.part_probes	= wr1043nd_v4_part_probes,
};

static struct gpio_led tl_wr1043nd_v4_leds_gpio[] __initdata = {
	{
		.name		= "tp-link:green:wps",
		.gpio		= TL_WR1043_V4_GPIO_LED_WPS,
		.active_low	= 1,
	},
	{
		.name		= "tp-link:green:system",
		.gpio		= TL_WR1043_V4_GPIO_LED_SYSTEM,
		.active_low	= 1,
	},
	{
		.name		= "tp-link:green:wlan",
		.gpio		= TL_WR1043_V4_GPIO_LED_WLAN,
		.active_low	= 1,
	},
	{
		.name		= "tp-link:green:usb",
		.gpio		= TL_WR1043_V4_GPIO_LED_USB,
		.active_low	= 1,
	},
};

static struct gpio_keys_button tl_wr1043nd_v4_gpio_keys[] __initdata = {
	{
		.desc		= "Reset button",
		.type		= EV_KEY,
		.code		= KEY_RESTART,
		.debounce_interval = TL_WR1043_V4_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= TL_WR1043_V4_GPIO_BTN_RESET,
		.active_low	= 1,
	},
	{
		.desc		= "RFKILL button",
		.type		= EV_KEY,
		.code		= KEY_RFKILL,
		.debounce_interval = TL_WR1043_V4_KEYS_DEBOUNCE_INTERVAL,
		.gpio		= TL_WR1043_V4_GPIO_BTN_RFKILL,
		.active_low	= 1,
	},
};

static const struct ar8327_led_info tl_wr1043nd_leds_ar8327[] = {
	AR8327_LED_INFO(PHY0_0, HW, "tp-link:green:lan4"),
	AR8327_LED_INFO(PHY1_0, HW, "tp-link:green:lan3"),
	AR8327_LED_INFO(PHY2_0, HW, "tp-link:green:lan2"),
	AR8327_LED_INFO(PHY3_0, HW, "tp-link:green:lan1"),
	AR8327_LED_INFO(PHY4_0, HW, "tp-link:green:wan"),
};

// taken from 1043v2 /end

static struct ar8327_pad_cfg tl_wr1043nd_v4_ar8327_pad0_cfg = {
	.mode = AR8327_PAD_MAC_SGMII,
	.sgmii_delay_en = true,
};

static struct ar8327_platform_data tl_wr1043nd_v4_ar8327_data = {
	.pad0_cfg = &tl_wr1043nd_v4_ar8327_pad0_cfg,
	.port0_cfg = {
		.force_link = 1,
		.speed = AR8327_PORT_SPEED_1000,
		.duplex = 1,
		.txpause = 1,
		.rxpause = 1,
	},
};

static struct mdio_board_info tl_wr1043nd_v4_mdio0_info[] = {
	{
		.bus_id = "ag71xx-mdio.0",
		.phy_addr = 0,
		.platform_data = &tl_wr1043nd_v4_ar8327_data,
	},
};

static void tl_wr1043nd_v4_get_mac(const char *name, char *mac)
{
	u8 *nvram = (u8 *) KSEG1ADDR(TL_WR1043_V4_DEVDATA_ADDR);
	int err;

	err = ath79_nvram_parse_mac_addr(nvram, TL_WR1043_V4_DEVDATA_SIZE,
					 name, mac);
	if (err)
		pr_err("no MAC address found for %s\n", name);
}

static void __init tl_wr1043nd_v4_setup(void)
{
	u8 *eeprom = (u8 *) KSEG1ADDR(TL_WR1043_V4_EEPROM_ADDR);
	u8 wlan24mac[ETH_ALEN] = {};

 //this copied from 1043v2
	ath79_register_m25p80(&wr1043nd_v4_flash_data);

	gpio_request_one(TL_WR1043_V4_GPIO_ENABLE_SWITCH,
			 GPIOF_OUT_INIT_HIGH | GPIOF_EXPORT_DIR_FIXED,
			 "Switch power");

	tl_wr1043nd_v4_get_mac("lanmac=", ath79_eth0_data.mac_addr);
	ath79_eth0_data.phy_if_mode = PHY_INTERFACE_MODE_SGMII;
	ath79_eth0_data.mii_bus_dev = &ath79_mdio0_device.dev;
	ath79_eth0_data.phy_mask = BIT(0);

	mdiobus_register_board_info(tl_wr1043nd_v4_mdio0_info,
	                            ARRAY_SIZE(tl_wr1043nd_v4_mdio0_info));

	ath79_register_usb();
	ath79_register_mdio(0, 0);
	ath79_register_eth(0);

	tl_wr1043nd_v4_get_mac("wlan24mac=", wlan24mac);
	ath79_register_wmac(eeprom + TL_WR1043_V4_WMAC_CALDATA_OFFSET, wlan24mac);

	ath79_register_leds_gpio(-1, ARRAY_SIZE(tl_wr1043nd_v4_leds_gpio),
	                         tl_wr1043nd_v4_leds_gpio);

	ath79_register_gpio_keys_polled(-1, TL_WR1043_V4_KEYS_POLL_INTERVAL,
	                                ARRAY_SIZE(tl_wr1043nd_v4_gpio_keys),
	                                tl_wr1043nd_v4_gpio_keys);
	
	gpio_request_one(TL_WR1043_V4_GPIO_USB_POWER,
			 GPIOF_OUT_INIT_HIGH | GPIOF_EXPORT_DIR_FIXED,
			 "USB power");
}

MIPS_MACHINE(ATH79_MACH_TL_WR1043ND_V4, "TL-WR1043ND-v4",
	     "TP-LINK TL-WR1043ND v4", tl_wr1043nd_v4_setup);
