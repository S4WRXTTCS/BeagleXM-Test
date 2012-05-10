/*
 * linux/arch/arm/mach-omap2/board-amg-xlcore-camera.c
 *
 * Copyright (C) 2011 Advanced Microscopy Group
 *
 * Modified from mach-omap2/board-omap3evm.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include <linux/i2c/twl.h>
#include <linux/input/matrix_keypad.h>
#include <linux/mmc/host.h>
#include <linux/regulator/machine.h>
#include <linux/usb/otg.h>

#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/usb.h>
#include <plat/common.h>

#include "common-board-devices.h"
#include "devices.h"
#include "hsmmc.h"
#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"

#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
#include <linux/smsc911x.h>
#include <plat/gpmc-smsc911x.h>

#define AMGXLCORE_SMSC911X_GPIO_RESET	137
#define AMGXLCORE_SMSC911X_GPIO_IRQ	29
#define AMGXLCORE_SMSC911X_CS		5

static struct omap_smsc911x_platform_data amgxlcore_smsc911x_config = {
	.cs		= AMGXLCORE_SMSC911X_CS,
	.gpio_irq	= AMGXLCORE_SMSC911X_GPIO_IRQ,
	.gpio_reset	= AMGXLCORE_SMSC911X_GPIO_RESET,
	.flags		= SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS,
};

static inline void __init amgxlcore_smsc911x_init(void)
{
	gpmc_smsc911x_init(&amgxlcore_smsc911x_config);
}

#else
static inline void __init amgxlcore_smsc911x_init(void) { return; }
#endif

/* -----------------------------------------------------------------------------
 * MT9T001 Camera
 */

#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
#include <media/mt9t001.h>

#include "../../../drivers/media/video/omap3isp/isp.h"
#include "../../../drivers/media/video/omap3isp/ispreg.h"

static struct mt9t001_platform_data amgxlcore_mt9t001_platform_data = {
	.clk_pol = 1,
};

#define MT9T001_I2C_ADDR		0x5d
#define MT9T001_I2C_BUS_NUM		3

static struct i2c_board_info amgxlcore_camera_i2c_device = {
	I2C_BOARD_INFO("mt9t001", MT9T001_I2C_ADDR),
	.platform_data = &amgxlcore_mt9t001_platform_data,
};

static struct isp_subdev_i2c_board_info amgxlcore_camera_primary_subdevs[] = {
	{
		.board_info = &amgxlcore_camera_i2c_device,
		.i2c_adapter_id = MT9T001_I2C_BUS_NUM,
	},
	{ NULL, 0, },
};

static struct isp_v4l2_subdevs_group amgxlcore_camera_subdevs[] = {
	{
		.subdevs = amgxlcore_camera_primary_subdevs,
		.interface = ISP_INTERFACE_PARALLEL,
		.bus = { .parallel = {
			.data_lane_shift	= 1,
			.clk_pol		= 0,
			.bridge			= ISPCTRL_PAR_BRIDGE_DISABLE,
		} },
	},
	{ NULL, 0, },
};

static struct isp_platform_data amgxlcore_isp_platform_data = {
	.subdevs = amgxlcore_camera_subdevs,
};

static void __init amgxlcore_camera_init(void)
{
	omap3_init_camera(&amgxlcore_isp_platform_data);
}
#else
static inline void __init amgxlcore_camera_init(void) { }
#endif

/* -----------------------------------------------------------------------------
 * Display
 */

#define AMG_XLCORE_LCD_PANEL_ENVDD	131
#define AMG_XLCORE_DVI_PANEL_EN_GPIO	133

static struct gpio amgxlcore_dss_gpios[] __initdata = {
	{ AMG_XLCORE_LCD_PANEL_ENVDD,   GPIOF_OUT_INIT_LOW, "lcd_panel_envdd" },
	{ AMG_XLCORE_DVI_PANEL_EN_GPIO,	GPIOF_OUT_INIT_LOW, "dvi_panel_en"    },
};

static int lcd_enabled;
static int dvi_enabled;

static void __init amgxlcore_display_init(void)
{
	int r;

	r = gpio_request_array(amgxlcore_dss_gpios,
			       ARRAY_SIZE(amgxlcore_dss_gpios));
	if (r)
		printk(KERN_ERR "failed to get display gpios\n");
}

static int amgxlcore_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}

	gpio_set_value(AMG_XLCORE_LCD_PANEL_ENVDD, 1);

	lcd_enabled = 1;
	return 0;
}

static void amgxlcore_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(AMG_XLCORE_LCD_PANEL_ENVDD, 0);

	lcd_enabled = 0;
}

static struct panel_generic_dpi_data amgxlcore_lcd_panel = {
	.name			= "auo_g121xn01",
	.platform_enable	= amgxlcore_enable_lcd,
	.platform_disable	= amgxlcore_disable_lcd,
};

static struct omap_dss_device amgxlcore_lcd_device = {
	.name			= "lcd",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.driver_name		= "generic_dpi_panel",
	.phy.dpi.data_lines	= 24,
	.data			= &amgxlcore_lcd_panel,
};

static int amgxlcore_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	gpio_set_value_cansleep(AMG_XLCORE_DVI_PANEL_EN_GPIO, 1);

	dvi_enabled = 1;
	return 0;
}

static void amgxlcore_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value_cansleep(AMG_XLCORE_DVI_PANEL_EN_GPIO, 0);

	dvi_enabled = 0;
}

static struct panel_generic_dpi_data amgxlcore_dvi_panel = {
	.name			= "generic",
	.platform_enable	= amgxlcore_enable_dvi,
	.platform_disable	= amgxlcore_disable_dvi,
};

static struct omap_dss_device amgxlcore_dvi_device = {
	.name			= "dvi",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.driver_name		= "generic_dpi_panel",
	.data			= &amgxlcore_dvi_panel,
	.phy.dpi.data_lines	= 24,
};

static struct omap_dss_device *amgxlcore_dss_devices[] = {
	&amgxlcore_lcd_device,
	&amgxlcore_dvi_device,
};

static struct omap_dss_board_info amgxlcore_dss_data = {
	.num_devices	= ARRAY_SIZE(amgxlcore_dss_devices),
	.devices	= amgxlcore_dss_devices,
	.default_device	= &amgxlcore_lcd_device,
};

/* -----------------------------------------------------------------------------
 * TWL4030 & I2C
 */

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= 27,
	},
	{}	/* Terminator */
};

static int board_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_DOWN),
	KEY(0, 2, KEY_ENTER),
	KEY(0, 3, KEY_M),

	KEY(1, 0, KEY_RIGHT),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_I),
	KEY(1, 3, KEY_N),

	KEY(2, 0, KEY_A),
	KEY(2, 1, KEY_E),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_O),

	KEY(3, 0, KEY_B),
	KEY(3, 1, KEY_F),
	KEY(3, 2, KEY_K),
	KEY(3, 3, KEY_P)
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data amgxlcore_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 4,
	.cols		= 4,
	.rep		= 1,
};

static struct twl4030_madc_platform_data amgxlcore_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_usb_data amgxlcore_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static int amgxlcore_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	omap_mux_init_gpio(27, OMAP_PIN_INPUT);
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	return 0;
}

static struct twl4030_gpio_platform_data amgxlcore_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= amgxlcore_twl_gpio_setup,
};

static struct twl4030_codec_audio_data amgxlcore_audio_data;

static struct twl4030_codec_data amgxlcore_codec_data = {
	.audio_mclk = 26000000,
	.audio = &amgxlcore_audio_data,
};

static struct regulator_consumer_supply amgxlcore_vdda_dac_supply[] = {
	REGULATOR_SUPPLY("vdda_dac", "omapdss_venc"),
};

/* VDAC for DSS driving S-Video */
static struct regulator_init_data amgxlcore_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(amgxlcore_vdda_dac_supply),
	.consumer_supplies	= amgxlcore_vdda_dac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply amgxlcore_vpll2_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi1"),
};

static struct regulator_init_data amgxlcore_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(amgxlcore_vpll2_supply),
	.consumer_supplies	= amgxlcore_vpll2_supply,
};

static struct regulator_consumer_supply amgxlcore_vmmc1_supply[] = {
	REGULATOR_SUPPLY("vmmc", "omap_hsmmc.0"),
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data amgxlcore_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(amgxlcore_vmmc1_supply),
	.consumer_supplies	= amgxlcore_vmmc1_supply,
};

static struct twl4030_platform_data amgxlcore_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.keypad		= &amgxlcore_kp_data,
	.madc		= &amgxlcore_madc_data,
	.usb		= &amgxlcore_usb_data,
	.gpio		= &amgxlcore_gpio_data,
	.codec		= &amgxlcore_codec_data,
	.vdac		= &amgxlcore_vdac,
	.vpll2		= &amgxlcore_vpll2,
	.vmmc1		= &amgxlcore_vmmc1,
};

static int __init amgxlcore_i2c_init(void)
{
	omap3_pmic_init("twl4030", &amgxlcore_twldata);
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}

/* -----------------------------------------------------------------------------
 *
 */

static struct omap_board_config_kernel amgxlcore_config[] __initdata = {
};

static void __init amgxlcore_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(mt46h32m32lf6_sdrc_params, NULL);
}

static struct usbhs_omap_board_data amgxlcore_usbhs_bdata __initconst = {

	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset = true,
	/* PHY reset GPIO will be runtime programmed based on EVM version */
	.reset_gpio_port[0] = -EINVAL,
	.reset_gpio_port[1] = -EINVAL,
	.reset_gpio_port[2] = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP |
				OMAP_PIN_OFF_WAKEUPENABLE),
	OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP |
				OMAP_PIN_OFF_WAKEUPENABLE),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct omap_musb_board_data amgxlcore_musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_PERIPHERAL,
	.power			= 100,
};

static void __init amgxlcore_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CUS);

	omap_board_config = amgxlcore_config;
	omap_board_config_size = ARRAY_SIZE(amgxlcore_config);

	omap_serial_init();

	amgxlcore_i2c_init();

	/* setup EHCI phy reset on MDC */
	omap_mux_init_gpio(24, OMAP_PIN_OUTPUT);
	amgxlcore_usbhs_bdata.reset_gpio_port[1] = 1;

	usb_musb_init(&amgxlcore_musb_board_data);
	usbhs_init(&amgxlcore_usbhs_bdata);

	amgxlcore_smsc911x_init();

	amgxlcore_display_init();
	omap_display_init(&amgxlcore_dss_data);

	amgxlcore_camera_init();
}

MACHINE_START(AMG_XLCORE_CAMERA, "AMG XL Core Camera")
	/* Maintainer: Jason Mecham - Westover AMG */
	.boot_params	= 0x80000100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= amgxlcore_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= amgxlcore_init,
	.timer		= &omap_timer,
MACHINE_END
