/***************************************************************************
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/*
 *	Copyright (C) 2019 Tom Jones tj@enoti.me
 */

/* 
 * port of freebsd gpio interface 
 */

/* 2014-12: Addition of the SWD protocol support is based on the initial work
 * on bcm2835gpio.c by Paul Fertser and modifications by Jean-Christian de Rivaz. */

/**
 * @file
 * This driver implements a bitbang jtag interface using gpio lines via
 * sysfs.
 * The aim of this driver implementation is use system GPIOs but avoid the
 * need for a additional kernel driver.
 * (Note memory mapped IO is another option, however it doesn't mix well with
 * the kernel gpiolib driver - which makes sense I guess.)
 *
 * A gpio is required for tck, tms, tdi and tdo. One or both of srst and trst
 * must be also be specified. The required jtag gpios are specified via the
 * sysfsgpio_jtag_nums command or the relevant sysfsgpio_XXX_num commang.
 * The srst and trst gpios are set via the sysfsgpio_srst_num and
 * sysfsgpio_trst_num respectively. GPIO numbering follows the kernel
 * convention of starting from 0.
 *
 * The gpios should not be in use by another entity, and must not be requested
 * by a kernel driver without also being exported by it (otherwise they can't
 * be exported by sysfs).
 *
 * The sysfs gpio interface can only manipulate one gpio at a time, so the
 * bitbang write handler remembers the last state for tck, tms, tdi to avoid
 * superfluous writes.
 * For speed the sysfs "value" entry is opened at init and held open.
 * This results in considerable gains over open-write-close (45s vs 900s)
 *
 * Further work could address:
 *  -srst and trst open drain/ push pull
 *  -configurable active high/low for srst & trst
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <sys/types.h>
#include <libgpio.h>

#include <jtag/interface.h>
#include "bitbang.h"

/*
 * Helper func to determine if gpio number valid
 *
 * Assume here that there will be less than 10000 gpios on a system
 */
static int is_gpio_valid(int gpio)
{
	return gpio >= 0 && gpio < 10000;
}

/*
 * Configure gpio
 * If the gpio is an output, it is initialized according to init_high,
 * otherwise it is ignored.
 *
 */
static int setup_gpio(gpio_handle_t handle, int gpio, int is_output, int init_high)
{
	int ret = -1;

	if (is_output) {
		gpio_pin_output(handle, gpio);
		if (init_high)
			gpio_pin_high(handle, gpio);
	} else
		gpio_pin_input(handle, gpio);
	return ret;
}

/* default to bus 0 */
static int gpiobus = 0;

/* gpio numbers for each gpio. Negative values are invalid */
static int tck_gpio = -1;
static int tms_gpio = -1;
static int tdi_gpio = -1;
static int tdo_gpio = -1;
static int trst_gpio = -1;
static int srst_gpio = -1;
static int swclk_gpio = -1;
static int swdio_gpio = -1;

/*
 * gpio handles for parent gpio controllers
 */
static int tck_handle = -1;
static int tms_handle = -1;
static int tdi_handle = -1;
static int tdo_handle = -1;
static int trst_handle = -1;
static int srst_handle = -1;
static int swclk_handle = -1;
static int swdio_handle = -1;

static int last_swclk;
static int last_swdio;
static bool last_stored;
static bool swdio_input;

static void freebsdgpio_swdio_drive(bool is_output)
{
	if (is_output) {
		gpio_pin_input(swdio_handle, swdio_gpio);
		gpio_pin_high(swdio_handle, swdio_gpio);
	} else
		gpio_pin_input(swdio_handle, swdio_gpio);


	last_stored = false;
	swdio_input = !is_output;
}

static int freebsdgpio_swdio_read(void)
{
	// the sysfs logic is really silly
	//return buf[0] != '0';

	return gpio_pin_get(swdio_handle, swdio_gpio) != GPIO_PIN_LOW;
}

static void freebsdgpio_swdio_write(int swclk, int swdio)
{
	if (!swdio_input) {
		if (!last_stored || (swdio != last_swdio)) {
			if (swdio)
				gpio_pin_high(swdio_handle, swdio);
			else
				gpio_pin_low(swdio_handle, swdio);
		}
	}

	/* write swclk last */
	if (!last_stored || (swclk != last_swclk)) {
		if (swclk)
			gpio_pin_high(swclk_handle, swclk);
		else
			gpio_pin_low(swclk_handle, swclk);
	}

	last_swdio = swdio;
	last_swclk = swclk;
	last_stored = true;
}

/*
 * Bitbang interface read of TDO
 *
 */
static bb_value_t freebsdgpio_read(void)
{
	gpio_value_t value;

	value = gpio_pin_get(tdo_handle, tdo_gpio);
	return (value == GPIO_PIN_HIGH) ? BB_HIGH : BB_LOW;
}

/*
 * Bitbang interface write of TCK, TMS, TDI
 *
 * Seeing as this is the only function where the outputs are changed,
 * we can cache the old value to avoid needlessly writing it.
 */
static int freebsdgpio_write(int tck, int tms, int tdi)
{
	if (swd_mode) {
		freebsdgpio_swdio_write(tck, tdi);
		return ERROR_OK;
	}

	static int last_tck;
	static int last_tms;
	static int last_tdi;

	static int first_time;

	if (!first_time) {
		last_tck = !tck;
		last_tms = !tms;
		last_tdi = !tdi;
		first_time = 1;
	}

	if (tdi != last_tdi) {
		if (tdi)
			gpio_pin_high(tdi_handle, tdi);
		else
			gpio_pin_low(tdi_handle, tdi);
	}

	if (tms != last_tms) {
		if (tms)
			gpio_pin_high(tms_handle, tms);
		else
			gpio_pin_low(tms_handle, tms);
	}

	/* write clk last */
	if (tck != last_tck) {
		if (tck)
			gpio_pin_high(tck_handle, tck);
		else
			gpio_pin_low(tck_handle, tck);
	}

	last_tdi = tdi;
	last_tms = tms;
	last_tck = tck;

	return ERROR_OK;
}

/*
 * Bitbang interface to manipulate reset lines SRST and TRST
 *
 * (1) assert or (0) deassert reset lines
 */
static int freebsdgpio_reset(int trst, int srst)
{
	LOG_DEBUG("freebsdgpio_reset");

	/* assume active low */
	if (srst_handle >= 0) {
		if (srst)
			gpio_pin_low(srst_handle, trst);
		else
			gpio_pin_high(srst_handle, trst);
	}

	/* assume active low */
	if (trst_handle >= 0) {
		if (trst)
			gpio_pin_low(srst_handle, trst);
		else
			gpio_pin_high(srst_handle, trst);
	}

	return ERROR_OK;
}

COMMAND_HANDLER( freebsdgpio_handle_bus)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], gpiobus);

	command_print(CMD, "gpiobus num: bus = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(freebsdgpio_handle_jtag_gpionums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tms_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], tdi_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"SysfsGPIO nums: tck = %d, tms = %d, tdi = %d, tdo = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(freebsdgpio_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD, "SysfsGPIO num: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(freebsdgpio_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD, "SysfsGPIO num: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(freebsdgpio_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD, "SysfsGPIO num: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(freebsdgpio_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD, "SysfsGPIO num: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(freebsdgpio_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "SysfsGPIO num: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(freebsdgpio_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "SysfsGPIO num: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(freebsdgpio_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD,
			"SysfsGPIO nums: swclk = %d, swdio = %d",
			swclk_gpio, swdio_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(freebsdgpio_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);

	command_print(CMD, "SysfsGPIO num: swclk = %d", swclk_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(freebsdgpio_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_gpio);

	command_print(CMD, "SysfsGPIO num: swdio = %d", swdio_gpio);
	return ERROR_OK;
}

static const struct command_registration freebsdgpio_command_handlers[] = {
	{
		.name = "freebsdgpio_bus",
		.handler = &freebsdgpio_handle_bus,
		.mode = COMMAND_CONFIG,
		.help = "gpio bus number",
		.usage = "[bus]",
	},
	{
		.name = "freebsdgpio_jtag_nums",
		.handler = &freebsdgpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo. (in that order)",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "freebsdgpio_tck_num",
		.handler = &freebsdgpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "freebsdgpio_tms_num",
		.handler = &freebsdgpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "freebsdgpio_tdo_num",
		.handler = &freebsdgpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "freebsdgpio_tdi_num",
		.handler = &freebsdgpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "freebsdgpio_srst_num",
		.handler = &freebsdgpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "freebsdgpio_trst_num",
		.handler = &freebsdgpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "freebsdgpio_swd_nums",
		.handler = &freebsdgpio_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio. (in that order)",
		.usage = "[swclk swdio]",
	},
	{
		.name = "freebsdgpio_swclk_num",
		.handler = &freebsdgpio_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "freebsdgpio_swdio_num",
		.handler = &freebsdgpio_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},
	COMMAND_REGISTRATION_DONE
};

static int freebsdgpio_init(void);
static int freebsdgpio_quit(void);

static const char * const freebsdgpio_transports[] = { "jtag", "swd", NULL };

struct jtag_interface freebsdgpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

static struct bitbang_interface freebsdgpio_bitbang = {
	.read = freebsdgpio_read,
	.write = freebsdgpio_write,
	//.reset = freebsdgpio_reset,
	.swdio_read = freebsdgpio_swdio_read,
	.swdio_drive = freebsdgpio_swdio_drive,
	.blink = NULL
};

struct adapter_driver freebsdgpio_adapter_driver = {
        .name = "freebsdgpio",
        .transports = freebsdgpio_transports,
        .commands = freebsdgpio_command_handlers,

        .init = freebsdgpio_init,
        .quit = freebsdgpio_quit,
        .reset = freebsdgpio_reset,
//        .speed = freebsdgpio_speed,	TODO
//        .khz = freebsdgpio_khz,
//        .speed_div = freebsdgpio_speed_div,

        .jtag_ops = &freebsdgpio_interface,
        .swd_ops = &bitbang_swd,
};

static bool freebsdgpio_jtag_mode_possible(void)
{
	if (!is_gpio_valid(tck_gpio))
		return 0;
	if (!is_gpio_valid(tms_gpio))
		return 0;
	if (!is_gpio_valid(tdi_gpio))
		return 0;
	if (!is_gpio_valid(tdo_gpio))
		return 0;
	return 1;
}

static bool freebsdgpio_swd_mode_possible(void)
{
	if (!is_gpio_valid(swclk_gpio))
		return 0;
	if (!is_gpio_valid(swdio_gpio))
		return 0;
	return 1;
}

static int freebsdgpio_init(void)
{
// TODO: all the set up we must do to replace this
	bitbang_interface = &freebsdgpio_bitbang;

	LOG_INFO("FreeBSD GPIO JTAG/SWD bitbang driver");

	// mode possible just checks if the configured pins are valid gpio
	// valid is a check that gpio >0 <= 1000
	if (freebsdgpio_jtag_mode_possible()) {
		if (freebsdgpio_swd_mode_possible())
			LOG_INFO("JTAG and SWD modes enabled");
		else
			LOG_INFO("JTAG only mode enabled (specify swclk and swdio gpio to add SWD mode)");
	} else if (freebsdgpio_swd_mode_possible()) {
		LOG_INFO("SWD only mode enabled (specify tck, tms, tdi and tdo gpios to add JTAG mode)");
	} else {
		LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode and/or swclk and swdio gpio for SWD mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_INFO("Defaulting to opening gpioc0");
	gpio_handle_t gpioc = gpio_open(0);

	tck_handle = gpioc;
	tms_handle = gpioc;
	tdi_handle = gpioc;
	tdo_handle = gpioc;
	trst_handle = gpioc;
	srst_handle = gpioc;
	swclk_handle = gpioc;
	swdio_handle = gpioc;

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST
	 * as outputs.  Drive TDI and TCK low, and TMS/TRST/SRST high.
	 * For SWD, SWCLK and SWDIO are configures as output high.
	 */
	if (tck_gpio >= 0) {
		tck_handle = setup_gpio(tck_handle, tck_gpio, 1, 0);
		if (tck_handle < 0)
			goto out_error;
	}

	if (tms_gpio >= 0) {
		tms_handle = setup_gpio(tms_handle, tms_gpio, 1, 1);
		if (tms_handle < 0)
			goto out_error;
	}

	if (tdi_gpio >= 0) {
		tdi_handle = setup_gpio(tdi_handle, tdi_gpio, 1, 0);
		if (tdi_handle < 0)
			goto out_error;
	}

	if (tdo_gpio >= 0) {
		tdo_handle = setup_gpio(tdo_handle, tdo_gpio, 0, 0);
		if (tdo_handle < 0)
			goto out_error;
	}

	/* assume active low*/
	if (trst_gpio >= 0) {
		trst_handle = setup_gpio(trst_handle, trst_gpio, 1, 1);
		if (trst_handle < 0)
			goto out_error;
	}

	/* assume active low*/
	if (srst_gpio >= 0) {
		srst_handle = setup_gpio(srst_handle, srst_gpio, 1, 1);
		if (srst_handle < 0)
			goto out_error;
	}

	if (swclk_gpio >= 0) {
		swclk_handle = setup_gpio(swclk_handle, swclk_gpio, 1, 0);
		if (swclk_handle < 0)
			goto out_error;
	}

	if (swdio_gpio >= 0) {
		swdio_handle = setup_gpio(swdio_handle, swdio_gpio, 1, 0);
		if (swdio_handle < 0)
			goto out_error;
	}

	if (freebsdgpio_swd_mode_possible()) {
		if (swd_mode)
			bitbang_swd_switch_seq(JTAG_TO_SWD);
		else
			bitbang_swd_switch_seq(SWD_TO_JTAG);
	}

	return ERROR_OK;

out_error:
	//cleanup_all_fds();
	return ERROR_JTAG_INIT_FAILED;
}

static int freebsdgpio_quit(void)
{
	// TOOD: tidy up
	//cleanup_all_fds();
	return ERROR_OK;
}

