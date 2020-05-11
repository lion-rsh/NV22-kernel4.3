/*
 * Copyright (c) 2013-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __AR0143_H__
#define __AR0143_H__

#include <media/nvc.h>
#include <uapi/media/nvc_image.h>
#include <uapi/media/ar0143.h>

#define AR0143_INVALID_COARSE_TIME  -1

#define AR0143_EEPROM_ADDRESS		0x50
#define AR0143_EEPROM_SIZE		1024
#define AR0143_EEPROM_STR_SIZE		(AR0143_EEPROM_SIZE * 2)
#define AR0143_EEPROM_BLOCK_SIZE	(1 << 8)
#define AR0143_EEPROM_NUM_BLOCKS \
	(AR0143_EEPROM_SIZE / AR0143_EEPROM_BLOCK_SIZE)

#define AR0143_OTP_LOAD_CTRL_ADDR	0x3D81
#define AR0143_OTP_BANK_SELECT_ADDR	0x3D84
#define AR0143_OTP_BANK_START_ADDR	0x3D00
#define AR0143_OTP_BANK_END_ADDR	0x3D0F
#define AR0143_OTP_NUM_BANKS		(32)
#define AR0143_OTP_BANK_SIZE \
	 (AR0143_OTP_BANK_END_ADDR - AR0143_OTP_BANK_START_ADDR + 1)
#define AR0143_OTP_SIZE \
	 (AR0143_OTP_BANK_SIZE * AR0143_OTP_NUM_BANKS)
#define AR0143_OTP_STR_SIZE (AR0143_OTP_SIZE * 2)

/* See notes in the nvc.h file on the GPIO usage */
enum ar0143_gpio_type {
	AR0143_GPIO_TYPE_PWRDN = 0,
	AR0143_GPIO_TYPE_RESET,
};

struct AR0143_EEPROM_data {
	struct i2c_client *i2c_client;
	struct i2c_adapter *adap;
	struct i2c_board_info brd;
	struct regmap *regmap;
};

struct ar0143_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *dovdd;
};

struct ar0143_regulators {
	const char *avdd;
	const char *dvdd;
	const char *dovdd;
};

struct ar0143_platform_data {
	unsigned cfg;
	unsigned num;
	const char *dev_name;
	unsigned gpio_count; /* see nvc.h GPIO notes */
	struct nvc_gpio_pdata *gpio; /* see nvc.h GPIO notes */
	struct nvc_imager_static_nvc *static_info;
	bool use_vcm_vdd;
	int (*probe_clock)(unsigned long);
	int (*power_on)(struct ar0143_power_rail *);
	int (*power_off)(struct ar0143_power_rail *);
	const char *mclk_name;
	struct nvc_imager_cap *cap;
	struct ar0143_regulators regulators;
	bool has_eeprom;
	bool use_cam_gpio;
};

#endif  /* __AR0143_H__ */