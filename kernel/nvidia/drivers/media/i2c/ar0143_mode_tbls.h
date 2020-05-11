/*
 * ar0143_mode_tbls.h - ar0143 sensor mode tables
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION, All Rights Reserved.
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

#ifndef __AR0143_TABLES__
#define __AR0143_TABLES__

#include <media/camera_common.h>

#define AR0143_TABLE_WAIT_MS	0xFE
#define AR0143_TABLE_END	0xFF
#define AR0143_MAX_RETRIES	3
#define AR0143_WAIT_MS		10

#define ENABLE_EXTRA_MODES 0

#define ar0143_reg struct reg_8

static const ar0143_reg ar0143_start[] = {
	{0x33,0x01}, /*csi enable*/
	{AR0143_TABLE_END, 0x00}
};

static const ar0143_reg ar0143_stop[] = {
	{0x33,0x00}, /*csi disable*/
	{AR0143_TABLE_END, 0x00}
};



static const ar0143_reg mode_1280x720[] = {

};

enum {
	AR0143_MODE_1280x720,

	AR0143_MODE_START_STREAM,
	AR0143_MODE_STOP_STREAM,
};

static const ar0143_reg *mode_table[] = {
	[AR0143_MODE_1280x720]          = mode_1280x720,

	[AR0143_MODE_START_STREAM]		= ar0143_start,
	[AR0143_MODE_STOP_STREAM]		= ar0143_stop,
};

static const int ar0143_30fps[] = {
	30,
};

static const struct camera_common_frmfmt ar0143_frmfmt[] = {
	{{1280, 719},	ar0143_30fps,	1, 1,	AR0143_MODE_1280x720},

#if ENABLE_EXTRA_MODES

#endif
};
#endif  /* __AR0143_TABLES__ */

