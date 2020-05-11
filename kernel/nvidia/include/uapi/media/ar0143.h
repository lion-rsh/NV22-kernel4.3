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

#ifndef __UAPI_AR0143_H__
#define __UAPI_AR0143_H__

#include <uapi/media/nvc.h>
#include <uapi/media/nvc_image.h>

#define AR0143_IOCTL_SET_MODE               _IOW('o', 1, struct ar0143_mode)
#define AR0143_IOCTL_SET_FRAME_LENGTH       _IOW('o', 2, __u32)
#define AR0143_IOCTL_SET_COARSE_TIME        _IOW('o', 3, __u32)
#define AR0143_IOCTL_SET_GAIN               _IOW('o', 4, __u16)
#define AR0143_IOCTL_GET_STATUS             _IOR('o', 5, __u8)
#define AR0143_IOCTL_SET_BINNING            _IOW('o', 6, __u8)
#define AR0143_IOCTL_TEST_PATTERN           _IOW('o', 7, \
						 enum AR0143_test_pattern)
#define AR0143_IOCTL_SET_GROUP_HOLD         _IOW('o', 8, struct ar0143_ae)
/* IOCTL to set the operating mode of camera.
 * This can be either stereo , leftOnly or rightOnly */
#define AR0143_IOCTL_SET_CAMERA_MODE        _IOW('o', 10, __u32)
#define AR0143_IOCTL_SYNC_SENSORS           _IOW('o', 11, __u32)
#define AR0143_IOCTL_GET_FUSEID             _IOR('o', 12, struct nvc_fuseid)
#define AR0143_IOCTL_SET_HDR_COARSE_TIME    _IOW('o', 13, struct ar0143_Hdr)
#define AR0143_IOCTL_READ_OTP_BANK          _IOWR('o', 14, \
						struct AR0143_otp_bank)
#define AR0143_IOCTL_SET_CAL_DATA           _IOW('o', 15, \
						struct AR0143_cal_data)
#define AR0143_IOCTL_GET_EEPROM_DATA        _IOR('o', 20, __u8 *)
#define AR0143_IOCTL_SET_EEPROM_DATA        _IOW('o', 21, __u8 *)
#define AR0143_IOCTL_GET_CAPS               _IOR('o', 22, struct nvc_imager_cap)
#define AR0143_IOCTL_SET_POWER              _IOW('o', 23, __u32)

#define AR0143_FUSE_ID_OTP_START_ADDR	0x1//modify by lk:org-0x3D00
#define AR0143_FUSE_ID_OTP_BANK 0
#define AR0143_FUSE_ID_SIZE		6//modify by lk:org-8
#define AR0143_FUSE_ID_STR_SIZE (AR0143_FUSE_ID_SIZE * 2)

#define AR0143_FRAME_LENGTH_ADDR_MSB		0x1A //modify by lk:org-0x380E
#define AR0143_FRAME_LENGTH_ADDR_MID		0x19 //Add by lk:
#define AR0143_FRAME_LENGTH_ADDR_LSB		0x18//modify by lk:org-0x380F
#define AR0143_COARSE_TIME_ADDR_1		0x22 //modify by lk: org-0x3500
#define AR0143_COARSE_TIME_ADDR_2		0x21//modify by lk: org-0x3501
#define AR0143_COARSE_TIME_ADDR_3		0x20//modify by lk: org-0x3502
#define AR0143_COARSE_TIME_SHORT_ADDR_1	0x25//modify by lk: org-0x3506
#define AR0143_COARSE_TIME_SHORT_ADDR_2	0x24//modify by lk: org-0x3507
#define AR0143_COARSE_TIME_SHORT_ADDR_3	0x23//modify by lk: org-0x3508
#define AR0143_GAIN_ADDR			    0x14//add by lk

#define AR0143_GROUP_HOLD_ADDR			0x01//modify by lk: org-0x3208
#define AR0143_TIMING_REG20			0x3820
#define VERTICAL_FLIP				((0x1 << 1) | (0x1 << 6))
#define AR0143_TIMING_REG21			0x3821
#define HORIZONTAL_MIRROR_MASK			(0x3 << 1)

struct ar0143_mode {
	int res_x;
	int res_y;
	int fps;
	__u32 frame_length;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u16 gain;
	__u8 hdr_en;
};

struct ar0143_ae {
	__u32 frame_length;
	__u8  frame_length_enable;
	__u32 coarse_time;
	__u32 coarse_time_short;
	__u8  coarse_time_enable;
	__s32 gain;
	__u8  gain_enable;
};

struct ar0143_fuseid {
	__u32 size;
	__u8  id[16];
};

struct ar0143_Hdr {
	__u32 coarse_time_long;
	__u32 coarse_time_short;
};

struct ar0143_otp_bank {
	__u32 id;
	__u8  buf[16];
};

struct ar0143_cal_data {
	int loaded;
	int rg_ratio;
	int bg_ratio;
	int rg_ratio_typical;
	int bg_ratio_typical;
	__u8 lenc[62];
};

#endif  /* __UAPI_AR0143_H__ */