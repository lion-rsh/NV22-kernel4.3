/*
 * Copyright (c) 2016, NVIDIA CORPORATION.  All rights reserved.
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


#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>

/* TI FPD Link III 954 deser I2C address */
#define TI954_ADDR  (0x30)
/* TI FPD Link III 913 ser I2C address */
#define TI913_ADDR  (0x58) //0x58 << 1 = 0xb0

#define ISP_ADDR (0x5d) //0x5d << 1  = 0xBA

struct fpdlink_reg_struct {
	u8 addr;
	u8 val;
};

struct fpdlink_reg_struct TI954_reg_list[] = {

	{0xB3,0x00},/*port1 receive enable*/
	{0x4c,0x01},
	{0x58,0x58},/*for 913*/

	{0x5b,(TI913_ADDR << 1)},/* TI 913 slave address, it is able to automatically get from TI 913 */
	{0x5c,(TI913_ADDR << 1)},/* TI 913 alias address */

	{0x5d,(ISP_ADDR << 1)},/* CAM salve address */
	{0x65,(ISP_ADDR << 1)},/* CAM alias address */

	{0x1f,0x03},/*CSI_TX_SPEED:0x03:400Mbps,  0x02:800Mbps, 0x00: 1.6Gbps */
	//{0x32,0x01},
	{0x20,0x20},/*FWD_CTL1:RX PORT0 or PORT1  0x10-port1 enable*/

	{0x70,0x1e},/*0x2b: raw10    0x1e: yuv*/
	{0x7c,0xE0},/* 0xE0  RAW10 lower 8 bit;   0xa0  RAW10 upper 8 bit*/
	{0x33,0x01},/*CSI_CTL register(address=0x33):0x01: 4lane,  0x11: 3lane,  0x21: 2lane,  0x31: 1lane*/
	{0xFF,0x00}
};

struct fpdlink_reg_struct TI913_reg_list[] = {
	{ 0x0d, 0x00},/* enable remote data */
	{ 0x0d, 0x99},/* enable remote data */
};

static int i2c_wr8(struct i2c_client *client, u8 addr, u8 val)
{
	int err;
	struct i2c_msg msg;
	u8 data[2];

	msg.addr = client->addr;
	msg.buf = data;
	msg.len = 2;
	msg.flags = 0;

	data[0] = addr;
	data[1] = val;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		printk("%s: wr8 register failed\n", __func__);
		return 0;
	}

	return 0;
}

static u8 i2c_rd8(struct i2c_client *client, u8 addr, u8 *val)
{
	int err;
	u8 buf[2] = { addr, 0 };

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		},
	};

	err = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (err != ARRAY_SIZE(msgs)) {
		printk("%s: reading register 0x%x from 0x%x failed\n",
			__func__, addr, client->addr);
	}

	return 0;
}

static int TI954_reg_write(struct i2c_client *client, u8 addr, u8 val)
{
	int addr_bak = client->addr;

	client->addr = TI954_ADDR;
	i2c_wr8(client, addr, val);
	client->addr = addr_bak;

	return 0;
}

static int TI954_reg_read(struct i2c_client *client, u8 addr, u8 *val)
{
	int addr_bak = client->addr;

	client->addr = TI954_ADDR;
	i2c_rd8(client, addr, val);
	client->addr = addr_bak;

	return 0;
}

static int TI913_reg_write(struct i2c_client *client, u8 addr, u8 val)
{
	int addr_bak = client->addr;

	client->addr = TI913_ADDR;
	i2c_wr8(client, addr, val);
	client->addr = addr_bak;

	return 0;
}

static int TI913_reg_read(struct i2c_client *client, u8 addr, u8 *val)
{
	int addr_bak = client->addr;

	client->addr = TI913_ADDR;
	i2c_rd8(client, addr, val);
	client->addr = addr_bak;

	return 0;
}


int fpdlink_init(struct i2c_client *client)
{
	int size, i;
	u8 val;

	size = sizeof(TI954_reg_list) / sizeof(struct fpdlink_reg_struct);
	for (i = 0; i < size; i++) {
		TI954_reg_write(client, TI954_reg_list[i].addr, TI954_reg_list[i].val);
		TI954_reg_read(client, TI954_reg_list[i].addr, &val);
		printk("TI954 0x%x = 0x%x\n", TI954_reg_list[i].addr, val);
	}

	mdelay(100);

	size = sizeof(TI913_reg_list) / sizeof(struct fpdlink_reg_struct);
	for (i = 0; i < size; i++) {
		TI913_reg_write(client, TI913_reg_list[i].addr, TI913_reg_list[i].val);
		TI913_reg_read(client, TI913_reg_list[i].addr, &val);
		mdelay(200);
		printk("TI913 0x%x = 0x%x\n", TI913_reg_list[i].addr, val);
	}

	return 0;
}

EXPORT_SYMBOL(fpdlink_init);