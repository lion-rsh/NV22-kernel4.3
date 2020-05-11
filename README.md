L4T_32.2.3

cd /home/lion/project/nv22/src/kernel-4.3/kernel/kernel-4.9
TEGRA_KERNEL_OUT=/home/lion/project/nv22/src/nv22-kernelout4.3
export CROSS_COMPILE=/home/lion/tools/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
make ARCH=arm64 O=$TEGRA_KERNEL_OUT tegra_defconfig
make ARCH=arm64 O=$TEGRA_KERNEL_OUT -j4

+++++++++++++++++++++++++++++++++++++++++++++

cd /home/lion/nvidia/xavier16G-jp4.2.3/JetPack_4.3_Linux_P2888/Linux_for_Tegra

sudo ./flash.sh -r -k APP -G ~/nvidia/xavier16G-jp4.2.3/JetPack_4.3_Linux_P2888/Linux_for_Tegra/bootloader/clone-system.img jetson-xavier mmcblk0p1

To flash the kernel on Jetson AGX Xavier using the default file <L4T>/kernel/Image:
sudo ./flash.sh -k kernel jetson-xavier mmcblk0p1
To flash mb1 bct on Jetson AGX Xavier using a predefined list of configuration files:
sudo ./flash.sh -k MB1_BCT jetson-xavier mmcblk0p1

sudo ./flash.sh -r jetson-xavier mmcblk0p1
sudo ./flash.sh -r -k kernel-dtb jetson-xavier mmcblk0p1
sudo ./flash.sh -r -K kernel/Image -d kernel/dtb/tegra194-p2888-0001-p2822-0000.dtb jetson-xavier mmcblk0p1
sudo ./flash.sh -r -K kernel/Image -d kernel/dtb jetson-xavier mmcblk0p1
sudo ./flash.sh -r -K kernel/Image jetson-xavier mmcblk0p1

sudo ./flash.sh -r -k kernel-dtb jetson-xavier mmcblk0p1  //烧写设备树文件

+++++++++++++++++++++++++++++++++++++++++++++

project name: NV22
function:{
	CAN for x2
	miniPCIeCAN-II x2
	PCIe--ETH (I210) x2
	USB3.0 x1
	FPD-link III x2
	GMSL x1
}

AGX-xavier4.3
author:lionking <lk_xiang@sina.com>
1.add 4G-driver(EC20F module)
date:2020.1.1
modified files:
2.add wifi module driver
date:2020.1.5
3.add sound card(tlv320aic32x4)
date:2020.1.7
modified files:
{
	tegra194-audio-p2822-0000.dtsi
	tlv320aic32x4.c
	tlv320aic32x4.h
	tlv320aic32x4-i2c.c
	kconfig
	tegra_machine_driver_mobile.c
}
4.add fpd-link camera(ov5693 ar0143)
date:2020.4.8
add files:
{
	ar0143.c
	ar0143_mode_tbls.h
	fpdlink.c

	tegra194-camera-ar0143-a00.dtsi
	tegra194-p2822-0000-camera-ar0143-a00.dtsi
}
modified files:
{
	Makefile
	kconfig
	tegra_deconfig
	sensor_common.c
	camera_common.c

	tegra194-camera-plugin-manager.dtsi
	tegra194-p2822-camera-modules.dtsi
}

5.add pcie x1 c4
date:2020.5.7
modified files:
{
	tegra194-p2888-0000-a00.dtsi
	tegra194-plugin-manager-e3366-1199.dtsi
}

6.add usb3.0(disabled type-c; enable usb3.0)
date:2020.5.7
modified files:
{
	tegra194-p2888-0001-p2822-0000-common.dtsi
}
