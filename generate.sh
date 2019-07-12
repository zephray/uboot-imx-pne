dd if=u-boot.bin of=u-boot-with-kernel.bin bs=128k conv=sync
cat ../linux-imx/arch/arm/boot/zImage >> u-boot-with-kernel.bin
cat ../linux-imx/arch/arm/boot/dts/imx6ul-14x14-pne.dtb >> u-boot-with-kernel.bin
./tools/mkimage -n ./board/freescale/mx6ul_14x14_evk/imximage.cfg.cfgtmp -T imximage -e 0x80800000 -d u-boot-with-kernel.bin u-boot-with-kernel.imx
sudo dd if=u-boot-with-kernel.imx of=/dev/sdb bs=1k seek=1 conv=fsync
