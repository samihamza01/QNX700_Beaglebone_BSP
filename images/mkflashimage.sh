#!/bin/sh
# script to build a binary IPL and boot image for the am335x beaglebone board
# need space after the v on the next line to work. Not sure why.
set -e -v 
# Convert IPL header into Binary format
${QNX_HOST}/usr/bin/ntoarmv7-objcopy --input-format=elf32-littlearm --output-format=binary ../src/hardware/ipl/boards/am335x/arm/beaglebone.le.v7/boot_header.o ./tmp-boot-header.bin
${QNX_HOST}/usr/bin/ntoarmv7-objcopy --input-format=elf32-littlearm --output-format=binary ../src/hardware/ipl/boards/am335x/arm/beaglebone.le.v7/ch_header.o ./tmp-ch-header.bin

# Convert IPL into Binary format
${QNX_HOST}/usr/bin/ntoarmv7-objcopy --input-format=elf32-littlearm --output-format=binary -R.scratch ../src/hardware/ipl/boards/am335x/arm/beaglebone.le.v7/ipl-am335x-beaglebone ./tmp-ipl-am335x-beaglebone.bin

# Cat boot header and ipl together
cat ./tmp-ch-header.bin ./tmp-boot-header.bin ./tmp-ipl-am335x-beaglebone.bin > ./tmp-header-ipl-am335x-beaglebone.bin

# Pad Binary IPL to 32K image, this is the image used by boot from UART
mkrec -s32k -ffull -r ./tmp-ipl-am335x-beaglebone.bin > ./ipl-uart-am335x-beaglebone.bin

# Pad Binary IPL with header to 32K image, this is the image to put on SD and boot from SD
mkrec -s32k -ffull -r ./tmp-header-ipl-am335x-beaglebone.bin > ./ipl-sd-am335x-beaglebone.bin

# clean up temporary files
rm -f tmp*.bin

echo "done!!!!!!!"
