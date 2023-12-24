ROOT_DIR := $(notdir $(CURDIR))
ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

HOST_MKIFS := mkifs

SUFFIXES := .build .ifs .raw

.PHONY: all clean

all: ipl-am335x-beaglebone ifs-ti-am335x-beaglebone.bin

clean:
	$(RM_HOST) ifs-ti-am335x-beaglebone.bin

ipl-am335x-beaglebone:
	$(SHELL) ./mkflashimage.sh

ifs-ti-am335x-beaglebone.bin: beaglebone.build
	$(HOST_MKIFS) -r../install -v $(MKIFSFLAGS) $^ $@


