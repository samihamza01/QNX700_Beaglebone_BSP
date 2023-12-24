ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

PSTAG_64 = .64
LIB_VARIANT = $(subst .o,,a.$(COMPOUND_VARIANT))$(PSTAG_$(PADDR_SIZE))

LINKER_TYPE=BOOTSTRAP
INSTALLDIR = boot/sys
LIBS += startup$(subst .,-,$(PSTAG_$(PADDR_SIZE))) lzo2 ucl drvr 

# The board level is one up from the CPU level
BOARD_ROOT=$(patsubst %/,%,$(dir $(CPU_ROOT)))
BOARD=$(notdir $(BOARD_ROOT))

NAME = startup-$(BOARD)
EXTRA_SILENT_VARIANTS+=$(subst -, ,$(BOARD) $(SECTION))
USEFILE = 

EXTRA_SRCVPATH +=	$(BOARD_ROOT)/overrides

EXTRA_INC_x86 = common_x86
EXTRA_INC_x86_64 = common_x86
EXTRA_INCVPATH +=	$(BOARD_ROOT)/overrides \
					$(LIBSTARTUP_ROOT)/$(CPU)/$(LIB_VARIANT) \
					$(LIBSTARTUP_ROOT)/$(CPU) \
					$(LIBSTARTUP_ROOT)/$(EXTRA_INC_$(CPU)) \
					$(LIBSTARTUP_ROOT) \
					$(LIBSTARTUP_ROOT)/public

EXTRA_LIBVPATH +=	$(LIBSTARTUP_ROOT)/$(CPU)/$(LIB_VARIANT) \
					$(USE_ROOT_LIB) \
					$(QNX_TARGET)/$(CPUVARDIR)/lib \
					$(QNX_TARGET)/$(CPUVARDIR)/usr/lib

EXTRA_INCVPATH+=$(INSTALL_ROOT_nto)/usr/include/xilinx


#LDBOOTSTRAPPOST_nto_x86_gcc_qcc:=$(subst -lc, -L$(QNX_TARGET)/x86/lib -lc, $(LDBOOTSTRAPPOST_nto_x86_gcc_qcc))

CCFLAG_64 = -D_PADDR_BITS=64
CCFLAGS_gcc_ = -O2 -fomit-frame-pointer
CCFLAGS_clang_ = -O2 -fomit-frame-pointer
CCFLAGS_gcc_qcc = -O2 -Wc,-fomit-frame-pointer 
CCFLAGS_$(BUILDENV) = -DBUILDENV_$(BUILDENV)
CCFLAGS = $(CCFLAGS_$(COMPILER_TYPE)_$(COMPILER_DRIVER)) $(CCFLAG_$(PADDR_SIZE)) $(CCFLAGS_$(BUILDENV))

LDFLAGS_gcc_qcc = -Wl,-Map,$@.map
LDFLAGS = $(LDFLAGS_$(COMPILER_TYPE)_$(COMPILER_DRIVER))

EXTRA_ICLEAN=$(BOARD_ROOT)/*.pinfo

define POST_INSTALL
	-$(foreach build,$(EXAMPLE_BUILDFILES) $(EXAMPLE_READMES), $(CP_HOST) $(build) $(INSTALL_ROOT_nto)/$(CPUVARDIR)/boot/build/$(BOARD).$(notdir $(build));)
endef

include $(MKFILES_ROOT)/qmacros.mk
ADD_USAGE=

-include $(PROJECT_ROOT)/roots.mk

ifndef LIBSTARTUP_ROOT
LIBSTARTUP_ROOT=$(PRODUCT_ROOT)/lib
endif

EXAMPLE_BUILDFILES:=$(wildcard *build $(BOARD_ROOT)/*build)
EXAMPLE_READMES:=$(wildcard $(BOARD_ROOT)/*readme)

EBF_PINFOS:=$(addsuffix .pinfo, $(EXAMPLE_BUILDFILES))
ERM_PINFOS:=$(addsuffix .pinfo, $(EXAMPLE_READMES))

EXTRA_DEPS = $(EBF_PINFOS) $(ERM_PINFOS)

include $(BOARD_ROOT)/pinfo.mk

ifeq ($(NEEDS_FDT),yes)
LIBS += fdt
endif



#####AUTO-GENERATED by packaging script... do not checkin#####
   INSTALL_ROOT_nto = $(PROJECT_ROOT)/../../../../install
   USE_INSTALL_ROOT=1
##############################################################

include $(MKFILES_ROOT)/qtargets.mk


-include $(PROJECT_ROOT)/announce.mk


define DO_PINFO
	@$(ECHO_HOST)  >$@ STATE=Experimental 
	@$(ECHO_HOST) >>$@ INSTALLDIR=$(CPUVARDIR)/boot/build/
	@$(ECHO_HOST) >>$@ INSTALLNAME=$(BOARD).$(patsubst %.pinfo,%,$(@F))
	@$(ECHO_HOST) >>$@ NAME=$(patsubst %.pinfo,%,$(@F))
	@$(ECHO_HOST) >>$@ USER=$(shell $(USER_HOST))
	@$(ECHO_HOST) >>$@ HOST=$(shell $(HOST_HOST))
	@$(ECHO_HOST) >>$@ DATE=$(shell $(DATE_HOST))
endef

$(EBF_PINFOS): $(EXAMPLE_BUILDFILES)
	$(DO_PINFO)
	@$(ECHO_HOST) >>$@ DESCRIPTION=Example build file for $(NAME)

$(ERM_PINFOS): $(EXAMPLE_READMES)
	$(DO_PINFO)
	@$(ECHO_HOST) >>$@ DESCRIPTION=Readme file for $(BOARD)

#
# This particular little kludge is to stop GCC from using F.P. instructions
# to move 8 byte quantities around. 
#
ifeq ($(BUILDENV),qss)
CC_nto_ppc_gcc += -mno-fp-moves
CC_nto_ppc_gcc_qcc += -Wc,-mno-fp-moves
else
CC_nto_ppc_gcc += -msoft-float
CC_nto_ppc_gcc_qcc += -Wc,-msoft-float
endif

CCFLAGS_arm += -marm -D_PADDR_BITS=64
CCFLAGS += $(CCFLAGS_$(CPU))

# Prevent building with debug for the standard build.
# Temporary until linker has been changed.
CCOPTS:=$(filter-out -g,$(CCOPTS))

