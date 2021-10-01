#  
#  Unified Building System for Linux
#  By CSK (csk@live.com)
#  

PRJ_ROOT:=../..
MOD_NAME:=libpng

include $(PRJ_ROOT)/scripts/def.mak


CSRC:= $(shell find core -name "*.c")
CXXSRC:= $(shell find core -name "*.cpp")

EXTRA_FLAGS:=-Wno-deprecated
CDEFS+=
DEP_AR+= 

INCLUDES+= 

all: $(STATIC_DEST)
additional_clean:
	rm -f $(STATIC_DEST)

include $(PRJ_ROOT)/scripts/common.mak



