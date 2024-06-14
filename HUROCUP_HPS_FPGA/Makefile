#
TARGET = HPS_FPGA_LED

#
ALT_DEVICE_FAMILY ?= soc_cv_av
SOCEDS_ROOT ?= $(SOCEDS_DEST_ROOT)
HWLIBS_ROOT = $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib
ALTLIB_ROOT = C:/altera/16.0/ip/altera/sopc_builder_ip
CROSS_COMPILE = arm-linux-gnueabihf-
NO_WARNING_ALERT = -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare -Wno-pointer-arith
CFLAGS = -g -Wall $(NO_WARNING_ALERT) -D$(ALT_DEVICE_FAMILY) -I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY)   -I$(HWLIBS_ROOT)/include/ -I$(ALTLIB_ROOT)/altera_avalon_pio/inc -I$(ALTLIB_ROOT)/altera_mp32/HAL/inc/ -IC:\Users\ICLAB\Desktop\HPS_test\HUROCUP_HPS_FPGA\include
LDFLAGS =  -g -Wall $(NO_WARNING_ALERT)
CC = $(CROSS_COMPILE)g++ -std=c++11
ARCH= arm
LINKMATH= -lm

build: $(TARGET)
$(TARGET): main.o
	$(CC) $(LDFLAGS)   $^ -o $@  $(LINKMATH)
%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@
	
	

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~ 