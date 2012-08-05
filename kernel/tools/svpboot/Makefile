CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
KERNELDIR ?= ../..

SCRIPT = svpboot.ld
ASFLAGS = -D__ASSEMBLY__ -I$(KERNELDIR)/include -I$(KERNELDIR)/arch/arm/include
ASFLAGS += -I$(KERNELDIR)/arch/arm/mach-ux500/include
LDFLAGS = -nostdlib -Wl,-T,$(SCRIPT)

OBJS = svp5500 svp8500v1 svp8500v2

all:
	$(CC) $(ASFLAGS) -c svpboot.S -DSVP5500 -o svp5500.o
	$(CC) $(ASFLAGS) -c svpboot.S -DSVP8500V1 -o svp8500v1.o
	$(CC) $(ASFLAGS) -c svpboot.S -DSVP8500V2 -o svp8500v2.o
	$(CC) $(LDFLAGS) svp5500.o -o svp5500
	$(CC) $(LDFLAGS) svp8500v1.o -o svp8500v1
	$(CC) $(LDFLAGS) svp8500v2.o -o svp8500v2

clean:
	rm -rf $(OBJS) *.o *.bin
