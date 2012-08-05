
mcde-objs			:= mcde_mod.o mcde_hw.o mcde_dss.o mcde_display.o mcde_bus.o mcde_fb.o
obj-$(CONFIG_FB_MCDE)		+= mcde.o

obj-$(CONFIG_MCDE_DISPLAY_GENERIC_DSI)		+= display-generic_dsi.o
obj-$(CONFIG_MCDE_DISPLAY_SONY_SY35560_DSI)	+= display-sony_sy35560_dsi.o
obj-$(CONFIG_MCDE_DISPLAY_WS2401_DSI)   	+= display-ws2401_dsi.o
obj-$(CONFIG_MCDE_DISPLAY_WS2401_DPI)   	+= display-ws2401_dpi.o
obj-$(CONFIG_MCDE_DISPLAY_VUIB500_DPI)		+= display-vuib500-dpi.o
obj-$(CONFIG_MCDE_DISPLAY_S6E63MN0_DPI)		+= display-s6e63m0.o
obj-$(CONFIG_MCDE_DISPLAY_LD9040_DPI)		+= display-ld9040.o
obj-$(CONFIG_MCDE_DISPLAY_GODIN_DPI)		+= display-godin.o
obj-$(CONFIG_MCDE_DISPLAY_I9060_DPI)		+= display-i9060-dpi.o
obj-$(CONFIG_MCDE_DISPLAY_AB8500_DENC)		+= display-ab8500.o
obj-$(CONFIG_MCDE_DISPLAY_AV8100)		+= display-av8100.o
obj-$(CONFIG_MCDE_DISPLAY_GAVINI_DPI)		+= display-gavini.o
obj-$(CONFIG_MACH_JANICE)		+= smart_mtp_s6e63m0.o

ifdef CONFIG_FB_MCDE_DEBUG
EXTRA_CFLAGS += -DDEBUG
endif
ifdef CONFIG_FB_MCDE_VDEBUG
EXTRA_CFLAGS += -DVERBOSE_DEBUG
endif
