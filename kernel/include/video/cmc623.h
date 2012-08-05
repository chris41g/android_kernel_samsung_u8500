#ifndef __CMC623_H
#define __CMC623_H


#define CMC623_I2C_DEVICE_NAME "image_convertor"

struct Cmc623PlatformData {
	int reset_gpio;		/* GPIO to reset chip */
	int bypass_gpio;	/* GPIO for bypass mode */
	int sleep_gpio;		/* GPIO for Sleep mode */
	int failsafeb_gpio;	/* GPIO for FAIL_SAFEB(shutdown) */
	int backlight_en_gpio;	/* GPIO to turn on backlight */
	int init_in_bypass;	/* start driver in bypass mode */
	void (* powerctrl) (int main, int io);	/* callback for power control */
};


#endif

