#define IMPROVE_SOURCE // KMJ_DA18

typedef enum
{
    DEFAULT_MODEL,
    ARCHER_KOR_SK,
    TICKER_TAPE,
    OSCAR,
    KESWICK,
    MODEL_TYPE_MAX
} Atmel_model_type;

typedef enum
{
    VERSION_1_2,
    VERSION_1_4,
    VERSION_1_5,
} Atmel_fw_version_type;


#define LONG(x) ((x)/BITS_PER_LONG)
// ]] ryun

/* firmware 2009.09.24 CHJ - start 1/2 */
#define QT602240_I2C_BOOT_ADDR 0x24
#define QT_WAITING_BOOTLOAD_COMMAND 0xC0
#define QT_WAITING_FRAME_DATA       0x80
#define QT_FRAME_CRC_CHECK          0x02
#define QT_FRAME_CRC_PASS           0x04
#define QT_FRAME_CRC_FAIL           0x03

#define WRITE_MEM_OK                1u
#define WRITE_MEM_FAILED            2u
#define READ_MEM_OK                 1u
#define READ_MEM_FAILED             2u

/* firmware 2009.09.24 CHJ - end 1/2 */

#if defined( CONFIG_ARCHER_TARGET_SK )  || defined ( CONFIG_MACH_KESWICK )

// This feature is about making qt chip enter sleep mode by gating power of Driver IC
// , not sending I2C sleep command.
#if defined(CONFIG_MACH_KESWICK)
#define FEATURE_SUSPEND_BY_DISABLING_POWER
#endif


//#define FEATURE_LOGGING_TOUCH_EVENT


#define FEATURE_TSP_FOR_TA


#ifndef CONFIG_MACH_KESWICK
#define FEATURE_DYNAMIC_SLEEP
#endif
/*
* Requires that sys/class/power_supply/battery/batt_temp file
* is present. Disabled for now.
#define FEATURE_CALIBRATE_BY_TEMP
*/

#define FEATURE_CAL_BY_INCORRECT_PRESS


#define FEATURE_VERIFY_INCORRECT_PRESS

#ifdef FEATURE_VERIFY_INCORRECT_PRESS
#define CHECK_COUNT_FOR_NORMAL 4
#define CHECK_COUNT_FOR_CALL 3
#endif
#endif

struct touchscreen_t;

struct touchscreen_t {
	struct input_dev * inputdevice;
	struct timer_list ts_timer;
	int touched;
	int irq;
	int irq_type;
	int irq_enabled;
	struct ts_device *dev;
	spinlock_t lock;
	struct early_suspend	early_suspend;// ryun 20200107 for early suspend
	struct work_struct  tsp_work;	// ryun 20100107 
	struct timer_list opp_set_timer;	// ryun 20100107 for touch boost
	struct work_struct constraint_wq;
	int opp_high;	// ryun 20100107 for touch boost	
};

#define TOUCHSCREEN_NAME			"touchscreen"

/*
* init				- Function to configure board's GPIOs and POWER ON, return 0 for success.
* exit				- Function to undo init, returns 0 for success.
* powerOn			- Function to Power on or off touch screen, return 0 for success.
* irq					- irq number.
* irq_type			- IRQ rising, falling, or level.
* x_max_res			- max screen resolution.
* y_max_res			-
* touch_x_max		- max touch screen resolution
* touch_y_max		-
* key_start_x		- Start location of Touch Keys on touch screen.
* key_start_y
* key_end_x 		- End location of Touch Keys on touch screen.
* key_end_y
* menu_start_x		- Each Touch Key's start location.
* search_start_x
* home_start_x
* back_start_x
*/
struct atmel_platform_data
{
	int (* init) (struct touchscreen_t *);
	int (* exit) (void);
	int (* powerOn) ( int on);
	int irq;
	int irq_type;
	int x_max_res;
	int y_max_res;
	int touch_x_max;
	int touch_y_max;
	int key_start_x;
	int key_start_y;
	int key_end_x; 
	int key_end_y;
	int menu_start_x;
	int search_start_x;
	int home_start_x;
	int back_start_x;
};

