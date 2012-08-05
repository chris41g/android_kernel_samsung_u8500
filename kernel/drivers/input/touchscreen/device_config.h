/**********************************************************

  DEVICE   : mxT224
  CUSTOMER : SAMSUNG
  PROJECT  : N1
  X SIZE   : X18
  Y SIZE   : Y11
  CHRGTIME : 2.59us
  X Pitch  :
  Y Pitch  :
***********************************************************/

#define __MXT224E_CONFIG__



/* SPT_USERDATA_T38 INSTANCE 0 */
#define T7_IDLEACQINT             64
#define T7_ACTVACQINT             255
#define T7_ACTV2IDLETO            20

/* _GEN_ACQUISITIONCONFIG_T8 INSTANCE 0 */
#define T8_CHRGTIME               36      /* 6 - 60  * 83 ns */
#define T8_ATCHDRIFT              0
#define T8_TCHDRIFT               20
#define T8_DRIFTST                20
#define T8_TCHAUTOCAL             0
#define T8_SYNC                   0
#define T8_ATCHCALST              10
#define T8_ATCHCALSTHR            10
#define T8_ATCHFRCCALTHR          50         /* V2.0 추가 */
#define T8_ATCHFRCCALRATIO        25         /* V2.0 추가 */

/* TOUCH_MULTITOUCHSCREEN_T9 INSTANCE 0 */
#define T9_CTRL                   139
#define T9_XORIGIN                0
#define T9_YORIGIN                0
#define T9_XSIZE                  18
#define T9_YSIZE                  11
#define T9_AKSCFG                 0
#define T9_BLEN                   0x10
#define T9_TCHTHR                 40
#define T9_TCHDI                  2
#define T9_ORIENT_NORM            0
#define T9_ORIENT_180             6	/* rotate 180 */
#define T9_MRGTIMEOUT             0
#define T9_MOVHYSTI               3
#define T9_MOVHYSTN               1
#define T9_MOVFILTER              0x00
#define T9_NUMTOUCH               10
#define T9_MRGHYST                10
#define T9_MRGTHR                 10
#define T9_AMPHYST                10
#define T9_XRANGE                 (480-1)
#define T9_YRANGE                 (800-1)
#define T9_XLOCLIP                0
#define T9_XHICLIP                0
#define T9_YLOCLIP                0
#define T9_YHICLIP                0
#define T9_XEDGECTRL              0
#define T9_XEDGEDIST              0
#define T9_YEDGECTRL              0
#define T9_YEDGEDIST              0
#define T9_JUMPLIMIT              10
#define T9_TCHHYST                5       /* V2.0 or MXT224E 추가 */

#define T9_XPITCH                 5       /* MXT224E 추가 */
#define T9_YPITCH                 5       /* MXT224E 추가 */

/* TOUCH_KEYARRAY_T15 */
#define T15_CTRL                  131 /* single key configuration*/ // 3 = multi-key
#define T15_XORIGIN               16
#define T15_YORIGIN               11
#define T15_XSIZE                 2
#define T15_YSIZE                 1
#define T15_AKSCFG                0
#define T15_BLEN                  0
#define T15_TCHTHR                40
#define T15_TCHDI                 3
#define T15_RESERVED_0            0
#define T15_RESERVED_1            0


/* SPT_COMMSCONFIG_T18 */
#define T18_CTRL                  0
#define T18_COMMAND               0



/* SPT_GPIOPWM_T19 INSTANCE 0 */
#define T19_CTRL                  0
#define T19_REPORTMASK            0
#define T19_DIR                   0
#define T19_INTPULLUP             0
#define T19_OUT                   0
#define T19_WAKE                  0
#define T19_PWM                   0
#define T19_PERIOD                0
#define T19_DUTY_0                0
#define T19_DUTY_1                0
#define T19_DUTY_2                0
#define T19_DUTY_3                0
#define T19_TRIGGER_0             0
#define T19_TRIGGER_1             0
#define T19_TRIGGER_2             0
#define T19_TRIGGER_3             0


/* TOUCH_PROXIMITY_T23 */
#define T23_CTRL                  0
#define T23_XORIGIN               0
#define T23_YORIGIN               0
#define T23_XSIZE                 0
#define T23_YSIZE                 0
#define T23_RESERVED              0
#define T23_BLEN                  0
#define T23_FXDDTHR               0
#define T23_FXDDI                 0
#define T23_AVERAGE               0
#define T23_MVNULLRATE            0
#define T23_MVDTHR                0


/* T24_[PROCI_ONETOUCHGESTUREPROCESSOR_T24 INSTANCE 0] */
#define T24_CTRL                  0
#define T24_NUMGEST               0
#define T24_GESTEN                0
#define T24_PROCESS               0
#define T24_TAPTO                 0
#define T24_FLICKTO               0
#define T24_DRAGTO                0
#define T24_SPRESSTO              0
#define T24_LPRESSTO              0
#define T24_REPPRESSTO            0
#define T24_FLICKTHR              0
#define T24_DRAGTHR               0
#define T24_TAPTHR                0
#define T24_THROWTHR              0


/* [SPT_SELFTEST_T25 INSTANCE 0] */
#define T25_CTRL                  0
#define T25_CMD                   0
#define T25_SIGLIM_0_UPSIGLIM     13500
#define T25_SIGLIM_0_LOSIGLIM     5500
#define T25_SIGLIM_1_UPSIGLIM     13500
#define T25_SIGLIM_1_LOSIGLIM     5500
#define T25_SIGLIM_2_UPSIGLIM     0
#define T25_SIGLIM_2_LOSIGLIM     0


/* PROCI_GRIPSUPPRESSION_T40 */

#define T40_CTRL                  0
#define T40_XLOGRIP               0
#define T40_XHIGRIP               0
#define T40_YLOGRIP               0
#define T40_YHIGRIP               0

/* PROCI_TOUCHSUPPRESSION_T42 */
#define T42_CTRL                  0
#define T42_APPRTHR               0   // 0 (TCHTHR/4), 1 to 255
#define T42_MAXAPPRAREA           0   // 0 (40ch), 1 to 255
#define T42_MAXTCHAREA            0   // 0 (35ch), 1 to 255
#define T42_SUPSTRENGTH           0   // 0 (128), 1 to 255
#define T42_SUPEXTTO              0   // 0 (never expires), 1 to 255 (timeout in cycles)
#define T42_MAXNUMTCHS            9   // 0 to 9 (maximum number of touches minus 1)
#define T42_SHAPESTRENGTH         0   // 0 (10), 1 to 31


/* SPT_CTECONFIG_T46  */
#define T46_CTRL                  0  //Reserved
#define T46_MODE                  2  //0: 16X14Y, 1: 17X13Y, 2: 18X12Y, 3: 19X11Y, 4: 20X10Y, 5: 21X15Y, 6: 22X8Y,
#define T46_IDLESYNCSPERX         0
#define T46_ACTVSYNCSPERX         0
#define T46_ADCSPERSYNC           0
#define T46_PULSESPERADC          0  //0:1  1:2   2:3   3:4 pulses
#define T46_XSLEW                 0  //1:500nsec,  0:350nsec
#define T46_SYNCDELAY			  0

/* PROCI_STYLUS_T47 */
#define T47_CTRL                  0
#define T47_CONTMIN               0
#define T47_CONTMAX               0
#define T47_STABILITY             0
#define T47_MAXTCHAREA            0
#define T47_AMPLTHR               0
#define T47_STYSHAPE              0
#define T47_HOVERSUP              0
#define T47_CONFTHR               0
#define T47_SYNCSPERX             0


/* PROCG_NOISESUPPRESSION_T48  */
#define T48_CTRL                  3
#define T48_CFG                   0
#define T48_CALCFG                2
#define T48_BASEFREQ              10
#define T48_FREQ_0                6
#define T48_FREQ_1                12
#define T48_FREQ_2                18
#define T48_FREQ_3                24
#define T48_MFFREQ_2              20
#define T48_MFFREQ_3              30
#define T48_NLGAIN                0
#define T48_NLTHR                 0
#define T48_GCLIMIT               0
#define T48_GCACTVINVLDADCS       0
#define T48_GCIDLEINVLDADCS       0
#define T48_GCINVALIDTHR          0
#define T48_GCMAXADCSPERX         0



/********************* END  *********************/

