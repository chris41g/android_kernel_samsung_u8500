
#ifndef __ST_MMIO_H__
#define __ST_MMIO_H__

extern struct class *sec_class;
extern struct class *camera_class;

int subPMIC_module_init(void);
void subPMIC_module_exit(void);
int subPMIC_PowerOn(int opt);
int subPMIC_PowerOff(int opt);
int subPMIC_PinOnOff(int pin, int on_off);

void check_VT_CAM_ID(int pin);




#endif
