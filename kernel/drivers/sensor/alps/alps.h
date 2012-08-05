#ifndef ___ALPS_H_INCLUDED
#define ___ALPS_H_INCLUDED

#define I2C_RETRY_DELAY	5
#define I2C_RETRIES		5

#define ON		1
#define OFF		0

int sensors_register(struct device *dev, void * drvdata,
		struct device_attribute *attributes[], char *name);

int accsns_get_acceleration_data(int *xyz);
int accsns_activate(int flgatm, int flg, int dtime);

int hscd_get_magnetic_field_data(int *xyz);
int hscd_activate(int flgatm, int flg, int dtime);

#endif
