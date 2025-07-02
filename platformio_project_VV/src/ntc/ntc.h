#ifndef NTC_H
#define NTC_H

#include "configuration.h"

void ntc_init();
float read_ntc_temperature(NTC_devices_t device);

#endif // NTC_H