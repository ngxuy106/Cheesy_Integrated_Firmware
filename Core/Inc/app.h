#ifndef APP_H_
#define APP_H_

#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "motor_api.h"
#include "vl53l0x.h"
#include "imu.h"
#include "battery.h"
#include "ble_telemetry.h"

void app_init(void);
void app_loop(void);

#endif /* APP_H_ */
