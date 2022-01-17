/*
 * ctrl.h
 *
 *  Created on: 16.01.2022
 *      Author: linus
 */

#ifndef INC_CTRL_H_
#define INC_CTRL_H_

#include "main.h"

#define BOILER_INIT() \
  { \
    .heat_pwr = 0,       \
    .state = IDLE,       \
	.temperature = 0,	\
	.active = 1,	\
   }

#define PUMP_INIT() \
  { \
    .motor_v = 0,       \
    .motor_en = 0,       \
    .state = IDLE,       \
   }

#define SYS_INIT() \
  { \
    .state = IDLE,       \
    .steam_button = 0,       \
    .lever_button = 0,       \
    .water_sensor = 0,       \
   }

struct BOILER_dev {
    uint16_t heat_pwr;
    uint8_t state;
    uint8_t active;
    float temperature;
};

struct PUMP_dev {
    uint16_t motor_v;
    uint16_t motor_en;
    uint8_t state;
};

struct SYS_dev {
	uint8_t state;
	uint8_t steam_button;
	uint8_t lever_button;
	uint8_t water_sensor;
};

enum state_t{
	IDLE=1,
	HEATING,
	READY,
	PRE_INFUSE,
	EXTRACTING,
};


#endif /* INC_CTRL_H_ */
