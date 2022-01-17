//
// Created by Linus on 18.04.21.
//

#ifndef COFFEE_CTRL_PID_H
#define COFFEE_CTRL_PID_H

#include "main.h"

#define PID_CALL 1000

#define PID_BOILER_INIT() \
  { \
    .kp = 1400,       \
    .ki = 0,       \
    .kd = 500,       \
    .target = 90,   \
    .temperature = 0,    \
    .prevTime = 0,  \
    .currTime = 0, \
    .error = 0,    \
    .currError = 0,\
    .p = 0,        \
    .i = 0,        \
    .d = 0,        \
   }

#define PID_STEAMER_INIT() \
  { \
    .kp = 1400,       \
    .ki = 0,       \
    .kd = 500,       \
    .target = 120,   \
    .temperature = 0,    \
    .prevTime = 0,  \
    .currTime = 0, \
    .error = 0,    \
    .currError = 0,\
    .p = 0,        \
    .i = 0,        \
    .d = 0,        \
    .pwr = 0,        \
   }


struct PID_dev {
    float kp;
    float ki;
    float kd;
    float target;
    float temperature;
    float prevTime;
    float currTime;
    float error;
    float currError;
    float p;
    float i;
    float d;
    uint16_t pwr;
};

extern void set(struct PID_dev * pid, float target);
extern float compute(struct PID_dev * pid, float temperature);



#endif //COFFEE_CTRL_PID_H
