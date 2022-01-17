/*
 * buzz.c
 *
 *  Created on: Jan 2, 2022
 *      Author: linus
 */

#include "tim.h"
#include "buzz.h"
#include "cmsis_os.h"

void play_tone(uint16_t freq, uint16_t length){
	TIM2->CCR4 = TIM_CLK / (freq * (TIM2->PSC+1) * 2);
	TIM2->ARR = TIM_CLK / (freq * (TIM2->PSC+1)) - 1;
    osDelay(length);
	TIM2->CCR4 = 0;
}
