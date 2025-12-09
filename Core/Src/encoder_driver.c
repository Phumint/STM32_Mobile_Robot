/*
 * encoder_driver.c
 *
 *  Created on: Dec 9, 2025
 *      Author: ASUS
 */

#include "encoder_driver.h"
#include "tim.h"

static volatile int32_t global_encoder_count = 0;
static volatile uint16_t prev_counter_val = 0;

void Encoder_Init(void) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    prev_counter_val = __HAL_TIM_GET_COUNTER(&htim2);
}

void Encoder_Update(void) {
    uint16_t curr_counter = __HAL_TIM_GET_COUNTER(&htim2);
    int16_t diff = (int16_t)(curr_counter - prev_counter_val);

    // Overflow logic handled implicitly by int16_t cast if timer is 16-bit
    global_encoder_count += diff;
    prev_counter_val = curr_counter;
}

int32_t Encoder_GetCount(void) {
    return global_encoder_count;
}
