/*
 * encoder_driver.h
 *
 *  Created on: Dec 9, 2025
 *      Author: ASUS
 */

/* Core/Inc/encoder_driver.h */
#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>

/**
 * @brief Start the encoder timer in encoder mode.
 */
void Encoder_Init(void);

/**
 * @brief Update the internal software counter.
 * MUST be called periodically (e.g., in a timer interrupt)
 * to handle hardware timer overflows.
 */
void Encoder_Update(void);

/**
 * @brief Get the total accumulated encoder count.
 * @return int32_t: Total ticks (positive or negative).
 */
int32_t Encoder_GetCount(void);

#endif /* ENCODER_DRIVER_H */
