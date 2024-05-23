/*
 * PWM0.h
 *
 * Created: 29/04/2024 16:48:04
 *  Author: adria
 */ 


#ifndef PWM0_H_
#define PWM0_H_

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define invertido 1
#define no_invertido 0

// Función para configurar PWM0 Modo Fast Canal A
void initPWM0FastA(uint8_t inverted, uint16_t prescaler);

// Función para configurar PWM0 Modo Fast Canal B
void initPWM0FastB(uint8_t inverted, uint16_t prescaler);

void updateDutyCycleA(uint8_t duty2);

void updateDutyCycleB(uint8_t duty2);

#endif /* PWM0_H_ */