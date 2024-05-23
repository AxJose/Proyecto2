/*
 * PWM1.h
 *
 * Created: 29/04/2024 16:38:25
 *  Author: adria
 */ 


#ifndef PWM1_H_
#define PWM1_H_

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define invertido 1
#define no_invertido 0

// Función para configurar PWM1 Modo Fast Canal A
void initPWM1FastA(uint8_t inverted, uint16_t prescaler);

void initPWM1FastB(uint8_t inverted, uint16_t prescaler);

void updateDutyCycle1(uint16_t duty);

void updateDutyCycle2(uint16_t duty);

#endif /* PWM1_H_ */