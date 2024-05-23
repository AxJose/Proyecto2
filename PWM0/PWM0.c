/*
 * PWM0.c
 *
 * Created: 29/04/2024 16:48:15
 *  Author: adria
 */ 

#include "PWM0.h"
#include <avr/interrupt.h>

uint8_t x;
uint8_t in_min;
uint8_t in_max;
uint8_t out_min;
uint8_t out_max;



long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void initPWM0FastA(uint8_t inverted, uint16_t prescaler){
	DDRD |= (1 << DDD6);
	
	if(inverted){
		// Configurando OC0B como invertido
		TCCR0A |= (1 << COM0A1) | (1 << COM0A0);
		
		}else{
		// Configurando OC0B como no invertido
		TCCR0A |= (1 << COM0A1);
	}
	
	// Configurando Modo PWM FAST TOP = 0xFF
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	
	if (prescaler == 1024){
		// Prescaler de 1024
		TCCR0B |= (1 << CS02) | (1 << CS00);
	}
}
void initPWM0FastB(uint8_t inverted, uint16_t prescaler){
	// Configuración del pin PD5 como salida (OC0B)
	DDRD |= (1 << DDD5);

	
	if(inverted){
		// Configurando OC0B como invertido
		TCCR0A |= (1 << COM0B1) | (1 << COM0B0);
		
		}else{
		// Configurando OC0B como no invertido
		TCCR0A |= (1 << COM0B1);
	}
	
	// Configurando Modo PWM FAST TOP = 0xFF
	TCCR0A |= (1 << WGM01) | (1 << WGM00);
	
	if (prescaler == 1024){
		// Prescaler de 1024
		TCCR0B |= (1 << CS02) | (1 << CS00);
	}
}

void updateDutyCycleA(uint8_t duty){
	
	uint8_t dutyA_map = map(duty, 0, 255, 7, 20);
	
	OCR0A = dutyA_map;
	
}

void updateDutyCycleB(uint8_t duty){
	
	uint8_t dutyB_map = map(duty, 0, 255, 7, 20);	
	
	OCR0B = dutyB_map;
}
	
