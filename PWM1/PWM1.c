/*
 * PWM1.c
 *
 * Created: 29/04/2024 16:38:15
 *  Author: adria
 */ 


#include "PWM1.h"
#include <avr/interrupt.h>

void initPWM1FastA(uint8_t inverted, uint16_t prescaler){
	// Configuración del pin PB1 como salida (OC1A)
	DDRB |= (1 << DDB1);
	
	ADCSRA |= (1 << ADSC); // Iniciar la secuencia del ADC
	
	if(inverted){
		// Configurando OC1A como invertido
		TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
		
		}else{
		// Configurando OC1A como no invertido
		TCCR1A |= (1 << COM1A1);
		
	}
	
	// Configurando Modo PWM FAST TOP = ICR1
	TCCR1B |= (1 << WGM12) | (1 << WGM13);
	TCCR1A |= (1 << WGM11);
	
	// Configurar ICR1 39999 para 20ms
	ICR1H = 0x9C;
	ICR1L = 0x3F;
	
	if(prescaler == 8){
		// Prescaler de 8
		TCCR1B |= (1 << CS11);

	}
}

void initPWM1FastB(uint8_t inverted, uint16_t prescaler){
	// Configuración del pin PB1 como salida (OC1B)
	DDRB |= (1 << DDB2);
	
	ADCSRA |= (1 << ADSC); // Iniciar la secuencia del ADC
	
	if(inverted){
		// Configurando OC1A como invertido
		TCCR1A |= (1 << COM1B1) | (1 << COM1B0);
		
		}else{
		// Configurando OC1A como no invertido
		TCCR1A |= (1 << COM1B1);
		
	}
	
	// Configurando Modo PWM FAST TOP = ICR1
	TCCR1B |= (1 << WGM12) | (1 << WGM13);
	TCCR1A |= (1 << WGM11);
	
	// Configurar ICR1 39999 para 20ms
	ICR1H = 0x9C;
	ICR1L = 0x3F;
	
	if(prescaler == 8){
		// Prescaler de 8
		TCCR1B |= (1 << CS11);

	}
}

void updateDutyCycle1(uint16_t duty){
	uint16_t valor_adc;

	valor_adc = ((duty*14.12) + 1198);
	
	ADCSRA |= (1 << ADSC); // Iniciar la secuencia del ADC
	OCR1AH = ((valor_adc & 0xFF00) >> 8);
	OCR1AL = (duty & 0x00FF);

}

void updateDutyCycle2(uint16_t duty){
	uint16_t valor_adc;

	valor_adc = ((duty*14.12) + 1198);
	
	ADCSRA |= (1 << ADSC); // Iniciar la secuencia del ADC
	OCR1BH = ((valor_adc & 0xFF00) >> 8);
	OCR1BL = (duty & 0x00FF);

}