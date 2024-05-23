/*
 * main.c
 * Proyecto 2
 * Created: 4/29/2024 4:34:48 PM
 *  Author: Adrián Pascual
 */ 

#define F_CPU 16000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "PWM1/PWM1.h"
#include "PWM0/PWM0.h"

void initADC(void);
void initButtons(void);
void readUART(void);
void initUART9600(void);
void WriteTextUART(const char * Texto);
void writeUART(char character);

// VARIABLES PARA ADC //
static volatile uint8_t bandera = 0;
static volatile uint16_t valor_adc;
static volatile uint16_t valor_adc2;
static volatile uint8_t adc_value1;
static volatile uint8_t adc_value2;
static volatile uint8_t adc_value3;
static volatile uint8_t adc_value4;
static volatile uint8_t adc_value5;
static volatile uint8_t adc_value6;

// VARIABLES PARA EEPROM //
volatile uint8_t memoria;
volatile uint8_t mem_value_1;
volatile uint8_t mem_value_2;
volatile uint8_t mem_value_3;
volatile uint8_t mem_value_4;
volatile uint8_t mem_value_5;
volatile uint8_t mem_value_6;


// VARIABLES PARA SERVOS //
volatile uint8_t posicion_s1;
volatile uint8_t posicion_s2;
volatile uint16_t posicion_s3;
volatile uint16_t posicion_s4;
volatile uint16_t posicion_s5;
volatile uint16_t posicion_s6;

// VARIABLES PARA UART //
volatile uint8_t bufferTx;
volatile uint8_t ascii_value;
volatile uint8_t prueba;

// OTRAS //
volatile uint8_t modo;
volatile uint8_t valor_servo;
volatile uint8_t adafruit = 0;

int CharToInt(char num){
	return num - '0';
}

int unir(int n1, int n2, int n3){
	return (n1*100)+ (n2*10) + (n3);
}

// INTERRUPCIÓN DE LECTURA //
ISR(USART_RX_vect){
	bufferTx = UDR0;  // Leer el dato recibido y almacenarlo en bufferTx
	ascii_value = bufferTx;
	
	if (adafruit == 1){
		if (ascii_value == 48){ // 48 = 0
			modo = 4; // Escribir en la EEPROM
		}else if (ascii_value == 49){
		modo = 3; // Leer la EEPROM
		}else if (ascii_value == 65){
		modo = 2;	// Seleccionar memoria
		memoria = 1;
		}else if (ascii_value == 66){
			modo = 2;	// Seleccionar memoria
			memoria = 2;
		}else if (ascii_value == 67){
		modo = 2;	// Seleccionar memoria
		memoria = 3;
		}else if (ascii_value == 68){
		modo = 2;	// Seleccionar memoria
		memoria = 4;
		}
	}
	
	if (ascii_value == 51){
		adafruit = 0;
		modo = 1;	// MANUAL
		}else if (ascii_value == 50){
		adafruit = 1;
		modo = 5; // ADAFRUIT
	}
	
	bufferTx = 0;  // Resetear bufferTx para la próxima recepción

}

ISR(ADC_vect){
	if (ADMUX == 0x60){
		adc_value1 = ADCH;
		
		}else if (ADMUX == 0x61){
		adc_value2 = ADCH;
		
		}else if (ADMUX == 0x62){
		adc_value3 = ADCH;
		
		}else if (ADMUX == 0x63){
		adc_value4 = ADCH;
	}
	
	// Apagar la bandera
	ADCSRA |= (1 << ADIF);
}

ISR(PCINT0_vect){
	_delay_ms(10);
	if (!(PINB & (1 << PB5))) {
		if (modo <= 1) {
			modo = 2; // Modo para escoger memoria
			
			}else if (modo == 2){
				if (adafruit  ==  1){
					WriteTextUART("READ\n");
				}
			modo = 3; // Pasar de MEMORIA a LEER
			
			}else if (modo == 3){
				if (adafruit  ==  1){
					WriteTextUART("WRITE\n");
				}
			modo = 4; // Pasar de LEER a ESCRIBIR
			
			}else if (modo > 3){
			modo = 2; // Pasar de ESCRIBIR a MEMORIA
		}
	}
}

ISR(PCINT1_vect){
	_delay_ms(10);
	if (!(PINC & (1 << PC5))) {
		// Incrementar al presionar el botón en PC5
		if (modo <= 2) {
			adafruit = 1;
			WriteTextUART("MODO_A\n");
			modo = 5; // Pasar de modo MANUAL a modo UART
		}
		else if (modo >= 3){
			if (adafruit  ==  1){
				WriteTextUART("MODO_M\n");
			}
			adafruit = 0;
			modo = 1; // Pasar de modo UART a modo MANUAL
		}
	}
	if (!(PINC & (1 << PC4))) {
		
		// Decrementar al presionar el botón en PC4
		if (modo == 2) {
			// ESCOGER QUE MEMORIA UTILIZAR
			if (memoria <= 3){
				memoria++;
				}else if (memoria == 4){
				memoria = 1;
			}
			if (adafruit  ==  1){
				if (memoria == 1){
				WriteTextUART("M_1\n");
				}else if (memoria == 2){
				WriteTextUART("M_2\n");
				}else if (memoria == 3){
				WriteTextUART("M_3\n");
				}else if (memoria == 4){
				WriteTextUART("M_4\n");
			}
				}
		}
	}
}

// FUNCIONES //
void initADC(void){
	
	// Referencia AVCC = 5V
	ADMUX |= (1 << REFS0);
	ADMUX &= ~(1 << REFS1);
	
	// Justificación a la izquierda
	ADMUX |= (1 << ADLAR);
	
	ADCSRA = 0;
	// Habilitar la interrupción del ADC
	ADCSRA |= (1 << ADIE);
	
	// Habilitar prescaler de 128, F_ADC = 125kHz
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	// Habilitando el ADC
	ADCSRA |= (1 << ADEN);
}

void initButtons(void){
	cli();
	// Configurar los pines de entrada
	DDRB &= ~(1 << PORTB5); // PB5
	DDRC &= ~((1 << PORTC4) | (1 << PORTC5)); //  PC4 y PC5
	
	PCICR |= (1 << PCIE0); // Habilitar interrupciones para el puerto B
	PCMSK0 |= (1 << PCINT5); // Habilitar las interrupciones externas para PB5
	
	PCICR |= (1 << PCIE1); // Habilitar interrupciones para el puerto C
	PCMSK1 |= (1 << PCINT12) | (1<<PCINT13); // Habilitar las interrupciones externas para PC4 y PC5
	
	sei();
}

uint8_t receivedChar(void){
	while (!(UCSR0A & (1 << RXC0)));
	return UDR0;
}

void writeUART(char character){
	while(!(UCSR0A & (1 << UDRE0))); // Esperar a que el buffer de transmisión esté vacío
	UDR0 = character; // Enviar el caracter
	//PORTB = character;
}

void WriteTextUART(const char * Texto){
	uint8_t i;
	for (i=0; Texto[i] != '\0'; i++){
		while (!(UCSR0A & (1<<UDRE0)) );
		UDR0 = Texto[i];
	}
}

void initUART9600(void)
{
	DDRB = 0xFF;
	
	// Paso 1: Rx como entrada y Tx como salida
	DDRD &= ~(1 << DDD0);
	DDRD |= (1 << DDD1);
	
	//Habilitar PC0 y PC1 como salida (bits 6 y 7)
	DDRC |= (1 << DDC0) | (1 << DDC1);
	
	// Paso 2: Configurar UCSR0A
	UCSR0A = 0;
	
	// Paso 3: Configurar UCSR0B -> Habilitamos ISR de recepción, también rx & tx
	UCSR0B = 0;
	UCSR0B |= (1<<RXCIE0) | (1 << RXEN0) | (1 << TXEN0); // Habilitar transmisión y recepción
	
	// Paso 4: Configurar UCSR0C -> Asyncrono, Paridad: None, 1 bit Stop, Data bit 8/bits
	UCSR0C = 0;
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // Configurar el tamaño de los datos en 8 bits
	
	// Paso 5: Configurar la velocidad de Baudrate: 9600
	UBRR0 = 103;
	
	sei();
}


// MAIN //
int main(void)
{
	cli();
	initADC();
	initUART9600();
	initButtons();
	
	initPWM0FastA(no_invertido, 1024);
	initPWM0FastB(no_invertido, 1024);
	initPWM1FastA(no_invertido, 8);
	initPWM1FastB(no_invertido, 8);
	

	int n1;
	int n2;
	int n3;
	
	int num;
	
	DDRD = 0xFF; // Habilitar puerto D como salida
	
	DDRB = 0b00011111; // Habilitar puerto B como salida
	
	DDRC = 0; // Habilitar puerto C como entrada
	
	sei(); // Habilitar interrupciones globales
	
	while(1){
		// MODO MANUAL //
	if (modo == 1){
		// Apagar LEDS
		PORTB &= ~(1 << PB4);
		PORTB &= ~(1 << PB3);
		PORTD &= ~(1 << PD4);
		PORTD &= ~(1 << PD3);
		PORTD &= ~(1 << PD2);
		
		// PRIMER POTENCIÓMETRO
		if (bandera	== 0){
			// Escoger canal  PINC0 -> ADC0 (Entrada)
			ADMUX = (ADMUX & 0xF0);
			ADCSRA |= (1<<ADSC); // Iniciar la secuencia del ADC
			updateDutyCycleA(adc_value1); // Actualizar ciclo de trabajo 1
			posicion_s1 = map(adc_value1, 0, 255, 7, 20);
			_delay_ms(10);
			bandera++;
		
		// SEGUNDO POTENCIÓMETRO	
			}else if (bandera == 1){
			// Escoger canal  PINC1 -> ADC1 (Entrada)
			ADMUX = (ADMUX & 0xF0) | 1;
			ADCSRA |= (1<<ADSC); // Iniciar la secuencia del ADC
			updateDutyCycleB(adc_value2); // Actualizar ciclo de trabajo 2
			posicion_s2 = map(adc_value2, 0, 255, 7, 20);
			_delay_ms(10);
			bandera++;
		
		// TERCER POTENCIÓMETRO	
			}else if (bandera == 2){
			// Escoger canal  PINC2 -> ADC2 (Entrada)
			ADMUX = (ADMUX & 0xF0) | 2;
			ADCSRA |= (1<<ADSC); // Iniciar la secuencia del ADC
			updateDutyCycle1(adc_value3);
			
			valor_adc = ((adc_value3*14.12) + 1198);
			
			posicion_s3 = ((valor_adc & 0xFF00) >> 8);
			posicion_s4 = (adc_value3 & 0x00FF);
			
			
			_delay_ms(10);
			bandera++;
		
		// CUARTO POTENCIÓMETRO	
			}else if (bandera == 3){
			// Escoger canal  PINC3 -> ADC3 (Entrada)
			ADMUX = (ADMUX & 0xF0) | 3;
			ADCSRA |= (1<<ADSC); // Iniciar la secuencia del ADC
			updateDutyCycle2(adc_value4);
			
			valor_adc2 = ((adc_value4*14.12) + 1198);
			
			posicion_s5 = ((valor_adc2 & 0xFF00) >> 8);
			posicion_s6 = (adc_value4 & 0x00FF);
			
			_delay_ms(10);
			bandera++;
			
		// REINICIAR	
			}else if (bandera == 4){
			bandera = 0;
			ADMUX = (ADMUX & 0xF0); 
		}
		}else if (modo == 5){
		// MODO UART //	ADAFRUIT
			PORTB &= ~(1 << PB4);
			PORTB &= ~(1 << PB3);
			PORTD |= (1 << PD4);
			PORTD &= ~(1 << PD3);
			PORTD &= ~(1 << PD2);
			if (ascii_value == 48){
				PORTB |= (1 << PB4);
				PORTB &= ~(1 << PB3);
			}else if (ascii_value == 49){
				PORTB &= ~(1 << PB4);
				PORTB |= (1 << PB3);
			}else if (ascii_value == 50){
				modo = 5; // ADAFRUIT
			}else if (ascii_value == 51){
			modo = 1;	// MANUAL
		}
		// ESCOGER MEMORIA //
		}else if (modo == 2){
			PORTB &= ~(1 << PB4);
			PORTB |= (1 << PB3);
			PORTD &= ~(1 << PD4);
			if (memoria == 1){
				// encender leds
				PORTD &= ~(1 << PD3);
				PORTD &= ~(1 << PD2);
				mem_value_1 = 0;
				mem_value_2 = 1;
				mem_value_3 = 2;
				mem_value_4 = 3;
				mem_value_5 = 4;
				mem_value_6 = 5;
				
			}else if (memoria == 2){
				//LEDS
				PORTD |= (1 << PD3);
				PORTD &= ~(1 << PD2);
				mem_value_1 = 6;
				mem_value_2 = 7;
				mem_value_3 = 8;
				mem_value_4 = 9;
				mem_value_5 = 10;
				mem_value_6 = 11;
				
			}else if (memoria == 3){
				//LEDS
				PORTD |= (1 << PD2);
				PORTD &= ~(1 << PD3);
				mem_value_1 = 12;
				mem_value_2 = 13;
				mem_value_3 = 14;
				mem_value_4 = 15;
				mem_value_5 = 16;
				mem_value_6 = 17;
				
			}else if (memoria == 4){
				//LEDS
				PORTD |= (1 << PD2);
				PORTD |= (1 << PD3);
				mem_value_1 = 18;
				mem_value_2 = 19;
				mem_value_3 = 20;
				mem_value_4 = 21;
				mem_value_5 = 22;
				mem_value_6 = 23;
			}
		
		// LEER EEPROM //
		}else if (modo == 3){
		PORTB &= ~(1 << PB3);
		PORTB |= (1 << PB4);
		PORTD &= ~(1 << PD4);
		
		OCR0B = eeprom_read_byte((uint8_t*)mem_value_1);
		OCR0A = eeprom_read_byte((uint8_t*)mem_value_2);
		OCR1AH = eeprom_read_byte((uint8_t*)mem_value_3);
		OCR1AL = eeprom_read_byte((uint8_t*)mem_value_4);
		OCR1BH = eeprom_read_byte((uint8_t*)mem_value_5);
		OCR1BL = eeprom_read_byte((uint8_t*)mem_value_6);
	
		
		// MODO EEPROM //	ESCRIBIR	
		}else if (modo == 4){
		PORTB |= (1 << PB4);
		PORTB |= (1 << PB3);
		PORTD &= ~(1 << PD4);
		
		eeprom_write_byte((uint8_t*)mem_value_1, posicion_s1);
		eeprom_write_byte((uint8_t*)mem_value_2, posicion_s2);
		eeprom_write_byte((uint8_t*)mem_value_3, posicion_s3); // TIMER 1A HIGH
		eeprom_write_byte((uint8_t*)mem_value_4, posicion_s4); // TIMER 1A LOW
		eeprom_write_byte((uint8_t*)mem_value_5, posicion_s5); // TIMER 1B HIGH
		eeprom_write_byte((uint8_t*)mem_value_6, posicion_s6); // TIMER 1B LOW
		}
	}
}



