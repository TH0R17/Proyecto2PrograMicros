/*
 * PROYECTO#2
 *
 * Autor: Nesthor Guillermo
 * Descripción: Brazo robótico por medio de servomotores
 * y potenciometros teniendo modos manual, eeprom, uart.
 */ 
#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>

volatile uint16_t adc4_value = 0;
volatile uint16_t adc5_value = 0;
volatile uint16_t adc6_value = 0;
volatile uint16_t adc7_value = 0;
volatile uint8_t current_channel = 4;

#define SERVO_MIN 800
#define SERVO_MAX 3200
#define SERVO_CENTER 1600

#define MAIN_MENU 0
#define UART_SERVO_SELECT 1
#define UART_ANGLE_INPUT 2

#define UART_MODE 0
#define POT_MODE 1
#define EEPROM_MODE 2

#define EEPROM_ADDR_SERVO1 0
#define EEPROM_ADDR_SERVO2 1
#define EEPROM_ADDR_SERVO3 2

volatile char receivedChar = 0;
volatile uint8_t servo_positions[3] = {90, 90, 90};
volatile uint8_t servo4_position = 90;
volatile uint8_t menu_state = MAIN_MENU;
volatile uint8_t selected_servo = 0;
volatile char angle_input[4] = {0};
volatile uint8_t input_index = 0;
volatile uint8_t current_mode = UART_MODE;
volatile uint8_t button_pressed = 0;

void setup();
void initUART();
void writeChar(char c);
void initADC();
void initTimer0Servo();
void initTimer1Servo();
void initTimer2Servo();
void printString(const char *s);
void printNumber(uint16_t num);
void setServoPosition(uint8_t servo_num, uint8_t angle);
void showMainMenu();
void SeleccionServoMenu();
void procesoEntradaAngulo();
void updateLEDs();
void checkButtons();
void guardado_EEPROM();
void cargado_EEPROM();

unsigned char lectura_EEPROM(unsigned int uiAddress) {
	while (EECR & (1 << EEPE));
	EEAR = uiAddress;
	EECR |= (1 << EERE);
	return EEDR;
}

void escritura_EEPROM(unsigned int uiAddress, unsigned char ucData) {
	while (EECR & (1 << EEPE));
	EEAR = uiAddress;
	EEDR = ucData;
	EECR |= (1 << EEMPE);
	EECR |= (1 << EEPE);
}

int main(void) {
	setup();
	
	while (1) {
		checkButtons();
		
		if(current_mode == POT_MODE) {
			uint8_t new_pos1 = 25 + (adc6_value / 3.69);
			uint8_t new_pos2 = 7 + (adc7_value / 3.59);
			uint8_t new_pos3 = 112 + (adc5_value / 12.14);
			uint8_t new_pos4 = adc4_value / 1.42;
			
			if(abs(new_pos1 - servo_positions[0]) > 2) {
				servo_positions[0] = new_pos1;
				setServoPosition(1, servo_positions[0]);
			}
			if(abs(new_pos2 - servo_positions[1]) > 2) {
				servo_positions[1] = new_pos2;
				setServoPosition(2, servo_positions[1]);
			}
			if(abs(new_pos3 - servo_positions[2]) > 2) {
				servo_positions[2] = new_pos3;
				setServoPosition(3, servo_positions[2]);
			}
			if(abs(new_pos4 - servo4_position) > 2) {
				servo4_position = new_pos4;
				setServoPosition(4, servo4_position);
			}
		}
		
		_delay_ms(20);
	}
}

void setup() {
	cli();
	DDRC &= ~((1 << PC4) | (1 << PC5) | (1 << PC6) | (1 << 7));
	PORTC &= ~((1 << PC4) | (1 << PC5) | (1 << PC6) | (1 << 7));

	DDRD &= ~((1 << PD3) | (1 << PD4) | (1 << PD5));
	PORTD |= (1 << PD3) | (1 << PD4) | (1 << PD5);

	DDRD |= (1 << PD7);
	PORTD &= ~(1 << PD7);
	DDRB |= (1 << PB5);
	PORTB &= ~(1 << PB5);
	DDRB |= (1 << PB4);
	PORTB &= ~(1 << PB4);

	initTimer0Servo();
	initTimer1Servo();
	initTimer2Servo();
	initADC();
	initUART();
	
	sei();
}

void initTimer0Servo() {
	DDRD |= (1 << 6);
	
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	TCCR0B = (1 << CS01);
	
	OCR0A = SERVO_CENTER / 8;
}

void initTimer1Servo() {
	DDRB |= (1 << PB1) | (1 << PB2);
	
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
	TCCR1B = (1 << WGM13) | (1 << CS11);
	
	ICR1 = 39999;
	
	OCR1A = SERVO_CENTER;
	OCR1B = SERVO_CENTER;
}

void initTimer2Servo() {
	DDRB |= (1 << PB3);
	
	TCCR2A = (1 << COM2A1) | (1 << WGM20);
	TCCR2B = (1 << CS22) | (1 << CS21);
	
	OCR2A = SERVO_CENTER / 8;
}

void initADC() {
	ADMUX = (1 << REFS0) | (1 << ADLAR) | (1 << MUX2);
	ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) |
	(1 << ADEN) | (1 << ADIE);
	ADCSRA |= (1 << ADSC);
}

void initUART() {
	DDRD |= (1 << DDD1);
	DDRD &= ~(1 << DDD0);
	
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	UBRR0 = 103;
}

void setServoPosition(uint8_t servo_num, uint8_t angle) {
	switch(servo_num) {
		case 1:
		if(angle < 25) angle = 25;
		if(angle > 120) angle = 120;
		OCR1A = SERVO_MIN + (angle * 13.33);
		servo_positions[0] = angle;
		break;
		case 2:
		if(angle < 7) angle = 7;
		if(angle > 78) angle = 78;
		OCR1B = SERVO_MIN + (angle * 13.33);
		servo_positions[1] = angle;
		break;
		case 3:
		if(angle < 112) angle = 112;
		if(angle > 133) angle = 133;
		OCR2A = (SERVO_MIN + (angle * 13.33)) / 8;
		servo_positions[2] = angle;
		break;
		case 4:
		if(angle > 180) angle = 180;
		OCR0A = (SERVO_MIN + (angle * 13.33)) / 8;
		servo4_position = angle;
		break;
	}
}

void updateLEDs() {
	PORTD &= ~(1 << PD7);
	PORTB &= ~((1 << PB4) | (1 << PB5));
	
	switch(current_mode) {
		case UART_MODE: PORTD |= (1 << PD7); break;
		case POT_MODE: PORTB |= (1 << PB4); break;
		case EEPROM_MODE: PORTB |= (1 << PB5); break;
	}
}

void checkButtons() {
	static uint8_t ultimo_estado_boton = 1;
	static uint8_t ultimo_guardado_boton = 1;
	static uint8_t ultimo_reproduccion_boton = 1;
	
	uint8_t current_mode_button_state = PIND & (1 << PD5);
	uint8_t current_save_button_state = PIND & (1 << PD4);
	uint8_t current_play_button_state = PIND & (1 << PD3);
	
	if(ultimo_estado_boton && !current_mode_button_state) {
		_delay_ms(20);
		if(!(PIND & (1 << PD5))) {
			current_mode = (current_mode + 1) % 3;
			updateLEDs();
			
			if(current_mode == UART_MODE) {
				printString("\nModo UART activado");
				menu_state = MAIN_MENU;
				showMainMenu();
				} else if(current_mode == POT_MODE) {
				printString("\nModo Potenciometro activado");
				} else {
				printString("\nModo EEPROM activado");
			}
		}
	}
	ultimo_estado_boton = current_mode_button_state;
	
	if(current_mode == EEPROM_MODE && ultimo_guardado_boton && !current_save_button_state) {
		_delay_ms(20);
		if(!(PIND & (1 << PD4))) {
			guardado_EEPROM();
			printString("\nPosiciones guardadas en EEPROM");
		}
	}
	ultimo_guardado_boton = current_save_button_state;
	
	if(current_mode == EEPROM_MODE && ultimo_reproduccion_boton && !current_play_button_state) {
		_delay_ms(20);
		if(!(PIND & (1 << PD3))) {
			cargado_EEPROM();
			printString("\nPosiciones cargadas desde EEPROM");
		}
	}
	ultimo_reproduccion_boton = current_play_button_state;
}

ISR(ADC_vect) {
	switch(current_channel) {
		case 4:
		adc4_value = ADCH;
		ADMUX = (ADMUX & 0xF0) | 0x05;
		current_channel = 5;
		break;
		case 5:
		adc5_value = ADCH;
		ADMUX = (ADMUX & 0xF0) | 0x06;
		current_channel = 6;
		break;
		case 6:
		adc6_value = ADCH;
		ADMUX = (ADMUX & 0xF0) | 0x07;
		current_channel = 7;
		break;
		case 7:
		adc7_value = ADCH;
		ADMUX = (ADMUX & 0xF0) | 0x04;
		current_channel = 4;
		break;
	}
	ADCSRA |= (1 << ADSC);
}

ISR(USART_RX_vect) {
	if(current_mode == UART_MODE) {
		receivedChar = UDR0;
		writeChar(receivedChar);
		
		switch(menu_state) {
			case MAIN_MENU:
			switch(receivedChar) {
				case '1':
				menu_state = UART_SERVO_SELECT;
				SeleccionServoMenu();
				break;
				case '8':
				printString("\nValores actuales:");
				printString("\nServo 1: ");
				printNumber(servo_positions[0]);
				printString("° (PC6: ");
				printNumber(adc6_value);
				printString(")");
				printString("\nServo 2: ");
				printNumber(servo_positions[1]);
				printString("° (PC7: ");
				printNumber(adc7_value);
				printString(")");
				printString("\nServo 3: ");
				printNumber(servo_positions[2]);
				printString("° (PC5: ");
				printNumber(adc5_value);
				printString(")");
				printString("\nServo 4: ");
				printNumber(servo4_position);
				printString("° (PC4: ");
				printNumber(adc4_value);
				printString(")");
				showMainMenu();
				break;
				default:
				printString("\nOpcion no valida");
				showMainMenu();
				break;
			}
			break;
			case UART_SERVO_SELECT:
			if(receivedChar >= '1' && receivedChar <= '4') {
				selected_servo = receivedChar - '0';
				menu_state = UART_ANGLE_INPUT;
				printString("\nIngrese angulo (0-180) para Servo ");
				printNumber(selected_servo);
				printString(": ");
				angle_input[0] = '\0';
				input_index = 0;
				} else if(receivedChar == '0') {
				menu_state = MAIN_MENU;
				showMainMenu();
				} else {
				printString("\nOpcion no valida");
				SeleccionServoMenu();
			}
			break;
			case UART_ANGLE_INPUT:
			if(receivedChar == '\r' || receivedChar == '\n') {
				if(input_index > 0) {
					procesoEntradaAngulo();
				}
				} else if(receivedChar >= '0' && receivedChar <= '9' && input_index < 3) {
				angle_input[input_index++] = receivedChar;
				angle_input[input_index] = '\0';
				} else if(receivedChar == 8 || receivedChar == 127) {
				if(input_index > 0) {
					angle_input[--input_index] = '\0';
					printString("\b \b");
				}
				} else {
				writeChar('\a');
			}
			break;
		}
	}
}

void writeChar(char c) {
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
}

void printString(const char *s) {
	for (uint8_t i = 0; s[i] != '\0'; i++) {
		writeChar(s[i]);
	}
}

void printNumber(uint16_t num) {
	char buffer[5];
	itoa(num, buffer, 10);
	printString(buffer);
}

void showMainMenu() {
	printString("\n\n=== MENU PRINCIPAL ===");
	printString("\n1. Controlar servos por UART");
	printString("\n8. Mostrar valores actuales");
	printString("\nSeleccione una opcion: ");
}

void SeleccionServoMenu() {
	printString("\n\n=== CONTROL POR UART ===");
	printString("\nSeleccione el servo a controlar:");
	printString("\n1. Servo 1 (Actual: ");
	printNumber(servo_positions[0]);
	printString("°)");
	printString("\n2. Servo 2 (Actual: ");
	printNumber(servo_positions[1]);
	printString("°)");
	printString("\n3. Servo 3 (Actual: ");
	printNumber(servo_positions[2]);
	printString("°)");
	printString("\n4. Servo 4 (Actual: ");
	printNumber(servo4_position);
	printString("°)");
	printString("\n0. Volver al menu principal");
	printString("\nSeleccione una opcion: ");
}

void procesoEntradaAngulo() {
	uint8_t angle = atoi(angle_input);
	
	if(angle <= 180) {
		setServoPosition(selected_servo, angle);
		printString("\nServo ");
		printNumber(selected_servo);
		printString(" movido a ");
		printNumber(angle);
		printString("°");
		} else {
		printString("\nError: Angulo debe ser 0-180");
	}
	
	angle_input[0] = '\0';
	input_index = 0;
	
	menu_state = UART_SERVO_SELECT;
	SeleccionServoMenu();
}

void guardado_EEPROM() {
	escritura_EEPROM(EEPROM_ADDR_SERVO1, servo_positions[0]);
	escritura_EEPROM(EEPROM_ADDR_SERVO2, servo_positions[1]);
	escritura_EEPROM(EEPROM_ADDR_SERVO3, servo_positions[2]);
}

void cargado_EEPROM() {
	servo_positions[0] = lectura_EEPROM(EEPROM_ADDR_SERVO1);
	servo_positions[1] = lectura_EEPROM(EEPROM_ADDR_SERVO2);
	servo_positions[2] = lectura_EEPROM(EEPROM_ADDR_SERVO3);
	
	setServoPosition(1, servo_positions[0]);
	setServoPosition(2, servo_positions[1]);
	setServoPosition(3, servo_positions[2]);
}