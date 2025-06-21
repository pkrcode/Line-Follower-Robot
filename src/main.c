/*
 * LineFollower.c
 *
 * Created: 26-03-2025 14:20:53
 * Author: Praveen_Kumar
 */
#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>  // Needed for cli() and sei()
#include <stdio.h>
#include "lcd.h"

#define LEFT_SENSOR_CHANNEL   3  // PF3 (ADC3)
#define CENTER_SENSOR_CHANNEL 2  // PF2 (ADC2)
#define RIGHT_SENSOR_CHANNEL  1  // PF1 (ADC1)
#define SHARP_SENSOR_CHANNEL 11 // PK3 (ADC11)

#define OBSTACLE_THRESHOLD 150   // Adjust based on Sharp IR readings
#define LINE_THRESHOLD     50    // Adjust based on white line sensor readings

// Motion directions
void motion_set(uint8_t direction);
void forward(void);
void left(void);
void right(void);
void stop(void);
void velocity(uint8_t left_speed, uint8_t right_speed);
unsigned int Sharp_GP2D120_estimation(unsigned char adc_value);

// Init
void init_devices(void);
void adc_init(void);
void motors_init(void);

// ADC
unsigned char ADC_Conversion(unsigned char Ch);

int main(void) {
	lcd_port_config();
	lcd_set_4bit();
	lcd_init();
	adc_init();
	uint16_t left_sensor, center_sensor, right_sensor, obstacle;

	init_devices();

	while (1) {
		// Clear LCD before updating new values
		lcd_clear();

		// Read sensor values
		unsigned char adc_value = ADC_Conversion(11);
		unsigned int distance_mm = Sharp_GP2D120_estimation(adc_value);
		left_sensor = ADC_Conversion(LEFT_SENSOR_CHANNEL);
		center_sensor = ADC_Conversion(CENTER_SENSOR_CHANNEL);
		right_sensor = ADC_Conversion(RIGHT_SENSOR_CHANNEL);
		obstacle = ADC_Conversion(SHARP_SENSOR_CHANNEL);

		// Display distance to obstacle on LCD (Row 1, Column 1)
		lcd_string(1, 1, "Dist:");
		lcd_numeric_value(1, 7, distance_mm, 3); // Display distance in mm (up to 3 digits)
		lcd_string(1, 11, "mm");

		// Display sensor readings on LCD (Row 2, Column 1)
		lcd_string(2, 1, "L:");
		lcd_numeric_value(2, 3, left_sensor, 3);
		lcd_string(2, 7, "C:");
		lcd_numeric_value(2, 9, center_sensor, 3);
		lcd_string(2, 13, "R:");
		lcd_numeric_value(2, 15, right_sensor, 3);

		if (distance_mm <= 150) {
			stop();
			// Display obstacle detected message
			lcd_string(1, 13, "OBS!");
			continue;
		}

		// White line following logic
		if (center_sensor < LINE_THRESHOLD && left_sensor >= LINE_THRESHOLD && right_sensor >= LINE_THRESHOLD) {
			forward();
			lcd_string(2, 1, "FWD "); // Display "Forward" motion
		}
		else if (left_sensor < LINE_THRESHOLD) {
			left();
			lcd_string(2, 1, "LEFT"); // Display "Left" motion
		}
		else if (right_sensor < LINE_THRESHOLD) {
			right();
			lcd_string(2, 1, "RGHT"); // Display "Right" motion
		}
		else {
			// All sensors are off the line ? try to reacquire line
			left();           // Try turning left to search for the line
			lcd_string(2, 1, "SRCH"); // Display "Searching" motion
			_delay_ms(300);   // Give some time to rotate and check again
			stop();
			lcd_string(2, 1, "STOP"); // Display "Stop" motion
		}

		// Small delay to ensure LCD updates are visible
		_delay_ms(100);
	}
}

void init_devices(void) {
	cli();         // Disable interrupts
	adc_init();    // Initialize ADC
	motors_init(); // Initialize motion ports
	sei();         // Enable interrupts
}

void adc_init(void) {
	ADCSRA = 0x00; //clear status register a
	ADCSRB = 0x00; //clear status register b and mux5 = 0
	ADMUX = 0x20; //Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86; //ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

unsigned char ADC_Conversion(unsigned char Ch) {
	unsigned char a;
	if (Ch > 7) {
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX = 0x20 | Ch;
	ADCSRA |= 0x40;
	while ((ADCSRA & 0x10) == 0);
	a = ADCH;
	ADCSRA |= 0x10;
	ADCSRB = 0x00;
	return a;
}

void motors_init(void) {
	DDRL |= 0x38; // PL3, PL4, PL5 as output (PWM)
	DDRA |= 0x0F; // PA0-PA3 as output (direction control)
	DDRB |= 0x06; // PB1, PB2 as output (PWM if OC1A/OC1B used)

	// Timer5 initialization for PWM (for PL3, PL4)
	TCCR5A = 0xA9;
	TCCR5B = 0x0B;
	OCR5A = 0;  // Left motor speed
	OCR5B = 0;  // Right motor speed
}

void velocity(uint8_t left_speed, uint8_t right_speed) {
	OCR5A = left_speed;
	OCR5B = right_speed;
}

void motion_set(uint8_t direction) {
	PORTA = (PORTA & 0xF0) | (direction & 0x0F);
}

// Motion direction functions
void forward(void) {
	motion_set(0x06); // PA1, PA2 high
	velocity(150, 150);
}

void left(void) {
	motion_set(0x05); // PA0, PA2 high
	velocity(100, 150);
}

void right(void) {
	motion_set(0x0A); // PA1, PA3 high
	velocity(150, 100);
}

void stop(void) {
	motion_set(0x00);
	velocity(0, 0);
}

unsigned int Sharp_GP2D120_estimation(unsigned char adc_value) {
	float distance_mm;

	if (adc_value < 10) return 800;

	distance_mm = 10.00 * (2799.6 * (1.00 / pow((double)adc_value, 1.1546)));
	if (distance_mm > 800) distance_mm = 800;
	if (distance_mm < 30) distance_mm = 30;

	return (unsigned int) distance_mm;
}