/*
 * main.c
 *
 * Created: 5/6/2024 12:05:16 PM
 *  Author: apm3
 */ 

// Based on https://onlinedocs.microchip.com/pr/GUID-F897CF19-8EAC-457A-BE11-86BDAC9B59CF-en-US-10/index.html?GUID-A6CB54F0-041D-4B12-A3E1-97602C36ED7B for DataStream Protocol.

#include <xc.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 16000000U
#define BAUD 57600								 // define baud
#define BAUDRATE (((F_CPU / (BAUD * 16UL))) - 1) // set baud rate
#define MTR_Pin PORTB1
#define kp_val 0;
#define kr_val 160000;	// 1*10^6 to acount for us		

uint8_t initADCMult(uint16_t sampleRate, uint8_t sampleTime, uint8_t pinNumbers[], uint8_t numPins);
void initPWMControl(void);

uint8_t TC0_BUFF;					// Buffer value for TC0 to get correct period
volatile uint16_t adc_value;		// variable to hold read adc value
volatile uint16_t ref_value_prev;		// variable to hold read adc value for ref voltage from previous cycle
volatile uint16_t adc_value_arr[2];		// variable to hold read adc values
volatile uint16_t control_value;

volatile bool NEW_VALUE;

uint8_t sample_rate = 122; //122 Hz
uint8_t sample_time	= 61; //200 us
uint8_t NUM_ADC_PINS = 2;
uint8_t ADC_PINS[2] = {0, 1}; //0 meas, 1 ref

int main(void)
{
	if (initADCMult(sample_rate, sample_time, ADC_PINS, NUM_ADC_PINS) == 1) {
		 exit(1);
	}
	
	initPWMControl(void);

	// Global enable interrupts
	sei();
	
    while(1)
    {
        if (NEW_VALUE) {
			control_value = controlAlg(kp_val , kr_val);
			
			ref_value_prev = adc_value_arr[1];
			
			PINB ^= (1<<PINB5); // Toggle onboard LED
			
			NEW_VALUE = false;
		}
    }
}

/*
 *	initADC: configures to read from the selected pin at the specified sampleRate and sampleTime
 *  using Timer0 to control the sampling rate.
 *		- sampleRate: target sample rate in Hz (uint8_t)
 *		- sampleTime: target sampleTime in us (uint8_t)
 *		- pinNumber: ADC pin number (A0-A5)
 *	Returns 1 on error
 */
uint8_t initADCMult(uint16_t sampleRate, uint8_t sampleTime, uint8_t pinNumbers[], uint8_t numPins) {
	
	// Turn on the ADC by disabling power reduction
	PRR0 &= ~(1<<PRADC);
	
	for (uint8_t i; i<numPins; i++) {
		uint8_t pinNumber = pinNumbers[i];
		// Set relevant pins as input
		// Check if pinNumber is valid
		if (pinNumber > 5){
			return 1;
		}
		DDRC &= ~(1 << pinNumber);		// Set pin as input
		PORTC &= ~(1 << pinNumber);		// Disable pullup
		DIDR0 |= (1 << pinNumber);		// Disable digital input buffer
	}
	
	// Configure ADC
	// Set Vref as AVCC
	ADMUX = ADMUX & ~(1<<REFS1) | (1 << REFS0);
	// Set data as right justified (can set to left justified and read only ADCL if 8-bit precision is enough)
	ADMUX &= ~(1 << ADLAR);
	// Set the MUX to read from the desired pinNumber
	ADMUX &= ~0b1111;
	ADMUX |= pinNumbers[0];	// Initially reads from first pin
	
	// Calculate the closest prescaler for selected sampleTime
	// Auto-triggered conversions, S&H time = 2 cycles, Total Conversion time = 13.5 cycles
	// Datasheet recommends 50kHz - 200kHz input clock -> conversion time 67.5-270 us.
	uint8_t pre = (sampleTime / 13.5) * (F_CPU / 1000000);
	for (uint8_t i=0; i<8; i++) {
		pre >>= 1;
		if (!pre) {
			pre = i+1;
			break;
		}
	}
	
	ADCSRA &= ~0b111;	// Zero out prescaler bits of ADCSRA
	ADCSRA |= pre;		// Set prescaler bits to desired value
	
	// Enable auto-triggering based on Timer0 overflow
	ADCSRB &= ~0b111;	// Clear ADTS bits of ADCSRB
	ADCSRB |= 0b100;	// Set ADTS[2:0] to 0b100 = Timer/Counter0 Overflow
	ADCSRA |= (1 << ADATE);	// Enable auto-triggering
	
	// Setup Timer0
	TCCR0A &= ~(0b11 << COM0A0);	// Clear COM0A[1:0] bits
	TCCR0A &= ~(0b11 << COM0B0);	// Clear COM0B[1:0] bits
	TCCR0A |= (0b00 << COM0A0);		// Disable OC0A pin output
	TCCR0A |= (0b00 << COM0B0);		// Disable OC0B pin output
	TCCR0A &= ~(0b11 <<WGM00);		// Clear WGM0[1:0] bits
	TCCR0B &= ~(0b1 << WGM02);		// Clear WGM0[2] bit
	
	// Calculate required PRE and TOP value by checking each prescaler
	// Possible PRE values are (1, 8, 64, 256, 1024)
	uint16_t TCO_PRE[5] = {0, 3, 6, 8, 10};
	uint16_t TOP = 0;
	pre = 5;
	
	// Correct sampleRate for number of channels
	sampleRate = sampleRate*numPins;
	for (uint8_t i=0; i<5; i++) {
		TOP = (F_CPU >> TCO_PRE[i]) / sampleRate;
		if (TOP < 0xFF){
			pre = i+1;
			break;
		}
		else {
			TOP = 0xFF;
		}
	}
	
	TCCR0B &= ~(0b111 << CS00);		// Clear CS0[2:0] bits
	TCCR0B |= pre;
	TC0_BUFF = 0xFF - TOP;
	TCNT0 = TC0_BUFF;
	// Enable the timers interrupt on overflow
	TIMSK0 |= (1 << TOIE0);
	
	// Enable the interrupt on conversion
	ADCSRA |= (1 << ADIE);
	// Enable the ADC
	ADCSRA |= (1 << ADEN);
	
	return 0;
}

ISR(TIMER0_OVF_vect) {
	TCNT0 = TC0_BUFF;		// Reset counter to buffer value for correct period.
}

ISR(ADC_vect) {
	volatile static uint8_t i = 0;
	
	adc_value_arr[i++] = ADCL | (ADCH<<8);
	
	
	if (i > NUM_ADC_PINS){
		i = 0;
		NEW_VALUE = true;
	}
	
	ADCSRA &= ~(1 << ADEN);
	
	ADMUX &= ~0b1111;
	ADMUX |= ADC_PINS[i];	// Set mux to read from next pin
	
	ADCSRA |= (1 << ADEN);
}

uint16_t controlAlg(uint8_t kp, uint8_t kr) {
	uint16_t val; // 0-1023, 0 = 0V, 1023 = 5V
	uint16_t err; // error val
	uint16_t diff;	// d/dt (Ref)
	uint16_t meas;  // measured pressure value
	
	meas = 20.0 / adc_value_arr[0];
	err = adc_value_arr[1] - meas;
	diff = adc_value_arr[1] - ref_value_prev;
	
	val = kp * err + kr * diff / (2 * sample_time);	// control alg might need scaling

	if (val < -500) {		//fitting val into a duty cycle (0-1023)
        val = 0;
    } else if (val > 500) {
        val = 1023;
    } else {
        val = ((val + 500) * 1023) / 1000;
    }

	return val;
} 

// PWM code
void initPWMControl(void) {
	TCCR1A = (1 << COM1A1) | (1 << WGM11) | (1 << WGM10); // Set up 10-bit phase correct PWM, clear on count up, set on count down, set TOP to 1023 (0x03FF)
	DDRB |= (1 << MTR_Pin); // Set pin B1 as output
	TIMSK1 = (1 << TOIE1); // Enable interrupt on overflow
	OCR1A = 511; //50% ie 0 RPM 
	TCCR1B = (1 << CS11) | (1 << CS10); // Prescaler set to 64, start timer
}

ISR(TIMER1_OVF_vect) {
	OCR1A = control_value; 
}

