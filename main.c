/* Minimal V-USB joystick example. Runs on USBasp hardware.
Copyright (C) 2014 Shay Green
Licensed under GPL v2 or later. See License.txt. */

#define  F_CPU   16000000    /* 16 MHz */

#include <stdint.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "usbdrv/usbdrv.h"

#define ENC1_BTN PINB2
#define ENC1_A PINB0
#define ENC1_B PINC0
#define ENC1_BTN_PRESSED (!(PINB & (1 << ENC1_BTN)))
#define ENC1_AON (PINB & (1 << ENC1_A))
#define ENC1_BON (PINC & (1 << ENC1_B))
#define ENC1_LEFT  0b00000001
#define ENC1_RIGHT 0b00000010

#define ENC2_BTN PINB3
#define ENC2_A PINB1
#define ENC2_B PINC1
#define ENC2_BTN_PRESSED (!(PINB & (1 << ENC2_BTN)))
#define ENC2_AON (PINB & (1 << ENC2_A))
#define ENC2_BON (PINC & (1 << ENC2_B))
#define ENC2_LEFT  0b00000100
#define ENC2_RIGHT 0b00001000

#define ENC3_BTN PINB4
#define ENC3_A PIND7
#define ENC3_B PINA1
#define ENC3_BTN_PRESSED (!(PINB & (1 << ENC3_BTN)))
#define ENC3_AON (PIND & (1 << ENC3_A))
#define ENC3_BON (PINA & (1 << ENC3_B))
#define ENC3_LEFT  0b00010000
#define ENC3_RIGHT 0b00100000

#define ENC4_BTN PINB5
#define ENC4_A PIND6
#define ENC4_B PINA0
#define ENC4_BTN_PRESSED (!(PINB & (1 << ENC4_BTN)))
#define ENC4_AON (PIND & (1 << ENC4_A))
#define ENC4_BON (PINA & (1 << ENC4_B))
#define ENC4_LEFT  0b01000000
#define ENC4_RIGHT 0b10000000

#define BTN_LEFT PINC2
#define BTN_LEFT_PRESSED (!(PINC & (1 << BTN_LEFT)))
#define BTN_DOWN PINC3
#define BTN_DOWN_PRESSED (!(PINC & (1 << BTN_DOWN)))
#define BTN_RIGHT PINC4
#define BTN_RIGHT_PRESSED (!(PINC & (1 << BTN_RIGHT)))
#define BTN_UP PINC5
#define BTN_UP_PRESSED (!(PINC & (1 << BTN_UP)))
#define BTN_CENTER PINC7
#define BTN_CENTER_PRESSED (!(PINC & (1 << BTN_CENTER)))

#define BTN1 PIND3
#define BTN1_PRESSED (!(PIND & (1 << BTN1)))
#define BTN2 PIND4
#define BTN2_PRESSED (!(PIND & (1 << BTN2)))
#define BTN3 PIND5
#define BTN3_PRESSED (!(PIND & (1 << BTN3)))

#define BTN4 PINA2
#define BTN4_PRESSED (!(PINA & (1 << BTN4)))
#define BTN5 PINA3
#define BTN5_PRESSED (!(PINA & (1 << BTN5)))
#define BTN6 PINB7
#define BTN6_PRESSED (!(PINB & (1 << BTN6)))

#define BTN_SWITCH PIND0
#define BTN_SWITCH_ON (PIND & (1 << BTN_SWITCH))

int8_t btn_switch_old = 0;

int8_t btn_switch_send = 0;
volatile int8_t encoder1 = 0;
volatile int8_t encoder2 = 0;
volatile int8_t encoder3 = 0;
volatile int8_t encoder4 = 0;
volatile int8_t state1 = 0;
volatile int8_t state2 = 0;
volatile int8_t state3 = 0;
volatile int8_t state4 = 0;
volatile int8_t pausePassed = 0;

static uint8_t report[3]; // current
static uint8_t report_out[3]; // last sent over USB

inline int reset_timer() {
	TCNT1H = 0;
	TCNT1L = 0;
	pausePassed = 0;
}

inline int checkState1() {
	if (state1 == 4 || state1 == -4) {
		encoder1 = state1 / 4;
		state1 = 0;
		reset_timer();
	}
}

inline int checkState2() {
	if (state2 == 4 || state2 == -4) {
		encoder2 = state2 / 4;
		state2 = 0;
		reset_timer();
	}
}

inline int checkState3() {
	if (state3 == 4 || state3 == -4) {
		encoder3 = state3 / 4;
		state3 = 0;
		reset_timer();
	}
}

inline int checkState4() {
	if (state4 == 4 || state4 == -4) {
		encoder4 = state4 / 4;
		state4 = 0;
		reset_timer();
	}
}

static void init_joy( void )
{
	DDRA &= 0;
	PORTA |= 0b11111111;

	DDRC &= 0;
	PORTC |= 0b11111111;

	DDRB &= 0;
	PORTB |= 0b11111111;

	DDRD &= 0b00000110;
	PORTD |= 0b11111000;

	// прерывания енкодеров порт B
	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT0);
	PCMSK0 |= (1 << PCINT1);

	// прерывания енкодеров порт C
	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT8);
	PCMSK1 |= (1 << PCINT9);

	// прерывания енкодеров порт A
	PCICR |= (1 << PCIE3);
	PCMSK3 |= (1 << PCINT24);
	PCMSK3 |= (1 << PCINT25);

	// прерывания енкодеров порт D
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT22);
	PCMSK2 |= (1 << PCINT23);
	PCMSK2 |= (1 << PCINT16); // switch

	// 25мс
	TIMSK1 |= (1 << OCIE1A);
	TCCR1B |= (1 << WGM12);
	OCR1AH = 0b00000110; 
	OCR1AL = 0b00011010;
	TCCR1B |= (1 << CS12);
}

static void read_joy( void )
{
	if (BTN1_PRESSED) {
		report[0] |= 0b1;
	}
	if (BTN2_PRESSED) {
		report[0] |= 0b10;
	}
	if (BTN3_PRESSED) {
		report[0] |= 0b100;
	}
	if (BTN4_PRESSED) {
		report[0] |= 0b1000;
	}
	if (BTN5_PRESSED) {
		report[0] |= 0b10000;
	}
	if (BTN6_PRESSED) {
		report[0] |= 0b100000;
	}
	if (ENC1_BTN_PRESSED) {
		report[0] |= 0b1000000;
	}
	if (ENC2_BTN_PRESSED) {
		report[0] |= 0b10000000;
	}

	if (ENC3_BTN_PRESSED) {
		report[1] |= 0b1;
	}
	if (ENC4_BTN_PRESSED) {
		report[1] |= 0b10;
	}
	if (BTN_LEFT_PRESSED) {
		report[1] |= 0b100;
	}
	if (BTN_DOWN_PRESSED) {
		report[1] |= 0b1000;
	}
	if (BTN_RIGHT_PRESSED) {
		report[1] |= 0b10000;
	}
	if (BTN_UP_PRESSED) {
		report[1] |= 0b100000;
	}
	if (BTN_CENTER_PRESSED) {
		report[1] |= 0b1000000;
	}
}

static void read_switches(void) {
	if (btn_switch_send == 1) {
		report[1] |= 0b10000000;
	}
}

static void read_encoders(void) {
	//поворот енкодера 1
	if (encoder1 < 0) {
		report[2] |= ENC1_LEFT;
	}
	if (encoder1 > 0) {
		report[2] |= ENC1_RIGHT;
	}

	// поворот енкодера 2
	if (encoder2 < 0) {
		report[2] |= ENC2_LEFT;
	}
	if (encoder2 > 0) {
		report[2] |= ENC2_RIGHT;
	}

	// поворот енкодера 3
	if (encoder3 < 0) {
		report[2] |= ENC3_LEFT;
	}
	if (encoder3 > 0) {
		report[2] |= ENC3_RIGHT;
	}

	// поворот енкодера 4
	if (encoder4 < 0) {
		report[2] |= ENC4_LEFT;
	}
	if (encoder4 > 0) {
		report[2] |= ENC4_RIGHT;
	}
}

static void reset_joy(void) {
	report[0] = 0;
	report[1] = 0;
	report[2] = 0;

	if (pausePassed) {
		encoder1 = 0;
		encoder2 = 0;
		encoder3 = 0;
		encoder4 = 0;
		btn_switch_send = 0;
	}
}

PROGMEM const char usbHidReportDescriptor [USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
	0x05, 0x01,     // USAGE_PAGE (Generic Desktop)
	0x09, 0x05,     // USAGE (Game Pad)
	0xa1, 0x01,     // COLLECTION (Application)
	0xa1, 0x00,     //     COLLECTION (Physical)
	0x05, 0x09,     //     USAGE_PAGE (Button)
	0x19, 0x01,     //     USAGE_MINIMUM (Button 1)
	0x29, 0x18,     //     USAGE_MAXIMUM (Button 24)
	0x15, 0x00,     //     LOGICAL_MINIMUM (0)
	0x25, 0x01,     //     LOGICAL_MAXIMUM (1)
	0x75, 0x01,     //     REPORT_SIZE (1)
	0x95, 0x18,     //     REPORT_COUNT (24)
	0x81, 0x02,     //     INPUT (Data,Var,Abs)
	0xc0,           //   END_COLLECTION
	0xc0            // END_COLLECTION
};

uint8_t usbFunctionSetup(uint8_t data[8])
{
	usbRequest_t const* rq = (usbRequest_t const*) data;

	if ( (rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_CLASS )
		return 0;
	
	switch ( rq->bRequest )
	{
	case USBRQ_HID_GET_REPORT: // HID joystick only has to handle this
		usbMsgPtr = (usbMsgPtr_t) report_out;
		return sizeof report_out;
	
	//case USBRQ_HID_SET_REPORT: // LEDs on joystick?
	
	default:
		return 0;
	}
}

ISR (TIMER1_COMPA_vect) {
	pausePassed = 1;
}

ISR(PCINT0_vect) {
	PCICR &= ~(1 << PCIE0);

	if (state1 == 0 && !ENC1_AON && ENC1_BON || state1 == -2 && ENC1_AON && !ENC1_BON) {
		state1--;
	}
	else if (state1 == 1 && !ENC1_AON && !ENC1_BON || state1 == 3 && ENC1_AON && ENC1_BON) {
		state1++;
	}
	checkState1();	
	if (ENC1_AON && ENC1_BON && state1 != 0) state1 = 0;


	if (state2 == 0 && !ENC2_AON && ENC2_BON || state2 == -2 && ENC2_AON && !ENC2_BON) {
		state2--;
	}
	else if (state2 == 1 && !ENC2_AON && !ENC2_BON || state2 == 3 && ENC2_AON && ENC2_BON) {
		state2++;
	}
	checkState2();
	if (ENC2_AON && ENC2_BON && state2 != 0) state2 = 0;

	PCICR |= (1 << PCIE0);
}

ISR(PCINT1_vect) {
	PCICR &= ~(1 << PCIE1);

	if (state1 == -1 && !ENC1_AON && !ENC1_BON || state1 == -3 && ENC1_AON && ENC1_BON) {
		state1--;
	}
	else if (state1 == 0 && ENC1_AON && !ENC1_BON || state1 == 2 && !ENC1_AON && ENC1_BON) {
		state1++;
	}
	checkState1();
	if (ENC1_AON && ENC1_BON && state1 != 0) state1 = 0;


	if (state2 == -1 && !ENC2_AON && !ENC2_BON || state2 == -3 && ENC2_AON && ENC2_BON) {
		state2--;
	}
	else if (state2 == 0 && ENC2_AON && !ENC2_BON || state2 == 2 && !ENC2_AON && ENC2_BON) {
		state2++;
	}
	checkState2();
	if (ENC2_AON && ENC2_BON && state2 != 0) state2 = 0;
	
	PCICR |= (1 << PCIE1);
}

ISR(PCINT3_vect) {
	PCICR &= ~(1 << PCIE3);

	if (state3 == -1 && !ENC3_AON && !ENC3_BON || state3 == -3 && ENC3_AON && ENC3_BON) {
		state3--;
	}
	else if (state3 == 0 && ENC3_AON && !ENC3_BON || state3 == 2 && !ENC3_AON && ENC3_BON) {
		state3++;
	}
	checkState3();
	if (ENC3_AON && ENC3_BON && state3 != 0) state3 = 0;

	if (state4 == -1 && !ENC4_AON && !ENC4_BON || state4 == -3 && ENC4_AON && ENC4_BON) {
		state4--;
	}
	else if (state4 == 0 && ENC4_AON && !ENC4_BON || state4 == 2 && !ENC4_AON && ENC4_BON) {
		state4++;
	}
	checkState4();
	if (ENC4_AON && ENC4_BON && state4 != 0) state4 = 0;

	PCICR |= (1 << PCIE3);
}

ISR(PCINT2_vect) {
	if (BTN_SWITCH_ON != btn_switch_old) {
		btn_switch_send = 1;
		btn_switch_old = BTN_SWITCH_ON;
		reset_timer();
		return;
	}
	PCICR &= ~(1 << PCIE2);

	if (state3 == 0 && !ENC3_AON && ENC3_BON || state3 == -2 && ENC3_AON && !ENC3_BON) {
		state3--;
	}
	else if (state3 == 1 && !ENC3_AON && !ENC3_BON || state3 == 3 && ENC3_AON && ENC3_BON) {
		state3++;
	}
	checkState3();
	if (ENC3_AON && ENC3_BON && state3 != 0) state3 = 0;

	// против частовой
	if (state4 == 0 && !ENC4_AON && ENC4_BON || state4 == -2 && ENC4_AON && !ENC4_BON) {
		state4--;
	}
	else if (state4 == 1 && !ENC4_AON && !ENC4_BON || state4 == 3 && ENC4_AON && ENC4_BON) {
		state4++;
	}
	checkState4();
	if (ENC4_AON && ENC4_BON && state4 != 0) state4 = 0;

	PCICR |= (1 << PCIE2);
}

int main( void )
{
	usbInit();
	init_joy();

	btn_switch_old = BTN_SWITCH_ON;

	_delay_ms(500);

	sei();
	
	while (1)
	{
		usbPoll();
		
		read_joy();
		// Don't bother reading joy if previous changes haven't gone out yet.
		// Forces delay after changes which serves to debounce controller as well.
		if ( usbInterruptIsReady() )
		{
			read_encoders();
			read_switches();

			// Don't send update unless joystick changed
			if ( memcmp( report_out, report, sizeof report ) )
			{
				memcpy( report_out, report, sizeof report );
				usbSetInterrupt( report_out, sizeof report_out );
			}
			reset_joy();
		}
	}
	
	return 0;
}