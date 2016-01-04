// ==============================================================================
// main.c
// firmware for a device based on the gnusb - OPEN SOURCE USB SENSOR BOX
//
// License:
// The project is built with AVR USB driver by Objective Development, which is
// published under an own licence based on the GNU General Public License (GPL).
// gnusb is also distributed under this enhanced licence. See Documentation.
//
// target-cpu: ATMega16 @ 12MHz
// created 2007-01-28 Michael Egger me@anyma.ch
//
// ==============================================================================

#include "gnusb.h"				// the gnusb library: setup and utility functions 
#include <util/delay.h>
// ==============================================================================
// Constants
// ------------------------------------------------------------------------------
#define LED_KEEP_ALIVE	100  	// number of passes before usb status led turns off

#define LED_PORT 	PORTC
#define SWITCH_PORT PORTB
#define	MUX_PORT	PORTA

#define BTN_DEBOUNCE_TOGGLE	100		// number of passes before a button cfan trigger again

// ==============================================================================
// Globals
// ------------------------------------------------------------------------------

typedef enum {
	toggle = 0,
	radio = 2,
	impulse = 3
} button_mode;

static u08		usb_reply[12];	
static u08		mux,do_debounce;
static u08		button_values[8],button_values_before[8],button_debounce[64];
button_mode 	button_modes[64];
static u08		led_values[8];



// ------------------------------------------------------------------------------
// - usbFunctionSetup
// ------------------------------------------------------------------------------
// this function gets called when the usb driver receives a non standard request
// that is: our own requests defined in ../common/gnusb_cmds.h
// here's where the magic happens...

uchar usbFunctionSetup(uchar data[8])
{
	
	switch (data[1]) {
	// 								----------------------------  get all values		
		case GNUSB_CMD_POLL:    
		
			usbMsgPtr = usb_reply;
	        return sizeof(usb_reply);
    		break;
    		
			
	// 								----------------------------   Start Bootloader for reprogramming the gnusb    		
		case GNUSB_CMD_START_BOOTLOADER:

			startBootloader();
			break;
			
		default:
			break;
				
	} 
	
	return 0;
}




// ------------------------------------------------------------------------------
// - Handle Buttons
// ------------------------------------------------------------------------------

void checkButtons(void){
	u08 i;
	
	if (TIFR & (1 << TOV0)) {
	
		TIFR |= (1 << TOV0); 	// clear timer overflow flag (BY WRITING 1 TO IT, STUPID...) 
		
		mux++;
		mux = mux % 8;
		button_values_before[mux] = button_values[mux];

		LED_PORT = 0;
		MUX_PORT = (1 << mux);
		LED_PORT = led_values[mux];						// all this happens really quickly so nothing is seen on leds

		for (i=0;i<128;i++);
		button_values[mux] = ~PINB;						// pullups : 1 = not pressed
	
								// debounce all buttons;
		for (i = 0; i < 64; i++) {
			if (button_debounce[i]) button_debounce[i]--;
		}
	}

	if (button_values[mux] == button_values_before[mux]) return; // nothing happened

	u08 btn_idx,trigger_hi,trigger_lo;
	
	
	trigger_hi = (~button_values_before[mux] & button_values[mux]);  // lo to high transitions
	trigger_lo = (button_values_before[mux] & ~button_values[mux]);  // lo to high transitions
	usb_reply[mux] = trigger_hi;

	
	for (i=0; i<8; i++){
		btn_idx = 8 * mux + i;
		
		switch (button_modes[btn_idx]) {
			case toggle:
				if (trigger_hi & (1 << i)) {
					if (button_debounce[btn_idx]) break;
					button_debounce[btn_idx] = BTN_DEBOUNCE_TOGGLE;  // don't let this button trigger too soon again
					
					led_values[mux] ^= (1 <<  i);
					
				}
				break;

			case radio:
				break;

			case impulse:
				break;
		}
		
		
	}
	
}


// ==============================================================================
// - main
// ------------------------------------------------------------------------------
int main(void)
{
	for (mux = 0; mux < 8; mux++) {
	//	led_values[mux] = (1 << mux);
		led_values[mux] = 0xFF;
	}

	mux = 0;
	// ------------------------- Initialize Hardware
		

	// PORTA: MUX
	DDRA 	= 0xff;		// set all pins to output
	PORTA 	= 0x00;		// all off
	
	// PORTB: Switches
	DDRB 	= 0x00;		// set all pins to input
	PORTB 	= 0xff;		// make sure pull-up resistors are turned ON

	// PORTC: LEDS
	DDRC 	= 0xff;		// set all pins to output
	PORTC 	= 0x00;		// turn off 
	
	// PORTD: gnusbCore stuff: USB, status leds, jumper
	initCoreHardware();
	ledOn(STATUS_LED_GREEN);

	TCCR0 = (1 << CS01); 	// start timer 0 fck/8
	TCCR0 |= (1 << CS00); 	// start timer 0 fck/64
		
	// ------------------------- Main Loop
	while(1) {
        wdt_reset();		// reset Watchdog timer - otherwise Watchdog will reset gnusb
    //    sleepIfIdle();		// go to low power mode if host computer is sleeping
		usbPoll();			// see if there's something going on on the usb bus
	
		checkButtons();
	}
	return 0;
}

