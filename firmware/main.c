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

#define WRITE_MODES 	0x02
#define WRITE_VALUES 	0x03

#define BTN_DEBOUNCE_TOGGLE	100		// number of passes before a button can trigger again

// ==============================================================================
// Globals
// ------------------------------------------------------------------------------

static u08		mux;
static u08		switch_states[8],switch_states_before[8],switch_debounce[64];

static u08 		button_modes[64];
static u08		led_values[8];								// state of all 
static u08 		write_state,write_idx,write_len;


// ------------------------------------------------------------------------------
// - read and write presets
// ------------------------------------------------------------------------------
// first 64 byytes in eeprom are reserved for button mode table
// preset 0 gets loaded on startup


void storePreset(u08 preset) {
	u08 i;
	u08 address;			
	address = 8 * preset;
	if (address > 0xF7) return;  // todo  gnusb.h only supports 8bit adressing of eeprom
	
	for (i = 0; i < 8; i++) {
		eepromWrite(64 + address + i,led_values[i]);
	}	
}

void recallPreset(u08 preset) {
	u08 i;
	
	u08 address;			
	address = 8 * preset;
	
	for (i = 0; i < 8; i++) {
		led_values[i] = eepromRead(64 + address + i);
	}	
}


// ------------------------------------------------------------------------------
// - usbFunctionSetup
// ------------------------------------------------------------------------------
// this function gets called when the usb driver receives a non standard request
// that is: our own requests defined in ../common/gnusb_cmds.h
// here's where the magic happens...

uchar usbFunctionSetup(uchar data[8])
{
	uchar i;
			
	switch (data[1]) {
	// 								----------------------------  get all values		
		case GNUSB_CMD_POLL:    
		
			usbMsgPtr = led_values;
	        return sizeof(led_values);
    		break;
    		
		case GNUSB_CMD_SETMODE:
			
			button_modes[data[2]] = data[4];
			eepromWrite(data[2],data[4]);
			break;
			
		case GNUSB_CMD_STORE_PRESET:
			storePreset(data[2]);
			break;
			
		case GNUSB_CMD_RECALL_PRESET:
			recallPreset(data[2]);
			break;

		case GNUSB_CMD_CLEAR:

			for (i = 0; i < 8; i++) {
				led_values[i] = 0;
			}
			break;
			
		case GNUSB_CMD_SET:
			write_idx = data[4];
			write_len = data[2];
			write_state = WRITE_VALUES;
			return 0xFF;
			break;

		case GNUSB_CMD_SET_ALL_MODES:
			write_idx = data[4];
			write_len = data[2];
			write_state = WRITE_MODES;
			return 0xFF;
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
// - usbFunctionWrite
// ------------------------------------------------------------------------------
uchar usbFunctionWrite(uchar* data, uchar len)
{

	uchar* data_end = data + len;
	uchar i;

	if (write_state == WRITE_VALUES) {
		for(; (data < data_end) && (write_idx < write_len); ++data, ++write_idx)
			led_values[write_idx] = *data;
	}  else if 	(write_state == WRITE_MODES) {
		for(; (data < data_end) && (write_idx < write_len); ++data, ++write_idx)
			button_modes[write_idx] = *data;	
	} else return 0xff; // stall
	if(write_idx >= write_len) {
	
		if (write_state == WRITE_MODES) {
			for (i = 0; i < 64; i++) {
				eepromWrite(i,button_modes[i]);
			}
		}
		
		return 1;  	// tell driver we've got all data
	}
	return 0;	 	// otherwise, tell we want still more data
}



// ------------------------------------------------------------------------------
// - Handle Buttons
// ------------------------------------------------------------------------------

void checkButtons(void){
	u08 i;
	
	if (TIFR & (1 << TOV0)) {	// only check buttons on timer overflow
	
		TIFR |= (1 << TOV0); 	// clear timer overflow flag (BY WRITING 1 TO IT, STUPID...) 
		
		mux++;
		mux = mux % 8;
		switch_states_before[mux] = switch_states[mux];

		LED_PORT = 0;
		MUX_PORT = (1 << mux);
		LED_PORT = led_values[mux];						

		for (i=0;i<128;i++);
		switch_states[mux] = ~PINB;						// pullups : 1 = not pressed
	
								// debounce all buttons;
		for (i = 0; i < 64; i++) {
			if (switch_debounce[i]) switch_debounce[i]--;
		}
	}

	if (switch_states[mux] == switch_states_before[mux]) return; // nothing happened

	u08 btn_idx,trigger_hi,trigger_lo,btn_mode,btn_radio_group;
	
	trigger_hi = (~switch_states_before[mux] & switch_states[mux]);  // lo to high transitions
	trigger_lo = (switch_states_before[mux] & ~switch_states[mux]);  // lo to high transitions
	//usb_reply[mux] = trigger_hi;

	
	for (i=0; i<8; i++){
		btn_idx = 8 * mux + i;
		btn_mode = (button_modes[btn_idx] & BTN_MODE_MASK);
		
		switch (btn_mode) {
			case BTN_MODE_TOGGLE:
				if (trigger_hi & (1 << i)) {
					if (switch_debounce[btn_idx]) break;
					switch_debounce[btn_idx] = BTN_DEBOUNCE_TOGGLE;  // don't let this button trigger too soon again
					
					led_values[mux] ^= (1 << (7 -i));
					
				}
				break;

			case BTN_MODE_RADIO:
				
				if (trigger_hi & (1 << i)) {

					u08 row,col;
					btn_radio_group = button_modes[btn_idx];
					
					// turn off all buttons in same group
					for (row = 0; row < 8; row++) {
						for (col = 0; col < 8; col++) {
							if (button_modes[ 8 * row + col] == btn_radio_group) {					
								led_values[row] &= ~(1 << (7 - col));
							}	
						}
					}
					
					// turn on this button
					led_values[mux] |=  (1 << (7 - i));
				}
				break;

			case BTN_MODE_IMPULSE:
			
				if (trigger_hi & (1 << i)) {
					led_values[mux]  |= (1 << (7 - i));
				} else if (trigger_lo & (1 << i)) {
					led_values[mux]  &= ~(1 << (7-i));
				}
				break;
		}
	}	
}

// ------------------------------------------------------------------------------
// - welcomeLights
// ------------------------------------------------------------------------------
// Small light show on plugin

void welcomeLights(void) {
 	u08 intro_steps = 128;
	u08 i;
	
	while (intro_steps) {
		if (TIFR & (1 << TOV0)) {	//timer overflow
			
				TIFR |= (1 << TOV0); 	// clear timer overflow flag (BY WRITING 1 TO IT, STUPID...) 
				
				mux++;
				if (mux == 8) {
					intro_steps--;
					mux = 0;
					if ((intro_steps % 4) == 0) {
						for (i = 0; i < 7; i++) {
							led_values[i] = led_values[i+1];
						}
					led_values[7] = (led_values[7] << 1);
					if (intro_steps > 64) led_values[7] |= 1;

					}
				}
							
				LED_PORT = 0;
				MUX_PORT = (1 << mux);
				LED_PORT = led_values[mux];						
			}
	}	
}

// ------------------------------------------------------------------------------
// - initState
// ------------------------------------------------------------------------------

void initState() {
	u08 i,j;
	
	// read button modes from eeprom
	for (i = 0; i < 64; i++) {
		//	button_modes[i] = 0x80;//eepromRead(i);
			button_modes[i] = eepromRead(i);
	}
	recallPreset(0);
}



// ------------------------------------------------------------------------------
// gets called before device goes to sleep
void goodNight() {
	TCCR0 = 0; 	// stop timer 0 fck/8
	PORTA = 0;
	PORTB = 0;	
	PORTC = 0;
}


// ------------------------------------------------------------------------------
// gets called before device goes to sleep
void goodMorning() {
	TCCR0 = (1 << CS01); 	// start timer 0 fck/8
	TCCR0 |= (1 << CS00); 	// start timer 0 fck/64
	PORTB 	= 0xff;		// make sure pull-up resistors are turned ON
}



// ==============================================================================
// - main
// ------------------------------------------------------------------------------
int main(void)
{

	

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


	TCCR0 = (1 << CS01); 	// start timer 0 fck/8
	TCCR0 |= (1 << CS00); 	// start timer 0 fck/64
	
	welcomeLights();	// show off a bit
	initState();
	
	// PORTD: gnusbCore stuff: USB, status leds, jumper
	initCoreHardware();
	ledOn(STATUS_LED_GREEN);

		
	// ------------------------- Main Loop
	while(1) {
        wdt_reset();		// reset Watchdog timer - otherwise Watchdog will reset gnusb
		sleepIfIdle();		// go to low power mode if host computer is sleeping
		usbPoll();			// see if there's something going on on the usb bus
	
		checkButtons();
	}
	return 0;
}
