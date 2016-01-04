// ==============================================================================
// gnusb_commands.h
//
// Commands shared between the gnusb firmware and host software
//
// License:
// The project is built with AVR USB driver by Objective Development, which is
// published under an own licence based on the GNU General Public License (GPL).
// gnusb is also distributed under this enhanced licence. See Documentation.
//
// created 2007-01-28 Michael Egger me@anyma.ch
// mdified 2007-11-13 "
//
// ==============================================================================



// get values of sensors connected to the gnusb
#define GNUSB_CMD_POLL 				2

// Set state of Leds connected to PORTC (8 bit)
#define GNUSB_CMD_SET_PORTC 			3

// Set state of Leds connected to PORTB (8 bit)
#define GNUSB_CMD_SET_PORTB 			4

#define GNUSB_CMD_INPUT_PORTB			5
#define GNUSB_CMD_INPUT_PORTC			6
#define GNUSB_CMD_SET_SMOOTHING			7


// specific to gnusb matrix
#define GNUSB_CMD_SETMODE			0xc1
#define GNUSB_CMD_STORE_PRESET		0xc2
#define GNUSB_CMD_RECALL_PRESET		0xc3
#define GNUSB_CMD_CLEAR				0xc4
#define GNUSB_CMD_SET				0xc5
#define GNUSB_CMD_SET_ALL_MODES		0xc6

#define BTN_MODE_NONE 		0x00
#define BTN_MODE_IMPULSE	0x40
#define BTN_MODE_TOGGLE		0x80
#define BTN_MODE_RADIO		0xC0
#define BTN_MODE_MASK		0xC0


// Start Bootloader for Software updates
#define GNUSB_CMD_START_BOOTLOADER 	0xf8



