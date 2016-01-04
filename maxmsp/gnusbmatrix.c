// ==============================================================================
//	gnusbmatrix.c
//	
//	Max-Interface to the [ a n y m a | gnusbmatrix - Open Source USB Sensor Box ]
//	
//	Authors:	Michael Egger
//	Copyright:	2007 [ a n y m a ]
//	Website:	www.anyma.ch
//	
//	License:	GNU GPL 2.0 www.gnu.org
//	
//	Version:	2007-11-12
// ==============================================================================



#include "ext.h"  				// you must include this - it contains the external object's link to available Max functions
#include "ext_common.h"

#include "../common/GNUSB_CMDs.h"		// codes used between gnusbmatrix client and host software, eg. between the max external and the gnusbmatrix firmware
#include </usr/local/include/usb.h>     // this is libusb, see http://libusb.sourceforge.net/ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ==============================================================================
// Constants
// ------------------------------------------------------------------------------

#define USBDEV_SHARED_VENDOR    	0x16C0  /* VOTI */
#define USBDEV_SHARED_PRODUCT   	0x05DC  /* Obdev's free shared PID */
#define OUTLETS 					9
#define DEFAULT_CLOCK_INTERVAL		40		// default interval for polling the gnusbmatrix: 40ms

// ==============================================================================
// Our External's Memory structure
// ------------------------------------------------------------------------------

typedef struct _gnusbmatrix				// defines our object's internal variables for each instance in a patch
{
	t_object 		p_ob;					// object header - ALL max external MUST begin with this...
	usb_dev_handle	*dev_handle;			// handle to the gnusbmatrix usb device
	void			*m_clock;				// handle to our clock
	double 			m_interval;				// clock interval for polling the gnusbmatrix
	double 			m_interval_bak;			// backup clock interval for polling the gnusbmatrix
	int				is_running;				// is our clock ticking?
	int				do_10_bit;				// output analog values with 8bit or 10bit resolution?
	int				debug_flag;
	void 			*outlets[OUTLETS];		// handle to the objects outlets
	int 			values[8];				// stored values from last poll
} t_gnusbmatrix;

void *gnusbmatrix_class;					// global pointer to the object class - so max can reference the object 


// ==============================================================================
// Function Prototypes
// ------------------------------------------------------------------------------

void *gnusbmatrix_new		(t_symbol *s);

void gnusbmatrix_assist		(t_gnusbmatrix *x, void *b, long m, long a, char *s);
void gnusbmatrix_bang		(t_gnusbmatrix *x);				
void gnusbmatrix_clear		(t_gnusbmatrix *x);
void gnusbmatrix_close		(t_gnusbmatrix *x);
void gnusbmatrix_debug		(t_gnusbmatrix *x,  long n);
void gnusbmatrix_int		(t_gnusbmatrix *x,long n);
void gnusbmatrix_open		(t_gnusbmatrix *x);
void gnusbmatrix_poll		(t_gnusbmatrix *x, long n);
void gnusbmatrix_recall		(t_gnusbmatrix *x, long n);
void gnusbmatrix_setmode	(t_gnusbmatrix *x, long btn, t_symbol *mode, long radiogroup);
void gnusbmatrix_start		(t_gnusbmatrix *x);
void gnusbmatrix_stop		(t_gnusbmatrix *x);
void gnusbmatrix_store		(t_gnusbmatrix *x, long n);
void gnusbmatrix_list		(t_gnusbmatrix *x, t_symbol *s, short ac, t_atom *av);
void gnusbmatrix_setmodes	(t_gnusbmatrix *x, t_symbol *s, short ac, t_atom *av);

// functions used to find the USB device
static int  	usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen);
void 			find_device(t_gnusbmatrix *x);



// ==============================================================================
// Implementation
// ------------------------------------------------------------------------------


//--------------------------------------------------------------------------
// - Message: clear 		-> clear all values
//--------------------------------------------------------------------------

void gnusbmatrix_clear		(t_gnusbmatrix *x){
	if (!(x->dev_handle)) find_device(x);
	else {
		usb_control_msg(x->dev_handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
							GNUSB_CMD_CLEAR, 0, 0, NULL, 0 , 1000);
	}
}

//--------------------------------------------------------------------------
// - Message: set 		-> set values
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------

void gnusbmatrix_list(t_gnusbmatrix *x, t_symbol *s, short ac, t_atom *av)
{
	int i;
	unsigned char  		buffer[8];
	int                 nBytes;

	if (ac > 8) ac = 8;
	
	for(i=0; i<ac; ++i,av++) {
			if (av->a_type==A_LONG)
				buffer[i] = MIN(MAX(av->a_w.w_long, 0), 255);
			else
				buffer[i] = 0;
		}

	if (!(x->dev_handle)) find_device(x);
	else {

		nBytes = usb_control_msg(x->dev_handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT,
									GNUSB_CMD_SET, ac, 0, buffer, ac, 1000);
	}
}

//--------------------------------------------------------------------------
// - Message: recall 		-> recall a preset
//--------------------------------------------------------------------------

void gnusbmatrix_recall		(t_gnusbmatrix *x, long n){
	if (n <  0) n =  0;
	if (n > 50) n = 50;	
	if (!(x->dev_handle)) find_device(x);
	else {
		usb_control_msg(x->dev_handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
							GNUSB_CMD_RECALL_PRESET, n, 0, NULL, 0 , 1000);
	}
}


//--------------------------------------------------------------------------
// - Message: store 		-> store a preset
//--------------------------------------------------------------------------
void gnusbmatrix_store		(t_gnusbmatrix *x, long n){
	if (n <  0) n =  0;
	if (n > 50) n = 50;	
	if (!(x->dev_handle)) find_device(x);
	else {
		usb_control_msg(x->dev_handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
							GNUSB_CMD_STORE_PRESET, n, 0, NULL, 0 , 1000);
	}
}

//--------------------------------------------------------------------------
// - Message: mode	 		-> change mode of button
//--------------------------------------------------------------------------
void gnusbmatrix_setmode	(t_gnusbmatrix *x, long btn, t_symbol *mode, long radiogroup){

	if (btn <  0) btn =  0;
	if (btn > 63) btn = 63;	
	
	unsigned char themode;
	

	if (mode == gensym("none") | mode == gensym("n")) 		{ 
		themode = BTN_MODE_NONE 	;
	} else if (mode == gensym("impulse") | mode == gensym("i")) 	{
		themode = BTN_MODE_IMPULSE;
	} else if (mode == gensym("toggle") | mode == gensym("t")) 	{
		themode = BTN_MODE_TOGGLE	;
	} else if (mode == gensym("radio") | mode == gensym("r")) 	{ 
		themode = BTN_MODE_RADIO	;
		if (radiogroup <  0) radiogroup =  0;
		if (radiogroup > 31) radiogroup = 31;
		themode = themode | (unsigned char)radiogroup;
	} else {
		post ("gnusbmatrix: unknown mode\n");
		return;
	}
		post ("mode %d\n",themode);

	if (!(x->dev_handle)) find_device(x);
	else {
		usb_control_msg(x->dev_handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
							GNUSB_CMD_SETMODE, (unsigned char)btn, (unsigned char)themode, NULL, 0 , 1000);
	}
}

//--------------------------------------------------------------------------
// - Message: setmodes	 		-> change mode of all buttons
//--------------------------------------------------------------------------

void gnusbmatrix_setmodes	(t_gnusbmatrix *x, t_symbol *s, short ac, t_atom *av){
	
	if (ac > 64) ac = 64;
	
	int 	i;
	char* 	buf = malloc( ac );
	int                 nBytes;


	for(i=0; i<ac; ++i,av++) {
		if (av->a_type==A_LONG)
			buf[i] = MIN(MAX(av->a_w.w_long, 0), 255);
		else
			buf[i] = 0;
	}

	if (!(x->dev_handle)) find_device(x);
	else {
	
		nBytes = usb_control_msg(x->dev_handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_OUT,
									GNUSB_CMD_SET_ALL_MODES, ac, 0, buf, ac, 1000);
	}
}

//--------------------------------------------------------------------------
// - Message: debug
//--------------------------------------------------------------------------

void gnusbmatrix_debug(t_gnusbmatrix *x, long n)	// x = the instance of the object; n = the int received in the left inlet 
{
	if (n)	x->debug_flag = 1;
	else 	x->debug_flag = 0;
}
//--------------------------------------------------------------------------
// - Message: bang  -> poll the gnusbmatrix
//--------------------------------------------------------------------------

void gnusbmatrix_bang(t_gnusbmatrix *x)	// poll the gnusbmatrix
{
	int                 nBytes,i,n;
	int					temp;
	unsigned char       buffer[8];
	t_atom				myList[3];
	t_atom				bitList[8];

	
	if (!(x->dev_handle)) find_device(x);
	else {
		// ask the gnusbmatrix to send us data
		nBytes = usb_control_msg(x->dev_handle, USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
									GNUSB_CMD_POLL, 0, 0, (char *)buffer, sizeof(buffer), 10);
		// let's see what has come back...							
		if(nBytes < sizeof(buffer)){
			if (x->debug_flag) {
				if(nBytes < 0)
					post( "USB error: %s\n", usb_strerror());
				post( "only %d bytes status received\n", nBytes);
			}
		} else {
			for (i = 0; i < 8; i++) {
				temp = buffer[i];
					
				if (x->values[i] != temp) {					// output if value has changed
					
					SETLONG(myList+1,7-i);
					for (n=0; n < 8; n++) {
						SETLONG(myList,n);
						SETLONG(myList+2,((temp & (1 << n)) != 0));
						SETLONG(bitList+n,((temp & (1 << n)) != 0));
						outlet_list(x->outlets[8], 0L,3,&myList);
					}
					outlet_list(x->outlets[i], 0L,8,&bitList);					
				x->values[i] = temp;
				}
			}
		}
	}
}


//--------------------------------------------------------------------------
// - Message: open 		-> open connection to gnusbmatrix
//--------------------------------------------------------------------------

void gnusbmatrix_open(t_gnusbmatrix *x)
{
	if (x->dev_handle) {
		post("gnusbmatrix: There is already a connection to www.anyma.ch/gnusbmatrix",0);
	} else find_device(x);
}

//--------------------------------------------------------------------------
// - Message: close 	-> close connection to gnusbmatrix
//--------------------------------------------------------------------------

void gnusbmatrix_close(t_gnusbmatrix *x)
{
	if (x->dev_handle) {
		usb_close(x->dev_handle);
		x->dev_handle = NULL;
		post("gnusbmatrix: Closed connection to www.anyma.ch/gnusbmatrix",0);
	} else
		post("gnusbmatrix: There was no open connection to www.anyma.ch/gnusbmatrix",0);
}

//--------------------------------------------------------------------------
// - Message: poll 		-> set polling interval
//--------------------------------------------------------------------------

void gnusbmatrix_poll(t_gnusbmatrix *x, long n){
	if (n > 0) { 
		x->m_interval = n;
		x->m_interval_bak = n;
		gnusbmatrix_start(x);
	} else {
		gnusbmatrix_stop(x);
	}
}


//--------------------------------------------------------------------------
// - Message: int 		-> zero stops / nonzero starts
//--------------------------------------------------------------------------

void gnusbmatrix_int(t_gnusbmatrix *x,long n) {
	if (n) {
		if (!x->is_running) gnusbmatrix_start(x);
	} else {
		if (x->is_running) gnusbmatrix_stop(x);
	}
}

//--------------------------------------------------------------------------
// - Message: start 	-> start automatic polling
//--------------------------------------------------------------------------

void gnusbmatrix_start (t_gnusbmatrix *x) { 
	if (!x->is_running) {
		clock_fdelay(x->m_clock,0.);
		x->is_running  = 1;
	}
} 

//--------------------------------------------------------------------------
// - Message: stop 		-> stop automatic polling
//--------------------------------------------------------------------------

void gnusbmatrix_stop (t_gnusbmatrix *x) { 
	if (x->is_running) {
		x->is_running  = 0;
		clock_unset(x->m_clock); 
		gnusbmatrix_close(x);
	}
} 



//--------------------------------------------------------------------------
// - The clock is ticking, tic, tac...
//--------------------------------------------------------------------------

void gnusbmatrix_tick(t_gnusbmatrix *x) { 
	clock_fdelay(x->m_clock, x->m_interval); 	// schedule another tick
	gnusbmatrix_bang(x); 								// poll the gnusbmatrix
} 


//--------------------------------------------------------------------------
// - Object creation and setup
//--------------------------------------------------------------------------

int main(void)
{
	setup((t_messlist **)&gnusbmatrix_class, (method)gnusbmatrix_new, 0L, (short)sizeof(t_gnusbmatrix), 0L, A_DEFSYM, 0); 
	// setup() loads our external into Max's memory so it can be used in a patch
	// gnusbmatrix_new = object creation method defined below, A_DEFLONG = its (optional) arguement is a long (32-bit) int 
	
															// Add message handlers
	addbang((method)gnusbmatrix_bang);
	addint((method)gnusbmatrix_int);
	addmess((method)gnusbmatrix_list,"list", A_GIMME, 0);	
	addmess((method)gnusbmatrix_debug,"debug", A_DEFLONG, 0);
	addmess((method)gnusbmatrix_open, "open", 0);		
	addmess((method)gnusbmatrix_close, "close", 0);	
	addmess((method)gnusbmatrix_poll, "poll", A_DEFLONG,0);	
	addmess((method)gnusbmatrix_recall, "recall",A_DEFLONG,0);	
	addmess((method)gnusbmatrix_store, "store", A_DEFLONG,0);	
	addmess((method)gnusbmatrix_setmode, "mode", A_DEFLONG,A_SYM,A_DEFLONG,0);	
	addmess((method)gnusbmatrix_start, "start", 0);	
	addmess((method)gnusbmatrix_stop, "stop", 0);	
	addmess((method)gnusbmatrix_clear, "clear", 0);	
	addmess((method)gnusbmatrix_setmodes, "modes", A_GIMME,0);	
	
	return 1;
}

//--------------------------------------------------------------------------

void *gnusbmatrix_new(t_symbol *s)		// s = optional argument typed into object box (A_SYM) -- defaults to 0 if no args are typed
{
	t_gnusbmatrix *x;										// local variable (pointer to a t_gnusbmatrix data structure)

	x = (t_gnusbmatrix *)newobject(gnusbmatrix_class); 			// create a new instance of this object
	x->m_clock = clock_new(x,(method)gnusbmatrix_tick); 	// make new clock for polling and attach gnsub_tick function to it
	
	x->m_interval = DEFAULT_CLOCK_INTERVAL;
	x->m_interval_bak = DEFAULT_CLOCK_INTERVAL;

	x->debug_flag = 0;
	x->dev_handle = NULL;
	int i;
													// create outlets and assign it to our outlet variable in the instance's data structure
	for (i=0; i < OUTLETS; i++) {
		x->outlets[i] = listout(x);	
	}	

	return x;					// return a reference to the object instance 
}



//--------------------------------------------------------------------------
// - Object destruction
//--------------------------------------------------------------------------

void gnusbmatrix_free(t_gnusbmatrix *x)
{
	if (x->dev_handle) usb_close(x->dev_handle);
	freeobject((t_object *)x->m_clock);  			// free the clock
}





//--------------------------------------------------------------------------
// - USB Utility Functions
//--------------------------------------------------------------------------


static int  usbGetStringAscii(usb_dev_handle *dev, int index, int langid, char *buf, int buflen)
{
char    buffer[256];
int     rval, i;

    if((rval = usb_control_msg(dev, USB_ENDPOINT_IN, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8) + index, langid, buffer, sizeof(buffer), 1000)) < 0)
        return rval;
    if(buffer[1] != USB_DT_STRING)
        return 0;
    if((unsigned char)buffer[0] < rval)
        rval = (unsigned char)buffer[0];
    rval /= 2;
    /* lossy conversion to ISO Latin1 */
    for(i=1;i<rval;i++){
        if(i > buflen)  /* destination buffer overflow */
            break;
        buf[i-1] = buffer[2 * i];
        if(buffer[2 * i + 1] != 0)  /* outside of ISO Latin1 range */
            buf[i-1] = '?';
    }
    buf[i-1] = 0;
    return i-1;
}

//--------------------------------------------------------------------------


void find_device(t_gnusbmatrix *x)
{
	usb_dev_handle      *handle = NULL;
	struct usb_bus      *bus;
	struct usb_device   *dev;
	
	usb_find_busses();
    usb_find_devices();
	 for(bus=usb_busses; bus; bus=bus->next){
        for(dev=bus->devices; dev; dev=dev->next){
            if(dev->descriptor.idVendor == USBDEV_SHARED_VENDOR && dev->descriptor.idProduct == USBDEV_SHARED_PRODUCT){
                char    string[256];
                int     len;
                handle = usb_open(dev); /* we need to open the device in order to query strings */
                if(!handle){
                    error ("Warning: cannot open USB device: %s", usb_strerror());
                    continue;
                }
                /* now find out whether the device actually is gnusbmatrix */
                len = usbGetStringAscii(handle, dev->descriptor.iManufacturer, 0x0409, string, sizeof(string));
                if(len < 0){
                    post("gnusbmatrix: warning: cannot query manufacturer for device: %s", usb_strerror());
                    goto skipDevice;
                }
                
			//	post("gnusbmatrix: seen device from vendor ->%s<-", string); 
                if(strcmp(string, "www.anyma.ch") != 0)
                    goto skipDevice;
                len = usbGetStringAscii(handle, dev->descriptor.iProduct, 0x0409, string, sizeof(string));
                if(len < 0){
                    post("gnusbmatrix: warning: cannot query product for device: %s", usb_strerror());
                    goto skipDevice;
                }
              //  post("gnusbmatrix: seen product ->%s<-", string);
                if(strcmp(string, "gnusbmatrix") == 0)
                    break;
skipDevice:
                usb_close(handle);
                handle = NULL;
            }
        }
        if(handle)
            break;
    }
	
    if(!handle){
        post("gnusbmatrix: Could not find USB device www.anyma.ch/gnusbmatrix");
		x->dev_handle = NULL;
		if (x->m_interval < 10000) x->m_interval *=2; // throttle polling down to max 20s if we can't find a gnusbmatrix
	} else {
		x->dev_handle = handle;
		 post("gnusbmatrix: Found USB device www.anyma.ch/gnusbmatrix");
		 x->m_interval = x->m_interval_bak;			// restore original polling interval
		 if (x->is_running) gnusbmatrix_tick(x);
		 else gnusbmatrix_bang(x);
	}
}