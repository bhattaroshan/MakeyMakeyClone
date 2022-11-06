
#define F_CPU 12000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include<util/delay.h>

#include "usbdrv.h"
#include "oddebug.h"

#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)

#define KEY_A       4
#define KEY_B       5
#define KEY_C       6
#define KEY_D       7
#define KEY_E       8
#define KEY_F       9
#define KEY_G       10
#define KEY_H       11
#define KEY_I       12
#define KEY_J       13
#define KEY_K       14
#define KEY_L       15
#define KEY_M       16
#define KEY_N       17
#define KEY_O       18
#define KEY_P       19
#define KEY_Q       20
#define KEY_R       21
#define KEY_S       22
#define KEY_T       23
#define KEY_U       24
#define KEY_V       25
#define KEY_W       26
#define KEY_X       27
#define KEY_Y       28
#define KEY_Z       29
#define KEY_1       30
#define KEY_2       31
#define KEY_3       32
#define KEY_4       33
#define KEY_5       34
#define KEY_6       35
#define KEY_7       36
#define KEY_8       37
#define KEY_9       38
#define KEY_0       39

#define KEY_F1      58
#define KEY_F2      59
#define KEY_F3      60
#define KEY_F4      61
#define KEY_F5      62
#define KEY_F6      63
#define KEY_F7      64
#define KEY_F8      65
#define KEY_F9      66
#define KEY_F10     67
#define KEY_F11     68
#define KEY_F12     69

#define KEY_UP_ARROW		0x52
#define KEY_DOWN_ARROW		0x51
#define KEY_LEFT_ARROW		0x50
#define KEY_RIGHT_ARROW		0x4F
#define KEY_SPACE			0x2C

#define LEFT_BUTTON    1
#define RIGHT_BUTTON   2
#define MIDDLE_BUTTON  3

//////////////////////////////////////////////////////////////////////
//																	//
//					   DONOT USE THESE SETTINGS						//
//					   THIS IS FOR INTERNAL USE						//
//																	//
//////////////////////////////////////////////////////////////////////

uint8_t xPositive = 0, xposonTime = 0;
uint8_t xNegative = 0, xnegonTime = 0;
uint8_t yPositive = 0, yposonTime = 0;
uint8_t yNegative = 0, ynegonTime = 0;

uint8_t mouseSpeed = 1;
uint16_t mouseSpeedCounter = 0;

uint8_t byteCounter=0,bitCounter=0;

static uchar    reportBufferKeyboard[8];    /* buffer for HID keyboard reports */
static uchar    reportBufferMouse[4];		/* buffer for HID Mouse reports */
static uchar    idleRate;           /* in 4 ms units */

//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//																	//
//					   TOTAL KEYS IN DEVICE							//
//																	//
//////////////////////////////////////////////////////////////////////

#define NUM_KEYS    12 //this is actual keyboard keys

#define TOTAL_KEYS 18  //this is total keys including mouse keys

//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//																	//
//					 MOVING AVERAGE FILTER SETTINGS					//
//																	//
//////////////////////////////////////////////////////////////////////

#define BUFFER_BYTES 		3   // used 24 bits for maf
#define RELEASE_THRESHOLD 	12	// threshold according to makey-makey
#define PRESS_THRESHOLD 	14  // threshol according to makey-makey

//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//																	//
//					 		MOUSE SETTINGS							//
//																	//
//////////////////////////////////////////////////////////////////////

#define MOUSE_SPEED 500			//according to my experimentation
uint8_t button_state = 0;		//these variable is to enable click drag feature

//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//																	//
//					STRUCTURE OF MOVING AVERAGE FILTER				//
//																	//
//////////////////////////////////////////////////////////////////////

struct measure
{
 uint8_t measurementBuffer[BUFFER_BYTES];
 uint8_t oldestMeasurement;
 int8_t bufferSum;
 uint8_t pressed;
};

struct measure inputs[TOTAL_KEYS];	//assign structure to each key

//////////////////////////////////////////////////////////////////////

static uchar keyPressed();

//////////////////////////////////////////////////////////////////////
//																	//
//					THESE ARE SCANCODES OF KEYBOARD					//
//																	//
//////////////////////////////////////////////////////////////////////

static const uchar  keyReport[NUM_KEYS + 1] PROGMEM = {
 			0,                     		//zero is unpress a key
			KEY_W,
			KEY_A,
			KEY_S,
			KEY_D,
			KEY_F,
			KEY_DOWN_ARROW,
			KEY_LEFT_ARROW,
			KEY_RIGHT_ARROW,
			KEY_UP_ARROW,
			KEY_SPACE,
			KEY_K,
			KEY_L,
};

//////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//																	//
//					 HID REPORT DESCRIPTOR PARSER					//
//	Use:															//
//   These are the parser for HID device, each hex numbers in 		//
//	 array has it's own meaning. Have to go in bit detail to 		//
//	 explain about this.											//
//																	//
//////////////////////////////////////////////////////////////////////

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] 
= {  //35 /* USB report descriptor */
    
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
	0x85, 0x01,					   //REPORT_ID(1) //edited report id
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (1) //multiple keystrokes
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0,                          // END_COLLECTION

	//	Mouse
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)	// 54
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x85, 0x02,                    //     REPORT_ID (2)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    //0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xc0,                          //   END_COLLECTION
    0xc0,                          // END_COLLECTION
};

/////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//																	//
//							USB FUNCTIONING							//
//	Use:															//
//		This function is used to control the flow of data, request,	//
//		to the USB.													//
//																	//
//////////////////////////////////////////////////////////////////////

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

uint8_t reportID;

    
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            
			reportID = rq->wValue.bytes[0];

			if(reportID==1)
			 {
            	usbMsgPtr = reportBufferKeyboard;
				return sizeof(reportBufferKeyboard);
			 }
			else if(reportID==2)
			 {
			  	usbMsgPtr = reportBufferMouse;
				return sizeof(reportBufferMouse);
			 }
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

/////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////
//																	//
//						INITIALIZE HARDWARE							//
//																	//
// Function Name : hardwareInit()									//
// return type : void												//
// argument : void													//
// 																	//
// USE:																//
// 	Initialized the hardware unit for usb and for keys.				//
//	In keys section pull-ups are disabled,external pull-ups used 10M//
//  																//
//////////////////////////////////////////////////////////////////////

static void hardwareInit(void)
{
uchar	i, j;

    PORTB = 0b11000000;    //de-activate pullups on all pins of PORTB
    DDRB = 0b11000000;     // all pins are input, MSB 2 pins are not present in uC
    PORTC = 0b11000000;    // de-activate pullups on all pins of PORTC 
    DDRC = 0b11000000;     // all pins are input, MSB 2 pins are not present in uC
    PORTD = 0b00000000;    // de-activate pullups on all pins of PORTD
    DDRD = 0b00000101;     // all pins input except USB (-> USB reset) 
	j = 0;

	while(--j){     /* USB Reset by device only required on Watchdog Reset */
		i = 0;
		while(--i); /* delay >10ms for USB reset */
	}
    
	DDRD = 0x00;

    /* configure timer 0 for a rate of 12M/(1024 * 256) = 45.78 Hz (~22ms) */
    TCCR0 = 5;      /* timer 0 prescaler: 1024 */
}

////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//																	//
//						PRESS KEYBOARD KEYS							//
//																	//
// Function Name : pressKey()										//
// return type : void												//
// argument : key (it's not actually scancode it's array member)	//
// 																	//
// USE:																//
// 	use to press send key press event in computer					//
//  																//
//////////////////////////////////////////////////////////////////////

void pressKey(uint8_t key)
{

 	uint8_t i;

	reportBufferKeyboard[0]=1; //this is report id
	reportBufferKeyboard[1]=0; //no modifier
	
	//check first if these key is already pressed or not!!!
	if(reportBufferKeyboard[2]!=pgm_read_byte(&keyReport[key]) &&
	   reportBufferKeyboard[3]!=pgm_read_byte(&keyReport[key]) &&
	   reportBufferKeyboard[4]!=pgm_read_byte(&keyReport[key]) && 
	   reportBufferKeyboard[5]!=pgm_read_byte(&keyReport[key]) &&
	   reportBufferKeyboard[6]!=pgm_read_byte(&keyReport[key]) &&
	   reportBufferKeyboard[7]!=pgm_read_byte(&keyReport[key]))
	   {  
	    
		//ok, this key is not pressed, press it now
		//there are 6 multiple keystrokes

    	for(i=2;i<8;i++)
	 	 {

		  //check if any buffer is empty
		  //if it's empty then only put character into buffer
		  //otherwise :(

	      if(reportBufferKeyboard[i]==0)
		   {
		    
			//ok, this buffer is still empty i can add keystroke to this buffer

		    reportBufferKeyboard[i]=pgm_read_byte(&keyReport[key]);

			//added to buffer i don't need to check for any more buffer
			//key is already ready to be pressed 
			//so come out of the loop

			break;
		   }
	 	 }
		 
		 //shit, no buffer are empty you tried to press more than 6 keys at a time :(

		 if(i==8)
		  return; //no space to send keystroke
	   }


   //wait until USB device is ready to send another reportBuffer data
   while(!usbInterruptIsReady()); 
	
   //this function actually sends the reportBuffer data
   usbSetInterrupt(reportBufferKeyboard,sizeof(reportBufferKeyboard));

}

//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//																	//
//						RELEASE KEYBOARD KEYS						//
//																	//
// Function Name : releaseKey()										//
// return type : void												//
// argument : key (it's not actually scancode it's array member)	//
// 																	//
// USE:																//
// 	use to relase key press event in computer						//
//  																//
//////////////////////////////////////////////////////////////////////

void releaseKey(uint8_t key)
{

 uint8_t i;//,j;

 

	 reportBufferKeyboard[0]=1; //this is report id
	 reportBufferKeyboard[1]=0; //no modifier
	 
	//find out if the request key is really pressed or not from our buffer

	 for(i=2;i<8;i++)
	  {
	   
	    if(reportBufferKeyboard[i]==pgm_read_byte(&keyReport[key]))
		//yes the key is pressed let's release it now
	    	reportBufferKeyboard[i]=0;
	  }
	
	//wait until ready
	while(!usbInterruptIsReady());
  	
	//this function actually sends the reportBuffer data
    usbSetInterrupt(reportBufferKeyboard,sizeof(reportBufferKeyboard));
  
}

//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//																	//
//						PRESS MOUSE KEYS							//
//																	//
// Function Name : pressMouse()										//
// return type : void												//
// argument : button (from LSB )									//
//			  		first bit is left mouse button					//
//					second bit is right mouse button				//
//					third bit is middle mouse button				//
// 																	//
// USE:																//
// 	use to press mouse button event in computer						//
//  																//
//////////////////////////////////////////////////////////////////////

void pressMouse(uint8_t button)
{

   reportBufferMouse[0]=2; //this is report id

   if(button==0)
    button_state=0b00000000;   	  //this is buttons
   else if(button==1) 			  //left button click
    button_state=0b00000001;	  
   else if(button==2)			  //right button click
    button_state=0b00000010;
   else if(button==3)			  //middle button click
    button_state=0b00000100;
   
   reportBufferMouse[1]=button_state;   //button state, to enable click drag feature
   reportBufferMouse[2]=0; 				//do not move on x axis
   reportBufferMouse[3]=0; 				//do not move on y axis

   //while(!usbInterruptIsReady()); //wait until interrupt is ready
   
   //wait until interrupt is ready
   if(usbInterruptIsReady())
   //this function actually sends the reportBuffer data
   	usbSetInterrupt(reportBufferMouse,sizeof(reportBufferMouse));
 
}

////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//																	//
//						RELEASE MOUSE KEYS							//
//																	//
// Function Name : releaseKey()										//
// return type : void												//
// argument : key (same as pressMouse function description)			//
// 																	//
// USE:																//
// 	use to relase mouse key press event in computer					//
//  																//
//////////////////////////////////////////////////////////////////////

void releaseMouse(uint8_t key)
{
   //reset appropriate bit from 3 bits
   button_state&=~key;   

   reportBufferMouse[0]=2; //this is report id

   reportBufferMouse[1]=button_state;   //this is buttons

   reportBufferMouse[2]=0; //do not move on x axis
   reportBufferMouse[3]=0; //do not move on y axis

   //while(!usbInterruptIsReady()); //wait until interrupt is ready
   
   //wait until interrupt is ready
   if(usbInterruptIsReady())
   //this function actually sends the reportBuffer data
   	usbSetInterrupt(reportBufferMouse,sizeof(reportBufferMouse));

}

/////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//																	//
//								MOVE MOUSE 							//
//																	//
// Function Name : moveMouse()										//
// return type : void												//
// argument : x +ve values moves to +ve x axis and vice-versa		//
//			  y +ve values moves to +ve y axis and vice-versa		//
// 																	//
// USE:																//
// 	use to relase mouse key press event in computer					//
//  																//
//////////////////////////////////////////////////////////////////////

void moveMouse(int8_t x, int8_t y)
{

     reportBufferMouse[0]=2;  //report id of mouse, see report descriptor for this id
	 reportBufferMouse[1]=button_state; // to keep last state of mouse button alive
	 reportBufferMouse[2]=x;	//move mouse cursor in x-axis
	 reportBufferMouse[3]=y;	//move mouse cursor in y-axis
	 
	 //if(x==0 && y==0)
	  //reportBufferMouse[1]=0;
	 //while(!usbInterruptIsReady()); //wait until interrupt is ready

	//wait until interrupt is ready
	if(usbInterruptIsReady())
	//this function actually sends the reportBuffer data
	 usbSetInterrupt(reportBufferMouse,sizeof(reportBufferMouse));

}

/////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
//																	//
//								keyPressed 							//
//																	//
// Function Name : keyPressed()										//
// return type : static uchar (unsigned char)						//						//
// argument : NULL													//
// 																	//
// USE:																//
// 	This function actually has moving average filter and finds		//
//	whether the keys are pressed or release							//
//  																//
//////////////////////////////////////////////////////////////////////

static uchar keyPressed(void)
{
 uint8_t i,newMeasurement=0,currentByte,currentMeasurement;

 for(i=0;i<TOTAL_KEYS;i++)
  {
   
   currentByte=inputs[i].measurementBuffer[byteCounter];

   inputs[i].oldestMeasurement=(currentByte>>bitCounter)&0x01;
   
   if(i<6)
    newMeasurement=(PINB&(1<<i));		//pin0 to 5 portb
   else if(i>=6 && i<12)
    newMeasurement=(PINC&(1<<(i-6)));   //this is pc0-5
   else if(i==12)
    newMeasurement=(PIND&(1<<1));       //this is pd1
   else if(i>12 && i<18)
    newMeasurement=(PIND&(1<<(i-10)));  //start from pd3 to pd7

   newMeasurement=!newMeasurement;

   if(newMeasurement)
    currentByte |= (1<<bitCounter);
   else
    currentByte &= ~(1<<bitCounter);
   
   inputs[i].measurementBuffer[byteCounter] = currentByte;
  }
   //update buffer sums
 for(i=0;i<TOTAL_KEYS;i++)
  {
   currentByte=inputs[i].measurementBuffer[byteCounter];
   currentMeasurement=(currentByte>>bitCounter)&0x01;
   if(currentMeasurement)
    inputs[i].bufferSum++;
   
   if(inputs[i].oldestMeasurement)
    inputs[i].bufferSum--;
  }
  
   bitCounter++;
   if(bitCounter==8)
    {
	 bitCounter=0;
	 byteCounter++;
	 if(byteCounter==BUFFER_BYTES)
	  byteCounter=0;
	}
    
	for(i=0;i<TOTAL_KEYS;i++)
	{

	 if (inputs[i].pressed)
	  {
	 	if(inputs[i].bufferSum<RELEASE_THRESHOLD) //release key
	  	 { 
		    inputs[i].pressed = 0;

			if(i==16)
			 releaseMouse(LEFT_BUTTON); //release left and right mouse button
			else if(i==17)
			 releaseMouse(RIGHT_BUTTON);
  			else if(i==15)
			 xPositive = 0; //right arrow
			else if(i==14)
			 xNegative = 0; //left arrow
			else if(i==13)
			 yNegative = 0; //up arrow
			else if(i==12)
			 yPositive =0;  //down arrow
			else
	  		 releaseKey(i+1);
			//return 0;
	  	 }
      }
      else if(!inputs[i].pressed)
	  {
	    if(inputs[i].bufferSum>PRESS_THRESHOLD) //press key
		 {
        	inputs[i].pressed = 1;
			
			if(i==16)
			 pressMouse(LEFT_BUTTON); //click left mouse button
			else if(i==17)
			 pressMouse(RIGHT_BUTTON); //right click
			else if(i==15)
			 xPositive=1;
			else if(i==14)
			 xNegative=1;
			else if(i==13)
			 yNegative=1;
			else if(i==12)
			 yPositive=1;
			else
			 pressKey(i+1);
			//return 1;
		 }
	  }
	 
	}

//////////////////////////////////////////////////////////////////////
//																	//
//							  MOUSE MOVEMENTS						//
//																	//
//////////////////////////////////////////////////////////////////////


//mouse speed

if(xPositive || xNegative ||  yPositive || yNegative)
{
 mouseSpeedCounter++;
 if(mouseSpeedCounter>MOUSE_SPEED)
  {
   mouseSpeedCounter=0;
   mouseSpeed++;
  }
}


//positive up xy axis movement
if(xPositive && yNegative)
 {
  moveMouse(mouseSpeed,-mouseSpeed);
  //xyposuponTime=1;
 }

//negative up xy axis movement
if(xNegative && yNegative)
 {
  moveMouse(-mouseSpeed,-mouseSpeed);
  //xypos
 }

//positive down xy axis movement
if(xPositive && yPositive)
 {
  moveMouse(mouseSpeed,mouseSpeed);
 }

//negative down xy axis movement
if(xNegative && yPositive)
 {
  moveMouse(-mouseSpeed,mouseSpeed);
 }

//positive x axis movement
if(xPositive)
 {
  moveMouse(mouseSpeed,0);
  xposonTime=1;
 }
else if(!xPositive && xposonTime==1)
 {
  moveMouse(0,0);
  xposonTime=0;
  mouseSpeedCounter=0;
  mouseSpeed=1;
 }

//negative x axis movement
if(xNegative)
 {
  moveMouse(-mouseSpeed,0);
  xnegonTime=1;
 }
else if(!xNegative && xnegonTime==1)
 {
  moveMouse(0,0);
  xnegonTime=0;
  mouseSpeedCounter=0;
  mouseSpeed=1;
 }

//positive y axis movement
if(yPositive)
 {
  moveMouse(0,mouseSpeed);
  yposonTime=1;
 }
else if(!yPositive && yposonTime==1)
 { 
  moveMouse(0,0);
  yposonTime=0;
  mouseSpeedCounter=0;
  mouseSpeed=1;
 }

//negative y axis movement
if(yNegative)
 {
  moveMouse(0,-mouseSpeed);
  ynegonTime=1;
 }
else if(!yNegative && ynegonTime==1)
{
 moveMouse(0,0);
 ynegonTime=0;
 mouseSpeedCounter=0;
 mouseSpeed=1;
}




while(TCNT1<=1116);
TCCR1B=0;
TCNT1=0;
TCCR1B=(1<<CS11);


return 0;

}

/////////////////////////////////////////////////////////////////////


//main file
int	main(void)
{

	uchar   idleCounter = 0; //device should remain idle for sometime
	
	wdt_enable(WDTO_2S); 	 //enable watchdog, in any case if restart is necesarry
	
	hardwareInit();			 //initialize hardware
	
	for(uint8_t i=0;i<TOTAL_KEYS;i++) //reset all buffers and values of struct to 0
	 {
	  for(uint8_t j=0;j<BUFFER_BYTES;j++)
	  	inputs[i].measurementBuffer[j]=0;

	  inputs[i].oldestMeasurement=0;
	  inputs[i].bufferSum=0;
	  inputs[i].pressed=0;
	 }
	
	TCCR1B=(1<<CS11);

	odDebugInit();
	usbInit();


	sei();
    DBG1(0x00, 0, 0);

	for(;;){			/* main event loop */
		wdt_reset();
		usbPoll();		//This function must be called at least once in 50ms
		
		keyPressed();	//check for key pressed

        if(TIFR & (1<<TOV0)){   // 22 ms timer 
            TIFR = 1<<TOV0;
            if(idleRate != 0){
                if(idleCounter > 4){
                    idleCounter -= 5;   /* 22 ms in units of 4 ms */
                }else{
                    idleCounter = idleRate;
                }
            }
        }

      
	}
	return 0;
}

/* ------------------------------------------------------------------------- */


