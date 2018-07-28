/*
FreeRTOS V7.0.1 - Copyright (C) 2011 Real Time Engineers Ltd.


***************************************************************************
*                                                                       *
*    FreeRTOS tutorial books are available in pdf and paperback.        *
*    Complete, revised, and edited pdf reference manuals are also       *
*    available.                                                         *
*                                                                       *
*    Purchasing FreeRTOS documentation will not only help you, by       *
*    ensuring you get running as quickly as possible and with an        *
*    in-depth knowledge of how to use FreeRTOS, it will also help       *
*    the FreeRTOS project to continue with its mission of providing     *
*    professional grade, cross platform, de facto standard solutions    *
*    for microcontrollers - completely free of charge!                  *
*                                                                       *
*    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
*                                                                       *
*    Thank you for using FreeRTOS, and thank you for your support!      *
*                                                                       *
***************************************************************************


This file is part of the FreeRTOS distribution and has been modified to 
demonstrate three simple tasks running.

FreeRTOS is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License (version 2) as published by the
Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
>>>NOTE<<< The modification to the GPL is included to allow you to
distribute a combined work that includes FreeRTOS without being obliged to
provide the source code for proprietary components outside of the FreeRTOS
kernel.  FreeRTOS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
more details. You should have received a copy of the GNU General Public
License and the FreeRTOS license exception along with FreeRTOS; if not it
can be viewed here: http://www.freertos.org/a00114.html and also obtained
by writing to Richard Barry, contact details for whom are available on the
FreeRTOS WEB site.

1 tab == 4 spaces!

http://www.FreeRTOS.org - Documentation, latest information, license and
contact details.

http://www.SafeRTOS.com - A version that is certified for use in safety
critical systems.

http://www.OpenRTOS.com - Commercial support, development, porting,
licensing and training services.
*/


/*
* Creates all the application tasks, then starts the scheduler.  The WEB
* documentation provides more details of the standard demo application tasks.
* In addition to the standard demo tasks, the following tasks and tests are
* defined and/or created within this file:
*
* "OLED" task - the OLED task is a 'gatekeeper' task.  It is the only task that
* is permitted to access the display directly.  Other tasks wishing to write a
* message to the OLED send the message on a queue to the OLED task instead of
* accessing the OLED themselves.  The OLED task just blocks on the queue waiting
* for messages - waking and displaying the messages as they arrive.
*
* "Check" hook -  This only executes every five seconds from the tick hook.
* Its main function is to check that all the standard demo tasks are still
* operational.  Should any unexpected behaviour within a demo task be discovered
* the tick hook will write an error to the OLED (via the OLED task).  If all the
* demo tasks are executing with their expected behaviour then the check task
* writes PASS to the OLED (again via the OLED task), as described above.
*
* "uIP" task -  This is the task that handles the uIP stack.  All TCP/IP
* processing is performed in this task.
*/




/*************************************************************************
* Please ensure to read http://www.freertos.org/portlm3sx965.html
* which provides information on configuring and running this demo for the
* various Luminary Micro EKs.
*************************************************************************/

/* Set the following option to 1 to include the WEB server in the build.  By
default the WEB server is excluded to keep the compiled code size under the 32K
limit imposed by the KickStart version of the IAR compiler.  The graphics
libraries take up a lot of ROM space, hence including the graphics libraries
and the TCP/IP stack together cannot be accommodated with the 32K size limit. */

//  set this value to non 0 to include the web server

#define mainINCLUDE_WEB_SERVER		1


/* Standard includes. */
#include <stdio.h>
//self add
#include <limits.h>
#include <string.h> 
#include <math.h> 

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware library includes. */
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "grlib.h"
#include "drivers/rit128x96x4.h"
#include "osram128x64x4.h"
#include "formike128x128x16.h"
//self add
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "inc/hw_ints.h"

/* Demo app includes. */
#include "lcd_message.h"
#include "bitmap.h"
//struct
#include "lab4header.h"
#include "uip.h"
#include "psock.h"
#include "httpd.h"
#include "httpd-cgi.h"
#include "httpd-fs.h"
/*-----------------------------------------------------------*/




/* 
The time between cycles of the 'check' functionality (defined within the
tick hook. 
*/

#define mainCHECK_DELAY	( ( portTickType ) 5000 / portTICK_RATE_MS )

// Size of the stack allocated to the uIP task.
#define mainBASIC_WEB_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 3 )

// The OLED task uses the sprintf function so requires a little more stack too.
#define mainOLED_TASK_STACK_SIZE	    ( configMINIMAL_STACK_SIZE + 50 )

//  Task priorities.
#define mainQUEUE_POLL_PRIORITY		    ( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		    ( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		    ( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		    ( tskIDLE_PRIORITY + 2 )
#define mainCREATOR_TASK_PRIORITY           ( tskIDLE_PRIORITY + 3 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )
#define mainGEN_QUEUE_TASK_PRIORITY	    ( tskIDLE_PRIORITY )



//  The maximum number of messages that can be waiting for display at any one time.
#define mainOLED_QUEUE_SIZE					( 20 )

// Dimensions the buffer into which the jitter time is written. 
#define mainMAX_MSG_LEN						25

/* 
The period of the system clock in nano seconds.  This is used to calculate
the jitter time in nano seconds. 
*/

#define mainNS_PER_CLOCK ( ( unsigned portLONG ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )


// Constants used when writing strings to the display.

#define mainCHARACTER_HEIGHT		    ( 9 )
#define mainMAX_ROWS_128		    ( mainCHARACTER_HEIGHT * 14 )
#define mainMAX_ROWS_96			    ( mainCHARACTER_HEIGHT * 10 )
#define mainMAX_ROWS_64			    ( mainCHARACTER_HEIGHT * 7 )
#define mainFULL_SCALE			    ( 15 )
#define ulSSI_FREQUENCY			    ( 3500000UL )

/*-----------------------------------------------------------*/

/*
* The task that handles the uIP stack.  All TCP/IP processing is performed in
* this task.
*/
extern void vuIP_Task( void *pvParameters );

/*
* The display is written two by more than one task so is controlled by a
* 'gatekeeper' task.  This is the only task that is actually permitted to
* access the display directly.  Other tasks wanting to display a message send
* the message to the gatekeeper.
*/

static void vOLEDTask( void *pvParameters );

/*
* Configure the hardware .
*/
static void prvSetupHardware( void );

/*
* Configures the high frequency timers - those used to measure the timing
* jitter while the real time kernel is executing.
*/
//extern void vSetupHighFrequencyTimer( void );

/*
* Hook functions that can get called by the kernel.
*/
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName );
void vApplicationTickHook( void );

/*-----------------------------------------------------------*/

/* 
The queue used to send messages to the OLED task. 
*/
xQueueHandle xOLEDQueue;

/*-----------------------------------------------------------*/

void vArrvingTrain(void *train);
void vDepartingTrain(void *train);
void vTrainCom(void *train);
void vSwitchControl(void *train);
void vTemperatureMeasurement(void *train);
void vLcdDisplay(void *train);
void vOledDisplay(void *train);
void vLocalKeypad(void *train);
void vSerialCom(void *train);
void vNoiseCapture(void *train);
void vNoiseProcessing(void *train);
unsigned short vRemoteCom( void *train );
void vCommand(void* train);

xTaskHandle AT;     // ArrvingTrain
xTaskHandle DT;     // DepartingTrain
xTaskHandle TC;     // TrainCom
xTaskHandle SC;     // SwitchControl
xTaskHandle TM;     // TeperatureMeasurement
xTaskHandle LD;     // LcdDisplay
xTaskHandle OD;     // OledDisplay
xTaskHandle LK;     // LocalKeypad
xTaskHandle SC;     // SerialCom
xTaskHandle NC;      // NoiseCapture
xTaskHandle NP;       // NoiseProcessing
xTaskHandle RC;    //remote com
xTaskHandle CO;    //COMMAND
int taskdelay = 500;

/*************************************************************************
* Please ensure to read http://www.freertos.org/portlm3sx965.html
* which provides information on configuring and running this demo for the
* various Luminary Micro EKs.
*************************************************************************/

int main( void )
{
  startUp();
  initialize();
  // |--------------------------------------------------------|
  // | Schedule Task                                          |
  // |--------------------------------------------------------|
  //
  // this is the startup task
  
  // set up the hardware
  intersectionLock = xSemaphoreCreateCounting(1,1);
  prvSetupHardware();
  
#if mainINCLUDE_WEB_SERVER != 0
  {
    /* 
    Create the uIP task if running on a processor that includes a MAC and PHY. 
    */
    
    if( SysCtlPeripheralPresent( SYSCTL_PERIPH_ETH ) )
    {
      xTaskCreate( vuIP_Task, ( signed portCHAR * ) "uIP", mainBASIC_WEB_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
    }
  }
#endif
  
  // create the tasks
  xOLEDQueue = xQueueCreate( mainOLED_QUEUE_SIZE, sizeof( xOLEDMessage ) );
  
  xTaskCreate(vArrvingTrain, "ArrvingTrain", 120, &myArrivingTrain, 9, &AT);   //1
  xTaskCreate(vTrainCom, "TrainCom", 90, &myTrainCom, 8, &TC);                 //2
  xTaskCreate(vDepartingTrain, "DepartingTrain", 80, &myDepartingTrain, 7, &DT);//3
  xTaskCreate(vSwitchControl, "SwitchControl", 100, &mySwitchControl, 6, &SC);  //4
  xTaskCreate(vTemperatureMeasurement, "TemperatureMeasurement", 80, &myTemperature, 5, &TM);//8
  xTaskCreate(vLocalKeypad, "LocalKeypad",80, &myLocalKeypad, 4, &LK);     //7
  xTaskCreate(vLcdDisplay, "LcdDisplay", 80, &myLcdDisplay, 1, &LD);           //5
  xTaskCreate(vOledDisplay, "OledDisplay", 170, &myOledDisplay, 1, &OD);           //6
  xTaskCreate(vSerialCom, "SerialCom", 300, &mySerialCom, 3, &SC);                  //11
  xTaskCreate(vNoiseCapture, "NoiseCapture", 80, &myNoiseCapture, 1, &NC);            //9
  xTaskCreate(vNoiseProcessing, "NoiseProcessing", 620, &myNoiseProcessing, 1, &NP);      //10
  xTaskCreate(vCommand, "Command", 90, &myCommand, 1, &CO);
  vTaskStartScheduler();
}

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
  ( void ) pxTask;
  ( void ) pcTaskName;
  
  while( 1 );
}

/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
  /* 
  If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
  a workaround to allow the PLL to operate reliably. 
  */
  
  if( DEVICE_IS_REVA2 )
  {
    SysCtlLDOSet( SYSCTL_LDO_2_75V );
  }
  
  // Set the clocking to run from the PLL at 50 MHz
  
  SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );
  
  /* 	
  Enable Port F for Ethernet LEDs
  LED0        Bit 3   Output
  LED1        Bit 2   Output 
  */
  
  SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOF );
  GPIODirModeSet( GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3), GPIO_DIR_MODE_HW );
  GPIOPadConfigSet( GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3 ), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );	
  
}


/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
  static xOLEDMessage xMessage = { "PASS" };
  static unsigned portLONG ulTicksSinceLastDisplay = 0;
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
  
  /* 
  Called from every tick interrupt.  Have enough ticks passed to make it
  time to perform our health status check again? 
  */
  
  ulTicksSinceLastDisplay++;
  if( ulTicksSinceLastDisplay >= mainCHECK_DELAY )
  {
    ulTicksSinceLastDisplay = 0;
    
  }
}

void startUp()
{
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ); 
  
  // Initialize the OLED display.
  RIT128x96x4Init(1000000);
  
  //
  // Enable the peripherals used by this example.
  //
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  
  //*********************************************************   
  // Timer0 Setup                                               
  //********************************************************* 
  
  //      // Set the clocking to run directly from the crystal
  //      
  //      SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  //  
  //      //
  //      // Configure the two 32-bit periodic timers.
  //      //
  //      TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
  //  
  //      TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / 2);
  //  
  //  
  //      //
  //      // Setup the interrupts for the timer timeouts.
  //      //
  //      IntEnable(INT_TIMER0A); 
  //   
  //      TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  //  
  //  
  //      //
  //      // Enable the timers.
  //      //
  //      TimerEnable(TIMER0_BASE, TIMER_A); 
  
  
  //*********************************************************   
  // Timer1 Setup                                               
  //*********************************************************  
  // Set the clocking to run directly from the crysta
  
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
  
  //
  // Configure the two 32-bit periodic timers.
  //
  TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
  
  TimerLoadSet(TIMER1_BASE, TIMER_A, 2 * SysCtlClockGet());
  
  //*********************************************************   
  // Timer2 Setup                                               
  //*********************************************************  
  // Set the clocking to run directly from the crysta
  
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
  
  //
  // Configure the two 32-bit periodic timers.
  //
  TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
  
  //*********************************************************   
  // Timer3 Setup                                               
  //*********************************************************  
  // Set the clocking to run directly from the crysta
  
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
  
  //
  // Configure the two 32-bit periodic timers.
  //
  TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
  
  //*********************************************************
  // ISR sw2 setup
  //*********************************************************   
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
  GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA,
                   GPIO_PIN_TYPE_STD_WPU);
  GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
  GPIOPinIntEnable(GPIO_PORTF_BASE, GPIO_PIN_1);
  IntEnable(INT_GPIOF);
  
  //*********************************************************
  // ISR sw3 setup
  //*********************************************************  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);
  GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA,
                   GPIO_PIN_TYPE_STD_WPU);
  GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_FALLING_EDGE);
  GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0);
  IntEnable(INT_GPIOE);
  
  // |--------------------------------------------------------|
  // | pwmgen.c - PWM signal generation example.              |
  // |                                                        |
  // | Copyright (c) 2005-2012 Texas Instruments Incorporated.|
  // | All rights reserved.                                   |
  // |--------------------------------------------------------|
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  SysCtlPWMClockSet(SYSCTL_PWMDIV_1); 
  
  
  unsigned long ulPeriod;
  //
  // Set GPIO F0 and G1 as PWM pins.  They are used to output the PWM0 and
  // PWM1 signals.
  //
  GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
  GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
  
  //
  // Compute the PWM period based on the system clock.
  //
  ulPeriod = SysCtlClockGet() / 70;/////////////////////////////////////////////////sound turned off
  
  //
  // Set the PWM period to 100 (A) Hz.
  //
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
                  PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ulPeriod);
  
  //
  // Set PWM0 to a duty cycle of 25% and PWM1 to a duty cycle of 75%.
  //
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ulPeriod / 4);
  PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, ulPeriod * 3 / 4);
  
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);  
  
  //===========================================================   
  // LCD Setup                                               
  //=========================================================== 
  
  //LCD display control
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7); //d7
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6); //d6
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_5); //d5
  GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4); //d4
  GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_7); //e
  GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6); //rs
  
  // Initialize the LCD display.
  delay(10000);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00);  // RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7  n
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6  f
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7  n
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6  f
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  //on/off
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6  d
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5  c
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4  b
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  lcdClear();  
  
  
  ///===========================================================
  // HYPERTERM setup (UTAR)
  //============================================================
  
  //
  // Enable the peripherals used by this example.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  
  //
  // Enable processor interrupts.
  //
  IntMasterEnable();
  
  //
  // Set GPIO A0 and A1 as UART pins.
  //
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  
  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  
  //
  // Enable the UART interrupt.
  //
  IntEnable(INT_UART0);
  UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
  
  //============================================================
  // KEYPAD setup
  //============================================================
  
  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_4);   // home -- 0 
  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_4);   // scroll --1
  GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_5);   // select -- 2
  //initialize
  
  GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA,
                   GPIO_PIN_TYPE_STD_WPD);
  GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA,
                   GPIO_PIN_TYPE_STD_WPD);
  GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_STRENGTH_2MA,
                   GPIO_PIN_TYPE_STD_WPD);
  //============================================================
  // Arriving train setup
  //============================================================
  
  GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4);   // output frequency
  GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_5);   // wireless input
  
  //=============================================================
  //ADC SETUP   for tempture
  //=============================================================
  
  
  //
  // Enable the first sample sequencer to capture the value of channel 0 when
  // the processor trigger occurs.
  //
  //ADCSequenceDisable(ADC0_BASE, 3);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1);
  ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH2);
  ADCSequenceEnable(ADC0_BASE, 0);
  ADCSequenceEnable(ADC0_BASE, 1);
  ADCSequenceEnable(ADC0_BASE, 2);
  
  //for noise
  ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH3);
  ADCSequenceEnable(ADC0_BASE, 3);
  return;
}   

void initialize()
{
  // |--------------------------------------------------------|
  // | Build Task Queue                                       |
  // |--------------------------------------------------------|
  
  // declear variables and set pointers
  
  arrivingTD.taskDataPtr = &myArrivingTrain;
  myArrivingTrain.trainArriving = &trainArriving;
  myArrivingTrain.checkTrain = &checkTrain;
  myArrivingTrain.arrivalDirection = &arrivalDirection;
  myArrivingTrain.departureDirection = &departureDirection;
  myArrivingTrain.globalCount = &globalCount;
  myArrivingTrain.arrivalTime = &arrivalTime;
  myArrivingTrain.arrivingTrainDistance = arrivingTrainDistance;
  myArrivingTrain.departingTrainFlag = &departingTrainFlag;
  myArrivingTrain.trainPresent = &trainPresent;
  
  trainCD.taskDataPtr = &myTrainCom;
  myTrainCom.trainArriving = &trainArriving;
  myTrainCom.departingTrainFlag = &departingTrainFlag;
  myTrainCom.trainSize = &trainSize;
  myTrainCom.arrivalDirection = &arrivalDirection;
  myTrainCom.departureDirection = &departureDirection;
  myTrainCom.checkTrain = &checkTrain;
  myTrainCom.globalCount = &globalCount;
  myTrainCom.trainDeparting = trainDeparting;
  myTrainCom.trainPresent = &trainPresent;
  
  departingTD.taskDataPtr = &myDepartingTrain;
  myDepartingTrain.departingTrainFlag = &departingTrainFlag;
  myDepartingTrain.departureDirection = &departureDirection;
  myDepartingTrain.globalCount = &globalCount;
  myDepartingTrain.trainDeparting = trainDeparting;
  myDepartingTrain.trainPresent = &trainPresent;
  
  SWCD.taskDataPtr = &mySwitchControl;
  mySwitchControl.departureDirection = &departureDirection;
  mySwitchControl.departingTrainFlag = &departingTrainFlag;
  mySwitchControl.trainSize = &trainSize;
  mySwitchControl.interLock = &interLock;
  mySwitchControl.traversalTime = &traversalTime;
  mySwitchControl.gridlock = &gridlock;
  mySwitchControl.globalCount = &globalCount;
  mySwitchControl.trainDeparting = trainDeparting;
  mySwitchControl.trainPresent = &trainPresent;
  
  LCDD.taskDataPtr = &myLcdDisplay;
  myLcdDisplay.departingTrainFlag = &departingTrainFlag;
  myLcdDisplay.trainSize = &trainSize;
  myLcdDisplay.traversalTime = &traversalTime;
  myLcdDisplay.departureDirection = &departureDirection;
  myLcdDisplay.interLock = &interLock;
  myLcdDisplay.gridlock = &gridlock;
  myLcdDisplay.globalCount = &globalCount;
  myLcdDisplay.trainPresent = &trainPresent;
  
  OLEDD.taskDataPtr = &myOledDisplay;
  myOledDisplay.departingTrainFlag = &departingTrainFlag;
  myOledDisplay.trainSize = &trainSize;
  myOledDisplay.interLock = &interLock;
  myOledDisplay.traversalTime = &traversalTime;
  myOledDisplay.trainArriving = &trainArriving;
  myOledDisplay.arrivalDirection = &arrivalDirection;
  myOledDisplay.globalCount = &globalCount;
  myOledDisplay.departureDirection = &departureDirection;
  myOledDisplay.mode = &mode;
  myOledDisplay.scroll = &scroll;
  myOledDisplay.select = &select;
  myOledDisplay.statusSelection = &statusSelection;
  myOledDisplay.gridlock = &gridlock;
  myOledDisplay.arrivingTrainDistance = arrivingTrainDistance;
  myOledDisplay.temperatureBuf=  temperatureBuf;
  myOledDisplay.trainDeparting = trainDeparting;
  myOledDisplay.trainPresent = &trainPresent;
  
  keypadD.taskDataPtr = &myLocalKeypad;
  myLocalKeypad.mode = &mode;
  myLocalKeypad.statusSelection = &statusSelection;
  myLocalKeypad.scroll = &scroll;
  myLocalKeypad.select = &select;
  myLocalKeypad.annunciation = &annunciation;
  myLocalKeypad.globalCount = &globalCount;
  
  serialCD.taskDataPtr = &mySerialCom;
  mySerialCom.departingTrainFlag = &departingTrainFlag;
  mySerialCom.trainSize = &trainSize;
  mySerialCom.traversalTime = &traversalTime;
  mySerialCom.departureDirection = &departureDirection;
  mySerialCom.interLock = &interLock;
  mySerialCom.gridlock = &gridlock;
  mySerialCom.trainArriving = &trainArriving;
  mySerialCom.globalCount = &globalCount;
  mySerialCom.arrivalTime = &arrivalTime;
  mySerialCom.arrivalDirection = &arrivalDirection;
  mySerialCom.trainDeparting = trainDeparting;  
  mySerialCom.trainPresent = &trainPresent;
  
  temperatureD.taskDataPtr = &myTemperature;
  myTemperature.departingTrainFlag = &departingTrainFlag;
  myTemperature.temperatureBuf = temperatureBuf;
  myTemperature.globalStartMeasure = &globalStartMeasure;
  
  noiseCD.taskDataPtr = &myNoiseCapture;
  myNoiseCapture.noiseCaptureBuf = &noiseCaptureBuf;
  myNoiseCapture.globalStartMeasure = &globalStartMeasure;
  
  noisePD.taskDataPtr = &myNoiseProcessing;
  myNoiseProcessing.noiseCaptureBuf = &noiseCaptureBuf;
  myNoiseProcessing.noiseTransBuf = &noiseTransBuf;
  myNoiseProcessing.noiseFrequency = &noiseFrequency;
  
  ramoteCD.taskDataPtr = &myRemote;
  myRemote.departingTrainFlag = &departingTrainFlag;
  myRemote.trainSize = &trainSize;
  myRemote.traversalTime = &traversalTime;
  myRemote.departureDirection = &departureDirection;
  myRemote.arrivalDirection = &arrivalDirection;
  myRemote.trainArriving = &trainArriving;
  myRemote.arrivalTime = &arrivalTime;
  myRemote.gridlock = &gridlock;
  myRemote.temperatureBuf = temperatureBuf;
  myRemote.trainPresent = &trainPresent;
  myRemote.arrivingTrainDistance = arrivingTrainDistance;
  myRemote.noiseTransBuf = noiseTransBuf;
  myRemote.showRecentData = &showRecentData;
  
  commandD.taskDataPtr = &myCommand;
  myCommand.readCommand = readCommand;
  myCommand.globalStartMeasure = &globalStartMeasure;
  myCommand.showRecentData = &showRecentData;
  
  return;
}

void vArrvingTrain(void* train)
{ 
  while (1)
  {
    //task1 = uxTaskGetStackHighWaterMark(NULL);
    arrivingTrainData *arrivingT = (arrivingTrainData*)train;
    static Bool firstTime = TRUE;   // the train first time appeared(away from intersection)
    int randD0;
    int randD1;
    int randN;
    static int count = 0; 
    static int prevDirection = 0;
    static double speed;
    //vCommand(train);
    //RIT128x96x4StringDraw(readCommand, 80, 20, 15);
    // press the switch then a new train arriving
    
    
    if (*(arrivingT->trainArriving) > 0) {
      count ++;
      
      if(firstTime) 
      {
        randD0 = randomInteger(0, 1);  // random number d;
        randD1 = randomInteger(0, 1);
        while (prevDirection == randD0*2 + randD1) {
          randD0 = randomInteger(0, 1);
          randD1 = randomInteger(0, 1);
        }
        randN = randomInteger(1,3);  // random number n, cannot be negative
        *(arrivingT->arrivalDirection) = randD0*2 + randD1;
        prevDirection = randD0 * 2 + randD1;
        firstTime = FALSE;
        *(arrivingT->arrivalTime) = 24 * randN;
        speed = 2000.0 /(double) *(arrivingT->arrivalTime); 
        *(arrivingT->arrivingTrainDistance) = 1000;
        for (int i = 1; i < 7; i ++)
        {
          *(arrivingT->arrivingTrainDistance + i)   = 0;
        }
      }
      
      // generate frequency within 1km
      if(1900 > speed * count * 0.5 && speed * count * 0.5 > 1000) 
      {
        IntEnable(INT_TIMER2A);
        TimerLoadSet(TIMER2_BASE, TIMER_A, (unsigned long)(SysCtlClockGet() / (2000000 / ( 53 * (2000 - speed * count * 0.5) - 5175))));
        TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        TimerEnable(TIMER2_BASE, TIMER_A);
        char dis[80];
        sprintf(dis, "%d", (*arrivingT->arrivingTrainDistance));
        if(globalStartDisplay) 
        {
          if((*arrivingT->arrivingTrainDistance)>999 )
          {
            RIT128x96x4StringDraw(dis, 102, 73, 15);
          } else if ((*arrivingT->arrivingTrainDistance)>99 )
          {
            RIT128x96x4StringDraw(" ", 102, 73, 15);
            RIT128x96x4StringDraw(dis, 108, 73, 15);
          } else if ((*arrivingT->arrivingTrainDistance)>9 )
          {
            RIT128x96x4StringDraw("  ", 102, 73, 15);
            RIT128x96x4StringDraw(dis, 114, 73, 15);
          } else {
            RIT128x96x4StringDraw("   ", 102, 73, 15);
            RIT128x96x4StringDraw(dis, 120, 73, 15);
          }
        }
      } else if (speed * count * 0.5 > 1900 && globalStartDisplay){
        RIT128x96x4StringDraw("100", 108, 73, 15);
      }
      
      // receive frequency
      // active timer3
      IntEnable(INT_TIMER3A);
      TimerLoadSet(TIMER3_BASE, TIMER_A, SysCtlClockGet() / 16000); 
      TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
      TimerEnable(TIMER3_BASE, TIMER_A);
      
      if (remainDistance < 0.9 * (*(arrivingT->arrivingTrainDistance)))
        
      {
        for (int i = 7 ; i > 0; i -- )
        {  
          *(arrivingT->arrivingTrainDistance+i) = *(arrivingT->arrivingTrainDistance+(i-1));
        }
        *(arrivingT->arrivingTrainDistance) = remainDistance;
      }
      
      
      switch (*(arrivingT->arrivalDirection)) {
      case 0:
        switch (*(arrivingT->globalCount) % 6) {
        case 0:
        case 1:
        case 2:
          if (globalStartDisplay)
          {
            RIT128x96x4StringDraw("North Arriving", 15, 73, 15);
          }
          break;
          
        default:
          RIT128x96x4StringDraw("               ", 13, 73, 15);
          break;
        }
        
        switch (*(arrivingT->globalCount) % 20) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 6:
        case 7:
        case 8:
        case 9:
        case 12:
        case 13:
        case 16:
        case 17:
          if(!*(arrivingT->departingTrainFlag))// blast priorityb
          {
            // Enable the PWM0 and PWM1 output signals.
            PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
            // Enable the PWM generator.
            PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          }
          break;
          
        default:
          if(!*(arrivingT->departingTrainFlag))// blast priorityb
          {
            PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
            PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          }
          break;
        }
        break;
        
      case 1:
        switch (*(arrivingT->globalCount) % 6) {
        case 0:
        case 1:
        case 2:
          if (globalStartDisplay)
          {
            RIT128x96x4StringDraw("South Arriving", 15, 73, 15);
          }
          break;
          
        default:
          RIT128x96x4StringDraw("               ", 13, 73, 15);
          break;
        }
        
        switch (*(arrivingT->globalCount) % 28) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 6:
        case 7:
        case 8:
        case 9:
        case 12:
        case 13:
        case 14:
        case 15:
        case 16:
        case 17:
        case 20:
        case 21:
        case 22:
        case 23:
        case 24:
        case 25:
          if(!*(arrivingT->departingTrainFlag))
          {
            // Enable the PWM0 and PWM1 output signals.
            PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
            // Enable the PWM generator.
            PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          }
          break;
          
        default:
          if(!*(arrivingT->departingTrainFlag))// blast priorityb
          {
            PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
            PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          }
          break;
        }
        break;
        
        
      case 2:
        switch (*(arrivingT->globalCount) % 8) {
        case 0:
        case 1:
        case 2:
        case 3:
          if (globalStartDisplay)
          {
            RIT128x96x4StringDraw("East  Arriving", 15, 73, 15);
          }
          break;
          
        default:
          RIT128x96x4StringDraw("               ", 13, 73, 15);
          break;
        }
        
        switch (*(arrivingT->globalCount) % 26) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 6:
        case 7:
        case 8:
        case 9:
        case 12:
        case 13:
        case 14:
        case 15:
        case 18:
        case 19:
        case 22:
        case 23:
          if(!*(arrivingT->departingTrainFlag))
          {
            // Enable the PWM0 and PWM1 output signals.
            PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
            // Enable the PWM generator.
            PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          }
          break;
          
        default:
          if(!*(arrivingT->departingTrainFlag))// blast priorityb
          {
            PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
            PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          }
          break;
        }
        break;
        
        
      case 3:
        switch (*(arrivingT->globalCount) % 4) {
        case 0:
        case 1:
          if (globalStartDisplay)
          {
            RIT128x96x4StringDraw("West  Arriving", 15, 73, 15);
          }
          break;
          
        default:
          RIT128x96x4StringDraw("               ", 13, 73, 15);
          break;
        }
        
        switch (*(arrivingT->globalCount) % 14) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 6:
        case 7:
        case 10:
        case 11:
          if(!*(arrivingT->departingTrainFlag))
          {
            // Enable the PWM0 and PWM1 output signals.
            PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
            // Enable the PWM generator.
            PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          }
          break;
          
        default:
          if(!*(arrivingT->departingTrainFlag))// blast priority
          {
            PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
            PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          }
          break;
        }
        break;
        
      }
      
      if (*(arrivingT->arrivingTrainDistance) < 400)
      {
        *(arrivingT->checkTrain) = TRUE;
        vTaskResume(TC);
      }
      
      if (0 == count*0.5 - *(arrivingT->arrivalTime))
      {
        firstTime = TRUE;
        tcGo = TRUE;
        count = 0;
        *(arrivingT->arrivalTime) = 0;
        TimerIntDisable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
        TimerDisable(TIMER2_BASE, TIMER_A);
        TimerIntDisable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
        TimerDisable(TIMER3_BASE, TIMER_A);
        remainDistance = 1000;
        *(arrivingT->trainArriving) = *(arrivingT->trainArriving) - 1;
        *(arrivingT->trainPresent) = *(arrivingT->trainPresent) + 1;
        *(arrivingT->arrivingTrainDistance) = 0;
        RIT128x96x4StringDraw("   ", 108, 73, 15);
        RIT128x96x4StringDraw("               ", 13, 73, 15);
      }
    } else {
      GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, 0x00);
    }
    
    vTaskDelay(taskdelay * 2);
  }
  return;
}

// |--------------------------------------------------------|
// | trainCom task                                          |
// |--------------------------------------------------------|
void vTrainCom(void* train)
{
  while (1) 
  {   
    //task2 = uxTaskGetStackHighWaterMark(NULL);
    // re-cast the task argument as pointer to the data structure type
    trainComData *tCom = (trainComData*)train;
    if (tcGo && *(tCom->checkTrain)){
      // generate random direction and train size
      int randDirc;
      tcGo = FALSE;
      randDirc = randomInteger(0,3);
      while ((*(tCom->arrivalDirection) == randDirc) ){
        randDirc = randomInteger(0,3);
      }
      static int randSize;
      randSize = randomInteger(1,9);
      int index = 0;
      while (*(tCom->trainDeparting + (index + 1) )!= 0) 
      {
        index = index + 2;
      }
      
      *(tCom->trainDeparting + index) = randDirc;
      *(tCom->trainDeparting + (index + 1)) = randSize;
      *(tCom->checkTrain) = FALSE;
      vTaskSuspend(TC);
    }
    vTaskDelay(taskdelay);
  }
  return;
}

void vDepartingTrain(void* train)
{
  while (1)
  {
    //task3 = uxTaskGetStackHighWaterMark(NULL);
    static int countD = 0;
    
    departingTrainData *departingT = (departingTrainData*)train;
    if (*(departingT->departingTrainFlag)) { 
      switch (*(departingT->departureDirection)) {      
      case 0:
        switch (countD % 6) {
        case 0:
        case 1:
        case 2:
          if (globalStartDisplay){
            RIT128x96x4StringDraw("North Train Departing", 0, 85, 15);
          }
          break;
          
        default:
          RIT128x96x4StringDraw("                      ", 0, 85, 15);
          break;
        }
        
        switch (countD % 14) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 6:
        case 7:
        case 10:
        case 11:
          // Enable the PWM0 and PWM1 output signals.
          PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
          // Enable the PWM generator.
          PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          break;
          
        default:
          PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
          PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          break;
        }
        countD++;  
        break;   
      case 1: 
        switch ( countD % 6) {
        case 0:
        case 1:
        case 2:
          if (globalStartDisplay)
          {
            RIT128x96x4StringDraw("South Train Departing", 0, 85, 15);
          }
          break;
          
        default:
          RIT128x96x4StringDraw("                      ", 0, 85, 15);
          break;
        }
        
        switch (countD % 16) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 8:
        case 9:
        case 12:
        case 13:
          // Enable the PWM0 and PWM1 output signals.
          PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
          // Enable the PWM generator.
          PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          break;
          
        default:
          PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
          PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          break;
        }
        countD++;  
        break;
      case 2:
        
        switch (countD % 8) {
        case 0:
        case 1:
        case 2:
        case 3:
          if (globalStartDisplay)
          {
            RIT128x96x4StringDraw("East  Train Departing", 0, 85, 15);
          }
          break;
          
        default:
          RIT128x96x4StringDraw("                      ", 0, 85, 15);
          break;
        }
        switch (countD % 18) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 10:
        case 11:
        case 14:
        case 15:
          // Enable the PWM0 and PWM1 output signals.
          PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
          // Enable the PWM generator.
          PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          break;
          
        default:
          PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
          PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          break;
        }
        countD++;  
        break;
        
        
      case 3:
        switch (countD % 4) {
        case 0:
        case 1:
          if (globalStartDisplay)
          {
            RIT128x96x4StringDraw("West  Train Departing", 0, 85, 15);
          }
          break;
          
        default:
          RIT128x96x4StringDraw("                      ", 0, 85, 15);
          break;
        }   
        switch (countD % 20) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 12:
        case 13:
        case 16:
        case 17:
          // Enable the PWM0 and PWM1 output signals.
          PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
          // Enable the PWM generator.
          PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          break;
          
        default:
          PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
          PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          break;
        }
        countD++;  
        break;
      }
    }
    vTaskDelay(taskdelay);
  }
  return;
}

// |--------------------------------------------------------|
// | switchControl task                                     |
// |--------------------------------------------------------|
void vSwitchControl(void* train)
{
  while(1)
  {
    //task4 = uxTaskGetStackHighWaterMark(NULL);
    globalCount++;
    switchControlData *SWData = (switchControlData*)train;
    static double totalTime = 0;
    static int n = 0;// for gridlock
    static int m = 0;// for gridlock burst
    static double delayValue = 0;
    static double delayTime = 0;
    
    //static Bool waited = FALSE; //never waited for the gridlock before
    //static Bool firstTime = TRUE;
    
    if(*(SWData->trainPresent) > 0){        		  //waiting at intersection and gridlock
      totalTime += 0.5;  
      if (*(SWData->trainSize) == 0){                     // *(SWData->trainSize)==0
        n = randomInteger(-2,2);  
        *(SWData->departingTrainFlag) = TRUE;
        *(SWData->departureDirection) = *(SWData->trainDeparting);  
        *(SWData->trainSize) = *(SWData->trainDeparting + 1);
        *(SWData->traversalTime) = *(SWData->trainSize) * 6; 
	if(n<0){
          *(SWData->gridlock) = TRUE;
          delayValue = -12 * n ;    // 0.2 * n minutes
          delayTime = delayValue;
          m = randomInteger(-2,2);   	                                                // burst
	}
        if (m > 0)                                                              // burst success
        {
          delayValue = 3;
          burstFail = FALSE;
          m = 0;
        } else if (m < 0){                                                      // brust fail, use nuclear weapon
          delayValue = 6;
          burstFail = TRUE;
          m = 0;
        }
      } 
      
      if(*(SWData->gridlock))
      {
        delayValue -= 0.5;   // waited for one minor cycle
        if(0 == delayValue)
        {
          *(SWData->gridlock) = FALSE;
        }
      } else   // train go
      {
        xSemaphoreGive(intersectionLock);
        *(SWData->interLock) = ON;
        
        // if train passed, reset all data
        if (totalTime >  (*(SWData->traversalTime) + delayTime))
        {
          *(SWData->departingTrainFlag) = FALSE;
          *(SWData->interLock) = OFF;
          *(SWData->gridlock) = FALSE;
          *(SWData->traversalTime) = 0;
          *(SWData->trainSize) = 0;
          totalTime = 0;
          delayTime = 0;
          delayValue = 0;
          *(SWData->trainPresent)  = *(SWData->trainPresent) - 1;
          xSemaphoreTake(intersectionLock, 10);
          // stop the sound 
          PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
          PWMGenEnable(PWM0_BASE, PWM_GEN_0);
          RIT128x96x4StringDraw("                     ", 0, 85, 15);
          
          for (int i = 2; i < 15; i+=2){
            *(SWData->trainDeparting + i - 2) = *(SWData->trainDeparting + i);
            *(SWData->trainDeparting + i - 1) = *(SWData->trainDeparting + i + 1);
          }
          *(SWData->trainDeparting + 14) = 0;
          *(SWData->trainDeparting + 15) = 0;
        }
      }
    }   
    vTaskDelay(taskdelay); 
  }   
  return;
}

void vLcdDisplay(void* train)
{
  while(1) 
  {
    //task5 = uxTaskGetStackHighWaterMark(NULL);
    lcdDisplayData *lcdData = (lcdDisplayData*)train;
    static int move = 0;
    if (*(lcdData->trainPresent) > 0 && gridlock)
    {  
      if(burstFail)
      {
        printBurst();
        printX();
        
        for(int i = 0; i < (27 + move); i++)
        {    //shift to next line
          printSpace();
        }
        printNuclear();
        for(int i = 0; i < (28 - move); i++){    
          printSpace();
          
        }
        move += 2;
      } else if(!burstFail)
      {
        printBurst();
        printO();
        for(int i = 0; i < 67; i++)
        {    //shift to next line
          printSpace();
        }
      }
      
    } else if(*(lcdData->trainPresent) > 0 && *(lcdData->departingTrainFlag))
    {
      move = 0;
      // first line
      printSpace();    
      printNumber(*(lcdData->trainPresent)); //1
      printSpace();                   //1
      switch(*(lcdData->departureDirection)){ //1
      case 0:
        printNorth();
        break;
      case 1:
        printSouth();
        break;
      case 2:
        printEast();
        break;
      case 3:
        printWest();
        break;  
      }
      printSpace();                           //1
      printinterLock();  //2                          
      if (*(lcdData->interLock) == ON)              //4
      {
        printOn();
        printSpace();
        printSpace();
      }else 
      {
        printOff();
        printSpace();
      }
      
      printGridLock();       //2                     
      if (*(lcdData->gridlock))              //4
      {
        printOn();
        printSpace();
        printSpace();
      }else 
      {
        printOff();
        printSpace();
      }
      
      for(int i = 0; i < 23; i++){    //shift to next line
        printSpace();
      }
      // Second line
      printSpace(); //1
      printSize();                    //5
      printNumber(*(lcdData->trainSize));     //1
      printSpace();                           //1
      printTime();                            //5
      printNumber((*(lcdData->traversalTime)/10)%10);//1
      printNumber(*(lcdData->traversalTime)%10);//1
      for(int i = 0; i < 25; i++){    
        printSpace();
      }
    } else //if (!*(lcdData->departingTrainFlag))
    {
      lcdClear();
    }
    vTaskDelay(taskdelay * 3);
  }
  return;
}

void printOn()
{
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  
}

void printOff()
{
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
}

void printGridLock()
{
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
}

void printinterLock()
{
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
}

void printNorth()
{
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e       
  
}

void printSouth()
{
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e       
  
}

void printWest()
{
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e       
  
}

void printEast()
{
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e       
  
}

void printSize()
{
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
}

void printTime()
{
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
}

void lcdClear()
{
  
  
  //clear
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00); //4
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
}

void lcdSR()
{
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7 S/C
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6 L/R
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0x00); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
}

void printSpace()
{
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
}


void printNumber(short number)
{
  switch(number) {
  case 0:
    delay(20);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e    
    break;
    
  case 1:
    delay(20);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    break;
    
  case 2:
    delay(20);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    break;      
    
  case 3:
    delay(20);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    break;
  case 4:
    delay(20);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    break;
  case 5:
    delay(20);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    break;
  case 6:
    delay(20);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    break;
    
  case 7:
    delay(20);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    break;
  case 8:
    delay(20);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    break;
  case 9:
    delay(20);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
    
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
    break;
  }
}

void printBurst()
{
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
}

void printO()
{
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
}

void printX()
{
  delay(20);                            //F
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //A
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //I
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //L
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //U
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //R
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //E
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
}

void printNuclear()
{
  delay(20);                        //> 0011 1110
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //= 0011 1101
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //( 0010 1000
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //N 0100 1110
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //U 0101 0101
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //C
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //L
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //E
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //A
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //R
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            // )
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0x00); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  delay(20);                            //>
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0x00); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x00); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0xFF); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
  
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, 0xFF); //7
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0xFF); //6
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, 0xFF); //5
  GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x00); //4
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6, 0xFF); //RS
  
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0xFF); //e
  GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_7, 0x00); //e
}
// |--------------------------------------------------------|
// | OLEDDisplay task                                       |
// |--------------------------------------------------------|
void vOledDisplay(void *train)
{
  while(1)
  {
    //task6 = uxTaskGetStackHighWaterMark(NULL);
    // re-cast the task argument as pointer to the data structure type
    oledDisplayData *disData = (oledDisplayData*)train;
    
    if(OLEDChanged && globalStartDisplay)
    {
      // Screen 1 (home)
      
      
      switch (*disData->mode)
      { 
      case 0: //home
        RIT128x96x4StringDraw("Status", 30, 12, 15);
        RIT128x96x4StringDraw("Annunciation", 30, 48, 15);
        switch ((*disData->scroll) % 2)
        {
        case 0:
          RIT128x96x4StringDraw(">>", 15, 12, 15);
          RIT128x96x4StringDraw("  ", 15, 48, 15);
          break;
        case 1:
          RIT128x96x4StringDraw("  ", 15, 12, 15);
          RIT128x96x4StringDraw(">>", 15, 48, 15);
          break;
        }
        break;
        
      case 1: //status
        switch (*disData->statusSelection)
        {
        case 0:
          RIT128x96x4StringDraw("North Train", 30, 12, 15);
          RIT128x96x4StringDraw("South Train", 30, 24, 15);
          RIT128x96x4StringDraw("East  Train", 30, 36, 15);
          RIT128x96x4StringDraw("West  Train     ", 30, 48, 15);
          switch ((*disData->scroll) % 4)
          {
          case 0:
            RIT128x96x4StringDraw(">>", 15, 12, 15);
            RIT128x96x4StringDraw("  ", 15, 24, 15);
            RIT128x96x4StringDraw("  ", 15, 36, 15);
            RIT128x96x4StringDraw("  ", 15, 48, 15);
            break;
          case 1:
            RIT128x96x4StringDraw("  ", 15, 12, 15);
            RIT128x96x4StringDraw(">>", 15, 24, 15);
            RIT128x96x4StringDraw("  ", 15, 36, 15);
            RIT128x96x4StringDraw("  ", 15, 48, 15);
            break;          
          case 2:
            RIT128x96x4StringDraw("  ", 15, 12, 15);
            RIT128x96x4StringDraw("  ", 15, 24, 15);
            RIT128x96x4StringDraw(">>", 15, 36, 15);
            RIT128x96x4StringDraw("  ", 15, 48, 15);
            break;
          case 3:
            RIT128x96x4StringDraw("  ", 15, 12, 15);
            RIT128x96x4StringDraw("  ", 15, 24, 15);
            RIT128x96x4StringDraw("  ", 15, 36, 15);
            RIT128x96x4StringDraw(">>", 15, 48, 15);
            break;
          }//end cursor
          break;
          
        case 1:
          if(*disData->arrivalDirection == 0)
          {
            RIT128x96x4StringDraw("North Arriving  ", 15, 0, 15);
            RIT128x96x4StringDraw("Arriving No: ", 15, 12, 15);
            char TrainA0[2];
            char TrainA1[2];
            TrainA0[0] = (int)(*(disData->trainArriving)/10 )+ '0';
            TrainA0[1] = '\0';
            TrainA1[0] = (int)(*(disData->trainArriving)%10) + '0';
            TrainA1[1] = '\0';
            RIT128x96x4StringDraw(TrainA0, 90, 12, 15);
            RIT128x96x4StringDraw(TrainA1, 96, 12, 15);
            
            if (!*(disData->departingTrainFlag))
            {
              RIT128x96x4StringDraw("No Departing     ", 15, 24, 15);
            } else if (*disData->departureDirection == 0)
            {
              RIT128x96x4StringDraw("North Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 1)
            {
              RIT128x96x4StringDraw("South Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 2)
            {
              RIT128x96x4StringDraw("East  Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 3)
            {
              RIT128x96x4StringDraw("West  Departing  ", 15, 24, 15);
            }
            
            RIT128x96x4StringDraw("Train Present: ", 15, 36, 15);
            if(*(disData->trainPresent) > 0){
              RIT128x96x4StringDraw("Yes", 100, 36, 15);
            } else{
              RIT128x96x4StringDraw("No ", 100, 36, 15);
            }
            
            RIT128x96x4StringDraw("Lock:          ", 15, 48, 15);
            if(*(disData->interLock) == ON) 
            {
              RIT128x96x4StringDraw("On", 46, 48, 15);
            }
            
            
            RIT128x96x4StringDraw("TSize: ", 15, 60, 15);
            char TrainSZ[2];
            TrainSZ[0] = *(disData->trainSize) + '0';
            TrainSZ[1] = '\0';
            RIT128x96x4StringDraw(TrainSZ, 52, 60, 15);
            
            RIT128x96x4StringDraw("TTime: ", 70, 60, 15);
            char TTime0[2];
            char TTime1[2];
            TTime0[0] = (int)(*(disData->traversalTime)/10) + '0';
            TTime0[1] = '\0';
            TTime1[0] = (int)(*(disData->traversalTime)%10) + '0';
            TTime1[1] = '\0';
            RIT128x96x4StringDraw(TTime0, 106, 60, 15);
            RIT128x96x4StringDraw(TTime1, 112, 60, 15);
          }
          else
          {
            RIT128x96x4StringDraw("                 ", 0, 0, 15);
            RIT128x96x4StringDraw("                          ", 0, 12, 15);
            RIT128x96x4StringDraw("                          ", 0, 48, 15);
            RIT128x96x4StringDraw("                          ", 0, 60, 15);
            RIT128x96x4StringDraw("     No Train Arrive      ", 0, 24, 15);
            RIT128x96x4StringDraw("        From North        ", 0, 36, 15);
          }
          break;
          
        case 2:
          if(*disData->arrivalDirection == 1)
          {
            RIT128x96x4StringDraw("South Arriving  ", 15, 0, 15);
            RIT128x96x4StringDraw("Arriving No: ", 15, 12, 15);
            char TrainA0[2];
            char TrainA1[2];
            TrainA0[0] = (int)(*(disData->trainArriving)/10 )+ '0';
            TrainA0[1] = '\0';
            TrainA1[0] = (int)(*(disData->trainArriving)%10) + '0';
            TrainA1[1] = '\0';
            RIT128x96x4StringDraw(TrainA0, 90, 12, 15);
            RIT128x96x4StringDraw(TrainA1, 96, 12, 15);
            
            if (!*(disData->departingTrainFlag))
            {
              RIT128x96x4StringDraw("No Departing     ", 15, 24, 15);
            } else if (*disData->departureDirection == 0)
            {
              RIT128x96x4StringDraw("North Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 1)
            {
              RIT128x96x4StringDraw("South Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 2)
            {
              RIT128x96x4StringDraw("East  Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 3)
            {
              RIT128x96x4StringDraw("West  Departing  ", 15, 24, 15);
            }
            
            RIT128x96x4StringDraw("Train Present: ", 15, 36, 15);
            if(*(disData->trainPresent) > 0){
              RIT128x96x4StringDraw("Yes", 100, 36, 15);
            } else{
              RIT128x96x4StringDraw("No ", 100, 36, 15);
            }
            
            RIT128x96x4StringDraw("Lock:          ", 15, 48, 15);
            
            if(*(disData->interLock) == ON) 
            {
              RIT128x96x4StringDraw("On", 46, 48, 15);
            }
            
            
            RIT128x96x4StringDraw("TSize: ", 15, 60, 15);
            char TrainSZ[2];
            TrainSZ[0] = *(disData->trainSize) + '0';
            TrainSZ[1] = '\0';
            RIT128x96x4StringDraw(TrainSZ, 52, 60, 15);
            
            RIT128x96x4StringDraw("TTime: ", 70, 60, 15);
            char TTime0[2];
            char TTime1[2];
            TTime0[0] = (int)(*(disData->traversalTime)/10) + '0';
            TTime0[1] = '\0';
            TTime1[0] = (int)(*(disData->traversalTime)%10) + '0';
            TTime1[1] = '\0';
            RIT128x96x4StringDraw(TTime0, 106, 60, 15);
            RIT128x96x4StringDraw(TTime1, 112, 60, 15);
          }
          else
          {
            RIT128x96x4StringDraw("                 ", 0, 0, 15);
            RIT128x96x4StringDraw("                          ", 0, 12, 15);
            RIT128x96x4StringDraw("                          ", 0, 48, 15);
            RIT128x96x4StringDraw("                          ", 0, 60, 15);
            RIT128x96x4StringDraw("     No Train Arrive      ", 0, 24, 15);
            RIT128x96x4StringDraw("        From South        ", 0, 36, 15);
          }
          break;
          
        case 3:
          if(*disData->arrivalDirection == 2)
          {
            RIT128x96x4StringDraw("East  Arriving  ", 15, 0, 15);
            RIT128x96x4StringDraw("Arriving No: ", 15, 12, 15);
            char TrainA0[2];
            char TrainA1[2];
            TrainA0[0] = (int)(*(disData->trainArriving)/10 )+ '0';
            TrainA0[1] = '\0';
            TrainA1[0] = (int)(*(disData->trainArriving)%10) + '0';
            TrainA1[1] = '\0';
            RIT128x96x4StringDraw(TrainA0, 90, 12, 15);
            RIT128x96x4StringDraw(TrainA1, 96, 12, 15);
            
            if (!*(disData->departingTrainFlag))
            {
              RIT128x96x4StringDraw("No Departing      ", 15, 24, 15);
            } else if (*disData->departureDirection == 0)
            {
              RIT128x96x4StringDraw("North Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 1)
            {
              RIT128x96x4StringDraw("South Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 2)
            {
              RIT128x96x4StringDraw("East  Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 3)
            {
              RIT128x96x4StringDraw("West  Departing  ", 15, 24, 15);
            }
            
            RIT128x96x4StringDraw("Train Present: ", 15, 36, 15);
            if(*(disData->trainPresent) > 0){
              RIT128x96x4StringDraw("Yes", 100, 36, 15);
            } else{
              RIT128x96x4StringDraw("No ", 100, 36, 15);
            }
            
            RIT128x96x4StringDraw("Lock:          ", 15, 48, 15);
            
            if(*(disData->interLock) == ON) 
            {
              RIT128x96x4StringDraw("On", 46, 48, 15);
            }
            
            RIT128x96x4StringDraw("TSize: ", 15, 60, 15);
            char TrainSZ[2];
            TrainSZ[0] = *(disData->trainSize) + '0';
            TrainSZ[1] = '\0';
            RIT128x96x4StringDraw(TrainSZ, 52, 60, 15);
            
            RIT128x96x4StringDraw("TTime: ", 70, 60, 15);
            char TTime0[2];
            char TTime1[2];
            TTime0[0] = (int)(*(disData->traversalTime)/10) + '0';
            TTime0[1] = '\0';
            TTime1[0] = (int)(*(disData->traversalTime)%10) + '0';
            TTime1[1] = '\0';
            RIT128x96x4StringDraw(TTime0, 106, 60, 15);
            RIT128x96x4StringDraw(TTime1, 112, 60, 15);
          }
          else
          {
            RIT128x96x4StringDraw("                 ", 0, 0, 15);
            RIT128x96x4StringDraw("                          ", 0, 12, 15);
            RIT128x96x4StringDraw("                          ", 0, 48, 15);
            RIT128x96x4StringDraw("                          ", 0, 60, 15);
            RIT128x96x4StringDraw("     No Train Arrive      ", 0, 24, 15);
            RIT128x96x4StringDraw("        From East         ", 0, 36, 15);
          }
          break;
          
        case 4:
          if(*disData->arrivalDirection == 3)
          {
            RIT128x96x4StringDraw("West  Arriving  ", 15, 0, 15);
            RIT128x96x4StringDraw("Arriving No: ", 15, 12, 15);
            char TrainA0[2];
            char TrainA1[2];
            TrainA0[0] = (int)(*(disData->trainArriving)/10 )+ '0';
            TrainA0[1] = '\0';
            TrainA1[0] = (int)(*(disData->trainArriving)%10) + '0';
            TrainA1[1] = '\0';
            RIT128x96x4StringDraw(TrainA0, 90, 12, 15);
            RIT128x96x4StringDraw(TrainA1, 96, 12, 15);
            
            if (!*(disData->departingTrainFlag))
            {
              RIT128x96x4StringDraw("No Departing     ", 15, 24, 15);
            } else if (*disData->departureDirection == 0)
            {
              RIT128x96x4StringDraw("North Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 1)
            {
              RIT128x96x4StringDraw("South Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 2)
            {
              RIT128x96x4StringDraw("East  Departing  ", 15, 24, 15);
            } else if (*disData->departureDirection == 3)
            {
              RIT128x96x4StringDraw("West  Departing  ", 15, 24, 15);
            }
            
            RIT128x96x4StringDraw("Train Present: ", 15, 36, 15);
            if(*(disData->trainPresent) > 0){
              RIT128x96x4StringDraw("Yes", 100, 36, 15);
            } else{
              RIT128x96x4StringDraw("No ", 100, 36, 15);
            }
            
            RIT128x96x4StringDraw("Lock:          ", 15, 48, 15);
            
            if(*(disData->interLock) == ON) 
            {
              RIT128x96x4StringDraw("On", 46, 48, 15);
            }
            
            RIT128x96x4StringDraw("TSize: ", 15, 60, 15);
            char TrainSZ[2];
            TrainSZ[0] = *(disData->trainSize) + '0';
            TrainSZ[1] = '\0';
            RIT128x96x4StringDraw(TrainSZ, 52, 60, 15);
            
            RIT128x96x4StringDraw("TTime: ", 70, 60, 15);
            char TTime0[2];
            char TTime1[2];
            TTime0[0] = (int)(*(disData->traversalTime)/10) + '0';
            TTime0[1] = '\0';
            TTime1[0] = (int)(*(disData->traversalTime)%10) + '0';
            TTime1[1] = '\0';
            RIT128x96x4StringDraw(TTime0, 106, 60, 15);
            RIT128x96x4StringDraw(TTime1, 112, 60, 15);
          }
          else
          {
            RIT128x96x4StringDraw("                 ", 0, 0, 15);
            RIT128x96x4StringDraw("                          ", 0, 12, 15);
            RIT128x96x4StringDraw("                          ", 0, 48, 15);
            RIT128x96x4StringDraw("                          ", 0, 60, 15);
            RIT128x96x4StringDraw("     No Train Arrive      ", 0, 24, 15);
            RIT128x96x4StringDraw("        From West         ", 0, 36, 15);
          }
          break;
        }//end train status
        break;
        
      case 2: //annunciation
        if(*(disData->gridlock) && (*(disData->departingTrainFlag))) {
          RIT128x96x4StringDraw("Gridlock Warning!!!", 12, 24, 15);
          RIT128x96x4StringDraw("                     ", 15, 12, 15);
          RIT128x96x4StringDraw("                     ", 15, 48, 15);
        } else if (!*(disData->gridlock) && (*(disData->departingTrainFlag))){
          RIT128x96x4StringDraw("Train Passing!!!   ", 12, 24, 15);
          RIT128x96x4StringDraw("                     ", 15, 12, 15);
          RIT128x96x4StringDraw("                     ", 15, 48, 15);
        } else if(!*(disData->departingTrainFlag)){
          RIT128x96x4StringDraw("No Depareture Train", 12, 24, 15);
          RIT128x96x4StringDraw("                     ", 15, 12, 15);
          RIT128x96x4StringDraw("                     ", 15, 48, 15);
        }
        
        int temperatureReading0 = 32 * (*(disData->temperatureBuf)) / 100 + 33;
        double pretempReading0 = 0.384* (double)(*(disData->temperatureBuf + 3)) + 33.0 ;
        char temp0[80];
        sprintf(temp0, "%d", temperatureReading0);
        
        
        char temp1[80];
        int temperatureReading1 = 32 * (*(disData->temperatureBuf + 1)) / 100 + 33;
        double pretempReading1 = 0.384* (double)(*(disData->temperatureBuf + 4)) + 33.0 ;
        sprintf(temp1, "%d", temperatureReading1);
        
        
        char temp2[80];    
        int temperatureReading2 = 32 * (*(disData->temperatureBuf + 2)) / 100 + 33;
        double pretempReading2 = 0.384* (double)(*(disData->temperatureBuf + 5)) + 33.0 ;
        sprintf(temp2 , "%d", temperatureReading2);
        if(*(disData->trainArriving) > 0 || *(disData->departingTrainFlag))
        {
          char freq0[80];
          sprintf(freq0, "%d", noiseFrequency);
          RIT128x96x4StringDraw("Freq:",0,48,15);
          RIT128x96x4StringDraw(freq0, 30, 48, 15);
        } else {
          RIT128x96x4StringDraw("                               " ,0,48,15);  
        }
        if(*(disData->departingTrainFlag))
        {
          RIT128x96x4StringDraw("Temp:",0,60,15);
          RIT128x96x4StringDraw(temp0 ,30,60,15);  
          RIT128x96x4StringDraw(temp1 ,60,60,15); 
          RIT128x96x4StringDraw(temp2 ,90,60,15); 
        } else {
          RIT128x96x4StringDraw("                               " ,0,60,15);  
        }
        
        if ((temperatureReading0 > pretempReading0
             || temperatureReading1> pretempReading1
               ||temperatureReading2>pretempReading2)
            && (*(disData->temperatureBuf + 3) != 0 )
              && (*(disData->temperatureBuf + 4) != 0)
                && (*(disData->temperatureBuf + 5) != 0))
        {
          RIT128x96x4StringDraw("TOO HOT!!",0,0,15); // alarm
        } else 
        {
          RIT128x96x4StringDraw("          ",0,0,15); // alarm
        }
        break;
      } //end mode
      vTaskDelay(taskdelay);
    }
  }
  return;
}

void vLocalKeypad(void* train)
{
  while(1)
  {
    //task7 = uxTaskGetStackHighWaterMark(NULL);
    localKeypadData *keypadData = (localKeypadData*)train;
    modeSelect();
    trainStatus();
    scrollNselect();  
    // Home
    if(0x10 == GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_4) && key0)
    {
      OLEDChanged = TRUE;
      key0 = 0;
      *keypadData->mode = 0;
      *keypadData->select = 0;
      *keypadData->annunciation = FALSE;
      *keypadData->scroll = 0;
      *keypadData->statusSelection = 0;
      for (int i = 0; i < 101; i+=5)
      {
        for (int j = 0; j < 130; j+=5)
        {
          RIT128x96x4StringDraw(" ", j, i, 15);
        }
      }
    }
    if(0x00 == GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_4) && !key0){
      key0 = 1;
    }
    vTaskDelay(taskdelay);
  }
  return;
}

void vTemperatureMeasurement(void* train)
{
  while(1)
  {
    
    //task8 = uxTaskGetStackHighWaterMark(NULL);
    temperatureData *tempData = (temperatureData*) train;
    // int maxTemp;
    
    if (measure&& *(tempData->departingTrainFlag) && *(tempData->globalStartMeasure)) {
      ADCIntClear(ADC0_BASE, 0);
      // Trigger the ADC conversion.
      ADCProcessorTrigger(ADC0_BASE, 0);
      // Wait for conversion to be completed.
      while(!ADCIntStatus(ADC0_BASE, 0, false)) {}
      // Read ADC Value.
      ADCSequenceDataGet(ADC0_BASE, 0, &ulValue0);
      
      delay(5);
      ADCIntClear(ADC0_BASE, 1);
      // Trigger the ADC conversion.
      ADCProcessorTrigger(ADC0_BASE, 1);
      // Wait for conversion to be completed.
      while(!ADCIntStatus(ADC0_BASE, 1, false)) {}
      // Read ADC Value.
      ADCSequenceDataGet(ADC0_BASE, 1, &ulValue1);     
      
      delay(5);
      ADCIntClear(ADC0_BASE, 2);
      // Trigger the ADC conversion.
      ADCProcessorTrigger(ADC0_BASE, 2);
      // Wait for conversion to be completed.
      while(!ADCIntStatus(ADC0_BASE, 2, false)) {}
      // Read ADC Value.
      ADCSequenceDataGet(ADC0_BASE, 2, &ulValue2);
      
      
      for (int i = 15; i > 2; i--){
        *(tempData->temperatureBuf + i) = *(tempData->temperatureBuf + (i - 3));
      }
      *(tempData->temperatureBuf) = ulValue0 * 341 / 122;    //convert to millivolts
      *(tempData->temperatureBuf + 1) = ulValue1 * 341 /122; 
      *(tempData->temperatureBuf + 2) = ulValue2 * 341 /122;
      
      measure = FALSE;
    }
    vTaskDelay(taskdelay);
  }
  return;
}

void vNoiseCapture(void* train)
{
  while(1)
  {
    //task9 = uxTaskGetStackHighWaterMark(NULL);
    noiseCaptureData *captureData = (noiseCaptureData*) train;
    static int c = 0;
    
    if(c < 256 && captureNoise && *(captureData->globalStartMeasure))
    {
      captureNoise = FALSE;
      ADCIntClear(ADC0_BASE, 3);
      // Trigger the ADC conversion.
      ADCProcessorTrigger(ADC0_BASE, 3);
      // Wait for conversion to be completed.
      while(!ADCIntStatus(ADC0_BASE, 3, false)) {}
      // Read ADC Value.
      ADCSequenceDataGet(ADC0_BASE, 3, &ulValue3);
      //noiseCaptureBuf [c]= ulValue3;
      noiseCaptureBuf [c]= (signed short)(0.063* ulValue3 - 32);
      c++;
    }
    
    if (c == 256){
      c = 0;
      processNoise = TRUE;
    }
  }
  return;
}

void vNoiseProcessing(void* train)
{
  while(1)
  {
    //task10 = uxTaskGetStackHighWaterMark(NULL);
    noiseProcessingData *processingData = (noiseProcessingData*) train;
    static signed int peakIndex = 0;  
    //static int freq = 0;
    static int read = 0;
    if (processNoise){
      if (read%5 == 0){
        processNoise = FALSE;
        signed int imagBuf[256];
        for (int i = 0; i < 256; i ++)
        {
          imagBuf[i] = 0;
        }
        peakIndex = optfft(noiseCaptureBuf, imagBuf);
        noiseFrequency = peakIndex * 16000 / 256 ;
        for (int i = 15; i > 0; i--){
          *(processingData->noiseTransBuf + i) = *(processingData->noiseTransBuf + (i - 1));
        }
        *(processingData->noiseTransBuf) = noiseFrequency;
        measure = FALSE;
        for(int i = 0; i < 256; i++)
        {
          noiseCaptureBuf[i] = 0;
        }
      }
      read++;
    }
    vTaskDelay(500); 
  }
  return;
}

/*******************************************************************************/
/* optfft.c                                                                    */
/*                                                                             */
/* An optimized version of the fft function using only 16-bit integer math.    */
/*                                                                             */
/* Optimized by Brent Plump                                                    */
/* Based heavily on code by Jinhun Joung                                       */
/*                                                                             */
/* - Works only for input arrays of 256 length.                                */
/* - Requires two arrays of 16-bit ints.  The first contains the samples, the  */
/*   second contains all zeros.  The samples range from -31 to 32              */
/* - Returns the index of the peak frequency                                   */
/*******************************************************************************/
#include "optfft.h"

#define ABS(x)  (((x)<0)?(-(x)):(x))
#define CEILING(x) (((x)>511)?511:(x))

signed int optfft(signed int real[256], signed int imag[256]) {
  
  signed int i, i1, j, l, l1, l2, t1, t2, u;
  
#include "tables.c"
  
  /* Bit reversal. */
  /*Do the bit reversal */
  l2 = 128;
  i=0;
  for(l=0;l<255;l++) {
    if(l < i) {
      j=real[l];real[l]=real[i];real[i]=j;
    }
    l1 = l2;
    while (l1 <= i){
      i -= l1;
      l1 >>= 1;
    }
    i += l1;
  }
  
  /* Compute the FFT */
  u = 0;
  l2 = 1;
  for(l=0;l<8;l++){
    l1 = l2;
    l2 <<= 1;
    for(j=0;j<l1;j++){
      for(i=j;i<256;i+=l2){
        i1 = i + l1;
        t1 = (u1[u]*real[i1] - u2[u]*imag[i1])/32; 
        t2 = (u1[u]*imag[i1] + u2[u]*real[i1])/32;
        real[i1] = real[i]-t1;
        imag[i1] = imag[i]-t2;
        real[i] += t1;
        imag[i] += t2;
      }
      u++;
    }
  }
  
  /* Find the highest amplitude value */
  /* start at index 1 because 0 can hold high values */
  j=1;
  l=0;	   	
  for ( i=1; i<(128); i++ ) {
    l1 = square[CEILING(ABS(real[i]))]+square[CEILING(ABS(imag[i]))];
    if (l1 > l) {
      j = i;
      l = l1;
    }
  }
  return (j);
}


void vSerialCom(void* train)
{
  while(1)
  {
    //task11 = uxTaskGetStackHighWaterMark(NULL);
    ///============================================================
    // HYPERTERM setup (UTAR)
    //============================================================
    
    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    
    //
    // Enable processor interrupts.
    //
    IntMasterEnable();
    
    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
    
    //
    // Enable the UART interrupt.
    //
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    
    
    serialComData *serialData = (serialComData*)train;
    if(startSerial) {
      
      
      ////////////////////////////
      UARTSend((unsigned char *)"Train Present: " , 16);
      unsigned char TP[2];
      TP[0] = (*(serialData->trainPresent)) + 48;
      if (*(serialData->trainPresent) < 10) {
        UARTSend((unsigned char *)TP, 1);
      } else 
      {
        int temp4 = (*(serialData->trainArriving));
        char twoDigit4[3];  
        int i_4 = 1; 
        while ( temp4 != 0) { 
          int digt4 = temp4 % 10; 
          twoDigit4[i_4] =  digt4 + 48; 
          temp4 = temp4 / 10;  
          i_4--; 
        }
        UARTSend((unsigned char *)twoDigit4, 2);
      }
      
      UARTSend((unsigned char *)"\n\r", 2);
      
      // print current direction and train size
      UARTSend((unsigned char *) "Departure Direction: ", 22);
      if((*serialData->departingTrainFlag)){
        switch(*(serialData->departureDirection)) {        
        case 0:
          UARTSend((unsigned char *)"NORTH\n\r", 8);
          UARTSend((unsigned char *)"Train Size: ", 13);
          unsigned char TS[1];
          TS[1] = (*(serialData->trainSize)) + 48;
          UARTSend((unsigned char *)TS, 1);
          UARTSend((unsigned char *)"\n\r", 2);
          break;
          
        case 1:
          UARTSend((unsigned char *)"SOUTH\n\r", 8);
          UARTSend((unsigned char *)"Train Size: ", 13);
          unsigned char TS2[1];
          TS2[1] = (*(serialData->trainSize)) + 48;
          UARTSend((unsigned char *)TS2, 1);
          UARTSend((unsigned char *)"\n\r", 2);
          break;
        case 2:
          
          UARTSend((unsigned char *)"EAST\n\r", 7);
          UARTSend((unsigned char *)"Train Size: ", 13);
          unsigned char TS3[1];
          TS3[1] = (*(serialData->trainSize)) + 48;
          UARTSend((unsigned char *)TS3, 1);
          UARTSend((unsigned char *)"\n\r", 2);
          break; 
        case 3:
          
          UARTSend((unsigned char *)"WEST\n\r", 7);
          UARTSend((unsigned char *)"Train Size: ", 13);
          unsigned char TS4[1];
          TS4[1] = (*(serialData->trainSize)) + 48;
          UARTSend((unsigned char *)TS4, 1);
          UARTSend((unsigned char *)"\n\r", 2);
          break;
        }
      } else
      {
        UARTSend((unsigned char *)"NA\n\r", 4);
        UARTSend((unsigned char *)"Train Size: ", 13);
        UARTSend((unsigned char *)"NA\n\r", 4);
      }
      // PRINT TRAVERSAL TIME
      UARTSend((unsigned char *)"Traversal Time: " , 17);
      
      int temp2 = (*(serialData->traversalTime));
      char twoDigit2[3];  
      int i_2 = 1; 
      while ( temp2 != 0) { 
        int digt2 = temp2 % 10; 
        twoDigit2[i_2] =  digt2 + 48; 
        temp2 = temp2 / 10;  
        i_2 --; 
      }
      
      if (*(serialData->traversalTime) == 0) {
        UARTSend((unsigned char *)"NA\n\r", 4);
      } else {
        UARTSend((unsigned char *)twoDigit2, 2);
        UARTSend((unsigned char *)"\n\r", 2);
      }
      
      UARTSend((unsigned char *)"Intersection Availability: " , 28);
      
      if(*serialData->departingTrainFlag){
        UARTSend((unsigned char *)"Not Available!\n\r", 16);     
      } else {
        UARTSend((unsigned char *)"Available!\n\r", 12);     
      }
      
      // print train arriving 
      UARTSend((unsigned char *)"Train Arriving: " , 17);
      unsigned char TA[2];
      TA[0] = (*(serialData->trainArriving)) + 48;
      if (*(serialData->trainArriving) < 10) {
        UARTSend((unsigned char *)TA, 1);
      } else 
      {
        int temp = (*(serialData->trainArriving));
        char twoDigit[3];  
        int i = 1; 
        while ( temp != 0) { 
          int digt = temp % 10; 
          twoDigit[i] =  digt + 48; 
          temp = temp / 10;  
          i--; 
        }
        UARTSend((unsigned char *)twoDigit, 2);
      }
      UARTSend((unsigned char *)"\n\r", 2);
      
      UARTSend((unsigned char *)"Arrival Direction: ", 20);
      switch(*(serialData->arrivalDirection)) {
      case 4:
        UARTSend((unsigned char *)"NA\n\r", 4);
        break;
      case 0:
        UARTSend((unsigned char *)"NORTH\n\r", 8);
        break;
      case 1:
        UARTSend((unsigned char *)"SOUTH\n\r", 8);
        break;
      case 2:
        UARTSend((unsigned char *)"EAST\n\r", 7);
        break; 
      case 3:
        UARTSend((unsigned char *)"WEST\n\r", 7);
        break;
      }
      
      
      UARTSend((unsigned char *)"Arrival Time: ", 15);
      if(*(serialData->arrivalTime) != 0)
      {
        int temp3 = (*(serialData->arrivalTime));
        char twoDigit3[3];  
        int i_3 = 1; 
        while ( temp3 != 0) { 
          int digt3 = temp3 % 10; 
          twoDigit3[i_3] =  digt3 + 48; 
          temp3 = temp3 / 10;  
          i_3 --; 
        }    
        UARTSend((unsigned char *)twoDigit3, 2);
        UARTSend((unsigned char *)"\n\r", 2);
      } else 
      {
        UARTSend((unsigned char *)"NA\n\r", 4);
      }
      UARTSend((unsigned char *)"Gridlock: ", 12);
      if (*(serialData->gridlock))
      {
        UARTSend((unsigned char *)"YES", 4);
        //UARTSend((unsigned char *)"\n\r", 2);
      } else {
        UARTSend((unsigned char *)"NO", 3);
      }
      
      UARTSend((unsigned char *)"\n\r", 2);
      
      
      // PRINT NOISE
      UARTSend((unsigned char *)"Noise: ", 8);
      if (noiseTransBuf[0] != NULL && noiseTransBuf[0] != 0) 
      {
        int noisePrint = noiseTransBuf[0];
        if (noisePrint > 999)
        {
          char printNoise[5]; 
          int i_4 = 3; 
          while ( noisePrint != 0) { 
            int digt4 = noisePrint % 10; 
            printNoise[i_4] =  digt4 + 48; 
            noisePrint = noisePrint / 10;  
            i_4 --; 
          }    
          UARTSend((unsigned char *)printNoise, 4);
          UARTSend((unsigned char *)"\n\r", 2); 
        } else
        {
          char printNoise1[4]; 
          int i_T4 = 2; 
          while ( noisePrint != 0) { 
            int digtT4 = noisePrint % 10; 
            printNoise1[i_T4] =  digtT4 + 48; 
            noisePrint = noisePrint / 10;  
            i_T4 --; 
          }    
          UARTSend((unsigned char *)printNoise1, 3);
          UARTSend((unsigned char *)"\n\r", 2);
          
          
          
        }
      } else 
      {
        UARTSend((unsigned char *)"NA\n\r", 4); 
      }
      
      
      // print tempertature  
      UARTSend((unsigned char *)"Temperature: ", 14);
      int temperatureReading0 = 32 * (temperatureBuf[0]) / 100 + 33;
      int temperatureReading1 = 32 * (temperatureBuf[1])  / 100 + 33;
      int temperatureReading2 = 32 * (temperatureBuf[2]) / 100 + 33;
      
      // print temperature0
      if (temperatureReading0 > 99)
      { 
        char printTemp0[4]; 
        int i_5 = 2; 
        while ( temperatureReading0 != 0) { 
          int digt5 = temperatureReading0 % 10; 
          printTemp0[i_5] =  digt5 + 48; 
          temperatureReading0 = temperatureReading0 / 10;  
          i_5 --; 
        } 
        UARTSend((unsigned char *)printTemp0, 3);
        UARTSend((unsigned char *)" ", 2);
      } else 
      {
        int temp6 = temperatureReading0;
        char twoDigit6[3];  
        int i_6 = 1; 
        while ( temp6 != 0) { 
          int digt6 = temp6 % 10; 
          twoDigit6[i_6] =  digt6 + 48; 
          temp6 = temp6 / 10;  
          i_6 --; 
        }    
        UARTSend((unsigned char *)twoDigit6, 2);
        UARTSend((unsigned char *)" ", 2);
      }
      
      // print temperature1
      if (temperatureReading1 > 99)
      { 
        char printTemp1[4]; 
        int i_7 = 2; 
        while ( temperatureReading1 != 0) { 
          int digt7 = temperatureReading1 % 10; 
          printTemp1[i_7] =  digt7 + 48; 
          temperatureReading1 = temperatureReading1 / 10;  
          i_7 --; 
        } 
        UARTSend((unsigned char *)printTemp1, 3);
        UARTSend((unsigned char *)" ", 2);
      } else 
      {
        int temp8 = temperatureReading1;
        char twoDigit8[3];  
        int i_8 = 1; 
        while ( temp8 != 0) { 
          int digt8 = temp8 % 10; 
          twoDigit8[i_8] =  digt8 + 48; 
          temp8 = temp8 / 10;  
          i_8 --; 
        }    
        UARTSend((unsigned char *)twoDigit8, 2);
        UARTSend((unsigned char *)" ", 2);
      }
      // print temperature2
      if (temperatureReading2 > 99)
      { 
        char printTemp2[4]; 
        int i_9 = 2; 
        while ( temperatureReading2 != 0) { 
          int digt9 = temperatureReading2 % 10; 
          printTemp2[i_9] =  digt9 + 48; 
          temperatureReading2 = temperatureReading2 / 10;  
          i_9 --; 
        } 
        UARTSend((unsigned char *)printTemp2, 3);
        
      } else 
      {
        int tempT1 = temperatureReading2;
        char twoDigitT1[3];  
        int i_T1 = 1; 
        while ( tempT1 != 0) { 
          int digtT1 = tempT1 % 10; 
          twoDigitT1[i_T1] =  digtT1 + 48; 
          tempT1 = tempT1 / 10;  
          i_T1 --; 
        }    
        UARTSend((unsigned char *)twoDigitT1, 2);
        
      }
      UARTSend((unsigned char *)"\n\r", 2);
      
      
      
      
      UARTSend((unsigned char *)"\n\r", 2);
    }
    vTaskDelay(taskdelay);
  }
  return;
  
}

void vCommand(void* train)
{
  while(1)
  {  
    //task1 = uxTaskGetStackHighWaterMark(NULL);
    static int countDisplay = 0;
    commandData *comData = (commandData*)train;
    if (strstr(readCommand,"S"))
    {
      *(comData->globalStartMeasure) = TRUE;
      commandAccept = 1;
      
    } else if (strstr(readCommand,"P"))
    {
      *(comData->globalStartMeasure) = FALSE;
      commandAccept = 1;
      
    } else if (strstr(readCommand, "D"))
    {
      readCommand[0] = NULL;
      countDisplay ++;
      OLEDChanged = TRUE;
      execute = TRUE;
      
      if (countDisplay % 2 == 1)
      {
        globalStartDisplay = FALSE;
        for (int i = 0; i < 101; i+=5)
        {
          for (int j = 0; j < 130; j+=5)
          {
            RIT128x96x4StringDraw(" ", j, i, 15);
          }
        }
      }else 
      {
        globalStartDisplay = TRUE;
      }
    } else if (strstr(readCommand, "M")){
      
      if (strstr(readCommand, "temp")){
        // show temperatureBuf
        *(comData->showRecentData) = 1;
        commandAccept = 1;
        
      } else if (strstr(readCommand, "noise")){
        //show noiseBuf
        *(comData->showRecentData) = 2;
        commandAccept = 1;
        
      } else {
        // show error
        commandAccept = 2;
      }
    } else if (readCommand[0] == NULL)
    {
      commandAccept = 0;
    } else 
    {
      commandAccept = 2;
    }
    //readCommand[0] = NULL;
    //vTaskDelay(taskdelay);
  } 
}
// |--------------------------------------------------------|
// | delay Function                                         |
// | Fome EE 472 WEB SITE                                   |
// | Assignment 1, project1a-2014.c                         |
// |--------------------------------------------------------| 
void delay(unsigned long aValue)
{
  volatile unsigned long i = 0;
  
  volatile unsigned int j = 0;
  
  for (i = aValue; i > 0; i--)
  {
    for (j = 0; j < 100; j++);
  }
  
  return;
}

int randomInteger(int low, int high)
{       
  static int seed = 8464863;
  double randNum = 0.0;
  int multiplier = 2743;
  int addOn = 5923;
  double max = INT_MAX + 1.0;
  
  int retVal = 0;
  
  if (low > high)
    retVal = randomInteger(high, low);
  else
  {
    seed = seed*multiplier + addOn;
    randNum = seed;
    
    if (randNum <0)
    {
      randNum = randNum + max;
    }
    
    randNum = randNum/max;
    
    retVal =  ((int)((high-low+1)*randNum))+low;
  }
  seed += 357;
  return retVal;
}

void GPIOFIntHandler(void) { ///sw2
  // Setup the interrupts for the timer timeouts.
  IntEnable(INT_TIMER1A);
  // Enable the timers.
  //
  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER1_BASE, TIMER_A);
  // CLEAR GPIO 
  
  GPIOPinIntClear(GPIO_PORTF_BASE, GPIO_PIN_1);
  addingTrain = TRUE;
}

void GPIOEIntHandler(void) { /// sw3
  // Setup the interrupts for the timer timeouts.
  IntEnable(INT_TIMER1A);
  // Enable the timers.
  //
  TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER1_BASE, TIMER_A);
  // CLEAR GPIO 
  
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);
  measuringTemp = TRUE;
}

void Timer1IntHandler(void)
{
  
  TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  if (addingTrain)
  {
    trainArriving++;
    startSerial = TRUE;
    addingTrain = FALSE;
  }
  if (measuringTemp)
  {
    measure = TRUE;
  }
  
  TimerIntDisable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
  
  TimerDisable(TIMER1_BASE, TIMER_A);
  
}

void Timer2IntHandler(void)
{
  TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
  if(test11)
  {
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, 0x10);
    
  } else if (!test11)
  {
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, 0x00);
  }
  test11 = 1 - test11;
}

void Timer3IntHandler(void)
{
  static int countDistance = 0; 
  TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
  captureNoise = TRUE;
  if (0x20 == GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_5) )
  {
    countDistance ++;
  } else if (0x00 == GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_5) && countDistance > 0 ){    
    remainDistance = (int)(countDistance * 2.35 + 97);
    countDistance = 0;
  }  
}

void
UARTIntHandler(void)
{
  unsigned long ulStatus;
  
  //
  // Get the interrrupt status.
  //
  ulStatus = UARTIntStatus(UART0_BASE, true);
  
  //
  // Clear the asserted interrupts.
  //
  UARTIntClear(UART0_BASE, ulStatus);
  
  //
  // Loop while there are characters in the receive FIFO.
  //
  while(UARTCharsAvail(UART0_BASE))
  {
    //
    // Read the next character from the UART and write it back to the UART.
    //
    UARTCharPutNonBlocking(UART0_BASE, UARTCharGetNonBlocking(UART0_BASE));
  }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
  //
  // Loop while there are more characters to send.
  //
  while(ulCount--)
  {
    //
    // Write the next character to the UART.
    //
    UARTCharPutNonBlocking(UART0_BASE, *pucBuffer++);
    for(int i = 0; i < 3000; i++);
    //delay(5);
  }
}

//---------------------------------
// keypad functions:
//---------------------------------
// Mode Select
void modeSelect(void)
{
  if(mode == 0)
  {
    if(scroll % 2 == 0 && 1 == select)
    {
      mode = 1;
      select = 0;
      scroll = 0;
    }
    else if (scroll % 2 == 1 && 1 == select)
    {
      mode = 2;
      select = 0;
      scroll = 0;
    }   
  }
}

// Train Status
void trainStatus(void)
{
  if (1 == mode && 1 == select){
    statusSelection = (scroll % 4) + 1;
  }  
}

//Scrolling and Selection
void scrollNselect(void)
{    
  if(0x10 == GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_4) && key1)
  {
    scroll++; 
    key1 = 0;
    OLEDChanged = TRUE;
  }  
  if(!key1 && (0x00 == GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_4)))
  {
    key1 = 1;
  } 
  
  //select
  if(0x20 == GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_5) && key2)
  {
    select = 1;
    key2 = 0;
    OLEDChanged = TRUE;
  } 
  if(!key2 && 0x00 == GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_5) )
  {
    key2 = 1;
  }
}
////////////////////////////////////////////////////////////////////////////////////
unsigned short vRemoteCom( void *train )
{
  remoteCommunicationData *remoteData = (remoteCommunicationData*) train;
  // print departure direction
  char departD[6];
  switch (departureDirection)
  {
  case 0 :
    strcpy (departD,"NORTH");
    break;
  case 1:
    strcpy (departD,"SOUTH");
    break;
  case 2:
    strcpy (departD,"EAST");
    break;
  case 3:
    strcpy (departD,"WEST");
    break;
  case 4:
    strcpy (departD,"NA");
    break;
    
  }
  // print intersection status
  char intersection[14];
  if (departingTrainFlag) 
  {
    strcpy (intersection,"NOT AVAILABLE");
  } else{
    strcpy (intersection,"AVAILABLE");
  }
  // print arrival direction
  char arrivalD[6];
  switch (arrivalDirection)
  {
  case 0 :
    strcpy (arrivalD,"NORTH");
    break;
  case 1:
    strcpy (arrivalD,"SOUTH");
    break;
  case 2:
    strcpy (arrivalD,"EAST");
    break;
  case 3:
    strcpy (arrivalD,"WEST");
    break;
  case 4:
    strcpy (arrivalD,"NA");
    
  }
  // print gridlock status
  char grid[4];
  if (gridlock)
  {
    strcpy (grid,"ON");
  } else 
  {
    strcpy (grid,"OFF");
  }  
  
  
  int temperatureReading0 = 32 * (temperatureBuf[0]) / 100 + 33;
  int temperatureReading1 = 32 * (temperatureBuf[1])  / 100 + 33;
  int temperatureReading2 = 32 * (temperatureBuf[2]) / 100 + 33;
  
  // show recent data
  
  char dataName[25] = "Mnoise/Mtemp: ";
  int data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  
  switch (showRecentData)
  {
  case 1:
    strcpy(dataName, "Recent temperature:");
    if (globalStartMeasure)
    {
      for(int i = 0; i < 16; i ++){
        data[i] = 32 * temperatureBuf[i] / 100 + 33;
      }
    }
    break;
  case 2:
    strcpy(dataName, "Recent noise:");
    if(globalStartMeasure)
    {
      for(int i = 0; i < 16; i ++){
        data[i] = noiseTransBuf[i];
      }
    }
    break;
  } 
  // show command execute
  char showExecute[20];
  if (commandAccept == 1)
  {
    strcpy(showExecute, "Command Executed");
    commandAccept = 0;
    
  } else if(commandAccept == 2)
  {
    strcpy(showExecute, "Command Error!!!");
    commandAccept = 0;
    
  } else 
  {
    strcpy(showExecute, "                  ");
    if(execute)
    {
      strcpy(showExecute, "Command Executed");
      execute = FALSE;
    }
  }
  
  sprintf(uip_appdata,
          "<br>Train Present: %d"\
            "<br>Departing Direction: %s"\
              "<br>Train Size: %d"\
                "<br>Traversal Time: %d"\
                  "<br>Intersection Availability: %s"\
                    "<br>Arrival Direction: %s"\
                      "<br>Arrival Distance: %d"\
                        "<br>Gridlock: %s"\
                          "<br>Noise Signature: %d"\
                            "<br>Wheel Temperature: %d %d %d"\
                              "<br>%s %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d"\
                                "<br> %s"\
                                  "<p>"\
                                    "<input type=\"text\" name=\"Command\" value=\"\" size=\"50\">", 
                                    trainPresent, departD, trainSize, traversalTime,
                                    intersection,arrivalD, arrivingTrainDistance[0], grid, noiseTransBuf[0], 
                                    temperatureReading0,temperatureReading1,temperatureReading2, dataName, data[0],
                                    data[1],data[2],data[3],data[4],data[5],data[6],data[7], data[8],
                                    data[9],data[10],data[11],data[12],data[13],data[14],data[15],
                                    showExecute);
  
  return strlen( uip_appdata );
}
/*---------------------------------------------------------------------------*/
/////////////////////////////////////////////////////////////////////////////////////
void vOLEDTask( void *pvParameters )
{
  xOLEDMessage xMessage;
  unsigned portLONG ulY, ulMaxY;
  static portCHAR cMessage[ mainMAX_MSG_LEN ];
  extern volatile unsigned portLONG ulMaxJitter;
  unsigned portBASE_TYPE uxUnusedStackOnEntry;
  const unsigned portCHAR *pucImage;
  
  // Functions to access the OLED. 
  
  void ( *vOLEDInit )( unsigned portLONG ) = NULL;
  void ( *vOLEDStringDraw )( const portCHAR *, unsigned portLONG, unsigned portLONG, unsigned portCHAR ) = NULL;
  void ( *vOLEDImageDraw )( const unsigned portCHAR *, unsigned portLONG, unsigned portLONG, unsigned portLONG, unsigned portLONG ) = NULL;
  void ( *vOLEDClear )( void ) = NULL;
  
  
  vOLEDInit = RIT128x96x4Init;
  vOLEDStringDraw = RIT128x96x4StringDraw;
  vOLEDImageDraw = RIT128x96x4ImageDraw;
  vOLEDClear = RIT128x96x4Clear;
  ulMaxY = mainMAX_ROWS_96;
  pucImage = pucBasicBitmap;
  
  // Just for demo purposes.
  uxUnusedStackOnEntry = uxTaskGetStackHighWaterMark( NULL );
  
  ulY = ulMaxY;
  
  /* Initialise the OLED  */
  vOLEDInit( ulSSI_FREQUENCY );	
  
  while( 1 )
  {
    // Wait for a message to arrive that requires displaying.
    
    xQueueReceive( xOLEDQueue, &xMessage, portMAX_DELAY );
    
    // Write the message on the next available row. 
    
    ulY += mainCHARACTER_HEIGHT;
    if( ulY >= ulMaxY )
    {
      ulY = mainCHARACTER_HEIGHT;
      vOLEDClear();
    }
    
    // Display the message  
    
    sprintf( cMessage, "%s", xMessage.pcMessage);
    
    vOLEDStringDraw( cMessage, 0, ulY, mainFULL_SCALE );
    
  }
}