/*
    FreeRTOS V7.1.0 - Copyright (C) 2011 Real Time Engineers Ltd.
	

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


    This file is part of the FreeRTOS distribution.

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
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include<LPC21xx.h>
#include <stdlib.h>
#include<math.h>
#include<string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Demo application includes. */
#include "partest.h"
#include "flash.h"
#include "comtest2.h"
#include "serial.h"
#include "PollQ.h"
#include "BlockQ.h"
#include "semtest.h"
#include "dynamic.h"
#include "header.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainTX_ENABLE		( ( unsigned portLONG ) 0x00010000 )	/* UART1. */
#define mainRX_ENABLE		( ( unsigned portLONG ) 0x00040000 ) 	/* UART1. */
#define mainBUS_CLK_FULL	( ( unsigned portCHAR ) 0x01 )
#define mainLED_TO_OUTPUT	( ( unsigned portLONG ) 0xff0000 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned portLONG ) 115200 )
#define mainCOM_TEST_LED		( 3 )

/* Priorities for the demo application tasks. */
#define mainLED_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCOM_TEST_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )

/* Constants used by the "check" task.  As described at the head of this file
the check task toggles an LED.  The rate at which the LED flashes is used to
indicate whether an error has been detected or not.  If the LED toggles every
3 seconds then no errors have been detected.  If the rate increases to 500ms
then an error has been detected in at least one of the demo application tasks. */
#define mainCHECK_LED				( 7 )
#define mainNO_ERROR_FLASH_PERIOD	( ( portTickType ) 3000 / portTICK_RATE_MS  )
#define mainERROR_FLASH_PERIOD		( ( portTickType ) 500 / portTICK_RATE_MS  )

/*-----------------------------------------------------------*/

# define SPEED 60
#define PIE 0.0174532925

/*
 * Checks that all the demo application tasks are still executing without error
 * - as described at the top of the file.
 */
static portLONG prvCheckOtherTasksAreStillRunning( void );

/*
 * The task that executes at the highest priority and calls 
 * prvCheckOtherTasksAreStillRunning().  See the description at the top
 * of the file.
 */
static void vErrorChecks(void *pvParameters );

void timer_isr(void)__irq;

void timer_isr1(void)__irq;

void uart_isr(void)__irq;

//void FIQ_Handler(void)__fiq;


static void gps(void *pvParameters);

static void accelerometer(void *pvParameters);

static void alcohol(void *pvParameters);

static void resume(void *pvParameters);


unsigned char wal[]="ALCHOL ALERT";

unsigned char as[]="SMS SENT";

unsigned char R1[]="AT\r\n";  //send “AT” command modem will response

unsigned char R2[]="AT+CMGF=1\r\n";//Select TCPIP Application mode

unsigned char R3[]="AT+CMGS=\"9902664899\"\r\n";//set Local Port

unsigned char RS_1[]="AT+CMGS=\"";

unsigned char RS_2[]="\"\r\n";

unsigned char R4[]="OVER SPEED INDICATION, REGULATING FUEL SUPPLY ";//set csd or GPRS for Connection mode “ur GPRS provide address”

unsigned char R5[]="DISTANCE BREACH, TURNING OFF ENGINE ";

unsigned char R6[]="ALCHOL DETECTED, TURNING OFF ENGINE ";

unsigned char R7[]="COLLUSION DETECTED, CUTTING OFF FUEL SUPPLY AND TURNING OFF ENGINE ";

unsigned char s1[]="AT\r\n";  //send “AT” command modem will response

unsigned char s2[]="AT+CMGF=1\r\n";//Select TCPIP Application mode

unsigned char s3[]="AT+CMGR=2\r\n";//set Local Port

unsigned char s4[]="AT+CMGD=2\r\n";//set csd or GPRS for Connection mode “ur GPRS provide address”

unsigned char b[]="dist_set";

unsigned char w[]="COLLUSION ALERT";

unsigned char M[]="DISTANCE km";
 
unsigned char wd[]="DIST_BREACH";

unsigned char wel1[]=" WELCOME TO ";

unsigned char wel2[]=" GEO FENCING ";

unsigned char s_lat[]="Lattitude=";

unsigned char s_long[]=", Longitude=";

unsigned char s_dist[]=", Distance=";

unsigned int val=0,ss=0,flag;
unsigned char mob[20]="8147016465";

float dist=200;
unsigned char longi_data[13];
unsigned char lati_data[12];
unsigned char data=0,i,t,x[10],y[10];
unsigned int flag;
unsigned int latd1=601,longd1=4892;
unsigned int lat1,long1;
unsigned int lat2,long2,kma=0;
const unsigned int R=6371;
double dlat,dlong,p,q;
double d,s;
char *ptr;
int *str,z;
float a;
unsigned int m,n,cnt;
extern void key_init(void);
unsigned char flg1=0,flg2=0,test=0;
unsigned char bx[10];

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
//static void prvSetupHardware( void );

/*-----------------------------------------------------------*/

xTaskHandle xHandle;
xTaskHandle xHandle1;
xTaskHandle xHandle2;
xTaskHandle xHandle3;


/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */

void delay1(void)
{
	unsigned int i,j;
	for(i=0;i<1275;i++)
	for(j=0;j<1275;j++);
}

void send1(unsigned char* ch)
{	
	while(*ch!='\0')
	{
		
		while(!(U0LSR & 0x20));
		U0THR=*ch;	
		ch++;
	}	
//	*ch=0x00;
}

void call_enter(void)
{
	while(!(U0LSR & 0x20));
		U0THR=0x0d;
}

void call_Z(void)
{
	while(!(U0LSR & 0x20));
		U0THR=0x1A;
}

void new_line(void)
{
	while(!(U0LSR & 0x20));
		U0THR='\n';
}

void sms(void)
{
	unsigned long ijk = 5675800;
	unsigned char c=0;
	VICIntEnClr|= 0x40;

	send1(R1);
	call_enter();
	delay();
	
	
	send1(R2);
	call_enter();
	delay();

	send1(RS_1);
	send1(mob);
	send1(RS_2);
	call_enter();
	delay();

	switch(flag)
	{
		case 1:	 	send1(R5);
					call_Z();
					break;
	
		case 2:	 	send1(R4);
					call_Z();
				   	break;

		case 3:	 	send1(R6);
					call_Z();
				   	break;

		case 4:	 	send1(R7);
					send1(s_lat);
					send1(lati_data);
					send1(s_long);
					send1(longi_data);
					send1(s_dist);
					send1(bx);
					call_Z();
				   	break;

		case 5:	 	send1(s_lat);
					send1(lati_data);
					send1(s_long);
					send1(longi_data);
					send1(s_dist);
					send1(bx);
					call_Z();
				   	break;
	 default: break;
	}
	  
			while(1)
			{
				while(!(U0LSR & 0x01))
				{
				ijk--;
				if(ijk == 0)
				{	
					goto exit3;
				}
				}
					ijk = 5675800;
					c=U0RBR;
			}

	   
exit3:	
test = 1;			
}

void alert()
{
	int i=0;
	sms();

	lcd_cmd(0x01);
	lcd_cmd(0x80);

//IOSET0|=0x80000;
//delay1();
//IOCLR0|=0x80000;
//delay1();

	for(i=0;as[i]!='\0';i++)
	{	
		lcd_data(as[i]);
		delay();
	}
	delay1();
	delay1();
	delay1();
		
}

void dist_calc()
{
	float k,m,f,d,dx;
	if(latd1>lat2)
		k  = (latd1-lat2);
	else
		k  = (lat2-latd1);
	k = k/60;

	if(longd1>long2)
		m  = (longd1-long2);
	else
		m  = (long2-longd1);
	m = m/60;
		
	dlat  = k*PIE;
	dlong = m*PIE;
	f = (latd1/60)*PIE;
	dx = (lat2/60)*PIE;
	p = sin(dlat/2)*sin(dlat/2)+sin(dlong/2)*sin(dlong/2)*cos(f)*cos(dx);
	q = (2)*atan2(sqrt(p),sqrt(1-p));
	d = R*q;
	if( (d > dist) && (flg1 == 0))
	{
		flag=1;
		flg1 = 1;
		IOSET0=0x0400;
	 	delay1();
	  	delay1();
	 	IOCLR0=0x0400;
	   	delay1();
	   	lcd_cmd(0x01);
	   	lcd_cmd(0x80);
	    for(i=0;wd[i]!='\0';i++)
	   	{
	   		lcd_data(wd[i]);
	   		delay();
	   	}
	   	alert();
	}
	else if (d < dist)
	{
		flg1 = 0;	
	}
	lcd_cmd(0x01);
	lcd_cmd(0x80);
	for(i=0;M[i]!='\0';i++)
	{
	   lcd_data(M[i]);
	   delay();
	}
	
	lcd_cmd(0xc0);
	ftos(d);

delay1();
delay1();
} 

void speed_calc(void)
{
	float k,m,f,d;
	int i=0;
	unsigned char z[]="OVER SPEED";
	k  = (lat2-lat1)*0.01;
	m  = (long2-long1)*0.01;
	dlat  = k*PIE;
	dlong = m*PIE;
	f  = lat1*PIE*0.01;
	d  = lat2*PIE*0.01;
	p = sin(dlat/2)*sin(dlat/2)+sin(dlong/2)*sin(dlong/2)*cos(f)*cos(d);
	q = (2)*atan2(sqrt(p),sqrt(1-p));
	d = R*q;

		s = d/0.0001389;
		s=0;
	if(s>SPEED)
	{
		flag=2;
		IOSET0=0x0800;
		delay1();
		delay1();
		IOCLR0=0x0800;
		delay1();
		lcd_cmd(0x01);
		lcd_cmd(0x80);
		for(i=0;z[i]!='\0';i++)
		{
		   lcd_data(z[i]);
		   delay();
		}
		   alert();		
	}
}

void buzzer (void)
{
	/* timer1 init */

	VICIntEnable |= 0x20;
	VICVectCntl1 =0x25;
	VICVectAddr1 =(unsigned long)timer_isr1;

	T1PR 		 = 0x20;					         //for 1 minute Load prescaler
	T1TCR 		 = 0x02;							//Reset counter and prescaler
	T1MCR		 = 0x03;
	T1MR0		 = 0x300000;
	T1TCR		 = 0x01;

	while(1)
	 {
		if((IOPIN0 & 0x8000) == 0)
		{
			IOSET0=0x02000;
		 	delay();
		 	IOCLR0=0x02000;				 //p0.13 buzzer
			delay();		
		}
		else
		{
			IOSET0=0x02000;
		 	delay1();
		 	IOCLR0=0x02000;				 
			delay1();
			T1IR = 0x01;
			T1TCR= 0x02;
			T1TC = 0X00;
			T1PC = 0X00;
			VICIntEnClr |= 0x0000020;
			goto exit1;
		}
	 }
exit1:
	lcd_cmd(0x01);	
	delay1();
}
void check(unsigned int var)
{
	if(ss==0)
	{
		if(var >= 0x230)
		{
	//	if(var!=0)
			flag=4;	
			buzzer();
		}
		
		ss++;
	}
	else if(ss==1)
	{
		if(var >= 0x2F0 || ((var >= 0x250) && (var <= 270)))
		{
			flag=4;
			buzzer();
		}
	
		ss++;
	}	
	else if(ss==2)
	{ 
		if(var <= 0x340 && var >= 0x250)
		{
			flag=4;
			buzzer();
		
		}	
		ss++;
	}				
}


void A2D (void) 
{                     
	ADCR |= 0x01000000;                      /* Start A/D Conversion */
  	do {
    		val = ADDR;                            /* Read A/D Data Register */
  		} while ((val & 0x80000000) == 0);       /* Wait for end of A/D Conversion */
  		ADCR &= ~0x01000000;                     /* Stop A/D Conversion */
  		val = (val >> 6) & 0x03FF;               /* Extract AIN0 Value */

  		check(val);
                     /* Write 3. Hex Digit */
        /* Delay */
}


int main( void )
{
   	unsigned int xx = 0,ijk = 5675800;
	unsigned char lk=0;
	/* Setup the hardware for use with the Keil demo board. */
	PINSEL0= 0x02050085;   // uart 0,1, timer 0 match 0, timer1 match 0
	VPBDIV = 0x00;
	U1LCR  = 0x83;
	U1DLL  = 0x61;
	U1LCR  = 0X03;
	U0LCR  = 0x83;
	U0DLL  = 0x61;
	U0LCR  = 0X03;
	lcd_init();
	IODIR0|=0X00003C00;
	IOCLR0|=0X00003C00;
	IODIR1|=0X03000000;
	IODIR0|=0x01c00;  //P0.10 TURN OFF ENGINE P0.11 FUEL CONTROL OVERSPEED
	IOSET0|=0x01000;
	/*  Interrupt */

	VICIntEnable = 0x50;			 //uart 0,  timer 0

	/* timer0 init */

	VICVectCntl0 =0x24;
	VICVectAddr0 =(unsigned long)timer_isr;

	T0TCR 		|= 0x02;				    		 //Reset counter and prescaler
	T0PR 		 = 0x30;					         //for 1 minute Load prescaler	
	T0MCR		|= 0x03;
	T0MR0		 = 15000000;	
	
	IODIR0|=0x80000;

	/*  rsms init */

	VICDefVectAddr=(unsigned long) uart_isr;

	U0IER	= 0x01;
	T0TCR	= 0x01;

	lcd_cmd(0x01);
	lcd_cmd(0x80);
	for(xx=0;wel1[xx]!='\0';xx++)
	{	
		lcd_data(wel1[xx]);
	   	delay();	
	}
	lcd_cmd(0xC0);
	for(xx=0;wel2[xx]!='\0';xx++)
	{	
		lcd_data(wel2[xx]);
	   	delay();	
	}
	delay1();
	send1(s4);
	call_enter();

	while(1)
	{
		while(!(U0LSR & 0x01))
		{
		ijk--;
		if(ijk == 0)
		{	
			goto exit4;
		}
		}
			ijk = 5675800;
			lk=U0RBR;
	}
exit4:
	//	prvSetupHardware();

	/* Start the demo/test application tasks. */
//	vAltStartComTestTasks( mainCOM_TEST_PRIORITY, mainCOM_TEST_BAUD_RATE, mainCOM_TEST_LED );
//	vStartLEDFlashTasks( mainLED_TASK_PRIORITY );
//	vStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
//	vStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
//	vStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
//	vStartDynamicPriorityTasks();

	/* Start the check task - which is defined in this file.  This is the task
	that periodically checks to see that all the other tasks are executing 
	without error. */
	xTaskCreate( vErrorChecks, "Check", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
	 
	
	
	xTaskCreate(accelerometer, "accelerometer", 90, NULL, 4, &xHandle);
	xTaskCreate(alcohol, "alcohol", 90, NULL, 3, &xHandle1);
	xTaskCreate(gps, "gps", 90, NULL, 2, &xHandle2);
	xTaskCreate(resume, "resume", 90, NULL, 1, &xHandle3);
	
	
	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

static void vErrorChecks( void *pvParameters )
{
portTickType xDelayPeriod = mainNO_ERROR_FLASH_PERIOD;

	/* Parameters are not used. */
	( void ) pvParameters;

	
	/* Cycle for ever, delaying then checking all the other tasks are still
	operating without error.  If an error is detected then the delay period
	is decreased from mainNO_ERROR_FLASH_PERIOD to mainERROR_FLASH_PERIOD so
	the on board LED flash rate will increase.

	This task runs at the highest priority. */

	for( ;; )
	{
		/* The period of the delay depends on whether an error has been 
		detected or not.  If an error has been detected then the period
		is reduced to increase the LED flash rate. */
		vTaskDelay( xDelayPeriod );

		if( prvCheckOtherTasksAreStillRunning() != pdPASS )
		{
			/* An error has been detected in one of the tasks - flash faster. */
			xDelayPeriod = mainERROR_FLASH_PERIOD;
		}

		/* Toggle the LED before going back to wait for the next cycle. */
//		vParTestToggleLED( mainCHECK_LED );
	}
}
/*-----------------------------------------------------------*/

//static void prvSetupHardware( void )
//{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure the UART1 pins.  All other pins remain at their default of 0. */
//	PINSEL0 |= mainTX_ENABLE;
//	PINSEL0 |= mainRX_ENABLE;

	/* LED pins need to be output. */
//	IODIR1 = mainLED_TO_OUTPUT;

	/* Setup the peripheral bus to be the same as the PLL output. */
//	VPBDIV = mainBUS_CLK_FULL; 
////	PINSEL1=0x05000000;		   
// 	PINSEL0|=0x00000000;
//}
/*-----------------------------------------------------------*/

static portLONG prvCheckOtherTasksAreStillRunning( void )
{
portLONG lReturn = pdPASS;

	/* Check all the demo tasks (other than the flash tasks) to ensure
	that they are all still running, and that none of them have detected
	an error. */
//	if( xAreComTestTasksStillRunning() != pdPASS )
//	{
//		lReturn = pdFAIL;
//	}
//
//	if( xArePollingQueuesStillRunning() != pdTRUE )
//	{
//		lReturn = pdFAIL;
//	}
//
//	if( xAreBlockingQueuesStillRunning() != pdTRUE )
//	{
//		lReturn = pdFAIL;
//	}
//
//	if( xAreSemaphoreTasksStillRunning() != pdTRUE )
//	{
//		lReturn = pdFAIL;
//	}
//
//	if( xAreDynamicPriorityTasksStillRunning() != pdTRUE )
//	{
//		lReturn = pdFAIL;
//	}

	return lReturn;
}
/*-----------------------------------------------------------*/

static void accelerometer(void *pvParameters)
{	
	while(1)
	 {
	 	ADCR   = 0x002E0402;                     /* Setup A/D: 10-bit AIN0 @ 3MHz */
      	A2D();                               
	          /* Output A/D Conversion Result */
		ADCR   = 0x002E0404;                     /* Setup A/D: 10-bit AIN0 @ 3MHz */
      	A2D();  
	          /* Output A/D Conversion Result */
		ADCR   = 0x002E0408;                     /* Setup A/D: 10-bit AIN0 @ 3MHz */
      	A2D(); 
		delay();
		ss=0; 

		vTaskSuspend( xHandle );
	 } 	
	
}

static void alcohol(void *pvParameters)
{
	while(1)
	 {
	 	unsigned int i=0;  		
		unsigned int var=0; 
		ADCR = 0x002E0401;                     /* Setup A/D: 10-bit AIN0 @ 3MHz */                

		ADCR |= 0x01000000;                      /* Start A/D Conversion */
 		do
		 {
			var = ADDR;                            /* Read A/D Data Register */
		 } while ((var & 0x80000000) == 0);       /* Wait for end of A/D Conversion */
		ADCR &= ~0x01000000;                     /* Stop A/D Conversion */
		var = (var >> 6) & 0x3FF;               /* Extract AIN0 Value */

		if((var < 0x360) && (flg2 == 0))
		{
			 flag=3;
			 flg2 = 1;
			 lcd_cmd(0x01);
		     lcd_cmd(0xc0);
			 delay1();
			 delay1();
		     for(i=0;wal[i]!='\0';i++)
			 {
			   lcd_data(wal[i]);
			   delay();
			 }
	
			 IOSET0|=0x00400;
			 delay1();
			 IOCLR0|=0x00400;
			 delay1();
	  		 alert(); 
		}
		else if ( val > 0x360 )
		{
			flg2 = 0;
		}
		vTaskSuspend( xHandle1 );                
    } 	
}

static void gps( void *pvParameters )
{
	 while(1)
	 {
	 	while(!(U1LSR & 0x01));
			data=U1RBR;
		VICIntEnable|= 0x40;                     // Check the string '$GPGGA,'
			if(data=='$')
			{
				while(!(U1LSR & 0x01));
				data=U1RBR;
				if(data=='G')
				{
					while(!(U1LSR & 0x01));
					data=U1RBR;
					if(data=='P')
					{
						while(!(U1LSR & 0x01));
						data=U1RBR;
						if(data=='G')
						{
							while(!(U1LSR & 0x01));
							data=U1RBR;
							if(data=='G')
							{
								while(!(U1LSR & 0x01));
								data=U1RBR;
								if(data=='A')
								{
									while(!(U1LSR & 0x01));
									data=U1RBR;
									if(data==',')
									{
										while(!(U1LSR & 0x01));
										data=U1RBR;
										while(data!=',')
										{
											while(!(U1LSR & 0x01));
											data=U1RBR;
										}
										for(i=0;data!='N';i++)
										{
											while(!(U1LSR & 0x01));
											data=U1RBR;
//											if((data!='N')&&(data!=','))
											if(data == '.')
												break;
											lati_data[i]=data;
											  // Store the Latitude data
										}
											longi_data[++i] ='\0';
//										i++;
//										lati_data[i]='\0';
									}
									kma = 0;
									while(1)
									{
									while(!(U1LSR & 0x01));
									data=U1RBR;
									if(data==',')
										kma++;
									if(kma == 2)
										break;
									}
										for(i=0;data!='E';i++)
										{
											while(!(U1LSR & 0x01));
											data=U1RBR;
											if(data == '.')
												break;											
											longi_data[i]=data; 
											 // Store the Longitude data
										}
//										i++;
//										longi_data[i]='\0';
										lati_data[++i] = '\0';
									
									lat2 = stof_lati(lati_data);
									long2= stof_longi(longi_data);

										lcd_cmd(0x01);
										lcd_cmd(0x80);																
										dist_calc();

										vTaskSuspend( xHandle2 );		 															
									}

								}
					  }
				}
			}

		}	 
	}		
}

static void resume(void *pvParameters)
{
	while(1)
	{
		VICIntEnable|= 0x50;
delay1();
delay1();
		vTaskResume(xHandle);			  
		vTaskResume(xHandle1);			 
		vTaskResume(xHandle2);
delay1();
delay1();
				 
	}
}
void uart_isr()__irq
{
	unsigned char a[4],c;
	unsigned char b[]="DIST_SET =";
	int i=0,m=0;
	unsigned long ijk = 5675800;
		m=0;
		ijk = 5675800;
		while(1)
		{
			while(!(U0LSR & 0x01))
			{
			ijk--;
			if(ijk == 0)
			{	
				goto exit;
			}
			}
				ijk = 5675800;
				c=U0RBR;
				if(c=='+')
					break;
		}
		while(1)
		{
			while(!(U0LSR & 0x01))
			{
			ijk--;
			if(ijk == 0)
			{	
				goto exit;
			}
			}
				ijk = 5675800;
				c=U0RBR;
				if(c=='2')
					break;
		}				
		send1(s1);
		call_enter();

		send1(s2);
		call_enter();
	
		send1(s3);
		call_enter();
		IOSET0|=0x80000;
		ijk = 5675800;
		while(1)
			{
				while(!(U0LSR & 0x01))
				{
				ijk--;
				if(ijk == 0)
				{	
					goto exit;
				}
				}
					ijk = 5675800;
					c=U0RBR;
					if(c=='$')
						break;				
			}
			while(!(U0LSR & 0x01));
			c=U0RBR;			
		if(c == '.')
		{
			flag = 5;
			sms();
			lcd_cmd(0x01);
			lcd_cmd(0x80);
			delay();
			for(i=0;as[i]!='\0';i++)
			{	
				lcd_data(as[i]);
				delay();
			}			
			delay();	
			goto exit;
		}
		else if(c == '%')
		{
			m=0;
			while(!(U0LSR & 0x01));
			c=U0RBR;
			while(c != '%')
			{			
				mob[m]=c;
				m++;
				while(!(U0LSR & 0x01));
				c=U0RBR;
			}
			goto exit;
		}
		m=0;
		while(m<3)
		{
			a[m]=c;
			m++;
			while(!(U0LSR & 0x01));
			c=U0RBR;
		}
		a[3]='\0';
		lcd_cmd(0x01);
		delay();
		lcd_cmd(0x80);
		delay();
		 for(i=0;b[i]!='\0';i++)
		{
			lcd_data(b[i]);
			delay();
		}
		delay1();
		for(i=0;i<3;i++)
		{
			lcd_data(a[i]);
			delay1();
			delay1();
		}
		delay1();
		delay1();

			dist = 0.000000;
		   	dist=atof(a);
			 	
			lcd_cmd(0x01);
			lcd_cmd(0x80);
		
			ftos(dist);
exit:
			send1(s4);
			call_enter();
	
	  ijk = 5675800;
		while(1)
			{
			while(!(U0LSR & 0x01))
			{
			ijk--;
			if(ijk == 0)
			{	
				goto exit5;
			}
			}
				ijk = 5675800;
				c=U0RBR;
			}				
exit5: 
	m=0;
	while(m<35)
	{
		a[m] = '\0';				// CLEAR THE MESSAGE
		m++;
	}
	U0FCR=0X02;	
	U0IIR|=0x01;
	VICVectAddr=0x00;
	VICIntEnClr|=0x40;
}

void timer_isr()__irq
{	  
	  lcd_cmd(0x01);
	  lcd_cmd(0xc0);	   
	  speed_calc();
	  lat1=lat2;
	  long1=long2;
	  T0IR=0x01;
	  VICVectAddr=0x00;
	  VICIntEnClr |= 0x0000010;
}

void timer_isr1()__irq
{ 
	int i;
	IOSET0|=0x00c00;
	delay1();
	IOCLR0|=0x00c00;
	delay1();
	
	alert();

	while(1)
	{
		lcd_cmd(0x01);
		lcd_cmd(0xc0);
		for(i=0;w[i]!='\0';i++)
		{
		   lcd_data(w[i]);
		   delay();
		}
		delay1();
		delay1();
		delay1();
	}
/*		code Unreachable		*/
//	T1IR=0x01;
//	VICVectAddr=0x00;
//	VICIntEnClr |= 0x0000020;
}
