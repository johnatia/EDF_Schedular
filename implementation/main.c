/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


#define BUTTON1									PORT_1,PIN1
#define BUTTON2									PORT_1,PIN2
#define NEWLINE									'\n'
#define TaskStartCalcExcutionTime(PORT_x, PINx)		GPIO_write(PORT_x, PINx, PIN_IS_HIGH)

#define TaskEndCalcExcutionTime(PORT_x, PINx)			GPIO_write(PORT_x, PINx, PIN_IS_LOW	)
/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

#define ARRAY_SIZE(ARR)						(sizeof(ARR)/sizeof(ARR[0]))

/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/
xTaskHandle BlinkLED1_Handler 						= NULL, BlinkLED2_Handler 				= NULL;
xTaskHandle Button_1_Monitor_Handler			= NULL, Button_2_Monitor_Handler 	= NULL,
						Periodic_Transmitter_Handler 	= NULL, Uart_Receiver_Handler 		= NULL,
						Load_1_Simulation_Handler			= NULL, Load_2_Simulation_Handler	= NULL;

QueueHandle_t Queue_Str = NULL, Queue_Button_1 = NULL, Queue_Button_2 = NULL;
const char *str = "Hello EDF Schedular\n";
char RunTimeBuffer[190];
TickType_t Task_WCET[6] = {0};
float CPU_Load = 0.0;
uint16_t HyperPeriod = 100;

/**
	*	Task 1: ""Button_1_Monitor"", {Periodicity: 50, Deadline: 50}
	* This task will monitor rising and falling edge on button 1 and send this event to the consumer task.
	* (Note: The rising and failling edges are treated as separate events, hence they have separate strings)
	* WCET = 5.2us
***/
void Button_1_Monitor(void *pvParameters)
{
	static pinState_t Button_CurrentState, Button_PrevState;
	TickType_t StartTime = 0;
	char EdgeEvent;	
	TickType_t xLastWakeTime = xTaskGetTickCount();	
	Button_CurrentState = GPIO_read(BUTTON1);
	Button_PrevState = Button_CurrentState;
	while(1)
	{
		StartTime = xTaskGetTickCount();
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskStartCalcExcutionTime(PORT_0, PIN1);		//Start Calculation of the Task Excution Time
		StartTime = xTaskGetTickCount();
		Button_CurrentState = GPIO_read(BUTTON1);
		if(Button_CurrentState != Button_PrevState)	//Event is detected
		{
			if( (Button_CurrentState == PIN_IS_HIGH) && (Button_PrevState == PIN_IS_LOW) )
			{
				//Rising Edge is detected
				EdgeEvent = 'R';
				xQueueSend(Queue_Button_1 , (char *)&EdgeEvent , ( TickType_t ) 0 );
			}
			else if((Button_CurrentState == PIN_IS_LOW) && (Button_PrevState == PIN_IS_HIGH))
			{
				//Falling Edge is detected
				EdgeEvent = 'F';
				xQueueSend(Queue_Button_1 , (char *)&EdgeEvent , ( TickType_t ) 0 );
			}
		}
		Button_PrevState = Button_CurrentState;		//Save the last state of the button
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskEndCalcExcutionTime(PORT_0, PIN1);	//End Calculation of the Task Excution Time
		Task_WCET[0] = xTaskGetTickCount() - StartTime;
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 50 ) );
	}
}

/**
	*	Task 2: ""Button_2_Monitor"", {Periodicity: 50, Deadline: 50}
	* This task will monitor rising and falling edge on button 2 and send this event to the consumer task.
	* (Note: The rising and failling edges are treated as separate events, hence they have separate strings)
	* WCET = 5.2us
***/
void Button_2_Monitor(void *pvParameters)
{
	static pinState_t Button_CurrentState, Button_PrevState;
	char EdgeEvent;
	TickType_t StartTime = 0;

	TickType_t xLastWakeTime = xTaskGetTickCount();	
	Button_CurrentState = GPIO_read(BUTTON2);
	Button_PrevState = Button_CurrentState;

	
	while(1)
	{
		StartTime = xTaskGetTickCount();
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskStartCalcExcutionTime(PORT_0, PIN2);		//Start Calculation of the Task Excution Time
		Button_CurrentState = GPIO_read(BUTTON2);
		if(Button_CurrentState != Button_PrevState)	//Event is detected
		{
			if( (Button_CurrentState == PIN_IS_HIGH) && (Button_PrevState == PIN_IS_LOW) )
			{
				//Rising Edge is detected
				EdgeEvent = 'R';
				xQueueSend(Queue_Button_2 , (char *)&EdgeEvent , ( TickType_t ) 0 );
			}
			else if((Button_CurrentState == PIN_IS_LOW) && (Button_PrevState == PIN_IS_HIGH))
			{
				//Falling Edge is detected
				EdgeEvent = 'F';
				xQueueSend(Queue_Button_2 , (char *)&EdgeEvent , ( TickType_t ) 0 );
			}
		}
		Button_PrevState = Button_CurrentState;		//Save the last state of the button
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskEndCalcExcutionTime(PORT_0, PIN2);	//End Calculation of the Task Excution Time
		Task_WCET[1] = xTaskGetTickCount() - StartTime;
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 50 ) );
	}
}

/**
	*	Task 3: ""Periodic_Transmitter"", {Periodicity: 100, Deadline: 100}
	* This task will send preiodic string every 100ms to the consumer task.
	* WCET = 2.616667us
***/
void Periodic_Transmitter(void *pvParameters)
{
	TickType_t StartTime = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();	

	while(1)
	{
		StartTime = xTaskGetTickCount();
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskStartCalcExcutionTime(PORT_0, PIN3);		//Start Calculation of the Task Excution Time
		xQueueSend(Queue_Str , (void *)&str , ( TickType_t ) 0 );	
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskEndCalcExcutionTime(PORT_0, PIN3);	//End Calculation of the Task Excution Time
		Task_WCET[2] = xTaskGetTickCount() - StartTime;
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 100 ) );
	}
}

/**
*	Task 4: ""Uart_Receiver"", {Periodicity: 20, Deadline: 20}
	*	This is the consumer task which will write on UART any received string from other tasks.
	* WCET = 72.5us
***/
void Uart_Receiver(void *pvParameters)
{
	char  pQueue_Button_1 = 'N', pQueue_Button_2 =  'N';
	char *pStr = NULL;
	TickType_t StartTime = 0;
	volatile char buffer[14];
	TickType_t xLastWakeTime = xTaskGetTickCount();	
	
	while(1)
	{
		StartTime = xTaskGetTickCount();
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskStartCalcExcutionTime(PORT_0, PIN4);		//Start Calculation of the Task Excution Time
		//vSerialPutString(newstr , ARRAY_SIZE(newstr) );
			if( xQueueReceive( Queue_Button_1,(void *)&pQueue_Button_1, (TickType_t)0 ) == pdPASS )
      {
          //pQueue_Button_1 now points to the Ready Queue. 
					snprintf((char *)buffer , ARRAY_SIZE(buffer) , "Button1 = %c", pQueue_Button_1);
					buffer[13] = NEWLINE;
					vSerialPutString((const signed char *)buffer , ARRAY_SIZE(buffer));
      }
			if( xQueueReceive( Queue_Button_2, (void *)&pQueue_Button_2, (TickType_t)0 ) == pdPASS )
      {
          //pQueue_Button_2 now points to the Ready Queue. 
					snprintf((char *)buffer , ARRAY_SIZE(buffer) , "Button2 = %c", pQueue_Button_2);
					buffer[13] = NEWLINE;
					vSerialPutString((const signed char *)buffer , ARRAY_SIZE(buffer));
			}
			if( xQueueReceive( Queue_Str,(void *)&pStr ,(TickType_t)0 ) == pdPASS )
      {
          //pStr now points to the Ready Queue. 
					vSerialPutString((const signed char *)pStr , 20);
      }
			vTaskGetRunTimeStats(RunTimeBuffer);
			RunTimeBuffer[189] =  NEWLINE;
			vSerialPutString((const signed char *)RunTimeBuffer , ARRAY_SIZE(RunTimeBuffer) );

		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskEndCalcExcutionTime(PORT_0, PIN4);	//End Calculation of the Task Excution Time
		Task_WCET[2] = xTaskGetTickCount() - StartTime;
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 20 ) );
	}
}

/**
	*	Task 5: ""Load_1_Simulation"", {Periodicity: 10, Deadline: 10}, Execution time: 5ms
***/
void Load_1_Simulation(void * pvParameters)
{
	unsigned long counter = 0;
	TickType_t StartTime = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();	
	while(1)
	{
		StartTime = xTaskGetTickCount();
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskStartCalcExcutionTime(PORT_0, PIN5);		//Start Calculation of the Task Excution Time
		//counter = 45000 -> EX time = 12.06372ms 
		//counter = 44900 -> EX time = 12.0282ms
		//for(counter = 0; counter < 22000; counter++);	
		for(counter = 0; counter < 7378 * 5; counter++);	
		
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskEndCalcExcutionTime(PORT_0, PIN5);	//End Calculation of the Task Excution Time
		Task_WCET[4] = xTaskGetTickCount() - StartTime;
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 ) );
	}
}
/**
	*	Task 6: ""Load_2_Simulation"", {Periodicity: 100, Deadline: 100}, Execution time: 15ms
***/
void Load_2_Simulation(void * pvParameters)
{
	unsigned long counter = 0; 
	TickType_t StartTime = 0;
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		StartTime = xTaskGetTickCount();
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskStartCalcExcutionTime(PORT_0, PIN6);		//Start Calculation of the Task Excution Time
		//counter = 45000 -> EX time = 12.06372ms 
		//counter = 44900 -> EX time = 12.0282ms
		//for(counter = 0; counter < 60000; counter++);		
		for(counter = 0; counter < 7378 * 15; counter++);	
		//Calculate the Excution time of the Task using GPIO Pin and Logic Analyzer
		TaskEndCalcExcutionTime(PORT_0, PIN6);	//End Calculation of the Task Excution Time
		Task_WCET[5] = xTaskGetTickCount() - StartTime;
		//CPU_Load = (( (Task_WCET[0]*HyperPeriod/50.0)+(Task_WCET[1]*HyperPeriod/50.0)+(Task_WCET[2])+(Task_WCET[3]*HyperPeriod/20.0)+(Task_WCET[4]*HyperPeriod/10.0)+(Task_WCET[5]) ) /HyperPeriod);
		CPU_Load = (( (Task_WCET[0]/50.0)+(Task_WCET[1]/50.0)+(Task_WCET[2])+(Task_WCET[3]/20.0)+(Task_WCET[4]/10.0)+(Task_WCET[5]/100.0) ) );

		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 100 ) );
	}
}
/*
void BlinkLED1(void *pvParameters)
{
	while(1)
	{
		GPIO_write( PORT_0, PIN1 , !GPIO_read(PORT_0, PIN1) );
		vTaskDelay(1000);
	}
}

void BlinkLED2(void *pvParameters)
{
	while(1)
	{
		GPIO_write( PORT_0, PIN2 , !GPIO_read(PORT_0, PIN2) );
		vTaskDelay(500);
	}
}
*/
/**
	*	CallBack to simulate the Tick ISR on Logic Analyzer
***/
void vApplicationTickHook(void)
{
	GPIO_write( PORT_0, PIN0 ,PIN_IS_HIGH );
	GPIO_write( PORT_0, PIN0 ,PIN_IS_LOW );
}
/**
	*	CallBack to simulate the Idle Task on Logic Analyzer
***/
void vApplicationIdleHook()
{
	GPIO_write( PORT_0, PIN7 ,PIN_IS_HIGH );
	GPIO_write( PORT_0, PIN7 ,PIN_IS_LOW );
}
/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	
    /* Create Tasks here */
	/*
	xTaskCreate( BlinkLED1,
							 "Blink Led1",
							 configMINIMAL_STACK_SIZE,
							 ( void * ) NULL,
								2,  
							 &BlinkLED1_Handler );
	xTaskCreate( BlinkLED2,
							 "Blink Led2",
							 configMINIMAL_STACK_SIZE,
							 ( void * ) NULL,
								1,  
							 &BlinkLED2_Handler );
							 */
	/*********************************************************************/
	Queue_Button_1 = xQueueCreate( 10 , sizeof(char) );
	Queue_Button_2 = xQueueCreate( 10 , sizeof(char) );
	Queue_Str = xQueueCreate( 1 , sizeof(char *) );

	xTaskCreate( Button_1_Monitor,
							 "Button 1 Monitor",
							 configMINIMAL_STACK_SIZE,
							 ( void * ) NULL,
								2,  
							 &Button_1_Monitor_Handler );
							 
	xTaskCreate( Button_2_Monitor,
							 "Button 2 Monitor",
							 configMINIMAL_STACK_SIZE,
							 ( void * ) NULL,
								2,  
							 &Button_1_Monitor_Handler );	

	xTaskCreate( Periodic_Transmitter,
							 "Periodic Transmitter",
							 configMINIMAL_STACK_SIZE,
							 ( void * ) NULL,
								2,  
							 &Periodic_Transmitter_Handler );
				 
	xTaskCreate( Uart_Receiver,
							 "Uart Receiver",
							 configMINIMAL_STACK_SIZE,
							 ( void * ) NULL,
								3,  
							 &Uart_Receiver_Handler );
							 
	xTaskCreate( Load_1_Simulation,
							 "Load 1 Simulation",
							 configMINIMAL_STACK_SIZE,
							 ( void * ) NULL,
								4,  
							 &Load_1_Simulation_Handler );	
							 
	xTaskCreate( Load_2_Simulation,
							 "Load 2 Simulation",
							 configMINIMAL_STACK_SIZE,
							 ( void * ) NULL,
								1,  
							 &Load_2_Simulation_Handler );				
					 
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

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


