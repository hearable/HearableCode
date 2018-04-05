#include <stdint.h>
#include <stdbool.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_devices.h"

#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"
#include "portable.h"
#include "IntegrationTest.h"
#include "semphr.h"
#include "Utils.h"
#include "CyclicShift.h"
#include "Lowpass.h"

#ifndef PI
	#define PI 3.14159265359
#endif

void
am_ctimer_isr(void)
{
    uint32_t ui32Status;

    //
    // Check the timer interrupt status.
    //
    ui32Status = am_hal_ctimer_int_status_get(false);
    am_hal_ctimer_int_clear(ui32Status);

    //
    // Run handlers for the various possible timer events.
    //
    am_hal_ctimer_int_service(ui32Status);
}

//*****************************************************************************
//
// Sleep function called from FreeRTOS IDLE task.
// Do necessary application specific Power down operations here
// Return 0 if this function also incorporates the WFI, else return value same
// as idleTime
//
//*****************************************************************************
uint32_t am_freertos_sleep(uint32_t idleTime)
{
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    return 0;
}

//*****************************************************************************
//
// Recovery function called from FreeRTOS IDLE task, after waking up from Sleep
// Do necessary 'wakeup' operations here, e.g. to power up/enable peripherals etc.
//
//*****************************************************************************
void am_freertos_wakeup(uint32_t idleTime)
{
    return;
}


//*****************************************************************************
//
// FreeRTOS debugging functions.
//
//*****************************************************************************
void
vApplicationMallocFailedHook(void)
{
    //
    // Called if a call to pvPortMalloc() fails because there is insufficient
    // free memory available in the FreeRTOS heap.  pvPortMalloc() is called
    // internally by FreeRTOS API functions that create tasks, queues, software
    // timers, and semaphores.  The size of the FreeRTOS heap is set by the
    // configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
    //
    while (1);
}

void
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    //
    // Run time stack overflow checking is performed if
    // configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    // function is called if a stack overflow is detected.
    //
    while (1)
    {
        __asm("BKPT #0\n") ; // Break into the debugger
    }
}

//*****************************************************************************
//
// Initializes all tasks
//
//*****************************************************************************

typedef struct T1Arguments{
	int windowSize;
	float* ADCPointer;
	float* AlgorithmPointer;
} T1Arguments;
		
typedef struct T2Arguments{
	int windowSize;
	float* AlgorithmPointer;
	float* PWMPointer;
} T2Arguments;
		
typedef struct ADCArguments{
	int windowSize;
	float* dataArray;
} ADCArguments;
		
typedef struct AlgorithmArguments{
	int windowSize;
	float* currentWindowRe;
	float* currentWindowIm;
	float* currentOutput;
	float* previousWindowRe;
	float* previousWindowIm;
	float* hanningWindow;
	float* lowpassFilter;
	float* currentSample;
	float* currentResult;
	float* sinTable;
	float* cosTable;
	float* shiftVector;
	float sampleFrequency;
	float shiftFrequency;
	float cutOffFrequency;
	int lowpassRolloff;
} AlgorithmArguments;
		
typedef struct PWMArguments{
	int windowSize;
	float* dataArray;
} PWMArguments;

AlgorithmArguments Alg;
ADCArguments ADC;
T1Arguments T1;
T2Arguments T2;
PWMArguments PWM;
		
static TaskHandle_t xADCTask = NULL;
static TaskHandle_t xT1Task = NULL;
static TaskHandle_t xAlgorithmTask = NULL;
static TaskHandle_t xT2Task = NULL;
static TaskHandle_t xPWMTask = NULL;
		
static SemaphoreHandle_t SemaphoreT1ADC;
static SemaphoreHandle_t SemaphoreADCT1;
static SemaphoreHandle_t SemaphoreAlgT1;
static SemaphoreHandle_t SemaphoreT1Alg;
static SemaphoreHandle_t SemaphoreT2Alg;
static SemaphoreHandle_t SemaphoreAlgT2;
static SemaphoreHandle_t SemaphoreT2PWM;
static SemaphoreHandle_t SemaphorePWMT2;
static SemaphoreHandle_t SemaphoreT2T1;
static SemaphoreHandle_t SemaphoreT1T2;
		
void ADCTask(void* args){
	//am_util_debug_printf("ADC started!\n");
	ADCArguments* xargs = (ADCArguments*)args;
	
	int i = 0;
	//am_util_debug_printf("ADC init done!\n");
	xSemaphoreTake(SemaphoreT1ADC, portMAX_DELAY);
	//am_util_debug_printf("Semaphore taken!\n");
	for(;;){
		xargs->dataArray[i] = 512.0f;
		/*
		ACQUIRE DATA HERE
		*/
		i++;
		if(i==xargs->windowSize || i==(2*xargs->windowSize) || i==(3*xargs->windowSize)){
			i%=(3*xargs->windowSize);
			//am_util_debug_printf("ADC done!\n");
			xSemaphoreGive(SemaphoreADCT1);
			vTaskResume(xT1Task);
			vTaskDelay(32/portTICK_PERIOD_MS);
			xSemaphoreTake(SemaphoreT1ADC, portMAX_DELAY);
		}
	}
			
}

void AlgorithmTask(void* args){
	//am_util_debug_printf("Alg started!\n");
	AlgorithmArguments* xargs = (AlgorithmArguments*) args;
	int i = 0;
	
	for(;;){
		xSemaphoreTake(SemaphoreT1Alg, portMAX_DELAY);
		xSemaphoreTake(SemaphoreT2Alg, portMAX_DELAY);
		am_hal_stimer_counter_clear();
		ProcessData_tweaked(xargs->windowSize, 0.5f, xargs->sampleFrequency, xargs->shiftFrequency,
                    xargs->lowpassFilter, xargs->hanningWindow, xargs->sinTable, xargs->cosTable, xargs->previousWindowRe, xargs->previousWindowIm,
                    xargs->currentWindowRe, xargs->currentWindowIm, xargs->currentSample, xargs->shiftVector, xargs->currentOutput);
		ReturnWindowOutputHanning(xargs->windowSize, 0.5f, xargs->currentOutput, MID, xargs->currentResult);
		for(int i=0;i<xargs->windowSize/2;i++){
			xargs->currentResult[i] = 512 + xargs->currentResult[i]/2;
		}
		//am_util_debug_printf("%f \n",xargs->currentResult[5]);
		//am_util_debug_printf("Alg done!\n");
		am_util_debug_printf("%u ms - after Alg \n",(uint32_t)(am_hal_stimer_counter_get()*1000/32768))
		xSemaphoreGive(SemaphoreAlgT1);
		xSemaphoreGive(SemaphoreAlgT2);
		vTaskResume(xT2Task);
		vTaskSuspend(NULL);
	}
}

void T1Task(void* args){
	//am_util_debug_printf("Task1 started!\n");
	T1Arguments* xargs = (T1Arguments*) args;
	int i = 0;
	
	for(;;){
		xSemaphoreTake(SemaphoreADCT1, portMAX_DELAY);
		xSemaphoreTake(SemaphoreAlgT1, portMAX_DELAY);
		xSemaphoreTake(SemaphoreT2T1, portMAX_DELAY);
		for(int j=0;j<xargs->windowSize;j++){
			xargs->AlgorithmPointer[j] = xargs->ADCPointer[(i*xargs->windowSize)+j];
		}
		i++;
		i%=3;
		//am_util_debug_printf("T1 done!\n");
		xSemaphoreGive(SemaphoreT1ADC);
		xSemaphoreGive(SemaphoreT1Alg);
		xSemaphoreGive(SemaphoreT1T2);
		vTaskResume(xAlgorithmTask);
		vTaskSuspend(NULL);
	}
}

void T2Task(void* args){
	T2Arguments* xargs = (T2Arguments*) args;
	int i = 0;
	
	for(;;){
		xSemaphoreTake(SemaphorePWMT2, portMAX_DELAY);
		xSemaphoreTake(SemaphoreAlgT2, portMAX_DELAY);
		xSemaphoreTake(SemaphoreT1T2, portMAX_DELAY);
		for(int j=0;j<xargs->windowSize;j++){
			xargs->PWMPointer[(i*xargs->windowSize/2)+j] = xargs->AlgorithmPointer[j];
		}
		i++;
		i%=2;
		//am_util_debug_printf("T2 done!\n");
		xSemaphoreGive(SemaphoreT2PWM);
		xSemaphoreGive(SemaphoreT2Alg);
		xSemaphoreGive(SemaphoreT2T1);
		vTaskResume(xPWMTask);
		vTaskSuspend(NULL);
	}
}

void PWMTask(void* args){
	PWMArguments* xargs = (PWMArguments*) args;
	int i = 0;
	
	for(;;){
		xSemaphoreTake(SemaphoreT2PWM, portMAX_DELAY);
		for(int j=0;j<xargs->windowSize/2;j++){
			//am_util_debug_printf("%f \n",xargs->dataArray[j+(i*(xargs->windowSize)/2)]);
		}
		i++;
		i%=2;
		/*
		OUTPUT DATA HERE
		*/
		//am_util_debug_printf("PWM done\n");
		xSemaphoreGive(SemaphorePWMT2);
		vTaskSuspend(NULL);
	}
}

void
run_tasks(void)
{
    //
    // Set some interrupt priorities before we create tasks or start the scheduler.
    //
    // Note: Timer priority is handled by the FreeRTOS kernel, so we won't
    // touch it here.
    //
	
		am_util_debug_printf("Starting to setup tasks...\n");
	
		int windowSize = 256;
	
		am_util_debug_printf("Structs done...\n");
	
		// Initialization of ADC
		ADC.windowSize = windowSize;
		ADC.dataArray = calloc(3*windowSize, sizeof(float));
	 
		// Initialization of algorithm
		Alg.windowSize = windowSize;
		Alg.currentSample = calloc(windowSize, sizeof(float));
		Alg.currentWindowRe = calloc(windowSize, sizeof(float));
		Alg.currentWindowIm = calloc(windowSize, sizeof(float));
		Alg.previousWindowRe = calloc(windowSize, sizeof(float));
		Alg.previousWindowIm = calloc(windowSize, sizeof(float));
		Alg.hanningWindow = calloc(windowSize, sizeof(float));
		Alg.lowpassFilter = calloc(windowSize, sizeof(float));
		Alg.currentOutput = calloc(2*windowSize, sizeof(float));
		Alg.currentResult = calloc(windowSize, sizeof(float));
		Alg.sampleFrequency = 12000.0f;
		Alg.shiftFrequency = 400.0f;
		Alg.cutOffFrequency = 4000.0f;
		Alg.lowpassRolloff = 6;
		Alg.sinTable = (float*) calloc(windowSize/2, sizeof(float));
		Alg.cosTable = (float*) calloc(windowSize/2, sizeof(float));
		
		for (int i = 0; i < windowSize / 2; i++) {
			Alg.cosTable[i] = cos(2 * PI * i / windowSize);
			Alg.sinTable[i] = sin(2 * PI * i / windowSize);
		}
		
		Alg.shiftVector = (float*) calloc(windowSize, sizeof(float));
		InitializeCosineVectorAutoShift(Alg.shiftVector, Alg.windowSize, Alg.sampleFrequency, Alg.shiftFrequency);
		Alg.hanningWindow = HanningWindow(Alg.windowSize, PERIODIC);
		Alg.lowpassFilter = LowpassFilter(Alg.windowSize,Alg.sampleFrequency,Alg.cutOffFrequency,Alg.lowpassRolloff);
		// Initialization of algorithm done
	
		// Initialization of PWM
		PWM.windowSize = windowSize;
		PWM.dataArray = calloc(2*windowSize, sizeof(float));
	
		// Initialization of T1
		T1.windowSize = windowSize;
		T1.ADCPointer = ADC.dataArray;
		T1.AlgorithmPointer = Alg.currentSample;
		
		// Initialization of T2
		T2.windowSize = windowSize;
		T2.AlgorithmPointer = Alg.currentResult;
		T2.PWMPointer = PWM.dataArray;
		
		SemaphoreT1ADC = xSemaphoreCreateBinary();
		SemaphoreADCT1 = xSemaphoreCreateBinary();
		SemaphoreAlgT1 = xSemaphoreCreateBinary();
		SemaphoreT1Alg = xSemaphoreCreateBinary();
		SemaphoreT2Alg = xSemaphoreCreateBinary();
		SemaphoreAlgT2 = xSemaphoreCreateBinary();
		SemaphoreT2PWM = xSemaphoreCreateBinary();
		SemaphorePWMT2 = xSemaphoreCreateBinary();
		SemaphoreT2T1 = xSemaphoreCreateBinary();
		SemaphoreT1T2 = xSemaphoreCreateBinary();
		
		xSemaphoreGive(SemaphoreT1ADC);
		xSemaphoreGive(SemaphoreAlgT1);
		xSemaphoreGive(SemaphoreT2Alg);
		xSemaphoreGive(SemaphorePWMT2);
		xSemaphoreGive(SemaphoreT2T1);

		BaseType_t returnvalue;

    xTaskCreate( ADCTask, "ADCTask", windowSize*5*sizeof(float), &ADC, 0, &xADCTask );
		xTaskCreate( T1Task, "T1Task", windowSize*2*sizeof(float), &T1, 2, &xT1Task );
		xTaskCreate( AlgorithmTask, "AlgorithmTask", windowSize*15*sizeof(float), &Alg, 0, &xAlgorithmTask );
		xTaskCreate( T2Task, "T2Task", windowSize*2*sizeof(float), &T2, 3, &xT2Task );
		returnvalue = xTaskCreate( PWMTask, "PWMTask", windowSize*5*sizeof(float), &PWM, 0, &xPWMTask );
		
		am_util_debug_printf("Tasks started\n");
		
		am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);
		
		vTaskSuspend(xT1Task);
		vTaskSuspend(xT2Task);
		vTaskSuspend(xAlgorithmTask);
		vTaskSuspend(xPWMTask);
	
    vTaskStartScheduler();
		
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

}


