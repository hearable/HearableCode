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

const int windowSize = 64;

volatile uint32_t PWMIndex;
volatile uint32_t* PWMArray;

volatile uint32_t ADCIndex;
volatile uint32_t* ADCArray;

const int PWMPeriod = 1024;

void
am_ctimer_isr(void)
{
		//am_util_debug_printf("Timer Interrupt!\n");
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

void PWMInterrupt(void){

    am_hal_ctimer_period_set(2, AM_HAL_CTIMER_TIMERB, PWMPeriod, PWMArray[PWMIndex]);

    PWMIndex = (PWMIndex + 1) % windowSize;
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
	float* AlgorithmPointer;
} T1Arguments;
		
typedef struct T2Arguments{
	float* AlgorithmPointer;
} T2Arguments;
		
typedef struct AlgorithmArguments{
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
		

AlgorithmArguments Alg;
T1Arguments T1;
T2Arguments T2;
		
static TaskHandle_t xADCTask = NULL;
static TaskHandle_t xT1Task = NULL;
static TaskHandle_t xAlgorithmTask = NULL;
static TaskHandle_t xT2Task = NULL;
		
static SemaphoreHandle_t SemaphoreT1ADC;
static SemaphoreHandle_t SemaphoreADCT1;
static SemaphoreHandle_t SemaphoreAlgT1;
static SemaphoreHandle_t SemaphoreT1Alg;
static SemaphoreHandle_t SemaphoreT2Alg;
static SemaphoreHandle_t SemaphoreAlgT2;
static SemaphoreHandle_t SemaphoreT2T1;
static SemaphoreHandle_t SemaphoreT1T2;
		
void ADCTask(void* args){
	am_util_debug_printf("ADC started!\n");
	
	int i = 0;
	//am_util_debug_printf("ADC init done!\n");
	xSemaphoreTake(SemaphoreT1ADC, portMAX_DELAY);
	//am_util_debug_printf("Semaphore taken!\n");
	for(;;){
		ADCArray[i] = PWMPeriod/2;
		/*
		ACQUIRE DATA HERE
		*/
		i++;
		if(i==windowSize || i==(2*windowSize) || i==(3*windowSize)){
			i%=(3*windowSize);
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
		//am_hal_stimer_counter_clear();
		ProcessData_tweaked(windowSize, 0.5f, xargs->sampleFrequency, xargs->shiftFrequency,
                    xargs->lowpassFilter, xargs->hanningWindow, xargs->sinTable, xargs->cosTable, xargs->previousWindowRe, xargs->previousWindowIm,
                    xargs->currentWindowRe, xargs->currentWindowIm, xargs->currentSample, xargs->shiftVector, xargs->currentOutput);
		ReturnWindowOutputHanning(windowSize, 0.5f, xargs->currentOutput, MID, xargs->currentResult);
		for(int i=0;i<windowSize/2;i++){
			xargs->currentResult[i] = ((PWMPeriod/2) + (xargs->currentResult[i]/2));
			//am_util_debug_printf("%f \n",xargs->currentResult[i]);
		}
		//am_util_debug_printf("Alg done!\n");
		//am_util_debug_printf("%u ms - after Alg \n",(uint32_t)(am_hal_stimer_counter_get()*1000/32768))
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
		for(int j=0;j<windowSize;j++){
			xargs->AlgorithmPointer[j] = ADCArray[(i*windowSize)+j];
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
		xSemaphoreTake(SemaphoreAlgT2, portMAX_DELAY);
		xSemaphoreTake(SemaphoreT1T2, portMAX_DELAY);
		for(int j=0;j<windowSize/2;j++){
			PWMArray[(i*windowSize/2)+j] = (uint32_t)(xargs->AlgorithmPointer[j]+0.5f);
		}
		i++;
		i%=2;
		//am_util_debug_printf("T2 done!\n");
		xSemaphoreGive(SemaphoreT2Alg);
		xSemaphoreGive(SemaphoreT2T1);
		vTaskSuspend(NULL);
	}
}

/*void PWMTask(void* args){
	PWMArguments* xargs = (PWMArguments*) args;
	int i = 0;
	
	for(;;){
		xSemaphoreTake(SemaphoreT2PWM, portMAX_DELAY);
		for(int j=0;j<windowSize/2;j++){
			//am_util_debug_printf("%f \n",xargs->dataArray[j+(i*(windowSize)/2)]);
		}
		i++;
		i%=2;
		//am_util_debug_printf("PWM done\n");
		xSemaphoreGive(SemaphorePWMT2);
		vTaskSuspend(NULL);
	}
}*/

void
run_tasks(void)
{
    //
    // Set some interrupt priorities before we create tasks or start the scheduler.
    //
    // Note: Timer priority is handled by the FreeRTOS kernel, so we won't
    // touch it here.
    //
	
		am_util_debug_printf("Starting Setup...\n");
	
		PWMIndex = 0;
		PWMArray= (uint32_t*) calloc(windowSize, sizeof(uint32_t));
		for(int i=0;i<windowSize;i++){
			PWMArray[i] = PWMPeriod-1;
		}

		ADCIndex = 0;
		ADCArray = (uint32_t*) calloc(3*windowSize, sizeof(uint32_t));
	 
		// Initialization of algorithm
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
		InitializeCosineVectorAutoShift(Alg.shiftVector, windowSize, Alg.sampleFrequency, Alg.shiftFrequency);
		Alg.hanningWindow = HanningWindow(windowSize, PERIODIC);
		Alg.lowpassFilter = LowpassFilter(windowSize,Alg.sampleFrequency,Alg.cutOffFrequency,Alg.lowpassRolloff);
		// Initialization of algorithm done
	
		// Initialization of T1
		T1.AlgorithmPointer = Alg.currentSample;
		
		// Initialization of T2
		T2.AlgorithmPointer = Alg.currentResult;
		
		SemaphoreT1ADC = xSemaphoreCreateBinary();
		SemaphoreADCT1 = xSemaphoreCreateBinary();
		SemaphoreAlgT1 = xSemaphoreCreateBinary();
		SemaphoreT1Alg = xSemaphoreCreateBinary();
		SemaphoreT2Alg = xSemaphoreCreateBinary();
		SemaphoreAlgT2 = xSemaphoreCreateBinary();
		SemaphoreT2T1 = xSemaphoreCreateBinary();
		SemaphoreT1T2 = xSemaphoreCreateBinary();
		
		xSemaphoreGive(SemaphoreT1ADC);
		xSemaphoreGive(SemaphoreAlgT1);
		xSemaphoreGive(SemaphoreT2Alg);
		xSemaphoreGive(SemaphoreT2T1);

		BaseType_t returnvalue;

    xTaskCreate( ADCTask, "ADCTask", windowSize*5*sizeof(float), NULL, 0, &xADCTask );
		xTaskCreate( T1Task, "T1Task", windowSize*2*sizeof(float), &T1, 2, &xT1Task );
		xTaskCreate( AlgorithmTask, "AlgorithmTask", windowSize*15*sizeof(float), &Alg, 0, &xAlgorithmTask );
		xTaskCreate( T2Task, "T2Task", windowSize*2*sizeof(float), &T2, 3, &xT2Task );
		
		am_util_debug_printf("Tasks started\n");
		
		//am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    //am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);
		
		am_hal_gpio_pin_config(AM_BSP_GPIO_LED3, AM_HAL_PIN_30_TCTB2);
		
		// PWM Interrupt Setup
		am_hal_ctimer_config_single(2, AM_HAL_CTIMER_TIMERB,
                                (AM_HAL_CTIMER_FN_PWM_REPEAT |
                                 AM_HAL_CTIMER_HFRC_12MHZ |
                                 AM_HAL_CTIMER_INT_ENABLE |
                                 AM_HAL_CTIMER_PIN_ENABLE));


    am_hal_ctimer_period_set(2, AM_HAL_CTIMER_TIMERB, PWMPeriod, PWMPeriod-1);

		am_hal_ctimer_int_register(AM_HAL_CTIMER_INT_TIMERB2, PWMInterrupt);
		
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERB2);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_CTIMER);
    am_hal_interrupt_master_enable();
		//am_hal_interrupt_master_disable();

    am_hal_ctimer_start(2, AM_HAL_CTIMER_TIMERB);
		
		vTaskSuspend(xT1Task);
		vTaskSuspend(xT2Task);
		vTaskSuspend(xAlgorithmTask);
	
    vTaskStartScheduler();
		
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

}


