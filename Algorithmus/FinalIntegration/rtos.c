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

const int windowSize = 256;

// PWMIndex and PWMArray are used for data exchange between the Algorithm and the PWM generator
volatile uint32_t PWMIndex;
volatile uint32_t* PWMArray;

// ADCIndex and ADCArray are used for data exchange between the Algorithm and the ADC
volatile uint32_t ADCIndex;
volatile uint32_t* ADCArray;

const int PWMPeriod = 1024; 
const int ADCPeriod = 1024;

// For documentation on how the semaphores are used, please see the corresponding scheme
static SemaphoreHandle_t SemaphoreIntADC;
static SemaphoreHandle_t SemaphoreADCT1;
static SemaphoreHandle_t SemaphoreAlgT1;
static SemaphoreHandle_t SemaphoreT2Alg;
static SemaphoreHandle_t SemaphoreAlgT2;
static SemaphoreHandle_t SemaphoreT2T1;

static TaskHandle_t xADCTask = NULL;
static TaskHandle_t xT1Task = NULL;
static TaskHandle_t xT2Task = NULL;

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

void
am_adc_isr(void)
{
    uint32_t ui32Status;
		//am_util_debug_printf("Interrupt! \n");

    ui32Status = am_hal_adc_int_status_get(true);
    am_hal_adc_int_clear(ui32Status);
	
		xSemaphoreGiveFromISR(SemaphoreIntADC, NULL);
}

void PWMInterrupt(void){ // This is called if the timer associated with PWM generation causes an interrupt
		
    am_hal_ctimer_period_set(2, AM_HAL_CTIMER_TIMERB, PWMPeriod, PWMArray[PWMIndex]);
		//am_util_debug_printf("PWM: %u \n",PWMArray[PWMIndex]);
		//am_util_debug_printf("PWMIndex: %u \n",PWMIndex);
    PWMIndex = (PWMIndex + 1) % (2*windowSize);
}

uint32_t am_freertos_sleep(uint32_t idleTime)
{
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    return 0;
}

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
    while (1);
}

void
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    while (1)
    {
        __asm("BKPT #0\n") ; // Break into the debugger
    }
}

typedef struct T1Arguments{
	float* AlgorithmPointer;
} T1Arguments;
		
typedef struct T2Arguments{
	float* AlgorithmPointer;
} T2Arguments;
		
typedef struct AlgorithmArguments{ // All data storage which is used in the algorithm is statically allocated to facilitate the control of stack/heap size
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
		

AlgorithmArguments algArgs;
T1Arguments T1;
T2Arguments T2;
		
void ADCTask(void* args){
	uint32_t ui32FifoData;
	am_util_debug_printf("ADC started!\n");

	//am_util_debug_printf("ADC init done!\n");
	//am_util_debug_printf("Semaphore taken!\n");
	for(;;){
		xSemaphoreTake(SemaphoreIntADC, portMAX_DELAY);
		//taskENTER_CRITICAL();
		for(int i=0;i<12;i++)
      {
        ui32FifoData = am_hal_adc_fifo_pop();
				//am_util_stdio_printf("%d \n", ((ui32FifoData)&0x0000FFC0)>>6);
				ADCArray[ADCIndex] = (((AM_HAL_ADC_FIFO_FULL_SAMPLE(ui32FifoData))&0x00003FC0)>>6); // 8Bit Mask
				//am_util_debug_printf("%d ADC \n", ADCArray[ADCIndex]);
				
				if((ADCIndex+1)%windowSize == 0){
					for(int i=0;i<windowSize/2;i++){
						ADCArray[(ADCIndex+1+i)%(3*windowSize)] = ADCArray[((ADCIndex-(windowSize/2))+1+i)%(3*windowSize)];
					}
					ADCIndex += (windowSize/2);
					//am_util_debug_printf("Should start T1 now... \n");
					xSemaphoreGive(SemaphoreADCT1);
					//vTaskResume(xT1Task);
				}
				
				ADCIndex = (ADCIndex + 1) % (3*windowSize);
				
        //g_ui32ADCSampleBuffer[g_ui32ADCSampleIndex] = AM_HAL_ADC_FIFO_FULL_SAMPLE(ui32FifoData);
        //g_ui32ADCSampleIndex = (g_ui32ADCSampleIndex + 1) & ADC_SAMPLE_INDEX_M;
      }
			//taskEXIT_CRITICAL();
			//am_util_debug_printf("ADC done!\n");
	}		
}

void Algorithm(void){
	//am_util_debug_printf("Alg started!\n");
	
		//am_hal_stimer_counter_clear();
		// For reference on what ProcessData_tweaked does, refer to Utils.h
		ProcessData_tweaked(windowSize, 0.5f, algArgs.sampleFrequency, algArgs.shiftFrequency,
                    algArgs.lowpassFilter, algArgs.hanningWindow, algArgs.sinTable, algArgs.cosTable, algArgs.previousWindowRe, algArgs.previousWindowIm,
                    algArgs.currentWindowRe, algArgs.currentWindowIm, algArgs.currentSample, algArgs.shiftVector, algArgs.currentOutput);
		ReturnWindowOutputHanning(windowSize, 0.5f, algArgs.currentOutput, MID, algArgs.currentResult);
		for(int i=0;i<windowSize/2;i++){
			//am_util_debug_printf("%f  a\n",algArgs.currentResult[i]);
			algArgs.currentResult[i] = ((PWMPeriod/2) + (algArgs.currentResult[i])); // Dividing by 2.1f to be sure to never get 0 or PWMPeriod. Those values don't work well with PWM generation. Only affects Amplitude
			//am_util_debug_printf("%f Alg b\n",algArgs.currentResult[i]);
		}
		//am_util_debug_printf("Alg done!\n");
		xSemaphoreGive(SemaphoreAlgT2);
		//am_util_debug_printf("%u ms - after Alg \n",(uint32_t)(am_hal_stimer_counter_get()*1000/32768))
		//vTaskResume(xT2Task);
		//vTaskSuspend(NULL);
}

void T1Task(void* args){
	am_util_debug_printf("T1 started!\n");
	T1Arguments* xargs = (T1Arguments*) args;
	int i = 0; // The ADCArray has storage space for 3 windowSizes of data. Therefore, i is used to calculate which data segment is next to be used.
	//int k = 0; // The HoldAlgorithm array stores 5 windowSizes
	//float* HoldAlgorithm = calloc(5*windowSize, sizeof(float));
	//bool delayFlag = false;
	bool delayFlag = true;
	
	for(;;){
		xSemaphoreTake(SemaphoreADCT1, portMAX_DELAY);
		//taskENTER_CRITICAL(); // Make sure the data is transfered completely
		for(int j=0;j<windowSize;j++){
			//HoldAlgorithm[(k*windowSize)+j] = ADCArray[(i*windowSize)+j];
			//xargs->AlgorithmPointer[j] = HoldAlgorithm[(k*windowSize)+j];
			xargs->AlgorithmPointer[j] = ADCArray[(i*windowSize)+j];
		}
		i++;
		i%=3;
		/*k++;
		k%=5;
		if(k==0){
			delayFlag = true;
		}*/
		//taskEXIT_CRITICAL();
		//am_util_debug_printf("T1 done!\n");
		if(delayFlag == true){
			Algorithm();
		}
		//vTaskResume(xAlgorithmTask);
		//vTaskSuspend(NULL);
	}
}

void T2Task(void* args){
	T2Arguments* xargs = (T2Arguments*) args;
	int i = 0; // PWMArray has storage space for 2 windowSizes of data. This allows for smooth operation. 
	
	//float* container = calloc(windowSize, sizeof(float));
	for(;;){
		xSemaphoreTake(SemaphoreAlgT2, portMAX_DELAY);
		//taskENTER_CRITICAL(); // Make sure the data is transfered completely
		for(int j=0;j<windowSize;j++){ // Each data point is doubled. This allows for a higher PWM modulation frequency which isn't audible. (Kind of a hack, since the 48Mhz Clock isn't available on timers)
			//PWMArray[(i*windowSize)+2*j] = xargs->AlgorithmPointer[j];
			//PWMArray[(i*windowSize)+(2*j)+1] = xargs->AlgorithmPointer[j];
			PWMArray[(i*windowSize)+j] = xargs->AlgorithmPointer[j];
			//container[(2*j)] = xargs->AlgorithmPointer[j];
			//container[(2*j)+1] = 0.0f;
		}
		
		//float * temp = Interpolate(windowSize, 12000, 24000, container);
		
		//for(int j=0;j<windowSize;j++){ // Each data point is doubled. This allows for a higher PWM modulation frequency which isn't audible. (Kind of a hack, since the 48Mhz Clock isn't available on timers)
			//PWMArray[(i*windowSize)+j] = (uint32_t)(temp[j]+0.5f);
			//am_util_debug_printf("%u\n",(uint32_t)(container[j]+0.5f));
		//}
		//free(temp);
		i++;
		i%=2;
		//am_util_debug_printf("RTT: %u \n",timepassed_inms(32768));
		//taskEXIT_CRITICAL();
		//am_util_debug_printf("T2 done!\n");
		//vTaskSuspend(NULL);
	}
}

// The ADC is set to operate on signals from 0-1.5V
void
adc_config(void)
{
    am_hal_adc_config_t sADCConfig;

    am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_ADC);

    sADCConfig.ui32Clock = AM_HAL_ADC_CLOCK_HFRC;
    sADCConfig.ui32TriggerConfig = AM_HAL_ADC_TRIGGER_SOFT;
    sADCConfig.ui32Reference = AM_HAL_ADC_REF_INT_1P5;
    sADCConfig.ui32ClockMode = AM_HAL_ADC_CK_LOW_POWER;
    sADCConfig.ui32PowerMode = AM_HAL_ADC_LPMODE_0;
    sADCConfig.ui32Repeat = AM_HAL_ADC_REPEAT;
    am_hal_adc_config(&sADCConfig);

    am_hal_adc_int_enable(AM_HAL_ADC_INT_FIFOOVR1);

    am_hal_adc_slot_config(0, AM_HAL_ADC_SLOT_AVG_1 |
															AM_HAL_ADC_SLOT_8BIT |
                              AM_HAL_ADC_SLOT_CHSEL_SE0 |
                              AM_HAL_ADC_SLOT_ENABLE);

    am_hal_adc_enable();
}

void
init_timerA3_for_ADC(void)
{

    am_hal_ctimer_config_single(3, AM_HAL_CTIMER_TIMERA,
                                   AM_HAL_CTIMER_HFRC_12MHZ |
                                   AM_HAL_CTIMER_FN_REPEAT |
                                   AM_HAL_CTIMER_INT_ENABLE |
                                   AM_HAL_CTIMER_PIN_ENABLE);

    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA3);
		//TimerNumber, TimerSegment, Period, OnTime
    am_hal_ctimer_period_set(3, AM_HAL_CTIMER_TIMERA, ADCPeriod, ADCPeriod-1);

    //
    // Enable the timer A3 to trigger the ADC directly
    //
    am_hal_ctimer_adc_trigger_enable();

    //
    // Start the timer (TimerNumber, TimerSegment)
    //
    am_hal_ctimer_start(3, AM_HAL_CTIMER_TIMERA);
}

void
run_tasks(void)
{
		am_util_debug_printf("Starting Setup...\n");
	
		am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_ADC, 254);
		am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_CTIMER, 255);
	
		PWMIndex = 0;
		PWMArray= (uint32_t*) calloc(2*windowSize, sizeof(uint32_t));
		for(int i=0;i<2*windowSize;i++){
			PWMArray[i] = PWMPeriod-1;
		}

		ADCIndex = 0;
		ADCArray = (uint32_t*) calloc(3*windowSize, sizeof(uint32_t));
	 
		// Initialization of algorithm
		algArgs.currentSample = calloc(windowSize, sizeof(float));
		algArgs.currentWindowRe = calloc(windowSize, sizeof(float));
		algArgs.currentWindowIm = calloc(windowSize, sizeof(float));
		algArgs.previousWindowRe = calloc(windowSize, sizeof(float));
		algArgs.previousWindowIm = calloc(windowSize, sizeof(float));
		algArgs.hanningWindow = calloc(windowSize, sizeof(float));
		algArgs.lowpassFilter = calloc(windowSize, sizeof(float));
		algArgs.currentOutput = calloc(2*windowSize, sizeof(float));
		algArgs.currentResult = calloc(windowSize, sizeof(float));
		algArgs.sampleFrequency = 12000.0f;
		algArgs.shiftFrequency = 100.0f;
		algArgs.cutOffFrequency = 4000.0f;
		algArgs.lowpassRolloff = 4;
		algArgs.sinTable = (float*) calloc(windowSize/2, sizeof(float));
		algArgs.cosTable = (float*) calloc(windowSize/2, sizeof(float));
		
		for (int i = 0; i < windowSize / 2; i++) {
			algArgs.cosTable[i] = cos(2 * PI * i / windowSize);
			algArgs.sinTable[i] = sin(2 * PI * i / windowSize);
		}
		
		algArgs.shiftVector = (float*) calloc(windowSize, sizeof(float));
		InitializeCosineVectorAutoShift(algArgs.shiftVector, windowSize, algArgs.sampleFrequency, algArgs.shiftFrequency);
		algArgs.hanningWindow = HanningWindow(windowSize, PERIODIC);
		algArgs.lowpassFilter = LowpassFilter(windowSize,algArgs.sampleFrequency,algArgs.cutOffFrequency,algArgs.lowpassRolloff);
		// Initialization of algArgsorithm done
	
		// Initialization of T1
		T1.AlgorithmPointer = algArgs.currentSample;
		
		// Initialization of T2
		T2.AlgorithmPointer = algArgs.currentResult;
		
		SemaphoreIntADC = xSemaphoreCreateBinary();
		SemaphoreADCT1 = xSemaphoreCreateBinary();
		SemaphoreAlgT1 = xSemaphoreCreateBinary();
		SemaphoreT2Alg = xSemaphoreCreateBinary();
		SemaphoreAlgT2 = xSemaphoreCreateBinary();
		SemaphoreT2T1 = xSemaphoreCreateBinary();
		
		xSemaphoreGive(SemaphoreAlgT1);
		xSemaphoreGive(SemaphoreT2Alg);
		xSemaphoreGive(SemaphoreT2T1);

    xTaskCreate( ADCTask, "ADCTask", 400, NULL, 4, &xADCTask );
		xTaskCreate( T1Task, "T1Task", 400, &T1, 2, &xT1Task );
//		xTaskCreate( AlgorithmTask, "AlgorithmTask", 400, &Alg, 0, &xAlgorithmTask );
		xTaskCreate( T2Task, "T2Task", 400, &T2, 3, &xT2Task );
		
		am_hal_gpio_pin_config(AM_BSP_GPIO_LED3, AM_HAL_PIN_30_TCTB2);
		am_hal_gpio_pin_config(16, AM_HAL_PIN_16_ADCSE0);
		
		// PWM Interrupt Setup
		am_hal_ctimer_config_single(2, AM_HAL_CTIMER_TIMERB,
                                (AM_HAL_CTIMER_FN_PWM_REPEAT |
																 AM_HAL_CTIMER_HFRC_12MHZ|
                                 AM_HAL_CTIMER_INT_ENABLE |
                                 AM_HAL_CTIMER_PIN_ENABLE));

		
    am_hal_ctimer_period_set(2, AM_HAL_CTIMER_TIMERB, PWMPeriod, PWMPeriod/2);
		am_hal_ctimer_int_register(AM_HAL_CTIMER_INT_TIMERB2, PWMInterrupt);
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERB2);
		
		init_timerA3_for_ADC();
		adc_config();
		am_hal_adc_trigger();
		
		// Enable global interrupts
		am_hal_interrupt_enable(AM_HAL_INTERRUPT_ADC);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_CTIMER);
    am_hal_interrupt_master_enable();

		// Start timers for both PWM and ADC
    am_hal_ctimer_start(2, AM_HAL_CTIMER_TIMERB);
		am_hal_ctimer_start(3, AM_HAL_CTIMER_TIMERA);
		
		//vTaskSuspend(xT1Task);
		//vTaskSuspend(xT2Task);
		//vTaskSuspend(xAlgorithmTask);
		am_util_debug_printf("Tasks started\n");
		stimer_init();
	
    vTaskStartScheduler();
		
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);

}


