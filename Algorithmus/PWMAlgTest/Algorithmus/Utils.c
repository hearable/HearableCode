// General utilities to separate functionality from main.c

#include <stdio.h>
#include <stdlib.h>
#include "Hanning.h"
#include "FFT.h"
#include "math.h"
#include "Lowpass.h"
#include "CyclicShift.h"
#include "am_util.h"
#include "Utils.h"

#define XT_PERIOD               32768

void
stimer_init(void)
{
    // Configure the STIMER and run
    am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);
    am_hal_stimer_config(AM_HAL_STIMER_XTAL_32KHZ);
}

// Sets the previous data and applies the window function to the current sequence.
void ApplyWindow(int windowSize, float* currentWindowRe, float* currentWindowIm, float* previousWindowRe, float* previousWindowIm, float* currentSample, float* window){
    for(int i=0;i<windowSize;i++){
        previousWindowRe[i] = currentWindowRe[i];
        previousWindowIm[i] = currentWindowIm[i];
        currentWindowRe[i] = currentSample[i] * window[i];
        currentWindowIm[i] = 0; // All sampled data is real
    }
}

// Helper function to get a window full of ones
void GetSamples(int windowSize, float* currentSample){
    for(int i=0; i<windowSize; i++){
        if(i%2==0){
            if(i%4==0){
                currentSample[i] = 1;
            }
            else{
                currentSample[i] = 1;
            }
        }
        else {
            currentSample[i] = 1;
        }
    }
}

// Helper function to get window full of twos
void GetSamples2(int windowSize, float* currentSample){
    for(int i=0; i<windowSize; i++){
        if(i%2==0){
            if(i%4==0){
                currentSample[i] = 2;
            }
            else{
                currentSample[i] = 2;
            }
        }
        else {
            currentSample[i] = 2;
        }
    }
}

// GetSamples -> ApplyWindow -> (Any Algorithm you like) -> OverlapAdd -> ReturnWindowOutput

// Does all the processing with one function call:
// ApplyWindow -> FFT -> Lowpass -> CosineShift -> IFFT -> OverlapAdd -> ReturnWindowOutput
/*float* ProcessData(int windowSize, float overlappercentage, float sampleFrequency, float shiftFrequency,
                    float* lowpassFilter, float* window, float* previousWindowRe, float* previousWindowIm,
                    float* currentWindowRe, float* currentWindowIm, float* currentSample, OverlapPart part){

    int overlapIndex = (int)((windowSize+1)*(1.0f-overlappercentage));
    ApplyWindow(windowSize,currentWindowRe,currentWindowIm,previousWindowRe,previousWindowIm,currentSample,window);
																			
    Fft_transform(currentWindowRe, currentWindowIm, windowSize);
											
    for(int i=0;i<windowSize;i++){
            currentWindowIm[i] *= lowpassFilter[i];
            currentWindowRe[i] *= lowpassFilter[i];
    }

    CyclicCosineModulation(currentWindowRe, currentWindowIm, windowSize, sampleFrequency, shiftFrequency);
    Fft_inverseTransform(currentWindowRe, currentWindowIm, windowSize);
		
    float* currentResult = OverlapAddHanning(windowSize, overlappercentage, currentWindowRe, previousWindowRe);
    return ReturnWindowOutputHanning(windowSize, overlappercentage,currentResult, part);
}*/
										
void ProcessData_tweaked(int windowSize, float overlappercentage, float sampleFrequency, float shiftFrequency,
                    float* lowpassFilter, float* window, float* sin_table, float* cos_table, float* previousWindowRe, float* previousWindowIm,
                    float* currentWindowRe, float* currentWindowIm, float* currentSample, float* shiftVector, float* currentOutput){

    int overlapIndex = (int)((windowSize+1)*(1.0f-overlappercentage));
    ApplyWindow(windowSize,currentWindowRe,currentWindowIm,previousWindowRe,previousWindowIm,currentSample,window);				
    Fft_transform_static(currentWindowRe, currentWindowIm, windowSize, sin_table, cos_table);												
    /*for(int i=0;i<windowSize;i++){
            currentWindowIm[i] *= lowpassFilter[i];
            currentWindowRe[i] *= lowpassFilter[i];
    }*/
		CyclicCosineForwardShift(currentWindowRe, currentWindowIm, windowSize, sampleFrequency, shiftFrequency);
		/*for(int i=0;i<windowSize;i++){
            currentWindowIm[i] *= lowpassFilter[i];
            currentWindowRe[i] *= lowpassFilter[i];
    }*/
    Fft_inverseTransform_static(currentWindowRe, currentWindowIm, windowSize, sin_table, cos_table);

    OverlapAddHanning(windowSize, overlappercentage, currentWindowRe, previousWindowRe, currentOutput);
}
	/*									
float* ProcessDataReduced(int windowSize, float overlappercentage, float sampleFrequency, float shiftFrequency,
                    float* lowpassFilter, float* window, float* previousWindowRe, float* previousWindowIm,
                    float* currentWindowRe, float* currentWindowIm, float* cosinevector, float* currentSample, OverlapPart part){

    int overlapIndex = (int)((windowSize+1)*(1.0f-overlappercentage));
    ApplyWindow(windowSize,currentWindowRe,currentWindowIm,previousWindowRe,previousWindowIm,currentSample,window);
    CyclicCosineModulationInTime(currentWindowRe,windowSize,sampleFrequency,shiftFrequency);
    float* currentResult = OverlapAddHanning(windowSize, overlappercentage, currentWindowRe, previousWindowRe);
    return ReturnWindowOutputHanning(windowSize, overlappercentage,currentResult, part);
}*/
				
uint32_t timepassed_inms(uint32_t clockspeed){
	return (uint32_t)((((double)am_hal_stimer_counter_get())/(clockspeed))*1000);
}


// Test functionality with some pre-specified data
// You should allocate about 20*windowSize*sizeof(float) bytes in the heap
void TestSetup(Verbosity verb, int windowSize, int iterations){

    // Declaration + Initialization of used data arrays
		float* currentOutput;
    currentOutput = (float*) calloc(2*windowSize, sizeof(float));

    float* hanningWindow;
    hanningWindow = HanningWindow(windowSize, PERIODIC);

    float* previousWindowRe;
    previousWindowRe = (float*) calloc(windowSize, sizeof(float));
    float* previousWindowIm;
    previousWindowIm = (float*) calloc(windowSize, sizeof(float));

    float* currentWindowRe;
    currentWindowRe = (float*) calloc(windowSize, sizeof(float));
    float* currentWindowIm;
    currentWindowIm = (float*) calloc(windowSize, sizeof(float));

    float* currentSample;
    currentSample = (float*) calloc(windowSize, sizeof(float));

    float sampleFrequency = 12000.0f;
    float cutOffFrequency = 6000.0f; // Sensibly audible frequency cutoff
    float shiftFrequency = 440.0f; // Use even multiples of the frequency resolution for coherent overlap
    int lowpassRolloff = 4;

    float* lowpass = LowpassFilter(windowSize,sampleFrequency,cutOffFrequency,lowpassRolloff);

    float overlappercentage = 0.5f;
    int overlapIndex = (int)((windowSize)*(1.0f-overlappercentage)); // Upward adjusted index for overlapping windows

		float* sinTable = (float*) calloc(windowSize/2, sizeof(float));
		float* cosTable = (float*) calloc(windowSize/2, sizeof(float));
		
		for (int i = 0; i < windowSize / 2; i++) {
			cosTable[i] = cos(2 * PI * i / windowSize);
			sinTable[i] = sin(2 * PI * i / windowSize);
		}

		float* cosinevector = (float*) calloc(windowSize, sizeof(float));
		
		InitializeCosineVectorAutoShift(cosinevector, windowSize, sampleFrequency, shiftFrequency);
		
		float* currentResult;
    currentResult = (float*) calloc(windowSize, sizeof(float));

		uint32_t time = 0;
		stimer_init();

    for(int i=0; i<iterations; i++){
        GetSamples(windowSize,currentSample);
				ProcessData_tweaked(windowSize, overlappercentage, sampleFrequency, shiftFrequency, lowpass, hanningWindow, sinTable, cosTable, previousWindowRe, previousWindowIm, currentWindowRe, currentWindowIm, currentSample, cosinevector, currentOutput);
			
				if(i == (iterations-1)){
						ReturnWindowOutputHanning(windowSize, overlappercentage, currentOutput, MID, currentResult);
						if(verb == VERBOSE){ // MID
							for(int j=0;j<windowSize-overlapIndex;j++){
								am_util_stdio_printf("%f \n",currentResult[j]);
							}
						}
						ReturnWindowOutputHanning(windowSize, overlappercentage, currentOutput, TAIL, currentResult);
					if(verb == VERBOSE){ // TAIL
							for(int j=0;j<overlapIndex;j++){
								am_util_stdio_printf("%f \n",currentResult[j]);
							}
						}
            free(currentResult);
        } else if(i == 0){
					ReturnWindowOutputHanning(windowSize, overlappercentage, currentOutput, HEAD, currentResult);
					if(verb == VERBOSE){// HEAD
            for(int j=0;j<overlapIndex;j++){
               am_util_stdio_printf("%f \n",currentResult[j]);
            }
					}
          free(currentResult);
        }  else {
					ReturnWindowOutputHanning(windowSize, overlappercentage, currentOutput, MID, currentResult);
						if(verb == VERBOSE){// MID
							for(int j=0;j<windowSize-overlapIndex;j++){
								am_util_stdio_printf("%f \n",currentResult[j]);
							}
						}
        }
				
    }
		free(currentOutput);
		
		time = timepassed_inms(XT_PERIOD); // Get time which has passed
		am_util_stdio_printf("%u ms\n",time);
		
    return;
}
