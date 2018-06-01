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

float* Interpolate(int windowSize, float sampleFrequency, float targetSampleFrequency, float* samples){
		int factor = (int)targetSampleFrequency/sampleFrequency;
	
		float* resultVectorRe = calloc(factor*windowSize, sizeof(float));
		float* resultVectorIm = calloc(factor*windowSize, sizeof(float));
		float* lowpass = LowpassFilter(factor*windowSize,factor*sampleFrequency,sampleFrequency,0);
		
		for(int i=0;i<windowSize;i++){
			resultVectorRe[factor*i] = samples[i];
		}
		
		Fft_transform(resultVectorRe, resultVectorIm, factor*windowSize);
		
		for(int i=0;i<2*windowSize;i++){
			resultVectorRe[(i+1)%(factor*windowSize)]*=lowpass[i]*factor;
			resultVectorIm[(i+1)%(factor*windowSize)]*=lowpass[i]*factor;
		}
		
		Fft_inverseTransform(resultVectorRe, resultVectorIm, factor*windowSize);
		
		free(lowpass);
		free(resultVectorIm);
		
		return resultVectorRe;
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
    for(int i=0;i<windowSize;i++){
            currentWindowIm[i] *= lowpassFilter[i];
            currentWindowRe[i] *= lowpassFilter[i];
    }
		CyclicCosineForwardShift(currentWindowRe, currentWindowIm, windowSize, sampleFrequency, shiftFrequency);
    Fft_inverseTransform_static(currentWindowRe, currentWindowIm, windowSize, sin_table, cos_table);

    OverlapAddHanning(windowSize, overlappercentage, currentWindowRe, previousWindowRe, currentOutput);
}
					
void MelFilter(int melSize, float samplingFrequency, int windowSize, float* real, float* imag, float* melValues);

void ProcessData_tweaked_MFCC(int windowSize, int melSize, float overlappercentage, float sampleFrequency, float shiftFrequency,
                    float* lowpassFilter, float* window, float* sin_table, float* cos_table, float* previousWindowRe, float* previousWindowIm,
                    float* currentWindowRe, float* currentWindowIm, float* currentSample, float* shiftVector, float* currentOutput){

		float* melValues = calloc(melSize, sizeof(float));
		float* testreal = (float*) calloc(windowSize, sizeof(float));
		float* testimag = (float*) calloc(windowSize, sizeof(float));
											
    int overlapIndex = (int)((windowSize+1)*(1.0f-overlappercentage));
    ApplyWindow(windowSize,currentWindowRe,currentWindowIm,previousWindowRe,previousWindowIm,currentSample,window);				
    Fft_transform_static(currentWindowRe, currentWindowIm, windowSize, sin_table, cos_table);												
						
		MelFilter(melSize, sampleFrequency, windowSize, testreal, testimag, melValues);
											
		for(int i=0;i<melSize;i++){
			am_util_debug_printf("current melValue: %f \n", melValues[i]);
		}									
		
		// void MelFilter(int melSize, float samplingFrequency, int windowSize, float* real, float* imag, float* melValues){
		
    Fft_inverseTransform_static(currentWindowRe, currentWindowIm, windowSize, sin_table, cos_table);

    OverlapAddHanning(windowSize, overlappercentage, currentWindowRe, previousWindowRe, currentOutput);
}
										
void createMelFilter(int melSize, float samplingFrequency, int windowSize, float* melFrequencies){
	
	float freqStep = samplingFrequency/melSize;
	
	float freq = 0;
	for(int i=0;i<melSize+2;i++){
		melFrequencies[i] = freq;
		freq = 2595 * log(1+(i*freqStep/700));
		am_util_stdio_printf("%f \n",melFrequencies[i]);
	}
}

float* createTriangFilter(int midpoint, int endpoint){
	float* returnvalue = (float*) calloc((endpoint), sizeof(float));
	
	float upscale = 1.0f/(midpoint);
	float downscale = 1.0f/(endpoint-midpoint);
	
	for(int i=0;i<midpoint;i++){
		returnvalue[i] = upscale * i;
	}
	for(int i=midpoint;i<=endpoint;i++){
		returnvalue[i] = 1 - (downscale * (i-midpoint));
	}
	
	return returnvalue;
}

void applyMelFilter(int melSize, float samplingFrequency, int windowSize, float* melFilter, float* real, float* imag, float* melValues){
	
	float frequencyResolution = samplingFrequency/windowSize;
	float currentMelValue = 0;
	int startPoint = 0;
	int midPoint = 0;
	int endPoint = 0;
	for(int i=0;i<melSize;i++){
		startPoint = (int)(melFilter[i]/frequencyResolution);
		midPoint = (int)(melFilter[i+1]/frequencyResolution);
		endPoint = (int)(melFilter[i+2]/frequencyResolution);
		float* triangFilter = createTriangFilter(midPoint, endPoint);
		for(int j=startPoint;j<endPoint;j++){
			currentMelValue += triangFilter[i-startPoint]*log(real[i]*real[i]+imag[i]*imag[i]);
		}
		melValues[i] = currentMelValue;
		currentMelValue = 0;
		free(triangFilter);
	}
}

void MelFilter(int melSize, float samplingFrequency, int windowSize, float* real, float* imag, float* melValues){
	float* melFrequencies = calloc(melSize+2, sizeof(float));
	createMelFilter(melSize, samplingFrequency, windowSize, melFrequencies);
	applyMelFilter(melSize, samplingFrequency, windowSize, melFrequencies, real, imag, melValues);
	free(melFrequencies);
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


// Test functionality with some pre-specified data
// You should allocate about 20*windowSize*sizeof(float) bytes in the heap
void TestSetup2(Verbosity verb, int windowSize, int iterations){

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
