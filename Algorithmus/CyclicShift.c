#include "CyclicShift.h"
#include <stdlib.h>
#include <math.h>

// This function is implemented in the frequency domain
void CyclicForwardShift(float* spectrumRe, float* spectrumIm, int windowSize, float sampleFrequency, float shiftFrequency){

    float frequencyResolution = sampleFrequency/windowSize;
	
    int offset = ((int)(shiftFrequency/frequencyResolution)); // Offset in Binarray; Round up to the next bin
    float* tempDataRe = (float*) calloc(windowSize, sizeof(float));
    float* tempDataIm = (float*) calloc(windowSize, sizeof(float));

    for(int i=0;i<windowSize;i++){
        tempDataRe[i] = spectrumRe[i];
        tempDataIm[i] = spectrumIm[i];
    }

    for(int i=0;i<windowSize;i++){
        spectrumRe[((i+offset+windowSize)%windowSize)] = tempDataRe[i]; // Shift the spectrum by offset
        spectrumIm[((i+offset+windowSize)%windowSize)] = tempDataIm[i];
    }

    free(tempDataRe);
    free(tempDataIm);
    return;
}

// This function is implemented in the frequency domain
void CyclicBackwardShift(float* spectrumRe, float* spectrumIm, int windowSize, float sampleFrequency, float shiftFrequency){

		float frequencyResolution = sampleFrequency/windowSize;
	
    int offset = ((int)(shiftFrequency/frequencyResolution));
    float* tempDataRe = (float*) calloc(windowSize, sizeof(float));
    float* tempDataIm = (float*) calloc(windowSize, sizeof(float));

    for(int i=0;i<windowSize;i++){
        tempDataRe[i] = spectrumRe[i];
        tempDataIm[i] = spectrumIm[i];
    }

    for(int i=0;i<windowSize;i++){
        spectrumRe[((i-offset+windowSize)%windowSize)] = tempDataRe[i]; // Shift the spectrum by offset
        spectrumIm[((i-offset+windowSize)%windowSize)] = tempDataIm[i];
    }

    free(tempDataRe);
    free(tempDataIm);
    return;
}

void CyclicCosineForwardShift(float* spectrumRe, float* spectrumIm, int windowSize, float sampleFrequency, float shiftFrequency){
		float* tempDataRe = (float*) calloc(windowSize/2+1, sizeof(float));
    float* tempDataIm = (float*) calloc(windowSize/2+1, sizeof(float));
	
		float frequencyResolution = sampleFrequency/windowSize;
	
    int offset = ((int)(shiftFrequency/frequencyResolution));
	
		for(int i=0;i<(windowSize/2)+1;i++){
        tempDataRe[i] = spectrumRe[i];
        tempDataIm[i] = spectrumIm[i];
    }
		
		tempDataRe[0] /= 2;
		tempDataIm[0] /= 2;
		tempDataRe[(windowSize/2)] /= 2;
		tempDataIm[(windowSize/2)] /= 2;
		
		CyclicForwardShift(tempDataRe, tempDataIm, ((windowSize/2)+1), sampleFrequency, shiftFrequency);
		
		for(int i=0;i<(windowSize/2)+1;i++){
			spectrumRe[i] = tempDataRe[i];
			spectrumIm[i] = tempDataIm[i];
		}
		
		for(int i=1;i<(windowSize/2);i++){
			spectrumRe[windowSize-i] = spectrumRe[i];
			spectrumIm[windowSize-i] = -spectrumIm[i];
		}
		
		spectrumRe[0] *= 0;
		spectrumIm[0] *= 0;
		spectrumRe[(windowSize/2)] *= 2;
		spectrumIm[(windowSize/2)] *= 2;
	
		free(tempDataRe);
    free(tempDataIm);
		return;
}

// CosineForwardShift means that the time function stays real by multiplying it with a Cosine.
// This function is implemented in the frequency domain
void CyclicCosineModulation(float* spectrumRe, float* spectrumIm, int windowSize, float sampleFrequency, float shiftFrequency){

    float* tempDataRe1 = (float*) calloc(windowSize, sizeof(float));
    float* tempDataIm1 = (float*) calloc(windowSize, sizeof(float));

    float* tempDataRe2 = (float*) calloc(windowSize, sizeof(float));
    float* tempDataIm2 = (float*) calloc(windowSize, sizeof(float));

    for(int i=0;i<windowSize;i++){
        tempDataRe1[i] = spectrumRe[i];
        tempDataIm1[i] = spectrumIm[i];
        tempDataRe2[i] = spectrumRe[i];
        tempDataIm2[i] = spectrumIm[i];
    }

    // Cos(x) = 1/2(e^(ix)+e^(-ix))

    CyclicForwardShift(tempDataRe1, tempDataIm1, windowSize, sampleFrequency, shiftFrequency);
    CyclicBackwardShift(tempDataRe2, tempDataIm2, windowSize, sampleFrequency, shiftFrequency);

    for(int i=0;i<windowSize;i++){
        spectrumRe[i] = (1.0f/2)*(tempDataRe1[i]+tempDataRe2[i]);
        spectrumIm[i] = (1.0f/2)*(tempDataIm1[i]+tempDataIm2[i]);
    }

    free(tempDataRe1);
    free(tempDataRe2);
    free(tempDataIm1);
    free(tempDataIm2);
    return;
}

// This function is implemented in time domain
void CyclicCosineModulationInTime(float* data, int windowSize, float sampleFrequency, float shiftFrequency){
		for(int i=0;i<windowSize;i++){
			data[i] *= cos(2*PI*i*(shiftFrequency/sampleFrequency));
		}
}

// This function is implemented in time domain
// Saves time by not always recomputing this cosine terms
void CyclicCosineModulationInTime_static(float* data, int windowSize, float* vector){
		for(int i=0;i<windowSize;i++){
			data[i] *= vector[i];
		}
}

// This function is implemented in time domain
// Helper function to initialize a frequency shift vector to save some time in the long term
void InitializeCosineVector(float* vector, int windowSize, float sampleFrequency, float shiftFrequency, float phaseShift){
		for(int i=0;i<windowSize;i++){
			vector[i] = cos((2*PI*(((shiftFrequency + phaseShift)/sampleFrequency))*i));
		}
}

void InitializeCosineVectorAutoShift(float* vector, int windowSize, float sampleFrequency, float shiftFrequency){
		
		float frequencyResolution = sampleFrequency/windowSize;
		float effectiveShiftFrequency = ((int)(shiftFrequency/frequencyResolution))*frequencyResolution;
	
		for(int i=0;i<windowSize;i++){
			vector[i] = cos((2*PI*((effectiveShiftFrequency/sampleFrequency))*i));
		}
}
