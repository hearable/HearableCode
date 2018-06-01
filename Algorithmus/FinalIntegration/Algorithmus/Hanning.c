#include "Hanning.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>


// Implements the Hanning Window in time domain
float* HanningWindow(int windowSize, HanningPeriodicity periodicity){
    float *hanningValues;
    int effectiveWindowSize = 0;
    int halfWindowSize = 0;
    int copyIndex = 0;

    hanningValues = (float*) calloc(windowSize, sizeof(float)); // Allocate needed memory
    memset(hanningValues, 0, windowSize*sizeof(float)); // Make sure allocated memory is all zeros t first

    if(periodicity == PERIODIC){
        effectiveWindowSize = windowSize - 1;
    } else {
        effectiveWindowSize = windowSize;
    }

    if(effectiveWindowSize%2==0){
        halfWindowSize = effectiveWindowSize/2;
        for(int i=0; i<halfWindowSize; i++){ // Need only to calculate half the values; the function is symmetric
            hanningValues[i] = 0.5*(1-cos(2*PI*(i+1)/(effectiveWindowSize+1)));
        }
        copyIndex = halfWindowSize - 1;
        for(int i=halfWindowSize; i<effectiveWindowSize; i++){ // Copy symmetric values
            hanningValues[i] = hanningValues[copyIndex];
            copyIndex--;
        }
    } else {
        halfWindowSize = (effectiveWindowSize+1)/2;
        for(int i=0; i<halfWindowSize; i++){ // Need only to calculate half the values; the function is symmetric
            hanningValues[i] = 0.5*(1-(cos(2*PI*(i+1)/(effectiveWindowSize+1))));
        }
        copyIndex = halfWindowSize - 2;
        for(int i=halfWindowSize; i<effectiveWindowSize; i++){ // Copy symmetric values
            hanningValues[i] = hanningValues[copyIndex];
            copyIndex--;
        }
    }

    if(periodicity == PERIODIC){ // periodic approach leads to constant amplitude with overlap-add approach
        for(int i=windowSize-1; i>0; i--){ // Push every value one position forward
            hanningValues[i] = hanningValues[i-1];
        }
        hanningValues[0] = 0.0f; // Set leftmost value to zero
    }

    return hanningValues;
}

// After applying OverlapAddHanning to two processed windows, apply ReturnWindowOutputHanning to get
// the part of the sum which you're after -> HEAD, MID, TAIL
void ReturnWindowOutputHanning(int windowSize, float overlapPercentage, float* overlapAddResult, OverlapPart part, float* result){
    int overlapIndex = (int)((windowSize)*(1-overlapPercentage));
    switch(part){
        case(HEAD):
            for(int i=overlapIndex;i<windowSize;i++){
                result[i-overlapIndex] = overlapAddResult[i];
            }
        break;
        case(MID):
            for(int i=overlapIndex;i<windowSize;i++){
                result[i-overlapIndex] = overlapAddResult[i];
            }
        break;
        case(TAIL):
            for(int i=windowSize;i<windowSize+overlapIndex;i++){
                result[i-windowSize] = overlapAddResult[i];
            }
        break;
    }
}

// Applies the overlap-Add method to Hanning windowed-STFT
void OverlapAddHanning(int windowSize, float overlapPercentage, float* currentWindowRe, float* previousWindowRe, float* currentResult){
    int overlapIndex = (int)((windowSize)*(1.0f-overlapPercentage));

    for(int i=0;i<overlapIndex;i++){
        currentResult[i] = previousWindowRe[i];
    }
    for(int i=overlapIndex;i<windowSize;i++){
        currentResult[i] = currentWindowRe[(i-overlapIndex)] + previousWindowRe[i];
    }
    for(int i=windowSize;i<windowSize+overlapIndex;i++){
        currentResult[i] = currentWindowRe[i-overlapIndex];
    }
}

