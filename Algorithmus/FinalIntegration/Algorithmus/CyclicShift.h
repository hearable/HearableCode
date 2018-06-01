#ifndef CYCLICSHIFT_H_INCLUDED
#define CYCLICSHIFT_H_INCLUDED

#ifndef PI
	#define PI 3.14159265359
#endif

// Same as multiplying a function in time by e^(-2*pi*i*(shiftFrequency/sampleFrequency))
void CyclicForwardShift(float* spectrumRe, float* spectrumIm, int windowSize, float sampleFrequency, float shiftFrequency);

// Same as multiplying a function in time by e^(2*pi*i*(shiftFrequency/sampleFrequency))
void CyclicBackwardShift(float* spectrumRe, float* spectrumIm, int windowSize, float sampleFrequency, float shiftFrequency);

// Effectively shift frequencies forward by a constant frequency
void CyclicCosineForwardShift(float* spectrumRe, float* spectrumIm, int windowSize, float sampleFrequency, float shiftFrequency);

// Same as multiplying a function in time by cos(-2*pi*i*(shiftFrequency/sampleFrequency))
void CyclicCosineModulation(float* spectrumRe, float* spectrumIm, int windowSize, float sampleFrequency, float shiftFrequency);

// Multiplies a function in time by e^(-2*pi*i*(shiftFrequency/sampleFrequency))
void CyclicCosineModulationInTime(float* data, int windowSize, float sampleFrequency, float shiftFrequency);

// Does the same as CyclicCosineForwardShiftInTime, but uses a static vector
void CyclicCosineModulationInTime_static(float* data, int windowSize, float* vector);

// Initializes the vector used in CyclicCosineModulationInTime_static
void InitializeCosineVector(float* vector, int windowSize, float sampleFrequency, float shiftFrequency, float phaseShift);

// Initializes the vector used in CyclicCosineModulationInTime_static
void InitializeCosineVectorAutoShift(float* vector, int windowSize, float sampleFrequency, float shiftFrequency);

#endif // UTILS_H_INCLUDED
