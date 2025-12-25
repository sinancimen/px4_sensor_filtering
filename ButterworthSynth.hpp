// Written by Sinan Cimen, 2025. https://github.com/sinancimen
// Modified version of https://thecodehound.com/butterworth-filter-design-in-c/

#pragma once

#include <cassert>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr int BUTTERWORTH_MAX_ORDER = 10; // Maximum order of Butterworth filter supported
constexpr int BUTTERWORTH_MAX_COEFFS = BUTTERWORTH_MAX_ORDER + 1;

class IIR_Coeffs // Struct to store IIR filter coefficients
{
public:
	double a[BUTTERWORTH_MAX_COEFFS] = {};
	double b[BUTTERWORTH_MAX_COEFFS] = {}; // filter will have N+1 coefficients
	int order = 0; // Filter order between 1 and BUTTERWORTH_MAX_ORDER
};

IIR_Coeffs butter_synth(int N, double fc, double fs); // Order N, cutoff freq fc, sampling freq fs
