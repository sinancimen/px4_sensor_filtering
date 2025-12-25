// Taken from https://thecodehound.com/butterworth-filter-design-in-c/
#pragma once

#include <cassert>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

constexpr int BUTTERWORTH_MAX_ORDER = 10;
constexpr int BUTTERWORTH_MAX_COEFFS = BUTTERWORTH_MAX_ORDER + 1;

class IIR_Coeffs
{
public:
	double a[BUTTERWORTH_MAX_COEFFS] = {};
	double b[BUTTERWORTH_MAX_COEFFS] = {};
	int order = 0; // valid values: 1..BUTTERWORTH_MAX_ORDER
};

IIR_Coeffs butter_synth(int N, double fc, double fs);
