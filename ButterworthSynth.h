// Taken from https://thecodehound.com/butterworth-filter-design-in-c/
#pragma once

#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class IIR_Coeffs
{
public:
	std::vector<double> a;
	std::vector<double> b;
};

IIR_Coeffs butter_synth(int N, double fc, double fs);