// Taken from https://thecodehound.com/butterworth-filter-design-in-c/
#include "ButterworthSynth.h"
#include <complex>
#include <cmath>
#include <numeric>
#include <assert.h>

using namespace std::literals::complex_literals;

using Polynomial = std::vector<std::complex<double>>;

Polynomial operator*(const Polynomial& p, const Polynomial& q)
{
	size_t n = p.size() + q.size() - 1;
	Polynomial result(n);
	for (size_t i = 0; i < p.size(); i++)
		for (size_t j = 0; j < q.size(); j++)
			result[i + j] += p[i] * q[j];
	return result;
}

// Emulate Matlab 'poly' function.  Calculate the coefficients of the polynomial 
// with the specified roots.
static std::vector<std::complex<double>> poly(std::vector<std::complex<double>> roots)
{
	Polynomial result{1.0};
	for (auto root : roots)
	{
		Polynomial factor({-root, 1.0});
		result = result * factor;
	}

	// Matlab returns the highest order coefficients first.
	std::reverse(result.begin(), result.end());
	return result;
}

// Emulate Matlab 'sum' function.  
static std::complex<double> sum(const std::vector<std::complex<double>>& v)
{
	return std::accumulate(v.begin(), v.end(), 0.0i);
}

// Taken from here : https://www.dsprelated.com/showarticle/1119.php
// The blog post gives Matlab source code which I have converted to C++ below:
IIR_Coeffs butter_synth(int N, double fc, double fs)
{
	IIR_Coeffs coeffs;
	std::vector<std::complex<double>> pa(N);
	std::vector<std::complex<double>> p(N);
	std::vector<std::complex<double>> q(N, -1.0);
	assert(fc < fs / 2); // Cutoff frequency must be less that fs/2

	// I. Find poles of analog filter
	for (int i = 0; i < N; i++)
	{
		int k = i + 1;
		double theta = (2 * k - 1) * M_PI / (2 * N);
		pa[i] = -sin(theta) + 1.0i * cos(theta);
	}

	// II. Scale poles in frequency
	double Fc = fs / M_PI * tan(M_PI * fc / fs);
	for (size_t i=0; i<pa.size(); i++)
		pa[i] *= 2 * M_PI * Fc;

	// III. Find coeffs of digital filter poles and zeros in the z plane
	for (size_t i = 0; i < N; i++)
		p[i] = (1.0 + pa[i] / (2 * fs)) / (1.0 - pa[i] / (2 * fs));

	auto a = poly(p);
	for (size_t i = 0; i < a.size(); i++)
		a[i] = a[i].real();

	auto b = poly(q);
	auto K = sum(a) / sum(b);
	for (size_t i = 0; i < b.size(); i++)
		b[i] *= K;

	for (auto coeff : a)
		coeffs.a.push_back(coeff.real());
	for (auto coeff : b)
		coeffs.b.push_back(coeff.real());
	return coeffs;
}
