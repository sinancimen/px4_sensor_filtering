// Written by Sinan Cimen, 2025. https://github.com/sinancimen
// Modified version of https://thecodehound.com/butterworth-filter-design-in-c/

#include "ButterworthSynth.hpp"
#include <cmath>
#include <cassert>


struct Cplx { // Struct to imiatate std::complex
    double re{0.0}, im{0.0};
    Cplx() = default;
    Cplx(const Cplx& o) = default;
    Cplx(double r, double i=0.0): re(r), im(i) {}
    Cplx operator+(const Cplx& o) const { return {re + o.re, im + o.im}; }
    Cplx operator+(const double& o) const { return {re + o, im}; }
    Cplx& operator+=(const Cplx& o) { re += o.re; im += o.im; return *this; }
    Cplx operator-(const Cplx& o) const { return {re - o.re, im - o.im}; }
    Cplx& operator=(const Cplx& o) { re = o.re; im = o.im; return *this; }
    Cplx& operator=(double r) { re = r; im = 0.0; return *this; }
    Cplx operator-() const { return {-re, -im}; }
    Cplx operator*(const Cplx& o) const { return {re*o.re - im*o.im, re*o.im + im*o.re}; }
    Cplx operator*=(const double& r) { re *= r; im *= r; return *this; }
    Cplx operator/(const Cplx& o) const {
        double d = o.re*o.re + o.im*o.im;
        return {(re*o.re + im*o.im)/d, (im*o.re - re*o.im)/d};
    }
    Cplx conj() const { return {re, -im}; }
    double real() const { return re; }
};

struct Poly { // Struct to represent a polynomial
	Cplx c[BUTTERWORTH_MAX_COEFFS]; // Coefficients in lowest-order-first form
	size_t size; // Number of coefficients
	Poly() {
		for (size_t i = 0; i < BUTTERWORTH_MAX_COEFFS; ++i) c[i] = Cplx(0.0);
		c[0] = Cplx(1.0);
		size = 1;
	}
};

static Poly multiply(const Poly& p, const Poly& q) // Multiply two polynomials
{
	Poly result;
	result.size = p.size + q.size - 1;
	assert(result.size <= BUTTERWORTH_MAX_COEFFS);
	for (size_t i = 0; i < p.size; i++)
		for (size_t j = 0; j < q.size; j++)
			result.c[i + j] += p.c[i] * q.c[j];
	return result;
}

// Calculate the coefficients of the polynomial with the specified roots.
static Poly poly(const Cplx roots[], size_t N)
{
	Poly result;
	for (size_t i = 0; i < N; ++i)
	{
		Poly factor;
		factor.c[0] = -roots[i];
		factor.c[1] = Cplx(1.0);
		factor.size = 2;
		result = multiply(result, factor); // multiply each root equation into result to have the full polynomial
	}
	return result;
}

static Cplx sum(const Poly& p) // Sum of polynomial coefficients
{
	Cplx s = Cplx(0.0);
	for (size_t i = 0; i < p.size; ++i) s += p.c[i];
	return s;
}

// Taken from: https://www.dsprelated.com/showarticle/1119.php
IIR_Coeffs butter_synth(int N, double fc, double fs)
{
	assert(N >= 1 && N <= BUTTERWORTH_MAX_ORDER);
	IIR_Coeffs coeffs;

	Cplx pa[BUTTERWORTH_MAX_ORDER]; // Analog filter poles. Analog Butterworth filter has no zeros.
	Cplx p[BUTTERWORTH_MAX_ORDER]; // Digital filter poles.
	Cplx q[BUTTERWORTH_MAX_ORDER]; // Digital filter zeros.
	for (size_t i = 0; i < BUTTERWORTH_MAX_ORDER; ++i) q[i] = Cplx(-1.0); // All zeros at z = -1

	// I. Find poles of analog filter for normalized cutoff frequency of 1 rad/s
	for (int i = 0; i < N; i++)
	{
		int k = i + 1;
		double theta = (2 * k - 1) * M_PI / (2 * N);
		pa[i] = Cplx(-sin(theta), cos(theta));
	}

	// II. Scale poles in frequency
	double Fc = fs / M_PI * tan(M_PI * fc / fs); // Fc is the analog cutoff frequency for a given digital cutoff frequency fc and sampling frequency fs	
	for (size_t i = 0; i < static_cast<size_t>(N); i++)
		pa[i] *= 2 * M_PI * Fc; // Scaled analog filter poles

	// III. Find coeffs of digital filter poles and zeros in the z plane using bilinear transform
	for (size_t i = 0; i < static_cast<size_t>(N); i++)
		p[i] = (Cplx(1.0) + pa[i] / (2 * fs)) / (Cplx(1.0) - pa[i] / (2 * fs));

	auto a_poly = poly(p, static_cast<size_t>(N)); // Generate polynomial from poles p, in lowest-order-first form
	// Store coefficients highest-order-first: a[0] is coef for z^N, a[N] is constant term
	for (size_t i = 0; i < a_poly.size; ++i)
		coeffs.a[i] = a_poly.c[a_poly.size - 1 - i].real();

	auto b_poly = poly(q, static_cast<size_t>(N)); // Numerator polynomial from zeros q, which are all -1, in lowest-order-first form
	Cplx K = sum(a_poly) / sum(b_poly);
	for (size_t i = 0; i < b_poly.size; ++i)
		coeffs.b[i] = (b_poly.c[b_poly.size - 1 - i] * K).real();

	coeffs.order = N;
	return coeffs;
}
