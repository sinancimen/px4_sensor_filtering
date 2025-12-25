// Taken from https://thecodehound.com/butterworth-filter-design-in-c/
#include "ButterworthSynth.hpp"
#include <cmath>
#include <cassert>


struct Cplx {
    double re{0.0}, im{0.0};
    Cplx() = default;
    Cplx(const Cplx& o) = default; // avoid deprecated implicit copy constructor
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

struct Poly {
	Cplx c[BUTTERWORTH_MAX_COEFFS];
	size_t size;
	Poly() {
		for (size_t i = 0; i < BUTTERWORTH_MAX_COEFFS; ++i) c[i] = Cplx(0.0);
		c[0] = Cplx(1.0);
		size = 1;
	}
};

static Poly multiply(const Poly& p, const Poly& q)
{
	Poly result;
	for (size_t i = 0; i < BUTTERWORTH_MAX_COEFFS; ++i) result.c[i] = Cplx(0.0);
	result.size = p.size + q.size - 1;
	assert(result.size <= BUTTERWORTH_MAX_COEFFS);
	for (size_t i = 0; i < p.size; i++)
		for (size_t j = 0; j < q.size; j++)
			result.c[i + j] += p.c[i] * q.c[j];
	return result;
}

// Emulate Matlab 'poly' function.  Calculate the coefficients of the polynomial
// with the specified roots.
static Poly poly(const Cplx roots[], size_t N)
{
	Poly result; // initialized to 1
	for (size_t i = 0; i < N; ++i)
	{
		Poly factor;
		for (size_t k = 0; k < BUTTERWORTH_MAX_COEFFS; ++k) factor.c[k] = Cplx(0.0);
		factor.c[0] = -roots[i];
		factor.c[1] = Cplx(1.0);
		factor.size = 2;
		result = multiply(result, factor);
	}

	// Keep coefficients in lowest-order-first (constant term first) to make convolution easy.
	// The caller will store them as highest-order-first without performing an explicit swap here.
	return result;
}

// Emulate Matlab 'sum' function over the first 'size' coefficients.
static Cplx sum(const Poly& p)
{
	Cplx s = Cplx(0.0);
	for (size_t i = 0; i < p.size; ++i) s += p.c[i];
	return s;
}

// Taken from here : https://www.dsprelated.com/showarticle/1119.php
// The blog post gives Matlab source code which I have converted to C++ below:
IIR_Coeffs butter_synth(int N, double fc, double fs)
{
	assert(N >= 1 && N <= BUTTERWORTH_MAX_ORDER);
	IIR_Coeffs coeffs; // zero-initialized

	Cplx pa[BUTTERWORTH_MAX_ORDER];
	Cplx p[BUTTERWORTH_MAX_ORDER];
	Cplx q[BUTTERWORTH_MAX_ORDER];
	for (size_t i = 0; i < BUTTERWORTH_MAX_ORDER; ++i) q[i] = Cplx(-1.0);

	// I. Find poles of analog filter
	for (int i = 0; i < N; i++)
	{
		int k = i + 1;
		double theta = (2 * k - 1) * M_PI / (2 * N);
		pa[i] = Cplx(-sin(theta), cos(theta));
	}

	// II. Scale poles in frequency
	double Fc = fs / M_PI * tan(M_PI * fc / fs);
	for (size_t i = 0; i < static_cast<size_t>(N); i++)
		pa[i] *= 2 * M_PI * Fc;

	// III. Find coeffs of digital filter poles and zeros in the z plane
	for (size_t i = 0; i < static_cast<size_t>(N); i++)
		p[i] = (Cplx(1.0) + pa[i] / (2 * fs)) / (Cplx(1.0) - pa[i] / (2 * fs));

	auto a_poly = poly(p, static_cast<size_t>(N));
	// Store coefficients highest-order-first: a[0] is coef for z^N, a[N] is constant term
	for (size_t i = 0; i < a_poly.size; ++i)
		coeffs.a[i] = a_poly.c[a_poly.size - 1 - i].real();

	auto b_poly = poly(q, static_cast<size_t>(N));
	Cplx K = sum(a_poly) / sum(b_poly);
	for (size_t i = 0; i < b_poly.size; ++i)
		coeffs.b[i] = (b_poly.c[b_poly.size - 1 - i] * K).real();

	coeffs.order = N;
	return coeffs;
}
