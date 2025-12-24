#pragma once
#include "ButterworthSynth.h"
#include <vector>
#include <stdexcept>
#include <algorithm>

class ButterworthIIR {
public:
    ButterworthIIR() = default;

    explicit ButterworthIIR(const IIR_Coeffs& c) {
        setCoeffs(c);
    }

    void setCoeffs(const IIR_Coeffs& c) {
        if (c.a.empty() || c.b.empty() || c.a.size() != c.b.size())
            throw std::invalid_argument("a and b must be non-empty and same length");
        if (std::abs(c.a[0] - 1.0) > 1e-9)
            throw std::invalid_argument("Expected a[0] == 1.0");

        a_ = c.a;
        b_ = c.b;
        const std::size_t N = a_.size() - 1;

        xHist_.assign(N + 1, 0.0);
        yHist_.assign(N + 1, 0.0);
    }

    void reset(double value = 0.0) {
        std::fill(xHist_.begin(), xHist_.end(), value);
        std::fill(yHist_.begin(), yHist_.end(), value);
    }

    double process(double x) {
        // shift history (xHist_[0] is newest)
        for (std::size_t i = xHist_.size() - 1; i > 0; --i) xHist_[i] = xHist_[i - 1];
        xHist_[0] = x;

        double y = 0.0;
        // feedforward
        for (std::size_t k = 0; k < b_.size(); ++k)
            y += b_[k] * xHist_[k];

        // feedback (skip a[0])
        for (std::size_t k = 1; k < a_.size(); ++k)
            y -= a_[k] * yHist_[k];

        // shift y history
        for (std::size_t i = yHist_.size() - 1; i > 0; --i) yHist_[i] = yHist_[i - 1];
        yHist_[0] = y;

        return y;
    }

    void processBuffer(const double* in, double* out, std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) out[i] = process(in[i]);
    }

private:
    std::vector<double> a_, b_;
    std::vector<double> xHist_, yHist_;
};