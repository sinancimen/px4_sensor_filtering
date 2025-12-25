// Written by Sinan Cimen, 2025. https://github.com/sinancimen

#pragma once
#include "ButterworthSynth.hpp"

class ButterworthIIR {
public:
    ButterworthIIR() = default;

    explicit ButterworthIIR(const IIR_Coeffs& c) {
        setCoeffs(c);
    }

    void setCoeffs(const IIR_Coeffs& c) {
        assert(c.order >= 1 && c.order <= BUTTERWORTH_MAX_ORDER);
        order_ = static_cast<std::size_t>(c.order);

        for (std::size_t i = 0; i <= order_; ++i) {
            a_[i] = c.a[i];
            b_[i] = c.b[i];
            xHist_[i] = 0.0;
            yHist_[i] = 0.0;
        }

        for (std::size_t i = order_ + 1; i < BUTTERWORTH_MAX_COEFFS; ++i) {
            a_[i] = 0.0;
            b_[i] = 0.0;
            xHist_[i] = 0.0;
            yHist_[i] = 0.0;
        }
    }

    void reset(double value = 0.0) {
        if (order_ == 0) return;
        for (std::size_t i = 0; i <= order_; ++i) {
            xHist_[i] = value;
            yHist_[i] = value;
        }
    }

    double process(double x) {
        if (order_ == 0) return x;
        // shift history (xHist_[0] is newest)
        for (std::size_t i = order_; i > 0; --i) xHist_[i] = xHist_[i - 1];
        xHist_[0] = x;

        double y = 0.0;
        // feedforward
        // coefficients are stored highest-order-first: b_[0] corresponds to x[n-order]
        for (std::size_t k = 0; k <= order_; ++k)
            y += b_[k] * xHist_[order_ - k];

        // feedback (skip a[0] which is the leading coefficient)
        // a_[k] corresponds to coefficient for y[n - (order_ - k)] so map to yHist_[order_ - k]
        for (std::size_t k = 1; k <= order_; ++k)
            y -= a_[k] * yHist_[order_ - k];
        // shift y history
        for (std::size_t i = order_; i > 0; --i) yHist_[i] = yHist_[i - 1];
        yHist_[0] = y;

        return y;
    }

    void processBuffer(const double* in, double* out, std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) out[i] = process(in[i]);
    }

private:
    double a_[BUTTERWORTH_MAX_COEFFS] = {};
    double b_[BUTTERWORTH_MAX_COEFFS] = {};
    double xHist_[BUTTERWORTH_MAX_COEFFS] = {};
    double yHist_[BUTTERWORTH_MAX_COEFFS] = {};
    std::size_t order_ = 0;
};
