/*
 * fft_types.h
 *
 * Complex data type and AXI4 streaming conversion utilities for FFT Abstract Model.
 */

#ifndef FFT_TYPES_H
#define FFT_TYPES_H

#include <systemc.h>
#include <complex>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <auto_gen_fields.h>

using namespace sc_core;

// Complex number structure compatible with SystemC tracing and MatchLib serialization
struct complex_t {
    double real;
    double imag;

    complex_t(double r = 0.0, double i = 0.0) : real(r), imag(i) {}

    // Basic arithmetic operators
    complex_t operator+(const complex_t& o) const {
        return complex_t(real + o.real, imag + o.imag);
    }

    complex_t operator-(const complex_t& o) const {
        return complex_t(real - o.real, imag - o.imag);
    }

    complex_t operator*(const complex_t& o) const {
        return complex_t(
            real * o.real - imag * o.imag,
            real * o.imag + imag * o.real
        );
    }

    double magnitude() const {
        return std::sqrt(real * real + imag * imag);
    }

    // Auto-generation macro for field tracing and Matchlib marshaling
    AUTO_GEN_FIELD_METHODS_V2(complex_t, (real, imag))
};

// Stream input operator
inline std::istream& operator>>(std::istream& is, complex_t& c) {
    is >> c.real >> c.imag;
    return is;
}

// Convert complex numbers into packed AXI bus words
template<typename AxiCfg>
inline sc_uint<AxiCfg::dataWidth> pack_complex(double r, double i) {
    sc_uint<AxiCfg::dataWidth> res = 0;
    if (AxiCfg::dataWidth == 64) {
        static const int HALF_WIDTH = AxiCfg::dataWidth / 2;
        int r_int = static_cast<int>(std::round(r));
        int i_int = static_cast<int>(std::round(i));
        res.range(AxiCfg::dataWidth - 1, HALF_WIDTH) = r_int;
        res.range(HALF_WIDTH - 1, 0) = i_int;
    } else {
        int r_int = static_cast<int>(std::round(r));
        res.range(AxiCfg::dataWidth - 1, 0) = r_int;
    }
    return res;
}

template<typename AxiCfg>
inline sc_uint<AxiCfg::dataWidth> pack_complex(const complex_t& val) {
    return pack_complex<AxiCfg>(val.real, val.imag);
}

// Unpack AXI bus words into complex numbers
template<typename AxiCfg>
inline complex_t unpack_complex(sc_uint<AxiCfg::dataWidth> raw) {
    static const int HALF_WIDTH = AxiCfg::dataWidth / 2;
    if (AxiCfg::dataWidth == 64) {
        int r_int = raw.range(AxiCfg::dataWidth - 1, HALF_WIDTH).to_int();
        int i_int = raw.range(HALF_WIDTH - 1, 0).to_int();
        return complex_t(static_cast<double>(r_int), static_cast<double>(i_int));
    } else {
        int r_int = raw.range(AxiCfg::dataWidth - 1, 0).to_int();
        return complex_t(static_cast<double>(r_int), 0.0);
    }
}

#endif // FFT_TYPES_H
