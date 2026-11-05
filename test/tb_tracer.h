/*
 * tb_tracer.h
 *
 * Testbench utilities for FFT Abstract Model verification:
 *   1. GoldenFFT: Reference analytical FFT calculator using canonical DFT formula
 *      with standard C++ std::complex.
 *   2. DataTracer: CSV logger for input and output data points with accurate
 *      timestamps (in ns) for plotting and verification.
 */

#ifndef TB_TRACER_H
#define TB_TRACER_H

#include <systemc.h>
#include <fstream>
#include <string>
#include <vector>
#include <complex>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sys/stat.h>
#include "fft_types.h"

// Reference Golden FFT implementation using canonical DFT formula
class GoldenFFT {
private:
    static bool compare_impl(const std::vector<complex_t>& actual,
                             const std::vector<complex_t>& golden,
                             double tol,
                             bool round_golden,
                             std::string* err_msg) {
        if (actual.size() != golden.size()) {
            if (err_msg) {
                *err_msg = "Size mismatch: actual=" + std::to_string(actual.size()) +
                           ", golden=" + std::to_string(golden.size());
            }
            return false;
        }

        for (size_t i = 0; i < actual.size(); ++i) {
            double g_r = round_golden ? std::round(golden[i].real) : golden[i].real;
            double g_i = round_golden ? std::round(golden[i].imag) : golden[i].imag;
            double diff_r = std::abs(actual[i].real - g_r);
            double diff_i = std::abs(actual[i].imag - g_i);
            if (diff_r > tol || diff_i > tol) {
                if (err_msg) {
                    *err_msg = "Mismatch at bin " + std::to_string(i) +
                               ": actual=(" + std::to_string(actual[i].real) + ", " + std::to_string(actual[i].imag) +
                               "), golden=(" + std::to_string(g_r) + ", " + std::to_string(g_i) +
                               "), diff=(" + std::to_string(diff_r) + ", " + std::to_string(diff_i) +
                               "), tol=" + std::to_string(tol);
                }
                return false;
            }
        }
        return true;
    }

public:
    // Standard Canonical DFT formula using std::complex<double>
    static std::vector<complex_t> compute(const std::vector<complex_t>& in) {
        const int n = in.size();
        std::vector<complex_t> out(n);
        const double PI = std::acos(-1.0);

        for (int k = 0; k < n; ++k) {
            std::complex<double> sum(0.0, 0.0);
            for (int t = 0; t < n; ++t) {
                double angle = -2.0 * PI * k * t / n;
                std::complex<double> twiddle(std::cos(angle), std::sin(angle));
                std::complex<double> sample(in[t].real, in[t].imag);
                sum += sample * twiddle;
            }
            out[k] = complex_t(sum.real(), sum.imag());
        }
        return out;
    }

    // Direct floating point comparison (for streaming channels)
    static bool compare(const std::vector<complex_t>& actual,
                        const std::vector<complex_t>& golden,
                        double tol = 1e-4,
                        std::string* err_msg = nullptr) {
        return compare_impl(actual, golden, tol, false, err_msg);
    }

    // Rounded comparison (for integer-packed AXI words read from memory)
    static bool compare_rounded(const std::vector<complex_t>& actual,
                                const std::vector<complex_t>& golden,
                                double tol = 1e-3,
                                std::string* err_msg = nullptr) {
        return compare_impl(actual, golden, tol, true, err_msg);
    }
};

// CSV Logger for saving test stimulus and simulation results
class DataTracer {
private:
    std::string module_name;
    std::string out_dir;
    std::ofstream in_file;
    std::ofstream out_file;
    bool in_open;
    bool out_open;
    int in_sample_idx;
    int out_sample_idx;

    static void create_dir_if_not_exists(const std::string& dir) {
        struct stat st;
        if (stat(dir.c_str(), &st) != 0) {
            mkdir(dir.c_str(), 0777);
        }
    }

public:
    DataTracer(const std::string& name = "fft", const std::string& directory = "./out/data")
        : module_name(name),
          out_dir(directory),
          in_open(false),
          out_open(false),
          in_sample_idx(0),
          out_sample_idx(0)
    {
        create_dir_if_not_exists("./out");
        create_dir_if_not_exists(out_dir);

        std::string in_path = out_dir + "/" + module_name + "_input.csv";
        std::string out_path = out_dir + "/" + module_name + "_output.csv";

        in_file.open(in_path.c_str(), std::ios::out | std::ios::trunc);
        if (in_file.is_open()) {
            in_open = true;
            in_file << "sample_index,time_ns,real,imag,magnitude,phase_rad\n";
        }

        out_file.open(out_path.c_str(), std::ios::out | std::ios::trunc);
        if (out_file.is_open()) {
            out_open = true;
            out_file << "sample_index,time_ns,real,imag,magnitude,phase_rad\n";
        }
    }

    ~DataTracer() {
        close();
    }

    void close() {
        if (in_open) {
            in_file.close();
            in_open = false;
        }
        if (out_open) {
            out_file.close();
            out_open = false;
        }
    }

    // Helper to get time in accurate nanoseconds
    static double current_time_ns() {
        return sc_core::sc_time_stamp().to_seconds() * 1e9;
    }

    void log_input(const complex_t& val, double time_ns = -1.0) {
        if (!in_open) return;
        if (time_ns < 0.0) time_ns = current_time_ns();
        double mag = val.magnitude();
        double phase = std::atan2(val.imag, val.real);
        in_file << in_sample_idx++ << ","
                << std::fixed << std::setprecision(3) << time_ns << ","
                << std::setprecision(6) << val.real << ","
                << val.imag << ","
                << mag << ","
                << phase << "\n";
        in_file.flush();
    }

    void log_output(const complex_t& val, double time_ns = -1.0) {
        if (!out_open) return;
        if (time_ns < 0.0) time_ns = current_time_ns();
        double mag = val.magnitude();
        double phase = std::atan2(val.imag, val.real);
        out_file << out_sample_idx++ << ","
                << std::fixed << std::setprecision(3) << time_ns << ","
                << std::setprecision(6) << val.real << ","
                << val.imag << ","
                << mag << ","
                << phase << "\n";
        out_file.flush();
    }
};

#endif // TB_TRACER_H
