/*
 * fft_timed.h
 *
 * Abstract Timing-Centric FFT Module.
 *
 * Timing Parameters:
 *   - FIRST_SAMPLE_LATENCY (T1): Clock cycles of idle gap after the last input
 *     sample completes before the first output sample becomes valid.
 *   - INTER_SAMPLE_GAP     (T2): Clock cycles of idle gap between consecutive
 *     output samples (idle time between sample valid pulses).
 *
 * Timing Behavior:
 *   - Consumes N complex input samples sequentially via in_data (Connections channel).
 *   - Computes FFT in 0 simulation time using canonical standard C++ complex arithmetic.
 *   - Waits exactly T1 (FIRST_SAMPLE_LATENCY) idle clock cycles after the last input sample is received.
 *   - Streams the first output sample on out_data (Connections channel: val/rdy/msg).
 *   - Streams the remaining (N-1) output samples with exactly T2 (INTER_SAMPLE_GAP) idle clock
 *     cycles between consecutive output pulses.
 */

#ifndef FFT_TIMED_H
#define FFT_TIMED_H

#include <systemc.h>
#include <connections/connections.h>
#include <vector>
#include <complex>
#include <cmath>
#include "fft_types.h"

using namespace sc_core;
using namespace Connections;

template<int N_SIZE, int FIRST_SAMPLE_LATENCY = 8, int INTER_SAMPLE_GAP = 1>
SC_MODULE(FFTTimed) {
    // Clock and active-low asynchronous reset
    sc_in<bool> clk;
    sc_in<bool> rst_n;

    // Connections streaming interfaces
    In<complex_t> in_data;
    Out<complex_t> out_data;

    // Configurable timing parameters (T1, T2)
    int first_sample_latency;
    int inter_sample_gap;

    // Internal sample storage (module-level persistent hardware registers)
    std::vector<complex_t> in_buffer;
    std::vector<complex_t> out_buffer;

    // Telemetry counters
    unsigned long total_blocks_processed;
    unsigned long total_samples_in;
    unsigned long total_samples_out;

    SC_HAS_PROCESS(FFTTimed);

    FFTTimed(sc_module_name name,
             int t1 = FIRST_SAMPLE_LATENCY,
             int t2 = INTER_SAMPLE_GAP)
        : sc_module(name),
          clk("clk"),
          rst_n("rst_n"),
          in_data("in_data"),
          out_data("out_data"),
          first_sample_latency(t1),
          inter_sample_gap(t2),
          in_buffer(N_SIZE),
          out_buffer(N_SIZE),
          total_blocks_processed(0),
          total_samples_in(0),
          total_samples_out(0)
    {
        SC_THREAD(process_thread);
        sensitive << clk.pos();
        async_reset_signal_is(rst_n, false);
    }

    void set_timing_parameters(int t1, int t2) {
        first_sample_latency = (t1 >= 0) ? t1 : 0;
        inter_sample_gap = (t2 >= 0) ? t2 : 0;
    }

    // Canonical DFT formula using standard C++ complex arithmetic
    static void compute_fft(const std::vector<complex_t>& in, std::vector<complex_t>& out) {
        const int n = in.size();
        out.resize(n);
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
    }

private:
    void process_thread() {
        in_data.Reset();
        out_data.Reset();
        total_blocks_processed = 0;
        total_samples_in = 0;
        total_samples_out = 0;
        wait();

        while (true) {
            // Step 1: Collect N input samples
            for (int i = 0; i < N_SIZE; ++i) {
                in_buffer[i] = in_data.Pop();
                total_samples_in++;
            }

            // Step 2: Compute FFT in 0 simulation time
            compute_fft(in_buffer, out_buffer);

            // Step 3: Wait exactly T1 idle clock cycles AFTER the last input finishes
            for (int c = 0; c < first_sample_latency; ++c) {
                wait();
            }

            // Step 4: Stream first output sample (active for 1 clock cycle)
            out_data.Push(out_buffer[0]);
            total_samples_out++;
            wait();

            // Step 5: Stream remaining samples with exactly T2 idle cycles between them
            for (int i = 1; i < N_SIZE; ++i) {
                for (int c = 0; c < inter_sample_gap; ++c) {
                    wait();
                }
                out_data.Push(out_buffer[i]);
                total_samples_out++;
                wait();
            }

            total_blocks_processed++;
        }
    }
};

#endif // FFT_TIMED_H
