/*
 * tb_fft.cpp
 *
 * Comprehensive unit testbench for FFTTimed module.
 *
 * Verification Features:
 *   - The DUT drives its own hardware out_valid port (1 cycle per sample, exact T1/T2 idle gaps).
 *   - Exact cycle timing:
 *       T1: Exact idle gap in clock cycles after last input completes before first output begins.
 *       T2: Exact idle gap in clock cycles between consecutive output pulses (idle gap between values).
 *   - 25-cycle buffer gaps between test scenarios to verify idle channel behavior.
 *   - Rigorous mathematical verification against canonical Golden FFT across all scenarios.
 */

#define CONNECTIONS_NAMING_ORIGINAL
#define BOOST_NULLPTR nullptr

#include <systemc.h>
#include <connections/connections.h>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include <iomanip>
#include "fft_timed.h"
#include "tb_tracer.h"

using namespace sc_core;
using namespace Connections;

SC_MODULE(TbFFT) {
    sc_clock clk;
    sc_signal<bool> rst_n;

    Combinational<complex_t> in_chan;
    Combinational<complex_t> out_chan;

    enum {
        N = 8,
        T1_VAL = 10,
        T2_VAL = 3
    };

    // Hardware out_valid port driven by DUT
    sc_signal<bool> in_valid;
    sc_signal<bool> out_valid;

    FFTTimed<N, T1_VAL, T2_VAL> dut;

    sc_signal<int> cycle_count;
    int test_failures;

    // VCD waveform tracing signals
    sc_trace_file* tf;
    sc_signal<double> trace_in_real;
    sc_signal<double> trace_in_imag;
    sc_signal<double> trace_out_real;
    sc_signal<double> trace_out_imag;
    sc_signal<double> trace_out_magnitude;

    // CSV Data Logger
    DataTracer tracer;

    // Synchronization events
    sc_event start_producer_event;
    sc_event consumer_finished_event;

    std::vector<complex_t> test_stimulus;
    std::vector<complex_t> captured_output;
    std::vector<int> captured_output_cycles;
    int last_input_end_cycle;

    int samples_to_consume;
    int consumer_initial_stall;

    SC_CTOR(TbFFT)
        : clk("clk", 1, SC_NS),
          rst_n("rst_n"),
          in_chan("in_chan"),
          out_chan("out_chan"),
          in_valid("in_valid"),
          out_valid("out_valid"),
          dut("dut"),
          cycle_count("cycle_count"),
          test_failures(0),
          tf(nullptr),
          trace_in_real("trace_in_real"),
          trace_in_imag("trace_in_imag"),
          trace_out_real("trace_out_real"),
          trace_out_imag("trace_out_imag"),
          trace_out_magnitude("trace_out_magnitude"),
          tracer("fft_unit", "./out/data"),
          last_input_end_cycle(0),
          samples_to_consume(0),
          consumer_initial_stall(0)
    {
        dut.clk(clk);
        dut.rst_n(rst_n);
        dut.in_data(in_chan);
        dut.out_data(out_chan);
        dut.out_valid(out_valid);

        // VCD waveform tracing setup
        tf = sc_create_vcd_trace_file("./out/vcd/fft_unit");
        if (tf) {
            sc_trace(tf, clk, "ctrl.clk");
            sc_trace(tf, rst_n, "ctrl.rst_n");
            sc_trace(tf, cycle_count, "ctrl.cycle_count");

            sc_trace(tf, in_valid, "input.valid");
            sc_trace(tf, trace_in_real, "input.real");
            sc_trace(tf, trace_in_imag, "input.imag");

            sc_trace(tf, out_valid, "output.valid");
            sc_trace(tf, trace_out_real, "output.real");
            sc_trace(tf, trace_out_imag, "output.imag");
            sc_trace(tf, trace_out_magnitude, "output.magnitude");
        }

        SC_METHOD(cycle_tracker);
        sensitive << clk.posedge_event();

        SC_THREAD(producer_thread);
        sensitive << clk.posedge_event();

        SC_THREAD(consumer_thread);
        sensitive << clk.posedge_event();

        SC_THREAD(test_orchestrator);
        sensitive << clk.posedge_event();
    }

    ~TbFFT() {
        if (tf) {
            sc_close_vcd_trace_file(tf);
        }
    }

    void cycle_tracker() {
        if (!rst_n.read()) {
            cycle_count.write(0);
        } else {
            cycle_count.write(cycle_count.read() + 1);
        }
    }

    void reset_dut() {
        rst_n.write(false);
        wait(5);
        rst_n.write(true);
        wait(2);
    }

    // Producer process: pushes 1 sample per clock cycle with accurate 1-cycle in_valid pulse
    void producer_thread() {
        in_chan.ResetWrite();
        in_valid.write(false);
        trace_in_real.write(0.0);
        trace_in_imag.write(0.0);
        wait();

        while (true) {
            wait(start_producer_event);
            for (size_t i = 0; i < test_stimulus.size(); ++i) {
                const complex_t& val = test_stimulus[i];
                trace_in_real.write(val.real);
                trace_in_imag.write(val.imag);
                tracer.log_input(val);

                in_valid.write(true);
                in_chan.Push(val);
                wait();
                in_valid.write(false);
            }
            last_input_end_cycle = cycle_count.read();
            trace_in_real.write(0.0);
            trace_in_imag.write(0.0);
        }
    }

    // Consumer process: pops output samples (out_valid is driven by the DUT)
    void consumer_thread() {
        out_chan.ResetRead();
        trace_out_real.write(0.0);
        trace_out_imag.write(0.0);
        trace_out_magnitude.write(0.0);
        wait();

        while (true) {
            while (samples_to_consume <= 0) {
                wait();
            }

            if (consumer_initial_stall > 0) {
                wait(consumer_initial_stall);
                consumer_initial_stall = 0;
            }

            captured_output.clear();
            captured_output_cycles.clear();

            while (samples_to_consume > 0) {
                complex_t sample = out_chan.Pop();

                captured_output.push_back(sample);
                captured_output_cycles.push_back(cycle_count.read());

                trace_out_real.write(sample.real);
                trace_out_imag.write(sample.imag);
                trace_out_magnitude.write(sample.magnitude());
                tracer.log_output(sample);

                samples_to_consume--;
            }

            trace_out_real.write(0.0);
            trace_out_imag.write(0.0);
            trace_out_magnitude.write(0.0);
            consumer_finished_event.notify();
        }
    }

    void run_concurrent_transfer(const std::vector<complex_t>& stimulus, int initial_stall = 0) {
        test_stimulus = stimulus;
        samples_to_consume = stimulus.size();
        consumer_initial_stall = initial_stall;
        start_producer_event.notify();
        wait(consumer_finished_event);
    }

    // Verify output channel remains completely idle (no spurious out_valid pulses) during buffer gap
    void verify_idle_channel(int duration_cycles, const char* context) {
        for (int c = 0; c < duration_cycles; ++c) {
            wait();
            if (out_valid.read()) {
                std::cerr << "  FAIL: Spurious out_valid detected during buffer gap after "
                          << context << " at cycle " << cycle_count.read() << std::endl;
                test_failures++;
                return;
            }
        }
    }

    void test_orchestrator() {
        std::cout << "\n========================================================" << std::endl;
        std::cout << "  RUNNING UNIT TESTS FOR FFT Timed MODULE (N=" << N << ")" << std::endl;
        std::cout << "  (DUT hardware out_valid port, exact T1/T2 idle gaps)" << std::endl;
        std::cout << "========================================================" << std::endl;

        reset_dut();
        wait(25); // Initial buffer gap

        test_scenario_1_timing_accuracy();
        verify_idle_channel(25, "Scenario 1"); // 25-cycle buffer gap

        test_scenario_2_zero_gap_streaming();
        verify_idle_channel(25, "Scenario 2"); // 25-cycle buffer gap

        test_scenario_3_zero_latency();
        verify_idle_channel(25, "Scenario 3"); // 25-cycle buffer gap

        test_scenario_4_multiframe_streaming();
        verify_idle_channel(25, "Scenario 4"); // 25-cycle buffer gap

        test_scenario_5_backpressure();
        verify_idle_channel(25, "Scenario 5"); // 25-cycle buffer gap

        test_scenario_6_math_correctness();
        verify_idle_channel(25, "Scenario 6"); // 25-cycle buffer gap

        std::cout << "\n--------------------------------------------------------" << std::endl;
        if (test_failures == 0) {
            std::cout << "  >>> ALL FFTTimed SCENARIOS PASSED SUCCESSFULLY! <<<" << std::endl;
        } else {
            std::cout << "  >>> FFTTimed COMPLETED WITH " << test_failures << " FAILURES! <<<" << std::endl;
        }
        std::cout << "--------------------------------------------------------\n" << std::endl;

        sc_stop();
    }

    // SCENARIO 1: Exact cycle timing (T1 and T2)
    void test_scenario_1_timing_accuracy() {
        std::cout << "\n[Test 1] Timing Accuracy (T1=" << T1_VAL << ", T2=" << T2_VAL << ") + Value Check..." << std::endl;
        dut.set_timing_parameters(T1_VAL, T2_VAL);

        std::vector<complex_t> stimulus(N);
        for (int i = 0; i < N; ++i) stimulus[i] = complex_t(i + 1, (i + 1) * 0.5);

        std::vector<complex_t> golden = GoldenFFT::compute(stimulus);

        run_concurrent_transfer(stimulus);

        // 1. T1: Idle gap AFTER last input finishes before first output starts
        int first_output_start_cycle = captured_output_cycles[0] - 1;
        int idle_t1 = first_output_start_cycle - last_input_end_cycle;
        std::cout << "  Last input ended at cycle " << last_input_end_cycle
                  << ", First output started at cycle " << first_output_start_cycle
                  << " (Idle gap T1 = " << idle_t1 << " cycles, expected = " << T1_VAL << ")" << std::endl;

        if (idle_t1 != T1_VAL) {
            std::cerr << "  FAIL: Idle gap T1=" << idle_t1 << " != expected " << T1_VAL << std::endl;
            test_failures++;
        }

        // 2. T2: Idle gap BETWEEN consecutive output values (not including the values)
        bool t2_ok = true;
        for (int i = 1; i < N; ++i) {
            int idle_t2 = captured_output_cycles[i] - captured_output_cycles[i - 1] - 1;
            if (idle_t2 != T2_VAL) {
                std::cerr << "  FAIL: Gap between sample " << (i - 1) << " and " << i
                          << " was " << idle_t2 << " idle cycles, expected T2=" << T2_VAL << std::endl;
                t2_ok = false;
                test_failures++;
            }
        }

        // 3. Mathematical correctness
        std::string err_msg;
        bool values_ok = GoldenFFT::compare(captured_output, golden, 1e-4, &err_msg);
        if (!values_ok) {
            std::cerr << "  FAIL: Value mismatch: " << err_msg << std::endl;
            test_failures++;
        }

        if (idle_t1 == T1_VAL && t2_ok && values_ok) {
            std::cout << "  PASS: Exact T1=" << T1_VAL << " idle cycles after input, T2=" << T2_VAL
                      << " idle cycles between outputs, and all " << N << " values verified!" << std::endl;
        }
    }

    // SCENARIO 2: Zero inter-sample gap streaming (T2 = 0)
    void test_scenario_2_zero_gap_streaming() {
        std::cout << "\n[Test 2] Zero Gap Streaming (T1=4, T2=0) + Value Check..." << std::endl;
        dut.set_timing_parameters(4, 0);

        std::vector<complex_t> stimulus(N);
        for (int i = 0; i < N; ++i) stimulus[i] = complex_t(std::cos(i), std::sin(i));

        std::vector<complex_t> golden = GoldenFFT::compute(stimulus);

        run_concurrent_transfer(stimulus);

        int first_output_start_cycle = captured_output_cycles[0] - 1;
        int idle_t1 = first_output_start_cycle - last_input_end_cycle;
        if (idle_t1 != 4) {
            std::cerr << "  FAIL: T1=" << idle_t1 << ", expected 4" << std::endl;
            test_failures++;
        }

        bool contiguous = true;
        for (int i = 1; i < N; ++i) {
            int idle_t2 = captured_output_cycles[i] - captured_output_cycles[i - 1] - 1;
            if (idle_t2 != 0) {
                std::cerr << "  FAIL: Output " << i << " idle gap=" << idle_t2 << ", expected 0" << std::endl;
                contiguous = false;
                test_failures++;
            }
        }

        std::string err_msg;
        bool values_ok = GoldenFFT::compare(captured_output, golden, 1e-4, &err_msg);
        if (!values_ok) {
            std::cerr << "  FAIL: Value mismatch: " << err_msg << std::endl;
            test_failures++;
        }

        if (contiguous && values_ok && idle_t1 == 4) {
            std::cout << "  PASS: Continuous streaming (0 idle gap between outputs) and values verified!" << std::endl;
        }
    }

    // SCENARIO 3: Zero initial latency (T1 = 0)
    void test_scenario_3_zero_latency() {
        std::cout << "\n[Test 3] Zero Initial Latency (T1=0, T2=2) + Value Check..." << std::endl;
        dut.set_timing_parameters(0, 2);

        std::vector<complex_t> stimulus(N, complex_t(2.5, -1.5));
        std::vector<complex_t> golden = GoldenFFT::compute(stimulus);

        run_concurrent_transfer(stimulus);

        int first_output_start_cycle = captured_output_cycles[0] - 1;
        int idle_t1 = first_output_start_cycle - last_input_end_cycle;
        if (idle_t1 != 0) {
            std::cerr << "  FAIL: T1=" << idle_t1 << ", expected 0" << std::endl;
            test_failures++;
        }

        std::string err_msg;
        bool values_ok = GoldenFFT::compare(captured_output, golden, 1e-4, &err_msg);
        if (!values_ok) {
            std::cerr << "  FAIL: Value mismatch: " << err_msg << std::endl;
            test_failures++;
        }

        if (idle_t1 == 0 && values_ok) {
            std::cout << "  PASS: T1=0 verified with accurate Golden FFT values!" << std::endl;
        }
    }

    // SCENARIO 4: Multi-frame consecutive streaming (3 frames = 24 samples)
    void test_scenario_4_multiframe_streaming() {
        std::cout << "\n[Test 4] Multi-frame Streaming (3 frames = 24 samples) + Value Check..." << std::endl;
        dut.set_timing_parameters(5, 1);

        const int num_frames = 3;
        std::vector<complex_t> stimulus(num_frames * N);
        for (int f = 0; f < num_frames; ++f) {
            for (int i = 0; i < N; ++i) {
                stimulus[f * N + i] = complex_t((f + 1) * (i + 1), (i + 1) * 2.0);
            }
        }

        run_concurrent_transfer(stimulus);

        bool all_frames_ok = true;
        for (int f = 0; f < num_frames; ++f) {
            std::vector<complex_t> frame_in(stimulus.begin() + f * N, stimulus.begin() + (f + 1) * N);
            std::vector<complex_t> frame_out(captured_output.begin() + f * N, captured_output.begin() + (f + 1) * N);
            std::vector<complex_t> golden = GoldenFFT::compute(frame_in);

            std::string err_msg;
            if (!GoldenFFT::compare(frame_out, golden, 1e-4, &err_msg)) {
                std::cerr << "  FAIL: Frame " << f << " mismatch: " << err_msg << std::endl;
                all_frames_ok = false;
                test_failures++;
            }
        }

        if (all_frames_ok) {
            std::cout << "  PASS: All " << num_frames << " consecutive FFT frames verified against Golden FFT!" << std::endl;
        }
    }

    // SCENARIO 5: Consumer backpressure handling (25-cycle stall)
    void test_scenario_5_backpressure() {
        std::cout << "\n[Test 5] Backpressure Stall (Consumer stalled 25 cycles) + Value Check..." << std::endl;
        dut.set_timing_parameters(3, 1);

        std::vector<complex_t> stimulus(N);
        for (int i = 0; i < N; ++i) stimulus[i] = complex_t(i * 10.0, -i * 5.0);

        std::vector<complex_t> golden = GoldenFFT::compute(stimulus);

        run_concurrent_transfer(stimulus, 25);

        std::string err_msg;
        bool values_ok = GoldenFFT::compare(captured_output, golden, 1e-4, &err_msg);
        if (!values_ok) {
            std::cerr << "  FAIL: Value mismatch under backpressure: " << err_msg << std::endl;
            test_failures++;
        } else {
            std::cout << "  PASS: Buffer integrity preserved under backpressure, values match Golden FFT!" << std::endl;
        }
    }

    // SCENARIO 6: Mathematical verification against analytical benchmarks
    void test_scenario_6_math_correctness() {
        std::cout << "\n[Test 6] Analytical Benchmark Verification (Impulse & DC)..." << std::endl;
        dut.set_timing_parameters(2, 0);

        // Case A: Kronecker Delta Impulse [1, 0, 0, 0, 0, 0, 0, 0] -> FFT should be all 1s
        std::vector<complex_t> impulse(N, complex_t(0, 0));
        impulse[0] = complex_t(1.0, 0.0);
        run_concurrent_transfer(impulse);

        bool impulse_ok = true;
        for (int i = 0; i < N; ++i) {
            if (std::abs(captured_output[i].real - 1.0) > 1e-5 || std::abs(captured_output[i].imag) > 1e-5) {
                impulse_ok = false;
                test_failures++;
            }
        }

        // Case B: Constant DC signal [2, 2, 2, 2, 2, 2, 2, 2] -> FFT bin 0 is 16, others 0
        std::vector<complex_t> dc(N, complex_t(2.0, 0.0));
        run_concurrent_transfer(dc);

        bool dc_ok = true;
        if (std::abs(captured_output[0].real - (2.0 * N)) > 1e-5 || std::abs(captured_output[0].imag) > 1e-5) {
            dc_ok = false;
            test_failures++;
        }
        for (int i = 1; i < N; ++i) {
            if (std::abs(captured_output[i].real) > 1e-5 || std::abs(captured_output[i].imag) > 1e-5) {
                dc_ok = false;
                test_failures++;
            }
        }

        if (impulse_ok && dc_ok) {
            std::cout << "  PASS: Impulse and DC responses verified perfectly against analytical values!" << std::endl;
        }
    }
};

int sc_main(int argc, char* argv[]) {
    sc_report_handler::set_actions(SC_ID_IEEE_1666_DEPRECATION_, SC_DO_NOTHING);
    TbFFT tb("tb_fft");
    sc_start();
    return (tb.test_failures == 0) ? 0 : 1;
}
