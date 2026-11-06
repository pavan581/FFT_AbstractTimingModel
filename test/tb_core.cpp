/*
 * tb_core.cpp
 *
 * Integrated testbench for the Core module (AXI DMA + Abstract Timed FFT).
 *
 * Verification Features:
 *   - Verifies end-to-end processing: Memory -> DMA read -> FFT -> DMA write -> Memory.
 *   - Hierarchical signal grouping in VCD traces (ctrl.*, input.*, output.*).
 *   - Golden FFT verification across single-block, multi-block, and tone signals.
 *   - Dynamic T1 / T2 reconfiguration verification.
 */

#define CONNECTIONS_NAMING_ORIGINAL
#define BOOST_NULLPTR nullptr

#include <systemc.h>
#include <axi/axi4.h>
#include <connections/connections_trace.h>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include <iomanip>
#include "core.h"
#include "memory.h"
#include "tb_tracer.h"

using namespace sc_core;
using namespace axi;

typedef axi::cfg::standard AxiCfg;

SC_MODULE(TbCore) {
    sc_clock clk;
    sc_signal<bool> rst_n;

    // Control and status interface
    sc_signal<bool> start;
    sc_signal<sc_uint<AxiCfg::addrWidth>> base_addr;
    sc_signal<int> num_samples;
    sc_signal<bool> busy;
    sc_signal<bool> done;

    // AXI memory bus channels
    typename axi4<AxiCfg>::read::template chan<> mem_read_chan;
    typename axi4<AxiCfg>::write::template chan<> mem_write_chan;

    // Parameters
    enum {
        N = 16,
        T1_DEFAULT = 12,
        T2_DEFAULT = 2,
        MEM_DEPTH = 8192,
        bytesPerBeat = AxiCfg::dataWidth / 8
    };

    // Submodules
    Memory<MEM_DEPTH, AxiCfg> mem;
    Core<N, AxiCfg, T1_DEFAULT, T2_DEFAULT> core;

    sc_signal<int> cycle_count;
    int test_failures;

    // VCD waveform tracing
    sc_trace_file* tf;

    // CSV Data Logger
    DataTracer tracer;

    SC_CTOR(TbCore)
        : clk("clk", 1, SC_NS),
          rst_n("rst_n"),
          start("start"),
          base_addr("base_addr"),
          num_samples("num_samples"),
          busy("busy"),
          done("done"),
          mem_read_chan("mem_read_chan"),
          mem_write_chan("mem_write_chan"),
          mem("mem"),
          core("core"),
          cycle_count("cycle_count"),
          test_failures(0),
          tf(nullptr),
          tracer("core_unit", "./out/data")
    {
        // Hierarchical signal grouping for GTKWave
        tf = sc_create_vcd_trace_file("./out/vcd/core_unit");
        if (tf) {
            // Group: Control interface
            sc_trace(tf, clk, "ctrl.clk");
            sc_trace(tf, rst_n, "ctrl.rst_n");
            sc_trace(tf, cycle_count, "ctrl.cycle_count");
            sc_trace(tf, start, "core.ctrl.start");
            sc_trace(tf, busy, "core.ctrl.busy");
            sc_trace(tf, done, "core.ctrl.done");
            sc_trace(tf, base_addr, "core.cfg.base_addr");
            sc_trace(tf, num_samples, "core.cfg.num_samples");

            // Group: FFT Core Connections ports (rdy, val, msg)
            sc_trace(tf, core.fft.in_data.rdy, "fft.in.rdy");
            sc_trace(tf, core.fft.in_data.val, "fft.in.valid");
            sc_trace(tf, core.fft.in_data.msg, "fft.in.msg");

            sc_trace(tf, core.fft.out_data.rdy, "fft.out.rdy");
            sc_trace(tf, core.fft.out_data.val, "fft.out.valid");
            sc_trace(tf, core.fft.out_data.msg, "fft.out.msg");
        }

        // Memory connections
        mem.clk(clk);
        mem.rst_n(rst_n);
        mem.read_port(mem_read_chan);
        mem.write_port(mem_write_chan);

        // Core connections
        core.clk(clk);
        core.rst_n(rst_n);
        core.start(start);
        core.base_addr(base_addr);
        core.num_samples(num_samples);
        core.busy(busy);
        core.done(done);
        core.mem_read_port(mem_read_chan);
        core.mem_write_port(mem_write_chan);

        SC_METHOD(cycle_tracker);
        sensitive << clk.posedge_event();

        SC_THREAD(run_core_tests);
        sensitive << clk.posedge_event();
    }

    ~TbCore() {
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

    void reset_system() {
        rst_n.write(false);
        start.write(false);
        base_addr.write(0);
        num_samples.write(0);
        wait(5);
        rst_n.write(true);
        wait(2);
    }

    void run_core_tests() {
        std::cout << "\n========================================================" << std::endl;
        std::cout << "  RUNNING UNIT TESTS FOR INTEGRATED Core MODULE (N=" << N << ")" << std::endl;
        std::cout << "  (Verifying BOTH cycle timing AND Golden FFT values)" << std::endl;
        std::cout << "========================================================" << std::endl;

        reset_system();
        wait(25); // Initial buffer gap

        test_scenario_1_single_block_fft();
        wait(25); // Buffer gap to observe idle behavior in later cycles

        test_scenario_2_multi_block_fft();
        wait(25); // Buffer gap

        test_scenario_3_timing_variation();
        wait(25); // Buffer gap

        test_scenario_4_frequency_tone();
        wait(25); // Buffer gap

        std::cout << "\n--------------------------------------------------------" << std::endl;
        if (test_failures == 0) {
            std::cout << "  >>> ALL Core MODULE SCENARIOS PASSED! <<<" << std::endl;
        } else {
            std::cout << "  >>> Core MODULE COMPLETED WITH " << test_failures << " FAILURES! <<<" << std::endl;
        }
        std::cout << "--------------------------------------------------------\n" << std::endl;

        sc_stop();
    }

    // SCENARIO 1: Single-block end-to-end processing
    void test_scenario_1_single_block_fft() {
        std::cout << "\n[Test 1] Single Block End-to-End Processing (N=" << N
                  << ", T1=" << T1_DEFAULT << ", T2=" << T2_DEFAULT << ")..." << std::endl;

        const unsigned int base = 0x0;

        std::vector<complex_t> input_samples(N);
        for (int i = 0; i < N; ++i) {
            input_samples[i] = complex_t(i + 1, std::round(5.0 * std::sin(i)));
            mem.write_word(i, pack_complex<AxiCfg>(input_samples[i]));
            tracer.log_input(input_samples[i]);
        }

        std::vector<complex_t> golden = GoldenFFT::compute(input_samples);

        int start_cycle = cycle_count.read();
        base_addr.write(base);
        num_samples.write(N);
        start.write(true);

        wait();
        while (!done.read()) {
            wait();
        }
        int finish_cycle = cycle_count.read();
        start.write(false);
        wait(2);

        int total_cycles = finish_cycle - start_cycle;
        std::cout << "  Transfer & FFT calculation finished in " << total_cycles << " cycles." << std::endl;

        // Read and verify output written back to memory
        unsigned int out_start = N;
        std::vector<complex_t> actual(N);
        for (int i = 0; i < N; ++i) {
            actual[i] = unpack_complex<AxiCfg>(mem.read_word(out_start + i));
            tracer.log_output(actual[i]);
        }

        std::string err;
        bool match = GoldenFFT::compare_rounded(actual, golden, 1e-3, &err);
        if (match) {
            std::cout << "  PASS: End-to-end memory output matches Golden FFT across all " << N << " bins!" << std::endl;
        } else {
            std::cerr << "  FAIL: Memory output mismatch: " << err << std::endl;
            test_failures++;
        }
    }

    // SCENARIO 2: Multi-block continuous processing
    void test_scenario_2_multi_block_fft() {
        std::cout << "\n[Test 2] Multi-block Continuous Processing (64 samples = 4 x 16-point FFTs)..." << std::endl;
        const int total_count = 64;
        const unsigned int base = 0x1000;
        const unsigned int base_word = base / bytesPerBeat;

        std::vector<complex_t> all_inputs(total_count);
        std::vector<complex_t> all_golden(total_count);

        for (int b = 0; b < 4; ++b) {
            std::vector<complex_t> block_in(N);
            for (int i = 0; i < N; ++i) {
                block_in[i] = complex_t(b * 10.0 + i + 1, (i + 1) * 3.0);
                all_inputs[b * N + i] = block_in[i];
                mem.write_word(base_word + b * N + i, pack_complex<AxiCfg>(block_in[i]));
                tracer.log_input(block_in[i]);
            }
            std::vector<complex_t> block_golden = GoldenFFT::compute(block_in);
            for (int i = 0; i < N; ++i) {
                all_golden[b * N + i] = block_golden[i];
            }
        }

        base_addr.write(base);
        num_samples.write(total_count);
        start.write(true);

        wait();
        while (!done.read()) {
            wait();
        }
        start.write(false);
        wait(2);

        unsigned int out_word_start = base_word + total_count;
        std::vector<complex_t> all_actual(total_count);
        for (int i = 0; i < total_count; ++i) {
            all_actual[i] = unpack_complex<AxiCfg>(mem.read_word(out_word_start + i));
            tracer.log_output(all_actual[i]);
        }

        std::string err;
        bool mb_ok = GoldenFFT::compare_rounded(all_actual, all_golden, 1e-3, &err);
        if (mb_ok) {
            std::cout << "  PASS: Multi-block 64-sample FFT verified across all 4 frames against Golden FFT!" << std::endl;
        } else {
            std::cerr << "  FAIL: Multi-block value error: " << err << std::endl;
            test_failures++;
        }
    }

    // SCENARIO 3: Dynamic timing parameter variations
    void test_scenario_3_timing_variation() {
        std::cout << "\n[Test 3] Dynamic Timing Parameter Variation + Value Check..." << std::endl;
        const unsigned int base = 0x2000;
        const unsigned int base_word = base / bytesPerBeat;

        std::vector<complex_t> test_in(N);
        for (int i = 0; i < N; ++i) {
            test_in[i] = complex_t(i + 5, 2);
            mem.write_word(base_word + i, pack_complex<AxiCfg>(test_in[i]));
        }
        std::vector<complex_t> golden = GoldenFFT::compute(test_in);

        // Test configuration A: T1=4, T2=0 (Fast)
        core.set_timing_parameters(4, 0);
        base_addr.write(base);
        num_samples.write(N);
        start.write(true);

        int t_start_A = cycle_count.read();
        wait();
        while (!done.read()) wait();
        int t_end_A = cycle_count.read();
        start.write(false);
        wait(2);
        int duration_A = t_end_A - t_start_A;

        // Verify output values for Config A
        std::vector<complex_t> actual_A(N);
        for (int i = 0; i < N; ++i) {
            actual_A[i] = unpack_complex<AxiCfg>(mem.read_word(base_word + N + i));
        }
        std::string err_A;
        bool val_A_ok = GoldenFFT::compare_rounded(actual_A, golden, 1e-3, &err_A);

        // Test configuration B: T1=24, T2=5 (Slow)
        core.set_timing_parameters(24, 5);
        base_addr.write(base);
        num_samples.write(N);
        start.write(true);

        int t_start_B = cycle_count.read();
        wait();
        while (!done.read()) wait();
        int t_end_B = cycle_count.read();
        start.write(false);
        wait(2);
        int duration_B = t_end_B - t_start_B;

        // Verify output values for Config B
        std::vector<complex_t> actual_B(N);
        for (int i = 0; i < N; ++i) {
            actual_B[i] = unpack_complex<AxiCfg>(mem.read_word(base_word + N + i));
        }
        std::string err_B;
        bool val_B_ok = GoldenFFT::compare_rounded(actual_B, golden, 1e-3, &err_B);

        std::cout << "  Config A (T1=4, T2=0) duration:  " << duration_A << " cycles." << std::endl;
        std::cout << "  Config B (T1=24, T2=5) duration: " << duration_B << " cycles." << std::endl;

        if (duration_B > duration_A && val_A_ok && val_B_ok) {
            std::cout << "  PASS: Dynamic timing variations verified and values match Golden FFT!" << std::endl;
        } else {
            std::cerr << "  FAIL: Timing variation check failed (val_A=" << val_A_ok << ", val_B=" << val_B_ok << ")" << std::endl;
            test_failures++;
        }
    }

    // SCENARIO 4: Complex tone spectrum verification
    void test_scenario_4_frequency_tone() {
        std::cout << "\n[Test 4] Complex Tone Spectrum Verification..." << std::endl;
        const unsigned int base = 0x3000;
        const unsigned int base_word = base / bytesPerBeat;
        const int target_bin = 3;

        std::vector<complex_t> tone(N);
        const double PI = std::acos(-1.0);
        for (int i = 0; i < N; ++i) {
            double angle = 2.0 * PI * target_bin * i / N;
            tone[i] = complex_t(std::round(10.0 * std::cos(angle)), std::round(10.0 * std::sin(angle)));
            mem.write_word(base_word + i, pack_complex<AxiCfg>(tone[i]));
        }
        std::vector<complex_t> golden = GoldenFFT::compute(tone);

        core.set_timing_parameters(T1_DEFAULT, T2_DEFAULT);
        base_addr.write(base);
        num_samples.write(N);
        start.write(true);

        wait();
        while (!done.read()) wait();
        start.write(false);
        wait(2);

        std::vector<complex_t> actual(N);
        for (int i = 0; i < N; ++i) {
            actual[i] = unpack_complex<AxiCfg>(mem.read_word(base_word + N + i));
        }

        std::string err;
        bool tone_ok = GoldenFFT::compare_rounded(actual, golden, 1e-3, &err);

        // Find peak bin in actual output
        int max_bin = 0;
        double max_mag = 0.0;
        for (int i = 0; i < N; ++i) {
            double mag = actual[i].magnitude();
            if (mag > max_mag) {
                max_mag = mag;
                max_bin = i;
            }
        }

        if (tone_ok && max_bin == target_bin) {
            std::cout << "  PASS: Tone spectrum verified against Golden FFT (peak located at bin " << max_bin << ")!" << std::endl;
        } else {
            std::cerr << "  FAIL: Tone peak at bin " << max_bin << " (expected " << target_bin << ")" << std::endl;
            test_failures++;
        }
    }
};

int sc_main(int argc, char* argv[]) {
    sc_report_handler::set_actions(SC_ID_IEEE_1666_DEPRECATION_, SC_DO_NOTHING);
    TbCore tb("tb_core");
    sc_start();
    return (tb.test_failures == 0) ? 0 : 1;
}
