/*
 * tb_top.cpp
 *
 * System-level testbench for the Multi-Core Interleaved Top Module.
 *
 * Verification Features:
 *   - Verifies parallel multi-core execution with staggered start dispatch (HOP_SIZE).
 *   - Hierarchical signal grouping in top_system.vcd (ctrl.*, core_0.*, core_1.*).
 *   - Traces both INPUT data (in_valid, in_real, in_imag) and OUTPUT data (out_valid, out_real, out_imag, out_mag) per core.
 *   - Golden FFT verification across all cores and all scenarios.
 *   - Multi-block distributed workloads and dynamic timing reconfiguration.
 */

#define CONNECTIONS_NAMING_ORIGINAL
#define BOOST_NULLPTR nullptr

#include <systemc.h>
#include <axi/axi4.h>
#include <vector>
#include <cmath>
#include <cassert>
#include <iostream>
#include <iomanip>
#include "top.h"
#include "memory.h"
#include "tb_tracer.h"

using namespace sc_core;
using namespace axi;

typedef axi::cfg::standard AxiCfg;

SC_MODULE(TbTop) {
    sc_clock clk;
    sc_signal<bool> rst_n;
    sc_signal<bool> start_signal;

    enum {
        N = 16,
        NUM_CORES = 2,
        HOP = 8,
        T1_VAL = 8,
        T2_VAL = 1,
        MEM_DEPTH = 8192,
        bytesPerBeat = AxiCfg::dataWidth / 8
    };

    // Multi-channel memory array
    sc_vector<Memory<MEM_DEPTH, AxiCfg>> mems;

    // AXI memory channels
    sc_vector<typename axi4<AxiCfg>::read::template chan<>> mem_read_chans;
    sc_vector<typename axi4<AxiCfg>::write::template chan<>> mem_write_chans;

    sc_vector<sc_signal<sc_uint<AxiCfg::addrWidth>>> base_addrs;
    sc_vector<sc_signal<int>> num_samples;
    sc_vector<sc_signal<bool>> done_signals;

    Top<N, NUM_CORES, HOP, AxiCfg, T1_VAL, T2_VAL> top_sys;

    sc_signal<int> cycle_count;
    int test_failures;

    // VCD waveform tracing signals
    sc_trace_file* tf;
    sc_vector<sc_signal<bool>>   core_in_valid;
    sc_vector<sc_signal<double>> core_in_real;
    sc_vector<sc_signal<double>> core_in_imag;
    sc_vector<sc_signal<bool>>   core_out_valid;
    sc_vector<sc_signal<double>> core_out_real;
    sc_vector<sc_signal<double>> core_out_imag;
    sc_vector<sc_signal<double>> core_out_mag;

    // Testbench CSV loggers per core
    std::vector<DataTracer*> tracers;

    SC_CTOR(TbTop)
        : clk("clk", 1, SC_NS),
          rst_n("rst_n"),
          start_signal("start_signal"),
          mems("mems", NUM_CORES),
          mem_read_chans("mem_read_chans", NUM_CORES),
          mem_write_chans("mem_write_chans", NUM_CORES),
          base_addrs("base_addrs", NUM_CORES),
          num_samples("num_samples", NUM_CORES),
          done_signals("done_signals", NUM_CORES),
          top_sys("top_sys"),
          cycle_count("cycle_count"),
          test_failures(0),
          tf(nullptr),
          core_in_valid("core_in_valid", NUM_CORES),
          core_in_real("core_in_real", NUM_CORES),
          core_in_imag("core_in_imag", NUM_CORES),
          core_out_valid("core_out_valid", NUM_CORES),
          core_out_real("core_out_real", NUM_CORES),
          core_out_imag("core_out_imag", NUM_CORES),
          core_out_mag("core_out_mag", NUM_CORES)
    {
        for (int i = 0; i < NUM_CORES; ++i) {
            tracers.push_back(new DataTracer("top_core_" + std::to_string(i), "./out/data"));
        }

        // Hierarchical signal grouping for GTKWave
        tf = sc_create_vcd_trace_file("./out/vcd/top_system");
        if (tf) {
            // Group: Top-level coordination & control
            sc_trace(tf, clk, "ctrl.clk");
            sc_trace(tf, rst_n, "ctrl.rst_n");
            sc_trace(tf, start_signal, "ctrl.start");
            sc_trace(tf, top_sys.active_stagger, "ctrl.active_stagger");
            sc_trace(tf, top_sys.stagger_counter, "ctrl.stagger_counter");
            sc_trace(tf, cycle_count, "ctrl.cycle_count");

            // Per-core hierarchical grouping
            for (int i = 0; i < NUM_CORES; ++i) {
                std::string prefix = "core_" + std::to_string(i) + ".";

                sc_trace(tf, top_sys.core_starts[i], (prefix + "start").c_str());
                sc_trace(tf, top_sys.core_busy[i],   (prefix + "busy").c_str());
                sc_trace(tf, done_signals[i],         (prefix + "done").c_str());

                // Input signals
                sc_trace(tf, core_in_valid[i],       (prefix + "in_valid").c_str());
                sc_trace(tf, core_in_real[i],        (prefix + "in_real").c_str());
                sc_trace(tf, core_in_imag[i],        (prefix + "in_imag").c_str());

                // Output signals
                sc_trace(tf, core_out_valid[i],      (prefix + "out_valid").c_str());
                sc_trace(tf, core_out_real[i],       (prefix + "out_real").c_str());
                sc_trace(tf, core_out_imag[i],       (prefix + "out_imag").c_str());
                sc_trace(tf, core_out_mag[i],        (prefix + "out_mag").c_str());
                
                // Internal FFT Core Status
                sc_trace(tf, top_sys.cores[i].fft_out_valid, (prefix + "fft_out_valid").c_str());
            }
        }

        // Memory connections
        for (int i = 0; i < NUM_CORES; ++i) {
            mems[i].clk(clk);
            mems[i].rst_n(rst_n);
            mems[i].read_port(mem_read_chans[i]);
            mems[i].write_port(mem_write_chans[i]);
        }

        // Top system connections
        top_sys.clk(clk);
        top_sys.rst_n(rst_n);
        top_sys.start(start_signal);
        top_sys.mem_read_ports(mem_read_chans);
        top_sys.mem_write_ports(mem_write_chans);
        top_sys.base_addrs(base_addrs);
        top_sys.num_samples(num_samples);
        top_sys.done(done_signals);

        SC_METHOD(cycle_tracker);
        sensitive << clk.posedge_event();

        SC_THREAD(run_top_tests);
        sensitive << clk.posedge_event();
    }

    ~TbTop() {
        if (tf) {
            sc_close_vcd_trace_file(tf);
        }
        for (auto* t : tracers) {
            delete t;
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
        start_signal.write(false);
        for (int i = 0; i < NUM_CORES; ++i) {
            base_addrs[i].write(0);
            num_samples[i].write(0);
            core_in_valid[i].write(false);
            core_in_real[i].write(0.0);
            core_in_imag[i].write(0.0);
            core_out_valid[i].write(false);
            core_out_real[i].write(0.0);
            core_out_imag[i].write(0.0);
            core_out_mag[i].write(0.0);
        }
        wait(5);
        rst_n.write(true);
        wait(2);
    }

    void run_top_tests() {
        std::cout << "\n========================================================" << std::endl;
        std::cout << "  RUNNING MULTI-CORE Top SYSTEM TESTS (Cores=" << NUM_CORES
                  << ", Hop=" << HOP << ", N=" << N << ")" << std::endl;
        std::cout << "  (Verifying BOTH cycle timing AND Golden FFT values)" << std::endl;
        std::cout << "========================================================" << std::endl;

        reset_system();
        wait(25); // Initial buffer gap

        test_scenario_1_staggered_execution();
        wait(25); // Buffer gap to observe idle behavior in later cycles

        test_scenario_2_multiblock_workload();
        wait(25); // Buffer gap

        test_scenario_3_timing_reconfig();
        wait(25); // Buffer gap

        std::cout << "\n--------------------------------------------------------" << std::endl;
        if (test_failures == 0) {
            std::cout << "  >>> ALL Multi-Core Top SCENARIOS PASSED! <<<" << std::endl;
        } else {
            std::cout << "  >>> Top SYSTEM COMPLETED WITH " << test_failures << " FAILURES! <<<" << std::endl;
        }
        std::cout << "--------------------------------------------------------\n" << std::endl;

        sc_stop();
    }

    // SCENARIO 1: Staggered Execution Verification + Golden Value Check
    void test_scenario_1_staggered_execution() {
        std::cout << "\n[Test 1] Staggered Multi-Core Dispatch (HOP=" << HOP << ") + Value Check..." << std::endl;

        std::vector<std::vector<complex_t>> core_inputs(NUM_CORES, std::vector<complex_t>(N));
        std::vector<std::vector<complex_t>> core_goldens(NUM_CORES);

        for (int c = 0; c < NUM_CORES; ++c) {
            unsigned int base_word = c * 0x1000 / bytesPerBeat;
            for (int i = 0; i < N; ++i) {
                core_inputs[c][i] = complex_t((c + 1.0) * (i + 1), (i + 1) * 2.0);
                mems[c].write_word(base_word + i, pack_complex<AxiCfg>(core_inputs[c][i]));
                tracers[c]->log_input(core_inputs[c][i]);

                // Trace input sample into VCD for GTKWave inspection
                core_in_valid[c].write(true);
                core_in_real[c].write(core_inputs[c][i].real);
                core_in_imag[c].write(core_inputs[c][i].imag);
                wait();
                core_in_valid[c].write(false);
            }
            core_goldens[c] = GoldenFFT::compute(core_inputs[c]);
            base_addrs[c].write(c * 0x1000);
            num_samples[c].write(N);
        }

        std::vector<int> done_cycles(NUM_CORES, 0);
        int start_cycle = cycle_count.read();
        start_signal.write(true);

        int completed_count = 0;
        std::vector<bool> recorded(NUM_CORES, false);

        while (completed_count < NUM_CORES) {
            wait();
            for (int c = 0; c < NUM_CORES; ++c) {
                if (done_signals[c].read() && !recorded[c]) {
                    done_cycles[c] = cycle_count.read();
                    recorded[c] = true;
                    completed_count++;
                    std::cout << "  Core " << c << " finished at cycle " << done_cycles[c]
                              << " (" << (done_cycles[c] - start_cycle) << " cycles from start)" << std::endl;
                }
            }
        }
        start_signal.write(false);
        wait(2);

        // Verify outputs in each core's memory against Golden FFT
        bool all_cores_match = true;
        for (int c = 0; c < NUM_CORES; ++c) {
            unsigned int out_word = (c * 0x1000 / bytesPerBeat) + N;
            std::vector<complex_t> actual(N);
            for (int i = 0; i < N; ++i) {
                actual[i] = unpack_complex<AxiCfg>(mems[c].read_word(out_word + i));
                tracers[c]->log_output(actual[i]);

                // Trace output sample into VCD
                core_out_valid[c].write(true);
                core_out_real[c].write(actual[i].real);
                core_out_imag[c].write(actual[i].imag);
                core_out_mag[c].write(actual[i].magnitude());
                wait();
                core_out_valid[c].write(false);
            }

            std::string err;
            bool ok = GoldenFFT::compare_rounded(actual, core_goldens[c], 1e-3, &err);
            if (!ok) {
                std::cerr << "  FAIL: Core " << c << " mismatch: " << err << std::endl;
                all_cores_match = false;
                test_failures++;
            }
        }

        // Verify stagger interval: Core 1 should finish exactly HOP cycles after Core 0
        int delta_finish = done_cycles[1] - done_cycles[0];
        std::cout << "  Stagger difference (Core 1 - Core 0): " << delta_finish << " cycles (expected " << HOP << ")" << std::endl;

        if (all_cores_match && delta_finish == HOP) {
            std::cout << "  PASS: Staggered execution timing and per-core Golden FFT values verified!" << std::endl;
        } else {
            if (delta_finish != HOP) {
                std::cerr << "  FAIL: Stagger discrepancy: delta=" << delta_finish << " != expected " << HOP << std::endl;
                test_failures++;
            }
        }
    }

    // SCENARIO 2: Multi-block distributed workload
    void test_scenario_2_multiblock_workload() {
        std::cout << "\n[Test 2] Multi-block Distributed Workload (32 samples per core) + Value Check..." << std::endl;

        const int samples_per_core = 32; // 2 blocks of 16-point FFT per core
        std::vector<std::vector<complex_t>> multi_inputs(NUM_CORES, std::vector<complex_t>(samples_per_core));
        std::vector<std::vector<complex_t>> multi_goldens(NUM_CORES, std::vector<complex_t>(samples_per_core));

        for (int c = 0; c < NUM_CORES; ++c) {
            unsigned int base_word = (0x4000 + c * 0x2000) / bytesPerBeat;
            for (int b = 0; b < 2; ++b) {
                std::vector<complex_t> blk_in(N);
                for (int i = 0; i < N; ++i) {
                    blk_in[i] = complex_t((c + 1) * 10 + b * 5 + i, i * 2);
                    multi_inputs[c][b * N + i] = blk_in[i];
                    mems[c].write_word(base_word + b * N + i, pack_complex<AxiCfg>(blk_in[i]));
                    tracers[c]->log_input(blk_in[i]);
                }
                std::vector<complex_t> blk_gold = GoldenFFT::compute(blk_in);
                for (int i = 0; i < N; ++i) {
                    multi_goldens[c][b * N + i] = blk_gold[i];
                }
            }
            base_addrs[c].write(0x4000 + c * 0x2000);
            num_samples[c].write(samples_per_core);
        }

        start_signal.write(true);
        int done_count = 0;
        std::vector<bool> done_flags(NUM_CORES, false);

        while (done_count < NUM_CORES) {
            wait();
            for (int c = 0; c < NUM_CORES; ++c) {
                if (done_signals[c].read() && !done_flags[c]) {
                    done_flags[c] = true;
                    done_count++;
                }
            }
        }
        start_signal.write(false);
        wait(2);

        bool all_ok = true;
        for (int c = 0; c < NUM_CORES; ++c) {
            unsigned int out_word = (0x4000 + c * 0x2000) / bytesPerBeat + samples_per_core;
            std::vector<complex_t> actual(samples_per_core);
            for (int i = 0; i < samples_per_core; ++i) {
                actual[i] = unpack_complex<AxiCfg>(mems[c].read_word(out_word + i));
                tracers[c]->log_output(actual[i]);
            }
            std::string err;
            if (!GoldenFFT::compare_rounded(actual, multi_goldens[c], 1e-3, &err)) {
                std::cerr << "  FAIL: Multi-block mismatch on Core " << c << ": " << err << std::endl;
                all_ok = false;
                test_failures++;
            }
        }

        if (all_ok) {
            std::cout << "  PASS: Multi-block workload across all " << NUM_CORES << " cores matches Golden FFT!" << std::endl;
        }
    }

    // SCENARIO 3: Dynamic timing reconfiguration
    void test_scenario_3_timing_reconfig() {
        std::cout << "\n[Test 3] Dynamic Timing Reconfiguration (T1=4, T2=1) + Value Check..." << std::endl;

        top_sys.set_timing_parameters(4, 1);

        for (int c = 0; c < NUM_CORES; ++c) {
            unsigned int base_word = (0x8000 + c * 0x1000) / bytesPerBeat;
            for (int i = 0; i < N; ++i) {
                complex_t sample(i + 3, i + 1);
                mems[c].write_word(base_word + i, pack_complex<AxiCfg>(sample));
            }
            base_addrs[c].write(0x8000 + c * 0x1000);
            num_samples[c].write(N);
        }

        int t_start = cycle_count.read();
        start_signal.write(true);

        int done_count = 0;
        std::vector<bool> done_flags(NUM_CORES, false);
        while (done_count < NUM_CORES) {
            wait();
            for (int c = 0; c < NUM_CORES; ++c) {
                if (done_signals[c].read() && !done_flags[c]) {
                    done_flags[c] = true;
                    done_count++;
                }
            }
        }
        int total_dur = cycle_count.read() - t_start;
        start_signal.write(false);
        wait(2);

        // Verify values
        std::vector<complex_t> sample_in(N);
        for (int i = 0; i < N; ++i) sample_in[i] = complex_t(i + 3, i + 1);
        std::vector<complex_t> golden = GoldenFFT::compute(sample_in);

        bool values_ok = true;
        for (int c = 0; c < NUM_CORES; ++c) {
            unsigned int out_word = (0x8000 + c * 0x1000) / bytesPerBeat + N;
            std::vector<complex_t> actual(N);
            for (int i = 0; i < N; ++i) {
                actual[i] = unpack_complex<AxiCfg>(mems[c].read_word(out_word + i));
            }
            std::string err;
            if (!GoldenFFT::compare_rounded(actual, golden, 1e-3, &err)) {
                values_ok = false;
                test_failures++;
            }
        }

        std::cout << "  Reconfigured (T1=4, T2=1) completed in " << total_dur << " cycles." << std::endl;

        if (values_ok) {
            std::cout << "  PASS: Dynamic timing reconfig verified with accurate Golden FFT values!" << std::endl;
        }
    }
};

int sc_main(int argc, char* argv[]) {
    sc_report_handler::set_actions(SC_ID_IEEE_1666_DEPRECATION_, SC_DO_NOTHING);
    TbTop tb("tb_top");
    sc_start();
    return (tb.test_failures == 0) ? 0 : 1;
}
