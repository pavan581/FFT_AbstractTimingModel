/*
 * top.h
 *
 * Multi-Core Interleaved FFT System Coordinator.
 * Coordinates parallel processing cores with staggered start triggers (HOP_SIZE)
 * to balance AXI bus bandwidth and eliminate memory port contention.
 * Pure architectural hardware model without testbench instrumentation.
 */

#ifndef TOP_H
#define TOP_H

#include <systemc.h>
#include <vector>
#include <axi/axi4.h>
#include "core.h"

using namespace sc_core;
using namespace axi;

template<int N_SIZE, int NUM_CORES, int HOP_SIZE, typename AxiCfg, int FIRST_SAMPLE_LATENCY = 8, int INTER_SAMPLE_GAP = 1, int OUTPUT_HOLD_CYCLES = 1>
SC_MODULE(Top) {
    // Clock and active-low reset
    sc_in<bool> clk;
    sc_in<bool> rst_n;
    sc_in<bool> start;

    // External multi-channel AXI memory interfaces
    sc_vector<typename axi4<AxiCfg>::read::template master<>> mem_read_ports;
    sc_vector<typename axi4<AxiCfg>::write::template master<>> mem_write_ports;

    // Per-core configuration and status signals
    sc_vector<sc_in<sc_uint<AxiCfg::addrWidth>>> base_addrs;
    sc_vector<sc_in<int>> num_samples;
    sc_vector<sc_out<bool>> done;

    // Internal inter-core coordinating signals
    sc_vector<sc_signal<bool>> core_starts;
    sc_vector<sc_signal<bool>> core_busy;

    // Cores array
    sc_vector<Core<N_SIZE, AxiCfg, FIRST_SAMPLE_LATENCY, INTER_SAMPLE_GAP, OUTPUT_HOLD_CYCLES>> cores;

    // Stagger control state signals
    sc_signal<bool> active_stagger;
    sc_signal<int> stagger_counter;

    SC_HAS_PROCESS(Top);

    Top(sc_module_name name)
        : sc_module(name),
          clk("clk"),
          rst_n("rst_n"),
          start("start"),
          mem_read_ports("mem_read_ports", NUM_CORES),
          mem_write_ports("mem_write_ports", NUM_CORES),
          base_addrs("base_addrs", NUM_CORES),
          num_samples("num_samples", NUM_CORES),
          done("done", NUM_CORES),
          core_starts("core_starts", NUM_CORES),
          core_busy("core_busy", NUM_CORES),
          cores("core", NUM_CORES),
          active_stagger("active_stagger"),
          stagger_counter("stagger_counter")
    {
        for (int i = 0; i < NUM_CORES; ++i) {
            cores[i].clk(clk);
            cores[i].rst_n(rst_n);
            cores[i].start(core_starts[i]);
            cores[i].base_addr(base_addrs[i]);
            cores[i].num_samples(num_samples[i]);
            cores[i].busy(core_busy[i]);
            cores[i].done(done[i]);

            cores[i].mem_read_port(mem_read_ports[i]);
            cores[i].mem_write_port(mem_write_ports[i]);
        }

        SC_METHOD(stagger_control_logic);
        sensitive << clk.pos() << rst_n.neg();
    }

    // Dynamic timing reconfiguration for all cores
    void set_timing_parameters(int t1, int t2, int t3 = 1) {
        for (int i = 0; i < NUM_CORES; ++i) {
            cores[i].set_timing_parameters(t1, t2, t3);
        }
    }

    // Stagger core activation based on HOP_SIZE
    void stagger_control_logic() {
        if (!rst_n.read()) {
            active_stagger.write(false);
            stagger_counter.write(0);
            for (int i = 0; i < NUM_CORES; ++i) {
                core_starts[i].write(false);
            }
        } else {
            bool is_active = active_stagger.read();
            int cnt = stagger_counter.read();

            if (start.read()) {
                if (!is_active) {
                    is_active = true;
                    cnt = 0;
                    active_stagger.write(true);
                    stagger_counter.write(0);
                }

                for (int i = 0; i < NUM_CORES; ++i) {
                    if (cnt >= i * HOP_SIZE) {
                        core_starts[i].write(true);
                    } else {
                        core_starts[i].write(false);
                    }
                }
                stagger_counter.write(cnt + 1);
            } else {
                active_stagger.write(false);
                stagger_counter.write(0);
                for (int i = 0; i < NUM_CORES; ++i) {
                    core_starts[i].write(false);
                }
            }
        }
    }
};

#endif // TOP_H
