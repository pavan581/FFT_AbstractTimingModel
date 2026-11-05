/*
 * memory.h
 *
 * Multi-port AXI4 SRAM Simulation Model.
 * Supports concurrent AXI4 read and write transactions.
 */

#ifndef MEMORY_H
#define MEMORY_H

#include <systemc.h>
#include <axi/axi4.h>
#include <connections/connections.h>
#include <string>

using namespace sc_core;
using namespace axi;

template<unsigned DEPTH = 4096, typename AxiCfg = axi::cfg::standard>
SC_MODULE(Memory) {
    sc_in<bool> clk;
    sc_in<bool> rst_n; // Active-low reset

    // AXI read and write slave interfaces
    typename axi4<AxiCfg>::read::template slave<> read_port;
    typename axi4<AxiCfg>::write::template slave<> write_port;

    sc_uint<AxiCfg::dataWidth> mem[DEPTH];

    static const int bytesPerBeat = AxiCfg::dataWidth / 8;
    static const int addrShift = (AxiCfg::dataWidth == 64) ? 3 : 2;

    // Helper functions for testbench inspection and preloading
    void write_word(unsigned int word_index, sc_uint<AxiCfg::dataWidth> val) {
        if (word_index < DEPTH) {
            mem[word_index] = val;
        }
    }

    sc_uint<AxiCfg::dataWidth> read_word(unsigned int word_index) const {
        if (word_index < DEPTH) {
            return mem[word_index];
        }
        return 0;
    }

    void clear() {
        for (unsigned int i = 0; i < DEPTH; ++i) {
            mem[i] = 0;
        }
    }

    // Read channel process
    void read_port_process() {
        read_port.reset();
        wait();

        while (true) {
            if (!rst_n.read()) {
                read_port.reset();
                wait();
                continue;
            }

            typename axi4<AxiCfg>::AddrPayload req;
            if (read_port.ar.PopNB(req)) {
                unsigned int addr = req.addr;
                int len = req.len;

                for (int beat = 0; beat <= len; ++beat) {
                    typename axi4<AxiCfg>::ReadPayload resp;
                    unsigned int word_idx = addr >> addrShift;
                    resp.data = (word_idx < DEPTH) ? mem[word_idx] : sc_uint<AxiCfg::dataWidth>(0);
                    resp.id = req.id;
                    resp.resp = 0; // OKAY
                    resp.last = (beat == len);

                    read_port.rwrite(resp);
                    addr += bytesPerBeat;
                }
            } else {
                wait();
            }
        }
    }

    // Write channel process
    void write_port_process() {
        write_port.reset();
        wait();

        while (true) {
            if (!rst_n.read()) {
                write_port.reset();
                wait();
                continue;
            }

            typename axi4<AxiCfg>::AddrPayload req;
            if (write_port.aw.PopNB(req)) {
                unsigned int addr = req.addr;
                int len = req.len;

                for (int beat = 0; beat <= len; ++beat) {
                    typename axi4<AxiCfg>::WritePayload wdata = write_port.w.Pop();
                    unsigned int word_idx = addr >> addrShift;
                    if (word_idx < DEPTH) {
                        mem[word_idx] = wdata.data;
                    }
                    addr += bytesPerBeat;
                }

                typename axi4<AxiCfg>::WRespPayload resp;
                resp.id = req.id;
                resp.resp = 0; // OKAY
                write_port.bwrite(resp);
            } else {
                wait();
            }
        }
    }

    SC_CTOR(Memory)
        : clk("clk"),
          rst_n("rst_n"),
          read_port("read_port"),
          write_port("write_port")
    {
        clear();

        SC_THREAD(read_port_process);
        sensitive << clk.pos();
        async_reset_signal_is(rst_n, false);

        SC_THREAD(write_port_process);
        sensitive << clk.pos();
        async_reset_signal_is(rst_n, false);
    }
};

#endif // MEMORY_H

