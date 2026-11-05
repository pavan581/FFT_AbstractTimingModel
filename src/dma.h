/*
 * dma.h
 *
 * Direct Memory Access (DMA) controller for AXI4 interface transactions.
 * Orchestrates concurrent AXI read address generation, read data stream unpacking,
 * and write data streaming back to memory.
 */

#ifndef DMA_H
#define DMA_H

#include <systemc.h>
#include <axi/axi4.h>
#include <connections/connections.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include "fft_types.h"

using namespace sc_core;
using namespace axi;
using namespace Connections;

template<typename AxiCfg, int N_SIZE = 8, int FIRST_SAMPLE_LATENCY = 8, int INTER_SAMPLE_GAP = 1>
SC_MODULE(DMA) {
    sc_in<bool> clk;
    sc_in<bool> rst_n; // Active-low asynchronous reset

    // Control and status signals
    sc_in<bool> start;
    sc_in<sc_uint<AxiCfg::addrWidth>> base_addr;
    sc_in<int> num_samples;
    sc_out<bool> busy;
    sc_out<bool> done;

    // AXI Master interfaces
    typename axi4<AxiCfg>::read::template master<> mem_read_port;
    typename axi4<AxiCfg>::write::template master<> mem_write_port;

    // Connections streaming channels to/from FFT core
    Out<complex_t> fft_out;
    In<complex_t> fft_in;

    typedef typename axi4<AxiCfg>::AddrPayload AddrPayload;
    typedef typename axi4<AxiCfg>::ReadPayload ReadPayload;
    typedef typename axi4<AxiCfg>::WritePayload WritePayload;
    typedef typename axi4<AxiCfg>::WRespPayload WRespPayload;

    static const int bytesPerBeat = AxiCfg::dataWidth / 8;

    enum DmaState { IDLE, RUN, FINISHED };

    // Helper to generate AXI address requests
    static AddrPayload create_addr_req(typename axi4<AxiCfg>::Addr addr, int len) {
        AddrPayload req;
        req.addr = addr;
        req.id = 0;
        req.len = len; // len = beats - 1
        req.size = (AxiCfg::dataWidth == 64) ? 3 : 2;
        req.burst = 1; // INCR burst
        return req;
    }

    // Helper to generate AXI write data payloads
    static WritePayload create_write_payload(typename axi4<AxiCfg>::Data data, bool last) {
        WritePayload w_pay;
        w_pay.data = data;
        w_pay.wstrb = ~0; // All byte lanes active
        w_pay.last = last;
        return w_pay;
    }

    // Process: AXI Read Address Generation
    void read_addr_thread() {
        mem_read_port.ar.Reset();
        wait();

        while (true) {
            // Wait for transfer initiation
            while (!start.read()) {
                wait();
            }

            int total = num_samples.read();
            typename axi4<AxiCfg>::Addr addr = base_addr.read();

            while (total > 0) {
                int len = (total > 256) ? 256 : total;
                AddrPayload req = create_addr_req(addr, len - 1);
                mem_read_port.ar.Push(req);
                addr += len * bytesPerBeat;
                total -= len;
            }

            // Wait for start to deassert before acknowledging next request
            while (start.read()) {
                wait();
            }
        }
    }

    // Process: AXI Read Data Streaming to FFT
    void read_data_thread() {
        mem_read_port.r.Reset();
        fft_out.Reset();
        wait();

        while (true) {
            while (!start.read()) {
                wait();
            }

            int total = num_samples.read();
            if (total > 0) {
                // Ensure inputs are rounded to multiples of N_SIZE blocks
                int total_padded = ((total + N_SIZE - 1) / N_SIZE) * N_SIZE;
                int pad_count = total_padded - total;

                // Read real samples from AXI memory
                for (int i = 0; i < total; ++i) {
                    ReadPayload resp = mem_read_port.r.Pop();
                    complex_t sample = unpack_complex<AxiCfg>(resp.data);
                    fft_out.Push(sample);
                }

                // Push zero-padding if samples don't align with N_SIZE block
                for (int i = 0; i < pad_count; ++i) {
                    fft_out.Push(complex_t(0.0, 0.0));
                }
            }

            while (start.read()) {
                wait();
            }
        }
    }

    // Process: AXI Write Data Streaming from FFT to Memory
    void write_thread() {
        mem_write_port.aw.Reset();
        mem_write_port.w.Reset();
        mem_write_port.b.Reset();
        fft_in.Reset();
        busy.write(false);
        done.write(false);
        DmaState state = IDLE;
        wait();

        while (true) {
            switch (state) {
                case IDLE:
                    busy.write(false);
                    done.write(false);
                    if (start.read()) {
                        state = RUN;
                    } else {
                        wait();
                    }
                    break;

                case RUN: {
                    busy.write(true);
                    done.write(false);

                    int total = num_samples.read();
                    if (total > 0) {
                        int total_padded = ((total + N_SIZE - 1) / N_SIZE) * N_SIZE;
                        int pad_count = total_padded - total;

                        // Output destination offset (e.g. directly after input samples buffer or custom offset)
                        typename axi4<AxiCfg>::Addr addr = base_addr.read() + total_padded * bytesPerBeat;

                        int remaining = total;
                        while (remaining > 0) {
                            int len = (remaining > 256) ? 256 : remaining;

                            // Address write request
                            AddrPayload aw_pay = create_addr_req(addr, len - 1);
                            mem_write_port.aw.Push(aw_pay);

                            // Stream data beats
                            for (int i = 0; i < len; ++i) {
                                complex_t out_val = fft_in.Pop();
                                typename axi4<AxiCfg>::Data packed = pack_complex<AxiCfg>(out_val);
                                WritePayload w_pay = create_write_payload(packed, i == len - 1);
                                mem_write_port.w.Push(w_pay);
                            }

                            // Wait for write response
                            mem_write_port.b.Pop();

                            addr += len * bytesPerBeat;
                            remaining -= len;
                        }

                        // Consume and discard any padding outputs
                        for (int i = 0; i < pad_count; ++i) {
                            fft_in.Pop();
                        }
                    }

                    state = FINISHED;
                    break;
                }

                case FINISHED:
                    busy.write(false);
                    done.write(true);
                    while (start.read()) {
                        wait();
                    }
                    done.write(false);
                    state = IDLE;
                    break;
            }
        }
    }

    SC_HAS_PROCESS(DMA);

    DMA(sc_module_name name)
        : sc_module(name),
          clk("clk"),
          rst_n("rst_n"),
          start("start"),
          base_addr("base_addr"),
          num_samples("num_samples"),
          busy("busy"),
          done("done"),
          mem_read_port("mem_read_port"),
          mem_write_port("mem_write_port"),
          fft_out("fft_out"),
          fft_in("fft_in")
    {
        SC_THREAD(read_addr_thread);
        sensitive << clk.pos();
        async_reset_signal_is(rst_n, false);

        SC_THREAD(read_data_thread);
        sensitive << clk.pos();
        async_reset_signal_is(rst_n, false);

        SC_THREAD(write_thread);
        sensitive << clk.pos();
        async_reset_signal_is(rst_n, false);
    }
};

#endif // DMA_H

