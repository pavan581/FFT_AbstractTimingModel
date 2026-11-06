/*
 * core.h
 *
 * Integrated Processing Core.
 * Combines an AXI4 DMA controller with an abstract timed FFT compute engine.
 * Pure architectural hardware model without testbench instrumentation.
 */

#ifndef CORE_H
#define CORE_H

#include <systemc.h>
#include <axi/axi4.h>
#include <connections/connections.h>
#include "dma.h"
#include "fft_timed.h"

using namespace sc_core;
using namespace axi;
using namespace Connections;

template<int N_SIZE, typename AxiCfg, int FIRST_SAMPLE_LATENCY = 8, int INTER_SAMPLE_GAP = 1, int OUTPUT_HOLD_CYCLES = 1>
SC_MODULE(Core) {
    // Clock and active-low reset
    sc_in<bool> clk;
    sc_in<bool> rst_n;

    // Control and status interface
    sc_in<bool> start;
    sc_in<sc_uint<AxiCfg::addrWidth>> base_addr;
    sc_in<int> num_samples;
    sc_out<bool> busy;
    sc_out<bool> done;

    // AXI Memory master ports
    typename axi4<AxiCfg>::read::template master<> mem_read_port;
    typename axi4<AxiCfg>::write::template master<> mem_write_port;

    // Internal Connections channels connecting DMA <-> FFT
    Combinational<complex_t> dma_to_fft_chan;
    Combinational<complex_t> fft_to_dma_chan;

    // Submodules
    DMA<AxiCfg, N_SIZE, FIRST_SAMPLE_LATENCY, INTER_SAMPLE_GAP, OUTPUT_HOLD_CYCLES> dma;
    FFTTimed<N_SIZE, FIRST_SAMPLE_LATENCY, INTER_SAMPLE_GAP, OUTPUT_HOLD_CYCLES> fft;

    SC_CTOR(Core)
        : clk("clk"),
          rst_n("rst_n"),
          start("start"),
          base_addr("base_addr"),
          num_samples("num_samples"),
          busy("busy"),
          done("done"),
          mem_read_port("mem_read_port"),
          mem_write_port("mem_write_port"),
          dma_to_fft_chan("dma_to_fft_chan"),
          fft_to_dma_chan("fft_to_dma_chan"),
          dma("dma"),
          fft("fft")
    {
        // DMA port bindings
        dma.clk(clk);
        dma.rst_n(rst_n);
        dma.start(start);
        dma.base_addr(base_addr);
        dma.num_samples(num_samples);
        dma.busy(busy);
        dma.done(done);
        dma.mem_read_port(mem_read_port);
        dma.mem_write_port(mem_write_port);
        dma.fft_out(dma_to_fft_chan);
        dma.fft_in(fft_to_dma_chan);

        // FFT port bindings
        fft.clk(clk);
        fft.rst_n(rst_n);
        fft.in_data(dma_to_fft_chan);
        fft.out_data(fft_to_dma_chan);
    }

    void set_timing_parameters(int t1, int t2, int t3 = 1) {
        fft.set_timing_parameters(t1, t2, t3);
    }
};

#endif // CORE_H
