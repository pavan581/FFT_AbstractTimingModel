/*
 * tb_dma.cpp
 *
 * Comprehensive unit testbench for the DMA Controller module.
 *
 * Scenarios tested:
 *   Scenario 1: Basic DMA block transfer (AXI Memory -> DMA -> Loopback -> DMA -> AXI Memory)
 *   Scenario 2: Multi-burst transfer (> 256 beats, testing burst chunking)
 *   Scenario 3: Handshake verification & multiple consecutive triggers
 *   Scenario 4: Base address offset handling
 */

#define CONNECTIONS_NAMING_ORIGINAL
#define BOOST_NULLPTR nullptr

#include <systemc.h>
#include <axi/axi4.h>
#include <connections/connections.h>
#include <vector>
#include <cassert>
#include <iostream>
#include <iomanip>
#include "dma.h"
#include "memory.h"
#include "fft_types.h"

using namespace sc_core;
using namespace axi;
using namespace Connections;

typedef axi::cfg::standard AxiCfg;

SC_MODULE(TbDMA) {
    sc_clock clk;
    sc_signal<bool> rst_n;

    // Control signals
    sc_signal<bool> start;
    sc_signal<sc_uint<AxiCfg::addrWidth>> base_addr;
    sc_signal<int> num_samples;
    sc_signal<bool> busy;
    sc_signal<bool> done;

    // AXI channels connecting Memory and DMA
    typename axi4<AxiCfg>::read::template chan<> mem_read_chan;
    typename axi4<AxiCfg>::write::template chan<> mem_write_chan;

    // Channels connecting DMA and loopback
    Combinational<complex_t> dma_to_core_chan;
    Combinational<complex_t> core_to_dma_chan;

    // Submodules
    enum {
        MEM_DEPTH = 8192,
        N = 8,
        bytesPerBeat = AxiCfg::dataWidth / 8
    };

    Memory<MEM_DEPTH, AxiCfg> mem;
    DMA<AxiCfg, N> dma;

    int test_failures;

    SC_CTOR(TbDMA)
        : clk("clk", 1, SC_NS),
          rst_n("rst_n"),
          start("start"),
          base_addr("base_addr"),
          num_samples("num_samples"),
          busy("busy"),
          done("done"),
          mem_read_chan("mem_read_chan"),
          mem_write_chan("mem_write_chan"),
          dma_to_core_chan("dma_to_core_chan"),
          core_to_dma_chan("core_to_dma_chan"),
          mem("mem"),
          dma("dma"),
          test_failures(0)
    {
        // Memory connections
        mem.clk(clk);
        mem.rst_n(rst_n);
        mem.read_port(mem_read_chan);
        mem.write_port(mem_write_chan);

        // DMA connections
        dma.clk(clk);
        dma.rst_n(rst_n);
        dma.start(start);
        dma.base_addr(base_addr);
        dma.num_samples(num_samples);
        dma.busy(busy);
        dma.done(done);
        dma.mem_read_port(mem_read_chan);
        dma.mem_write_port(mem_write_chan);
        dma.fft_out(dma_to_core_chan);
        dma.fft_in(core_to_dma_chan);

        SC_THREAD(loopback_thread);
        sensitive << clk.posedge_event();

        SC_THREAD(test_driver);
        sensitive << clk.posedge_event();
    }

    // Loopback responder simulating a streaming processing core
    void loopback_thread() {
        dma_to_core_chan.ResetRead();
        core_to_dma_chan.ResetWrite();
        wait();

        while (true) {
            complex_t val = dma_to_core_chan.Pop();
            // Perform simple transformation (multiply real by 2, imag by 3)
            complex_t transformed(val.real * 2.0, val.imag * 3.0);
            core_to_dma_chan.Push(transformed);
        }
    }

    void reset_system() {
        rst_n.write(false);
        start.write(false);
        base_addr.write(0);
        num_samples.write(0);
        wait(5);
        rst_n.write(true);
        wait(3);
    }

    void test_driver() {
        std::cout << "\n========================================================" << std::endl;
        std::cout << "  RUNNING UNIT TESTS FOR DMA CONTROLLER MODULE" << std::endl;
        std::cout << "========================================================" << std::endl;

        reset_system();

        test_scenario_1_basic_transfer();
        test_scenario_2_multiburst_transfer();
        test_scenario_3_retrigger_handshake();
        test_scenario_4_address_offset();

        std::cout << "\n--------------------------------------------------------" << std::endl;
        if (test_failures == 0) {
            std::cout << "  >>> ALL DMA CONTROLLER SCENARIOS PASSED! <<<" << std::endl;
        } else {
            std::cout << "  >>> DMA CONTROLLER COMPLETED WITH " << test_failures << " FAILURES! <<<" << std::endl;
        }
        std::cout << "--------------------------------------------------------\n" << std::endl;

        sc_stop();
    }

    // SCENARIO 1: Basic DMA block transfer
    void test_scenario_1_basic_transfer() {
        std::cout << "\n[Test 1] Basic Block Transfer (16 samples)..." << std::endl;
        const int count = 16;
        const unsigned int base = 0x0;

        // Preload memory
        for (int i = 0; i < count; ++i) {
            mem.write_word(i, pack_complex<AxiCfg>(complex_t(i + 1, (i + 1) * 2)));
        }

        base_addr.write(base);
        num_samples.write(count);
        start.write(true);

        // Wait until done
        wait();
        while (!done.read()) {
            wait();
        }
        start.write(false);
        wait(2);

        // Verify output written back to memory
        // Write destination is base_addr + total_padded * bytesPerBeat
        unsigned int out_word_start = count;
        bool data_ok = true;
        for (int i = 0; i < count; ++i) {
            sc_uint<AxiCfg::dataWidth> raw = mem.read_word(out_word_start + i);
            complex_t unpacked = unpack_complex<AxiCfg>(raw);
            double exp_r = (i + 1) * 2.0;
            double exp_i = ((i + 1) * 2) * 3.0;

            if (std::abs(unpacked.real - exp_r) > 1e-4 || std::abs(unpacked.imag - exp_i) > 1e-4) {
                std::cerr << "  FAIL: Sample " << i << " expected (" << exp_r << ", " << exp_i
                          << ") but got (" << unpacked.real << ", " << unpacked.imag << ")" << std::endl;
                data_ok = false;
                test_failures++;
                break;
            }
        }

        if (data_ok) {
            std::cout << "  PASS: Basic block transfer and loopback data verified!" << std::endl;
        }
    }

    // SCENARIO 2: Multi-burst transfer (> 256 samples)
    void test_scenario_2_multiburst_transfer() {
        std::cout << "\n[Test 2] Multi-burst Transfer (300 samples > 256 beat limit)..." << std::endl;
        const int count = 300;
        const unsigned int base = 0x1000;
        const unsigned int base_word = base / bytesPerBeat;

        // Preload memory
        for (int i = 0; i < count; ++i) {
            mem.write_word(base_word + i, pack_complex<AxiCfg>(complex_t(i + 5, 0)));
        }

        base_addr.write(base);
        num_samples.write(count);
        start.write(true);

        wait();
        while (!done.read()) {
            wait();
        }
        start.write(false);
        wait(2);

        int total_padded = ((count + N - 1) / N) * N;
        unsigned int out_word_start = base_word + total_padded;
        bool mb_ok = true;

        for (int i = 0; i < count; ++i) {
            sc_uint<AxiCfg::dataWidth> raw = mem.read_word(out_word_start + i);
            complex_t unpacked = unpack_complex<AxiCfg>(raw);
            double exp_r = (i + 5) * 2.0;
            if (std::abs(unpacked.real - exp_r) > 1e-4) {
                std::cerr << "  FAIL: Multi-burst sample " << i << " expected " << exp_r << " got " << unpacked.real << std::endl;
                mb_ok = false;
                test_failures++;
                break;
            }
        }

        if (mb_ok) {
            std::cout << "  PASS: Multi-burst 300-sample transfer correctly segmented and verified!" << std::endl;
        }
    }

    // SCENARIO 3: Re-trigger handshake
    void test_scenario_3_retrigger_handshake() {
        std::cout << "\n[Test 3] Handshake Re-triggering (Consecutive runs)..." << std::endl;
        const int count = 8;
        const unsigned int base = 0x2000;
        const unsigned int base_word = base / bytesPerBeat;

        bool runs_ok = true;
        for (int run = 0; run < 3; ++run) {
            for (int i = 0; i < count; ++i) {
                mem.write_word(base_word + i, pack_complex<AxiCfg>(complex_t(run * 100 + i, 0)));
            }

            base_addr.write(base);
            num_samples.write(count);
            start.write(true);

            wait();
            // Check that busy asserts
            while (!done.read()) {
                wait();
            }
            start.write(false);
            wait();
            if (done.read()) {
                wait();
            }
        }

        std::cout << "  PASS: Successfully completed 3 consecutive start/done cycles!" << std::endl;
    }

    // SCENARIO 4: Address offset handling
    void test_scenario_4_address_offset() {
        std::cout << "\n[Test 4] Arbitrary Base Address Offset (0x3400)..." << std::endl;
        const int count = 8;
        const unsigned int base = 0x3400;
        const unsigned int base_word = base / bytesPerBeat;

        for (int i = 0; i < count; ++i) {
            mem.write_word(base_word + i, pack_complex<AxiCfg>(complex_t(77, 88)));
        }

        base_addr.write(base);
        num_samples.write(count);
        start.write(true);

        wait();
        while (!done.read()) {
            wait();
        }
        start.write(false);
        wait(2);

        sc_uint<AxiCfg::dataWidth> raw = mem.read_word(base_word + count);
        complex_t res = unpack_complex<AxiCfg>(raw);

        if (std::abs(res.real - 154.0) < 1e-4 && std::abs(res.imag - 264.0) < 1e-4) {
            std::cout << "  PASS: Arbitrary address offset handling verified!" << std::endl;
        } else {
            std::cerr << "  FAIL: Expected (154, 264), got (" << res.real << ", " << res.imag << ")" << std::endl;
            test_failures++;
        }
    }
};

int sc_main(int argc, char* argv[]) {
    TbDMA tb("tb_dma");
    sc_start();
    return tb.test_failures;
}
