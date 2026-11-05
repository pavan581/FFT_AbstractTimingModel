# FFT Abstract Timing Model

A high-level, timing-accurate SystemC/MatchLib model for Fast Fourier Transform (FFT) processing with AXI4 memory interfaces and multi-core interleaved scheduling.

---

## 1. Overview & Motivation

In system-level timing analysis and architectural exploration, modeling the exact low-level butterfly arithmetic logic inside each FFT pipeline stage introduces unnecessary simulation overhead and limits architectural agility. 

This project provides an **Abstract Timing Model** for the FFT processing subsystem where:
- The AXI4 bus interfaces, DMA controller state machines, MatchLib Channels, and multi-core interleaving are preserved.
- The FFT core uses internal block buffering and standard FFT algorithms to produce mathematically accurate frequency spectra.
- Precise, configurable timing behavior is modeled using two key variables:
  - **`FIRST_SAMPLE_LATENCY` ($T_1$)**: Clock cycles elapsed from the moment all $N$ input samples of a block have arrived into the input buffer until the first output sample is produced and available.
  - **`INTER_SAMPLE_GAP` ($T_2$)**: Clock cycles inserted as a gap/delay between consecutive output samples during the output streaming phase.

---

## 2. Timing Variables ($T_1$, $T_2$)

| Variable | Suitable Name | Description |
|---|---|---|
| **$T_1$** | `FIRST_SAMPLE_LATENCY` | Execution latency / delay in clock cycles after receiving all $N$ inputs before sample 0 is pushed. |
| **$T_2$** | `INTER_SAMPLE_GAP` | Inter-sample interval / delay in clock cycles between subsequent samples ($k$ and $k+1$). |

Both variables can be set:
1. Statically via C++ template parameters:
   ```cpp
   FFTTimed<N_SIZE, FIRST_SAMPLE_LATENCY, INTER_SAMPLE_GAP> fft;
   ```
2. Dynamically at runtime:
   ```cpp
   fft.set_timing_parameters(t1, t2);
   ```

---

## 3. Architecture & Hierarchy

```
+-----------------------------------------------------------------------------------------+
|                                    Top (top.h)                                          |
|                                                                                         |
|   +-----------------------+     +-----------------------+     +---------------------+   |
|   |     Core 0 (core.h)   |     |     Core 1 (core.h)   |     |   Core k (core.h)   |   |
|   | +-------------------+ |     | +-------------------+ |     | +-----------------+ |   |
|   | | DMA Controller    | |     | | DMA Controller    | |     | | DMA Controller  | |   |
|   | +---------|---------+ |     | +---------|---------+ |     | +--------|--------+ |   |
|   |           |           |     |           |           |     |          |          |   |
|   |   (Connections Chan)  |     |   (Connections Chan)  |     |  (Connections Chan) |   |
|   |           |           |     |           |           |     |          |          |   |
|   | +---------v---------+ |     | +---------v---------+ |     | +--------v--------+ |   |
|   | | FFTTimed (fft.h)  | |     | | FFTTimed (fft.h)  | |     | | FFTTimed (fft.h)| |   |
|   | | (T1, T2 timing)   | |     | | (T1, T2 timing)   | |     | | (T1, T2 timing) | |   |
|   | +-------------------+ |     | +-------------------+ |     | +-----------------+ |   |
|   +-----------|-----------+     +-----------|-----------+     +----------|----------+   |
|               |                             |                            |              |
|        (AXI4 Master)                 (AXI4 Master)                (AXI4 Master)         |
+---------------|-----------------------------|----------------------------|--------------+
                |                             |                            |
         +------v-----+                +------v-----+               +------v-----+
         |  Memory 0  |                |  Memory 1  |               |  Memory k  |
         +------------+                +------------+               +------------+
```

### Modules
1. **`FFTTimed` (`src/fft_timed.h`)**:
   - Internal input and output sample storage (`in_buffer`, `out_buffer`) allocated as class member variables at module construction time, accurately modeling hardware SRAM/register storage.
   - Ingests $N$ samples from upstream via `Connections::In<complex_t>`.
   - Computes canonical Discrete Fourier Transform (DFT) using standard C++ `std::complex` arithmetic in zero simulation time (abstract behavioral execution).
   - Enforces exact $T_1$ (`FIRST_SAMPLE_LATENCY`) clock cycles of pipeline latency from the last input sample to the first output sample.
   - Streams subsequent output samples with an exact gap of $T_2$ (`INTER_SAMPLE_GAP`) clock cycles.
2. **`DMA` (`src/dma.h`)**:
   - Manages AXI4 master read (`ar`, `r`) and write (`aw`, `w`, `b`) ports.
   - Supports multi-burst transfers ($> 256$ beats per transaction).
   - Coordinates `IDLE -> RUN -> FINISHED -> IDLE` state machine.
3. **`Core` (`src/core.h`)**:
   - Ties together DMA and FFTTimed using MatchLib `Combinational<complex_t>` channels.
4. **`Top` (`src/top.h`)**:
   - Coordinates multiple cores with `HOP_SIZE` staggered starts to optimize memory bus bandwidth.
5. **`Memory` (`src/memory.h`)**:
   - AXI4 slave dual-port SRAM model for verification.

---

## 4. Test Suites & Scenarios

Comprehensive testbenches are provided for every module:
- **`test/tb_fft.cpp`**: Unit tests for FFTTimed:
  - Scenario 1: Timing accuracy measurement ($T_1$, $T_2$).
  - Scenario 2: Zero-gap back-to-back streaming ($T_2 = 0$).
  - Scenario 3: Zero initial latency ($T_1 = 0$).
  - Scenario 4: Multi-frame consecutive processing.
  - Scenario 5: Consumer backpressure handling.
  - Scenario 6: Mathematical verification (Impulse, DC, Sine).
- **`test/tb_dma.cpp`**: Unit tests for DMA:
  - Scenario 1: Basic block transfer and loopback.
  - Scenario 2: Multi-burst transfers ($> 256$ beats).
  - Scenario 3: Handshake re-triggering across multiple runs.
  - Scenario 4: Arbitrary base address offsets.
- **`test/tb_core.cpp`**: Integrated single-core tests:
  - Scenario 1: End-to-end memory-to-memory processing.
  - Scenario 2: Multi-block continuous processing (e.g. 64 samples).
  - Scenario 3: Dynamic timing parameter variation ($T_1, T_2$).
  - Scenario 4: Frequency spectrum verification against tone.
- **`test/tb_top.cpp`**: Top-level multi-core tests:
  - Scenario 1: Staggered dispatch with `HOP_SIZE`.
  - Scenario 2: Distributed multi-block workload across all cores.
  - Scenario 3: Dynamic timing reconfiguration.

---

## 5. Building and Running

Run all tests inside the `eloquent_mayer` container:

```bash
# Build all targets
make all

# Run individual testbenches
make test_fft
make test_dma
make test_core
make test_top

# Run all testbenches
make run_all

# Generate analysis plots from simulation CSV traces
make plot
```

---

## 6. Waveform Tracing & Data Plotting

### VCD Waveform Tracing
Simulations automatically generate cycle-accurate SystemC VCD traces in `out/vcd/`:
- `out/vcd/fft_unit.vcd`: Clock, reset, and cycle counters for standalone FFT module.
- `out/vcd/core_unit.vcd`: Memory handshake, DMA `start`, `busy`, `done`, and address signals.
- `out/vcd/top_system.vcd`: Stagger logic, per-core starts, busy, and multi-core done flags.

Open these traces in GTKWave or any standard waveform viewer:
```bash
gtkwave out/vcd/core_unit.vcd
```

### CSV Data Logging
Input and output complex samples are automatically logged to `out/data/*.csv`:
- Columns: `sample_index,time_ns,real,imag,magnitude,phase_rad`
- Files generated:
  - `out/data/fft_unit_input.csv` & `out/data/fft_unit_output.csv`
  - `out/data/core_unit_input.csv` & `out/data/core_unit_output.csv`
  - `out/data/top_core_0_input.csv` & `out/data/top_core_0_output.csv`
  - `out/data/top_core_1_input.csv` & `out/data/top_core_1_output.csv`

### Plotting Script (`plot_fft_data.py`)
A Python visualization utility is provided to plot time-domain inputs, frequency-domain output spectra, phase response, and timing profiles:
```bash
python3 plot_fft_data.py --prefix core_unit
python3 plot_fft_data.py --prefix fft_unit
python3 plot_fft_data.py --prefix top_core_0
```
Generated high-resolution plot images are saved to `out/plots/*.png`.

