#!/usr/bin/env python3
"""
plot_fft_data.py

Visualization utility for FFT Abstract Model simulation outputs.
Reads CSV data files generated during SystemC simulation and plots:
  1. Time-Domain Input Signal (Real & Imaginary components with magnitude envelope)
  2. Frequency-Domain Magnitude Spectrum (Linear scale & Logarithmic dBFS scale)
  3. Frequency-Domain Phase Angle (in radians with +/- pi reference lines)
  4. Timing & Latency Profile (Visualizing exact T1 first-sample delay & T2 inter-sample gaps)

Usage:
  python3 plot_fft_data.py [--prefix PREFIX] [--data-dir DIR] [--out-dir DIR] [--show]
Example:
  python3 plot_fft_data.py --prefix fft_unit
  python3 plot_fft_data.py --prefix core_unit
  python3 plot_fft_data.py --prefix top_core_0
"""

import os
import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg') # Headless Linux environment
import matplotlib.pyplot as plt

def parse_args():
    parser = argparse.ArgumentParser(description="Plot FFT simulation results with detailed timing and frequency analysis.")
    parser.add_argument("--prefix", type=str, default=None,
                        help="Prefix of the CSV files (e.g. 'fft_unit', 'core_unit', 'top_core_0'). If omitted, plots all found.")
    parser.add_argument("--data-dir", type=str, default="./out/data",
                        help="Directory containing CSV trace files (default: ./out/data)")
    parser.add_argument("--out-dir", type=str, default="./out/plots",
                        help="Directory to save generated plots (default: ./out/plots)")
    parser.add_argument("--show", action="store_true",
                        help="Display interactive plot window (requires GUI display)")
    return parser.parse_args()

def load_csv(filepath):
    if not os.path.isfile(filepath):
        return None
    try:
        data = np.genfromtxt(filepath, delimiter=',', names=True, dtype=None, encoding='utf-8')
        if data.size == 0:
            return None
        # Handle 1-row edge case
        if data.ndim == 0:
            data = np.array([data])
        return data
    except Exception as e:
        print(f"[WARN] Failed to load {filepath}: {e}")
        return None

def plot_dataset(prefix, data_dir, out_dir, show=False):
    in_file = os.path.join(data_dir, f"{prefix}_input.csv")
    out_file = os.path.join(data_dir, f"{prefix}_output.csv")

    in_data = load_csv(in_file)
    out_data = load_csv(out_file)

    if in_data is None and out_data is None:
        print(f"[SKIP] No data files found for prefix: {prefix}")
        return

    print(f"[INFO] Generating plots for [{prefix}]...")

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.patch.set_facecolor('#f8f9fa')
    fig.suptitle(f"FFT Abstract Model Analysis: {prefix}", fontsize=16, fontweight='bold', color='#212529', y=0.98)

    # -------------------------------------------------------------
    # 1. Time-Domain Input Signal
    # -------------------------------------------------------------
    ax1 = axes[0, 0]
    ax1.set_facecolor('#ffffff')
    if in_data is not None and len(in_data) > 0:
        idx = in_data['sample_index']
        real = in_data['real']
        imag = in_data['imag']
        mag = in_data['magnitude']

        ax1.plot(idx, real, 'o-', color='#1f77b4', linewidth=1.8, markersize=5, label='Real Re{x[n]}')
        ax1.plot(idx, imag, 's--', color='#ff7f0e', linewidth=1.5, markersize=4, label='Imag Im{x[n]}')
        ax1.fill_between(idx, -mag, mag, color='#ced4da', alpha=0.3, label='Magnitude Envelope')

        ax1.set_title("Input Signal (Time Domain)", fontsize=12, fontweight='semibold')
        ax1.set_xlabel("Sample Index (n)", fontsize=10)
        ax1.set_ylabel("Amplitude", fontsize=10)
        ax1.grid(True, linestyle=':', alpha=0.7)
        ax1.legend(loc='upper right', framealpha=0.9)
    else:
        ax1.text(0.5, 0.5, "Input data not available", ha='center', va='center', color='gray')

    # -------------------------------------------------------------
    # 2. Frequency-Domain Magnitude Spectrum (Linear & dB)
    # -------------------------------------------------------------
    ax2 = axes[0, 1]
    ax2.set_facecolor('#ffffff')
    if out_data is not None and len(out_data) > 0:
        # If multiple frames exist, take the first frame for clean spectrum viewing
        max_idx = out_data['sample_index'].max()
        frame_len = len(out_data) if max_idx < 16 else 16
        subset = out_data[:frame_len]

        bins = np.arange(len(subset))
        mag = subset['magnitude']

        # Dual axis: left is linear magnitude, right is dB scale
        markerline, stemlines, baseline = ax2.stem(bins, mag, linefmt='#2ca02c', markerfmt='o', basefmt='k-')
        plt.setp(stemlines, 'linewidth', 2)
        plt.setp(markerline, 'markersize', 6, 'color', '#2ca02c')

        ax2.set_title("Output Frequency Spectrum (|X[k]|)", fontsize=12, fontweight='semibold')
        ax2.set_xlabel("FFT Bin Index (k)", fontsize=10)
        ax2.set_ylabel("Linear Magnitude", fontsize=10, color='#2ca02c')
        ax2.tick_params(axis='y', labelcolor='#2ca02c')
        ax2.grid(True, linestyle=':', alpha=0.7)

        # Secondary y-axis for dBFS
        max_mag = np.max(mag) if np.max(mag) > 0 else 1.0
        mag_db = 20 * np.log10(np.maximum(mag / max_mag, 1e-6))
        ax2_db = ax2.twinx()
        ax2_db.plot(bins, mag_db, 'r:', alpha=0.5, label='Normalized dB')
        ax2_db.set_ylabel("Normalized Magnitude (dB)", fontsize=10, color='#d62728')
        ax2_db.tick_params(axis='y', labelcolor='#d62728')
        ax2_db.set_ylim(-60, 5)
    else:
        ax2.text(0.5, 0.5, "Output spectrum not available", ha='center', va='center', color='gray')

    # -------------------------------------------------------------
    # 3. Frequency-Domain Phase Angle
    # -------------------------------------------------------------
    ax3 = axes[1, 0]
    ax3.set_facecolor('#ffffff')
    if out_data is not None and len(out_data) > 0:
        frame_len = len(out_data) if out_data['sample_index'].max() < 16 else 16
        subset = out_data[:frame_len]
        bins = np.arange(len(subset))
        phase = subset['phase_rad']

        markerline, stemlines, baseline = ax3.stem(bins, phase, linefmt='#9467bd', markerfmt='s', basefmt='k-')
        plt.setp(stemlines, 'linewidth', 1.8)
        plt.setp(markerline, 'markersize', 5, 'color', '#9467bd')

        ax3.axhline(np.pi, color='gray', linestyle='--', alpha=0.4, label='+pi')
        ax3.axhline(-np.pi, color='gray', linestyle='--', alpha=0.4, label='-pi')
        ax3.axhline(0.0, color='gray', linestyle='-', alpha=0.3)

        ax3.set_title("Frequency Phase Response", fontsize=12, fontweight='semibold')
        ax3.set_xlabel("FFT Bin Index (k)", fontsize=10)
        ax3.set_ylabel("Phase Angle (radians)", fontsize=10)
        ax3.set_ylim(-np.pi * 1.15, np.pi * 1.15)
        ax3.grid(True, linestyle=':', alpha=0.7)
    else:
        ax3.text(0.5, 0.5, "Phase data not available", ha='center', va='center', color='gray')

    # -------------------------------------------------------------
    # 4. Timing & Latency Profile (T1 Delay & T2 Inter-sample Gap)
    # -------------------------------------------------------------
    ax4 = axes[1, 1]
    ax4.set_facecolor('#ffffff')
    if in_data is not None and out_data is not None and len(in_data) > 0 and len(out_data) > 0:
        n_show = min(len(in_data), len(out_data), 16)
        in_subset = in_data[:n_show]
        out_subset = out_data[:n_show]

        t_in = in_subset['time_ns']
        t_out = out_subset['time_ns']
        y_in = np.zeros(n_show)
        y_out = np.ones(n_show)

        # Plot arrival events on a 2-lane raster
        ax4.scatter(t_in, y_in, color='#1f77b4', s=70, zorder=3, label='Input Sample Arrived', marker='o')
        ax4.scatter(t_out, y_out, color='#d62728', s=70, zorder=3, label='Output Sample Emitted', marker='^')

        # Connect corresponding input-to-output lines or highlight T1
        t_last_in = t_in[-1]
        t_first_out = t_out[0]

        # Latency span annotation
        ax4.axvspan(t_last_in, t_first_out, color='#ffecb3', alpha=0.5, label='T1 Hardware Latency')
        ax4.annotate(f"T1 = {t_first_out - t_last_in:.1f} ns",
                     xy=((t_last_in + t_first_out) / 2.0, 0.5),
                     xytext=((t_last_in + t_first_out) / 2.0, 0.5),
                     ha='center', va='center',
                     bbox=dict(boxstyle="round,pad=0.3", fc="#ffe082", ec="#ffb300", lw=1.5),
                     fontsize=9, fontweight='bold')

        ax4.set_title("Hardware Timing Profile (T1 Latency & T2 Interval)", fontsize=12, fontweight='semibold')
        ax4.set_xlabel("Simulation Time (ns)", fontsize=10)
        ax4.set_yticks([0, 1])
        ax4.set_yticklabels(['Input Stream', 'Output Stream'], fontsize=10, fontweight='semibold')
        ax4.set_ylim(-0.3, 1.4)
        ax4.grid(True, linestyle=':', alpha=0.7)
        ax4.legend(loc='upper right', framealpha=0.9, fontsize=9)
    else:
        ax4.text(0.5, 0.5, "Timing comparison not available", ha='center', va='center', color='gray')

    plt.tight_layout(rect=[0.02, 0.03, 0.98, 0.95])
    out_path = os.path.join(out_dir, f"{prefix}_analysis.png")
    plt.savefig(out_path, dpi=200)
    plt.close(fig)
    print(f"[SUCCESS] High-resolution plot saved to: {out_path}")

    if show:
        plt.show()

def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    if args.prefix:
        plot_dataset(args.prefix, args.data_dir, args.out_dir, args.show)
    else:
        # Automatically discover all dataset prefixes in data_dir
        prefixes = set()
        if os.path.isdir(args.data_dir):
            for fname in os.listdir(args.data_dir):
                if fname.endswith("_input.csv"):
                    prefixes.add(fname[:-10])
                elif fname.endswith("_output.csv"):
                    prefixes.add(fname[:-11])

        if not prefixes:
            print(f"[WARN] No CSV data files found in {args.data_dir}")
            sys.exit(0)

        for p in sorted(prefixes):
            plot_dataset(p, args.data_dir, args.out_dir, args.show)

if __name__ == "__main__":
    main()
