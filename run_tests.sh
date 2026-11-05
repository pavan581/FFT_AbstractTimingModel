#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================================="
echo "    STARTING FULL VERIFICATION FOR FFT ABSTRACT MODEL     "
echo "=========================================================="

FAILED=0

run_suite() {
    local target="$1"
    local name="$2"
    echo ""
    echo ">>> Running $name ($target)..."
    if "$target"; then
        echo ">>> [PASS] $name"
    else
        echo ">>> [FAIL] $name"
        FAILED=$((FAILED + 1))
    fi
}

run_suite "./bin/test_fft" "FFT Timed Unit Tests"
run_suite "./bin/test_dma" "DMA Controller Tests"
run_suite "./bin/test_core" "Integrated Core Tests"
run_suite "./bin/test_top" "Multi-Core Interleaved Top Tests"

echo ""
echo "=========================================================="
if [ $FAILED -eq 0 ]; then
    echo "  ALL VERIFICATION SUITES PASSED CLEANLY (0 failures)!"
    echo "=========================================================="
    exit 0
else
    echo "  VERIFICATION FAILED: $FAILED suite(s) reported errors!"
    echo "=========================================================="
    exit 1
fi

