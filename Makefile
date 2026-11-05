SHELL := /bin/bash

CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2

# Dependency paths (defaults match eloquent_mayer container environment)
SYSTEMC_HOME ?= /opt/systemc
CONNECTIONS_HOME ?= /workspace/library/matchlib_connections-master
MATCHLIB_HOME ?= /workspace/library/matchlib-main
BOOST_HOME ?= /workspace/library/Boost-Preprocessor-master
RAPIDJSON_HOME ?= /workspace/library/rapidjson-master
AC_TYPES ?= /workspace/library/ac_types-master
AC_SIMUTILS ?= /workspace/library/ac_simutils-master
MATCHLIB_EXAMPLES ?= /workspace/library/matchlib_examples
MEMORY ?= /workspace/library/FI_mem

SYSTEMC_LIB ?= $(SYSTEMC_HOME)/lib-linux64

SRC_DIR = src
TEST_DIR = test
BIN_DIR = bin
BUILD_DIR = build
OUT_DIR = out

INCDIRS = -I. -I$(SRC_DIR) -I$(TEST_DIR)
INCDIRS += -I$(SYSTEMC_HOME)/include -I$(SYSTEMC_HOME)/src
INCDIRS += -I$(CONNECTIONS_HOME)/include
INCDIRS += -I$(MATCHLIB_HOME)/cmod/include
INCDIRS += -I$(BOOST_HOME)/include
INCDIRS += -I$(RAPIDJSON_HOME)/include
INCDIRS += -I$(AC_TYPES)/include
INCDIRS += -I$(AC_SIMUTILS)/include
INCDIRS += -I$(MATCHLIB_EXAMPLES)/include
INCDIRS += -I$(MEMORY)

ALL_CXXFLAGS = $(CXXFLAGS) $(INCDIRS) -DSC_ALLOW_DEPRECATED_IEEE_API -DSC_INCLUDE_DYNAMIC_PROCESSES -DBOOST_NULLPTR=nullptr
LDFLAGS = -L$(SYSTEMC_LIB) -Wl,-rpath,$(SYSTEMC_LIB) -lsystemc -lm -lpthread

TARGETS = $(BIN_DIR)/test_fft $(BIN_DIR)/test_dma $(BIN_DIR)/test_core $(BIN_DIR)/test_top

.PHONY: all clean run_all test_fft test_dma test_core test_top plot

all: $(TARGETS)

$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(OUT_DIR):
	mkdir -p $(OUT_DIR)/data $(OUT_DIR)/vcd $(OUT_DIR)/plots

$(BIN_DIR)/test_fft: $(TEST_DIR)/tb_fft.cpp $(SRC_DIR)/fft_timed.h $(SRC_DIR)/fft_types.h $(TEST_DIR)/tb_tracer.h | $(BIN_DIR) $(OUT_DIR)
	@echo "[CXX] Compiling and linking test_fft..."
	$(CXX) $(ALL_CXXFLAGS) $< $(LDFLAGS) -o $@

$(BIN_DIR)/test_dma: $(TEST_DIR)/tb_dma.cpp $(SRC_DIR)/dma.h $(SRC_DIR)/memory.h $(SRC_DIR)/fft_types.h | $(BIN_DIR) $(OUT_DIR)
	@echo "[CXX] Compiling and linking test_dma..."
	$(CXX) $(ALL_CXXFLAGS) $< $(LDFLAGS) -o $@

$(BIN_DIR)/test_core: $(TEST_DIR)/tb_core.cpp $(SRC_DIR)/core.h $(SRC_DIR)/dma.h $(SRC_DIR)/fft_timed.h $(SRC_DIR)/memory.h $(SRC_DIR)/fft_types.h $(TEST_DIR)/tb_tracer.h | $(BIN_DIR) $(OUT_DIR)
	@echo "[CXX] Compiling and linking test_core..."
	$(CXX) $(ALL_CXXFLAGS) $< $(LDFLAGS) -o $@

$(BIN_DIR)/test_top: $(TEST_DIR)/tb_top.cpp $(SRC_DIR)/top.h $(SRC_DIR)/core.h $(SRC_DIR)/dma.h $(SRC_DIR)/fft_timed.h $(SRC_DIR)/memory.h $(SRC_DIR)/fft_types.h $(TEST_DIR)/tb_tracer.h | $(BIN_DIR) $(OUT_DIR)
	@echo "[CXX] Compiling and linking test_top..."
	$(CXX) $(ALL_CXXFLAGS) $< $(LDFLAGS) -o $@

test_fft: $(BIN_DIR)/test_fft | $(OUT_DIR)
	@echo "Running FFTTimed Testbench..."
	@$<

test_dma: $(BIN_DIR)/test_dma | $(OUT_DIR)
	@echo "Running DMA Controller Testbench..."
	@$<

test_core: $(BIN_DIR)/test_core | $(OUT_DIR)
	@echo "Running Core Testbench..."
	@$<

test_top: $(BIN_DIR)/test_top | $(OUT_DIR)
	@echo "Running Top System Testbench..."
	@$<

plot:
	@echo "Generating analysis plots from simulation CSV traces..."
	@python3 plot_fft_data.py --prefix core_unit
	@python3 plot_fft_data.py --prefix fft_unit
	@python3 plot_fft_data.py --prefix top_core_0

run_all: $(TARGETS) | $(OUT_DIR)
	@echo ""
	@echo "=========================================================="
	@echo "  EXECUTING ALL TEST SUITES FOR FFT ABSTRACT MODEL"
	@echo "=========================================================="
	@$(BIN_DIR)/test_fft
	@$(BIN_DIR)/test_dma
	@$(BIN_DIR)/test_core
	@$(BIN_DIR)/test_top
	@echo "=========================================================="
	@echo "  ALL TEST SUITES EXECUTED SUCCESSFULLY!"
	@echo "=========================================================="

clean:
	rm -rf $(BIN_DIR) $(BUILD_DIR)
