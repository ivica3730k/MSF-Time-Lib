CXX      ?= c++
# -I. so `#include <Arduino.h>` inside the library resolves to the mock here.
CXXFLAGS ?= -std=c++14 -Wall -Wextra -O0 -g -I.
BIN      := run_tests

SOURCES := test_main.cpp Arduino.cpp

.PHONY: all test clean
all: $(BIN)

$(BIN): $(SOURCES) Arduino.h test_runner.h ../src/MSF-Time-Lib.h
	$(CXX) $(CXXFLAGS) $(SOURCES) -o $(BIN)

test: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
