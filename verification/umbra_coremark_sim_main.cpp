#include "Vumbra_coremark_sim_top.h"
#include "verilated.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

static std::uint64_t parse_max_cycles(int argc, char** argv) {
    constexpr const char* prefix = "+MAX_CYCLES=";
    for (int i = 1; i < argc; ++i) {
        if (std::strncmp(argv[i], prefix, std::strlen(prefix)) == 0) {
            char* end = nullptr;
            const auto value = std::strtoull(argv[i] + std::strlen(prefix), &end, 10);
            if (end != nullptr && *end == '\0' && value != 0) return value;
        }
    }
    return 100000000ULL;
}

int main(int argc, char** argv) {
    VerilatedContext context;
    context.commandArgs(argc, argv);
    Vumbra_coremark_sim_top top{&context};
    const std::uint64_t max_cycles = parse_max_cycles(argc, argv);

    auto clock_cycle = [&](bool reset_n) {
        top.rst_n = reset_n;
        top.clk = 0;
        top.eval();
        context.timeInc(1);
        top.clk = 1;
        top.eval();
        context.timeInc(1);
    };

    for (int i = 0; i < 4; ++i) clock_cycle(false);

    for (std::uint64_t cycle = 0; cycle < max_cycles; ++cycle) {
        clock_cycle(true);

        if (top.uart_valid) {
            std::cout.put(static_cast<char>(top.uart_char));
        }
        if (top.done_valid) {
            std::cout.flush();
            if (top.done_value != 1U) {
                std::cerr << "COREMARK_TARGET_BAD_DONE value=0x" << std::hex
                          << top.done_value << std::dec
                          << " cycles=" << top.cycle_count_o << '\n';
                top.final();
                return 1;
            }
            std::cout << "COREMARK_TARGET_DONE cycles=" << top.cycle_count_o
                      << " commits=" << top.commit_count_o << '\n';
            top.final();
            return 0;
        }
        if (context.gotFinish()) {
            std::cerr << "COREMARK_TARGET_UNEXPECTED_FINISH cycles="
                      << top.cycle_count_o << '\n';
            top.final();
            return 1;
        }
    }

    std::cerr << "COREMARK_WATCHDOG max_cycles=" << max_cycles
              << " commits=" << top.commit_count_o << '\n';
    top.final();
    return 124;
}
