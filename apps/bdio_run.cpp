// Runs one mission and prints its metrics. Optionally writes the per-tick
// flight recorder and the discrete event log for replay and plotting.
//
//   bdio_run --scenario combined --seed 42 --controller hardened --csv run.csv --events events.csv

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "bdio/harness.hpp"

namespace {

void usage() {
    std::fprintf(stderr,
                 "usage: bdio_run [--scenario nominal|sensor|power|comms|seu|combined] [--seed N]\n"
                 "                [--controller naive|hardened] [--ticks N] [--csv FILE] [--events FILE]\n");
}

}  // namespace

int main(int argc, char** argv) {
    bdio::RunOptions opts;
    const char* csv_path = nullptr;
    const char* events_path = nullptr;

    for (int i = 1; i < argc; i+=2) {
        const std::string arg = argv[i];
        const char* val = i + 1 < argc ? argv[i + 1] : nullptr;
        if (val == nullptr) {
            usage();
            return 2;
        }
        if (arg == "--scenario") {
            if (!bdio::parse_scenario(val, opts.scenario)) {
                std::fprintf(stderr, "unknown scenario: %s\n", val);
                return 2;
            }
        } else if (arg == "--seed") {
            opts.seed = std::strtoull(val, nullptr, 10);
        } else if (arg == "--controller") {
            if (std::strcmp(val, "naive") == 0) {
                opts.kind = bdio::ControllerKind::Naive;
            } else if (std::strcmp(val, "hardened") == 0) {
                opts.kind = bdio::ControllerKind::Hardened;
            } else {
                usage();
                return 2;
            }
        } else if (arg == "--ticks") {
            opts.ticks = static_cast<uint32_t>(std::strtoul(val, nullptr, 10));
        } else if (arg == "--csv") {
            csv_path = val;
        } else if (arg == "--events") {
            events_path = val;
        } else {
            usage();
            return 2;
        }
    }

    if (csv_path != nullptr && (opts.csv = std::fopen(csv_path, "w")) == nullptr) {
        std::perror(csv_path);
        return 1;
    }
    if (events_path != nullptr && (opts.events = std::fopen(events_path, "w")) == nullptr) {
        std::perror(events_path);
        return 1;
    }

    const bdio::RunMetrics m = bdio::run_mission(opts);
    std::printf("%s\n", bdio::to_json(m).c_str());

    if (opts.csv != nullptr) {
        std::fclose(opts.csv);
    }
    if (opts.events != nullptr) {
        std::fclose(opts.events);
    }
    return 0;
}
