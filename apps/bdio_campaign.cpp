// Fault campaign: runs N missions per scenario against both controllers with
// identical seeds and writes one JSON line per mission.
//
//   bdio_campaign --runs 1000 --out results/campaign.jsonl
//   bdio_campaign --runs 200 --scenario combined --seed-base 5000

#include <cstdio>
#include <cstdlib>
#include <string>

#include "bdio/harness.hpp"

int main(int argc, char** argv) {
    uint32_t runs = 200;
    uint64_t seed_base = 1;
    std::string only;
    const char* out_path = "campaign.jsonl";

    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string arg = argv[i];
        const char* val = argv[i + 1];
        if (arg == "--runs") {
            runs = static_cast<uint32_t>(std::strtoul(val, nullptr, 10));
        } else if (arg == "--seed-base") {
            seed_base = std::strtoull(val, nullptr, 10);
        } else if (arg == "--scenario") {
            only = val;
        } else if (arg == "--out") {
            out_path = val;
        } else {
            std::fprintf(stderr, "usage: bdio_campaign [--runs N] [--seed-base N] [--scenario NAME] [--out FILE]\n");
            return 2;
        }
    }

    std::FILE* out = std::fopen(out_path, "w");
    if (out == nullptr) {
        std::perror(out_path);
        return 1;
    }

    std::printf("%-9s %-9s %10s %12s %10s %10s %9s\n", "scenario", "ctrl", "violations", "runs_w_viol",
                "detected", "false_safe", "science");
    for (uint8_t si = 0; si < static_cast<uint8_t>(bdio::Scenario::Count); ++si) {
        const auto scenario = static_cast<bdio::Scenario>(si);
        if (!only.empty() && only != bdio::to_string(scenario)) {
            continue;
        }
        for (const auto kind : {bdio::ControllerKind::Naive, bdio::ControllerKind::Hardened}) {
            uint64_t violations = 0;
            uint64_t runs_with_violation = 0;
            uint64_t scored = 0;
            uint64_t detected = 0;
            uint64_t false_safe = 0;
            uint64_t planned = 0;
            uint64_t science = 0;
            for (uint32_t r = 0; r < runs; ++r) {
                bdio::RunOptions opts;
                opts.scenario = scenario;
                opts.kind = kind;
                opts.seed = seed_base + r;
                const bdio::RunMetrics m = bdio::run_mission(opts);
                std::fprintf(out, "%s\n", bdio::to_json(m).c_str());
                violations += m.total_violations;
                runs_with_violation += m.total_violations > 0 ? 1 : 0;
                scored += m.faults_scored;
                detected += m.faults_detected;
                false_safe += m.false_safe_entries;
                planned += m.planned_science_ticks;
                science += m.science_ticks;
            }
            const double det_pct = scored > 0 ? 100.0 * static_cast<double>(detected) / static_cast<double>(scored) : 0.0;
            const double sci_pct = planned > 0 ? 100.0 * static_cast<double>(science) / static_cast<double>(planned) : 0.0;
            std::printf("%-9s %-9s %10llu %11.1f%% %9.1f%% %10llu %8.1f%%\n", bdio::to_string(scenario),
                        bdio::to_string(kind), static_cast<unsigned long long>(violations),
                        100.0 * static_cast<double>(runs_with_violation) / runs, det_pct,
                        static_cast<unsigned long long>(false_safe), sci_pct);
            std::fflush(stdout);
        }
    }
    std::fclose(out);
    return 0;
}
