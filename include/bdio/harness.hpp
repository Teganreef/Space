#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <string>

#include "bdio/faults.hpp"
#include "bdio/types.hpp"

namespace bdio {

enum class ControllerKind : uint8_t { Naive, Hardened };
const char* to_string(ControllerKind k);

// Safety invariants checked by the harness against ground truth, independent
// of anything the controller believes about itself. Counted as episodes: a
// violation that clears and returns within a minute is the same episode.
enum Violation : uint8_t {
    kDeepDischarge,    // true state of charge below 10%
    kPayloadLowPower,  // payload running while true state of charge below 20%
    kInvalidMode,      // mode variable holds a value that is not a mode
    kPayloadInSafe,    // payload running while in safe mode
    kDuplicateExec,    // the same command sequence number executed twice
    kStaleExec,        // a command older than the age limit executed
    kViolationCount,
};
const char* violation_name(int v);

struct RunMetrics {
    Scenario scenario = Scenario::Nominal;
    uint64_t seed = 0;
    ControllerKind kind = ControllerKind::Naive;
    uint32_t ticks = 0;

    std::array<uint32_t, kViolationCount> violations{};
    uint32_t total_violations = 0;

    uint32_t faults_scored = 0;
    uint32_t faults_detected = 0;
    uint64_t latency_sum = 0;
    uint32_t latency_max = 0;

    // Per fault kind: [scored, detected, latency sum].
    std::array<std::array<uint32_t, 3>, static_cast<std::size_t>(FaultKind::Count)> by_kind{};

    uint32_t safe_entries = 0;
    uint32_t false_safe_entries = 0;
    uint32_t recoveries = 0;
    uint64_t recovery_ticks_sum = 0;

    uint32_t planned_science_ticks = 0;
    uint32_t science_ticks = 0;
    std::array<uint32_t, 6> mode_ticks{};

    double min_soc = 1.0;
    double final_soc = 0.0;
};

struct RunOptions {
    Scenario scenario = Scenario::Nominal;
    uint64_t seed = 1;
    ControllerKind kind = ControllerKind::Hardened;
    uint32_t ticks = kMissionTicks;
    std::FILE* csv = nullptr;     // per-tick flight recorder
    std::FILE* events = nullptr;  // discrete events: faults, detections, mode changes
};

RunMetrics run_mission(const RunOptions& opts);

std::string to_json(const RunMetrics& m);

}  // namespace bdio
