#pragma once

#include <cstdint>

namespace atlas_mission_manager {

enum class MissionState : std::uint8_t {
  Bootstrap = 0,
  WaitMcuStatus = 1,
  WaitStart = 2,
  Precheck = 3,
  Initializing = 4,
  Running = 5,
  Aborting = 6,
  ReportingDone = 7,
  WaitMcuFinished = 8,
  ReportingFail = 9,
  WaitMcuFault = 10,
  WaitReset = 11,
  RecoveryRequired = 12,
  ShuttingDown = 13,
};

const char* mission_state_name(MissionState state) noexcept;

}  // namespace atlas_mission_manager
