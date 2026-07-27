#pragma once

#include <cstdint>

namespace atlas_mission_manager {

enum class MissionEvent : std::uint8_t {
  None = 0,
  McuStatusUpdated,
  AutoTaskStart,
  AutoTaskReset,
  CommonPrecheckPassed,
  CommonPrecheckFailed,
  InitializationCompleted,
  TaskFlowSucceeded,
  TaskFlowFailed,
  TaskFlowCancelled,
  ReportRequestAccepted,
  ReportRequestFailed,
  McuFinishedConfirmed,
  McuFaultConfirmed,
  McuLeftAutoPi,
  McuEStopDetected,
  McuStatusTimeout,
  ShutdownRequested,
};

}  // namespace atlas_mission_manager
