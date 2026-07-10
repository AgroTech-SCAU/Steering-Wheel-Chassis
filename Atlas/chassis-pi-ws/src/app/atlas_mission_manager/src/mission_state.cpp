#include "atlas_mission_manager/mission_state.hpp"

namespace atlas_mission_manager {

const char* mission_state_name(const MissionState state) noexcept {
  switch (state) {
    case MissionState::Bootstrap:
      return "BOOTSTRAP";
    case MissionState::WaitMcuStatus:
      return "WAIT_MCU_STATUS";
    case MissionState::WaitStart:
      return "WAIT_START";
    case MissionState::Precheck:
      return "PRECHECK";
    case MissionState::Initializing:
      return "INITIALIZING";
    case MissionState::Running:
      return "RUNNING";
    case MissionState::Aborting:
      return "ABORTING";
    case MissionState::ReportingDone:
      return "REPORTING_DONE";
    case MissionState::WaitMcuFinished:
      return "WAIT_MCU_FINISHED";
    case MissionState::ReportingFail:
      return "REPORTING_FAIL";
    case MissionState::WaitMcuFault:
      return "WAIT_MCU_FAULT";
    case MissionState::WaitReset:
      return "WAIT_RESET";
    case MissionState::RecoveryRequired:
      return "RECOVERY_REQUIRED";
    case MissionState::ShuttingDown:
      return "SHUTTING_DOWN";
    default:
      return "UNKNOWN";
  }
}

}  // namespace atlas_mission_manager
