#pragma once

#include <cstdint>

namespace atlas_mission_manager::error {

constexpr std::int32_t kNone = 0;
constexpr std::int32_t kIllegalTransition = 1001;
constexpr std::int32_t kDuplicateStartConflict = 1002;
constexpr std::int32_t kDryRunFailed = 1003;
constexpr std::int32_t kInitializationFailed = 1004;

constexpr std::int32_t kMcuStatusUnavailable = 1401;
constexpr std::int32_t kMcuStatusTimeout = 1402;
constexpr std::int32_t kStartConfirmationTimeout = 1403;
constexpr std::int32_t kMcuNotAutoPi = 1404;
constexpr std::int32_t kAutoStartLatchMismatch = 1405;
constexpr std::int32_t kPiOffline = 1406;
constexpr std::int32_t kChassisNotReady = 1407;
constexpr std::int32_t kOdomNotReady = 1408;
constexpr std::int32_t kArmNotReady = 1409;
constexpr std::int32_t kResultServiceUnavailable = 1410;
constexpr std::int32_t kResultServiceRejected = 1411;
constexpr std::int32_t kDoneConfirmationTimeout = 1412;
constexpr std::int32_t kFailConfirmationTimeout = 1413;
constexpr std::int32_t kMissingRunContext = 1414;

constexpr std::int32_t kRouteConfigError = 1501;
constexpr std::int32_t kTaskFlowUnavailable = 1502;
constexpr std::int32_t kArmCommandRejected = 1503;
constexpr std::int32_t kArmMotionTimeout = 1504;
constexpr std::int32_t kVisionTargetUnavailable = 1505;
constexpr std::int32_t kCoordinateTransformFailed = 1506;

}  // namespace atlas_mission_manager::error
