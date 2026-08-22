// Copyright 2026 yangxuan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ATLAS_MISSION_YASMIN__STATES_HPP_
#define ATLAS_MISSION_YASMIN__STATES_HPP_

#include <memory>
#include <string>

#include "atlas_mission_yasmin/runtime.hpp"
#include "yasmin/state.hpp"

namespace atlas_mission_yasmin
{

namespace outcomes
{

inline constexpr char kOk[] = "ok";
inline constexpr char kRetry[] = "retry";
inline constexpr char kFailed[] = "failed";
inline constexpr char kReset[] = "reset";
inline constexpr char kRecovery[] = "recovery";
inline constexpr char kNext[] = "next";
inline constexpr char kRouteDone[] = "route_done";
inline constexpr char kShutdown[] = "shutdown";

}  // namespace outcomes

class RuntimeState : public yasmin::State
{
public:
  RuntimeState(Runtime::SharedPtr runtime, yasmin::Outcomes outcomes);

protected:
  Runtime::SharedPtr runtime_;
};

class BootstrapState final : public RuntimeState
{
public:
  explicit BootstrapState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class WaitMcuState final : public RuntimeState
{
public:
  explicit WaitMcuState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class WaitStartState final : public RuntimeState
{
public:
  explicit WaitStartState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class PrecheckState final : public RuntimeState
{
public:
  explicit PrecheckState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class StartRunState final : public RuntimeState
{
public:
  explicit StartRunState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class ReportDoneState final : public RuntimeState
{
public:
  explicit ReportDoneState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class ReportFailState final : public RuntimeState
{
public:
  explicit ReportFailState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class RecoveryState final : public RuntimeState
{
public:
  explicit RecoveryState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class WaitResetState final : public RuntimeState
{
public:
  explicit WaitResetState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class PrepareWaypointState final : public RuntimeState
{
public:
  explicit PrepareWaypointState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class PreMoveState final : public RuntimeState
{
public:
  explicit PreMoveState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class NavigateState final : public RuntimeState
{
public:
  explicit NavigateState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class RunJobsState final : public RuntimeState
{
public:
  explicit RunJobsState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

class AdvanceState final : public RuntimeState
{
public:
  explicit AdvanceState(Runtime::SharedPtr runtime);
  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override;
};

}  // namespace atlas_mission_yasmin

#endif  // ATLAS_MISSION_YASMIN__STATES_HPP_
