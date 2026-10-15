// Copyright 2026 ROBOTIS CO., LTD.
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

#pragma once

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_EXPORT __attribute__((dllexport))
    #define A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_IMPORT __attribute__((dllimport))
  #else
    #define A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_EXPORT __declspec(dllexport)
    #define A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_IMPORT __declspec(dllimport)
  #endif
  #ifdef A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_BUILDING_DLL
    #define A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC \
      A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_EXPORT
  #else
    #define A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC \
      A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_IMPORT
  #endif
#else
  #define A1_JOINT_TRAJECTORY_COMMAND_BROADCASTER_PUBLIC __attribute__((visibility("default")))
#endif
