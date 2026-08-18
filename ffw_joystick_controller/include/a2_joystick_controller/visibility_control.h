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

#ifndef A2_JOYSTICK_CONTROLLER__VISIBILITY_CONTROL_H_
#define A2_JOYSTICK_CONTROLLER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define A2_JOYSTICK_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define A2_JOYSTICK_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define A2_JOYSTICK_CONTROLLER_EXPORT __declspec(dllexport)
    #define A2_JOYSTICK_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef A2_JOYSTICK_CONTROLLER_BUILDING_DLL
    #define A2_JOYSTICK_CONTROLLER_PUBLIC A2_JOYSTICK_CONTROLLER_EXPORT
  #else
    #define A2_JOYSTICK_CONTROLLER_PUBLIC A2_JOYSTICK_CONTROLLER_IMPORT
  #endif
  #define A2_JOYSTICK_CONTROLLER_LOCAL
#else
  #define A2_JOYSTICK_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
  #define A2_JOYSTICK_CONTROLLER_LOCAL __attribute__ ((visibility("hidden")))
#endif

#endif  // A2_JOYSTICK_CONTROLLER__VISIBILITY_CONTROL_H_
