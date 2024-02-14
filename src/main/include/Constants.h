// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <math.h>
#include <units/angle.h>
#include <units/length.h>

#include <string>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr int kSecondControllerPort = 1;
const bool usingFieldOrientedTurn = true;

}  // namespace OperatorConstants

namespace BindingConstants {

const int shootCompositionButton = 1;
const int shootButton = 2;
const int moveToShooterButton = 3;
const int trackSpeakerAxis = 4;

}  // namespace BindingConstants

namespace DrivetrainConstants {

const units::meter_t length = 29_in;
const units::meter_t width = 29_in;
const units::meter_t diagonal = units::inch_t{std::sqrt(
    (length.value() * length.value()) + (width.value() * width.value()))};

}  // namespace DrivetrainConstants

namespace ElevatorConstants {

enum setpointOptions { bottom, intake, amp, speaker, climb };
// TODO: Figure these out
const int bottomTicks = 0;
const int intakeTicks = 0;
const int ampTicks = 0;
const int speakerTicks = 0;
const int climbTicks = 0;

}  // namespace ElevatorConstants

namespace VisionConstants {
const std::string tagFamily = "tag36h11";
const int rSourceCenterID = 4;
const int bSourceCenterID = 7;
const units::degree_t frontCameraHFOV = 100_deg;
const units::degree_t frontCameraVFOV =
    std::atan2(units::radian_t{frontCameraHFOV}.value(),
               units::radian_t{138_deg}.value()) *
    1_rad;
}  // namespace VisionConstants