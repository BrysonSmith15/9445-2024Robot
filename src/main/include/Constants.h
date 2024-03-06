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
const bool usingFieldOrientedTurn = false;

}  // namespace OperatorConstants

namespace BindingConstants {
const int shootCompositionTrigger = 2;
const int shootButton = 6;
const int intakeButton = 1;
const int moveToShooterAxis = 3;

const int elevatorDownAngle = 180;
const int elevatorUpAngle = 0;

const int elevatorSourceButton = 3;
const int trackSpeakerButton = 5;
const int elevatorAmpButton = 2;

const int trackSpeakerAxis = 2;

const int elevatorManualUpButton = 8;
const int elevatorManualDownButton = 7;

}  // namespace BindingConstants

namespace DrivetrainConstants {

const units::meter_t length = 29_in;
const units::meter_t width = 29_in;
const units::meter_t diagonal = units::inch_t{std::sqrt(
    (length.value() * length.value()) + (width.value() * width.value()))};

}  // namespace DrivetrainConstants

namespace ElevatorConstants {

const double speed = 0.52;
const double gearRatio = 7.0 / 32.0;
const int stableTicks = 0;
const int stableSpeed = 0.3;
enum setpointOptions { bottom, source, amp, speaker, climb };
// (x + y) / 2 takes average of max and min angles
const units::degree_t bottomTicks = 0_deg;
const units::degree_t sourceTicks = (36.9938_deg + 38.9370_deg) / 2;
const units::degree_t ampTicks = (54.98819155_deg + 57.22044685_deg) / 2;
const units::degree_t speakerTicks = (42.41_deg + 58.4692_deg) / 2;
// TODO: this is maybe right
const units::degree_t climbTicks = 90_deg;

}  // namespace ElevatorConstants

namespace VisionConstants {
const std::string tagFamily = "tag36h11";
const int rSourceCenterID = 4;
const int bSourceCenterID = 7;
const int frontCameraXRes = 640;
const int frontCameraYRes = 480;
const units::degree_t frontCameraHFOV = 100_deg;
const units::degree_t frontCameraVFOV =
    std::atan2(units::radian_t{frontCameraHFOV}.value(),
               units::radian_t{138_deg}.value()) *
    1_rad;
}  // namespace VisionConstants