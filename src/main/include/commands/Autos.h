// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <subsystems/Drivetrain.h>
#include <subsystems/Elevator.h>
#include <subsystems/Intake.h>
#include <subsystems/Shooter.h>

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
frc2::CommandPtr ShootAndDrive(Drivetrain* drivetrain, Elevator* elevator,
                               Intake* intake, Shooter* shooter);

}  // namespace autos
