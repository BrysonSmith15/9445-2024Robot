// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/button/CommandJoystick.h>

#include <functional>

#include "RobotContainer.h"
#include "subsystems/Drivetrain.h"

/**
 * This command implements typical drive
 * using the 2024 swerve drive chassis
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveCommand : public frc2::CommandHelper<frc2::Command, DriveCommand> {
 public:
  /**
   * Creates a new Drive.
   *
   * @param drivetrain The drivetrain class used by this command.
   * @param xTranslationSupplier A function which returns the desired X state in
   * mps
   * @param yTranslationSupplier A function which returns the desired Y state in
   * mps
   * @param thetaSupplier A function which returns the desired rotational state
   * in rad_per_sec
   */
  DriveCommand(Drivetrain* drivetrain,
               std::function<units::meters_per_second_t()> xTranslationSupplier,
               std::function<units::meters_per_second_t()> yTranslationSupplier,
               std::function<units::radians_per_second_t()> thetaSupplier);
  void Execute() override;
  void End(bool interrupted) override;

 private:
  Drivetrain* drivetrain;
  std::function<units::meters_per_second_t()> xTranslation;
  std::function<units::meters_per_second_t()> yTranslation;
  std::function<units::radians_per_second_t()> theta;
};
