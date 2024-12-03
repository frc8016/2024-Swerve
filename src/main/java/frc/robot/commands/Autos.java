// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Command start(SwerveSubsystem swerve) {
    return Commands.runOnce(() -> swerve.drive1(), swerve);

    // Commands.runOnce(() -> swerve.setAngle(0), swerve);
  }

  public static Command idk(SwerveSubsystem swerve) {
    return new StartEndCommand(() -> swerve.drive505(.1), () -> swerve.drive505(0), swerve);

    // return Commands.runOnce(() -> swerve.setModuleStates(new SwerveModuleState[]));

  }
}
