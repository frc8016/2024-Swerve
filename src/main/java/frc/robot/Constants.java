// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kJoystickPort = 1;
  }

  public static class SwerveDriveConstants {
    // IDK if we need these
    public static final double ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_0 = 169.716796875;
    public static final double ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_1 = 0;
    public static final double ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_2 = 0;
    public static final double ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_3 = 0;
    // Module 0
    public static final int SWERVE_DRIVE_MOTOR_0 = 2;
    public static final int SWERVE_STEER_MOTOR_0 = 1;
    public static final int SWERVE_ENCODER_0 = 9;
    // Module 1
    public static final int SWERVE_DRIVE_MOTOR_1 = 4;
    public static final int SWERVE_STEER_MOTOR_1 = 3;
    public static final int SWERVE_ENCODER_1 = 10;
    // Module 2
    public static final int SWERVE_DRIVE_MOTOR_2 = 6;
    public static final int SWERVE_STEER_MOTOR_2 = 5;
    public static final int SWERVE_ENCODER_2 = 11;
    // Module 3
    public static final int SWERVE_DRIVE_MOTOR_3 = 8;
    public static final int SWERVE_STEER_MOTOR_3 = 7;
    public static final int SWERVE_ENCODER_3 = 12;
  }

  public static class SwerveModuleConstants {
    public static final double ENCODER_TPR = 4096;
    public static final double WHEEL_CIRCUMFERENCE = .5;
    public static final double kDriveConversionFactor = (WHEEL_CIRCUMFERENCE * Math.PI) / ENCODER_TPR;
    public static final double kSteerConversionFactor = (2 * Math.PI) / ENCODER_TPR;
  }
}
