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
    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 3;
    public static final int kDriverFieldOrientedButtonIdx = 1;

    public static final double kDeadBand = 0.05;
  }

  public static class SwerveDriveConstants {
    // IDK if we need these
    public static final double ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_0 =
        -2.35159519855 / SwerveModuleConstants.ENCODER_TPR; // radians .374512 rotations
    public static final double ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_1 =
        -2.47277899357 / SwerveModuleConstants.ENCODER_TPR; // .393311 rotations
    public static final double ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_2 =
        -6.05002258362 / SwerveModuleConstants.ENCODER_TPR; // .962891 rotations
    public static final double ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_3 =
        (-4.51910563397 / SwerveModuleConstants.ENCODER_TPR); // .719482 rotations
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

    public static final double kMaxSpeed = 0.1;

    public static final double kAngularMaxSpeed = 2 * 2 * Math.PI;
  }

  public static class SwerveModuleConstants {
    public static final double ENCODER_TPR = 4096;
    public static final double WHEEL_CIRCUMFERENCE = .5; // is that real?
    public static final double kDriveMotorGearRatio = 0; // Im too lazy to google this rn
    public static final double kSteerMotorGearRatio = 0; // Im too lazy to google this rn
    public static final double kDriveEncoderRot2Meter =
        kDriveMotorGearRatio * Math.PI * (WHEEL_CIRCUMFERENCE * 2);
    public static final double kTurningEncoderRot2Rad = kSteerMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec =
        kDriveEncoderRot2Meter / 60; // yall i dont even know
    public static final double kTurningEncoderRPM2RadPerSec =
        kTurningEncoderRot2Rad / 60; // same fer here
    public static final double kPSteer = 0.5;

    // WHAT DOES THE K MEAN? AHHHHHHHHHHH
    public static final double kDriveConversionFactor =
        (WHEEL_CIRCUMFERENCE * Math.PI) / ENCODER_TPR;
    public static final double kSteerConversionFactor = (2 * Math.PI) / ENCODER_TPR;
  }
}
// constanst more like constantly losing it
// hehehe
