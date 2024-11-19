package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

public class SwerveSubsystem extends SubsystemBase {

  public static SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;
  static WPI_PigeonIMU gyro;
  static SwerveModule[] swerveModules;

  public SwerveSubsystem() {

    swerveModules = new SwerveModule[4];

    swerveModules[0] =
        new SwerveModule(
            SwerveDriveConstants.SWERVE_DRIVE_MOTOR_0,
            SwerveDriveConstants.SWERVE_STEER_MOTOR_0,
            true,
            true,
            SwerveDriveConstants.SWERVE_ENCODER_0,
            true,
            SwerveDriveConstants.ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_0);
    swerveModules[1] =
        new SwerveModule(
            SwerveDriveConstants.SWERVE_DRIVE_MOTOR_1,
            SwerveDriveConstants.SWERVE_STEER_MOTOR_1,
            true,
            true,
            SwerveDriveConstants.SWERVE_ENCODER_1,
            true,
            SwerveDriveConstants.ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_1);
    swerveModules[2] =
        new SwerveModule(
            SwerveDriveConstants.SWERVE_DRIVE_MOTOR_2,
            SwerveDriveConstants.SWERVE_STEER_MOTOR_2,
            true,
            true,
            SwerveDriveConstants.SWERVE_ENCODER_2,
            true,
            SwerveDriveConstants.ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_2);
    swerveModules[3] =
        new SwerveModule(
            SwerveDriveConstants.SWERVE_DRIVE_MOTOR_3,
            SwerveDriveConstants.SWERVE_STEER_MOTOR_3,
            true,
            true,
            SwerveDriveConstants.SWERVE_ENCODER_3,
            true,
            SwerveDriveConstants.ABSOLUTE_ENCODER_ANGLE_OF_OFFSET_MODULE_3);
    // kinematics
    // should probs throw this in constants
    kinematics =
        new SwerveDriveKinematics(
            new Translation2d(
                Units.inchesToMeters(14.25), Units.inchesToMeters(14.25)), // front left
            new Translation2d(
                Units.inchesToMeters(14.25), Units.inchesToMeters(-14.25)), // front right
            new Translation2d(
                Units.inchesToMeters(-14.25), Units.inchesToMeters(14.25)), // back left
            new Translation2d(
                Units.inchesToMeters(-14.25), Units.inchesToMeters(-14.25)) // back right
            // translation 2ds that set the position of the module reletive to the center of the
            // chassis
            );

    gyro = new WPI_PigeonIMU(0); // needs acutual id from constants
    // resets the gyro position (in a cool way)
    new Thread(
            () -> {
              try {
                Thread.sleep(1000);
                zeroHeading();
              } catch (Exception e) {
              }
            })
        .start();

    odometry =
        new SwerveDriveOdometry(
            kinematics,
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            new Pose2d(0, 0, new Rotation2d()));
  }
  // end of constructor
  public void zeroHeading() {
    gyro.reset();
  }
  // returns the angle but in a cool way
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 2 * Math.PI);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters(); // returns the pose in meters
  }
  /*
  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(pose, getRotation2d());
  }*/

  public SwerveModulePosition[] getCurrentModulePositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(
          swerveModules[0].getDistance().getValue(), swerveModules[0].getAngle()),
      new SwerveModulePosition(
          swerveModules[1].getDistance().getValue(), swerveModules[1].getAngle()),
      new SwerveModulePosition(
          swerveModules[2].getDistance().getValue(), swerveModules[2].getAngle()),
      new SwerveModulePosition(
          swerveModules[3].getDistance().getValue(), swerveModules[3].getAngle())
    };
  }

  public static double getYaw() {
    return gyro.getYaw();
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), getCurrentModulePositions());

    SmartDashboard.putNumber("Robot Heading", getHeading());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

    SmartDashboard.putNumber("module 0", swerveModules[0].getAbsoluteEncoderRad());
    SmartDashboard.putNumber("module 1", swerveModules[1].getAbsoluteEncoderRad());
    SmartDashboard.putNumber("module 2", swerveModules[2].getAbsoluteEncoderRad());
    SmartDashboard.putNumber("module 3", swerveModules[3].getAbsoluteEncoderRad());
  }

  public void stopModules() {
    swerveModules[0].stop();
    swerveModules[1].stop();
    swerveModules[2].stop();
    swerveModules[3].stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredState) {
    // SwerveDriveKinematics.normalizeWheelSpeeds(desiredState, SwerveDriveConstants.kMaxSpeed);
    swerveModules[0].setDesiredState(desiredState[0]);
    swerveModules[1].setDesiredState(desiredState[1]);
    swerveModules[2].setDesiredState(desiredState[2]);
    swerveModules[3].setDesiredState(desiredState[3]);
  }

  public void print() {
    System.out.println("Button pressed");
  }

  public void drive1(double speed) {
    swerveModules[0].drive(speed);
    swerveModules[1].drive(speed);
    swerveModules[2].drive(speed);
    swerveModules[3].drive(speed);
  }

  public void drive2(double rotation) {
    swerveModules[0].turn(rotation);
    swerveModules[1].turn(rotation);
    swerveModules[2].turn(rotation);
    swerveModules[3].turn(rotation);
  }
}
