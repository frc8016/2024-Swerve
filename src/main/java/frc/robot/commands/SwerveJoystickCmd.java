package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.Supplier;

public class SwerveJoystickCmd extends Command {

  private final SwerveSubsystem swerveSubsystem;

  private final Supplier<Double> xSpdFunction, ySpdFunction, steerSpdFUnction;
  private final Supplier<Boolean> fieldOrientedFunction;
  private final SlewRateLimiter xLimiter, yLimiter, steerLimiter;

  public SwerveJoystickCmd(
      SwerveSubsystem swerveSubsystem,
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdSFunction,
      Supplier<Double> steerSpdFunction,
      Supplier<Boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdSFunction;
    this.steerSpdFUnction = steerSpdFunction;
    this.fieldOrientedFunction = fieldOrientedFunction;

    this.xLimiter = new SlewRateLimiter(0); // max acceleration
    this.yLimiter = new SlewRateLimiter(0); // same as x
    this.steerLimiter = new SlewRateLimiter(0); // max agluar acceleration
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // get joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double steerSpeed = steerSpdFUnction.get();

    // apply deadband
    xSpeed = Math.abs(xSpeed) > OperatorConstants.kDeadBand ? xSpeed : 0;
    ySpeed = Math.abs(ySpeed) > OperatorConstants.kDeadBand ? ySpeed : 0;
    steerSpeed = Math.abs(steerSpeed) > OperatorConstants.kDeadBand ? steerSpeed : 0;

    // smooth drivine
    xSpeed = xLimiter.calculate(xSpeed) * SwerveDriveConstants.kMaxSpeed;
    ySpeed = yLimiter.calculate(ySpeed) * SwerveDriveConstants.kMaxSpeed;
    steerSpeed = steerLimiter.calculate(steerSpeed) * SwerveDriveConstants.kAngularMaxSpeed;

    // construct desired chassis
    ChassisSpeeds chassisSpeeds;
    if (fieldOrientedFunction.get()) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, steerSpeed, swerveSubsystem.getRotation2d());
    } else {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, steerSpeed, swerveSubsystem.getRotation2d());
    }

    // convert chassis speeds to module states
    SwerveModuleState[] moduleStates =
        SwerveSubsystem.kinematics.toSwerveModuleStates(chassisSpeeds);

    // output module states to wheels
    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
