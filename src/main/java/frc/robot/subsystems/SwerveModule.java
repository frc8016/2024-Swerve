package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.function.BooleanSupplier;
// imports

public class SwerveModule {
  // declare hardware
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  // help >:(
  private final Double driveEncoder;
  private final Double steerEncoder;

  private final CANcoder absoluteEncoder;

  private PIDController turningPidController; // sobbing idk how to do pid well somebosy help me



  private static Double absoluteEncoderAngleOfOffset;

  // TPR = ticks per revolution
  private static final double DRIVE_ENCODER_TPR = 4096;
  private static final double STEER_ENCODER_TPR = 4096;
  private static final double WHEEL_CIRC = .5; // wheel circumference
  // conversion factors: convert encoder ticks to wheel revolutions
  private static final double driveConversionFactor = (WHEEL_CIRC * Math.PI) / DRIVE_ENCODER_TPR;

  // the swerve module so the actual object that we are calling and using and stuff
  public SwerveModule(
      int driveMotorCANID,
      int steerMotorCANID,
      boolean driveMotorReversed,
      boolean steerMotorReversed,
      int cancoderCANID,
      boolean canCoderReversed,
      double absoluteEncoderOffset) {
    // motors & encoder

    driveMotor = new TalonFX(driveMotorCANID);
    steerMotor = new TalonFX(steerMotorCANID);
    absoluteEncoder = new CANcoder(cancoderCANID);

    // steering motor config
    steerMotor.setInverted(steerMotorReversed);
    // drive motor config
    driveMotor.setInverted(driveMotorReversed);
    driveMotor.set(driveConversionFactor);

    SwerveModule.absoluteEncoderAngleOfOffset = absoluteEncoderOffset;

    // encoders in the motors
    // Idk how to this correctly for krackens someobe help me
    driveEncoder = driveMotor.getPosition().getValue();
    steerEncoder = steerMotor.getPosition().getValue();

    // reset devices to factory defaults
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    steerMotor.getConfigurator().apply(new TalonFXConfiguration());
    absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

    // cancoder configuration
    CANcoderConfigurator cfg = absoluteEncoder.getConfigurator();
    cfg.apply(new CANcoderConfiguration());
    MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs();
    cfg.refresh(magnetSensorConfig);
    cfg.apply(
        magnetSensorConfig
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

    // add conversion factors
    // add PID control
    turningPidController = new PIDController(0.1, 0.0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    // reset encoders
    resetEncoders();
  }
  
  // returns the drive motors embeded encoder position
  // also wth vs code stop making errors where there arent any
  public double getDrivePosition() {
    return driveMotor.getPosition().getValue();
  }
  // same as above except with the steer motor
  public double getSteerPosition() {
    return steerMotor.getPosition().getValue();
  }

  // returns the drive motors velocity in rotations per second
  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValue();
  }
  // same as above but with steer motor
  public double getSteerVelocity() {
    return steerMotor.getVelocity().getValue();
  }
  // I decided to use radians :)
  // returns the abs encoder position in radians

  // keeps the value with 2pi
  public double reZeroEncoder() {
    if (getAbsoluteEncoderRad() >= 2 * Math.PI) {
      return getAbsoluteEncoderRad() - (2 * Math.PI);
    } else if (getAbsoluteEncoderRad() <= 0) {
      return getAbsoluteEncoderRad() + (2 * Math.PI);
    } else {
      return getAbsoluteEncoderRad();
    }
  }

  public double getAbsoluteEncoderRad() {
    double angle =
        ((absoluteEncoder.getPosition().getValue()) * 2 * Math.PI) - absoluteEncoderAngleOfOffset;
    return angle;
  }
  // resets the encoders crazy right
  // sets the drive encoder (which idk if its even real at this point) to zero, and the steer
  // encoder to the value of the abs encoder (the CAN coder)
  public void resetEncoders() {
    driveMotor.setPosition(0);
    steerMotor.setPosition(getAbsoluteEncoderRad());
  }

  // gets the state
  public SwerveModuleState getStates() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
  }
  // gets the rotations or something
  public StatusSignal<Double> getDistance() {
    return driveMotor.getPosition();
  }
  // gets the steer motor position in radians
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(steerMotor.getPosition().getValue());
  }
  // makeies the state
  public void setState(SwerveModuleState state) {
    double tragetSpeed = state.speedMetersPerSecond;
    double targetAngle = state.angle.getRadians(); // degrees or radians?  Radians :)
    // am i using these anywhare?
    double targetVelocity = tragetSpeed / driveConversionFactor;
    double targetSteerAngle = targetAngle * (STEER_ENCODER_TPR / 2 * Math.PI);

    // set target velocity
    driveMotor.set(targetVelocity);

    // set target angle
    steerMotor.set(targetSteerAngle);
  }
  // tells it the state we want it to be in
  public void setDesiredState(SwerveModuleState state) {
     if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getStates().angle);

    driveMotor.set(
        state.speedMetersPerSecond); // (speed / contant(max speed (set it hella low to start)))

    steerMotor.set(
        turningPidController.calculate(
            reZeroEncoder(), state.angle.getRadians()));// pid controller ahhhhhhhh!!!!!!!!

  return;
  }
  // reports you on stopit
  // yes I think im funny
  public void stop() {
    driveMotor.set(0);
    steerMotor.set(0);
  }

  public void setAngleGoal(double setpoint) {
    turningPidController.setSetpoint(setpoint);
  }

  public double getSetpoint() {
    return turningPidController.getSetpoint();
  }

  public void setSetpoint() {
    turningPidController.setSetpoint(1);
  }

  // gets the postion and checks if it == the goal yadayada
  public boolean checkAnglePosition() {
    if (reZeroEncoder() == getSetpoint()) {
      return true;
    } else if (reZeroEncoder() != getSetpoint()) {
      return false;
    }
    return checkAnglePosition();
  }

  public BooleanSupplier checkAngleSupplier() {
    return (() -> checkAnglePosition());
  }

  public void moveToAngle() {
    if (checkAnglePosition() == false) {
      steerMotor.set(.1);
    } else {
      stop();
    }
  }

  public void set(double speed) {
    steerMotor.set(speed);
  }
}
