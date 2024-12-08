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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// imports

public class SwerveModule {
  // declare hardware
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  // help >:(
  private final Double driveEncoder;
  private final Double steerEncoder;

  private final CANcoder absoluteEncoder;

  private PIDController m_drivePIDController; // sobbing idk how to do pid well somebosy help me

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(0.1, 0.1, 0.1, new TrapezoidProfile.Constraints(3, 2));

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  private final SimpleMotorFeedforward m_steerFeedforward = new SimpleMotorFeedforward(0.1, 0.1);

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
    // turningPidController = new PIDController(0.1, 0.0, 0);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // reset encoders
    resetEncoders();
  }

  // gets the state
  public SwerveModuleState getStates() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveVelocity(), new Rotation2d(getSteerPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getSteerPosition());

    SwerveModuleState.optimize(desiredState, encoderRotation);

    // desiredState.cosineScale(encoderRotation);

    final double driveOutput =
        m_drivePIDController.calculate(getDriveVelocity(), desiredState.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    final double turnOutput =
        m_turningPIDController.calculate(getSteerPosition(), desiredState.angle.getRadians());

    final double turnFeedForward =
        m_steerFeedforward.calculate((m_turningPIDController.getSetpoint().velocity));

    driveMotor.setVoltage(driveOutput + driveFeedforward);
    steerMotor.setVoltage(turnOutput + turnFeedForward);
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

  // gets the rotations or something
  public StatusSignal<Double> getDistance() {
    return driveMotor.getPosition();
  }
  // gets the steer motor position in radians
  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(steerMotor.getPosition().getValue());
  }
  // makeies the state

  // tells it the state we want it to be in

  // reports you on stopit
  // yes I think im funny
  public void stop() {
    driveMotor.set(0);
    steerMotor.set(0);
  }
}
