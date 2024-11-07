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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

  private TalonFX driveMotor;
  private TalonFX steerMotor;
  private CANcoder absoluteEncoder;
  // TPR = ticks per revolution
  private static final double DRIVE_ENCODER_TPR = 4096;
  private static final double STEER_ENCODER_TPR = 4096;
  private static final double WHEEL_CIRC = .5; // wheel circumference
  // conversion factors: convert encoder ticks to wheel revolutions
  private static final double driveConversionFactor = (WHEEL_CIRC * Math.PI) / DRIVE_ENCODER_TPR;
  private static final double steerConversionFactor = (2 * Math.PI) / STEER_ENCODER_TPR;

  public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID) {
    // motors & encoder
    driveMotor = new TalonFX(driveMotorCANID);
    steerMotor = new TalonFX(steerMotorCANID);
    absoluteEncoder = new CANcoder(cancoderCANID);

    // encoders
    Double driveEncoder = driveMotor.getPosition().getValue();
    Double steerEncoder = steerMotor.getPosition().getValue();

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

    // steering motor config
    steerMotor.setInverted(false);

    // position & velocity conversion factors
    // in radians & radians per second
    // TPR = ticks per revolution

    // swerve PID wrap around for tuning motor. allows it to go through 0
    double STEER_PID_WA = 2 * Math.PI;
    // PID gains for steer motor
    double STEER_P_GAIN = 0;
    double STEER_I_GAIN = 0;
    double STEER_D_GAIN = 0;

    // drive motor config
    driveMotor.setInverted(false);
    driveMotor.set(driveConversionFactor);
  }

  public StatusSignal<Double> getDistance() {
    return driveMotor.getPosition();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(steerMotor.getPosition().getValue());
  }

  public void setState(SwerveModuleState state) {
    double tragetSpeed = state.speedMetersPerSecond;
    double targetAngle = state.angle.getDegrees(); // degrees or radians?

    double targetVelocity = tragetSpeed / driveConversionFactor;

    double targetSteerAngle = targetAngle * (STEER_ENCODER_TPR / 360);

    // set target velocity
    driveMotor.set(targetVelocity);

    // set target angle
    steerMotor.set(targetSteerAngle);
  }
}
