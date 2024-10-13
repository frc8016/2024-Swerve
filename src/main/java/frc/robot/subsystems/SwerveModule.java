package frc.robot.subsystems;

import javax.management.ConstructorParameters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;

public class SwerveModule {

    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder absoluteEncoder;

public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID){
    //motors & encoder
    driveMotor = new TalonFX(driveMotorCANID);
    steerMotor = new TalonFX(steerMotorCANID);
    absoluteEncoder = new CANcoder(cancoderCANID);

    //encoders
    Double driveEncoder = driveMotor.getPosition().getValue();
    Double steerEncoder = steerMotor.getPosition().getValue();

  

    //reset devices to factory defaults 
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    steerMotor.getConfigurator().apply(new TalonFXConfiguration());
    absoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());

    //cancoder configuration
    CANcoderConfigurator cfg = absoluteEncoder.getConfigurator();
    cfg.apply(new CANcoderConfiguration());
    MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs();
    cfg.refresh(magnetSensorConfig);
    cfg.apply(magnetSensorConfig
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));


    //steering motor config 
    steerMotor.setInverted(false);
    
    //position & velocity conversion factors 
    // in radians & radians per second
   

    //swerve PID wrap around for tuning motor. allows it to go through 0
    
    //PID gains for steer motor
   

    //drive motor config 
    driveMotor.setInverted(false);
    

    //drive motor converions factors
    
}

public StatusSignal<Double> getDistance(){
    return driveMotor.getPosition();
}   

public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(steerMotor.getPosition().getValue());
}

public void setState(SwerveModuleState state){
    //driveMotor.set(ControlMode.Position, state.driveVelocity);

}

}
