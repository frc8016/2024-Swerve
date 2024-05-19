package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModule {

    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder absoluteEncoder;

public SwerveModule(int driveMotorCANID, int steerMotorCANID, int cancoderCANID){

    driveMotor = new TalonFX(driveMotorCANID);
    steerMotor = new TalonFX(steerMotorCANID);
    absoluteEncoder = new CANcoder(cancoderCANID);

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

    //drive motor config 
    driveMotor.setInverted(false);
    
}

    
}
