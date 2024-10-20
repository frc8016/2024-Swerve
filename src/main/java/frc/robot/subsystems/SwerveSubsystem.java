package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;



public class SwerveSubsystem {

    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    WPI_PigeonIMU gyro;
    SwerveModule[] swerveModules; //psudo code


    public SwerveSubsystem() {

        swerveModules = new SwerveModule[4];

        swerveModules[0] = new SwerveModule(2, 1, 9);
        swerveModules[1] = new SwerveModule(4, 3, 10);
        swerveModules[2] = new SwerveModule(6, 5, 11);
        swerveModules[3] = new SwerveModule(8, 7, 12);

        //kinematics
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(14.25), Units.inchesToMeters(14.25)), //front left 
            new Translation2d(Units.inchesToMeters(14.25), Units.inchesToMeters(-14.25)), // front right 
            new Translation2d(Units.inchesToMeters(-14.25), Units.inchesToMeters(14.25)), // back left 
            new Translation2d(Units.inchesToMeters(-14.25), Units.inchesToMeters(-14.25)) // back right 
        //translation 2ds that set the position of the module reletive to the center of the chassis 
        );

        gyro = new WPI_PigeonIMU(0); // needs acutual id from constants
        
        odometry = new SwerveDriveOdometry(
            kinematics,
             gyro.getRotation2d(), 
             new SwerveModulePosition[]
                {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
                 new Pose2d(0,0, new Rotation2d()));

    }
    //drive function 
    public void drive(){
  
        ChassisSpeeds testSpeeds = new ChassisSpeeds(
            Units.inchesToMeters(14), 
            Units.inchesToMeters(4), 
            Units.degreesToRadians(30));

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);

         swerveModules[0].setState(swerveModuleStates[0]);
         swerveModules[1].setState(swerveModuleStates[1]);
         swerveModules[2].setState(swerveModuleStates[2]);
         swerveModules[3].setState(swerveModuleStates[3]);
     

    }

    
    public SwerveModulePosition[] getCurrentModulePositions(){
        return new SwerveModulePosition[]{
            new SwerveModulePosition(swerveModules[0].getDistance().getValue(), swerveModules[0].getAngle()),
            new SwerveModulePosition(swerveModules[1].getDistance().getValue(), swerveModules[1].getAngle()),
            new SwerveModulePosition(swerveModules[2].getDistance().getValue(), swerveModules[2].getAngle()),
            new SwerveModulePosition(swerveModules[3].getDistance().getValue(), swerveModules[3].getAngle())

        };
    }


 
  


}
