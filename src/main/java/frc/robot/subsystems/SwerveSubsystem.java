package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
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

        swerveModules = new SwerveModule[4]; // psudo code 

        swerveModules[0] = new SwerveModule(0, 0, 0);
        swerveModules[1] = new SwerveModule(0, 0, 0);
        swerveModules[2] = new SwerveModule(0, 0, 0);
        swerveModules[3] = new SwerveModule(0, 0, 0);

        //kinematics
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(14.25), Units.inchesToMeters(14.25)), //front left 
            new Translation2d(Units.inchesToMeters(14.25), Units.inchesToMeters(-14.25)), // front right 
            new Translation2d(Units.inchesToMeters(-14.25), Units.inchesToMeters(14.25)), // back left 
            new Translation2d(Units.inchesToMeters(-14.25), Units.inchesToMeters(-14.25)) // back right 
        //translation 2ds that set the position of the module reletive to the center of the chassis 
        );

        gyro = new WPI_PigeonIMU(0); // needs acutual id from constants 

       


    }
    //drive function 
    public void drive(){
        ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(14), Units.inchesToMeters(4), Units.degreesToRadians(30));

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);

    }
    
    public SwerveModulePosition[] getCurrentModulePositions(){
        return new SwerveModulePosition[]{

        };
    }


}
