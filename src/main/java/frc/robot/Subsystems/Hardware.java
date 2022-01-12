package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class Hardware {
    public static final double MAX_SPEED = 3.0;//Meters per second
    public static final double MAX_ANGULAR_SPEED = Math.PI;//Half rotation per second
    public static final double WHEEL_DIST = Units.feetToMeters(1);

    private final Translation2d frontRightLocation = new Translation2d(WHEEL_DIST, WHEEL_DIST);
    private final Translation2d frontLeftLocation = new Translation2d(-1 * WHEEL_DIST, WHEEL_DIST);
    private final Translation2d backLeftLocation = new Translation2d(-1 * WHEEL_DIST, -1 * WHEEL_DIST);
    private final Translation2d backRightLocation = new Translation2d(WHEEL_DIST, -1 * WHEEL_DIST);

    private final SwerveModuleState frontRightModule = new SwerveModuleState();
    private final SwerveModuleState frontLeftModule = new SwerveModuleState();
    private final SwerveModuleState backLeftModule = new SwerveModuleState();
    private final SwerveModuleState backRightModule = new SwerveModuleState();

    private final PigeonIMU pigeon = new PigeonIMU(0);//I don't know how to get a rotation2d object from a pigeon, will have to ask Danny at some point
    private final Rotation2d rotation2d = new Rotation2d();//For now this'll work to not have so many errors

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(frontRightLocation, frontLeftLocation, backLeftLocation, backRightLocation);

    private final SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(swerveKinematics, rotation2d);

    public Hardware(){
        //Reset pigeon here, couldn't find the command for it

    }

    /**
     * Method for driving the roboto
     * 
     * @param xSpeed Speed in x direction
     * @param ySpeed Speed in y direction
     * @param rotation Angular speed
     * @param fieldRelative Whether it's in field relative control mode or not
     */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative){ 
        var swerveModuleStates = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rotation));
        if(fieldRelative){
            swerveModuleStates = swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, rotation2d));
        }
    }

    public void updateOdom(){
        //Update odometry and stuff here, couldn't find the commands for it
    }

}
