package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;

public class DriveTrain {
    private Robot robot;

    //module indexes
    private static final int FRONT_LEFT_INDEX = 0;
    private static final int FRONT_RIGHT_INDEX = 1;
    private static final int BACK_LEFT_INDEX = 2;
    private static final int BACK_RIGHT_INDEX = 3;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    //Translation 2d Objects
    private final Translation2d frontRightLocation = new Translation2d(Hardware.WHEEL_DIST, -Hardware.WHEEL_DIST);
    private final Translation2d frontLeftLocation = new Translation2d( Hardware.WHEEL_DIST,  Hardware.WHEEL_DIST);
    private final Translation2d backLeftLocation = new Translation2d( -Hardware.WHEEL_DIST,  Hardware.WHEEL_DIST);
    private final Translation2d backRightLocation = new Translation2d(-Hardware.WHEEL_DIST, -Hardware.WHEEL_DIST);
    

    public final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(backLeftLocation, backRightLocation, frontLeftLocation, frontRightLocation);

    public final SwerveDriveOdometry swerveOdometry;

    

    public DriveTrain(Robot robot){
        this.robot = robot;
        if(Robot.TALON_BOT){
            frontRightModule = new SwerveModule(robot.hardware.frontRightDrive, robot.hardware.frontRightTalon, robot.hardware.frontRightDriveEnc, 0.0, FRONT_RIGHT_INDEX);
            frontLeftModule = new SwerveModule(robot.hardware.frontLeftDrive, robot.hardware.frontLeftTalon, robot.hardware.frontLeftDriveEnc, 0.0, FRONT_LEFT_INDEX);
            backLeftModule = new SwerveModule(robot.hardware.backLeftDrive, robot.hardware.backLeftTalon, robot.hardware.backLeftDriveEnc, 0.0, BACK_LEFT_INDEX);
            backRightModule = new SwerveModule(robot.hardware.backRightDrive, robot.hardware.backRightTalon, robot.hardware.backRightDriveEnc, 0.0, BACK_RIGHT_INDEX);
            robot.hardware.initTalons();
        }else{
            frontRightModule = new SwerveModule(robot.hardware.frontRightDrive, robot.hardware.frontRightRotation, robot.hardware.frontRightDriveEnc, robot.hardware.frontRightEncoder);
            frontLeftModule = new SwerveModule(robot.hardware.frontLeftDrive, robot.hardware.frontLeftRotation, robot.hardware.frontLeftDriveEnc, robot.hardware.frontLeftEncoder);
            backLeftModule = new SwerveModule(robot.hardware.backLeftDrive, robot.hardware.backLeftRotation, robot.hardware.backLeftDriveEnc, robot.hardware.backLeftEncoder);
            backRightModule = new SwerveModule(robot.hardware.backRightDrive, robot.hardware.backRightRotation, robot.hardware.backRightDriveEnc, robot.hardware.backRightEncoder);
        }
        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, Rotation2d.fromDegrees(robot.hardware.pigeon.getFusedHeading()));
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
        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, -xSpeed, rotation, Rotation2d.fromDegrees(-robot.hardware.pigeon.getFusedHeading()))
                : new ChassisSpeeds(ySpeed, -xSpeed, rotation)
        );
        setSwerveModuleStates(swerveModuleStates);
    }

    /**
     * Returns the swerve module state array
     * 
     * @return array of SwerveModuleState 0 is fl, 1 is fr, 2 is bl, 3 is br
     */
    public SwerveModuleState[] getSwerveModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeftModule.getState();
        states[1] = frontRightModule.getState();
        states[2] = backLeftModule.getState();
        states[3] = backRightModule.getState();
        return states;
    }

    /**
     * Sets each swerve module to its state determined by the states array
     * 
     * @param states an array of the swerve module states. 0 for fl, 1 for fr, 2 for bl, 3 for br 
     */
    public void setSwerveModuleStates(SwerveModuleState[] states){
        if(Robot.TALON_BOT){
            frontLeftModule.setTalonDesiredState(states[FRONT_LEFT_INDEX]);
            frontRightModule.setTalonDesiredState(states[FRONT_RIGHT_INDEX]);
            backLeftModule.setTalonDesiredState(states[BACK_LEFT_INDEX]);
            backRightModule.setTalonDesiredState(states[BACK_RIGHT_INDEX]);
        }else{
            frontLeftModule.setDesiredState(states[0]);
            frontRightModule.setDesiredState(states[1]);
            backLeftModule.setDesiredState(states[2]);
            backRightModule.setDesiredState(states[3]);
        }
    }

    /**
     * updates the odometry
     */
    public void updateOdom(){
        swerveOdometry.update(Rotation2d.fromDegrees(robot.hardware.pigeon.getFusedHeading()), getSwerveModuleStates());
    }

    public void setOdometry(Pose2d currentPoint){
        swerveOdometry.resetPosition(currentPoint, Rotation2d.fromDegrees(robot.hardware.pigeon.getFusedHeading()));
    }

    public void resetOdometry(){
        this.setOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(robot.hardware.pigeon.getFusedHeading())));
    }

}