package frc.robot.Subsystems;



import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class Hardware {
    private Robot robot;

    //Constants
    public static final double MAX_SPEED = 3.0;//Meters per second
    public static final double MAX_ANGULAR_SPEED = Math.PI;//Half rotation per second
    public static final double WHEEL_DIST = Units.feetToMeters(1);

    //drive motors (ids: 1, 2, 3, 4)
    public CANSparkMax frontRightDrive = new CANSparkMax(1, MotorType.kBrushless); public RelativeEncoder frontRightDriveEnc = frontRightDrive.getEncoder();
    public CANSparkMax frontLeftDrive = new CANSparkMax(2, MotorType.kBrushless); public RelativeEncoder frontLeftDriveEnc = frontLeftDrive.getEncoder();
    public CANSparkMax backLeftDrive = new CANSparkMax(3, MotorType.kBrushless); public RelativeEncoder backLeftDriveEnc = backLeftDrive.getEncoder();
    public CANSparkMax backRightDrive = new CANSparkMax(4, MotorType.kBrushless); public RelativeEncoder backRightDriveEnc = backRightDrive.getEncoder();

    //rotation motors (ids: 11, 12, 13, 14)
    public CANSparkMax frontRightRotation = new CANSparkMax(11, MotorType.kBrushless);
    public CANSparkMax frontLeftRotation = new CANSparkMax(12, MotorType.kBrushless);
    public CANSparkMax backLeftRotation = new CANSparkMax(13, MotorType.kBrushless);
    public CANSparkMax backRightRotation = new CANSparkMax(14, MotorType.kBrushless);

    //rotation motor encoders
    private RelativeEncoder frontRightEncoder = frontRightRotation.getEncoder();
    private RelativeEncoder frontLeftEncoder = frontLeftRotation.getEncoder();
    private RelativeEncoder backLeftEncoder = backLeftRotation.getEncoder();
    private RelativeEncoder backRightEncoder = backRightRotation.getEncoder();

    //Translation 2d Objects
    private final Translation2d frontRightLocation = new Translation2d(WHEEL_DIST, WHEEL_DIST);
    private final Translation2d frontLeftLocation = new Translation2d(-1 * WHEEL_DIST, WHEEL_DIST);
    private final Translation2d backLeftLocation = new Translation2d(-1 * WHEEL_DIST, -1 * WHEEL_DIST);
    private final Translation2d backRightLocation = new Translation2d(WHEEL_DIST, -1 * WHEEL_DIST);

    //SwerveModuleState objects
    private final SwerveModuleState frontRightModule = new SwerveModuleState();
    private final SwerveModuleState frontLeftModule = new SwerveModuleState();
    private final SwerveModuleState backLeftModule = new SwerveModuleState();
    private final SwerveModuleState backRightModule = new SwerveModuleState();

    //SwerveModule objects
    private final SwerveModule frontRight = new SwerveModule(frontRightDrive, frontRightRotation, frontRightDriveEnc, frontRightEncoder);
    private final SwerveModule frontLeft = new SwerveModule(frontLeftDrive, frontLeftRotation, frontLeftDriveEnc, frontLeftEncoder);
    private final SwerveModule backLeft = new SwerveModule(backLeftDrive, backLeftRotation, backLeftDriveEnc, backLeftEncoder);
    private final SwerveModule backRight = new SwerveModule(backRightDrive, backRightRotation, backRightDriveEnc, backRightEncoder);

    public final PigeonIMU pigeon = new PigeonIMU(0);//I don't know how to get a rotation2d object from a pigeon, will have to ask Danny at some point
    public final Rotation2d rotation2d = Rotation2d.fromDegrees(pigeon.getFusedHeading());//For now this'll work to not have so many errors
    

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(frontRightLocation, frontLeftLocation, backLeftLocation, backRightLocation);

    private final SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(swerveKinematics, rotation2d);

    //Shooter motor. ids 5, 6(follower)
    public CANSparkMax shooterMotor = new CANSparkMax(5, MotorType.kBrushless);
    public CANSparkMax shooterFollower = new CANSparkMax(6, MotorType.kBrushless);

    //Intake motors, ids 7-10(if needed)
    public CANSparkMax intakeMotor = new CANSparkMax(7, MotorType.kBrushless);
    public CANSparkMax intakeSecond = new CANSparkMax(8, MotorType.kBrushless);//Assuming that there is in fact a second motor
    public CANSparkMax hopperMotor = new CANSparkMax(9, MotorType.kBrushless);//Again assuming that there is going to some sort of belt or something from intake to shooter 

    //Climber motors, ids 15-18. I can't imagine it taking more than 4 motors, nor can I imagine our robot having 18 motors on it
    public CANSparkMax climberOne = new CANSparkMax(15, MotorType.kBrushless);
    public CANSparkMax climberTwo = new CANSparkMax(16, MotorType.kBrushless);
    public CANSparkMax climberThree = new CANSparkMax(17, MotorType.kBrushless);

    public Hardware(Robot robot){
        this.robot = robot;
        //Reset pigeon here, couldn't find the command for it
        shooterFollower.follow(shooterMotor);
        shooterFollower.setInverted(true);
        Rotation2d.fromDegrees(pigeon.getFusedHeading());
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
        swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, Rotation2d.fromDegrees(pigeon.getFusedHeading()))
                : new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );   
        if(fieldRelative){
            swerveModuleStates = swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, rotation2d));
        }
    }

    public void updateOdom(){
        //Update odometry and stuff here, couldn't find the commands for it
    }

}
