package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class DriveTrain {
    private Robot robot;

    private static final double FRONT_RIGHT_P = 0.0039;
    private static final double FRONT_RIGHT_I = 0.0;
    private static final double FRONT_RIGHT_D = 0.0;
    private static final double FRONT_RIGHT_FF = 0.001;

    private static final double FRONT_LEFT_P = 0.0038;
    private static final double FRONT_LEFT_I = 0.0;
    private static final double FRONT_LEFT_D = 0.0;
    private static final double FRONT_LEFT_FF = 0.001;

    private static final double BACK_RIGHT_P = 0.0038;
    private static final double BACK_RIGHT_I = 0.0;
    private static final double BACK_RIGHT_D = 0.0;
    private static final double BACK_RIGHT_FF = 0.001;

    private static final double BACK_LEFT_P = 0.0038;
    private static final double BACK_LEFT_I = 0.0;
    private static final double BACK_LEFT_D = 0.0;
    private static final double BACK_LEFT_FF = 0.001;

    //drive motors (ids: 11, 12, 13, 14)
    private CANSparkMax frontRightDrive = new CANSparkMax(11, MotorType.kBrushless);
    private CANSparkMax frontLeftDrive = new CANSparkMax(12, MotorType.kBrushless);
    private CANSparkMax backLeftDrive = new CANSparkMax(13, MotorType.kBrushless);
    private CANSparkMax backRightDrive = new CANSparkMax(14, MotorType.kBrushless);
    
    private RelativeEncoder frontRightDriveEnc = frontRightDrive.getEncoder();
    private RelativeEncoder frontLeftDriveEnc = frontLeftDrive.getEncoder();
    private RelativeEncoder backLeftDriveEnc = backLeftDrive.getEncoder();
    private RelativeEncoder backRightDriveEnc = backRightDrive.getEncoder();

    //rotation motors (ids: 1, 2, 3, 4)
    private CANSparkMax frontRightRotation = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax frontLeftRotation = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax backLeftRotation = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax backRightRotation = new CANSparkMax(4, MotorType.kBrushless);

    //Talon rotation motors so we can test with old swerve bot
    private WPI_TalonSRX frontRightTalon = new WPI_TalonSRX(1);
    private WPI_TalonSRX frontLeftTalon = new WPI_TalonSRX(2);
    private WPI_TalonSRX backLeftTalon = new WPI_TalonSRX(3);
    private WPI_TalonSRX backRightTalon = new WPI_TalonSRX(4);

    //rotation motor encoders
    private RelativeEncoder frontRightEncoder = frontRightRotation.getEncoder();
    private RelativeEncoder frontLeftEncoder = frontLeftRotation.getEncoder();
    private RelativeEncoder backLeftEncoder = backLeftRotation.getEncoder();
    private RelativeEncoder backRightEncoder = backRightRotation.getEncoder();

    public final BasePigeon pigeon;

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
    private final Translation2d frontRightLocation = new Translation2d(Robot.WHEEL_DIST, -Robot.WHEEL_DIST);
    private final Translation2d frontLeftLocation = new Translation2d( Robot.WHEEL_DIST,  Robot.WHEEL_DIST);
    private final Translation2d backLeftLocation = new Translation2d( -Robot.WHEEL_DIST,  Robot.WHEEL_DIST);
    private final Translation2d backRightLocation = new Translation2d(-Robot.WHEEL_DIST, -Robot.WHEEL_DIST);
    

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(backLeftLocation, backRightLocation, frontLeftLocation, frontRightLocation);

    private final SwerveDriveOdometry swerveOdometry;

    

    public DriveTrain(Robot robot){
        this.robot = robot;
        if(Robot.TALON_BOT){
            pigeon = new PigeonIMU(25);
        }else{
            pigeon = new Pigeon2(25);
        }
        initSparks();
        this.pigeon.setYaw(0.0);
        if(Robot.TALON_BOT){
            frontRightModule = new SwerveModule(this.frontRightDrive, this.frontRightTalon, this.frontRightDriveEnc, 0.0, FRONT_RIGHT_INDEX);
            frontLeftModule = new SwerveModule(this.frontLeftDrive, this.frontLeftTalon, this.frontLeftDriveEnc, 0.0, FRONT_LEFT_INDEX);
            backLeftModule = new SwerveModule(this.backLeftDrive, this.backLeftTalon, this.backLeftDriveEnc, 0.0, BACK_LEFT_INDEX);
            backRightModule = new SwerveModule(this.backRightDrive, this.backRightTalon, this.backRightDriveEnc, 0.0, BACK_RIGHT_INDEX);
            this.initTalons();
        }else{
            frontRightModule = new SwerveModule(this.frontRightDrive, this.frontRightRotation, this.frontRightDriveEnc, this.frontRightEncoder);
            frontLeftModule = new SwerveModule(this.frontLeftDrive, this.frontLeftRotation, this.frontLeftDriveEnc, this.frontLeftEncoder);
            backLeftModule = new SwerveModule(this.backLeftDrive, this.backLeftRotation, this.backLeftDriveEnc, this.backLeftEncoder);
            backRightModule = new SwerveModule(this.backRightDrive, this.backRightRotation, this.backRightDriveEnc, this.backRightEncoder);
        }
        swerveOdometry = new SwerveDriveOdometry(swerveKinematics, Rotation2d.fromDegrees(this.pigeon.getYaw()));
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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, -xSpeed, rotation, Rotation2d.fromDegrees(-this.pigeon.getYaw()))
                : new ChassisSpeeds(ySpeed, -xSpeed, rotation)
        );
        this.updateOdometry();
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
    public void updateOdometry(){
        swerveOdometry.update(Rotation2d.fromDegrees(this.pigeon.getYaw()), getSwerveModuleStates());
    }

    /**
     * Sets the current position and rotation of the odometry
     */
    public void setOdometry(Pose2d currentPoint){
        swerveOdometry.resetPosition(currentPoint, Rotation2d.fromDegrees(this.pigeon.getYaw()));
    }

    /**
     * Resets odometry to be the origin (x: 0, y: 0, rotation: 0)
     */
    public void resetOdometry(){
        this.setOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(this.pigeon.getYaw())));
    }

    public Pose2d getOdometryPoseMeters(){
        return swerveOdometry.getPoseMeters();
    }

    /**
     * Runs each WPI_TalonSRX through the initTalon function
     */
    public void initTalons(){
        initTalon(frontLeftTalon, 0.0);
        initTalon(frontRightTalon, 0.0);
        initTalon(backLeftTalon, 0.0);
        initTalon(backRightTalon, 0.0);
    }

    /**
     * Initates a WPI_TalonSRX with the correct encoder counts and offsets
     * @param talon the motor being initated
     * @param offset the offset of the encoder
     */
    public void initTalon(WPI_TalonSRX talon, double offset){
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
        Timer.delay(0.3);
        //if the talons get offset, uncomment the lower line, hand set the modules to 0, 
        //redeploy code, and then recomment the line, and redeploy.
        
        talon.setSelectedSensorPosition(0, 0, 0);
    }

    /**
     * Initializes all the spark maxes
     */
    public void initSparks(){
        if(!Robot.TALON_BOT){
            initSpark(frontRightRotation);
            initSpark(frontLeftRotation);
            initSpark(backLeftRotation);
            initSpark(backRightRotation);
        }
        initSpark(frontRightDrive);
        initSpark(frontLeftDrive);
        initSpark(backLeftDrive);
        initSpark(backRightDrive);

        setPIDF(frontRightDrive, DriveTrain.FRONT_RIGHT_P, DriveTrain.FRONT_RIGHT_I, DriveTrain.FRONT_RIGHT_D, DriveTrain.FRONT_RIGHT_FF);
        setPIDF(frontLeftDrive, DriveTrain.FRONT_LEFT_P, DriveTrain.FRONT_LEFT_I, DriveTrain.FRONT_LEFT_D, DriveTrain.FRONT_LEFT_FF);
        setPIDF(backRightDrive, DriveTrain.BACK_RIGHT_P, DriveTrain.BACK_RIGHT_I, DriveTrain.BACK_RIGHT_D, DriveTrain.BACK_RIGHT_FF);
        setPIDF(backLeftDrive, DriveTrain.BACK_LEFT_P, DriveTrain.BACK_LEFT_I, DriveTrain.BACK_LEFT_D, DriveTrain.BACK_LEFT_FF);
    }

    /**
     * Sets the parameters for the pid controller
     * @param spark the spark max being affected
     * @param p the p value in a PID controller
     * @param i the i value in a PID controller
     * @param d the d value in a PID controller
     * @param f the f (feed forward) value
     */
    private void setPIDF(CANSparkMax spark, double p, double i, double d, double f){
        SparkMaxPIDController pidCon = spark.getPIDController();

        pidCon.setP(p);
        pidCon.setI(i);
        pidCon.setD(d);
        pidCon.setFF(f);
    }

    /**
     * Initializes a single spark max
     */
    public void initSpark(CANSparkMax spark){
        spark.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Gets the current yaw angle of the robot
     */
    public double getPigeonHeading(){
        return this.pigeon.getYaw();
    }

    /**
     * Resets the current yaw angle of the robot such that the way it is currently facing is the new forward direction
     */
    public void resetPigeonHeading(){
        this.pigeon.setYaw(0.0);
    }
}