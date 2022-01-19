package frc.robot.Subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
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

    //Talon rotation motors so we can test with old swerve bot
    public TalonSRX frontRightTalon = new TalonSRX(1);
    public TalonSRX frontLeftTalon = new TalonSRX(2);
    public TalonSRX backLeftTalon = new TalonSRX(3);
    public TalonSRX backRightTalon = new TalonSRX(4);

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
    

    public final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(frontRightLocation, frontLeftLocation, backLeftLocation, backRightLocation);

    public final SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(swerveKinematics, rotation2d);

    //Shooter motor. ids 5, 6(follower)
    public CANSparkMax shooterTopMotor = new CANSparkMax(15, MotorType.kBrushless);
    public CANSparkMax shooterBotomMotor = new CANSparkMax(16, MotorType.kBrushless);
    public CANSparkMax shooterRotationMotor = new CANSparkMax(17, MotorType.kBrushless);

    //Intake motors, ids 7-10(if needed)
    public CANSparkMax rakeMotor = new CANSparkMax(7, MotorType.kBrushless);
    public CANSparkMax leftFeedMotor = new CANSparkMax(8, MotorType.kBrushless);
    public CANSparkMax rightFeedMotor = new CANSparkMax(9, MotorType.kBrushless);
    public CANSparkMax shooterFeedMotor = new CANSparkMax(10, MotorType.kBrushless);


    //Climber motors, ids 15-18. I can't imagine it taking more than 4 motors, nor can I imagine our robot having 18 motors on it
    public CANSparkMax climberOne = new CANSparkMax(5, MotorType.kBrushless);
    public CANSparkMax climberTwo = new CANSparkMax(6, MotorType.kBrushless);

    //Pneumatics, ids 20-25
    public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 20, 21);
    public DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 22, 23);

    public Hardware(Robot robot){
        this.robot = robot;
        //Reset pigeon here, couldn't find the command for it
        
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
        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, Rotation2d.fromDegrees(pigeon.getFusedHeading()))
                : new ChassisSpeeds(xSpeed, ySpeed, rotation)
        );
        setSwerveModuleStates(swerveModuleStates);
    }
    /**
     * 
     * @return array of SwerveModuleState 0 is fl, 1 is fr, 2 is bl, 3 is br
     */
    public SwerveModuleState[] getSwerveModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[0] = frontLeft.getState();
        states[1] = frontRight.getState();
        states[2] = backLeft.getState();
        states[3] = backRight.getState();
        return states;
    }

    /**
     * 
     * @param states an array of the swerve module states. 0 for fl, 1 for fr, 2 for bl, 3 for br 
     */
    public void setSwerveModuleStates(SwerveModuleState[] states){
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);
    }

    /**
     * updates the odometry
     */
    public void updateOdom(){
        swerveOdometry.update(Rotation2d.fromDegrees(pigeon.getFusedHeading()), getSwerveModuleStates());
    }

    public void initTalons(){
        initTalon(frontLeftTalon, 0);
        initTalon(frontRightTalon, 0);
        initTalon(backLeftTalon, 0);
        initTalon(backRightTalon, 0);
    }
    public void initTalon(TalonSRX talon, double offset){
        double startOffset = 0;
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 30);
        startOffset = talon.getSelectedSensorPosition() - offset; 
        Timer.delay(0.3);
        talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        Timer.delay(0.3);

        talon.setSelectedSensorPosition(startOffset, 0, 30);
    }
}
