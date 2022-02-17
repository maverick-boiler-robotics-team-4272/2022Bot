package frc.robot.Subsystems;



import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Hardware {
    private Robot robot;
    
    //Constants
    public static final double MAX_SPEED = 3.0;//Meters per second
    public static final double MAX_ACC = 0.5;
    public static final double MAX_ANGULAR_SPEED = 4 * Math.PI;//Half rotation per second
    public static final double WHEEL_DIST = Units.feetToMeters(0.5);
    
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

    //drive motors (ids: 1, 2, 3, 4)
    public CANSparkMax frontRightDrive = new CANSparkMax(11, MotorType.kBrushless);
    public CANSparkMax frontLeftDrive = new CANSparkMax(12, MotorType.kBrushless);
    public CANSparkMax backLeftDrive = new CANSparkMax(13, MotorType.kBrushless);
    public CANSparkMax backRightDrive = new CANSparkMax(14, MotorType.kBrushless);
    
    public RelativeEncoder frontRightDriveEnc = frontRightDrive.getEncoder();
    public RelativeEncoder frontLeftDriveEnc = frontLeftDrive.getEncoder();
    public RelativeEncoder backLeftDriveEnc = backLeftDrive.getEncoder();
    public RelativeEncoder backRightDriveEnc = backRightDrive.getEncoder();

    //rotation motors (ids: 11, 12, 13, 14)
    public CANSparkMax frontRightRotation = new CANSparkMax(1, MotorType.kBrushless);
    public CANSparkMax frontLeftRotation = new CANSparkMax(2, MotorType.kBrushless);
    public CANSparkMax backLeftRotation = new CANSparkMax(3, MotorType.kBrushless);
    public CANSparkMax backRightRotation = new CANSparkMax(4, MotorType.kBrushless);

    //Talon rotation motors so we can test with old swerve bot
    public WPI_TalonSRX frontRightTalon = new WPI_TalonSRX(1);
    public WPI_TalonSRX frontLeftTalon = new WPI_TalonSRX(2);
    public WPI_TalonSRX backLeftTalon = new WPI_TalonSRX(3);
    public WPI_TalonSRX backRightTalon = new WPI_TalonSRX(4);

    //rotation motor encoders
    public RelativeEncoder frontRightEncoder = frontRightRotation.getEncoder();
    public RelativeEncoder frontLeftEncoder = frontLeftRotation.getEncoder();
    public RelativeEncoder backLeftEncoder = backLeftRotation.getEncoder();
    public RelativeEncoder backRightEncoder = backRightRotation.getEncoder();

    public final PigeonIMU pigeon = new PigeonIMU(25);//I don't know how to get a rotation2d object from a pigeon, will have to ask Dannie at some point

    //Shooter motor. ids 5, 6(follower)
    public CANSparkMax shooterTopMotor = new CANSparkMax(15, MotorType.kBrushless);
    public CANSparkMax shooterBottomMotor = new CANSparkMax(16, MotorType.kBrushless);
    public CANSparkMax shooterRotationMotor = new CANSparkMax(17, MotorType.kBrushless);

    //Intake motors, ids 7-10(if needed)
    public CANSparkMax rakeMotor = new CANSparkMax(7, MotorType.kBrushless);
    public CANSparkMax leftFeedMotor = new CANSparkMax(8, MotorType.kBrushless);
    public CANSparkMax rightFeedMotor = new CANSparkMax(9, MotorType.kBrushless);
    public CANSparkMax shooterFeedMotor = new CANSparkMax(10, MotorType.kBrushless);


    //Climber motors, ids 15-18. I can't imagine it taking more than 4 motors, nor can I imagine our robot having 18 motors on it
    public CANSparkMax climberOne = new CANSparkMax(5, MotorType.kBrushless);
    public CANSparkMax climberTwo = new CANSparkMax(6, MotorType.kBrushless);

    // //Pneumatics, ids 20-25
    // public DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 20, 21);
    // public DoubleSolenoid climberSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 22, 23);

    public Hardware(Robot robot){
        this.robot = robot;
        
        //Reset pigeon here, couldn't find the command for it
        
        initSparks();
        this.pigeon.setFusedHeading(0);
        shooterTopMotor.setInverted(true);
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
        
        // talon.setSelectedSensorPosition(0, 0, 0);
    }

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
        //0.0070289
        setPIDF(frontRightDrive, Hardware.FRONT_RIGHT_P, Hardware.FRONT_RIGHT_I, Hardware.FRONT_RIGHT_D, Hardware.FRONT_RIGHT_FF);
        setPIDF(frontLeftDrive, Hardware.FRONT_LEFT_P, Hardware.FRONT_LEFT_I, FRONT_LEFT_D, FRONT_LEFT_FF);
        setPIDF(backRightDrive, Hardware.BACK_RIGHT_P, Hardware.BACK_RIGHT_I, Hardware.BACK_RIGHT_D, Hardware.BACK_RIGHT_FF);
        setPIDF(backLeftDrive, Hardware.BACK_LEFT_P, Hardware.BACK_LEFT_I, Hardware.BACK_LEFT_D, Hardware.BACK_LEFT_FF);
    }

    private void setPIDF(CANSparkMax spark, double p, double i, double d, double f){
        SparkMaxPIDController pidCon = spark.getPIDController();

        pidCon.setP(p);
        pidCon.setI(i);
        pidCon.setD(d);
        pidCon.setFF(f);
    }

    public void initSpark(CANSparkMax spark){
        spark.setIdleMode(IdleMode.kBrake);
    }

    public double getPigeonHeading(){
        return this.pigeon.getFusedHeading();
    }

    public void resetPigeonHeading(){
        this.pigeon.setFusedHeading(0.0);
    }
}
