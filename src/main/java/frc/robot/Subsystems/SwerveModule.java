package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Robot;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SwerveModule {
    private Robot robot;
    public SwerveModule(Robot robot){
        this.robot = robot;
    }
    private static final double WHEEL_RADIUS = 0.5;//Don't know actual value yet
    private static final double ENCODER_RESOLUTION = 4096;

    private static final double MAX_SPEED = 3.0;//Meters per second
    private static final double MAX_ANGULAR_VELOCITY = Math.PI;

    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turningEncoder;

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);
    private final SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(0, 0);

    /**
     * Constructs a new swerve module object
     * 
     * @param driveMotor drive motor
     * @param turningMotor turning motor
     * @param driveEndoder drive encoder
     * @param turningEncoder turning encoder
     */
    public SwerveModule(
    CANSparkMax driveMotor, 
    CANSparkMax turningMotor, 
    RelativeEncoder driveEncoder, 
    RelativeEncoder turningEncoder){
        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;
        this.turningMotor = turningMotor;
        this.turningEncoder = turningEncoder;

        driveEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS);
        turningEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS);


    }
}