package frc.robot.Subsystems;


import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private Robot robot;
    public SwerveModule(Robot robot){
        this.robot = robot;
    }
    private static final double WHEEL_RADIUS = 1.5;// inches
    private static final double ENCODER_RESOLUTION = 4096;

    private static final double MAX_SPEED = 3.0;//Meters per second
    private static final double MAX_ANGULAR_VELOCITY = Math.PI;
    private static final double MAX_ANGULAR_ACC = Math.PI * 2;//Meters per second squared

    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;
    private WPI_TalonSRX talonMotor;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turningEncoder;

    private final PIDController drivePIDController = new PIDController(1, 0, 0);
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACC));

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 1);
    private final SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(1, 1);


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
        turningEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS);//Set distance per pulse using the encoder resolution is what we actually need to do
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Swerve module constructor for talons
     * @param driveMotor CANSparkMax
     * @param turningMotor WPI_TalonSRX
     * @param driveEncoder RelativeEncoder
     */
    public SwerveModule(
        CANSparkMax driveMotor,
        WPI_TalonSRX turningMotor,
        RelativeEncoder driveEncoder){
        this.driveMotor = driveMotor;
        this.talonMotor = turningMotor;
        this.driveEncoder = driveEncoder;
        driveEncoder.setVelocityConversionFactor(2 * Math.PI * Units.inchesToMeters(WHEEL_RADIUS) / 5.25 / 60);
    

        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns swerve module state object
     * @return SwerveModuleState
     */
    public SwerveModuleState getState(){
        if(robot.TALON_BOT){
            return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(((talonMotor.getSelectedSensorPosition()) / 4096.0) % 1 * 360.0));
        }else{
            return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d.fromDegrees(turningEncoder.getPosition() * 360.0));
        }
    }
    /**
     * Sets the desired state of the swerve module
     * @param desiredState
     */
    public void setDesiredState(SwerveModuleState desiredState){
        //Optimize to avoid spinning over 90 degrees, or pi/2 radians
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.getPosition()));

        //Calculate drive output
        final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        final double driveFeedfwd = driveFeedforward.calculate(state.speedMetersPerSecond);

        //Calculate turning motor
        final double turnOutput = turningPIDController.calculate(turningEncoder.getPosition(), state.angle.getRadians());

        final double turnFeedforward = turningFeedforward.calculate(turningPIDController.getSetpoint().velocity);

       
        driveMotor.setVoltage(driveOutput + driveFeedfwd);
        turningMotor.setVoltage(turnOutput + turnFeedforward);
    }

    /**
     * Sets the module to the desired state, this is the one to use for talons
     * @param desired The desired SwerveModuleState
     */
    public void setTalonDesiredState(SwerveModuleState desired){
        //Optimize to avoid spinning over 90 degrees, or pi/2 radians
        SwerveModuleState state = SwerveModuleState.optimize(desired, Rotation2d.fromDegrees(((talonMotor.getSelectedSensorPosition()) / 4096.0) % 1.0 * 360.0));//Again, need the talon conversion rate

        //Calculate drive output
        double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        double driveFeedfwd = driveFeedforward.calculate(state.speedMetersPerSecond);

        //Calculate turning motor
        double turnOutput = turningPIDController.calculate(((talonMotor.getSelectedSensorPosition()) / 4096.0) % 1.0  * (2 * Math.PI)/*Need talon conversion rate from native to meters or
                                something*/, state.angle.getRadians());

        double turnFeedforward = turningFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput + driveFeedfwd);
        talonMotor.set(ControlMode.PercentOutput,turnOutput);

    }
}