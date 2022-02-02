package frc.robot.Subsystems;

import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private static final double WHEEL_RADIUS = 1.5;// inches
    private static final double GEAR_RATIO = 5.25;
    private static final double SECONDS_PER_MINUTE = 60;

    private static final double ENCODER_RESOLUTION = 4096;

    private static final double MAX_SPEED = 3.0;// Meters per second
    private static final double MAX_ANGULAR_VELOCITY = Math.PI;
    private static final double MAX_ANGULAR_ACC = Math.PI * 2;// Meters per second squared

    private static final double RAD_PER_ROT = 2 * Math.PI;

    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;
    private WPI_TalonSRX talonMotor;
    private double talonOffset;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turningEncoder;

    private final PIDController drivePIDController = new PIDController(1, 0, 0);
    private final ProfiledPIDController turningPIDController = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACC));

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.1387, 0.067812);
    private final SimpleMotorFeedforward turningFeedforward = new SimpleMotorFeedforward(1, 1);

    /**
     * Constructs a new swerve module object
     * 
     * @param driveMotor     drive motor
     * @param turningMotor   turning motor
     * @param driveEndoder   drive encoder
     * @param turningEncoder turning encoder
     */
    public SwerveModule(
            CANSparkMax driveMotor,
            CANSparkMax turningMotor,
            RelativeEncoder driveEncoder,
            RelativeEncoder turningEncoder) {
        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;
        this.turningMotor = turningMotor;
        this.turningEncoder = turningEncoder;

        driveEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS);
        turningEncoder.setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS);// Set distance per pulse using the
                                                                               // encoder resolution is what we actually
                                                                               // need to do
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Swerve module constructor for talons
     * 
     * @param driveMotor   CANSparkMax
     * @param turningMotor WPI_TalonSRX
     * @param driveEncoder RelativeEncoder
     */
    public SwerveModule(
            CANSparkMax driveMotor,
            WPI_TalonSRX turningMotor,
            RelativeEncoder driveEncoder,
            double talonOffset) {
        this.driveMotor = driveMotor;
        this.talonMotor = turningMotor;
        this.driveEncoder = driveEncoder;
        this.talonOffset = talonOffset;
        driveEncoder.setVelocityConversionFactor(2 * Math.PI * Units.inchesToMeters(SwerveModule.WHEEL_RADIUS)
                / SwerveModule.GEAR_RATIO / SwerveModule.SECONDS_PER_MINUTE);

        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public double getTalonOffset() {
        return talonOffset;
    }

    /**
     * Returns swerve module state object
     * 
     * @return SwerveModuleState
     */
    public SwerveModuleState getState() {
        if (Robot.TALON_BOT) {
            return new SwerveModuleState(driveEncoder.getVelocity(), Rotation2d
                    .fromDegrees(((talonMotor.getSelectedSensorPosition() - talonOffset) / 4096.0) % 1 * 360.0));
        } else {
            return new SwerveModuleState(driveEncoder.getVelocity(),
                    Rotation2d.fromDegrees(turningEncoder.getPosition() * 360.0));
        }
    }

    /**
     * Sets the desired state of the swerve module
     * 
     * @param desiredState
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize to avoid spinning over 90 degrees, or pi/2 radians
        SwerveModuleState state = SwerveModuleState.optimize(desiredState,
                new Rotation2d(turningEncoder.getPosition()));

        // Calculate drive output
        final double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

        final double driveFeedfwd = driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate turning motor
        final double turnOutput = turningPIDController.calculate(turningEncoder.getPosition(),
                state.angle.getRadians());

        final double turnFeedforward = turningFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput + driveFeedfwd);
        turningMotor.setVoltage(turnOutput + turnFeedforward);
    }

    /**
     * Sets the module to the desired state, this is the one to use for talons
     * 
     * @param desired The desired SwerveModuleState
     */
    public void setTalonDesiredState(SwerveModuleState desired) {
        // Optimize to avoid spinning over 90 degrees, or pi/2 radians
        double current = ((talonMotor.getSelectedSensorPosition() - talonOffset) / 4096.0) * 360.0;
        SwerveModuleState state = SwerveModuleState.optimize(desired,
                Rotation2d.fromDegrees(current));


        driveMotor.getPIDController().setReference(Units.metersToInches(state.speedMetersPerSecond)
                * 60.0 / (3.0 * Math.PI) / 5.25, CANSparkMax.ControlType.kVelocity);

        setHeading(state.angle);
    }

    public Rotation2d getHeading() {
        if (Robot.TALON_BOT) {
            double heading = (this.talonMotor.getSelectedSensorPosition() / 4096.0) * 360.0;
            return Rotation2d.fromDegrees(heading < 0.0 ? heading + 360.0 : heading);
        } else {
            return new Rotation2d(0.0);
        }
    }

    public void setHeading(Rotation2d rotation) {
        double currentHeadingRadians = getHeading().getRadians();
        double setPointHeadingRadians = rotation.getRadians();

        setPointHeadingRadians = optimizeTalons(currentHeadingRadians, setPointHeadingRadians);//Doesn't work btw (2/1/22)
        double otherHeadingRadians = (rotation.getRadians() - Math.PI);
        // if(Math.abs(otherHeadingRadians) > Math.abs(setPointHeadingRadians)){
        //     setPointHeadingRadians = otherHeadingRadians;
        //     //Reverse speed, but just wanna test talons
        // }

        double deltaRadians = setPointHeadingRadians - currentHeadingRadians;
        double deltaPulses = deltaRadians / ((2.0 * Math.PI) / 4096.0);

        double currentPulses = talonMotor.getSelectedSensorPosition();
        double referencePulses = currentPulses + deltaPulses;

        talonMotor.set(ControlMode.MotionMagic, referencePulses);
    }

    public static double optimizeTalons(double currAngle, double desiredAngle){
        double lowerBound;
        double upperBound;
        double modCurrAngle = currAngle % (Math.PI);

        if(modCurrAngle >= 0){
            lowerBound = currAngle - modCurrAngle;
            upperBound = currAngle - (Math.PI + modCurrAngle);
        }else{
            upperBound = currAngle - modCurrAngle;
            lowerBound = currAngle - (360 + modCurrAngle);
        }

        while(desiredAngle < lowerBound){
            desiredAngle += (2 * Math.PI);
        }
        while(desiredAngle > upperBound){
            desiredAngle -= (2 * Math.PI);
        }

        if(desiredAngle - currAngle > Math.PI){
            desiredAngle -= (2 * Math.PI);
        }
        if(desiredAngle - currAngle < Math.PI){
            desiredAngle += (2 * Math.PI);
        }
        return desiredAngle;
    }
}