package frc.robot.Subsystems;

import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import org.ejml.dense.row.mult.SubmatrixOps_FDRM;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private static final double TALON_WHEEL_RADIUS = 1.5;// inches
    private static final double TALON_GEAR_RATIO = 5.25;
    private static final double SECONDS_PER_MINUTE = 60;

    private static final double NEO_WHEEL_RADIUS = 2.0;
    private static final double NEO_DRIVE_GEAR_RATIO = 6.75;
    private static final double NEO_STEER_GEAR_RATIO = 150.0 / 7.0;

    private static final double ENCODER_RESOLUTION = 4096;

    private static final double MAX_SPEED = 4.0;// Meters per second
    private static final double MAX_ANGULAR_VELOCITY = Math.PI;
    private static final double MAX_ANGULAR_ACC = Math.PI * 2;// Meters per second squared

    private static final double RAD_PER_ROT = 2 * Math.PI;

    private CANSparkMax driveMotor;
    private CANSparkMax turningMotor;
    private WPI_TalonSRX talonMotor;
    private double offset;
    private CANCoder canCoder;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turningEncoder;

    private final int moduleId;

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
            RelativeEncoder turningEncoder,
            double offset,
            CANCoder canCoder,
            int moduleId) {
        this.driveMotor = driveMotor;
        this.driveEncoder = driveEncoder;
        this.turningMotor = turningMotor;
        this.turningEncoder = turningEncoder;
        this.offset = offset;
        this.canCoder = canCoder;
        this.moduleId = moduleId;

        driveEncoder.setVelocityConversionFactor(SwerveModule.RAD_PER_ROT * Units.inchesToMeters(SwerveModule.NEO_WHEEL_RADIUS)
        / SwerveModule.NEO_DRIVE_GEAR_RATIO / SwerveModule.SECONDS_PER_MINUTE);
        turningEncoder.setPositionConversionFactor(360.0 / SwerveModule.NEO_STEER_GEAR_RATIO);
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
            double offset,
            int moduleId) {
        this.driveMotor = driveMotor;
        this.talonMotor = turningMotor;
        this.driveEncoder = driveEncoder;
        this.offset = offset;
        this.moduleId = moduleId;
        driveEncoder.setVelocityConversionFactor(SwerveModule.RAD_PER_ROT * Units.inchesToMeters(SwerveModule.TALON_WHEEL_RADIUS)
        / SwerveModule.TALON_GEAR_RATIO / SwerveModule.SECONDS_PER_MINUTE);
    }

    public double getOffset() {
        return offset;
    }

    /**
     * Returns swerve module state object
     * 
     * @return SwerveModuleState
     */
    public SwerveModuleState getState() {
        if (Robot.TALON_BOT) {
            return new SwerveModuleState(-driveEncoder.getVelocity(), Rotation2d
                    .fromDegrees(getHeading().getDegrees() % 1.0));
        } else {
            return new SwerveModuleState(driveEncoder.getVelocity(),
                    getHeading());
        }
    }

    /**
     * Sets the desired state of the swerve module
     * 
     * @param desiredState
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize to avoid spinning over 90 degrees, or pi/2 radians
        SwerveModuleState state = SwerveModule.optimize(desiredState,
                getHeading());
        driveMotor.getPIDController().setReference(Units.metersToInches(state.speedMetersPerSecond)
        * SwerveModule.SECONDS_PER_MINUTE / (2 * SwerveModule.NEO_WHEEL_RADIUS * Math.PI) / SwerveModule.NEO_DRIVE_GEAR_RATIO, ControlType.kVelocity);   

        if(state.speedMetersPerSecond != 0.0){
            turningMotor.getPIDController().setReference(state.angle.getDegrees(), ControlType.kPosition);
        }
    }

    /**
     * Sets the module to the desired state, this is the one to use for talons
     * 
     * @param desired The desired SwerveModuleState
     */
    public void setTalonDesiredState(SwerveModuleState desired) {
        // Optimize to avoid spinning over 90 degrees, or pi/2 radians
        SwerveModuleState state = SwerveModule.optimize(desired,
                getHeading());

        driveMotor.getPIDController().setReference(Units.metersToInches(state.speedMetersPerSecond)
         * SwerveModule.SECONDS_PER_MINUTE / (2 * SwerveModule.TALON_WHEEL_RADIUS * Math.PI) / SwerveModule.TALON_GEAR_RATIO, CANSparkMax.ControlType.kVelocity);

        setHeading(state);
    }

    public Rotation2d getHeading() {
        if (Robot.TALON_BOT) {
            double heading = (talonMotor.getSelectedSensorPosition() / 4096.0) * 360.0;
            return Rotation2d.fromDegrees(heading);
        } else {
            return Rotation2d.fromDegrees(turningEncoder.getPosition());
        }
    }

    public void setHeading(SwerveModuleState state) {
        if(state.speedMetersPerSecond == 0.0) return;
        double currentHeadingRadians = getHeading().getRadians();
        double setPointHeadingRadians = state.angle.getRadians();

        // System.out.printf("Module %d: Set Points Equal?: %b\n", this.moduleId, (Math.abs(optimizedSetPoint - setPointHeadingRadians) < 1e-4));
        double deltaRadians = setPointHeadingRadians - currentHeadingRadians;
        double deltaPulses = deltaRadians / ((2.0 * Math.PI) / 4096.0);

        double currentPulses = talonMotor.getSelectedSensorPosition();
        double referencePulses = currentPulses + deltaPulses;

        talonMotor.set(ControlMode.MotionMagic, referencePulses);
    }

    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        boolean inverted = false;

        double desiredDegrees = desiredState.angle.getDegrees() % 360.0;
        if(desiredDegrees < 0.0){
            desiredDegrees += 360.0;
        }

        double currentDegrees = currentAngle.getDegrees();
        double currentMod = currentDegrees % 360.0;
        if(currentMod < 0.0){
            currentMod += 360.0;
        }

        if(Math.abs(currentMod - desiredDegrees) > 90.0 && Math.abs(currentMod - desiredDegrees) <= 270.0){
            inverted = true;
            desiredDegrees -= 180.0;
        }

        double deltaAngle = desiredDegrees - currentMod;
        if(deltaAngle < 0.0){
            deltaAngle += 360.0;
        }

        double counterClockWiseAngle = deltaAngle;
        double clockWiseAngle = deltaAngle - 360.0;

        if(Math.abs(counterClockWiseAngle) < Math.abs(clockWiseAngle)){
            desiredDegrees = counterClockWiseAngle;
        }else{
            desiredDegrees = clockWiseAngle;
        }

        double magnitude = desiredState.speedMetersPerSecond;

        if(inverted){
            magnitude *= -1.0;
        }

        desiredDegrees += currentDegrees;

        return new SwerveModuleState(magnitude, Rotation2d.fromDegrees(desiredDegrees));
    }
}