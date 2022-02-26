package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Shooter {
    private Robot robot;
    
    //nested array has values [shooter speed, hood angle]
    private double[][] shooterSetpoints = {
        {0, 0},
        {0.25, -10},
        {0.5, -15},
        {0.75, -20}
    };

    private static final double HOOD_MOTOR_GEAR_RATIO = 1.0 / 12.0;
    private static final double RACK_AND_PINION_RATIO = HOOD_MOTOR_GEAR_RATIO / 2.5;

    private double hoodAmt;
    private double shooterAmt;
    //Shooter motor. ids 5, 6(follower)
    private CANSparkMax shooterTopMotor = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax shooterBottomMotor = new CANSparkMax(15, MotorType.kBrushless);
    private CANSparkMax shooterRotationMotor = new CANSparkMax(10, MotorType.kBrushless);


    public Shooter(Robot robot){
        this.robot = robot;
        this.shooterRotationMotor.getEncoder().setPositionConversionFactor(1);
        SmartDashboard.putNumber("RPM", 60.0);
        SmartDashboard.putNumber("Rotation Motor P", 0.0000001);
        SmartDashboard.putNumber("Rotation Motor I", 0.0);
        SmartDashboard.putNumber("Rotation Motor D", 0.0);
        SmartDashboard.putNumber("Rotation Motor F", 0.0001);
        SmartDashboard.putNumber("Rotation Motor ACC", 40000.0);
        SmartDashboard.putNumber("Rotation Motor MAX VEL", 40000.0);
        SparkMaxPIDController shooterPIDController = this.shooterRotationMotor.getPIDController();
        shooterPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        shooterPIDController.setP(0.0000001);
        shooterPIDController.setI(0.0);
        shooterPIDController.setD(0.0);
        shooterPIDController.setFF(0.0001);
        shooterPIDController.setSmartMotionMaxAccel(40000.0, 0);
        shooterPIDController.setSmartMotionMaxVelocity(40000.0, 0);
        shooterPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
        shooterPIDController.setSmartMotionAllowedClosedLoopError(0.0, 0);
        this.shooterRotationMotor.getEncoder().setPosition(0.0);
        this.shooterTopMotor.setInverted(true);
    }

    public void resetMotor(){
        shooterRotationMotor.getEncoder().setPosition(0.0);
    }

    public void resetPID(){
        SparkMaxPIDController controller = shooterRotationMotor.getPIDController();
        controller.setP(SmartDashboard.getNumber("Rotation Motor P", 0.0001));
        controller.setI(SmartDashboard.getNumber("Rotation Motor I", 0.0));
        controller.setD(SmartDashboard.getNumber("Rotation Motor D", 0.0));
        controller.setFF(SmartDashboard.getNumber("Rotation Motor F", 0.001));
        controller.setSmartMotionMaxAccel(SmartDashboard.getNumber("Rotation Motor ACC", 100.0), 0);
        controller.setSmartMotionMaxVelocity(SmartDashboard.getNumber("Rotation Motor MAX VEL", 300.0), 0);
    }

    /**
     * Starts the shooter wheel based on the trigger value
     * @param triggerValue amount an xbox trigger is pressed
     */
    public void shoot(double triggerValue){
        double percentOffSet = 0.8;
        this.shooterTopMotor.set(triggerValue * percentOffSet);
        this.shooterBottomMotor.set(triggerValue);
    }

    public double getShooterAmount(){
        return shooterAmt;
    }
    /**
     * Control the shooter motors seperately
     * 
     * @param topMotorVal value for the top motor to run at
     * @param bottomMotorVal value for the bottom motor to run at
     */
    public void shoot(double topMotorVal, double bottomMotorVal){
        this.shooterTopMotor.set(-topMotorVal);
        this.shooterBottomMotor.set(-bottomMotorVal);
    }

    public void putHoodDataToDashboard(){
        SmartDashboard.putNumber("Hood Position", shooterRotationMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Hood Velocity", shooterRotationMotor.getEncoder().getVelocity());
    }

    public void setShooter(int pov){
        int index = pov / 90;
        shooterAmt = shooterSetpoints[index][0];
        hoodAmt = shooterSetpoints[index][1];
        SmartDashboard.putNumber("Hood Set Position", hoodAmt);
        setMotor();
        // shooterRotationMotor.getPIDController().setReference(-SmartDashboard.getNumber("RPM", 60.0), ControlType.kVelocity);
    }

    public void setMotor(){
        shooterRotationMotor.getPIDController().setReference(hoodAmt, ControlType.kPosition);
    }

    /**
     * Assuming that we have a camera this funciton will align the bot with the reflective strips on the hub
     * If, IF, I can figure out how I will have it calculate the perfect position and just auto put itself there, distance, angle, everything, but that's a later issue
     */
    public void cameraAlign(){
        
    }
}
