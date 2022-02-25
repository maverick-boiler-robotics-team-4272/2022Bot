package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Shooter {
    private Robot robot;
    
    //nested array has values [shooter speed, hood angle]
    private double[][] shooterSetpoints = {
        {0, 0},
        {0.25, 0.25},
        {0.5, 0.333},
        {0.75, 0.666}
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
        this.shooterRotationMotor.getEncoder().setPositionConversionFactor(RACK_AND_PINION_RATIO);
        SmartDashboard.putNumber("RPM", 60.0);
        SmartDashboard.putNumber("Rotation Motor P", 0.0001);
        SmartDashboard.putNumber("Rotation Motor I", 0.000001);
        SmartDashboard.putNumber("Rotation Motor D", 20.0);
        SmartDashboard.putNumber("Rotation Motor FF", 0.0);
        SparkMaxPIDController shooterPIDController = this.shooterBottomMotor.getPIDController();
        shooterPIDController.setP(1.0);
        shooterPIDController.setI(0.0);
        shooterPIDController.setD(0.0);
        shooterPIDController.setFF(0.0);
        this.shooterRotationMotor.getEncoder().setPosition(0.0);
        this.shooterTopMotor.setInverted(true);
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
        this.shooterTopMotor.set(topMotorVal);
        this.shooterBottomMotor.set(bottomMotorVal);
    }

    public void resetPID(){
        SparkMaxPIDController controller = shooterRotationMotor.getPIDController();
        controller.setP(SmartDashboard.getNumber("Rotation Motor P", 1.0));
        controller.setI(SmartDashboard.getNumber("Rotation Motor I", 0.0));
        controller.setD(SmartDashboard.getNumber("Rotation Motor D", 0.0));
        controller.setFF(SmartDashboard.getNumber("Rotation Motor FF", 0.0));
        
    }

    public double getHoodPosition(){
        return shooterRotationMotor.getEncoder().getPosition();
    }

    public void setShooter(int pov){
        int index = pov / 90;
        shooterAmt = shooterSetpoints[index][0];
        hoodAmt = shooterSetpoints[index][1];
        SmartDashboard.putNumber("Hood Set Position", hoodAmt);
        shooterRotationMotor.getPIDController().setReference(hoodAmt, ControlType.kSmartMotion);
        // shooterRotationMotor.getPIDController().setReference(-SmartDashboard.getNumber("RPM", 60.0), ControlType.kVelocity);
    }

    /**
     * Assuming that we have a camera this funciton will align the bot with the reflective strips on the hub
     * If, IF, I can figure out how I will have it calculate the perfect position and just auto put itself there, distance, angle, everything, but that's a later issue
     */
    public void cameraAlign(){
        
    }
}
