package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
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
        this.shooterRotationMotor.getPIDController().setP(1.0);  
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

    public void setShooter(int pov){
        int index = pov / 90;
        shooterAmt = shooterSetpoints[index][0];
        hoodAmt = shooterSetpoints[index][1];
        shooterRotationMotor.getPIDController().setReference(hoodAmt, ControlType.kSmartMotion);
    }

    /**
     * Assuming that we have a camera this funciton will align the bot with the reflective strips on the hub
     * If, IF, I can figure out how I will have it calculate the perfect position and just auto put itself there, distance, angle, everything, but that's a later issue
     */
    public void cameraAlign(){
        
    }
}
