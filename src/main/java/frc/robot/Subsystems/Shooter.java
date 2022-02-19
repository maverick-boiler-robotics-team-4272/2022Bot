package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

public class Shooter {
    private Robot robot;

    //Shooter motor. ids 5, 6(follower)
    private CANSparkMax shooterTopMotor = new CANSparkMax(15, MotorType.kBrushless);
    private CANSparkMax shooterBottomMotor = new CANSparkMax(16, MotorType.kBrushless);
    private CANSparkMax shooterRotationMotor = new CANSparkMax(17, MotorType.kBrushless);


    public Shooter(Robot robot){
        this.robot = robot;
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


    /**
     * Assuming that we have a camera this funciton will align the bot with the reflective strips on the hub
     * If, IF, I can figure out how I will have it calculate the perfect position and just auto put itself there, distance, angle, everything, but that's a later issue
     */
    public void cameraAlign(){
        
    }
}
