package frc.robot.Subsystems;

import frc.robot.Robot;

public class Shooter {
    private Robot robot;
    public Shooter(Robot robot){
        this.robot = robot;
    }

    

    /**
     * Starts the shooter wheel based on the trigger value
     * @param triggerValue amount an xbox trigger is pressed
     */
    public void shoot(double triggerValue){
        double percentOffSet = 0.8;
        robot.hardware.shooterTopMotor.set(triggerValue * percentOffSet);
        robot.hardware.shooterBottomMotor.set(triggerValue);
    }

    /**
     * Control the shooter motors seperately
     * 
     * @param topMotorVal value for the top motor to run at
     * @param bottomMotorVal value for the bottom motor to run at
     */
    public void shoot(double topMotorVal, double bottomMotorVal){
        robot.hardware.shooterTopMotor.set(topMotorVal);
        robot.hardware.shooterBottomMotor.set(bottomMotorVal);
    }


    /**
     * Assuming that we have a camera this funciton will align the bot with the reflective strips on the hub
     * If, IF, I can figure out how I will have it calculate the perfect position and just auto put itself there, distance, angle, everything, but that's a later issue
     */
    public void cameraAlign(){
        
    }
}
