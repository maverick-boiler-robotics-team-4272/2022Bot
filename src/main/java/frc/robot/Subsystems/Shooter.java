package frc.robot.Subsystems;

import frc.robot.Robot;

public class Shooter {
    private Robot robot;
    public Shooter(Robot robot){
        this.robot = robot;
    }

    

    /**
     * Starts the shooter wheel based on the trigger value. 
     * Has built in deadzone
     * @param triggerValue amount an xbox trigger is pressed
     */
    public void shoot(double triggerValue){
        //deadzone set in function to keep teleop easier to manage
        if(triggerValue <= robot.TRIGGER_DEADZONE){
            triggerValue = 0;
        }
        double MAX_SHOOT_SPEED = 20.0;//meters per second
        triggerValue *= MAX_SHOOT_SPEED;//Makes the shooter speed a percent of the max speed
        robot.hardware.shooterMotor.set(triggerValue);
    }

    /**
     * Assuming that we have a camera this funciton will align the bot with the reflective strips on the hub
     * If, IF, I can figure out how I will have it calculate the perfect position and just auto put itself there, distance, angle, everything, but that's a later issue
     */
    public void cameraAlign(){
        
    }
}
