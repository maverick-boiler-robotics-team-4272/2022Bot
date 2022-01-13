package frc.robot.Subsystems;

import frc.robot.Robot;

public class Shooter {
    private Robot robot;
    public Shooter(Robot robot){
        this.robot = robot;
    }

    private static final double SHOOTER_TRIGGER_DEADZONE = 0.1;


    /**
     * Starts the shooter wheel based on the trigger value. 
     * Has built in deadzone set at {@value #SHOOTER_TRIGGER_DEADZONE} 
     * @param triggerValue amount an xbox trigger is pressed
     */
    public void shoot(double triggerValue){
        //deadzone set in function to keep teleop easier to manage
        if(triggerValue <= SHOOTER_TRIGGER_DEADZONE){
            triggerValue = 0;
        }
        double MAX_SHOOT_SPEED = 20.0;//meters per second
        triggerValue *= MAX_SHOOT_SPEED;//Makes the shooter speed a percent of the max speed
        robot.hardware.shooterMotor.set(triggerValue);
    }
}
