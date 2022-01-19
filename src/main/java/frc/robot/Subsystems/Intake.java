package frc.robot.Subsystems;

import frc.robot.Robot;

public class Intake {
    private Robot robot;
    public Intake(Robot robot){
        this.robot = robot;
    }

    /**
     * Runs intake motors at a value determined by trigger value
     * 
     * @param triggerVal
     */
    public void runIntake(double triggerVal){
        robot.hardware.rakeMotor.set(triggerVal);
    }
}
