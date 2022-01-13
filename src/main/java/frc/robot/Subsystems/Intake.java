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
        final double MAX_INTAKE_SPEED = 20.0;//Meters per second
        triggerVal *= MAX_INTAKE_SPEED;
        robot.hardware.intakeMotor.set(triggerVal);
    }
}
