package frc.robot.Subsystems;

import java.sql.Driver;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;

public class Teleop {
    private Robot robot;
    public Teleop(Robot robot){
        this.robot = robot;
    }

    private XboxController driveController = new XboxController(0);
    private XboxController opController = new XboxController(1);

    public boolean fieldRelative = true;

    public void run(){

        //Drive
        double driveX = (driveController.getLeftX() > robot.JSTICK_DEADZONE)
                        ? driveController.getLeftX()
                        : 0;
        double driveY = (driveController.getLeftY() > robot.JSTICK_DEADZONE)
                        ? driveController.getLeftY()
                        : 0;

        //Field Relative Toggle
        if(driveController.getStartButtonPressed()){
            fieldRelative = !fieldRelative;
        }

        //Shooter
        double shooterVal = (driveController.getRightTriggerAxis() > robot.TRIGGER_DEADZONE)
                            ? driveController.getRightTriggerAxis()
                            : 0;
        robot.shooter.shoot(shooterVal);

        //Intake
        double intakeVal = (opController.getLeftTriggerAxis() > robot.TRIGGER_DEADZONE)
                            ? opController.getLeftTriggerAxis()
                            : 0;
        robot.intake.runIntake(triggerVal);
    }
}
