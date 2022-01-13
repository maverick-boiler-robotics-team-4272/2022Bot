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
    private XboxController opCotnroller = new XboxController(1);

    public void run(){

        //Drive
        double driveX = driveController.getLeftX();
        double driveY = driveController.getLeftY();

        //Shooter
        robot.shooter.shoot(driveController.getRightTriggerAxis());

        //Intake
        
    }
}
