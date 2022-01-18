package frc.robot.Subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Robot;

public class Teleop {
    private Robot robot;
    public Teleop(Robot robot){
        this.robot = robot;
    }
    //Xbox controllers
    private XboxController driveController = new XboxController(0);
    private XboxController opController = new XboxController(1);

    public boolean fieldRelative = true;

    public void run(){

        //Drive
        double driveX = driveController.getLeftX();
        double driveY = driveController.getLeftY();
        double hyp = Math.sqrt(Math.pow(driveX, 2) + Math.pow(driveY, 2));
        double angle = Math.atan2(driveY, driveX);
        hyp = deadzoneEqautions(Robot.JSTICK_DEADZONE, hyp);

        driveX = Math.cos(angle) * hyp;
        driveY = Math.sin(angle) * hyp;

        double rotX = driveController.getRightX();

        rotX = deadzoneEqautions(Robot.JSTICK_DEADZONE, rotX);

        robot.hardware.drive(driveX, driveY, rotX, fieldRelative);

        //Field Relative Toggle
        if(driveController.getStartButtonPressed()){
            fieldRelative = !fieldRelative;
        }
        
        //Shooter
        double shooterTop = deadzoneEqautions(Robot.TRIGGER_DEADZONE, driveController.getLeftTriggerAxis());
        double shooterBottom = deadzoneEqautions(Robot.TRIGGER_DEADZONE, driveController.getRightTriggerAxis());
        robot.shooter.shoot(shooterTop, shooterBottom);

        //Intake
        double intakeVal = (opController.getLeftTriggerAxis() > Robot.TRIGGER_DEADZONE)
                            ? opController.getLeftTriggerAxis()
                            : 0;
        robot.intake.runIntake(intakeVal);
    }
    /**
     * Calculates a new magnitude of input taking the dead zone into account. This stops it from jumping
     * from 0 to the deadzone value.
     * @param deadZoneRadius deadzone radius
     * @param hyp magnitude of input
     * @return
     */
    private static double deadzoneEqautions(double deadZoneRadius, double hyp){
        if(hyp >= deadZoneRadius){
            return (1/(1-deadZoneRadius)) * (hyp - deadZoneRadius);
        }else if(hyp <= -deadZoneRadius){
            return (1/(1-deadZoneRadius)) * (hyp + deadZoneRadius);
        }

        return 0;
    }
}
