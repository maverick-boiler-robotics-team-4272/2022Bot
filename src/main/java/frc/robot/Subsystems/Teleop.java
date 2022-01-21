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

    /**
     * Loop that has all of our inputs to run the robot
     */
    public void run(){

        //Drive
        double driveX = driveController.getLeftX();
        double driveY = driveController.getLeftY();
        if(Math.abs(driveX) <= 0.08){
            driveX = 0;
        }
        if(Math.abs(driveY) <= 0.08){
            driveY = 0;
        }
        double initDriveX = driveX;
        double initDriveY = driveY;
        double hyp = Math.sqrt(Math.pow(driveX, 2) + Math.pow(driveY, 2));
        double angle = Math.atan2(driveY, driveX);
        hyp = deadzoneEqautions(Robot.JSTICK_DEADZONE, hyp);

        driveX = Math.cos(angle) * hyp;
        driveY = Math.sin(angle) * hyp;

        double rotX = driveController.getRightX();

        rotX = deadzoneEqautions(Robot.JSTICK_DEADZONE, rotX);

        if(driveController.getRightBumperPressed()){
        System.out.println("initDriveX, initDriveY = " + initDriveX + ", " + initDriveY);

        System.out.println("Hyp = " + hyp);
        System.out.println("Angle = " + angle);
        
        System.out.println("driveX, driveY = " + driveX + ", " + driveY);
            
        }
        // robot.hardware.drive(driveX, driveY, rotX, fieldRelative);

        //Field Relative Toggle
        if(driveController.getStartButtonPressed()){
            String fieldRelativeOnOrNot;
            fieldRelative = !fieldRelative;
            if(fieldRelative){
                fieldRelativeOnOrNot = "On";
            }else{
                fieldRelativeOnOrNot = "Off";
            }
            System.out.println("Field relative is: " + fieldRelativeOnOrNot);

        }

        
        //Shooter
        double shooterTop = deadzoneEqautions(Robot.TRIGGER_DEADZONE, driveController.getLeftTriggerAxis());
        double shooterBottom = deadzoneEqautions(Robot.TRIGGER_DEADZONE, driveController.getRightTriggerAxis());
        double shooterSpeed = deadzoneEqautions(Robot.TRIGGER_DEADZONE, driveController.getRightTriggerAxis());
        //robot.shooter.shoot(shooterTop, shooterBottom);//This is for individually controlling the motors
        robot.shooter.shoot(shooterSpeed);//This is for having it on one trigger and running one wheel slower than the other
                                                //by a set percentage

        //Intake
        double intakeVal = (opController.getLeftTriggerAxis() > Robot.TRIGGER_DEADZONE)
                            ? opController.getLeftTriggerAxis()
                            : 0;
        robot.intake.runIntake(intakeVal);
    }

    /**
     * Calculates a new magnitude of input taking the dead zone into account. This stops it from jumping
     * from 0 to the deadzone value.
     * 
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
