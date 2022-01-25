package frc.robot.Subsystems;

import java.sql.Driver;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Robot;

public class Teleop {
    private Robot robot;
    public Teleop(Robot robot){
        this.robot = robot;
    }
    //Xbox controllers
    private XboxController driveController = new XboxController(0);
    private XboxController opController = new XboxController(1);
    private double percentTop = .8;
    private double percentBottom = .8;
    public boolean fieldRelative = false;

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
        // System.out.println("initDriveX, initDriveY = " + initDriveX + ", " + initDriveY);

        // System.out.println("Hyp = " + hyp);
        // System.out.println("Angle = " + angle);
        
        // System.out.println("driveX, driveY = " + driveX + ", " + driveY);
            
        }
        robot.hardware.drive(driveX, driveY, rotX, fieldRelative);
        //robot.hardware.drive(0.125, 0.0, 0.0, false);
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

        //top changing
        if(driveController.getYButtonPressed()){
            percentTop+=0.01;
            System.out.println("Large percent now " + (percentTop * 100.0) + "%");
        }
        if(driveController.getAButtonPressed()){
            percentTop-=0.01;
            System.out.println("Large percent now " + (percentTop * 100.0) + "%");
        }
        if(driveController.getXButtonPressed()){
            percentTop+=0.05;
            System.out.println("Large percent now " + (percentTop * 100.0) + "%");
        }
        if(driveController.getBButtonPressed()){
            percentTop-=0.05;
            System.out.println("Large percent now " + (percentTop * 100.0) + "%");
        }

        //bottom changing
        if(driveController.getPOV() == 0){
            percentBottom+=0.01;
            System.out.println("Small percent now " + (percentBottom * 100.0) + "%");
        }
        if(driveController.getPOV() == 180){
            percentBottom-=0.01;
            System.out.println("Small percent now " + (percentBottom * 100.0) + "%");
        }
        if(driveController.getPOV() == 270){
            percentBottom+=0.05;
            System.out.println("Small percent now " + (percentBottom * 100.0) + "%");
        }
        if(driveController.getPOV() == 90){
            percentBottom-=0.05;
            System.out.println("Small percent now " + (percentBottom * 100.0) + "%");
        }

        if(driveController.getRightBumper()){
            robot.shooter.shoot(percentTop, percentBottom);
            driveController.setRumble(RumbleType.kLeftRumble, 1);
            driveController.setRumble(RumbleType.kRightRumble, 1);
        }
        if(driveController.getRightBumperReleased()){
            robot.shooter.shoot(0,0);
            driveController.setRumble(RumbleType.kLeftRumble, 0);
            driveController.setRumble(RumbleType.kRightRumble, 0);
        }
        // robot.shooter.shoot(shooterSpeed);//This is for having it on one trigger and running one wheel slower than the other
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
