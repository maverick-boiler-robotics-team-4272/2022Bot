package frc.robot.Subsystems;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private double flFF = 0.1;
    private double frFF = 0.1;
    private double brFF = 0.1;
    private double blFF = 0.1;

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
        
        robot.hardware.drive(driveX * Hardware.MAX_SPEED, driveY * Hardware.MAX_SPEED, rotX * Hardware.MAX_ANGULAR_SPEED, fieldRelative);
        // robot.hardware.drive(0, -0.1, 0, false);
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

        if(driveController.getBButtonPressed()){
            robot.hardware.resetPigeonHeading();
        }
        
        //Intake
        double intakeVal = (opController.getLeftTriggerAxis() > Robot.TRIGGER_DEADZONE)
                            ? opController.getLeftTriggerAxis()
                            : 0;
        robot.intake.runIntake(intakeVal);
        SmartDashboard.putNumber("Pigeon Heading", robot.hardware.pigeon.getFusedHeading());

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
