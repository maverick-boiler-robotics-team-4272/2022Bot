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
    public XboxController opController = new XboxController(1);
    private double percentTop = .8;
    private double percentBottom = .8;

    private double flFF = 0.1;
    private double frFF = 0.1;
    private double brFF = 0.1;
    private double blFF = 0.1;

    public boolean fieldRelative = true;

    /**
     * Loop that has all of our inputs to run the robot
     */
    public void run(){

        ////////////////////////// Drive ///////////////////////////
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
        hyp = deadzoneEquations(Robot.JSTICK_DEADZONE, hyp);

        driveX = Math.cos(angle) * hyp;
        driveY = Math.sin(angle) * hyp;

        double rotX;
        if(driveController.getLeftBumper()){
            rotX = robot.driveTrain.aimAtHub();
        }else{
            rotX = driveController.getRightX();
            rotX = deadzoneEquations(Robot.JSTICK_DEADZONE, rotX);
        }
        
        robot.driveTrain.drive(driveX * Robot.MAX_SPEED, driveY * Robot.MAX_SPEED, rotX * Robot.MAX_ANGULAR_SPEED, fieldRelative);

        ////////////////////// Field Relative Toggle /////////////////////////////
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
            robot.driveTrain.resetPigeonHeading();
        }

        if(opController.getAButtonPressed()){
            robot.shooter.resetMotor();
        }
        
        ///////////////////////// Intake ////////////////////
        robot.intake.runIntake(Teleop.deadzoneEquations(Robot.TRIGGER_DEADZONE, opController.getRightTriggerAxis()));
        if(opController.getRightTriggerAxis() < Robot.TRIGGER_DEADZONE && opController.getLeftTriggerAxis() > Robot.TRIGGER_DEADZONE){
            robot.intake.runIntake(-Teleop.deadzoneEquations(Robot.TRIGGER_DEADZONE, opController.getLeftTriggerAxis()));
        }

        ////////////////// Hood/Shooter //////////////////////////
        int pov = driveController.getPOV();
        if(pov >= 0){
            robot.shooter.setShooter(pov);
        }
        robot.shooter.setMotor();

        if(driveController.getLeftTriggerAxis() > Robot.TRIGGER_DEADZONE){
            robot.shooter.shoot();
            robot.shooter.setHood();
            //robot.shooter.setShooter(pov);
        }else{
            robot.shooter.stopShooter();
        }

        if(driveController.getRightBumper()){
            robot.intake.shooterFeedMotor.set(-0.8);
        }

        if(driveController.getYButtonPressed()){
            robot.shooter.resetPID();
        }

        if(driveController.getBackButtonPressed()){
            robot.shooter.updateShooter();
        }

        ////////////////////// Climber //////////////////
        robot.climber.runClimbers(-(Teleop.deadzoneEquations(Robot.JSTICK_DEADZONE, opController.getLeftY())), Teleop.deadzoneEquations(Robot.JSTICK_DEADZONE, opController.getRightY()));
        
        //////////////////// Pneumatics //////////////////
        if(opController.getYButtonPressed()){
            System.out.println("Op Y Button");
            robot.pneumatics.toggleClimber();
        }

        if(opController.getBButtonPressed()){
            System.out.println("Op B Button");

            robot.pneumatics.toggleIntake();
        }

        if(opController.getXButtonPressed()){
            System.out.println("Op X Button");

            robot.pneumatics.toggleClimbSafety();
        }

    }

    /**
     * Calculates a new magnitude of input taking the dead zone into account. This stops it from jumping
     * from 0 to the deadzone value.
     * 
     * @param deadZoneRadius deadzone radius
     * @param hyp magnitude of input
     * @return
     */
    private static double deadzoneEquations(double deadZoneRadius, double hyp){
        if(hyp >= deadZoneRadius){
            return (1/(1-deadZoneRadius)) * (hyp - deadZoneRadius);
        }else if(hyp <= -deadZoneRadius){
            return (1/(1-deadZoneRadius)) * (hyp + deadZoneRadius);
        }

        return 0;
    }
}