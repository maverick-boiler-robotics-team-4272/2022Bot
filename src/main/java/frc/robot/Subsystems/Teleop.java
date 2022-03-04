package frc.robot.Subsystems;

import java.sql.Driver;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
        hyp = deadzoneEquations(Constants.JSTICK_DEADZONE, hyp);

        driveX = Math.cos(angle) * hyp;
        driveY = Math.sin(angle) * hyp;

        double rotX;
        if(driveController.getLeftBumper()){
            rotX = robot.driveTrain.aimAtHub();
        }else{
            rotX = driveController.getRightX();
            rotX = deadzoneEquations(Constants.JSTICK_DEADZONE, rotX);
        }
        
        robot.driveTrain.drive(driveX * Constants.MAX_SPEED, driveY * Constants.MAX_SPEED, rotX * Constants.MAX_ANGULAR_SPEED, fieldRelative);

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

        if(driveController.getAButtonPressed()){
            robot.shooter.fixHood();
        }
        
        ///////////////////////// Intake ////////////////////
        if(driveController.getRightBumper() || (opController.getRightBumper() && robot.intake.getFeedSensor())){
            robot.intake.feedShooter();
            //robot.intake.runIntake(Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, driveController.getRightTriggerAxis()));
        }else if(opController.getRightTriggerAxis() > Constants.TRIGGER_DEADZONE){
            robot.intake.runIntake(Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, opController.getRightTriggerAxis()));
        }else if(opController.getRightTriggerAxis() < Constants.TRIGGER_DEADZONE && opController.getLeftTriggerAxis() > Constants.TRIGGER_DEADZONE){
            robot.intake.runIntake(-Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, opController.getLeftTriggerAxis()));
        }else{
            robot.intake.runIntake(0.0);
        }

        ////////////////// Hood/Shooter //////////////////////////
        int pov = driveController.getPOV();
        if(pov >= 0){
            robot.shooter.setShooter(pov);
            robot.shooter.setHood();
        }

        if(driveController.getLeftTriggerAxis() > Constants.TRIGGER_DEADZONE){
            robot.shooter.shoot();
            //robot.shooter.setHood();
            //robot.shooter.setShooter(pov);
        }else{
            robot.shooter.stopShooter();
        }

        if(driveController.getYButtonPressed()){
            robot.shooter.resetPID();
        }

        if(driveController.getBackButtonPressed()){//Using for putting hood at zero for shooting wrong colored balls out
            //robot.shooter.updateShooter();
            //robot.shooter.setHood();
            robot.shooter.zeroHood();
        }

        ////////////////////// Climber //////////////////
        
        robot.climber.runClimbers(-(Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, opController.getRightY())), Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, opController.getLeftY()));

        //////////////////// Pneumatics //////////////////
        if(opController.getYButtonPressed() && robot.pneumatics.getIntake() == Value.kForward){
            robot.pneumatics.toggleClimber();
        }

        if(opController.getBButtonPressed()){
            robot.pneumatics.climberUp();
            robot.pneumatics.toggleIntake();
        }

        if(opController.getXButtonPressed()){
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