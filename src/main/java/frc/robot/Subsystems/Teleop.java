package frc.robot.Subsystems;

import java.lang.Character.Subset;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;

public class Teleop {
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
    private boolean intakeStopped = true;
    private boolean shooterStopped = true;
    
    private Intake = Subsystems.getIntake();
    private DriveTrain = Subsystems.getDriveTrain();
    private Climber = Subsystems.getClimber();
    private Shooter = Subsystems.getShooter();
    private Pneumatics = Subsystems.getPneumatics();


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
            rotX = Subsystems.getDriveTrain().aimAtHub();
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
            // System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0));
            // rotX = 0.0;
        }else{
            NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            rotX = driveController.getRightX();
            rotX = Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, rotX);
        }
        
        Subsystems.getDriveTrain().drive(driveX * Constants.MAX_SPEED, driveY * Constants.MAX_SPEED, rotX * Constants.MAX_ANGULAR_SPEED, fieldRelative);

        ////////////////////// Field Relative Toggle /////////////////////////////
        if(driveController.getStartButtonPressed() || driveController.getAButtonPressed()){
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
            Subsystems.getDriveTrain().resetPigeonHeading();
        }

        if(driveController.getAButtonPressed()){
            Subsystems.getShooter().fixHood();
        }
        
        ///////////////////////// Intake ////////////////////
        double opRTrigger = Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, opController.getRightTriggerAxis());
        double opLTrigger = Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, opController.getLeftTriggerAxis());

        if(opRTrigger > 0){
            intakeStopped = false;
            Subsystems.getIntake().runIntake(opRTrigger);
        }else if(opLTrigger > 0){
            intakeStopped = false;
            Subsystems.getIntake().runIntake(-opLTrigger);
        }else if(!intakeStopped){
            intakeStopped = true;
            Subsystems.getIntake().stopIntake();

        }

        ////////////////// Hood/Shooter //////////////////////////
        int pov = driveController.getPOV();
        if(pov >= 0){
            Subsystems.getShooter().setShooter(pov);
            Subsystems.getShooter().setHood();
        }
        

        ////////////////////// Climber //////////////////
        double lClimbSpeed = Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, opController.getLeftY());
        double rClimbSpeed = Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, opController.getRightY());


        if(opController.getRightBumper()){
            if(Math.abs(lClimbSpeed) > Math.abs(rClimbSpeed)){
                rClimbSpeed = lClimbSpeed;
            }else{
                lClimbSpeed = rClimbSpeed;
            }
            Subsystems.getClimber().runClimbers(rClimbSpeed, lClimbSpeed);
        }else{
            Subsystems.getClimber().runClimbers(rClimbSpeed, lClimbSpeed);
        }

        //////////////////// Pneumatics //////////////////
        if(opController.getYButtonPressed()){
            Subsystems.getPneumatics().toggleClimber();
        }

        if(opController.getBButtonPressed()){
            Subsystems.getPneumatics().toggleIntake();
        }

        if(opController.getXButtonPressed()){
            Subsystems.getPneumatics().toggleClimbSafety();
        }


        if(driveController.getRightBumper()){
            Subsystems.getIntake().testLidar();
        }

        if(driveController.getRightTriggerAxis() > Constants.TRIGGER_DEADZONE){
            shooterStopped = false;
            Subsystems.getShooter().shoot();
        }else if(driveController.getLeftTriggerAxis() > Constants.TRIGGER_DEADZONE){
            shooterStopped = false;
            Subsystems.getShooter().revShooter();
        }else if(!shooterStopped){
            shooterStopped = true;    
            Subsystems.getShooter().stopShooter();
        }

        if(driveController.getYButtonPressed()){
            Subsystems.getShooter().resetPID();
        }

        if(driveController.getBackButtonPressed()){
            //Subsystems.getShooter().updateShooter();
            //Subsystems.getShooter().setHood();
            
            Subsystems.getShooter().fixHood();
        }

        /*
        if(driveController.getLeftBumperPressed()){
            Subsystems.getDriveTrain().resetAimPID();
        }
        */
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
