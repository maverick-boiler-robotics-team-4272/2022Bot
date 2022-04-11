package frc.robot.Subsystems;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Limelight.LEDMode;

public class Teleop {
    //Xbox controllers

    //Currently Used buttons
    //Left Trigger
    //Right Trigger
    //Left Bumper
    //Right Bumper
    //Start Button
    //Back Button
    //A Button
    //B Button
    //X Button
    //Y Button
    //D-pad

    //Currently Unused
    //Right Stick Click
    //Left Stick Click
    private XboxController driveController = new XboxController(0);
    
    //Currently Used Buttons
    //Left Trigger
    //Right Trigger
    //Left Bumber
    //Right Bumper
    //A Button
    //B Button
    //X Button
    //Y Button

    //Currently Unused Buttons
    //Start Button
    //Back Button
    //Right Stick Click
    //Left Stick Click
    private XboxController opController = new XboxController(1);

    public boolean fieldRelative = true;
    private boolean intakeStopped = true;
    private boolean shooterStopped = true;
    private boolean fixingHood = false;
    private boolean finishAuto = false;

    private boolean intakeOverride = false;
    private boolean translating = true;
    private boolean aimed = false;
    
    private Intake intake = Subsystems.getIntake();
    private DriveTrain drivetrain = Subsystems.getDriveTrain();
    private Climber climber = Subsystems.getClimber();
    private Shooter shooter = Subsystems.getShooter();
    private Pneumatics pneumatics = Subsystems.getPneumatics();


    /**
     * Loop that has all of our inputs to run the robot
     */
    public void run(){

        ////////////////////////// Drive ///////////////////////////
        double driveX = driveController.getLeftX();
        double driveY = driveController.getLeftY();

        if(finishAuto){
            shooter.shoot();
            if(driveX + driveY > 0 /*|| !intake.ballPresent()*/){
                finishAuto = false;
                shooter.stopShooter();
            }
            return;
        }

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

        

        if(driveController.getLeftTriggerAxis() > Constants.TRIGGER_DEADZONE){
            Limelight.setLEDMode(LEDMode.ON);

            // 3 point shooter calibration code.
            shooter.setShooter(Limelight.getFlywheelSpeed(), Limelight.getHoodAngle(), -0.8);
            
            rotX = drivetrain.aimAtHub();

            if(Limelight.getAimed() || aimed){
                translating = false;
                aimed = true;
                drivetrain.setXConfig();
                shooter.shoot();
            }else{
                translating = true;
                shooter.revShooter();
            }
        }else if(!driveController.getLeftBumper()){
            Limelight.setLEDMode(LEDMode.OFF);
            aimed = false;
            rotX = driveController.getRightX();
            rotX = Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, rotX);
            translating = true;
        }else{
            aimed = false;
            rotX = driveController.getRightX();
            rotX = Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, rotX);
        }

        if(driveController.getLeftBumperPressed()){
            Limelight.setLEDMode(LEDMode.ON);
            rotX = drivetrain.aimAtHub();

            if(Limelight.getAimed() || aimed){
                aimed = true;
                drivetrain.setXConfig();
                translating = false;
            }else{
                translating = true;
            }
        }else if(driveController.getLeftBumperReleased()){
            translating = true;
            Limelight.setLEDMode(LEDMode.OFF);
        }

        if(translating){
            drivetrain.drive(driveX * Constants.MAX_SPEED, driveY * Constants.MAX_SPEED, rotX * Constants.MAX_ANGULAR_SPEED, fieldRelative);
        }

        ////////////////////// Field Relative Toggle /////////////////////////////
        if(driveController.getStartButtonPressed() /*|| driveController.getAButtonPressed()*/){
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
            pneumatics.toggleIntake();
        }

        if(driveController.getXButtonPressed()){
            shooter.reBurnFlash();
        }
        
        ///////////////////////// Intake ////////////////////
        double opRTrigger = Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, opController.getRightTriggerAxis());
        double opLTrigger = Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, opController.getLeftTriggerAxis());

        if(opController.getAButtonPressed()){
            intakeOverride = !intakeOverride;
            SmartDashboard.putBoolean("intake override", intakeOverride);
        }

        if(opRTrigger > 0){
            intakeStopped = false;
            intake.runIntake(opRTrigger, false, intakeOverride, false);
        }else if(opLTrigger > 0){
            intakeStopped = false;
            intake.runIntake(opLTrigger, true, intakeOverride, false);
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

        Subsystems.getClimber().runClimbers(rClimbSpeed, lClimbSpeed);
        
        if(opController.getLeftBumperPressed()){
            climber.disableSoftLimits();            
        }else if(opController.getLeftBumperReleased()){
            climber.zeroClimbers();
        }

        if(opController.getRightBumperPressed()){
            // shooter.resetPID();
        }

        //////////////////// Pneumatics //////////////////
        if(opController.getYButtonPressed()){
            Subsystems.getPneumatics().toggleClimber();
        }

        if(opController.getBButtonPressed()){
            Subsystems.getPneumatics().toggleIntake();
        }

        if(opController.getXButtonPressed()){
            intake.setIntakeToStuckCurrentLimit();
        }else if(opController.getXButtonReleased()){
            intake.setIntakeToUnStuckCurrentLimit();
        }


        if(driveController.getRightBumper()){
            Subsystems.getIntake().runIntake(0.6, false, false, false);
        }else if(driveController.getRightBumperReleased()){
            intake.stopIntake();
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
            Subsystems.getDriveTrain().resetAimPID();
            Subsystems.getShooter().resetPID();
            Subsystems.getShooter().updateShooter();
        }

        if(driveController.getBackButtonPressed()){
            // Subsystems.getShooter().updateShooter();
            // Subsystems.getShooter().setHood();
            if(fixingHood){
                fixingHood = false;
            }else{
                fixingHood = true;
            }
        }

        if(fixingHood){
            fixingHood = shooter.fixHood();
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

    public void autod(){
        finishAuto = true;
    }
}
