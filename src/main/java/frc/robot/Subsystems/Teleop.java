package frc.robot.Subsystems;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Limelight.LEDMode;

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
    private boolean fixingHood = false;
    private boolean finishAuto = false;

    private boolean intakeOverride = false;
    private boolean translating = true;
    
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
                shooter.stopShooterAndFeed();
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

        if(driveController.getLeftBumperPressed()){
            Limelight.setLEDMode(LEDMode.ON);
            drivetrain.setXConfig();
            translating = false;
        }else if(driveController.getLeftBumperReleased()){
            translating = true;
            //Limelight.setLEDMode(LEDMode.OFF);
        }

        SmartDashboard.putNumber("Limelight distance", Limelight.getDistanceFeet());

        if(driveController.getLeftTriggerAxis() > Constants.TRIGGER_DEADZONE){
            Limelight.setLEDMode(LEDMode.ON);

            // 3 point shooter calibration code.
            // shooter.setShooter(Limelight.getFlywheelSpeed(), Limelight.getHoodAngle(), -0.5);
            
            rotX = drivetrain.aimAtHub();
            //shooter.revShooter();
            if(Limelight.getAimed()){
                translating = false;
                drivetrain.setXConfig();
                shooter.shoot();
            }else{
                translating = true;
                shooter.revShooter();
            }
        }else{
            //Limelight.setLEDMode(LEDMode.OFF);
            rotX = driveController.getRightX();
            rotX = Teleop.deadzoneEquations(Constants.JSTICK_DEADZONE, rotX);
            translating = true;
        }

        if(translating || driveController.getRightBumper()){
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

        if(driveController.getXButton()){
            intake.runIntake(0.6, false, false, false);
        }else if(driveController.getXButtonReleased()){
            intake.stopIntake();
            intake.stopFeedShooter();
        }
        
        ///////////////////////// Intake ////////////////////
        double opRTrigger = Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, opController.getRightTriggerAxis());
        double opLTrigger = Teleop.deadzoneEquations(Constants.TRIGGER_DEADZONE, opController.getLeftTriggerAxis());

        if(opController.getAButtonPressed()){
            intakeOverride = !intakeOverride;
        }
        SmartDashboard.putBoolean("intake override", intakeOverride);

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

        //////////////////// Pneumatics //////////////////
        if(opController.getYButtonPressed()){
            Subsystems.getPneumatics().toggleClimber();
        }

        if(opController.getBButtonPressed()){
            Subsystems.getPneumatics().toggleIntake();
        }

        if(opController.getXButtonPressed()){
            intake.mechanicalProblemsBeingFixedInCode();
        }else if(opController.getXButtonReleased()){
            intake.mechanicalProblemsFixedInCode();
        }


        if(driveController.getRightBumper()){
            Subsystems.getIntake().runIntake(0.6, false, true, false);
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
