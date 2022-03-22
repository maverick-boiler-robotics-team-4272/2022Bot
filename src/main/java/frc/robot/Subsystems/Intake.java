package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

    //Intake motors, ids 7-10(if needed)
        //Beam breaks are true for unbroke and false for broken
    private DigitalInput shooterBeamBreak = new DigitalInput(16); //top feed
    private DigitalInput midFeedBeamBreak = new DigitalInput(15); //mid feed
    private DigitalInput lowFeedBeamBreak = new DigitalInput(14); //close to intake
    private DutyCycle lidar = new DutyCycle(new DigitalInput(18));

    private boolean ballInFeed = false;
    private boolean ballInHopper = false;

    private boolean ballCheckPoint1 = false;
    
    //runIntakeComplex
    private boolean b1 = false;
    private boolean b2 = false;
    private boolean b1Mid = false;
    

    private CANSparkMax intakeMotor = new CANSparkMax(8, MotorType.kBrushless);
    private CANSparkMax shooterFeedMotor = new CANSparkMax(9, MotorType.kBrushless);

    public Intake(){
        intakeMotor.setSmartCurrentLimit(45);
        shooterFeedMotor.setSmartCurrentLimit(45);
        shooterFeedMotor.setOpenLoopRampRate(0.5);
    }

    /**
     * Runs intake motors at a value determined by trigger value
     * 
     * @param triggerVal
     */
    public void runIntake(double triggerVal){

        boolean botBeam = lowFeedBeamBreak.get();
        boolean midBeam = midFeedBeamBreak.get();
        boolean shooterBeam = shooterBeamBreak.get();
        boolean hopperBeam = (lidar.getOutput() >= 0.09 || lidar.getOutput() <= 0.04);
        //true if unbroken, false if broken

        if(!hopperBeam && !b1){
            triggerVal *= 0.25;
        }else if(!hopperBeam){
            triggerVal = 0;
        }

        intakeMotor.set(triggerVal);

        if(shooterBeam){
            b1 = false;
            feedShooter();
        }else{
            b1 = true;
            stopFeedShooter();
        }
        
    }

    /**
     * Function to only run the intake, and not the feed
     * @param val speed to run the intake at
     */
    public void runIntakeOnly(double val){
        intakeMotor.set(val);
    }
    
    public void beamBreaksToSmart(){

        boolean botBeam = !lowFeedBeamBreak.get();
        boolean midBeam = !midFeedBeamBreak.get();
        boolean shooterBeam = !shooterBeamBreak.get();
        boolean hopperBeam = lidar.getOutput() < 0.08 && lidar.getOutput() > 0.01;

        SmartDashboard.putNumber("hopBeamVal", lidar.getOutput());
        SmartDashboard.putBoolean("botBeam", botBeam);
        SmartDashboard.putBoolean("midBeam", midBeam);
        SmartDashboard.putBoolean("shooterBeam", shooterBeam);
        SmartDashboard.putBoolean("hopperBeam", hopperBeam);
    }
    /**
     * Run the intake and feed, using beam breaks to figure out when to stop
     * @param triggerVal speed to run intake at
     * @param inverted whether the feed motor is inverted
     */
    public void runIntakeComplex(double triggerVal, boolean inverted){

        boolean midBeam = !midFeedBeamBreak.get();
        boolean shooterBeam = !shooterBeamBreak.get();
        boolean hopperBeam = lidar.getOutput() < 0.1;
        double feedVal = -0.6;

        if(inverted){
            intakeMotor.set(-triggerVal);
            shooterFeedMotor.set(-feedVal);
            return;
        }

        //intake normally if no balls
        //if there is a ball in hopper then slow down intake to allow feed to grab ball
        //if ball is in feed go up to shooter beam then back up to mid to make sure the ball doesn't jam the intake
        //then slowly SLOWLY run the feed to try and get both balls in the feed until the shooter beam is set off

        if(hopperBeam){//slow down intake if ball in hopper
            triggerVal *= 0.2;
        }
        
        if(shooterBeam && !b1){//detects ball at shooter beam and sets b1 to true
            b1 = true;
        }

        if(b1 && !b1Mid && !midBeam){//ball at shooter reverse feed til midBeam
            feedVal = 0.3;
        }else if(b1 && !b1Mid && midBeam){
            b1Mid = true;
            feedVal = 0;
        }

        if(b1Mid && !b2 && hopperBeam){ //if b1 has been put to mid and b2 doesn't exist yet and there's a ball in the hopper then b2 is true
            b2 = true;
        }else if(b1Mid && b2){ //if b1 is mid and b2 is there then just move feed slowly
            triggerVal *= 0.15;
            feedVal = -0.2;
        }else if(b1Mid){
            feedVal = 0;
        }

        if(shooterBeam && b1 && b2){
            feedVal = 0;
            triggerVal = 0;
        }

        shooterFeedMotor.set(feedVal);

        intakeMotor.set(triggerVal);
        
    }

    /**
     * Reverses the feed motor until the middle beam break is not triggered
     * @return whether the mid beam is triggered or not
     */
    public boolean reverseToMid(){
        boolean midBeam = midFeedBeamBreak.get();
        if(midBeam){
            reverseFeed(0.5);
            return false;
        }else{
            stopFeedShooter();
            return true;
        }
    }

    /**
     * Reverses the feed until the low beam break is not hit
     * @return whether the low beam is triggered or not
     */
    public boolean reverseToLow(){
        boolean lowBeam = lowFeedBeamBreak.get();
        if(lowBeam){
            reverseFeed(0.2);
            return false;
        }else{
            stopFeedShooter();
            return true;
        }
    }

    /**
     * Reverses the feed motor
     * @param val speed to run the motor at
     */
    public void reverseFeed(double val){
        shooterFeedMotor.set(val);
    }

    /**
     * Reverses the feed motor a little bit, to make sure we can rev the shooter
     */
    public void reverseABit(){
        double initEnc = shooterFeedMotor.getEncoder().getPosition();
        double difference = 2;
        shooterFeedMotor.getPIDController().setReference(initEnc + difference, ControlType.kPosition);
    }

    /**
     * 
     * @return true if a ball is in the robot anywhere there's a sensor, false otherwise
     */
    public boolean ballPresent(){
        
        boolean botBeam = !lowFeedBeamBreak.get();
        boolean midBeam = !midFeedBeamBreak.get();
        boolean shooterBeam = !shooterBeamBreak.get();
        boolean hopperBeam = lidar.getOutput() < 0.12 && lidar.getOutput() > 0.01;

        return (botBeam || midBeam || shooterBeam || hopperBeam);
    }

    /**
     * Resets all the booleans
     */
    public void resetBall(){
        b1 = false;
        b2 = false;
        b1Mid = false;
    }

    /**
     * Completely stops the intake
     */
    public void stopIntake(){
        intakeMotor.set(0.0);
        stopFeedShooter();
    }

    /**
     * Runs shooter feed motor 
     */
    public void feedShooter(){
        feedShooter(-0.6);
    }

    /**
     * Runs the shooter feed motor 
     * @param feedPercent speed to run it at
     */
    public void feedShooter(double feedPercent){
        shooterFeedMotor.set(feedPercent);
    }

    /**
     * Gets the current value of the beam break underneath the shooter
     * @return whether the beam break is tripped or not
     */
    public boolean getShooterBeam(){
        return !(shooterBeamBreak.get());
    }

    /**
     * Stops shooter feed motor
     */
    public void stopFeedShooter(){
        shooterFeedMotor.set(0.0);
    }


    /**
     * Sets the intake motor's current limit
     * @param lim the current limit
     */
    public void setIntakeCurrentLimit(int lim){
        this.intakeMotor.setSmartCurrentLimit(lim);
    }
}
