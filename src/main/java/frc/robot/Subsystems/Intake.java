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
    private Lidar lidar = new Lidar(18);

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
        intakeMotor.burnFlash();
        shooterFeedMotor.burnFlash();        
    }

    /**
     * Run the intake and feed, using beam breaks to figure out when to stop
     * @param triggerVal speed to run intake at
     * @param inverted whether the feed motor is inverted
     */
    public void runIntake(double triggerVal, boolean inverted, boolean override, boolean intakeOnly){

        boolean midBeam = !midFeedBeamBreak.get();
        boolean shooterBeam = !shooterBeamBreak.get();
        boolean hopperBeam = lidar.getRawDutyCycle() < 0.1 && lidar.getRawDutyCycle() > 0.01;
        double feedVal = -0.6;

        if(intakeOnly){
            intakeMotor.set(triggerVal);
            return;
        }

        if(inverted){
            intakeMotor.set(-triggerVal);
            shooterFeedMotor.set(-feedVal);
            return;
        }

        if(override){
            intakeMotor.set(triggerVal);
            if(midBeam){
                shooterFeedMotor.set(0.0);
            }else{
                shooterFeedMotor.set(feedVal);
            }
            return;
        }

        //if no ball then just run norms
        //if ball in hopper, slow intake speed to allow feed succ
        //if ball in feed and not hitting mid beam, run norm speed
        //if ball in feed and hit mid beam, slow feed
        //run feed til ball hit shooter beam
        //only run intake

        if(midBeam && !b1Mid){
            b1Mid = true;
            b1 = true;
        }else if(b1Mid && midBeam){
            feedVal = -0.3;
        }else if(b1Mid && !midBeam){
            
            feedVal = 0;
        }

        if(b1 && hopperBeam){
            b2 = true;
        }

        if(hopperBeam){
            triggerVal = 0.2;
        }
        
        intakeMotor.set(triggerVal);

        shooterFeedMotor.set(feedVal);
        
    }
    
    public void beamBreaksToSmart(){

        boolean botBeam = !lowFeedBeamBreak.get();
        boolean midBeam = !midFeedBeamBreak.get();
        boolean shooterBeam = !shooterBeamBreak.get();
        boolean hopperBeam = lidar.getRawDutyCycle() < 0.08 && lidar.getRawDutyCycle() > 0.01;

        SmartDashboard.putNumber("hopBeamVal", lidar.getRawDutyCycle());
        SmartDashboard.putBoolean("botBeam", botBeam);
        SmartDashboard.putBoolean("midBeam", midBeam);
        SmartDashboard.putBoolean("shooterBeam", shooterBeam);
        SmartDashboard.putBoolean("hopperBeam", hopperBeam);
    }
    

    public int getBallCount(){
        int count = 0;

        if(b1){
            count++;
        }
        if(b2){
            count++;
        }

        return count;
    }

    public void shotBall(){
        if(b1 && b2){
            b1 = false;
        }else if(!b1 && b2){
            b2 = false;
        }
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
        boolean hopperBeam = lidar.getRawDutyCycle() < 0.12 && lidar.getRawDutyCycle() > 0.01;

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
        intakeMotor.set(0.15);
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

    public void mechanicalProblemsBeingFixedInCode(){
        setIntakeCurrentLimit(80);
    }

    public void mechanicalProblemsFixedInCode(){
        setIntakeCurrentLimit(45);
    }

    public boolean getB1(){
        return b1;
    }

    public boolean getB2(){
        return b2;
    }
}
