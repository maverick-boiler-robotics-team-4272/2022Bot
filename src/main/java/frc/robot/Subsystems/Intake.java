package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {

    //Intake motors, ids 7-10(if needed)
        //Beam breaks are true for unbroke and false for broken
    private DigitalInput shooterBeamBreak = new DigitalInput(16); //top feed
    private DigitalInput midFeedBeamBreak = new DigitalInput(15); //mid feed
    private DigitalInput lowFeedBeamBreak = new DigitalInput(14); //close to intake
    private Lidar lidar17 = new Lidar(17);
    private Lidar lidar18 = new Lidar(18);
    private Lidar lidar19 = new Lidar(19);
    
    //run intake booleans
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
     * @param override intake override to not use hopperbeam
     * @param intakeOnly to run just the intake and not the feed
     */
    public void runIntake(double triggerVal, boolean inverted, boolean override, boolean intakeOnly){

        boolean midBeam = !midFeedBeamBreak.get();
        boolean shooterBeam = !shooterBeamBreak.get();
        boolean hopperBeam = false;
        double feedVal = -0.6;

        if(intakeOnly){
            intakeMotor.set(triggerVal);
            return;
        }

        if(inverted){
            intakeMotor.set(-triggerVal);
            shooterFeedMotor.set(-feedVal);
            resetBall();
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

        //////pre 4/5 code//////
        /*
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
        */

        /*
        if(shooterBeam && !b1Mid){
            b1Mid = true;
        }

        if(b1Mid && !b1 && !midBeam){
            feedVal = 0.3;
        }else if(b1Mid && !b1 && midBeam){
            b1 = true;
            feedVal = 0;
        }

        if(b1){
            feedVal = 0;
        }

        if(b1 && hopperBeam){
            b2 = true;
        }else if(hopperBeam){
            triggerVal = 0.2;
        }

        if(b1 && b2){
            feedVal = 0;
            triggerVal = 0;
        }

        if(midBeam && !b1Mid){
            b1Mid = true;
        }
        */

        if(b1Mid && !b1 && midBeam){
            feedVal = 0.3;
        }else if(b1Mid && !b1 && !midBeam){
            feedVal = 0;
            b1 = true;
        }

        if(b1){
            feedVal = 0;
        }

        if(b1 && !b2 && hopperBeam){
            b2 = true;
        }


        intakeMotor.set(triggerVal);

        shooterFeedMotor.set(feedVal);
        
    }
    
    /**
     * 
     */
    public void beamBreaksToSmart(){

        boolean botBeam = !lowFeedBeamBreak.get();
        boolean midBeam = !midFeedBeamBreak.get();
        boolean shooterBeam = !shooterBeamBreak.get();

        boolean hopperBeam17 = lidar17.getRawDutyCycle() < 0.08 && lidar17.getRawDutyCycle() > 0.01;

        Constants.TUNING_TABLE.putNumber("hopBeamVal17", lidar17.getRawDutyCycle());

        boolean hopperBeam18 = lidar18.getRawDutyCycle() < 0.08 && lidar18.getRawDutyCycle() > 0.01;

        Constants.TUNING_TABLE.putNumber("hopBeamVal18", lidar18.getRawDutyCycle());

        boolean hopperBeam19 = lidar19.getRawDutyCycle() < 0.08 && lidar19.getRawDutyCycle() > 0.01;

        Constants.TUNING_TABLE.putNumber("hopBeamVal19", lidar19.getRawDutyCycle());

        Constants.TUNING_TABLE.putBoolean("botBeam", botBeam);
        Constants.TUNING_TABLE.putBoolean("midBeam", midBeam);
        Constants.TUNING_TABLE.putBoolean("shooterBeam", shooterBeam);
        Constants.TUNING_TABLE.putBoolean("hopperBeam", hopperBeam17);
        Constants.TUNING_TABLE.putBoolean("B1", b1);
        Constants.TUNING_TABLE.putBoolean("B2", b2);
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
        boolean hopperBeam = false;
        
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

    public void setIntakeToStuckCurrentLimit(){
        setIntakeCurrentLimit(Constants.INTAKE_STUCK_CURRENT_LIMIT);
    }

    public void setIntakeToUnStuckCurrentLimit(){
        setIntakeCurrentLimit(Constants.INTAKE_UN_STUCK_CURRENT_LIMIT);
    }

    public boolean getB1(){
        return b1;
    }

    public boolean getB2(){
        return b2;
    }

    /**
     * Gets the current value of the beam break underneath the shooter
     * @return whether the beam break is tripped or not
     */
    public boolean getShooterBeam(){
        return !(shooterBeamBreak.get());
    }

    public boolean getMidBeam(){
        return !(midFeedBeamBreak.get());
    }

    public boolean getLowBeam(){
        return !(lowFeedBeamBreak.get());
    }

    public boolean getHopperBeam(){
        return false;
        
        /*
        //return (lidar.getRawDutyCycle() < 0.1 && lidar.getRawDutyCycle() > 0.01)||
                (lidar.getRawDutyCycle() < 0.1 && lidar.getRawDutyCycle() > 0.01) ||
                (lidar.getRawDutyCycle() < 0.1 && lidar.getRawDutyCycle() > 0.01);
        */
    }
}
