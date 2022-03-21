package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

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
    private boolean b1Low = false;
    private boolean feedShooter = false;
    
    private boolean ballsFull = false;
    

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
        // System.out.println("sensor" + getFeedSensor());
        boolean botBeam = lowFeedBeamBreak.get();
        boolean midBeam = midFeedBeamBreak.get();
        boolean shooterBeam = shooterBeamBreak.get();
        boolean hopperBeam = (lidar.getOutput() >= 0.09 || lidar.getOutput() <= 0.04);
        if(!hopperBeam && !b1){
            triggerVal *= 0.25;
        }else if(!hopperBeam){
            triggerVal = 0;
        }
        intakeMotor.set(triggerVal);
        //true if unbroken, false if broken
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
    
    /**
     * Run the intake and feed, using beam breaks to figure out when to stop
     * @param triggerVal speed to run intake at
     * @param inverted whether the feed motor is inverted
     */
    public void runIntakeComplex(double triggerVal, boolean inverted){
        boolean botBeam = !lowFeedBeamBreak.get();
        boolean midBeam = !midFeedBeamBreak.get();
        boolean shooterBeam = !shooterBeamBreak.get();
        boolean hopperBeam = lidar.getOutput() < 0.1;
        double feedVal = -0.6;

        if(inverted){
            intakeMotor.set(triggerVal);
            shooterFeedMotor.set(-feedVal);
            return;
        }

        if(!(b1 || b2)){
            feedVal = -0.6;
        }else if(b1 && b2){
            if(shooterBeam){
                stopFeedShooter();
                stopIntake();
            }else{
                feedVal *= 0.6;
                triggerVal *= 0.2;
            }
        }

        if(hopperBeam && !b1){
            triggerVal *= 0.2;
        }else if(hopperBeam && b1){
            reverseToLow();
            b2 = true;
        }

        if(shooterBeam){
            b1 = true;
            feedVal = 0;
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
     * Resets all the booleans
     */
    public void resetBall(){
        ballInHopper = false;
        ballInFeed = false;
        ballsFull = false;
        ballCheckPoint1 = false;

        b1 = false;
        b2 = false;
        b1Mid = false;
        b1Low = false;
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
