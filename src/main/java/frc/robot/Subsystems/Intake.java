package frc.robot.Subsystems;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.AnalogInput;

public class Intake {

    //Intake motors, ids 7-10(if needed)
        //Beam breaks are true for unbroke and false for broken
    private DigitalInput shooterBeamBreak = new DigitalInput(16); //top feed
    private DigitalInput midFeedBeamBreak = new DigitalInput(15); //mid feed
    private DigitalInput lowFeedBeamBreak = new DigitalInput(14); //close to intake
    private DutyCycle lidar = new DutyCycle(new DigitalInput(18));

    private boolean ballInFeed = false;
    private boolean ballInHopper = false;
    private boolean ballsFull = false;

    private boolean ballCheckPoint1 = false;
    

    private CANSparkMax intakeMotor = new CANSparkMax(8, MotorType.kBrushless);
    private CANSparkMax shooterFeedMotor = new CANSparkMax(9, MotorType.kBrushless);

    public Intake(){
        intakeMotor.setSmartCurrentLimit(45);
    }

    /**
     * Runs intake motors at a value determined by trigger value
     * 
     * @param triggerVal
     */
    public void runIntake(double triggerVal){
        // System.out.println("sensor" + getFeedSensor());
        intakeMotor.set(triggerVal);
        boolean botBeam = lowFeedBeamBreak.get();
        boolean midBeam = midFeedBeamBreak.get();
        boolean shooterBeam = shooterBeamBreak.get();
        boolean hopperBeam = (lidar.getOutput() >= 0.09 || lidar.getOutput() <= 0.04);
        //true if unbroken, false if broken
        System.out.println("shooterBeam: " + shooterBeam);
        System.out.println("midBeam: " + midBeam);
        System.out.println("hopperBeam: " + hopperBeam);
        if(shooterBeam){
            feedShooter();
        }else{
            stopFeedShooter();
        }
        
    }

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

    public boolean reverseToLow(boolean lowBeam){
        if(lowBeam){
            reverseFeed(0.2);
            return false;
        }else{
            stopFeedShooter();
            return true;
        }
    }

    public void reverseFeed(double val){
        shooterFeedMotor.set(val);
    }

    public void reverseABit(){

    }

    public void checkBalls(boolean hopperBeam, boolean lowBeam, 
                            boolean midBeam, boolean highBeam){
        int ballCount = 0;
        if(hopperBeam){
            ballInHopper = true;
        }
    }

    public void resetBall(){
        ballInHopper = false;
        ballInFeed = false;
        ballsFull = false;
        ballCheckPoint1 = false;
    }

    public void stopIntake(){
        intakeMotor.set(0.0);
        stopFeedShooter();
    }

    public void testLidar(){
        System.out.println("lidar: " + lidar.getOutput()); // < 0.13 = ball (* 157.48 = inches)
    }
    /**
     * Runs shooter feed motor 
     */
    public void feedShooter(){
        feedShooter(-0.6);
    }

    public void feedShooter(double feedPercent){
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
     * @param lim
     */
    public void setIntakeCurrentLimit(int lim){
        this.intakeMotor.setSmartCurrentLimit(lim);
    }
}
