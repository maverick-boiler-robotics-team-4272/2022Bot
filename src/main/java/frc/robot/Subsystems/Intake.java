package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {

    //Intake motors, ids 7-10(if needed)
    private DigitalInput feedBreakSensor = new DigitalInput(0); //or 4, was 0 before the mxmp board thingy
    private DigitalInput feedBreakSensorTwenty = new DigitalInput(20);
    private DigitalInput feedBreakSensorFourteen = new DigitalInput(14);
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
        if(feedBreakSensor.get() && triggerVal > 0){
            feedShooter(-0.55);
        }else if(triggerVal < 0){
            feedShooter(0.55);
        }else{
            stopFeedShooter();
        }
    }

    /**
     * Runs shooter feed motor 
     */
    public void feedShooter(){
        feedShooter(-0.8);
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
     * Returns state of shooter feed beam break sensor(located towards the top of the shooter feed)
     * @return
     */
    public boolean getFeedSensor(){
        return feedBreakSensor.get();
    }

    /**
     * Sets the intake motor's current limit
     * @param lim
     */
    public void setIntakeCurrentLimit(int lim){
        this.intakeMotor.setSmartCurrentLimit(lim);
    }

    public void testBeamBreaks(){
        System.out.println("14: " + feedBreakSensorFourteen);
        System.out.println("20: " + feedBreakSensorTwenty);
        System.out.println("0: " + feedBreakSensor);
        System.out.println("-------------------");
    }
}
