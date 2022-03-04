package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Robot;

public class Intake {
    private Robot robot;

    //Intake motors, ids 7-10(if needed)
    private DigitalInput feedBreakSensor = new DigitalInput(0); //or 4, was 0 before the mxmp board thingy
    private CANSparkMax intakeMotor = new CANSparkMax(8, MotorType.kBrushless);
    private CANSparkMax shooterFeedMotor = new CANSparkMax(9, MotorType.kBrushless);

    public Intake(Robot robot){
        this.robot = robot;
        shooterFeedMotor.setInverted(true);
        intakeMotor.setSmartCurrentLimit(45);
    }

    /**
     * Runs intake motors at a value determined by trigger value
     * 
     * @param triggerVal
     */
    public void runIntake(double triggerVal){
        System.out.println("sensor" + getFeedSensor());
        intakeMotor.set(triggerVal);
        if(feedBreakSensor.get() && triggerVal > 0){
            shooterFeedMotor.set(-0.55);
        }else if(triggerVal < 0){
            shooterFeedMotor.set(0.55);
        }else{
            shooterFeedMotor.set(0);
        }
    }
    /**
     * Runs shooter feed motor 
     */
    public void feedShooter(){
        shooterFeedMotor.set(-0.5);
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
}
