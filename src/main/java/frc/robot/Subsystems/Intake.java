package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Robot;

public class Intake {
    private Robot robot;

    //Intake motors, ids 7-10(if needed)
    private CANSparkMax intakeMotor = new CANSparkMax(8, MotorType.kBrushless);
    private CANSparkMax shooterFeedMotor = new CANSparkMax(9, MotorType.kBrushless);

    public Intake(Robot robot){
        this.robot = robot;
        shooterFeedMotor.setInverted(true);
    }

    /**
     * Runs intake motors at a value determined by trigger value
     * 
     * @param triggerVal
     */
    public void runIntake(double triggerVal){
        intakeMotor.set(triggerVal);
        shooterFeedMotor.set(-triggerVal);
    }
    /**
     * Runs shooter feed motor to feed shooter
     */
    public void feedShooter(){
        shooterFeedMotor.set(-0.8);
    }

    public void stopFeedShooter(){
        shooterFeedMotor.set(0.0);
    }
}
