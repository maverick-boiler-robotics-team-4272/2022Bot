package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;

public class Intake {
    private Robot robot;

    //Intake motors, ids 7-10(if needed)
    private CANSparkMax rakeMotor = new CANSparkMax(8, MotorType.kBrushless);
    private CANSparkMax leftFeedMotor = new CANSparkMax(6, MotorType.kBrushless);
    private CANSparkMax rightFeedMotor = new CANSparkMax(16, MotorType.kBrushless);
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
        this.rakeMotor.set(triggerVal);
    }
}
