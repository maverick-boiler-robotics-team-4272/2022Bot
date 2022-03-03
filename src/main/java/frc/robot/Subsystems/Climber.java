package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
public class Climber {
    private Robot robot;


    //Climber motors, ids 15-18. I can't imagine it taking more than 4 motors, nor can I imagine our robot having 18 motors on it
    private CANSparkMax climberLeft = new CANSparkMax(17, MotorType.kBrushless);
    private CANSparkMax climberRight = new CANSparkMax(7, MotorType.kBrushless);

    public Climber(Robot robot){
        this.robot = robot;
    }
    
    /**
     * Runs climber motors with two joystick values
     * @param leftStickAxis
     * @param rightStickAxis
     */
    public void runClimbers(double leftStickAxis, double rightStickAxis){
        climberLeft.set(leftStickAxis);
        climberRight.set(rightStickAxis);
    }
}
