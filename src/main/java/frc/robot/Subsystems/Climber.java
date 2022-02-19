package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Robot;
public class Climber {
    private Robot robot;


    //Climber motors, ids 15-18. I can't imagine it taking more than 4 motors, nor can I imagine our robot having 18 motors on it
    private CANSparkMax climberOne = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax climberTwo = new CANSparkMax(6, MotorType.kBrushless);

    public Climber(Robot robot){
        this.robot = robot;
    }
    
}
