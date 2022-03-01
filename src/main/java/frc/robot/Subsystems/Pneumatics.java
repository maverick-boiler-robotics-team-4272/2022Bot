package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Pneumatics {

    //Solenoid id's 4/5 do nothing
    private DoubleSolenoid climberSolenoid = new DoubleSolenoid(41, PneumaticsModuleType.REVPH, 1, 0);
    private DoubleSolenoid intakeSolenoid = new DoubleSolenoid(41, PneumaticsModuleType.REVPH, 3, 2);
    private Solenoid climbSafetySolenoid = new Solenoid(41, PneumaticsModuleType.REVPH, 6);
    private DoubleSolenoid extraSolenoids = new DoubleSolenoid(41, PneumaticsModuleType.REVPH, 4, 5);


    public Pneumatics(){
        extraSolenoids.set(Value.kForward);
        intakeSolenoid.set(Value.kReverse);
        climberSolenoid.set(Value.kForward);
        climbSafetySolenoid.set(true);
    }

    public void extendClimber(){
        climberSolenoid.set(Value.kForward);
    }

    public void retractClimber(){
        climberSolenoid.set(Value.kReverse);
    }

    public void toggleClimber(){
        climberSolenoid.toggle();
    }

    public void extendIntake(){
        intakeSolenoid.set(Value.kForward);
    }

    public void retractIntake(){
        intakeSolenoid.set(Value.kReverse);
    }

    public void toggleIntake(){
        intakeSolenoid.toggle();
    }

    public void extendClimbSafety(){
        climbSafetySolenoid.set(false);
    }

    public void retractClimbSafety(){
        climbSafetySolenoid.set(true);
    }

    public void toggleClimbSafety(){
        climbSafetySolenoid.toggle();
    }
}
