package frc.robot.Auto;

import frc.robot.Subsystems.Constants;

public class Setpoint {
    private double time;
    private double hoodPosition;
    private double velocity;
    private boolean shooter;

    public Setpoint(double time, double hoodPosition, double shooterVelocity){
        this.time = time;
        this.hoodPosition = Math.max(Constants.HOOD_MIN, Math.min(hoodPosition, Constants.HOOD_MAX));
        this.velocity = Math.max(Constants.SHOOTER_MIN, Math.min(shooterVelocity, Constants.SHOOTER_MAX));
        this.shooter = true;
    }

    public Setpoint(double time, double intakeVelocity){
        this(time, 0.0, intakeVelocity);
        this.shooter = false;
    }

    public double getHoodPosition(){
        return this.hoodPosition;
    }

    public double getVelocity(){
        return this.velocity;
    }

    public boolean isShooter(){
        return this.shooter;
    }

    public double getTime(){
        return this.time;
    }
}
