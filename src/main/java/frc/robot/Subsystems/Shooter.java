package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Shooter {
    private Robot robot;
    
    //nested array has values [shooter speed, hood angle]
    private double[][] shooterSetpoints = {
        {1200.0, -15.0},//0.25 //Close Low goal
        {2000.0, -6},//0.38 //Close high goal
        {2370.0, -9.5},//0.43 //Edge of tarmac high goal
        {2350.0, -24},//0.48 //Launcpad high goal
        {1000, 0}//Shooting out wrong colors
    };

    private double hoodAmt;
    private double shooterAmt;
    //Shooter motor. ids 5, 6(follower)
    private CANSparkMax shooterMotor = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax shooterFollowerMotor = new CANSparkMax(15, MotorType.kBrushless);
    private CANSparkMax hoodMotor = new CANSparkMax(10, MotorType.kBrushless);
    private SparkMaxPIDController shooterPIDController = shooterMotor.getPIDController();
    private SparkMaxPIDController hoodPIDController = hoodMotor.getPIDController();
    
    public Shooter(Robot robot){
        this.robot = robot;
        this.hoodMotor.getEncoder().setPositionConversionFactor(1);
        /*SmartDashboard.putNumber("RPM", 60.0);
        SmartDashboard.putNumber("Rotation Motor P", 0.0000001);
        SmartDashboard.putNumber("Rotation Motor I", 0.0);
        SmartDashboard.putNumber("Rotation Motor D", 0.0);
        SmartDashboard.putNumber("Rotation Motor F", 0.0001);
        SmartDashboard.putNumber("Rotation Motor ACC", 40000.0);
        SmartDashboard.putNumber("Rotation Motor MAX VEL", 40000.0);*/

        
        hoodPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        hoodPIDController.setP(0.0000001);
        hoodPIDController.setI(0.0);
        hoodPIDController.setD(0.0);
        hoodPIDController.setFF(0.0001);
        hoodPIDController.setSmartMotionMaxAccel(40000.0, 0);
        hoodPIDController.setSmartMotionMaxVelocity(40000.0, 0);
        hoodPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
        hoodPIDController.setSmartMotionAllowedClosedLoopError(0.0, 0);
        System.out.println(hoodMotor.getForwardLimitSwitch(Type.kNormallyOpen));
        this.hoodMotor.getEncoder().setPosition(0.0);
        this.shooterMotor.setInverted(true);
        //System.out.println(hoodMotor.getForwardLimitSwitch(Type.kNormallyClosed).toString());
        shooterFollowerMotor.follow(shooterMotor, true);
        
        
        SmartDashboard.putNumber("Shooter Velocity Set", 0.0);
        SmartDashboard.putNumber("Hood Setpoint", 0.0);
        SmartDashboard.putNumber("Shooter Motor P", 0.0000005);
        SmartDashboard.putNumber("Shooter Motor I", 0.000000002);
        SmartDashboard.putNumber("Shooter Motor D", 0.0);
        SmartDashboard.putNumber("Shooter Motor F", 0.00018);
        shooterPIDController.setP(0.0000005);
        shooterPIDController.setI(0.000000002);
        shooterPIDController.setD(0.0);
        shooterPIDController.setFF(0.00018);

        shooterPIDController.setSmartMotionMaxVelocity(3000.0, 0);
        shooterPIDController.setSmartMotionMaxAccel(4000.0, 0);
        shooterPIDController.setSmartMotionAllowedClosedLoopError(5.0, 0);

    }

    /**
     * Resets pid values
     */
    public void resetPID(){
        hoodPIDController.setP(SmartDashboard.getNumber("Rotation Motor P", 0.0001));
        hoodPIDController.setI(SmartDashboard.getNumber("Rotation Motor I", 0.0));
        hoodPIDController.setD(SmartDashboard.getNumber("Rotation Motor D", 0.0));
        hoodPIDController.setFF(SmartDashboard.getNumber("Rotation Motor F", 0.001));
        hoodPIDController.setSmartMotionMaxAccel(SmartDashboard.getNumber("Rotation Motor ACC", 100.0), 0);
        hoodPIDController.setSmartMotionMaxVelocity(SmartDashboard.getNumber("Rotation Motor MAX VEL", 300.0), 0);

        shooterPIDController.setP(SmartDashboard.getNumber("Shooter Motor P", 1.0));
        shooterPIDController.setI(SmartDashboard.getNumber("Shooter Motor I", 0.0));
        shooterPIDController.setD(SmartDashboard.getNumber("Shooter Motor D", 0.0));
        shooterPIDController.setFF(SmartDashboard.getNumber("Shooter Motor F", 0.0));
    }

    /**
     * Starts the shooter wheel based on the shooter amount variable that is determined by the dpad
     */
    public void shoot(){
        System.out.println("beam break: " + robot.intake.getFeedSensor());
        //this.shooterMotor.set(shooterAmt);
        this.shooterMotor.getPIDController().setReference(shooterAmt, ControlType.kSmartVelocity);
        SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());
        System.out.println("Shooter Vel: " + shooterMotor.getEncoder().getVelocity());
        // System.out.println("Shooter Vel: " + shooterMotor.getEncoder().getVelocity());
        // System.out.println("shooterAmt: " + shooterAmt);
        if(shooterMotor.getEncoder().getVelocity() >= shooterAmt - (Constants.SHOOTER_DEADZONE) &&
            shooterMotor.getEncoder().getVelocity() <= shooterAmt + (Constants.SHOOTER_DEADZONE) &&
            shooterAmt > 500){
            robot.intake.feedShooter();
        }else{
            robot.intake.stopFeedShooter();
        }
    }

    /**
     * Stops the shooter
     */
    public void stopShooter(){
        this.shooterMotor.set(0);
    }

    /**
     * Pushes hood and shooter data to Smart Dashboard
     */
    public void putShooterDataToDashboard(){
        SmartDashboard.putNumber("Hood Position", hoodMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Hood Velocity", hoodMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());
        //SmartDashboard.putNumber("Hood Setpoint", 0.0);
        SmartDashboard.putNumber("Shooter Percent", 0.0);
        
        SmartDashboard.putNumber("shooter amount", shooterAmt);
        SmartDashboard.putNumber("hood amount", hoodAmt);

    }

    /**
     * Sets shooterAmt and hoodAmt variables to specified values in an array who's index is determined by
     * the inputted dpad value
     * 
     * @param pov
     */
    public void setShooter(int pov){
        int index = pov / 90;
        //putShooterDataToDashboard();
        SmartDashboard.putNumber("Shooter Index", index);
        setShooter(shooterSetpoints[index][0], shooterSetpoints[index][1]);
    }

    public void setShooter(double shooterAmount, double hoodAmount){
        shooterAmt = shooterAmount;
        hoodAmt = hoodAmount;
        setHood();
    }

    /**
     * Updates the shooter and hood from smart dashboard
     */
    public void updateShooter() {
        shooterAmt = SmartDashboard.getNumber("Shooter Velocity Set", 0.0);
        hoodAmt = SmartDashboard.getNumber("Hood Setpoint", 0.0);
    }

    /**
     * Sets hood position
     */
    public void setHood(){
        hoodMotor.getPIDController().setReference(hoodAmt, ControlType.kSmartMotion);
    }

    /**
     * Sets the hood to 0
     */
    public void zeroHood(){
        hoodAmt = shooterSetpoints[4][1];
        shooterAmt = shooterSetpoints[4][0];
        setHood();
    }

    /**
     * Set shooterMotor from smart dashboard
     */
    public void tuneShoot(){
        this.shooterMotor.set(SmartDashboard.getNumber("Shooter Percent", 0.0));
    }

    /**
     * Pushes the hood down until it hits the limit switch to 0 it
     */
    public void fixHood(){
        double initTime = Timer.getFPGATimestamp();
        if(Timer.getFPGATimestamp() - initTime < 3 /*|| hoodMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()*/){
            hoodMotor.set(0.2);
        }else{
            hoodMotor.getEncoder().setPosition(0);
            hoodMotor.set(0);
        }
    }

    public boolean getHoodAtPosition(){
        double hoodPos = hoodMotor.getEncoder().getPosition();
        System.out.println("HoodPos: " + hoodPos);
        System.out.println("HoodAmt: " + hoodAmt);
        if(hoodPos < (hoodAmt - (hoodAmt * Constants.HOOD_DEADZONE))
        && hoodPos > (hoodAmt + (hoodAmt * Constants.HOOD_DEADZONE))){
            return true;
        }else{
            return false;
        }
    }
    

    /**
     * Assuming that we have a camera this funciton will align the bot with the reflective strips on the hub
     * If, IF, I can figure out how I will have it calculate the perfect position and just auto put itself there, distance, angle, everything, but that's a later issue
     */
    public void cameraAlign(){
        
    }
}
