package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class Shooter {
    private Robot robot;
    
    //nested array has values [shooter speed, hood angle]
    private double[][] shooterSetpoints = {
        {1330.0, -15.0},//0.25
        {2060.0, -3.5},//0.38
        {2340.0, -15.5},//0.43
        {2600.0, -23}//0.48
    };

    private static final double HOOD_MOTOR_GEAR_RATIO = 1.0 / 12.0;
    private static final double RACK_AND_PINION_RATIO = HOOD_MOTOR_GEAR_RATIO / 2.5;

    private double hoodAmt;
    private double shooterAmt;
    //Shooter motor. ids 5, 6(follower)
    private CANSparkMax shooterMotor = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax shooterFollowerMotor = new CANSparkMax(15, MotorType.kBrushless);
    private CANSparkMax hoodMotor = new CANSparkMax(10, MotorType.kBrushless);


    public Shooter(Robot robot){
        this.robot = robot;
        this.hoodMotor.getEncoder().setPositionConversionFactor(1);
        SmartDashboard.putNumber("RPM", 60.0);
        SmartDashboard.putNumber("Rotation Motor P", 0.0000001);
        SmartDashboard.putNumber("Rotation Motor I", 0.0);
        SmartDashboard.putNumber("Rotation Motor D", 0.0);
        SmartDashboard.putNumber("Rotation Motor F", 0.0001);
        SmartDashboard.putNumber("Rotation Motor ACC", 40000.0);
        SmartDashboard.putNumber("Rotation Motor MAX VEL", 40000.0);
        SparkMaxPIDController hoodPIDController = this.hoodMotor.getPIDController();
        hoodPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        hoodPIDController.setP(0.0000001);
        hoodPIDController.setI(0.0);
        hoodPIDController.setD(0.0);
        hoodPIDController.setFF(0.0001);
        hoodPIDController.setSmartMotionMaxAccel(40000.0, 0);
        hoodPIDController.setSmartMotionMaxVelocity(40000.0, 0);
        hoodPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
        hoodPIDController.setSmartMotionAllowedClosedLoopError(0.0, 0);
        this.hoodMotor.getEncoder().setPosition(0.0);
        this.shooterMotor.setInverted(true);
        shooterFollowerMotor.follow(shooterMotor, true);
    }

    public void resetMotor(){
        hoodMotor.getEncoder().setPosition(0.0);
    }

    public void resetPID(){
        SparkMaxPIDController hoodPIDController = hoodMotor.getPIDController();
        hoodPIDController.setP(SmartDashboard.getNumber("Rotation Motor P", 0.0001));
        hoodPIDController.setI(SmartDashboard.getNumber("Rotation Motor I", 0.0));
        hoodPIDController.setD(SmartDashboard.getNumber("Rotation Motor D", 0.0));
        hoodPIDController.setFF(SmartDashboard.getNumber("Rotation Motor F", 0.001));
        hoodPIDController.setSmartMotionMaxAccel(SmartDashboard.getNumber("Rotation Motor ACC", 100.0), 0);
        hoodPIDController.setSmartMotionMaxVelocity(SmartDashboard.getNumber("Rotation Motor MAX VEL", 300.0), 0);

        SparkMaxPIDController shooterPIDController = shooterMotor.getPIDController();
        shooterPIDController.setP(SmartDashboard.getNumber("Shooter Motor P", 0.0001));
        shooterPIDController.setI(SmartDashboard.getNumber("Shooter Motor I", 0.0));
        shooterPIDController.setD(SmartDashboard.getNumber("Shooter Motor D", 0.0));
        shooterPIDController.setFF(SmartDashboard.getNumber("Shooter Motor F", 0.001));
    }

    /**
     * Starts the shooter wheel based on the trigger value
     * @param triggerValue amount an xbox trigger is pressed
     */
    public void shoot(){
        //this.shooterTopMotor.set(shooterAmt);
        this.shooterMotor.getPIDController().setReference(shooterAmt, ControlType.kVelocity);
        //this.shooterBottomMotor.set(triggerValue);
    }

    public void stopShooter(){
        this.shooterMotor.set(0);
    }

    public double getShooterAmount(){
        return shooterAmt;
    }
    
    public void putHoodDataToDashboard(){
        SmartDashboard.putNumber("Hood Position", hoodMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Hood Velocity", hoodMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());

    }
    public void putShooterDataToDashboard(){
        SmartDashboard.putNumber("Hood Setpoint", 0.0);
        SmartDashboard.putNumber("Shooter Percent", 0.0);
    }

    public void setShooter(int pov){
        int index = pov / 90;
        //putShooterDataToDashboard();
        shooterAmt = shooterSetpoints[index][0];
        hoodAmt = shooterSetpoints[index][1];
        setMotor();
        // shooterRotationMotor.getPIDController().setReference(-SmartDashboard.getNumber("RPM", 60.0), ControlType.kVelocity);
    }

    public void updateShooter() {
        shooterAmt = SmartDashboard.getNumber("Shooter Velocity Set", 0.0);
        hoodAmt = SmartDashboard.getNumber("Hood Setpoint", 0.0);
    }

    public void setMotor(){
        hoodMotor.getPIDController().setReference(hoodAmt, ControlType.kSmartMotion);
    }

    public void setHood(){
        hoodMotor.getPIDController().setReference(hoodAmt, ControlType.kSmartMotion);
    }

    public void tuneShoot(){
        this.shooterMotor.set(SmartDashboard.getNumber("Shooter Percent", 0.0));
        //this.shooterBottomMotor.set(SmartDashboard.getNumber("Shooter Percent", 0.0));
    }
    

    /**
     * Assuming that we have a camera this funciton will align the bot with the reflective strips on the hub
     * If, IF, I can figure out how I will have it calculate the perfect position and just auto put itself there, distance, angle, everything, but that's a later issue
     */
    public void cameraAlign(){
        
    }
}
