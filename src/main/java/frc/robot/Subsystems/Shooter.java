package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {    
    

    public enum ShooterPositions{
        //shootAmt, hoodAmt, feedAmt
        FENDER_LOW(
            1200.0, -15.0, -0.5
        ),
        FENDER_HIGH(
            2300.0, -0.25, -0.5 //2250
        ),
        TARMAC(
            2450.0, -11, -0.5 //2300, -12.5
        ),
        LAUNCHPAD(
            2625.0, -20.0, -0.5
        ),
        EJECT(
            1000.0, 0.0, -0.5
        ),
        MID_TARMAC(
            2300.0, -10.0, -0.8
        ),
        MID_LAUNCHPAD(
            2450.0, -16.0, -0.8
        );

        public final double shootAmt;
        public final double hoodAmt;
        public final double feedAmt;

        private ShooterPositions(double shootAmt, double hoodAmt, double feedAmt){
            this.shootAmt = shootAmt;
            this.hoodAmt = hoodAmt;
            this.feedAmt = feedAmt;
        }
    }

    private double hoodAmt;
    private double shooterAmt;
    private double feedAmt;
    //Shooter motor. ids 5, 6(follower)
    private CANSparkMax shooterMotor = new CANSparkMax(5, MotorType.kBrushless);
    private CANSparkMax shooterFollowerMotor = new CANSparkMax(15, MotorType.kBrushless);
    private CANSparkMax hoodMotor = new CANSparkMax(10, MotorType.kBrushless);
    private SparkMaxPIDController shooterPIDController = shooterMotor.getPIDController();
    private SparkMaxPIDController hoodPIDController = hoodMotor.getPIDController();

    private boolean ballOffWheel = false;
    
    private boolean shooterAtSpeed = false;
    public Shooter(){
        this.hoodMotor.getEncoder().setPositionConversionFactor(1);

        
        hoodPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        hoodPIDController.setP(0.0000001);
        hoodPIDController.setI(0.0);
        hoodPIDController.setD(0.0);
        hoodPIDController.setFF(0.0001);
        hoodPIDController.setSmartMotionMaxAccel(40000.0, 0);
        hoodPIDController.setSmartMotionMaxVelocity(40000.0, 0);
        hoodPIDController.setSmartMotionMinOutputVelocity(0.0, 0);
        hoodPIDController.setSmartMotionAllowedClosedLoopError(0.0, 0);
        // System.out.println(hoodMotor.getForwardLimitSwitch(Type.kNormallyOpen));
        this.hoodMotor.getEncoder().setPosition(0.0);
        this.shooterMotor.setInverted(true);
        //System.out.println(hoodMotor.getForwardLimitSwitch(Type.kNormallyClosed).toString());
        shooterFollowerMotor.follow(shooterMotor, true);
        
        
        SmartDashboard.putNumber("Shooter Velocity Set", 0.0);
        SmartDashboard.putNumber("Hood Setpoint", 0.0);
        SmartDashboard.putNumber("Feed Setpoint", 0.0);
        SmartDashboard.putNumber("Shooter Motor P", 0.0001);
        SmartDashboard.putNumber("Shooter Motor I", 0.000000002);
        SmartDashboard.putNumber("Shooter Motor D", 0.0);
        SmartDashboard.putNumber("Shooter Motor F", 0.00018);
        shooterPIDController.setP(0.0001);
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
        this.shooterMotor.getPIDController().setReference(shooterAmt, ControlType.kSmartVelocity);
        SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());

        if(shooterMotor.getEncoder().getVelocity() >= shooterAmt - (Constants.SHOOTER_DEADZONE) &&
            shooterMotor.getEncoder().getVelocity() <= shooterAmt + (Constants.SHOOTER_DEADZONE) &&
            shooterAmt > 500){
            shooterAtSpeed = true;
        }
        if(!Subsystems.getIntake().getShooterBeam()){
            ballOffWheel = true;
        }
        if(!ballOffWheel){
            ballOffWheel = Subsystems.getIntake().reverseToMid();
            return;
        }

        if(shooterAtSpeed && ballOffWheel){
            Subsystems.getIntake().feedShooter(feedAmt);
            Subsystems.getIntake().resetBall();
        }else{
            Subsystems.getIntake().stopFeedShooter();
        }
    }

    /**
     * Stops the shooter
     */
    public void stopShooter(){
        this.shooterMotor.set(0);
        if(shooterAtSpeed){
            Subsystems.getIntake().stopFeedShooter();
        }
        shooterAtSpeed = false;
        ballOffWheel = false;
    }

    public void stopShooterAndFeed(){
        stopShooter();
        Subsystems.getIntake().stopFeedShooter();
    }

     /**
     * Pushes the hood down until it hits the limit switch to 0 it
     */
    public boolean fixHood(){
        System.out.println("LIM SWITCH: " + hoodMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed());
        if(!hoodMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()){
            hoodMotor.set(0.1);
            return true;
        }else{
            System.out.println("Hood pos before: " + hoodMotor.getEncoder().getPosition());
            hoodMotor.getEncoder().setPosition(0);
            setShooter(ShooterPositions.EJECT);
            System.out.println("Hood pos after: " + hoodMotor.getEncoder().getPosition());
            return false;//return false once done
        }

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
        ShooterPositions[] shooterSetpoints = ShooterPositions.values();
        setShooter(shooterSetpoints[index].shootAmt, shooterSetpoints[index].hoodAmt, shooterSetpoints[index].feedAmt);
    }

    public void setShooter(double shooterAmount, double hoodAmount, double feedAmount){
        shooterAmt = shooterAmount;
        hoodAmt = hoodAmount;
        feedAmt = feedAmount;
        setHood();
    }

    public void setShooter(ShooterPositions shooterPosition){
        setShooter(shooterPosition.shootAmt, shooterPosition.hoodAmt, shooterPosition.feedAmt);
    }

    public void revShooter(){
        
        if(!Subsystems.getIntake().getShooterBeam()){
            ballOffWheel = true;
        }
        if(!ballOffWheel){
            ballOffWheel = Subsystems.getIntake().reverseToMid();
            return;
        }
        shooterMotor.getPIDController().setReference(shooterAmt * 0.9, ControlType.kSmartVelocity);
    }

    /**
     * Updates the shooter and hood from smart dashboard
     */
    public void updateShooter() {
        shooterAmt = SmartDashboard.getNumber("Shooter Velocity Set", 0.0);
        hoodAmt = SmartDashboard.getNumber("Hood Setpoint", 0.0);
        feedAmt = SmartDashboard.getNumber("Feed Setpoint", 0.0);
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
        hoodAmt = ShooterPositions.EJECT.hoodAmt;
        shooterAmt = ShooterPositions.EJECT.shootAmt;
        feedAmt = ShooterPositions.EJECT.feedAmt;
        setHood();
    }

    /**
     * Set shooterMotor from smart dashboard
     */
    public void tuneShoot(){
        this.shooterMotor.set(SmartDashboard.getNumber("Shooter Percent", 0.0));
    }

    /**
     * Returns if the hood is within a certain deadzone of its desired position
     * @return
     */
    public boolean getHoodAtPosition(){
        double hoodPos = hoodMotor.getEncoder().getPosition();
        // System.out.println("HoodPos: " + hoodPos);
        // System.out.println("HoodAmt: " + hoodAmt);
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
