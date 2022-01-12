package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.*;
public class Hardware {
    private Robot robot;

    //drive motors (ids: 1, 2, 3, 4)
    public CANSparkMax frontRightDrive = new CANSparkMax(1, MotorType.kBrushless);
    public CANSparkMax frontLeftDrive = new CANSparkMax(2, MotorType.kBrushless);
    public CANSparkMax backLeftDrive = new CANSparkMax(3, MotorType.kBrushless);
    public CANSparkMax backRightDrive = new CANSparkMax(4, MotorType.kBrushless);

    //rotation motors (ids: 11, 12, 13, 14)
    public CANSparkMax frontRightRotation = new CANSparkMax(11, MotorType.kBrushless);
    public CANSparkMax frontLeftRotation = new CANSparkMax(12, MotorType.kBrushless);
    public CANSparkMax backLeftRotation = new CANSparkMax(13, MotorType.kBrushless);
    public CANSparkMax backRightRotation = new CANSparkMax(14, MotorType.kBrushless);

    //rotation motor encoders
    private RelativeEncoder frontRightEncoder = frontRightRotation.getEncoder();
    private RelativeEncoder frontLeftEncoder = frontLeftRotation.getEncoder();
    private RelativeEncoder backLeftEncoder = backLeftRotation.getEncoder();
    private RelativeEncoder backRightEncoder = backRightRotation.getEncoder();

    //IMU (pigeon)
    public PigeonIMU pigeon = new PigeonIMU(0);

    //error enum
    public enum Error{
        OK,
        FAIL
    };
    
    public Hardware(Robot robot){
        this.robot = robot;

    }

    /**
     * Initializes all of the motors
     */
    public void init(){
        initMotor(frontLeftDrive);
        initMotor(frontLeftRotation);

        initMotor(frontRightDrive);
        initMotor(frontRightRotation);

        initMotor(backLeftDrive);
        initMotor(backLeftRotation);

        initMotor(backRightDrive);
        initMotor(backRightRotation);
    }

    /**
     * Initializing a single motor controller
     * 
     * @param motor CANSparkMax motor
     */
    private void initMotor(CANSparkMax motor){
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(false);
    }

    /**
     * Sets everything for a whole module
     * @param module
     * module ids are as follows: front right is 1, front left is 2, back left is 3, and back right is 4
     * @param rotation
     * rotation angle, in degrees
     * @param speed
     * linear speed
     */
    public Error setModule(int module, double degrees, double speed){
        CANSparkMax driveMotor;
        RelativeEncoder rotationEncoder;
        switch(module){
            case 1:
                driveMotor = frontRightDrive;
                rotationEncoder = frontRightEncoder;
                break;
            case 2:
                driveMotor = frontLeftDrive;
                rotationEncoder = frontLeftEncoder;
                break;
            case 3:
                driveMotor = backLeftDrive;
                rotationEncoder = backLeftEncoder;
                break;
            case 4:
                driveMotor = backRightDrive;
                rotationEncoder = backRightEncoder;
                break;
            default:
                driveMotor = null;
                rotationEncoder = null;
                return Error.FAIL;

        }

        driveMotor.set(speed);
        rotationEncoder.setPosition(degrees / 360.0);
        return Error.OK;
    }
}