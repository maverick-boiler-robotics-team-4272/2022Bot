// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.Auto;
import frc.robot.Auto.Auto.Paths;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Limelight.LEDMode;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    //Constants

    private final SendableChooser<Paths> AUTO_CHOOSER = new SendableChooser<>();
    public Teleop teleop;
    public Auto auto;

    //Deadzone constants

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Subsystems.initSubsystems();
        auto = new Auto();
        teleop = new Teleop();
        Paths[] paths = Paths.values();
        SmartDashboard.putNumber("Limelight Ground Angle", 45.0);
        SmartDashboard.putNumber("Wheel Distance", Constants.WHEEL_DIST);
        for(int pathInd = 0; pathInd < paths.length; pathInd++){
            AUTO_CHOOSER.addOption(paths[pathInd].name(), paths[pathInd]);
        }
        AUTO_CHOOSER.setDefaultOption("TERMINAL_5_BALL", Paths.TERMINAL_5_BALL);
        SmartDashboard.putData("Auto choices", AUTO_CHOOSER);
        SmartDashboard.putNumber("Pigeon Heading", Subsystems.getDriveTrain().pigeon.getYaw());
        Subsystems.getShooter().putShooterDataToDashboard();
        Limelight.setLEDMode(LEDMode.ON);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        Limelight.setLEDMode(LEDMode.ON);

        Subsystems.getShooter().putShooterDataToDashboard();
        Subsystems.getDriveTrain().putCANCodersToSmartDashboard();
        Subsystems.getIntake().beamBreaksToSmart();
        SmartDashboard.putNumber("Limelight tx", Limelight.getTX());
        SmartDashboard.putBoolean("Limelight aimed", Limelight.getAimed());
        SmartDashboard.putNumber("Limelight Distance", Limelight.getDistanceFeet() * 12.0 - 6.0);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {

        auto.setPath(AUTO_CHOOSER.getSelected());
        auto.initPath();
        teleop.autod();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        
        auto.run();
        Subsystems.getIntake().beamBreaksToSmart();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // auto.stopAuto();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        teleop.run();

    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}