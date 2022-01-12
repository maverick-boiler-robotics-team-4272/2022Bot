package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.Math.Vector2;
import frc.robot.Robot;

public class Teleop {
    //xbox controllers for driving and operating
    private XboxController driveController = new XboxController(0);
    private XboxController operatorController = new XboxController(1);
    
    //swerve drive drive train

    private Translation2d frontLeftTranslation = new Translation2d(0, 0);
    private Translation2d frontRightTranslation = new Translation2d(0, 0);
    private Translation2d backLeftTranslation = new Translation2d(0, 0);
    private Translation2d backRightTranslation = new Translation2d(0, 0);

    private SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(frontLeftTranslation, frontRightTranslation, backLeftTranslation, backRightTranslation);


    private Robot robot;

    public Teleop(Robot robot){
        this.robot = robot;
    }

    public void init(){

    }

    public void run(){
        Vector2 translation = new Vector2(driveController.getLeftX(), driveController.getLeftY());
    }
}
