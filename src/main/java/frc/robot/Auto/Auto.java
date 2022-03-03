package frc.robot.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Subsystems.Constants;

public class Auto {
    private Robot robot;
    public enum Paths {
        HANGAR_3_BALL(0, 
            new Setpoint(0.0, 1.0),
            new Setpoint(1.0, 20.0, 1000.0)
        ),
        HANGAR_2_BALL(1),
        TERMINAL_3_BALL(2),
        TERMINAL_2_BALL(3),
        OFF_TARMAC(4),
        TUNE_PATH(5),
        TERMINAL_4_BALL(6),
        SHOOT_N_BACK_UP(7,
            new Setpoint(-5.0, -10.0, 2000.0)
        );

        final int index;
        final Setpoint[] setpoints;
        Paths(int index, Setpoint... setpoints) {
            this.index = index;
            this.setpoints = setpoints;
        }
    }

    private PathPlannerTrajectory[] paths = new PathPlannerTrajectory[Paths.values().length];
    private Pose2d[] startPoints = new Pose2d[Paths.values().length];
    private double startTime;

    // Controller
    public PIDController xPid = new PIDController(2.0, 0.01, 0);
    public PIDController yPid = new PIDController(2.0, 0.01, 0);
    public ProfiledPIDController thetaPid = new ProfiledPIDController(4.5, 0, 0,
    new TrapezoidProfile.Constraints(Constants.MAX_ANGULAR_SPEED, Constants.MAX_ANGULAR_ACC));
    private SwerveAutoController controller = new SwerveAutoController(
            xPid,
            yPid,
            thetaPid
    );

    private Paths path;

    public Auto(Robot robot) {
        this.robot = robot;
        Paths[] paths = Paths.values();
        
        for(int i = 0; i < paths.length; i++){
           int index = paths[i].index;
           String name = paths[i].name();
           this.paths[index] = PathPlanner.loadPath(name, Constants.MAX_SPEED, Constants.MAX_ACC);
           this.startPoints[index] = this.paths[index].getInitialState().poseMeters;
        }
    }

    /**
     * Sets path equal to path inputed
     * 
     * @param path
     */
    public void setPath(Paths path) {
        this.path = path;
    }

    /**
     * Initiates the path by setting the odometry to that of the path's start point,
     * and starting the time
     */
    public void initPath() {
        robot.driveTrain.setOdometry(startPoints[path.index]);
        robot.driveTrain.setHeading(paths[path.index].getInitialState().holonomicRotation);
        startTime = Timer.getFPGATimestamp();
    }

    /**
     * Runs the path
     */
    public void run() {
        double currentTime = Timer.getFPGATimestamp() - startTime;
        if(path.setpoints.length > 0){
            currentTime += Math.min(path.setpoints[0].getTime(), 0.0);
        }
        if (currentTime > paths[path.index].getTotalTimeSeconds()) {
            robot.driveTrain.drive(0, 0, 0, false);
            return;
        }
        System.out.println("Testing");
        for (int i = 0; i < path.setpoints.length; i++){
            Setpoint setpoint = path.setpoints[i];
            double setpointTime = currentTime - setpoint.getTime();
            if(setpoint.isShooter()){
                if (setpointTime < Constants.RAMP_UP_DEADZONE && setpointTime > -Constants.FIRE_DEADZONE){
                    robot.shooter.setShooter(setpoint.getVelocity(), setpoint.getHoodPosition());
                }

                if(setpointTime < Constants.FIRE_DEADZONE && setpointTime > -Constants.FIRE_DEADZONE){
                    robot.shooter.shoot();
                    break;
                }else{
                    System.out.println("stopping shooting");
                    robot.shooter.stopShooter();
                }
            }else{
                if(setpointTime < Constants.RAMP_UP_DEADZONE && setpointTime > -Constants.RAMP_UP_DEADZONE){
                    robot.intake.runIntake(setpoint.getVelocity());
                    break;
                }else{
                    robot.intake.runIntake(0.0);
                }

            }
        }
        
        Pose2d currentOdomPos = robot.driveTrain.getOdometryPoseMeters();
        PathPlannerState goal = (PathPlannerState) paths[path.index].sample(currentTime);
        ChassisSpeeds speeds = controller.calculate(currentOdomPos, goal,
                goal.holonomicRotation);
        robot.driveTrain.drive(-speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
    }
}