package frc.robot.Auto;

import java.sql.Time;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class Auto {
    private Robot robot;

    private PathPlannerTrajectory[] paths = new PathPlannerTrajectory[1];
    private Pose2d[] startPoints = new Pose2d[1];
    private double startTime;

    // Controller
    private HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(1, 0, 0),
            new PIDController(1, 0, 0),
            new ProfiledPIDController(1, 0, 0,
                    new TrapezoidProfile.Constraints(Robot.MAX_ANGULAR_SPEED, Robot.MAX_ANGULAR_ACC)));

    public enum Paths {
        NO_SHOOT(0);

        int index;

        Paths(int index) {
            this.index = index;
        }
    }

    private Paths path;

    public Auto(Robot robot) {
        this.robot = robot;
        this.paths[0] = PathPlanner.loadPath(Filesystem.getDeployDirectory() + "/pathplanner/NO_SHOOT.path",
                Robot.MAX_SPEED, Robot.MAX_ACC);
        this.startPoints[0] = this.paths[0].sample(0).poseMeters;
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
        startTime = Timer.getFPGATimestamp();
    }

    /**
     * Runs the path
     */
    public void run() {

        Pose2d currentOdomPos = robot.driveTrain.getOdometryPoseMeters();
        Trajectory.State goal = paths[path.index].sample(Timer.getFPGATimestamp() - startTime);
        ChassisSpeeds speeds = controller.calculate(currentOdomPos, goal,
                Rotation2d.fromDegrees(robot.driveTrain.getPigeonHeading()));
        robot.driveTrain.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, true);

    }
}