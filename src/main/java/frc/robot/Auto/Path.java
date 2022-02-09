package frc.robot.Auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import frc.robot.Robot;
import frc.robot.Subsystems.Hardware;

public class Path {
    private PathPlannerTrajectory trajectory;

    public Path(String path){
        trajectory = PathPlanner.loadPath(path, Hardware.MAX_SPEED, Hardware.MAX_ACC);
    }

    public Pose2d getPose2dOnPath(double time){
        State samp = trajectory.sample(time);
        return samp.poseMeters;
    }
}
