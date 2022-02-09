package frc.robot.Auto;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class Auto {
    private Robot robot;

    private Path[] paths = new Path[3];

    public enum Paths{
        NO_SHOOT
    }

    private Paths path;

    public Auto(Robot robot){
        this.robot = robot;
        paths[0] = new Path(Filesystem.getDeployDirectory()+"/pathplanner/NO_SHOOT.path");
    }
    
    public void setPath(Paths path) {
        this.path = path;
    }

    public void run(){
        int path;
        switch(this.path){
            case NO_SHOOT:
                path = 0;
            default:
                path = 0;
        }

        Pose2d currentOdomPos = robot.hardware.swerveOdometry.getPoseMeters();
    }
}
