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
        TERMINAL_3_BALL(2,
            new Setpoint(-2.0, 0.0)
        ),
        TERMINAL_2_BALL(3),
        OFF_TARMAC(4),
        TUNE_PATH(5),
        TERMINAL_4_BALL(6),
        SHOOT_N_BACK_UP(7,
            new Setpoint(-5.5, 0.0),
            new Setpoint(-Constants.RAMP_UP_DEADZONE, -10.0, 2000.0)
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
        if(path == Paths.TERMINAL_2_BALL){
            if(terminal2Ball()){
                stopAuto();
                return;
            }
        }else if(path == Paths.TERMINAL_3_BALL){
            if(terminal3Ball()){
                stopAuto();
                return;
            }
        }else if(path == Paths.OFF_TARMAC){
            if(offTarmac()){
                stopAuto();
                return;
            }
        }else{
            //System.out.println("Current Time: " + currentTime);
            
            if (currentTime > paths[path.index].getTotalTimeSeconds()) {
                robot.driveTrain.drive(0, 0, 0, false);
                robot.intake.stopFeedShooter();
                robot.shooter.stopShooter();
                return;
            }
            for (int i = 0; i < path.setpoints.length; i++){
                Setpoint setpoint = path.setpoints[i];
                double setpointTime = currentTime - setpoint.getTime();

                // if(i == 0){
                    //System.out.println("Setpoint Time: " + setpointTime);
                // }
                if(setpoint.isShooter()){
                    //if (setpointTime < Constants.RAMP_UP_DEADZONE && setpointTime > -Constants.RAMP_UP_DEADZONE){
                        //System.out.println("Inside the isShooter if statement");
                        robot.shooter.setShooter(setpoint.getVelocity(), setpoint.getHoodPosition());
                    //}
                    System.out.println("Hood at Pos: " + robot.shooter.getHoodAtPosition());
                    if(/*setpointTime < Constants.FIRE_DEADZONE && setpointTime > -Constants.FIRE_DEADZONE*/
                    robot.shooter.getHoodAtPosition()){
                        System.out.println("Curr time when trying to shoot: " + currentTime);
                        robot.shooter.shoot();
                        break;
                    }else{
                        robot.shooter.stopShooter();
                        robot.intake.stopFeedShooter();
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
        }
        Pose2d currentOdomPos = robot.driveTrain.getOdometryPoseMeters();
        PathPlannerState goal = (PathPlannerState) paths[path.index].sample(currentTime);
        ChassisSpeeds speeds = controller.calculate(currentOdomPos, goal,
                goal.holonomicRotation);
        robot.driveTrain.drive(-speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
    }

    /**
     * Runs terminal 2 ball commands based on time
     * @return finished or not
     */
    private boolean terminal2Ball(){
        double currentTime = Timer.getFPGATimestamp() - startTime;

        if(currentTime > 10){
            return true;
        }else if(currentTime > paths[3].getTotalTimeSeconds()){
            // robot.shooter.setShooter(2370, -9.5);
            robot.intake.runIntake(0.0);
            robot.shooter.shoot();
            return false;
        }else{
            robot.pneumatics.intakeOut();
            robot.shooter.setShooter(2500, -9.0);
            robot.intake.runIntake(0.5);
            robot.intake.stopFeedShooter();
            return false;
        }
    }

    /**
     * Runs the terminal 3 ball commands based on time
     * @return finished
     */
    private boolean terminal3Ball(){
        double currentTime = Timer.getFPGATimestamp() - startTime;

        if(currentTime > 14){
            return true;
        }else if(currentTime > 10){
                //Shoots last two balls
            robot.shooter.shoot();
            return false;
        }else if(currentTime > 9.9){
                //Stops the intake before shooting
            robot.intake.runIntake(0.0);
        }else if(currentTime > 9.8 && currentTime < 9.9){
                //Runs intake backward to prevent balls from touching the shooter and shooting earlier
            robot.intake.runIntake(-0.5);
        }else if(currentTime > 2){
                //Puts intake out and runs it while setting the hood and shooter setpoints to prepare for
                //last shots. also stops the shooter from running
            robot.pneumatics.intakeOut();
            robot.intake.runIntake(0.65);
            robot.shooter.setShooter(2450.0, -9.5);
            robot.shooter.stopShooter();
            return false;
        }else if(currentTime < 2){
                //Sets current limit higher in the first half second of auto to prevent mechanical jams
            if(currentTime < 0.5){
                robot.intake.setIntakeCurrentLimit(60);
                robot.intake.runIntake(0.8);
            }else if(currentTime < 0.6){
                //Sets current limit back to regular and stops intake
                robot.intake.runIntake(0.0);
                robot.intake.setIntakeCurrentLimit(45);
            }
                //Pulls intake in and sets shooter to shoot first shot
            robot.pneumatics.intakeIn();
            robot.shooter.setShooter(2275.0, -7.0);
            robot.shooter.shoot();
            
            return false;
        }
        return false;
    }

    /**
     * Runs off tarmac commands
     * @return finished
     */
    private boolean offTarmac(){
        double currentTime = Timer.getFPGATimestamp() - startTime;
        if(currentTime > 6){
            robot.shooter.stopShooter();
            return true;
        }else if(currentTime > 3){
            robot.shooter.shoot();
        }else{
            robot.shooter.setShooter(2370, -9.5);
        }
        return false;
    }

    /**
     * Stops the bot at the end of auto
     */
    private void stopAuto(){
        robot.driveTrain.drive(0, 0, 0, false);
        robot.shooter.stopShooter();
        robot.intake.stopFeedShooter();
    }
}