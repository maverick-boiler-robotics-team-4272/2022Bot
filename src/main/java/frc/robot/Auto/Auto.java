package frc.robot.Auto;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.Shooter.ShooterPositions;

public class Auto {
    //The Paths enum is used to select our path
    //The name of the enum is what the file that holds the path data is called
    //the first parameter is an index. The index is what links to a few other things
    //the rest of the parameters are time based setpoints for shooting, intaking, and other things
    public enum Paths {
        HANGAR_2_BALL(0,
        new Setpoint(0.0, 0.1,
            Subsystems.getPneumatics()::intakeOut,
            () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC),
            () -> Subsystems.getIntake().runIntake(0.7)),
        new Setpoint(0.1, 2.0,
            () -> Setpoint.noop(),
            () -> Subsystems.getIntake().runIntake(0.7),
            () -> Subsystems.getIntake().stopIntake())
        ),
        TERMINAL_3_BALL(1,
        new Setpoint(-1.0, 1.5, 
            () -> Subsystems.getShooter().setShooter(ShooterPositions.FENDER_HIGH), 
            () -> Subsystems.getShooter().shoot(), 
            () -> Subsystems.getShooter().stopShooterAndFeed()),
        new Setpoint(0.55, 0.1, 
            () -> Subsystems.getPneumatics().intakeOut(), 
            () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC), 
            () -> Setpoint.noop()),
        new Setpoint(0.75, 4.5,
            () -> Subsystems.getIntake().runIntake(0.75),
            () -> Setpoint.noop(), 
            () -> Subsystems.getIntake().runIntake(0.0)),
        new Setpoint(5.0, 0.5,
            () -> Subsystems.getShooter().revShooter(),
            () -> Setpoint.noop(), 
            () -> Subsystems.getShooter().shoot())
        ),
        TERMINAL_2_BALL(2,
        new Setpoint(-2.0, 2.0,
            () -> Setpoint.noop(), 
            () -> Setpoint.noop(), 
            () -> Subsystems.getShooter().stopShooterAndFeed()),
        new Setpoint(1.0, 4.0, 
            () -> Subsystems.getShooter().stopShooterAndFeed(),
            () -> Subsystems.getIntake().runIntake(0.85),
            () -> Subsystems.getIntake().stopIntake()),
        new Setpoint(6.0, 3.0,
            () -> Subsystems.getIntake().reverseToMid(),
            () -> Subsystems.getShooter().shoot(),
            () -> Subsystems.getShooter().stopShooterAndFeed()
        )),
        OFF_TARMAC(3),
        TUNE_PATH(4),
        TERMINAL_5_BALL(5,
        new Setpoint(-1.75, 1.75,
            () -> Subsystems.getShooter().setShooter(ShooterPositions.FENDER_HIGH), 
            () -> Subsystems.getShooter().shoot(), 
            () -> Subsystems.getShooter().stopShooterAndFeed()),
        new Setpoint(0.1, 0.1, 
            () -> Subsystems.getPneumatics().intakeOut(), 
            () -> Setpoint.noop(), 
            () -> Subsystems.getIntake().runIntake(0.75)),
        new Setpoint(1.0, 1.75, 
            () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC), 
            () -> Subsystems.getIntake().runIntake(0.75), 
            () -> Subsystems.getIntake().runIntake(0.0)),
        new Setpoint(2.75, 1.25, 
            () -> Subsystems.getIntake().runIntake(0.2),  
            () ->Subsystems.getShooter().revShooter(), 
            () ->Subsystems.getIntake().stopIntake()),
        new Setpoint(5.5, 0.5, 
            () -> Subsystems.getIntake().reverseToMid(), 
            () -> Subsystems.getIntake().runIntakeOnly(0.75), 
            () -> Subsystems.getShooter().shoot()),
        new Setpoint(5.0, 4.0, 
            () -> Subsystems.getShooter().revShooter(), 
            () -> Subsystems.getShooter().shoot(), 
            () -> Subsystems.getShooter().stopShooterAndFeed())/*,
        new Setpoint(8.5, 2.0, 
            () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC), 
            () -> Subsystems.getIntake().runIntake(0.75), 
            () -> Subsystems.getIntake().runIntake(0.0)),
        new Setpoint(10.5, 1.0, 
            () -> Setpoint.noop(), 
            () -> Subsystems.getShooter().revShooter(), 
            () -> Setpoint.noop()),
        new Setpoint(12.0, 2.0, 
            () -> Setpoint.noop(), 
            () -> Subsystems.getShooter().shoot(), 
            () -> Subsystems.getShooter().stopShooterAndFeed())*/
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
    private boolean fiveBall = false;

    // PID Controllers
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

    public Auto() {
        Paths[] paths = Paths.values();
        
        for(int i = 0; i < paths.length; i++){
           int index = paths[i].index;
           String name = paths[i].name();
           this.paths[index] = PathPlanner.loadPath(name, Constants.MAX_SPEED, Constants.MAX_ACC);
           this.startPoints[index] = this.paths[index].getInitialState().poseMeters;
        }

        thetaPid.enableContinuousInput(-180, 180);
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
        if(path.equals(Paths.TERMINAL_5_BALL)){
            fiveBall = true;
            path = Paths.TERMINAL_3_BALL;
        }
        Subsystems.getDriveTrain().setOdometry(startPoints[path.index]);
        Subsystems.getDriveTrain().setHeading(paths[path.index].getInitialState().holonomicRotation);
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
        boolean stopped = false;
        if (currentTime > paths[path.index].getTotalTimeSeconds()) {
            if(path.name().equals(Paths.TERMINAL_3_BALL.name()) && fiveBall){
                path = Paths.TERMINAL_2_BALL;
                initPath();
                System.out.println("auto path: " + path.name());
                return;
            }

            System.out.println("auto path: " + path.name());
            Subsystems.getDriveTrain().drive(0, 0, 0, false);
            stopped = true;
            //Subsystems.getIntake().stopFeedShooter();
            //Subsystems.getShooter().stopShooter();
            // return;
        }
        for (int i = 0; i < path.setpoints.length; i++){
            Setpoint setpoint = path.setpoints[i];
            if(setpoint.isInTime(currentTime)){
                setpoint.inTime();
            }else{
                setpoint.outOfTime();
            }
        }
        Pose2d currentOdomPos = Subsystems.getDriveTrain().getOdometryPoseMeters();
        PathPlannerState goal = (PathPlannerState) paths[path.index].sample(currentTime);
        ChassisSpeeds speeds = controller.calculate(currentOdomPos, goal,
                goal.holonomicRotation);
        if(!stopped){
            Subsystems.getDriveTrain().drive(-speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond, -speeds.omegaRadiansPerSecond, false);
        }
    }

    public void stopAuto(){
        Subsystems.getIntake().runIntake(0.0);
        Subsystems.getShooter().stopShooterAndFeed();
    }

}
