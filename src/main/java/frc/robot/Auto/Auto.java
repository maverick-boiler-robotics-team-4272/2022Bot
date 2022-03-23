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
            () -> Subsystems.getPneumatics().intakeOut(),
            () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC),
            () -> Subsystems.getIntake().runIntake(0.7)),
        new Setpoint(0.1, 2.0,
            () -> Setpoint.noop(),
            () -> Subsystems.getIntake().runIntake(0.7),
            () -> Subsystems.getIntake().stopIntake())
        ),
        TERMINAL_3_BALL(1,
        new Setpoint(-2.0, 2.0,
            () -> Subsystems.getShooter().setShooter(ShooterPositions.FENDER_HIGH),
            () -> Subsystems.getShooter().shoot(),
            () -> Setpoint.noop())
        // new Setpoint(0.55, 0.1, 
        //     () -> Subsystems.getPneumatics().intakeOut(), 
        //     () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC)),
        // new Setpoint(0.75, 4.5,
        //     () -> Subsystems.getIntake().runIntake(0.75),
        //     () -> Subsystems.getIntake().stopIntake()),
        // new Setpoint(5.25, 1.0,
        //     () -> Subsystems.getShooter().shoot()
        // )),
        ),
        TERMINAL_2_BALL(2
        // new Setpoint(0.0, 0.1,
        //     () -> Subsystems.getShooter().stopShooterAndFeed()),
        // new Setpoint(1.0, 3.75, 
        //     () -> Subsystems.getShooter().stopShooterAndFeed(),
        //     () -> Subsystems.getIntake().runIntake(0.8)),
        // new Setpoint(4.8, 0.1,
        //     () -> Setpoint.noop(),
        //     () -> Subsystems.getIntake().stopIntake(),
        //     () -> Setpoint.noop()),
        // new Setpoint(6.0, 3.0,
        //     () -> Subsystems.getShooter().shoot()
        // )),
        ),
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
    private boolean fiveBall = false;
    private static Timer timer = new Timer();
    private double startTime;

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

        Auto.timer.reset();
        Auto.timer.start();
    }

    /**
     * Runs the path
     */
    public void run() {
        double currentTime = Auto.timer.get();
        if(path.setpoints.length > 0){
            currentTime += Math.min(path.setpoints[0].getTime(), 0.0);
        }
        boolean stopped = false;

        if(path.equals(Paths.TERMINAL_3_BALL)){
            terminal3Ball();
        }else if(path.equals(Paths.TERMINAL_2_BALL)){
            terminal2Ball();
        }

        if (currentTime > paths[path.index].getTotalTimeSeconds()) {
            Subsystems.getDriveTrain().drive(0, 0, 0, false);
            if(fiveBall && path.equals(Paths.TERMINAL_2_BALL) && Subsystems.getIntake().ballPresent()){
                Subsystems.getDriveTrain().drive(0, 0, Subsystems.getDriveTrain().aimAtHub(), false);
            }
            if(path.name().equals(Paths.TERMINAL_3_BALL.name()) && fiveBall){
                if(Subsystems.getIntake().ballPresent() || currentTime < 8.0){
                    return;
                }
                path = Paths.TERMINAL_2_BALL;
                initPath();
                System.out.println("auto path: " + path.name());
                return;
            }

            System.out.println("auto path: " + path.name());
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

    public static Timer getTimer(){
        return Auto.timer;
    }

    public void terminal3Ball(){
        double currTime = Auto.timer.get();
        System.out.println("time: " + currTime);
        if(currTime < 0.1){
            Subsystems.getShooter().setShooter(ShooterPositions.FENDER_HIGH);
        }else if(currTime > 0.15 && currTime < 2.0){
            Subsystems.getShooter().shoot();
        }else if(currTime < 2.1){
            Subsystems.getShooter().stopShooterAndFeed();
        }else if(currTime < 2.55 && currTime > 2.45){
            Subsystems.getShooter().setShooter(ShooterPositions.TARMAC);
            Subsystems.getPneumatics().intakeOut();
        }else if(currTime < 4.9 && currTime > 3.0){
            Subsystems.getIntake().runIntakeComplex(0.55, false);
        }else if(currTime > 5.0 && currTime < 7.19){
            Subsystems.getIntake().runIntakeComplex(0.55, false);
            Subsystems.getShooter().revShooter();
        }else if(currTime < 7.2){
            Subsystems.getIntake().stopIntake();
        }else if(currTime > 7.2){
            Subsystems.getShooter().shoot();
        }
    }

    public void terminal2Ball(){
        double currTime = Auto.timer.get();

        if(currTime < 1.0 && currTime > 0.8){
            Subsystems.getShooter().stopShooterAndFeed();
        }else if(currTime < 3.5){
            Subsystems.getIntake().runIntakeComplex(0.7, false);
            Subsystems.getShooter().setShooter(ShooterPositions.TARMAC);
        }else if(currTime < 4.5){
            Subsystems.getIntake().runIntakeComplex(0.7, false);
            Subsystems.getShooter().revShooter();
        }else if(currTime < 4.7 && currTime > 4.6){
            Subsystems.getIntake().stopIntake();
        }else if(currTime > 4.8){
            Subsystems.getShooter().shoot();
        }
    }
}
