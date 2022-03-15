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
import frc.robot.Subsystems.Constants;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Subsystems;
import frc.robot.Subsystems.Shooter.ShooterPositions;

public class Auto {
    public enum Paths {
        HANGAR_3_BALL(0),
        HANGAR_2_BALL(1),
        TERMINAL_3_BALL(2,
        new Setpoint(-1.5, 2.0, () -> Subsystems.getShooter().setShooter(ShooterPositions.FENDER_HIGH), Subsystems.getShooter()::shoot, Subsystems.getShooter()::stopShooterAndFeed),
        new Setpoint(1.2, 0.1, Subsystems.getPneumatics()::intakeOut, () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC), Setpoint::noop),
        new Setpoint(2.0, 7.0, () -> Subsystems.getIntake().runIntake(0.5), () -> Subsystems.getIntake().runIntake(0.5), () -> Subsystems.getIntake().runIntake(0.0)),
        new Setpoint(9.0, 0.1, () -> Subsystems.getIntake().runIntake(-0.5), () -> Subsystems.getIntake().runIntake(-0.5),  () -> Subsystems.getIntake().runIntake(0.0)),
        new Setpoint(10.0, 3.0, () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC), Subsystems.getShooter()::shoot, Subsystems.getShooter()::stopShooterAndFeed)
        ),
        TERMINAL_2_BALL(3),
        OFF_TARMAC(4),
        TUNE_PATH(5),
        TERMINAL_4_BALL(6),
        SHOOT_N_BACK_UP(7
            // new Setpoint(-2.0, 2.0, () -> Subsystems.getShooter().setShooter(ShooterPositions.MID_TARMAC), Subsystems.getShooter()::shoot, Subsystems.getShooter()::stopShooterAndFeed)
        ),
        TERMINAL_5_BALL(8
        new Setpoint(-1.0, 1.0, () -> Subsystems.getShooter().setShooter(ShooterPositions.FENDER_HIGH), Subsystems.getShooter()::shoot, Subsystems.getShooter()::stopShooterAndFeed),
        new Setpoint(1.2, 0.1, Subsystems.getPnuematics()::intakeOut, () -> Subsystems.getShooter().setShooter(ShooterPositions.TARMAC), Setpoint::noop),
        new Setpoint(2.0, 5.0, () -> Subsystems.getIntake().runIntake(0.6), () -> Subsystems.getIntake().runIntake(0.6), Subsystesm.getIntake().runIntake(0.0)),
        new Setpoint(6.5, 0.5, Subsystems.getShooter()::revShooter, Subsystesm.getShooter()::revShooter, Setpoint::noop),
        new Setpoint(7.0, 1.5, Subsystems.getShooter()::shoot, Subsystems.getShooter()::shoot, Subsystesm.getShooter()::stopShooterAndFeed),
        new Setpoint(8.5, 3.5, Subsystesm.getShooter().setShooter(ShooterPositions.LAUCH_PAD), Subsystesm.getIntake().runIntake(0.5), Subsytems.getIntake.runIntake(0.0)),
        new Setpoint(11.0, 1.0, Setpoint::noop, Subsystems.getShooter()::revShooter, Setpoint::noop),
        new Setpoint(12.0, 2.0, Subsystems.getShooter()::shoot, Subsystems.getShooter()::shoot, Subsystems.getShooter()::stopShooterAndFeed)
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
}
