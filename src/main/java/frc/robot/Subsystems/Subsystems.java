package frc.robot.Subsystems;

public class Subsystems {
    private static DriveTrain driveTrain;
    private static Climber climber;
    private static Shooter shooter;
    private static Intake intake;
    private static Pneumatics pneumatics;

    public static void initSubsystems(){
        driveTrain = new DriveTrain();
        climber = new Climber();
        shooter = new Shooter();
        intake = new Intake();
        pneumatics = new Pneumatics();
    }

    public static DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public static Climber getClimber() {
        return climber;
    }

    public static Intake getIntake() {
        return intake;
    }

    public static Pneumatics getPneumatics() {
        return pneumatics;
    }

    public static Shooter getShooter() {
        return shooter;
    }
}
