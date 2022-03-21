package frc.robot.Subsystems;

public class Subsystems {
    private static DriveTrain driveTrain;
    private static Climber climber;
    private static Shooter shooter;
    private static Intake intake;
    private static Pneumatics pneumatics;

    /**
     * Subsystems class for the purpose of removing the dependancy to the Robot in every constructor 
     */ 
    public static void initSubsystems(){
        driveTrain = new DriveTrain();
        climber = new Climber();
        climber.zeroClimbers();
        shooter = new Shooter();
        intake = new Intake();
        pneumatics = new Pneumatics();
    }

    /**
     * gets the drive train
     * @return
     */
    public static DriveTrain getDriveTrain() {
        return driveTrain;
    }

    /**
     * Gets the climber
     * @return
     */
    public static Climber getClimber() {
        return climber;
    }

    /**
     * Gets the intake
     * @return
     */
    public static Intake getIntake() {
        return intake;
    }

    /**
     * Gets the pneumatics
     * @return
     */
    public static Pneumatics getPneumatics() {
        return pneumatics;
    }

    /**
     * gets the shooter
     * @return
     */
    public static Shooter getShooter() {
        return shooter;
    }
}
