package frc.robot.Auto;

public class Setpoint {
    private double startTime;
    private double duration;
    private boolean started = false;
    private Runnable startMethod;
    private Runnable duringMethod;
    private Runnable endMethod;
    
    /**
     * 
     * @param time - what time to start the action
     * @param duration - how long to run it
     * @param start - what to do at the very start. Only run once
     * @param during - what to do during. Run consistently
     * @param end - what to do at the very end. Only run once
     */
    public Setpoint(double time, double duration, Runnable start, Runnable during, Runnable end){
        this.startTime = time;
        this.duration = duration;
        this.startMethod = start;
        this.duringMethod = during;
        this.endMethod = end;
    }

    /**
     * 
     * @param time - time to start our action
     * @param waitTime - amount of time to wait between startAction and endAction
     * @param startAction - what to run at the very beginning. Only run once
     * @param endAction - what to run at the very end. Only run once
     */
    public Setpoint(double time, double waitTime, Runnable startAction, Runnable endAction){
        this(time, waitTime, startAction, Setpoint::noop, endAction);
    }

    /**
     * 
     * @param time - time to run our action
     * @param deadzone - time deadzone after time to run our action
     * @param action - action to be run. Only run once.
     */
    public Setpoint(double time, double deadzone, Runnable action){
        this(time, deadzone, action, Setpoint::noop, Setpoint::noop);

    }

    /**
     * gets the the time value to start the action
     * @return the time value
     */
    public double getTime(){
        return startTime;
    }

    /**
     * runs the actions for when we are in time for running the action
     * runs the start method once, and then the during method for the remainder
     */
    public void inTime(){
        if(!started){
            started = true;
            startMethod.run();
        }else{
            duringMethod.run();
        }
    }

    /**
     * runs the action for when we are after running initially
     * runs the end method once
     */
    public void outOfTime(){
        if(started){
            started = false;
            endMethod.run();
        }
    }

    /**
     * Returns whether or not we are in the time for running our setpoint
     * @param currentTime current time for computing whether we are in the correct time
     * @return whether or not we are in the time for our setpoint to run
     */
    public boolean isInTime(double currentTime){
        double referenceTime = currentTime - startTime;
        return referenceTime < duration && referenceTime > 0;
    }

    /**
     * A no operation method. Used in the setpoint constructor for when you don't want anything to run
     */
    public static void noop(){

    }
}
