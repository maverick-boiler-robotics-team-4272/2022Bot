package frc.robot.Auto;

public class Setpoint {
    private double startTime;
    private double duration;
    private boolean started = false;
    private Runnable startMethod;
    private Runnable duringMethod;
    private Runnable endMethod;

    public Setpoint(double time, double deadzone,Runnable start, Runnable during, Runnable end){
        this.startTime = time;
        this.duration = deadzone;
        this.startMethod = start;
        this.duringMethod = during;
        this.endMethod = end;
    }

    public double getTime(){
        return startTime;
    }

    public void inTime(){
        if(!started){
            started = true;
            startMethod.run();
        }else{
            duringMethod.run();
        }
    }

    public void outOfTime(){

        if(started){
            started = false;
            endMethod.run();
        }
    }

    public boolean isInTime(double currentTime){
        double referenceTime = currentTime - startTime;
        return referenceTime < duration && referenceTime > 0;
    }

    public static void noop(){

    }
}
