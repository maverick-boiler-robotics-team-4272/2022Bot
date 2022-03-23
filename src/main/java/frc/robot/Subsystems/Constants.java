package frc.robot.Subsystems;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double TALON_FRONT_RIGHT_P = 0.0039;
    public static final double TALON_FRONT_RIGHT_I = 0.0;
    public static final double TALON_FRONT_RIGHT_D = 0.0;
    public static final double TALON_FRONT_RIGHT_FF = 0.001;

    public static final double TALON_FRONT_LEFT_P = 0.0038;
    public static final double TALON_FRONT_LEFT_I = 0.0;
    public static final double TALON_FRONT_LEFT_D = 0.0;
    public static final double TALON_FRONT_LEFT_FF = 0.001;

    public static final double TALON_BACK_RIGHT_P = 0.0038;
    public static final double TALON_BACK_RIGHT_I = 0.0;
    public static final double TALON_BACK_RIGHT_D = 0.0;
    public static final double TALON_BACK_RIGHT_FF = 0.001;

    public static final double TALON_BACK_LEFT_P = 0.0038;
    public static final double TALON_BACK_LEFT_I = 0.0;
    public static final double TALON_BACK_LEFT_D = 0.0;
    public static final double TALON_BACK_LEFT_FF = 0.001;

    public static final double STEER_FRONT_RIGHT_P = 0.01;
    public static final double STEER_FRONT_RIGHT_I = 0.0001;
    public static final double STEER_FRONT_RIGHT_D = 0.0;
    public static final double STEER_FRONT_RIGHT_FF = 0.0;

    public static final double STEER_FRONT_LEFT_P = 0.01;
    public static final double STEER_FRONT_LEFT_I = 0.0001;
    public static final double STEER_FRONT_LEFT_D = 0.0;
    public static final double STEER_FRONT_LEFT_FF = 0.0;

    public static final double STEER_BACK_RIGHT_P = 0.01;
    public static final double STEER_BACK_RIGHT_I = 0.0001;
    public static final double STEER_BACK_RIGHT_D = 0.0;
    public static final double STEER_BACK_RIGHT_FF = 0.0;

    public static final double STEER_BACK_LEFT_P = 0.01;
    public static final double STEER_BACK_LEFT_I = 0.0001;
    public static final double STEER_BACK_LEFT_D = 0.0;
    public static final double STEER_BACK_LEFT_FF = 0.0;

    public static final double DRIVE_FRONT_RIGHT_P = 0.0039;
    public static final double DRIVE_FRONT_RIGHT_I = 0.0;
    public static final double DRIVE_FRONT_RIGHT_D = 0.0;
    public static final double DRIVE_FRONT_RIGHT_FF = 0.001;

    public static final double DRIVE_FRONT_LEFT_P = 0.0038;
    public static final double DRIVE_FRONT_LEFT_I = 0.0;
    public static final double DRIVE_FRONT_LEFT_D = 0.0;
    public static final double DRIVE_FRONT_LEFT_FF = 0.001;

    public static final double DRIVE_BACK_RIGHT_P = 0.0038;
    public static final double DRIVE_BACK_RIGHT_I = 0.0;
    public static final double DRIVE_BACK_RIGHT_D = 0.0;
    public static final double DRIVE_BACK_RIGHT_FF = 0.001;

    public static final double DRIVE_BACK_LEFT_P = 0.0038;
    public static final double DRIVE_BACK_LEFT_I = 0.0;
    public static final double DRIVE_BACK_LEFT_D = 0.0;
    public static final double DRIVE_BACK_LEFT_FF = 0.001;

    public static final int FRONT_LEFT_INDEX = 0;
    public static final int FRONT_RIGHT_INDEX = 1;
    public static final int BACK_LEFT_INDEX = 2;
    public static final int BACK_RIGHT_INDEX = 3;

    public static final int FRONT_LEFT_FORWARD = 311;
    public static final int FRONT_RIGHT_FORWARD = 182;
    public static final int BACK_LEFT_FORWARD = 209;
    public static final int BACK_RIGHT_FORWARD = 59;

    
    public static final double TALON_WHEEL_RADIUS = 1.5;// inches
    public static final double TALON_GEAR_RATIO = 5.25;
    public static final double SECONDS_PER_MINUTE = 60;

    public static final double NEO_WHEEL_RADIUS = 2.0;
    public static final double NEO_DRIVE_GEAR_RATIO = 6.75;
    public static final double NEO_STEER_GEAR_RATIO = 150.0 / 7.0;

    public static final double ENCODER_RESOLUTION = 4096;

    public static final double RAD_PER_ROT = 2 * Math.PI;

    public static final boolean TALON_BOT = false;
    public static final double MAX_SPEED = 4.4;//Meters per second
    public static final double MAX_ACC = 4.75;
    public static final double MAX_ANGULAR_SPEED = 8 * Math.PI;//Half rotation per second
    public static final double MAX_ANGULAR_ACC = 1.5 * Math.PI;
    public static final double WHEEL_DIST = Constants.TALON_BOT ?  Units.feetToMeters(0.5) : Units.feetToMeters(1);
    
    public static final double TRIGGER_DEADZONE = 0.1;
    public static final double JSTICK_DEADZONE = 0.15;

    public static final double SHOOTER_DEADZONE = 100.0;

    public static final double RAMP_UP_DEADZONE = 1.0;
    public static final double FIRE_DEADZONE = 5.0;

    public static final double HOOD_MAX = 0.0;
    public static final double HOOD_MIN = -20.0;
    public static final double HOOD_DEADZONE = 0.025;
    public static final double SHOOTER_MAX = 3000.0;
    public static final double SHOOTER_MIN = 0.0;

    public static final float CLIMBER_SOFT_LIMIT = -310.f;
}
