package org.firstinspires.ftc.teamcode.constants;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * A class containing all constants for subsystems. <br><br>
 * This allows us to share constants across OP modes, and makes it easier to change them.
 */

@Config
public class Constants {
    public static boolean ROBOT_STOPPED = false;

    public enum InputState {
        MANUAL_CONTROL,
        PRESET_POSITIONS;

    }

    public static InputState SLIDE_INPUT_STATE = InputState.MANUAL_CONTROL;


    //Slide Subsystem/
    public static final double SLIDE_INTAKE = 0.0;
    public static final double SLIDE_CLIMB_POS = 0.0;

    public static final double SLIDE_INIT_POS = 0;
    public static double SLIDE_MANUAL_CONTROL_MAX = 0.72;
    public static double SLIDE_MAX_EXTENSION_TICKS = 1048;
    public static double SLIDE_MAX_EXTENSION_METERS = 0.822;
    public static double SLIDE_MOTOR_PASSIVE_POWER = 0.25;
    public static double SLIDE_MOTOR_PASSIVE_POWER_CLAW_WEIGHT = 0.1;
    public static double SLIDE_ALLOWED_ERROR = 0.01;
    public static double SLIDE_ALLOWED_VELOCITY_ERROR = 0.07;
    public static double SLIDE_MAX_ERROR_INTEGRATION = 0.1;
    public static double[] SLIDE_POSITIONS = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    public static PIDFCoefficients SLIDE_PIDF_COEFF = new PIDFCoefficients(0, 0, 0, 0);
    public static FeedforwardCoefficients SLIDE_FEEDFORWARD_COEFF = new FeedforwardCoefficients(0, 0, 0);
    public static double SPOOL_SIZE_METERS = 0;
    public static double SLIDE_MAX_VELOCITY = 0;
    public static double SLIDE_MAX_ACCEL = 0;

    public static final double OPEN_CLAW = 0.0;
    public static final double CLOSE_CLAW_AUTO = 0.0;
    public static final double CLOSE_CLAW_TELEOP = 0.0;

    public static final double INTAKE_SERVO_INIT_POS = 0;
    public static final double INTAKE_SERVO_INTAKE_POS = 0;
    public static final double INTAKE_SERVO_FIRST_PIXEL_POS = 0;
    public static final double INTAKE_SERVO_UP_POS = 0;
    public static final double INTAKE_SERVO_MID_POS = 0;
    public static final double INTAKE_SERVO_LOW_POS = 0;

    public static final double INTAKE_SERVO_FIRST_PIXEL_POS_AUTO = 0;
    public static final double INTAKE_SERVO_UP_POS_AUTO = 0;
    public static final double INTAKE_SERVO_LOW_POS_AUTO = 0;

    public static final double PIVOT_INIT_POS = 0;
    public static final double PIVOT_PIVOT_POS = 0;
    public static final double PIVOT_INTAKE_POS = 0;
    public static final double PIVOT_AUTO_POS = 0;

    public static final double ARM_SERVO_INIT_POSITION = 0.0;
    public static final double ARM_SERVO_PIVOT_POSITION = 0.0;
    public static final double ARM_SERVO_PIVOT_30 = 0.0;
    public static final double ARM_SERVO_INTAKE_POS = 0.0;
    public static final double ARM_SERVO_INIT_AUTO_POSITION = 0.0;

    public static final double ROTATE_SERVO_INIT_POSITION = 0;
    public static final double ROTATE_SERVO_45 = 0.0;
    public static final double ROTATE_SERVO_180 = 0.0;

    public static final double DRONE_SERVO_INIT_POS = 0.0;
    public static final double DRONE_SERVO_SCORE_POS = 0.0;

    public static final double CLIMB_SERVO_INIT_POS = 0;
    public static final double CLIMB_SERVO_CLIMB_POS = 0;
    public static final int CLIMB_MOTOR_INIT_POS = 0;
    public static final int CLIMB_MOTOR_CLIMB_POS = 0;

    public static final double PRELOAD_SERVO_INIT_POS = 0;
    public static final double PRELOAD_SERVO_SCORE_POS = 0;

    //April Tag IDs
    public static final int APRIL_TAG_PARK_ZONE_1 = 13;
    public static final int APRIL_TAG_PARK_ZONE_2 = 23;
    public static final int APRIL_TAG_PARK_ZONE_3 = 31;
    }

