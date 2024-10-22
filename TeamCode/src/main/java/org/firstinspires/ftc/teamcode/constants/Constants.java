package org.firstinspires.ftc.teamcode.constants;

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
    public static InputState CLIMB_INPUT_STATE = InputState.MANUAL_CONTROL;


    //Slide Subsystem/
    public static final double SLIDE_INTAKE = 0;
    public static double SLIDE_MANUAL_CONTROL_MAX = 0.72;
    public static int SLIDE_MAX_EXTENSION_TICKS = 845;
    public static double SLIDE_MAX_EXTENSION_METERS = 0.696;
    public static double SLIDE_MOTOR_PASSIVE_POWER = 0.2;
    public static double SLIDE_ALLOWED_ERROR = 0.03;
    public static double[] SLIDE_POSITIONS = {0.28, 0.35, 0.43, 0.51, 0.60, 0.67};
    public static PIDFCoefficients SLIDE_RETRACT_PIDF_COEFF = new PIDFCoefficients(5, 0, 0, 0);
    public static PIDFCoefficients SLIDE_EXTEND_PIDF_COEFF = new PIDFCoefficients(14, 0, 0, 0);

    public static final double INTAKE_SERVO_INIT_POS = 0.07;
    public static final double INTAKE_SERVO_INTAKE_POS = 0.495;
    public static final double INTAKE_SERVO_FIRST_PIXEL_POS = 0.369;
    public static final double ARM_SERVO_INIT_POSITION = 0;
    public static final double ARM_SERVO_PIVOT_POSITION = 0.46;
    public static final double ARM_SERVO_PIVOT_UP_POSITION = 0.68;

    public static final double ROTATE_SERVO_INIT_POSITION = 0.218;
    public static final double ROTATE_SERVO_45 = 0.3; //L

    public static final double BLOCK_SERVO_SCORE_POS = 0.2;
    public static final double BLOCK_SERVO_BLOCK_POS = 0;
    public static final double DRONE_SERVO_INIT_POS = 0;
    public static final double DRONE_SERVO_SCORE_POS = 0.4;

    public static final int CLIMB_MOTOR_INIT_POS = 0;
    public static final int CLIMB_MOTOR_CLIMB_POS = 7000;

    public static final double PRELOAD_SERVO_INIT_POS = 0.00;
    public static final double PRELOAD_SERVO_SCORE_POS = 0.2;
}