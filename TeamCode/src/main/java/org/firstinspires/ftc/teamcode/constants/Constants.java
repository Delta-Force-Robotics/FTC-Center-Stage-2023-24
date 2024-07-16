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

    public enum ExtendoState {
        MANUAL_CONTROL,
        PRESET_POSITIONS;
    }

    public static InputState SLIDE_INPUT_STATE = InputState.MANUAL_CONTROL;
    public static ExtendoState EXTENDO_STATE = ExtendoState.MANUAL_CONTROL;


    //Slide Subsystem/
    public static final double SLIDE_INTAKE = 0;
    public static final int SLIDE_CLIMB_POS = 0;

    public static final int SLIDE_INIT_POS = 0;
    public static double SLIDE_MANUAL_CONTROL_MAX = 2930;
    public static int SLIDE_MAX_EXTENSION_TICKS = 2930;
    public static double SLIDE_MAX_EXTENSION_METERS = 0.615;
    public static double SLIDE_MOTOR_PASSIVE_POWER = 0.3;
    public static double SLIDE_ALLOWED_ERROR = 0.03;
    public static double[] SLIDE_POSITIONS_AUTO = {0.0, 0.0, 0.0, 0.0};
    public static double[] SLIDE_POSITIONS_TELEOP = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    public static final double INTAKE_SERVO_INIT_POS = 0.08;
    public static final double INTAKE_SERVO_INTAKE_POS = 0.44;
    public static final double INTAKE_SERVO_FIRST_PIXEL_POS =0.275;
    public static final double INTAKE_SERVO_SECOND_PIXEL_POS =0.3;
    public static final double INTAKE_SERVO_THIRD_PIXEL_POS = 0.35;
    public static final double INTAKE_SERVO_FOURTH_PIXEL_POS =0.38;

    public static final double ARM_SERVO_INIT_POSITION = 0.166;
    public static final double ARM_SERVO_PIVOT_POSITION = 0.684;
    public static final double ARM_SERVO_SETLINE_3_POSITION = 0.551;
    public static final double ARM_SERVO_INTAKE_POS = 0.036;
    public static final double ARM_SERVO_INIT_AUTO_POSITION = 0.0;

    public static final double PIVOT_SERVO_INIT_POSITION = 0.18;
    public static final double PIVOT_SERVO_PIVOT_POSITION = 0.6;
    public static final double PIVOT_SERVO_SETLINE_3_POSITION = 0.74;
    public static final double PIVOT_SERVO_PURPLE_POSITION = 0.86;

    public static final double ANGLE_INIT_POSITION = 0.278;
    public static final double ANGLE_REVERSED = 0.842;
    public static final double ANGLE_L = 0.477;
    public static final double ANGLE_LINE_L = 0.56;
    public static final double ANGLE_R = 0.09;
    public static final double ANGLE_LINE_R = 0;

    public static final double CLAW_SERVO_OPEN_POS = 0.0;
    public static final double CLAW_SERVO_RIGHT_OPEN_POS = 0.0;
    public static final double CLAW_SERVO_LEFT_OPEN_POS = 0.02;
    public static final double CLAW_SERVO_CLOSE_POS = 0.5;
    public static final double CLAW_SERVO_RIGHT_CLOSE_POS = 0.5;
    public static final double CLAW_SERVO_LEFT_CLOSE_POS = 0.56;
    public static final int CLAW_BOTH = 0;
    public static final int CLAW_LEFT = 1;
    public static final int CLAW_RIGHT = 2;

    public static final double DRONE_SERVO_INIT_POS = 0;
    public static final double DRONE_SERVO_SCORE_POS = 0.45;

    public static final double TRAP_OPEN = 0.06;
    public static final double TRAP_BLOCK = 0.56;
}