package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.SlideManualCommand;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.CatchThread;
import org.firstinspires.ftc.teamcode.threads.IntakeThread;
import org.firstinspires.ftc.teamcode.threads.OuttakeThread;
import org.firstinspires.ftc.teamcode.threads.ScoreReleaseThread;

@TeleOp
public class TeleOpMain extends CommandOpMode {
    private DriveSubsystem driveSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private SlideSubsystem slideSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private DriveCommand driveCommand;
    private SlideManualCommand slideManualCommand;
    private PivotCommand pivotCommand;

    private CatchThread catchThread;
    private ScoreReleaseThread scoreReleaseThread;
    private IntakeThread intakeThread;
    private OuttakeThread outtakeThread;

    private InstantCommand droneInstantCommand;
    private InstantCommand trapClosed;
    private InstantCommand trapOpen;
    private InstantCommand rotateL;
    private InstantCommand rotateR;
    private InstantCommand rotateInit;
    private InstantCommand clawL;
    private InstantCommand clawR;
    private InstantCommand rotateLineL;
    private InstantCommand rotateLineR;
    private InstantCommand rotateReversed;
    private InstantCommand setline3;

    private GamepadEx driver1;
    private GamepadEx driver2;

    private Consumer<Integer> slideThreadExecutor;

    private DistanceSensor distanceSensorL;
    private DistanceSensor distanceSensorR;

    private boolean clawBool;
    public int rotatePosL = 0;
    public int rotatePosR = 0;

    public Motor slideMotorLeft;
    public Motor slideMotorRight;

    private Servo armServoL;
    private Servo armServoR;
    private Servo pivotServo;
    private Servo rotateServo;
    private Servo clawServoL;
    private Servo clawServoR;
    private Servo droneServo;

    private Motor intakeMotor;
    private Servo intakeServo;
    private Servo trap;

    public Motor extendoMotor;

    private Motor leftFront;
    private Motor leftBack;
    private Motor rightFront;
    private Motor rightBack;

    @Override
    public void initialize() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        Constants.EXTENDO_STATE = Constants.ExtendoState.MANUAL_CONTROL;

        leftFront = new Motor(hardwareMap, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        leftBack = new Motor(hardwareMap, HardwareConstants.ID_LEFT_BACK_MOTOR);
        rightFront = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        rightBack = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_BACK_MOTOR);

        armServoR = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_RIGHT);
        armServoL = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_LEFT);
        rotateServo = hardwareMap.get(Servo.class, HardwareConstants.ID_ROTATE_SERVO);
        pivotServo = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO);
        clawServoL = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_SERVO_L);
        clawServoR = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_SERVO_R);
        droneServo = hardwareMap.get(Servo.class, HardwareConstants.ID_DRONE_SERVO);

        intakeServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_SERVO);
        trap = hardwareMap.get(Servo.class, HardwareConstants.ID_TRAP_SERVO);
        intakeMotor = new Motor(hardwareMap, HardwareConstants.ID_INTAKE_MOTOR);

        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        driveSubsystem = new DriveSubsystem(leftFront, leftBack, rightFront, rightBack);
        intakeSubsystem = new IntakeSubsystem(intakeServo, trap, intakeMotor);
        scoreSubsystem = new ScoreSubsystem(armServoR, armServoL, rotateServo, pivotServo, clawServoL, clawServoR, droneServo, false);
        slideSubsystem = new SlideSubsystem(FtcDashboard.getInstance().getTelemetry(), slideMotorRight, slideMotorLeft, true);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);
        slideManualCommand = new SlideManualCommand(slideSubsystem, scoreSubsystem, () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        pivotCommand = new PivotCommand(scoreSubsystem, slideSubsystem, intakeSubsystem);

        scoreReleaseThread = new ScoreReleaseThread(slideSubsystem, scoreSubsystem, intakeSubsystem);
        catchThread = new CatchThread(scoreSubsystem, intakeSubsystem);
        intakeThread = new IntakeThread(intakeSubsystem, scoreSubsystem);
        outtakeThread = new OuttakeThread(intakeSubsystem);

        catchThread.setPriority(Thread.MIN_PRIORITY);
        scoreReleaseThread.setPriority(Thread.MIN_PRIORITY);
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);

        droneInstantCommand = new InstantCommand(() -> {
            scoreSubsystem.useDrone(Constants.DRONE_SERVO_SCORE_POS);
        });

        rotateL = new InstantCommand(() -> {
            scoreSubsystem.rotateClaw(Constants.ANGLE_L);
        });

        rotateR = new InstantCommand(() -> {
            scoreSubsystem.rotateClaw(Constants.ANGLE_R);
        });

        rotateLineL = new InstantCommand(() -> {
            scoreSubsystem.rotateClaw(Constants.ANGLE_LINE_L);
        });

        rotateLineR = new InstantCommand(() -> {
            scoreSubsystem.rotateClaw(Constants.ANGLE_LINE_R);
        });

        rotateInit = new InstantCommand(() -> {
            scoreSubsystem.rotateClaw(Constants.ANGLE_INIT_POSITION);
        });

        rotateReversed = new InstantCommand(() -> {
            scoreSubsystem.rotateClaw(Constants.ANGLE_REVERSED);
        });

        trapClosed = new InstantCommand(() -> {
            intakeSubsystem.useTrap(Constants.TRAP_BLOCK);
        });

        trapOpen = new InstantCommand(() -> {
            intakeSubsystem.useTrap(Constants.TRAP_OPEN);
        });

        clawL = new InstantCommand(() -> {
            scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_LEFT);
        });

        clawR = new InstantCommand(() -> {
            scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_RIGHT);
        });

        setline3 = new InstantCommand(() -> {
            scoreSubsystem.useArm(Constants.ARM_SERVO_SETLINE_3_POSITION);
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_SETLINE_3_POSITION);
        });

        new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> scoreReleaseThread.start());
        new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> catchThread.start());
        new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(droneInstantCommand);
        new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenPressed(() -> slideSubsystem.resetEnc());

        new GamepadButton(driver2, GamepadKeys.Button.X).toggleWhenPressed(trapClosed, trapOpen);
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_LEFT).toggleWhenPressed(rotateL, rotateLineL);
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).toggleWhenPressed(rotateR, rotateLineR);
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).toggleWhenPressed(rotateReversed, rotateInit);
        new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER).whenPressed(clawR);
        new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(clawL);
        new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(setline3);
        new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(() -> intakeThread.start());
        new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(() -> outtakeThread.start());
        driveSubsystem.setDefaultCommand(driveCommand);
        slideSubsystem.setDefaultCommand(slideManualCommand);
        scoreSubsystem.setDefaultCommand(pivotCommand);
    }
}
