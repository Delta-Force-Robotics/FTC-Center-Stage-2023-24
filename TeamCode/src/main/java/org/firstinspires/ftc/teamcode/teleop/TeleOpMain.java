package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.teamcode.commands.ClimbManualCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.SlideManualCommand;
import org.firstinspires.ftc.teamcode.commands.SlideOuttakeCommand;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.BackupThread;
import org.firstinspires.ftc.teamcode.threads.ClimbThread;
import org.firstinspires.ftc.teamcode.threads.IntakeThread;
import org.firstinspires.ftc.teamcode.threads.OuttakeThread;
import org.firstinspires.ftc.teamcode.threads.ScoreReleaseThread;
import org.firstinspires.ftc.teamcode.threads.ScoreThread;

@TeleOp
public class TeleOpMain extends CommandOpMode {
    private IMU imu;
    private Motor driveLeftFront;
    private Motor driveLeftBack;
    private Motor driveRightFront;
    private Motor driveRightBack;
    private Motor slideMotorLeft;
    private Motor slideMotorRight;
    private Motor intakeMotor;
    private Motor climbMotor;

    private Servo intakeServo;
    private Servo rotateServo;
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo blockServo;
    private Servo droneServo;

    private DriveSubsystem driveSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private SlideSubsystem slideSubsystem;
    private ClimbSubsystem climbSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private DriveCommand driveCommand;
    private SlideManualCommand slideManualCommand;
    private ClimbManualCommand climbManualCommand;
    private SlideOuttakeCommand slideOuttakeCommand;

    private IntakeThread intakeThread;
    private ClimbThread climbThread;
    private ScoreThread scoreThread;
    private ScoreReleaseThread scoreReleaseThread;
    private OuttakeThread outtakeThread;
    private BackupThread backupThread;

    private InstantCommand changeLevelUp;
    private InstantCommand changeLevelDown;
    private InstantCommand droneInstantCommand;
    private InstantCommand stopIntakeInstantCommand;
    private InstantCommand rotate45;
    private InstantCommand rotateInit;

    private GamepadEx driver1;
    private GamepadEx driver2;

    private Consumer<Double> scoreThreadExecutor;
    private Consumer<Double> climbThreadExecutor;
    private Consumer<Double> scoreReleaseThreadExecutor;

    private boolean clawBool;
    public int currLevel = 1;

    Timing.Timer timer;
    @Override
    public void initialize() {

        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        Constants.CLIMB_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;

        driveLeftFront = new Motor(hardwareMap, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        driveLeftBack = new Motor(hardwareMap, HardwareConstants.ID_LEFT_BACK_MOTOR);
        driveRightFront = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        driveRightBack = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_BACK_MOTOR);

        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        intakeMotor = new Motor(hardwareMap, HardwareConstants.ID_INTAKE_MOTOR);
        climbMotor = new Motor(hardwareMap, HardwareConstants.ID_CLIMB_MOTOR);

        armServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_LEFT);
        armServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_RIGHT);
        intakeServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_SERVO);
        rotateServo = hardwareMap.get(Servo.class, HardwareConstants.ID_FLIP_SERVO);
        blockServo = hardwareMap.get(Servo.class, HardwareConstants.ID_BLOCK_SERVO);
        droneServo = hardwareMap.get(Servo.class, HardwareConstants.ID_DRONE_SERVO);

        driveSubsystem = new DriveSubsystem(driveLeftFront, driveLeftBack, driveRightFront, driveRightBack);
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakeServo);
        scoreSubsystem = new ScoreSubsystem(armServoLeft, armServoRight, rotateServo, blockServo, droneServo, false);
        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, FtcDashboard.getInstance().getTelemetry(), true, false, hardwareMap);
        climbSubsystem = new ClimbSubsystem(climbMotor);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);
        slideManualCommand = new SlideManualCommand(slideSubsystem, () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        climbManualCommand = new ClimbManualCommand(climbSubsystem, () -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        slideOuttakeCommand = new SlideOuttakeCommand(intakeSubsystem, slideSubsystem, scoreSubsystem);

        intakeThread = new IntakeThread(intakeSubsystem, scoreSubsystem, false);
        scoreThread = new ScoreThread(slideSubsystem, scoreSubsystem);
        climbThread = new ClimbThread(climbSubsystem);

        scoreReleaseThread = new ScoreReleaseThread(slideSubsystem, scoreSubsystem);
        outtakeThread = new OuttakeThread(intakeSubsystem, scoreSubsystem);
        backupThread = new BackupThread(scoreSubsystem);

        intakeThread.setPriority(Thread.MIN_PRIORITY);
        climbThread.setPriority(Thread.MIN_PRIORITY);
        outtakeThread.setPriority(Thread.MIN_PRIORITY);
        scoreThread.setPriority(Thread.MIN_PRIORITY);
        backupThread.setPriority(Thread.MIN_PRIORITY);
        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);

        scoreThreadExecutor = (Double levelForSlides) -> {
            scoreThread.slideLevel = levelForSlides;
            scoreThread.interrupt();
            scoreThread.start();
        };

        scoreReleaseThreadExecutor = (Double levelForSlides) -> {
            scoreReleaseThread.slideLevel = levelForSlides;
            scoreReleaseThread.interrupt();
            scoreReleaseThread.start();
        };

        climbThreadExecutor = (Double climbPosition) -> {
            climbThread.climbPos = climbPosition;
            climbThread.interrupt();
            climbThread.start();
        };

        droneInstantCommand = new InstantCommand(() -> {
            scoreSubsystem.setDroneServoPos(Constants.DRONE_SERVO_SCORE_POS);
        });


        new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> scoreReleaseThreadExecutor.accept(Constants.SLIDE_INTAKE));

        new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(droneInstantCommand);
        new GamepadButton(driver1, GamepadKeys.Button.X).toggleWhenPressed(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS), () -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS));

        new GamepadButton(driver2, GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> backupThread.start());
        new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenPressed(() -> slideSubsystem.resetEnc());

        new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[currLevel-1]));

        new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(() ->scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_UP_POSITION));
        //new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_INIT_POSITION));

        //new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT).whenPressed(() ->scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_180));
        //new GamepadButton(driver1, GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_45));
        new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(() -> climbThreadExecutor.accept((double)Constants.CLIMB_MOTOR_CLIMB_POS));
        new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(() -> climbThreadExecutor.accept((double)Constants.CLIMB_MOTOR_INIT_POS));

        new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(() -> intakeThread.start());
        new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(() -> outtakeThread.start());

        driveSubsystem.setDefaultCommand(driveCommand);
        slideSubsystem.setDefaultCommand(slideManualCommand);
        climbSubsystem.setDefaultCommand(climbManualCommand);
        intakeSubsystem.setDefaultCommand(slideOuttakeCommand);

        telemetry.addData("CURRENT LEVEL", currLevel);
        telemetry.update();
    }
}
