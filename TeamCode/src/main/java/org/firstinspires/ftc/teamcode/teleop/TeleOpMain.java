package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.teamcode.commands.ClimbManualCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.SlideManualCommand;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.BackupThread;
import org.firstinspires.ftc.teamcode.threads.IntakeThread;
import org.firstinspires.ftc.teamcode.threads.OutakeThread;
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

    private IntakeThread intakeThread;
    private ScoreThread scoreThread;
    private ScoreReleaseThread scoreReleaseThread;
    private OutakeThread outakeThread;
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

       /* BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(parameters);*/

        driveSubsystem = new DriveSubsystem(driveLeftFront, driveLeftBack, driveRightFront, driveRightBack);
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakeServo);
        scoreSubsystem = new ScoreSubsystem(armServoLeft, armServoRight, rotateServo, blockServo, droneServo, false);
        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, FtcDashboard.getInstance().getTelemetry(), true, false);
        climbSubsystem = new ClimbSubsystem(climbMotor);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftX, driver1::getLeftY, driver1::getRightX);
        slideManualCommand = new SlideManualCommand(slideSubsystem, () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        climbManualCommand = new ClimbManualCommand(climbSubsystem, () -> driver2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driver2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        // touchSensorCommand = new TouchSensorCommand(scoreSubsystem, intakeSubsystem);

        intakeThread = new IntakeThread(intakeSubsystem, scoreSubsystem, false);
        scoreThread = new ScoreThread(slideSubsystem, scoreSubsystem);

        scoreReleaseThread = new ScoreReleaseThread(slideSubsystem, scoreSubsystem);
        outakeThread = new OutakeThread(intakeSubsystem, scoreSubsystem);
        backupThread = new BackupThread(scoreSubsystem);

        intakeThread.setPriority(Thread.MIN_PRIORITY);
        outakeThread.setPriority(Thread.MIN_PRIORITY);
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

        changeLevelUp = new InstantCommand(() -> {
            // slideSubsystem.setLevel(0.2);
            if (currLevel > 0 && currLevel < 6) {
                currLevel++;
                telemetry.addData("CURRENT LEVEL", currLevel);
                telemetry.update();
            }
        });

        changeLevelDown = new InstantCommand(() -> {
            //slideSubsystem.setLevel(0);
            if (currLevel > 0 && currLevel < 6) {
                currLevel--;
                telemetry.addData("CURRENT LEVEL", currLevel);
                telemetry.update();
            }
        });

        droneInstantCommand = new InstantCommand(() -> {
            scoreSubsystem.setDroneServoPos(Constants.DRONE_SERVO_SCORE_POS);
        });

        rotateInit = new InstantCommand(() -> {
            scoreThread.selectRotate = false;
            backupThread.selectRotate = false;
        });

        rotate45 = new InstantCommand(() -> {
           scoreThread.selectRotate = true;
            backupThread.selectRotate = true;
        });

        new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> scoreReleaseThreadExecutor.accept(Constants.SLIDE_INTAKE));

        new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(droneInstantCommand);

        new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> backupThread.start());

        new GamepadButton(driver2, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[currLevel-1]));

        new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(changeLevelUp);
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(changeLevelDown);

        new GamepadButton(driver2, GamepadKeys.Button.DPAD_LEFT).whenPressed(rotateInit);
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).whenPressed(rotate45);
        new GamepadButton(driver2, GamepadKeys.Button.Y).whenPressed(() -> climbSubsystem.setClimbPos(Constants.CLIMB_MOTOR_CLIMB_POS));
        new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(() -> climbSubsystem.setClimbPos(Constants.CLIMB_MOTOR_INIT_POS));

        new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(() -> intakeThread.start());
        new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(() -> outakeThread.start());
        driveSubsystem.setDefaultCommand(driveCommand);
        slideSubsystem.setDefaultCommand(slideManualCommand);
        climbSubsystem.setDefaultCommand(climbManualCommand);

        telemetry.addData("CURRENT LEVEL", currLevel);
        telemetry.update();
    }
}
