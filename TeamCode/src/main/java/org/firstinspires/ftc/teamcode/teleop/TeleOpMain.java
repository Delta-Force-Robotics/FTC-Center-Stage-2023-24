package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.SlideManualCommand;
import org.firstinspires.ftc.teamcode.commands.TouchSensorCommand;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.IntakeThread;
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

    private Servo clawServo;
    private Servo intakeServo;
    private Servo pivotServo;
    private Servo rotateServo;
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo droneServo;
    private Servo climbServoLeft;
    private Servo climbServoRight;

    private DigitalChannel touchSensorLeft;
    private DigitalChannel touchSensorRight;

    private DriveSubsystem driveSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private SlideSubsystem slideSubsystem;
    private ClimbSubsystem climbSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private DriveCommand driveCommand;
    private SlideManualCommand slideManualCommand;
    private TouchSensorCommand touchSensorCommand;

    private IntakeThread intakeThread;
    private ScoreThread scoreThread;
    private ScoreReleaseThread scoreReleaseThread;

    private InstantCommand useClaw;
    private InstantCommand changeLevelUp;
    private InstantCommand changeLevelDown;
    private InstantCommand droneInstantCommand;
    private InstantCommand climbUpInstantCommand;
    private InstantCommand climbDownInstantCommand;
    private InstantCommand climbServoInstantCommand;
    private InstantCommand stopIntakeInstantCommand;
    private InstantCommand descoreInstantCommand;
    private InstantCommand rotate180;
    private InstantCommand rotate45;

    private GamepadEx driver1;
    private GamepadEx driver2;

    private boolean clawBool;

    @Override
    public void initialize() {

        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;

        driveLeftFront = new Motor(hardwareMap, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        driveLeftBack = new Motor(hardwareMap, HardwareConstants.ID_LEFT_BACK_MOTOR);
        driveRightFront = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        driveRightBack = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_BACK_MOTOR);

        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        intakeMotor = new Motor(hardwareMap, HardwareConstants.ID_INTSKE_MOTOR);
        climbMotor = new Motor(hardwareMap, HardwareConstants.ID_CLIMB_MOTOR);

        armServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_LEFT);
        armServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_RIGHT);
        clawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_SERVO);
        intakeServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_SERVO);
        pivotServo = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_CLAW_SERVO);
        rotateServo = hardwareMap.get(Servo.class, HardwareConstants.ID_FLIP_SERVO);
        droneServo = hardwareMap.get(Servo.class, HardwareConstants.ID_DRONE_SERVO);
        climbServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_LEFT_CLIMB_SERVO);
        climbServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_RIGHT_CLIMB_SERVO);

        touchSensorLeft = hardwareMap.get(DigitalChannel.class, "touchSensorLeft");
        touchSensorRight = hardwareMap.get(DigitalChannel.class, "touchSensorRight");

        driveSubsystem = new DriveSubsystem(driveLeftFront, driveLeftBack, driveRightFront, driveRightBack);
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakeServo);
        scoreSubsystem = new ScoreSubsystem(clawServo, pivotServo, armServoLeft, armServoRight, rotateServo, droneServo, touchSensorLeft, touchSensorRight, false);
        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, telemetry, true);
        climbSubsystem = new ClimbSubsystem(climbMotor, climbServoLeft, climbServoRight);

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        driveCommand = new DriveCommand(driveSubsystem, driver1::getLeftY, driver1::getRightX);
        slideManualCommand = new SlideManualCommand(slideSubsystem, () -> driver1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driver1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        touchSensorCommand = new TouchSensorCommand(scoreSubsystem, intakeSubsystem);

        intakeThread = new IntakeThread(intakeSubsystem, false);
        scoreThread = new ScoreThread(slideSubsystem, scoreSubsystem);
        scoreReleaseThread = new ScoreReleaseThread(scoreSubsystem, slideSubsystem);

        Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
        intakeThread.setPriority(Thread.MIN_PRIORITY);
        scoreThread.setPriority(Thread.MIN_PRIORITY);
        scoreReleaseThread.setPriority(Thread.MIN_PRIORITY);

        clawBool = false;
        useClaw = new InstantCommand(() -> {
            if (clawBool == false) {
                scoreSubsystem.useClaw(Constants.OPEN_CLAW);
                clawBool = true;
            } else {
                scoreSubsystem.useClaw(Constants.CLOSE_CLAW_TELEOP);
                clawBool = false;
            }
        });

        changeLevelUp = new InstantCommand(() -> {
            slideSubsystem.setCurrentLevel(slideSubsystem.getCurrLevel()+1);
        });

        changeLevelDown = new InstantCommand(() -> {
            slideSubsystem.setCurrentLevel(slideSubsystem.getCurrLevel()-1);
        });

        stopIntakeInstantCommand = new InstantCommand(() -> {
           scoreSubsystem.useArm(Constants.ARM_SERVO_INTAKE_POS);
           sleep(50);

           scoreSubsystem.useClaw(Constants.CLOSE_CLAW_TELEOP);
           scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_30);

           intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
           intakeSubsystem.setIntakePower(-1);
           sleep(2000);

           intakeSubsystem.setIntakePower(0);
        });

        descoreInstantCommand = new InstantCommand(() -> {
           scoreSubsystem.useClaw(Constants.CLOSE_CLAW_TELEOP);
        });

        droneInstantCommand = new InstantCommand(() -> {
            scoreSubsystem.setDroneServoPos(Constants.DRONE_SERVO_SCORE_POS);
        });

        rotate180 = new InstantCommand(() -> {
            scoreThread.selectRotate = true;
        });

        rotate45 = new InstantCommand(() -> {
            scoreThread.selectRotate = false;
        });

        climbUpInstantCommand = new InstantCommand(() -> {
            climbSubsystem.setClimbPos(Constants.CLIMB_MOTOR_CLIMB_POS);
        });

        climbDownInstantCommand = new InstantCommand(() -> {
            climbSubsystem.setClimbPos(Constants.CLIMB_MOTOR_INIT_POS);
        });

        climbServoInstantCommand = new InstantCommand(() -> {
            climbSubsystem.setClimbServoPos(Constants.CLIMB_SERVO_CLIMB_POS);
        });

        new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> scoreReleaseThread.start());
        new GamepadButton(driver1, GamepadKeys.Button.LEFT_BUMPER).whenPressed(descoreInstantCommand);

        new GamepadButton(driver1, GamepadKeys.Button.DPAD_UP).whenPressed(climbUpInstantCommand);
        new GamepadButton(driver1, GamepadKeys.Button.DPAD_DOWN).whenPressed(climbDownInstantCommand);
        new GamepadButton(driver1, GamepadKeys.Button.DPAD_LEFT).whenPressed(climbServoInstantCommand);

        new GamepadButton(driver1, GamepadKeys.Button.Y).whenPressed(droneInstantCommand);

        new GamepadButton(driver1, GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> scoreThread.start());

        new GamepadButton(driver2, GamepadKeys.Button.DPAD_UP).whenPressed(changeLevelUp);
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_DOWN).whenPressed(changeLevelDown);

        new GamepadButton(driver2, GamepadKeys.Button.DPAD_LEFT).whenPressed(rotate180);
        new GamepadButton(driver2, GamepadKeys.Button.DPAD_RIGHT).whenPressed(rotate45);

        new GamepadButton(driver2, GamepadKeys.Button.A).whenPressed(() -> intakeThread.start());
        new GamepadButton(driver2, GamepadKeys.Button.B).whenPressed(stopIntakeInstantCommand);
        new GamepadButton(driver2, GamepadKeys.Button.X).whenPressed(useClaw);
        driveSubsystem.setDefaultCommand(driveCommand);
        slideSubsystem.setDefaultCommand(slideManualCommand);
       // scoreSubsystem.setDefaultCommand(touchSensorCommand);
    }
}
