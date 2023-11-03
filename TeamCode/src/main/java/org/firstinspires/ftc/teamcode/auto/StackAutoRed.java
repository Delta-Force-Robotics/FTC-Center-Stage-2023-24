package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.IntakeAutoThread;
import org.firstinspires.ftc.teamcode.threads.IntakeThread;
import org.firstinspires.ftc.teamcode.threads.ScoreThread;
import org.firstinspires.ftc.teamcode.threads.SlideLevelThread;
import org.firstinspires.ftc.teamcode.roadrunner.drive.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

import java.util.function.Consumer;

@Autonomous
public class StackAutoRed extends LinearOpMode {

    private BarcodeUtil webcam;
    private BarCodeDetection.BarcodePosition barcodePosition;

    private Thread preloadThread;
    private Thread scoreAutoThread;
    private Thread retractThread;

    private SampleMecanumDrive drive;

    private SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private TrajectorySequence trajPreloadCaseA;
    private TrajectorySequence trajPreloadCaseB;
    private TrajectorySequence trajPreloadCaseC;
    private TrajectorySequence trajToIntakePreloadCaseA;
    private TrajectorySequence trajToIntakePreloadCaseB;
    private TrajectorySequence trajToIntakePreloadCaseC;
    private TrajectorySequence trajToIntakeCaseA;
    private TrajectorySequence trajToIntakeCaseB;
    private TrajectorySequence trajToIntakeCaseC;
    private TrajectorySequence trajToScoreCaseA;
    private TrajectorySequence trajToScoreCaseB;
    private TrajectorySequence trajToScoreCaseC;

    private TrajectorySequence parkSpotA;
    private TrajectorySequence parkSpotB;
    private TrajectorySequence parkSpotC;

    private Motor slideMotorLeft;
    private Motor slideMotorRight;
    private Motor intakeMotor;

    private Servo clawServo;
    private Servo intakeServo;
    private Servo pivotClawServo;
    private Servo pivotServoLeft;
    private Servo pivotServoRight;
    private Servo flipServo;
    private Servo droneServo;

    private Servo preloadServo;
    private DigitalChannel touchSensorLeft;
    private DigitalChannel touchSensorRight;

    private IntakeThread intakeThread;
    private ScoreThread scoreThread;
    private SlideLevelThread slideLevelThread;
    private IntakeAutoThread intakeAutoThread;

    private Consumer<Double> intakeThreadExecutor;
    private Consumer<Double> scoreThreadExecutor;

    private IMU imu;

    @Override
    public void runOpMode() {
        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        clawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_SERVO);
        intakeServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_SERVO);
        pivotClawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_CLAW_SERVO);
        flipServo = hardwareMap.get(Servo.class, HardwareConstants.ID_FLIP_SERVO);
        pivotServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_LEFT);
        pivotServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_RIGHT);
        droneServo = hardwareMap.get(Servo.class, HardwareConstants.ID_DRONE_SERVO);

        preloadServo = hardwareMap.get(Servo.class, HardwareConstants.ID_PRELOAD_SERVO);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_INIT_POS);

        webcam = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, 1);
        webcam.init();

        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, slideLevelThread, true);
        scoreSubsystem = new ScoreSubsystem(clawServo, pivotClawServo, pivotServoLeft, pivotServoRight, flipServo, droneServo, touchSensorLeft, touchSensorRight, true);
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakeServo);

        scoreThread = new ScoreThread(slideSubsystem, scoreSubsystem);

        preloadThread = new Thread(() -> {
            preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS);
        });

        scoreAutoThread = new Thread(() -> {
            slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1]);
            scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);
            sleep(500);

            scoreSubsystem.useClaw(Constants.OPEN_CLAW);
            sleep(500);

            scoreSubsystem.useClaw(Constants.CLOSE_CLAW_AUTO);
        });

        retractThread = new Thread(() -> {
            scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_INIT_POSITION);
            sleep(250);

            scoreSubsystem.pivotClaw(Constants.PIVOT_INIT_POS);
            scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION);

            slideSubsystem.setLevel(Constants.SLIDE_INTAKE);
        });

        intakeThreadExecutor = (Double intakeLevel) -> {
            intakeAutoThread.intakeLevel = intakeLevel;
            intakeAutoThread.interrupt();
            intakeAutoThread.start();
        };

        scoreThreadExecutor = (Double levelForSlides) -> {
            scoreThread.levelForSlides = levelForSlides;
            scoreThread.interrupt();
            scoreThread.start();
        };

        BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, -63.5, Math.toRadians(90)));

        trajPreloadCaseA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(-50, -25, Math.toRadians(90)), Math.toRadians(90))
                .build();

        trajPreloadCaseB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-36, -18))
                .build();

        trajPreloadCaseC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .splineToSplineHeading(new Pose2d(-50, -35, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-30, -35))
                .build();

        trajToIntakePreloadCaseA = drive.trajectorySequenceBuilder(trajPreloadCaseA.end())
                .lineTo(new Vector2d(-50, -11.6))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-60, -11.6))
                .build();

        trajToIntakePreloadCaseB = drive.trajectorySequenceBuilder(trajPreloadCaseB.end())
                .lineTo(new Vector2d(-36, -11.6))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-60, -11.6))
                .build();

        trajToIntakePreloadCaseC = drive.trajectorySequenceBuilder(trajPreloadCaseC.end())
                .splineToLinearHeading(new Pose2d(-36, -11.6, Math.toRadians(90)), Math.toRadians(90))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-60, -11.6))
                .build();

        trajToScoreCaseA = drive.trajectorySequenceBuilder(new Pose2d(-60, -11.6, Math.toRadians(180)))
                .lineTo(new Vector2d(20, -11.6))
                .splineToSplineHeading(new Pose2d(45, -27, Math.toRadians(180)), Math.toRadians(0))
                .build();

        trajToScoreCaseB = drive.trajectorySequenceBuilder(new Pose2d(-60, -11.6, Math.toRadians(180)))
                .lineTo(new Vector2d(20, -11.6))
                .splineToSplineHeading(new Pose2d(45, -35, Math.toRadians(180)), Math.toRadians(0))
                .build();

        trajToScoreCaseC = drive.trajectorySequenceBuilder(new Pose2d(-60, -11.6, Math.toRadians(180)))
                .lineTo(new Vector2d(20, -11.6))
                .splineToSplineHeading(new Pose2d(45, -43, Math.toRadians(180)), Math.toRadians(0))
                .build();

        trajToIntakeCaseA = drive.trajectorySequenceBuilder(trajToScoreCaseA.end())
                .splineToSplineHeading(new Pose2d(20, -11.6, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -11.6))
                .build();

        trajToIntakeCaseB = drive.trajectorySequenceBuilder(trajToScoreCaseB.end())
                .splineToSplineHeading(new Pose2d(20, -11.6, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -11.6))
                .build();

        trajToIntakeCaseC = drive.trajectorySequenceBuilder(trajToScoreCaseC.end())
                .splineToSplineHeading(new Pose2d(20, -11.6, Math.toRadians(180)), Math.toRadians(180))
                .lineTo(new Vector2d(-60, -11.6))
                .build();


        parkSpotA = drive.trajectorySequenceBuilder(trajToScoreCaseA.end())
                .lineTo(new Vector2d(50, -27))
                .build();

        parkSpotB = drive.trajectorySequenceBuilder(trajToScoreCaseB.end())
                .lineTo(new Vector2d(50, -35))
                .build();

        parkSpotC = drive.trajectorySequenceBuilder(trajToScoreCaseC.end())
                .lineTo(new Vector2d(50, -43))
                .build();


        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Element position", webcam.getBarcodePosition());
            telemetry.update();
            barcodePosition = webcam.getBarcodePosition();
        }
        Thread stopCamera = new Thread(() -> webcam.stopCamera());
        stopCamera.start();

        waitForStart();

        if(barcodePosition == BarCodeDetection.BarcodePosition.LEFT) {
            CaseA();
        } else if(barcodePosition == BarCodeDetection.BarcodePosition.MIDDLE) {
            CaseB();
        }   else {
            CaseC();
        }
        //    sleep(30000);
    }

    private void CaseA() {
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        drive.followTrajectorySequence(trajPreloadCaseA);

        preloadThread.start();
        sleep(200);

        drive.followTrajectorySequence(trajToIntakePreloadCaseA);
        intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseA);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseA);
        intakeRoutine(Constants.INTAKE_SERVO_UP_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseA);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseA);
        intakeRoutine(Constants.INTAKE_SERVO_LOW_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseA);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(parkSpotA);
    }

    private void CaseB() {
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        drive.followTrajectorySequence(trajPreloadCaseB);

        preloadThread.start();
        sleep(200);

        drive.followTrajectorySequence(trajToIntakePreloadCaseB);
        intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseB);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseB);
        intakeRoutine(Constants.INTAKE_SERVO_UP_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseB);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseB);
        intakeRoutine(Constants.INTAKE_SERVO_LOW_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseB);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(parkSpotB);
    }

    private void CaseC() {
        drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        drive.followTrajectorySequence(trajPreloadCaseC);

        preloadThread.start();
        sleep(200);

        drive.followTrajectorySequence(trajToIntakePreloadCaseC);
        intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseC);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseC);
        intakeRoutine(Constants.INTAKE_SERVO_UP_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseC);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseC);
        intakeRoutine(Constants.INTAKE_SERVO_LOW_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseC);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(parkSpotC);
    }

    public void intakeRoutine(double intakeLevel) {
        intakeThreadExecutor.accept(intakeLevel);
        sleep(400);

        intakeSubsystem.setIntakePower(0);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);

        scoreSubsystem.useClaw(Constants.CLOSE_CLAW_AUTO);
        sleep(50);
        scoreSubsystem.pivotClaw(Constants.PIVOT_PIVOT_POS);
        scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_45);
    }
}

