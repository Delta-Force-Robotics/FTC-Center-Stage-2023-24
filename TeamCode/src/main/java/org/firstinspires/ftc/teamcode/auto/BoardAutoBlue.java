package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.IntakeAutoThread;
import org.firstinspires.ftc.teamcode.threads.IntakeThread;
import org.firstinspires.ftc.teamcode.threads.ScoreThread;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

import java.util.function.Consumer;

@Autonomous
public class BoardAutoBlue extends LinearOpMode {

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
    private TrajectorySequence trajPreloadScoreCaseA;
    private TrajectorySequence trajPreloadScoreCaseB;
    private TrajectorySequence trajPreloadScoreCaseC;
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

    private Servo intakeServo;
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo rotateServo;
    private Servo droneServo;
    private Servo blockServo;

    private Servo preloadServo;

    private IntakeThread intakeThread;
    private ScoreThread scoreThread;
    private IntakeAutoThread intakeAutoThread;

    private Consumer<Double> intakeThreadExecutor;
    private Consumer<Double> scoreThreadExecutor;

    private IMU imu;

    @Override
    public void runOpMode() {
        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        intakeMotor = new Motor(hardwareMap, HardwareConstants.ID_INTAKE_MOTOR);

        intakeServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_SERVO);
        rotateServo = hardwareMap.get(Servo.class, HardwareConstants.ID_FLIP_SERVO);
        blockServo = hardwareMap.get(Servo.class, HardwareConstants.ID_BLOCK_SERVO);
        armServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_LEFT);
        armServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_RIGHT);
        droneServo = hardwareMap.get(Servo.class, HardwareConstants.ID_DRONE_SERVO);

        preloadServo = hardwareMap.get(Servo.class, HardwareConstants.ID_PRELOAD_SERVO);

        preloadServo.setDirection(Servo.Direction.REVERSE);
        preloadServo.setPosition(Constants.PRELOAD_SERVO_INIT_POS);

        webcam = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, BarCodeDetection.Color.BLUE);
        webcam.init();

        BNO055IMUNew.Parameters parameters = new BNO055IMUNew.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(IMU.class,"imu");
        imu.initialize(parameters);

        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, FtcDashboard.getInstance().getTelemetry(), true, true);
        scoreSubsystem = new ScoreSubsystem(armServoLeft, armServoRight, rotateServo, blockServo, droneServo, true);
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakeServo);

        scoreThread = new ScoreThread(slideSubsystem, scoreSubsystem);

        preloadThread = new Thread(() -> {
            preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS);
        });

        scoreAutoThread = new Thread(() -> {
            slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1]);
            sleep(100);

            scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);
            scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_45);
        });

        retractThread = new Thread(() -> {
            scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_INIT_POSITION);
            scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION);
            sleep(500);

            slideSubsystem.setLevel(Constants.SLIDE_INTAKE);
        });

        intakeThreadExecutor = (Double intakeLevel) -> {
            intakeAutoThread.intakeLevel = intakeLevel;
            intakeAutoThread.interrupt();
            intakeAutoThread.start();
        };

        scoreThreadExecutor = (Double levelForSlides) -> {
            scoreThread.slideLevel = levelForSlides;
            scoreThread.interrupt();
            scoreThread.start();
        };

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(11.6, 63.5, Math.toRadians(270)));

        trajPreloadCaseA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
               .setTangent(Math.toRadians(270))
               .splineToLinearHeading(new Pose2d(23, 37.5, Math.toRadians(90)), Math.toRadians(270))
                .build();

        trajPreloadCaseB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(11.6, 30.5))
                .build();

        trajPreloadCaseC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(5.5, 35, Math.toRadians(0)), Math.toRadians(200))
                .build();

        trajPreloadScoreCaseA = drive.trajectorySequenceBuilder(trajPreloadCaseA.end())
                .lineTo(new Vector2d(23,60))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(44.5, 40.5, Math.toRadians(180)), Math.toRadians(270))
                .lineTo(new Vector2d(45.5, 40.5))
                .build();

        trajPreloadScoreCaseB = drive.trajectorySequenceBuilder(trajPreloadCaseB.end())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(42, 35, Math.toRadians(180)), Math.toRadians(270))
                .build();

        trajPreloadScoreCaseC = drive.trajectorySequenceBuilder(trajPreloadCaseC.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(42, 28.5, Math.toRadians(180)), Math.toRadians(270))
                .build();

        parkSpotA = drive.trajectorySequenceBuilder(trajPreloadScoreCaseA.end())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(57.5,59, Math.toRadians(180)), Math.toRadians(350))
                .build();

        parkSpotB = drive.trajectorySequenceBuilder(trajPreloadScoreCaseB.end())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(57.5,59, Math.toRadians(180)), Math.toRadians(350))
                .build();

        parkSpotC = drive.trajectorySequenceBuilder(trajPreloadScoreCaseC.end())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(57.5,59, Math.toRadians(180)), Math.toRadians(350))
                .build();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Element position", webcam.getBarcodePosition());
            telemetry.update();
            barcodePosition = webcam.getBarcodePosition();
        }
        Thread stopCamera = new Thread(() -> webcam.stopCamera());
        stopCamera.start();

        waitForStart();

       if(barcodePosition == BarCodeDetection.BarcodePosition.RIGHT) {
            CaseC();
        } else if(barcodePosition == BarCodeDetection.BarcodePosition.MIDDLE) {
            CaseB();
        }   else {
            CaseA();
        }
        sleep(3000);
    }

    private void CaseA() {
       /* drive.setPoseEstimate(new Pose2d(11.6, 63.5, Math.toRadians(270)));

        drive.followTrajectorySequence(trajPreloadCaseA);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_LEFT_POS);
        sleep(600);

        /*drive.followTrajectorySequence(trajToScoreCaseA);
        scoreAutoThread.start();
        retractThread.start();

        /*drive.followTrajectorySequence(trajToIntakeCaseA);
        intakeRoutine(Constants.INTAKE_SERVO_UP_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseA);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseA);
        intakeRoutine(Constants.INTAKE_SERVO_LOW_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseA);
        scoreAutoThread.start();
        retractThread.start();*/

        drive.followTrajectorySequence(parkSpotA);
    }

    private void CaseB() {
        /*drive.setPoseEstimate(new Pose2d(11.6, 63.5, Math.toRadians(270)));

        drive.followTrajectorySequence(trajPreloadCaseB);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_LEFT_POS);
        sleep(600);

        /*drive.followTrajectorySequence(trajToScoreCaseB);
        scoreAutoThread.start();
        retractThread.start();

        /*drive.followTrajectorySequence(trajToIntakeCaseB);
        intakeRoutine(Constants.INTAKE_SERVO_UP_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseB);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseB);
        intakeRoutine(Constants.INTAKE_SERVO_LOW_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseB);
        scoreAutoThread.start();
        retractThread.start();*/

        drive.followTrajectorySequence(parkSpotB);
    }

    private void CaseC() {
        /*drive.setPoseEstimate(new Pose2d(11.6, 63.5, Math.toRadians(270)));

        drive.followTrajectorySequence(trajPreloadCaseC);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_LEFT_POS);
        sleep(600);

        /*drive.followTrajectorySequence(trajToScoreCaseC);
        scoreAutoThread.start();
        retractThread.start();

       /* drive.followTrajectorySequence(trajToIntakeCaseC);
        intakeRoutine(Constants.INTAKE_SERVO_UP_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseC);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseC);
        intakeRoutine(Constants.INTAKE_SERVO_LOW_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseC);
        scoreAutoThread.start();
        retractThread.start();*/

        drive.followTrajectorySequence(parkSpotC);
    }

    public void intakeRoutine(double intakeLevel) {
       /* intakeThreadExecutor.accept(intakeLevel);
        sleep(1000);

        intakeSubsystem.setIntakePower(0);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
        sleep(300);

        scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_30);*/
    }
}

