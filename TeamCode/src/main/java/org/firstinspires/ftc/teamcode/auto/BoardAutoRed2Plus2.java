package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.IntakeAutoThread;
import org.firstinspires.ftc.teamcode.threads.IntakeThread;
import org.firstinspires.ftc.teamcode.threads.ScoreReleaseThread;
import org.firstinspires.ftc.teamcode.threads.ScoreThread;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

import java.util.function.Consumer;

@Autonomous
public class BoardAutoRed2Plus2 extends LinearOpMode {

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

    private TrajectorySequence trajToIntakeCycle1CaseA;
    private TrajectorySequence trajToIntakeCycle1CaseB;
    private TrajectorySequence trajToIntakeCycle1CaseC;

    private TrajectorySequence trajToScoreCycle1CaseA;
    private TrajectorySequence trajToScoreCycle1CaseB;
    private TrajectorySequence trajToScoreCycle1CaseC;

    private TrajectorySequence trajToIntakeCycle2CaseA;
    private TrajectorySequence trajToIntakeCycle2CaseB;
    private TrajectorySequence trajToIntakeCycle2CaseC;

    private TrajectorySequence trajToScoreCycle2CaseA;
    private TrajectorySequence trajToScoreCycle2CaseB;
    private TrajectorySequence trajToScoreCycle2CaseC;

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
    private ScoreReleaseThread scoreReleaseThread;

    private Consumer<Double> scoreThreadExecutor;
    private Consumer<Double> retractThreadExecutor;

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

        webcam = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, BarCodeDetection.Color.RED);
        webcam.init();

        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, FtcDashboard.getInstance().getTelemetry(), true, true, hardwareMap);
        scoreSubsystem = new ScoreSubsystem(armServoLeft, armServoRight, rotateServo, blockServo, droneServo, true);
        intakeSubsystem = new IntakeSubsystem(intakeMotor, intakeServo);

        scoreThread = new ScoreThread(slideSubsystem, scoreSubsystem);
        scoreReleaseThread = new ScoreReleaseThread(slideSubsystem, scoreSubsystem);

        scoreThreadExecutor = (Double levelForSlides) -> {
            scoreThread.slideLevel = levelForSlides;
            scoreThread.interrupt();
            scoreThread.start();
        };

        retractThreadExecutor = (Double levelForSlides) -> {
            scoreReleaseThread.slideLevel = levelForSlides;
            scoreReleaseThread.interrupt();
            scoreReleaseThread.start();
        };

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(11.6, -61.5, Math.toRadians(270)));

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
        } else if(barcodePosition == BarCodeDetection.BarcodePosition.RIGHT) {
            CaseC();
        }   else {
            CaseB();
        }
        sleep(3000);
    }

    private void CaseA() {
        trajPreloadCaseA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(7, -34, Math.toRadians(0)), Math.toRadians(200),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();

        trajPreloadScoreCaseA = drive.trajectorySequenceBuilder(trajPreloadCaseA.end())
                .splineToLinearHeading(new Pose2d(9, -34, Math.toRadians(0)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(38, -27, Math.toRadians(180)), Math.toRadians(45),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.025);
                })
                .splineToLinearHeading(new Pose2d(53.5, -26.8, Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        trajToIntakeCycle1CaseA = drive.trajectorySequenceBuilder(trajPreloadScoreCaseA.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[0] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,-13.5, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS - 0.012))
                .splineToLinearHeading(new Pose2d(-57.3,-13.8, Math.toRadians(184)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.08))
                .splineToLinearHeading(new Pose2d(-57, -17, Math.toRadians(195)), Math.toRadians(180))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .splineToLinearHeading(new Pose2d(-57.5,-13.8, Math.toRadians(184)), Math.toRadians(180))
                .build();

        trajToScoreCycle2CaseA = drive.trajectorySequenceBuilder(trajToIntakeCycle1CaseA.end())
                .lineToLinearHeading(new Pose2d(-53,-14.3, Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-15, Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1] - 0.03);
                })
                .splineToLinearHeading(new Pose2d(53.7,-37, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        /*trajToIntakeCycle2CaseA = drive.trajectorySequenceBuilder(trajToScoreCycle1CaseA.end())
                .addDisplacementMarker(() -> {
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,-14, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_INTAKE_POS - 0.01))
                .lineToLinearHeading(new Pose2d(-56.7,-19, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycle2CaseA = drive.trajectorySequenceBuilder(trajToIntakeCycle2CaseA.end())
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .lineToLinearHeading(new Pose2d(-54,-17, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(24,-18, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
                })
                .splineToLinearHeading(new Pose2d(52.3,-34.5, Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .build();*/

        parkSpotA = drive.trajectorySequenceBuilder(trajToScoreCycle2CaseA.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .lineTo(new Vector2d(45,-36),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(245))
                .splineToLinearHeading(new Pose2d(52.3,-60, Math.toRadians(180)), Math.toRadians(5),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectorySequence(trajPreloadCaseA);

        drive.followTrajectorySequence(trajPreloadScoreCaseA);
        sleep(100);

        drive.followTrajectorySequence(trajToIntakeCycle1CaseA);
        sleep(500);

        drive.followTrajectorySequence(trajToScoreCycle2CaseA);
        sleep(100);

        drive.followTrajectorySequence(parkSpotA);
    }

    private void CaseB() {


        trajPreloadCaseB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(11.6, -32.4),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();


        trajPreloadScoreCaseB = drive.trajectorySequenceBuilder(trajPreloadCaseB.end())
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.04);
                })
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(54, -38.53, Math.toRadians(178)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        trajToIntakeCycle1CaseB = drive.trajectorySequenceBuilder(trajPreloadScoreCaseB.end())
                .addDisplacementMarker(() -> {
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,-9, Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS - 0.012))
                .splineToLinearHeading(new Pose2d(-55.6,-7.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.08))
                .splineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(190)), Math.toRadians(180))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .splineToLinearHeading(new Pose2d(-55.6,-9, Math.toRadians(180)), Math.toRadians(180))
                .build();

        trajToScoreCycle2CaseB = drive.trajectorySequenceBuilder(trajToIntakeCycle1CaseB.end())
                .lineToLinearHeading(new Pose2d(-53,-13.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(19,-14.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1] - 0.04);
                })
                .splineToLinearHeading(new Pose2d(55.3,-36.4, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        /*trajToIntakeCycle2CaseA = drive.trajectorySequenceBuilder(trajToScoreCycle1CaseA.end())
                .addDisplacementMarker(() -> {
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,-14, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_INTAKE_POS - 0.01))
                .lineToLinearHeading(new Pose2d(-56.7,-19, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycle2CaseA = drive.trajectorySequenceBuilder(trajToIntakeCycle2CaseA.end())
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .lineToLinearHeading(new Pose2d(-54,-17, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(24,-18, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
                })
                .splineToLinearHeading(new Pose2d(52.3,-34.5, Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .build();*/

        parkSpotB = drive.trajectorySequenceBuilder(trajToScoreCycle2CaseB.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1] + 0.02);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .lineTo(new Vector2d(45,-36),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(245))
                .splineToLinearHeading(new Pose2d(52.3,-60, Math.toRadians(180)), Math.toRadians(5),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();




        drive.followTrajectorySequence(trajPreloadCaseB);

        drive.followTrajectorySequence(trajPreloadScoreCaseB);
        sleep(100);

        drive.followTrajectorySequence(trajToIntakeCycle1CaseB);
        sleep(500);

        drive.followTrajectorySequence(trajToScoreCycle2CaseB);
        sleep(100);

        drive.followTrajectorySequence(parkSpotB);
    }

    private void CaseC() {


        trajPreloadCaseC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(80))
                .splineToLinearHeading(new Pose2d(23, -37.5, Math.toRadians(270)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();


        trajPreloadScoreCaseC = drive.trajectorySequenceBuilder(trajPreloadCaseC.end())
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.022);
                })
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(53.4, -47, Math.toRadians(178)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        trajToIntakeCycle1CaseC = drive.trajectorySequenceBuilder(trajPreloadScoreCaseC.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[0] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,-11, Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS - 0.012))
                .splineToLinearHeading(new Pose2d(-56.7,-9, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.08))
                .splineToLinearHeading(new Pose2d(-57.3, -14.5, Math.toRadians(195)), Math.toRadians(180))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .splineToLinearHeading(new Pose2d(-57.3,-12, Math.toRadians(180)), Math.toRadians(180))
                .build();

        trajToScoreCycle2CaseC = drive.trajectorySequenceBuilder(trajToIntakeCycle1CaseC.end())
                .lineToLinearHeading(new Pose2d(-53,-13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-13, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1] - 0.03);
                })
                .splineToLinearHeading(new Pose2d(53.7,-36, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        /*trajToIntakeCycle2CaseA = drive.trajectorySequenceBuilder(trajToScoreCycle1CaseA.end())
                .addDisplacementMarker(() -> {
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,-14, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_INTAKE_POS - 0.01))
                .lineToLinearHeading(new Pose2d(-56.7,-19, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycle2CaseA = drive.trajectorySequenceBuilder(trajToIntakeCycle2CaseA.end())
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .lineToLinearHeading(new Pose2d(-54,-17, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(24,-18, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
                })
                .splineToLinearHeading(new Pose2d(52.3,-34.5, Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .build();*/

        parkSpotC = drive.trajectorySequenceBuilder(trajToScoreCycle2CaseC.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .lineTo(new Vector2d(45,-36),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(245))
                .splineToLinearHeading(new Pose2d(52.3,-60, Math.toRadians(180)), Math.toRadians(5),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();




        drive.followTrajectorySequence(trajPreloadCaseC);

        drive.followTrajectorySequence(trajPreloadScoreCaseC);
        sleep(100);

        drive.followTrajectorySequence(trajToIntakeCycle1CaseC);
        sleep(500);

        drive.followTrajectorySequence(trajToScoreCycle2CaseC);
        sleep(100);

        drive.followTrajectorySequence(parkSpotC);
    }

    public void intakeRoutine(double intakeLevel) {
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
        intakeSubsystem.setIntakePower(1);
        intakeServo.setPosition(intakeLevel);
    }

    public void outtakeRoutine() {
        new Thread(() -> {
            scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
            intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
            intakeSubsystem.setIntakePower(-1);
            sleep(2000);

            intakeSubsystem.setIntakePower(0);
        }).start();
    }
}

