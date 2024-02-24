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
public class BoardAutoBlueSecret extends LinearOpMode {

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

        webcam = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, BarCodeDetection.Color.BLUE);
        webcam.init();

        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, FtcDashboard.getInstance().getTelemetry(), true, true);
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
        drive.setPoseEstimate(new Pose2d(11.6, 61.5, Math.toRadians(90)));

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
        sleep(3000);
    }

    private void CaseA() {


        trajPreloadCaseA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(23, 37.5, Math.toRadians(90)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();


        trajPreloadScoreCaseA = drive.trajectorySequenceBuilder(trajPreloadCaseA.end())
                .lineTo(new Vector2d(23,60),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(51, 45, Math.toRadians(181)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.01);
                })
                .build();

        trajToIntakeCaseA = drive.trajectorySequenceBuilder(trajPreloadCaseA.end())
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, 59,Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38,59,Math.toRadians(180)),Math.toRadians(0))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.017))
                .lineToLinearHeading(new Pose2d(-58,40,Math.toRadians(35)))
                .build();

        trajToScoreCaseA = drive.trajectorySequenceBuilder(trajToIntakeCaseA.end())
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                })
                .addDisplacementMarker(this::outtakeRoutine)
                .lineToLinearHeading(new Pose2d(-38,59,Math.toRadians(180)))
                .lineTo(new Vector2d(24,59))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = true;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] + 0.008);
                })
                .splineToLinearHeading(new Pose2d(51.1,43,Math.toRadians(180)),Math.toRadians(0))
                .build();

        parkSpotA = drive.trajectorySequenceBuilder(trajToScoreCaseA.end())
                .lineTo(new Vector2d(42,43))
                .addDisplacementMarker(() -> {
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55,60, Math.toRadians(180)), Math.toRadians(0))
                .build();



        drive.followTrajectorySequence(trajPreloadCaseA);
        sleep(600);

        drive.followTrajectorySequence(trajPreloadScoreCaseA);
        sleep(500);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
        sleep(500);

        drive.followTrajectorySequence(trajToIntakeCaseA);
        sleep(1000);

        drive.followTrajectorySequence(trajToScoreCaseA);
        sleep(900);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
        sleep(500);

        drive.followTrajectorySequence(parkSpotA);
    }

    private void CaseB() {

        trajPreloadCaseB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(11.6, 33),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();



        trajPreloadScoreCaseB = drive.trajectorySequenceBuilder(trajPreloadCaseB.end())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(50.8, 37.7, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] + 0.005);
                })

                .build();

        trajToIntakeCaseB = drive.trajectorySequenceBuilder(trajPreloadCaseB.end())
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, 59,Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38,59,Math.toRadians(180)),Math.toRadians(180))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.017))
                .lineToLinearHeading(new Pose2d(-58,40,Math.toRadians(215)))
                .build();

        trajToScoreCaseB = drive.trajectorySequenceBuilder(trajToIntakeCaseB.end())
                .lineToLinearHeading(new Pose2d(-38,59,Math.toRadians(180)))
                .addDisplacementMarker(this::outtakeRoutine)
                .lineTo(new Vector2d(24,59))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] + 0.008);
                })
                .splineToLinearHeading(new Pose2d(51.1,43,Math.toRadians(180)),Math.toRadians(0))
                .build();

        parkSpotB = drive.trajectorySequenceBuilder(trajToScoreCaseB.end())
                .lineTo(new Vector2d(42,43))
                .addDisplacementMarker(() -> {
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55,60, Math.toRadians(180)), Math.toRadians(0))
                .build();



        drive.followTrajectorySequence(trajPreloadCaseB);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS);
        sleep(600);

        drive.followTrajectorySequence(trajPreloadScoreCaseB);
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.032);
        sleep(1500);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
        sleep(850);

        drive.followTrajectorySequence(parkSpotB);
    }

    private void CaseC() {

        trajPreloadCaseC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(8, 34.5, Math.toRadians(0)), Math.toRadians(200),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();



        trajPreloadScoreCaseC = drive.trajectorySequenceBuilder(trajPreloadCaseC.end())
                //.setTangent(Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(48, 33, Math.toRadians(183)), Math.toRadians(300))
                .lineToLinearHeading(new Pose2d(51.5, 30, Math.toRadians(183)))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] + 0.005);
                })
                .build();

        trajToIntakeCaseC = drive.trajectorySequenceBuilder(trajPreloadCaseC.end())
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24, 58,Math.toRadians(180)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38,58,Math.toRadians(180)),Math.toRadians(180))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.017))
                .lineToLinearHeading(new Pose2d(-58,40,Math.toRadians(215)))
                .build();

        trajToScoreCaseC = drive.trajectorySequenceBuilder(trajToIntakeCaseC.end())
                .lineToLinearHeading(new Pose2d(-38,58,Math.toRadians(180)))
                .addDisplacementMarker(this::outtakeRoutine)
                .lineTo(new Vector2d(24,59))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] + 0.008);
                })
                .splineToLinearHeading(new Pose2d(51.1,43,Math.toRadians(180)),Math.toRadians(0))
                .build();

        parkSpotC = drive.trajectorySequenceBuilder(trajToScoreCaseC.end())
                .lineTo(new Vector2d(42,43))
                .addDisplacementMarker(() -> {
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(55,60, Math.toRadians(180)), Math.toRadians(0))
                .build();

        drive.followTrajectorySequence(trajPreloadCaseC);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS);
        sleep(600);

        drive.followTrajectorySequence(trajPreloadScoreCaseC);
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.028);
        sleep(1500);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
        sleep(850);

        drive.followTrajectorySequence(parkSpotC);
    }

    public void intakeRoutine(double intakeLevel) {
        intakeSubsystem.setIntakePos(intakeLevel);
        intakeMotor.set(0.8);
        sleep(1000);

        intakeSubsystem.setIntakePower(0);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
    }
    public void outtakeRoutine() {
        new Thread(() -> {
            scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
            intakeSubsystem.setIntakePos(0.030);
            intakeSubsystem.setIntakePower(-1);
            sleep(2000);

            intakeSubsystem.setIntakePower(0);
        }).start();
    }
}

