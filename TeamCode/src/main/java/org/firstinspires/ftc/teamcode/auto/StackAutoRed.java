package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Consumer;
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
import org.opencv.core.Mat;

import java.util.Vector;

@Autonomous
public class StackAutoRed extends LinearOpMode {

    private BarcodeUtil webcam;
    private BarCodeDetection.BarcodePosition barcodePosition;

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
    private TrajectorySequence trajToIntake2CaseA;
    private TrajectorySequence trajToIntake2CycleCaseA;
    private TrajectorySequence trajToIntake2CycleCaseB;
    private TrajectorySequence trajToIntake2CycleCaseC;
    private TrajectorySequence trajToIntake2CaseB;
    private TrajectorySequence trajToIntake2CaseC;
    private TrajectorySequence trajToScoreCaseA;
    private TrajectorySequence trajToScoreCaseB;
    private TrajectorySequence trajToScoreCaseC;
    private TrajectorySequence trajToScore2CaseA;
    private TrajectorySequence trajToScore2CycleCaseA;
    private TrajectorySequence trajToScore2CycleCaseB;
    private TrajectorySequence trajToScore2CycleCaseC;

    private TrajectorySequence trajToScore2CaseB;
    private TrajectorySequence trajToScore2CaseC;


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
        drive.setPoseEstimate(new Pose2d(-36, -61.5, Math.toRadians(270)));

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
        // Build the trajectories
        trajPreloadCaseA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-51,-21,Math.toRadians(135)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntakeCaseA = drive.trajectorySequenceBuilder(trajPreloadCaseA.end())
                .lineTo(new Vector2d(-57,-21),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-55,-12, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCaseA = drive.trajectorySequenceBuilder(trajToIntakeCaseA.end())
                .lineTo(new Vector2d(32,-12),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(49,-27),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntake2CaseA = drive.trajectorySequenceBuilder(trajToScoreCaseA.end())
                .lineTo(new Vector2d(32,-15.5),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-59,-15.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakeP = drive.trajectorySequenceBuilder(trajToIntake2CaseA.end())
                .lineToLinearHeading(new Pose2d(-50, -15.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakeSA = drive.trajectorySequenceBuilder(trajToIntakeP.end())
                .lineToLinearHeading(new Pose2d(32, -15.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScore2CaseA = drive.trajectorySequenceBuilder(trajToIntakeSA.end())
                .lineTo(new Vector2d(49,-32),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntake2CycleCaseA = drive.trajectorySequenceBuilder(trajToScore2CaseA.end())
                .lineTo(new Vector2d(32,-22),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-59,-20, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakeP2A = drive.trajectorySequenceBuilder(trajToIntake2CycleCaseA.end())
                .lineTo(new Vector2d(-53, -22),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakeS2A = drive.trajectorySequenceBuilder(trajToIntakeP2A.end())
                .lineToLinearHeading(new Pose2d(32, -22, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScore2CycleCaseA = drive.trajectorySequenceBuilder(trajToIntakeS2A.end())
                .lineTo(new Vector2d(50,-38),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadCaseA);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS);
        sleep(300);

        drive.followTrajectorySequence(trajToIntakeCaseA);

        drive.followTrajectorySequence(trajToScoreCaseA);
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0]- 0.03);
        sleep(850);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
        sleep(600);
        intakeRoutine(Constants.INTAKE_SERVO_UP_POS + 0.004);

        drive.followTrajectorySequence(trajToIntake2CaseA);
        sleep(600);

        intakeRoutine(Constants.INTAKE_SERVO_INTAKE_POS);
        drive.followTrajectorySequence(trajToIntakeP);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        sleep(400);
        intakeSubsystem.setIntakePower(-0.7);

        drive.followTrajectorySequence(trajToIntakeSA);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
        intakeSubsystem.setIntakePower(0);

        drive.followTrajectorySequence(trajToScore2CaseA);

        scoreThread.selectRotate = true;
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
        sleep(850);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
        sleep(700);
        intakeRoutine(Constants.INTAKE_SERVO_LOW_POS + 0.005);

        drive.followTrajectorySequence(trajToIntake2CycleCaseA);
        sleep(350);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
        sleep(400);

        drive.followTrajectorySequence(trajToIntakeP2A);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        intakeSubsystem.setIntakePower(-0.7);

        drive.followTrajectorySequence(trajToIntakeS2A);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
        intakeSubsystem.setIntakePower(0);

        drive.followTrajectorySequence(trajToScore2CycleCaseA);
        scoreThread.selectRotate = true;
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
        sleep(850);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
    }

    private void CaseB() {
        // Build the trajectories
        trajPreloadCaseB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-40, -24.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntakeCaseB = drive.trajectorySequenceBuilder(trajPreloadCaseB.end())
                .lineTo(new Vector2d(-53,-24.5),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(-53,-11),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCaseB = drive.trajectorySequenceBuilder(trajToIntakeCaseB.end())
                .lineTo(new Vector2d(32,-12),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(49,-40),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntake2CaseB = drive.trajectorySequenceBuilder(trajToScoreCaseB.end())
                .lineTo(new Vector2d(32,-15.5),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-58.8,-15.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakePB = drive.trajectorySequenceBuilder(trajToIntake2CaseB.end())
                .lineToLinearHeading(new Pose2d(-50, -15.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakeSB = drive.trajectorySequenceBuilder(trajToIntakePB.end())
                .lineToLinearHeading(new Pose2d(32, -15.5, Math.toRadians(185)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScore2CaseB = drive.trajectorySequenceBuilder(trajToIntakeSB.end())
                .lineTo(new Vector2d(49,-32),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntake2CycleCaseB = drive.trajectorySequenceBuilder(trajToScore2CaseB.end())
                .lineTo(new Vector2d(32,-22.5),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-59.4,-22.5, Math.toRadians(185)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakeP2B = drive.trajectorySequenceBuilder(trajToIntake2CycleCaseB.end())
                .lineTo(new Vector2d(-53, -22.5),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakeS2B = drive.trajectorySequenceBuilder(trajToIntakeP2B.end())
                .lineToLinearHeading(new Pose2d(32, -22.5, Math.toRadians(185)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScore2CycleCaseB = drive.trajectorySequenceBuilder(trajToIntakeS2B.end())
                .lineToLinearHeading(new Pose2d(50, -38, Math.toRadians(185)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadCaseB);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS);
        sleep(300);

        drive.followTrajectorySequence(trajToIntakeCaseB);

        drive.followTrajectorySequence(trajToScoreCaseB);
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0]- 0.05);
        sleep(850);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
        sleep(600);
        intakeRoutine(Constants.INTAKE_SERVO_UP_POS);

        drive.followTrajectorySequence(trajToIntake2CaseB);
        sleep(600);

        intakeRoutine(Constants.INTAKE_SERVO_INTAKE_POS);
        drive.followTrajectorySequence(trajToIntakePB);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        sleep(400);
        intakeSubsystem.setIntakePower(-0.7);

        drive.followTrajectorySequence(trajToIntakeSB);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
        intakeSubsystem.setIntakePower(0);

        drive.followTrajectorySequence(trajToScore2CaseB);

        scoreThread.selectRotate = true;
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
        sleep(850);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
        sleep(700);
        intakeRoutine(Constants.INTAKE_SERVO_LOW_POS + 0.005);

        drive.followTrajectorySequence(trajToIntake2CycleCaseB);
        sleep(350);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
        sleep(400);

        drive.followTrajectorySequence(trajToIntakeP2B);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        intakeSubsystem.setIntakePower(-0.7);

        drive.followTrajectorySequence(trajToIntakeS2B);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
        intakeSubsystem.setIntakePower(0);

        drive.followTrajectorySequence(trajToScore2CycleCaseB);
        scoreThread.selectRotate = true;
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
        sleep(850);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
    }

    private void CaseC() {
        // Build the trajectories
        trajPreloadCaseC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(95))
                .splineToLinearHeading(new Pose2d(-35,-35, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntakeCaseC = drive.trajectorySequenceBuilder(trajPreloadCaseC.end())
                .lineTo(new Vector2d(-34,-35),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(-52,-11),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCaseC = drive.trajectorySequenceBuilder(trajToIntakeCaseC.end())
                .lineTo(new Vector2d(32,-12),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineTo(new Vector2d(49,-40),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntake2CaseC = drive.trajectorySequenceBuilder(trajToScoreCaseC.end())
                .lineTo(new Vector2d(32,-15.5),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-58.8,-15.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakePC = drive.trajectorySequenceBuilder(trajToIntake2CaseC.end())
                .lineToLinearHeading(new Pose2d(-50, -15.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakeSC = drive.trajectorySequenceBuilder(trajToIntakePC.end())
                .lineToLinearHeading(new Pose2d(32, -15.5, Math.toRadians(185)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScore2CaseC = drive.trajectorySequenceBuilder(trajToIntakeSC.end())
                .lineTo(new Vector2d(49,-32),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntake2CycleCaseC = drive.trajectorySequenceBuilder(trajToScore2CaseC.end())
                .lineTo(new Vector2d(32,-22.5),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-59.4,-22.5, Math.toRadians(185)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakeP2C = drive.trajectorySequenceBuilder(trajToIntake2CycleCaseC.end())
                .lineTo(new Vector2d(-53, -22.5),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence trajToIntakeS2C = drive.trajectorySequenceBuilder(trajToIntakeP2C.end())
                .lineToLinearHeading(new Pose2d(32, -22.5, Math.toRadians(185)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScore2CycleCaseC = drive.trajectorySequenceBuilder(trajToIntakeS2C.end())
                .lineToLinearHeading(new Pose2d(50, -38, Math.toRadians(185)),
                        SampleMecanumDrive.getVelocityConstraint(65, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadCaseC);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS);
        sleep(300);

        drive.followTrajectorySequence(trajToIntakeCaseC);

        drive.followTrajectorySequence(trajToScoreCaseC);
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0]- 0.05);
        sleep(850);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
        sleep(600);
        intakeRoutine(Constants.INTAKE_SERVO_UP_POS);

        drive.followTrajectorySequence(trajToIntake2CaseC);
        sleep(600);

        intakeRoutine(Constants.INTAKE_SERVO_INTAKE_POS);
        drive.followTrajectorySequence(trajToIntakePC);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        sleep(400);
        intakeSubsystem.setIntakePower(-0.7);

        drive.followTrajectorySequence(trajToIntakeSC);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
        intakeSubsystem.setIntakePower(0);

        drive.followTrajectorySequence(trajToScore2CaseC);

        scoreThread.selectRotate = true;
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
        sleep(850);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
        sleep(700);
        intakeRoutine(Constants.INTAKE_SERVO_LOW_POS + 0.005);

        drive.followTrajectorySequence(trajToIntake2CycleCaseC);
        sleep(350);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
        sleep(400);

        drive.followTrajectorySequence(trajToIntakeP2C);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        intakeSubsystem.setIntakePower(-0.7);

        drive.followTrajectorySequence(trajToIntakeS2C);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
        intakeSubsystem.setIntakePower(0);

        drive.followTrajectorySequence(trajToScore2CycleCaseC);
        scoreThread.selectRotate = true;
        scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
        sleep(850);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
    }

    public void intakeRoutine(double intakeLevel) {
        intakeSubsystem.setIntakePower(1);
        intakeServo.setPosition(intakeLevel - 0.012);
        //sleep(3000);

        /*intakeSubsystem.setIntakePower(-0.7);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        sleep(1000);*/

        /*scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);

        intakeSubsystem.setIntakePower(0);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);*/
    }

    public void scoreRoutine() {
        slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[3]);
        sleep(150);

        scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);
        scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_45);
    }

    public void retractRoutine() {
        scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_INIT_POSITION);
        sleep(100);
        scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION);
        sleep(300);

        slideSubsystem.setLevel(Constants.SLIDE_INTAKE);
    }
}

