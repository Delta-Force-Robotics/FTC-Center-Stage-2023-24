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
import org.firstinspires.ftc.teamcode.threads.OuttakeThread;
import org.firstinspires.ftc.teamcode.threads.ScoreReleaseThread;
import org.firstinspires.ftc.teamcode.threads.ScoreThread;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

@Autonomous
public class StackAutoRed extends LinearOpMode {

    private BarcodeUtil webcam;
    private BarCodeDetection.BarcodePosition barcodePosition;

    private SampleMecanumDrive drive;

    private SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private TrajectorySequence trajPreloadSplineCaseA;
    private TrajectorySequence trajPreloadSplineCaseB;
    private TrajectorySequence trajPreloadSplineCaseC;
    private TrajectorySequence trajToIntakeSplineCaseA;
    private TrajectorySequence trajToIntakeSplineCaseB;
    private TrajectorySequence trajToIntakeSplineCaseC;
    private TrajectorySequence trajToScoreSplineCaseA;
    private TrajectorySequence trajToScoreSplineCaseB;
    private TrajectorySequence trajToScoreSplineCaseC;
    private TrajectorySequence trajToIntakeCycleSplineCaseA;
    private TrajectorySequence trajToIntakeCycleSplineCaseB;
    private TrajectorySequence trajToIntakeCycleSplineCaseC;
    private TrajectorySequence trajToScoreCycleSplineCaseA;
    private TrajectorySequence trajToScoreCycleSplineCaseB;
    private TrajectorySequence trajToScoreCycleSplineCaseC;
    private TrajectorySequence trajToParkSplineB;

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
    private OuttakeThread outtakeThread;

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
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);

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

        if (barcodePosition == BarCodeDetection.BarcodePosition.LEFT) {
            CaseA();
        } else if (barcodePosition == BarCodeDetection.BarcodePosition.RIGHT) {
            CaseC();
        } else CaseB();

        sleep(30000);
    }

    private void CaseA() {
        // Build the trajectories
        trajPreloadSplineCaseA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                /*.setTangent(Math.toRadians(155))
                .lineToLinearHeading(new Pose2d(-52, -25.5, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))*/
                .setTangent(Math.toRadians(120))
                .splineToLinearHeading(new Pose2d(-49,-26.3,Math.toRadians(90)),Math.toRadians(85),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();

        trajToIntakeSplineCaseA = drive.trajectorySequenceBuilder(trajPreloadSplineCaseA.end())
                /*.splineToLinearHeading(new Pose2d(-54,-9, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.017))
                .splineToLinearHeading(new Pose2d(-58,-12, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))*/
                /*.lineToLinearHeading(new Pose2d(-52, -23, Math.toRadians(184)))
                .splineToLinearHeading(new Pose2d(-57 ,-9, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))*/
                .lineToLinearHeading(new Pose2d(-49,-25,Math.toRadians(180)))
                .setTangent(Math.toRadians(250))
                .lineToLinearHeading(new Pose2d(-50 ,-12, Math.toRadians(184)))
                .build();

        trajToScoreSplineCaseA = drive.trajectorySequenceBuilder(trajToIntakeSplineCaseA.end())
                /*.lineTo(new Vector2d(-48,-12),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                })
                .lineTo(new Vector2d(-44,-12),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)*/
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(24,-11,Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.02);
                })
                .splineToLinearHeading(new Pose2d(52,-22.2, Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker( () -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                /*//.lineTo(new Vector2d(-48,-12))
                 .lineTo(new Vector2d(-44,-12))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-12,Math.toRadians(184)),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(53,-27, Math.toRadians(184)),Math.toRadians(0))*/
                .build();

        trajToIntakeCycleSplineCaseA = drive.trajectorySequenceBuilder(trajToScoreSplineCaseA.end())
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,-12, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.024))
                .lineToLinearHeading(new Pose2d(-57.5,-16, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycleSplineCaseA = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseA.end())
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(24,-14, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = true;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] + 0.006);
                })
                .splineToLinearHeading(new Pose2d(52,-32, Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadSplineCaseA);
        sleep(300);

        drive.followTrajectorySequence(trajToIntakeSplineCaseA);
        sleep(500);

        drive.followTrajectorySequence(trajToScoreSplineCaseA);
        sleep(850);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseA);
        sleep(1500);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseA);
        sleep(400);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
    }

    private void CaseB() {
        // Build the trajectories
        trajPreloadSplineCaseB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-48, -25 , Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();

        trajToIntakeSplineCaseB = drive.trajectorySequenceBuilder(trajPreloadSplineCaseB.end())
                //.addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS))
                .lineToLinearHeading(new Pose2d(-50, -25 , Math.toRadians(184)))
                .splineToLinearHeading(new Pose2d(-57.2,-12, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreSplineCaseB = drive.trajectorySequenceBuilder(trajToIntakeSplineCaseB.end())
               /* .lineTo(new Vector2d(-48,-12),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                })
                .lineTo(new Vector2d(-44,-12),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)*/
                /*.lineTo(new Vector2d(-48,-12),
                         SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 .addDisplacementMarker(() -> {
                     intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                 })
                 .lineTo(new Vector2d(-44,-12),
                         SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                         SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                 .addDisplacementMarker(this::outtakeRoutine)*/
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(24,-11,Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.025);
                })
                .splineToLinearHeading(new Pose2d(52,-31, Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker( () -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                /*//.lineTo(new Vector2d(-48,-12))
                 .lineTo(new Vector2d(-44,-12))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-12,Math.toRadians(184)),Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(53,-27, Math.toRadians(184)),Math.toRadians(0))*/
                .build();

        trajToIntakeCycleSplineCaseB = drive.trajectorySequenceBuilder(trajToScoreSplineCaseB.end())
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,-12, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.017))
                .lineToLinearHeading(new Pose2d(-58.8,-16, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycleSplineCaseB = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseB.end())
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(24,-14, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = true;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1] + 0.003);
                })
                .splineToLinearHeading(new Pose2d(52,-35, Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadSplineCaseB);
        sleep(300);

        drive.followTrajectorySequence(trajToIntakeSplineCaseB);
        sleep(500);

        drive.followTrajectorySequence(trajToScoreSplineCaseB);
        sleep(1000);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseB);
        sleep(1500);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseB);
        sleep(400);
        retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
    }

    private void CaseC() {
        // Build the trajectories
        trajPreloadSplineCaseC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-31,-34, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();

        trajToIntakeSplineCaseC = drive.trajectorySequenceBuilder(trajPreloadSplineCaseC.end())
                /*.addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.015))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57,-14,Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))*/
                .lineToLinearHeading(new Pose2d(-34,-34, Math.toRadians(180)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50,-14,Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreSplineCaseC = drive.trajectorySequenceBuilder(trajToIntakeSplineCaseC.end())
                /*.lineTo(new Vector2d(-48,-14),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                })
                .lineTo(new Vector2d(-44,-14),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)*/
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-14,Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.029);
                })
                .splineToLinearHeading(new Pose2d(53.5,-37.6,Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker( () -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        trajToIntakeCycleSplineCaseC = drive.trajectorySequenceBuilder(trajToScoreSplineCaseC.end())
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,-13, Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.018))
                .lineToLinearHeading(new Pose2d(-58.8,-16, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycleSplineCaseC = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseC.end())
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(24,-14, Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = true;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1] + 0.003);
                })
                .splineToLinearHeading(new Pose2d(52,-34, Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .build();

        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadSplineCaseC);
        sleep(500);

        drive.followTrajectorySequence(trajToIntakeSplineCaseC);
        sleep(300);

        drive.followTrajectorySequence(trajToScoreSplineCaseC);
        sleep(800);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseC);
        sleep(900);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseC);
    }

    public void intakeRoutine(double intakeLevel) {
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
        intakeSubsystem.setIntakePower(1);
        intakeServo.setPosition(intakeLevel);
    }

    public void outtakeRoutine() {
        new Thread(() -> {
            scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
            intakeSubsystem.setIntakePos(0.030);
            intakeSubsystem.setIntakePower(-1);
            sleep(2500);

            intakeSubsystem.setIntakePower(0);
        }).start();
    }
}

