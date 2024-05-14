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
public class StackAutoRedCSH extends LinearOpMode {

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

        slideSubsystem = new SlideSubsystem(slideMotorLeft, slideMotorRight, FtcDashboard.getInstance().getTelemetry(), true, true, hardwareMap);
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
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-48.2,-38,Math.toRadians(270)),Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .lineTo(new Vector2d(-48.2, -40))
                .build();

        /*trajToIntakeSplineCaseA = drive.trajectorySequenceBuilder(trajPreloadSplineCaseA.end())

                .build();*/

        trajToScoreSplineCaseA = drive.trajectorySequenceBuilder(trajPreloadSplineCaseA.end())
                .lineToLinearHeading(new Pose2d(-40,-60,Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-59,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.026);
                })
                .splineToLinearHeading(new Pose2d(52,-25.2,Math.toRadians(184)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
                })
                .build();

        trajToIntakeCycleSplineCaseA = drive.trajectorySequenceBuilder(trajToScoreSplineCaseA.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[0] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(22, -60,Math.toRadians(182)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38,-61,Math.toRadians(182)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS - 0.012);
                })
                .lineToLinearHeading(new Pose2d(-55.4,-46, Math.toRadians(160)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.004);
                })
                .lineToLinearHeading(new Pose2d(-55.5,-41, Math.toRadians(160)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                })
                .lineToLinearHeading(new Pose2d(-55.4,-46, Math.toRadians(160)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycleSplineCaseA = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseA.end())
                .addDisplacementMarker(this::outtakeRoutine)
                .lineToLinearHeading(new Pose2d(-36,-61.5,Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-61.5,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1] - 0.03);
                })
                .splineToLinearHeading(new Pose2d(54.4,-35,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
                })
                .build();

        TrajectorySequence parkA = drive.trajectorySequenceBuilder(trajToScoreCycleSplineCaseA.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1] + 0.03);
                })
                .lineTo(new Vector2d(48, -35))
                .addDisplacementMarker(() -> {
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .lineToLinearHeading(new Pose2d(48,-59,Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadSplineCaseA);

        /*drive.followTrajectorySequence(trajToIntakeSplineCaseA);
        sleep(500);*/

        drive.followTrajectorySequence(trajToScoreSplineCaseA);
        sleep(150);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseA);
        sleep(500);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseA);
        sleep(150);

        drive.followTrajectorySequence(parkA);
    }

    private void CaseB() {
        // Build the trajectories
        trajPreloadSplineCaseB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(-36,-32.6),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();

        trajToScoreSplineCaseB = drive.trajectorySequenceBuilder(trajPreloadSplineCaseB.end())
                .lineToLinearHeading(new Pose2d(-40,-60.5,Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-60.5,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.028);
                })
                .splineToLinearHeading(new Pose2d(52.2,-35.8,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
                })
                .build();

        trajToIntakeCycleSplineCaseB = drive.trajectorySequenceBuilder(trajToScoreSplineCaseB.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[0] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(22, -61.8,Math.toRadians(182)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38,-61.8,Math.toRadians(182)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS - 0.012);
                })
                .lineToLinearHeading(new Pose2d(-55.8,-45, Math.toRadians(160)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.006);
                })
                .lineToLinearHeading(new Pose2d(-55.8,-41, Math.toRadians(160)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                })
                .lineToLinearHeading(new Pose2d(-55.8,-46, Math.toRadians(160)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycleSplineCaseB = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseB.end())
                .addDisplacementMarker(this::outtakeRoutine)
                .lineToLinearHeading(new Pose2d(-36,-61.8,Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-61.8,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1] - 0.03);
                })
                .splineToLinearHeading(new Pose2d(54.4,-39,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
                })
                .build();

        TrajectorySequence parkB = drive.trajectorySequenceBuilder(trajToScoreCycleSplineCaseB.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1] + 0.03);
                })
                .lineTo(new Vector2d(48, -39))
                .addDisplacementMarker(() -> {
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .lineToLinearHeading(new Pose2d(48,-59,Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadSplineCaseB);

        drive.followTrajectorySequence(trajToScoreSplineCaseB);
        sleep(150);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseB);
        sleep(500);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseB);
        sleep(150);

        drive.followTrajectorySequence(parkB);
    }

    private void CaseC() {
        // Build the trajectories
        trajPreloadSplineCaseC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-32 ,-34, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();

        trajToScoreSplineCaseC = drive.trajectorySequenceBuilder(trajPreloadSplineCaseC.end())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-36,-60, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-60,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.028);
                })
                .splineToLinearHeading(new Pose2d(52.5,-39.7,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
                })
                .build();

        trajToIntakeCycleSplineCaseC = drive.trajectorySequenceBuilder(trajToScoreSplineCaseC.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[0] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(22, -61.7,Math.toRadians(182)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-38,-61.7,Math.toRadians(182)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS - 0.012);
                })
                .lineToLinearHeading(new Pose2d(-55.4,-46, Math.toRadians(160)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.004);
                })
                .lineToLinearHeading(new Pose2d(-55.3,-42, Math.toRadians(160)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                })
                .lineToLinearHeading(new Pose2d(-55.3,-46, Math.toRadians(160)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycleSplineCaseC = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseC.end())
                .addDisplacementMarker(this::outtakeRoutine)
                .lineToLinearHeading(new Pose2d(-36,-62,Math.toRadians(182)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,-62,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1] - 0.03);
                })
                .splineToLinearHeading(new Pose2d(54.4,-35,Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
                })
                .build();

        TrajectorySequence parkC = drive.trajectorySequenceBuilder(trajToScoreCycleSplineCaseC.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1] + 0.06);
                })
                .lineToLinearHeading(new Pose2d(48, -35, Math.toRadians(184)))
                .addDisplacementMarker(() -> {
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .lineToLinearHeading(new Pose2d(53,-60,Math.toRadians(184)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadSplineCaseC);

        /*drive.followTrajectorySequence(trajToIntakeSplineCaseA);
        sleep(500);*/

        drive.followTrajectorySequence(trajToScoreSplineCaseC);
        sleep(150);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseC);
        sleep(500);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseC);
        sleep(150);

        drive.followTrajectorySequence(parkC);
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
            sleep(1800);

            intakeSubsystem.setIntakePower(0);
        }).start();
    }
}