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

@Autonomous
public class StackAutoBlue extends LinearOpMode {

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

    private org.firstinspires.ftc.robotcore.external.Consumer<Double> scoreThreadExecutor;
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
        drive.setPoseEstimate(new Pose2d(-36, 61.5, Math.toRadians(90)));

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Element position", webcam.getBarcodePosition());
            telemetry.update();
            barcodePosition = webcam.getBarcodePosition();
        }
        Thread stopCamera = new Thread(() -> webcam.stopCamera());
        stopCamera.start();

        waitForStart();

        if (barcodePosition == BarCodeDetection.BarcodePosition.MIDDLE) {
            CaseB();
        } else if (barcodePosition == BarCodeDetection.BarcodePosition.RIGHT) {
            CaseC();
        } else CaseA();

        sleep(3000);
    }

    private void CaseA() {
        // Build the trajectories

        trajPreloadSplineCaseA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-31.5,34, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();

        trajToIntakeSplineCaseA = drive.trajectorySequenceBuilder(trajPreloadSplineCaseA.end())
                /*.addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.015))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57,14,Math.toRadians(176)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))*/
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-50,11,Math.toRadians(184)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreSplineCaseA = drive.trajectorySequenceBuilder(trajToIntakeSplineCaseA.end())
                /*.lineTo(new Vector2d(-48,12),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                })
                .lineTo(new Vector2d(-44,12),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)*/
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,12,Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.0265);
                })
                .splineToLinearHeading(new Pose2d(51.5,37.6,Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker( () -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        trajToIntakeCycleSplineCaseA = drive.trajectorySequenceBuilder(trajToScoreSplineCaseA.end())
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,8, Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_UP_POS - 0.015))
                .splineToLinearHeading(new Pose2d(-56.4,12, Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycleSplineCaseA = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseA.end())
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,7, Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = true;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
                })
                .splineToLinearHeading(new Pose2d(51,34,Math.toRadians(184)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadSplineCaseA);
        sleep(300);

        drive.followTrajectorySequence(trajToIntakeSplineCaseA);
        sleep(600);

        drive.followTrajectorySequence(trajToScoreSplineCaseA);
        sleep(850);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseA);
        sleep(1000);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseA);
    }

    private void CaseB() {
        // Build the trajectories
        trajPreloadSplineCaseB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-48, 25 , Math.toRadians(177)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .lineTo(new Vector2d(-52, 25))
                .build();

        trajToIntakeSplineCaseB = drive.trajectorySequenceBuilder(trajPreloadSplineCaseB.end())
                .splineToLinearHeading(new Pose2d(-57.2,12, Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreSplineCaseB = drive.trajectorySequenceBuilder(trajToIntakeSplineCaseB.end())
                /*.lineTo(new Vector2d(-48,10),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                })
                .lineTo(new Vector2d(-44,10),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)*/
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,10,Math.toRadians(181)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.02);
                })
                .splineToLinearHeading(new Pose2d(51,3.65, Math.toRadians(181)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker( () -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        trajToIntakeCycleSplineCaseB = drive.trajectorySequenceBuilder(trajToScoreSplineCaseB.end())
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,7, Math.toRadians(182)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_UP_POS - 0.015))
                .splineToLinearHeading(new Pose2d(-56.4,11, Math.toRadians(182)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycleSplineCaseB = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseB.end())
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,7, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = true;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
                })
                .splineToLinearHeading(new Pose2d(51,34,Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadSplineCaseB);

        drive.followTrajectorySequence(trajToIntakeSplineCaseB);
        sleep(600);

        drive.followTrajectorySequence(trajToScoreSplineCaseB);
        sleep(850);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseB);
        sleep(1000);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseB);
    }

    private void CaseC() {
        // Build the trajectories
        trajPreloadSplineCaseC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(280))
                /*.splineToLinearHeading(new Pose2d(-53.7, 30, Math.toRadians(176)), Math.toRadians(293),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))*/
                .setTangent(Math.toRadians(210))
                .splineToLinearHeading(new Pose2d(-55, 30, Math.toRadians(176)), Math.toRadians(290))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();

        trajToIntakeSplineCaseC = drive.trajectorySequenceBuilder(trajPreloadSplineCaseC.end())
                //.addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.017))
                /*.splineToLinearHeading(new Pose2d(-57.5,12, Math.toRadians(177)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))*/
                .splineToLinearHeading(new Pose2d(-57.5,12, Math.toRadians(177)),Math.toRadians(180))
                .build();

        trajToScoreSplineCaseC = drive.trajectorySequenceBuilder(trajToIntakeSplineCaseC.end())
                /*.lineTo(new Vector2d(-48, 13),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
                })
                .lineTo(new Vector2d(-44,12),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)*/
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(24,10,Math.toRadians(177)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.03);
                })
                .splineToLinearHeading(new Pose2d(51,23.2, Math.toRadians(177)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker( () -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        trajToIntakeCycleSplineCaseC = drive.trajectorySequenceBuilder(trajToScoreSplineCaseC.end())
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,8, Math.toRadians(177)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.022))
                .lineToLinearHeading(new Pose2d(-58,7.5, Math.toRadians(177)),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCycleSplineCaseC = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseC.end())
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(24,5, Math.toRadians(177)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = true;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
                })
                .splineToLinearHeading(new Pose2d(52,30, Math.toRadians(177)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> retractThreadExecutor.accept(Constants.SLIDE_INTAKE))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadSplineCaseC);
        sleep(300);

        drive.followTrajectorySequence(trajToIntakeSplineCaseC);
        sleep(1500);

        drive.followTrajectorySequence(trajToScoreSplineCaseC);
        sleep(750);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseC);
        sleep(1500);

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
            intakeSubsystem.setIntakePos(0.2);
            intakeSubsystem.setIntakePower(-1);
            sleep(2500);

            intakeSubsystem.setIntakePos(0.030);
            intakeSubsystem.setIntakePower(0);
        }).start();
        }
    }



