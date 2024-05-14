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
public class BoardAutoBlue2Plus2 extends LinearOpMode {

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
        drive.setPoseEstimate(new Pose2d(11.6, 61.5, Math.toRadians(90)));

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

        TrajectorySequence trajPreloadCaseA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(23, 39, Math.toRadians(90)), Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();


       TrajectorySequence trajPreloadScoreCaseA = drive.trajectorySequenceBuilder(trajPreloadCaseA.end())
               .addDisplacementMarker(() -> {
                   scoreThread.selectRotate = false;
                   scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.024);
               })
               .setTangent(Math.toRadians(90))
               .splineToLinearHeading(new Pose2d(52.8, 47, Math.toRadians(186)), Math.toRadians(0),
                       SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                       SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
               .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        trajToIntakeCycleSplineCaseA = drive.trajectorySequenceBuilder(trajPreloadScoreCaseA.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[0] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,11, Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS - 0.012))
                .splineToLinearHeading(new Pose2d(-55,12, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.08))
                .splineToLinearHeading(new Pose2d(-55, 15.5, Math.toRadians(165)), Math.toRadians(180))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .splineToLinearHeading(new Pose2d(-55,12, Math.toRadians(180)), Math.toRadians(180))
                .build();

        trajToScoreCycleSplineCaseA = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseA.end())
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .splineToLinearHeading(new Pose2d(-52,10, Math.toRadians(178)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(26.5,9, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
                })
                .splineToLinearHeading(new Pose2d(53.7,36, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(trajToScoreCycleSplineCaseA.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(56,16,Math.toRadians(186)),Math.toRadians(0))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadCaseA);

        drive.followTrajectorySequence(trajPreloadScoreCaseA);
        sleep(100);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseA);
        sleep(200);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseA);
        sleep(100);

        drive.followTrajectorySequence(park);
    }

    private void CaseB() {
        // Build the trajectories
      TrajectorySequence  trajPreloadCaseB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(new Vector2d(11.6, 33),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();



      TrajectorySequence  trajPreloadScoreCaseB = drive.trajectorySequenceBuilder(trajPreloadCaseB.end())
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.03);
                })
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(52.4, 38.3, Math.toRadians(186)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
              .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();


        trajToIntakeCycleSplineCaseB = drive.trajectorySequenceBuilder(trajPreloadScoreCaseB.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[0] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,11, Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS - 0.012))
                .splineToLinearHeading(new Pose2d(-55.5,12, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.08))
                .splineToLinearHeading(new Pose2d(-55, 15.5, Math.toRadians(165)), Math.toRadians(180))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .splineToLinearHeading(new Pose2d(-55.4,12, Math.toRadians(180)), Math.toRadians(180))
                .build();

        trajToScoreCycleSplineCaseB = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseB.end())
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .splineToLinearHeading(new Pose2d(-52,10, Math.toRadians(178)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(26.5,9, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
                })
                .splineToLinearHeading(new Pose2d(53.7,36, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(trajToScoreCycleSplineCaseB.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(56,16,Math.toRadians(186)),Math.toRadians(0))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadCaseB);

        drive.followTrajectorySequence(trajPreloadScoreCaseB);
        sleep(100);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseB);
        sleep(200);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseB);
        sleep(100);

        drive.followTrajectorySequence(park);
    }

    private void CaseC() {
        // Build the trajectories
        TrajectorySequence  trajPreloadCaseC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(7, 34.5, Math.toRadians(0)), Math.toRadians(200),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS))
                .build();



        TrajectorySequence  trajPreloadScoreCaseC = drive.trajectorySequenceBuilder(trajPreloadCaseC.end())
                .lineToLinearHeading(new Pose2d(11.5, 34.5, Math.toRadians(0)))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(42, 35.5, Math.toRadians(180)), Math.toRadians(300),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[0] - 0.017);
                })
                .lineToLinearHeading(new Pose2d(53.4, 33.2, Math.toRadians(186)))
                .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();


        trajToIntakeCycleSplineCaseC = drive.trajectorySequenceBuilder(trajPreloadScoreCaseC.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[0] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(24,11, Math.toRadians(180)),Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS - 0.012))
                .splineToLinearHeading(new Pose2d(-55.7,11, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.08))
                .splineToLinearHeading(new Pose2d(-55, 14.5, Math.toRadians(165)), Math.toRadians(180))
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .splineToLinearHeading(new Pose2d(-55,12, Math.toRadians(182)), Math.toRadians(180))
                .build();

        trajToScoreCycleSplineCaseC = drive.trajectorySequenceBuilder(trajToIntakeCycleSplineCaseC.end())
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS))
                .splineToLinearHeading(new Pose2d(-52,10, Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::outtakeRoutine)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(26.5,9, Math.toRadians(182)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    scoreThread.selectRotate = false;
                    scoreThreadExecutor.accept(Constants.SLIDE_POSITIONS[1]);
                })
                .splineToLinearHeading(new Pose2d(53.7,37, Math.toRadians(182)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS))
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(trajToScoreCycleSplineCaseC.end())
                .addDisplacementMarker(() -> {
                    slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[1] + 0.06);
                    retractThreadExecutor.accept(Constants.SLIDE_INTAKE);
                })
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(56,16,Math.toRadians(186)),Math.toRadians(0))
                .build();


        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadCaseC);

        drive.followTrajectorySequence(trajPreloadScoreCaseC);
        sleep(100);

        drive.followTrajectorySequence(trajToIntakeCycleSplineCaseC);
        sleep(200);

        drive.followTrajectorySequence(trajToScoreCycleSplineCaseC);
        sleep(100);

        drive.followTrajectorySequence(park);
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
            sleep(2500);

            intakeSubsystem.setIntakePos(0.030);
            intakeSubsystem.setIntakePower(0);
        }).start();
    }
}



