package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.threads.CatchThread;
import org.firstinspires.ftc.teamcode.threads.ScoreReleaseThread;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

@Autonomous
public class BlueFar extends LinearOpMode {

    private BarcodeUtil webcam;
    private BarCodeDetection.BarcodePosition barcodePosition;

    private SampleMecanumDrive drive;

    private SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private TrajectorySequence trajPurplePixelA;
    private TrajectorySequence trajYellowPixelA;

    private TrajectorySequence trajPurplePixelB;
    private TrajectorySequence trajYellowPixelB;

    private TrajectorySequence trajPurplePixelC;
    private TrajectorySequence trajYellowPixelC;

    private TrajectorySequence trajIntakeA;
    private TrajectorySequence trajScoreA;
    private TrajectorySequence trajParkA;

    private TrajectorySequence trajIntakeB;
    private TrajectorySequence trajScoreB;
    private TrajectorySequence trajParkB;

    private TrajectorySequence trajIntakeC;
    private TrajectorySequence trajScoreC;
    private TrajectorySequence trajParkC;

    private CatchThread catchThread;
    private ScoreReleaseThread scoreReleaseThread;
    private Consumer<Integer> extendoThreadExecutor;
    private Consumer<Integer> slideThreadExtecutor;

    private DriveSubsystem driveSubsystem;

    public Motor slideMotorLeft;
    public Motor slideMotorRight;

    private Servo armServoL;
    private Servo armServoR;
    private Servo pivotServo;
    private Servo rotateServo;
    private Servo clawServoL;
    private Servo clawServoR;
    private Servo droneServo;

    private Motor intakeMotor;
    private Servo intakeServo;
    private Servo trap;

    public Motor extendoMotor;

    private Motor leftFront;
    private Motor leftBack;
    private Motor rightFront;
    private Motor rightBack;


    @Override
    public void runOpMode() {
        webcam = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, BarCodeDetection.Color.BLUE);
        webcam.init();

        leftFront = new Motor(hardwareMap, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        leftBack = new Motor(hardwareMap, HardwareConstants.ID_LEFT_BACK_MOTOR);
        rightFront = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        rightBack = new Motor(hardwareMap, HardwareConstants.ID_RIGHT_BACK_MOTOR);

        armServoR = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_RIGHT);
        armServoL = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_LEFT);
        rotateServo = hardwareMap.get(Servo.class, HardwareConstants.ID_ROTATE_SERVO);
        pivotServo = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO);
        clawServoL = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_SERVO_L);
        clawServoR = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_SERVO_R);
        droneServo = hardwareMap.get(Servo.class, HardwareConstants.ID_DRONE_SERVO);

        intakeServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_SERVO);
        trap = hardwareMap.get(Servo.class, HardwareConstants.ID_TRAP_SERVO);
        intakeMotor = new Motor(hardwareMap, HardwareConstants.ID_INTAKE_MOTOR);

        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        driveSubsystem = new DriveSubsystem(leftFront, leftBack, rightFront, rightBack);
        intakeSubsystem = new IntakeSubsystem(intakeServo, trap, intakeMotor);
        scoreSubsystem = new ScoreSubsystem(armServoR, armServoL, rotateServo, pivotServo, clawServoL, clawServoR, droneServo, false);
        slideSubsystem = new SlideSubsystem(FtcDashboard.getInstance().getTelemetry(), slideMotorRight, slideMotorLeft, true);

        scoreReleaseThread = new ScoreReleaseThread(slideSubsystem, scoreSubsystem, intakeSubsystem);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, 61.5, Math.toRadians(90)));

        trajPurplePixelA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(this::purple)
                .lineToLinearHeading(new Pose2d(-37, 45, Math.toRadians(135)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_LEFT))
                .build();

        trajYellowPixelA = drive.trajectorySequenceBuilder(trajPurplePixelA.end())
                .addDisplacementMarker(this::retract)
                .lineToSplineHeading(new Pose2d(-42, 14, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(78, 19, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1200, true))
                .lineToLinearHeading(new Pose2d(90, 48.7, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajIntakeA = drive.trajectorySequenceBuilder(trajYellowPixelA.end())
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(205))
                .splineToSplineHeading(new Pose2d(70, 17, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::intake)
                .lineToLinearHeading(new Pose2d(-59.8, 16.8, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajScoreA = drive.trajectorySequenceBuilder(trajIntakeA.end())
                .addDisplacementMarker(this::catchThread)
                .lineToSplineHeading(new Pose2d(60, 17, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score3(2800, false))
                .lineToLinearHeading(new Pose2d(91.5, 11.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajParkA = drive.trajectorySequenceBuilder(trajScoreA.end())
                .addDisplacementMarker(this::scoreRelease)
                .forward(2)
                .strafeRight(14)
                .build();


        trajPurplePixelB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(this::purple)
                .lineToSplineHeading(new Pose2d(-48,25, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_LEFT))
                .lineToSplineHeading(new Pose2d(-48,13, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajYellowPixelB = drive.trajectorySequenceBuilder(trajPurplePixelB.end())
                .addDisplacementMarker(this::retract)
                .lineToSplineHeading(new Pose2d(78, 21, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1100, true))
                .lineToLinearHeading(new Pose2d(90.2, 46.4, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajIntakeB = drive.trajectorySequenceBuilder(trajYellowPixelB.end())
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(235))
                .splineToLinearHeading(new Pose2d(65, 16, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::intake)
                .lineToLinearHeading(new Pose2d(-59.2, 12, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajScoreB = drive.trajectorySequenceBuilder(trajIntakeB.end())
                .addDisplacementMarker(this::catchThread)
                .lineToSplineHeading(new Pose2d(60, 16, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score3(2800, false))
                .lineToLinearHeading(new Pose2d(93, 15, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajParkB = drive.trajectorySequenceBuilder(trajScoreB.end())
                .addDisplacementMarker(this::scoreRelease)
                .forward(2)
                .strafeRight(14)
                .build();


        trajPurplePixelC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(this::purple)
                .lineToLinearHeading(new Pose2d(-40,45,Math.toRadians(60)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_LEFT))
                .lineToLinearHeading(new Pose2d(-36,49,Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajYellowPixelC = drive.trajectorySequenceBuilder(trajPurplePixelC.end())
                .addDisplacementMarker(this::retract)
                .lineToSplineHeading(new Pose2d(-37, 12, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .turn(Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(68, 12, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1200, true))
                .lineToLinearHeading(new Pose2d(90, 26.6, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajIntakeC = drive.trajectorySequenceBuilder(trajYellowPixelC.end())
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(205))
                .splineToSplineHeading(new Pose2d(70, 13, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::intake)
                .lineToLinearHeading(new Pose2d(-59.8, 14.8, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajScoreC = drive.trajectorySequenceBuilder(trajIntakeC.end())
                .addDisplacementMarker(this::catchThread)
                .lineToSplineHeading(new Pose2d(60, 8, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score3(2800, false))
                .lineToLinearHeading(new Pose2d(92, 2, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajParkC = drive.trajectorySequenceBuilder(trajScoreC.end())
                .addDisplacementMarker(this::scoreRelease)
                .forward(2)
                .strafeRight(14)
                .build();

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

        // sleep(3000);
    }

    private void CaseA() {
        drive.followTrajectorySequence(trajPurplePixelA);
        drive.followTrajectorySequence(trajYellowPixelA);
        drive.followTrajectorySequence(trajIntakeA);
        sleep(350);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_SECOND_PIXEL_POS);
        sleep(1150);
        drive.followTrajectorySequence(trajScoreA);
        drive.followTrajectorySequence(trajParkA);
    }

    private void CaseB() {
        drive.followTrajectorySequence(trajPurplePixelB);
        drive.followTrajectorySequence(trajYellowPixelB);
        drive.followTrajectorySequence(trajIntakeB);
        sleep(350);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_SECOND_PIXEL_POS);
        sleep(1500);
        drive.followTrajectorySequence(trajScoreB);
        drive.followTrajectorySequence(trajParkB);
    }

    private void CaseC() {
        drive.followTrajectorySequence(trajPurplePixelC);
        drive.followTrajectorySequence(trajYellowPixelC);
        drive.followTrajectorySequence(trajIntakeC);
        sleep(350);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_SECOND_PIXEL_POS);
        sleep(1500);
        drive.followTrajectorySequence(trajScoreC);
        drive.followTrajectorySequence(trajParkC);
    }

    public void purple() {
        new Thread(() -> {
            scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);
            sleep(200);
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_PURPLE_POSITION);
        }).start();
    }

    public void retract() {
        new Thread(() -> {
            scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION - 0.015);
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION);
            intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_THIRD_PIXEL_POS);
        }).start();
    }

    public void score(int level, boolean rotate) {
        new Thread(() -> {
            slideSubsystem.setLevel(level);
            sleep(200);
            scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_PIVOT_POSITION);
            if (rotate) {
                scoreSubsystem.rotateClaw(Constants.ANGLE_LINE_R);
            } else scoreSubsystem.rotateClaw(Constants.ANGLE_INIT_POSITION);
            sleep(300);
            intakeSubsystem.useTrap(Constants.TRAP_BLOCK);
        }).start();
    }

    public void score3(int level, boolean rotate) {
        new Thread(() -> {
            slideSubsystem.setLevel(level);
            sleep(200);
            scoreSubsystem.useArm(Constants.ARM_SERVO_SETLINE_3_POSITION);
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_SETLINE_3_POSITION + 0.02);
            if (rotate) {
                scoreSubsystem.rotateClaw(Constants.ANGLE_LINE_R);
            } else scoreSubsystem.rotateClaw(Constants.ANGLE_INIT_POSITION);
            sleep(300);
            intakeSubsystem.useTrap(Constants.TRAP_BLOCK);
        }).start();
    }

    public void scoreRelease() {
        new Thread(() -> {
            scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH);
            sleep(1000);

            scoreSubsystem.rotateClaw(Constants.ANGLE_INIT_POSITION);
            scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION - 0.04);
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION);

            slideSubsystem.setLevel(0);
        }).start();
    }

    public void intake() {
        intakeSubsystem.setIntakePower(1);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.02);
        sleep(200);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.04);
    }

    public void catchThread() {
        new Thread(() -> {
            intakeSubsystem.setIntakePower(-1);
            scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH);

            scoreSubsystem.useArm(0.286);
            sleep(450);
            intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS + 0.28);
            intakeSubsystem.setIntakePower(0);
            intakeSubsystem.useTrap(Constants.TRAP_OPEN - 0.03);
            sleep(390);

            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION - 0.08);
            scoreSubsystem.useArm(Constants.ARM_SERVO_INTAKE_POS + 0.08);
            sleep(350);

            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION);
            sleep(50);

            scoreSubsystem.useArm(Constants.ARM_SERVO_INTAKE_POS);
            sleep(400);

            scoreSubsystem.useClaw(Constants.CLAW_SERVO_CLOSE_POS, Constants.CLAW_BOTH);
            sleep(300);
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION - 0.1);
            intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
            scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION - 0.055);
            intakeSubsystem.setIntakePower(-1);
            intakeSubsystem.intake = true;

            sleep(500);
            intakeSubsystem.setIntakePower(0);
        }).start();
    }
}


