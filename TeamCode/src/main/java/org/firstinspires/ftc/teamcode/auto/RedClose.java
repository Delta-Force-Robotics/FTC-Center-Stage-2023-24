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
public class RedClose extends LinearOpMode {

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

    private TrajectorySequence trajParkA;

    private TrajectorySequence trajParkB;

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
        webcam = new BarcodeUtil(hardwareMap, "Webcam 1", telemetry, BarCodeDetection.Color.RED);
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
        drive.setPoseEstimate(new Pose2d(11.6, -61.5, Math.toRadians(270)));

        trajPurplePixelA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(this::purple)
                .lineToLinearHeading(new Pose2d(11, -38, Math.toRadians(315)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_LEFT))
                .build();

        trajYellowPixelA = drive.trajectorySequenceBuilder(trajPurplePixelA.end())
                .lineToLinearHeading(new Pose2d(11.6, -45, Math.toRadians(315)))
                .addDisplacementMarker(() -> score(1100, true))
                .lineToSplineHeading(new Pose2d(46, -23.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajParkA = drive.trajectorySequenceBuilder(trajYellowPixelA.end())
                .setTangent(Math.toRadians(290))
                .forward(5)
                .addDisplacementMarker(this::scoreRelease)
                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        trajPurplePixelB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(this::purple)
                .lineToSplineHeading(new Pose2d(11.6, -39, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_LEFT))
                .build();

        trajYellowPixelB = drive.trajectorySequenceBuilder(trajPurplePixelB.end())
                .lineToSplineHeading(new Pose2d(11.6, -45, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1000, true))
                .lineToSplineHeading(new Pose2d(45.2, -32, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajParkB = drive.trajectorySequenceBuilder(trajYellowPixelB.end())
                .forward(5)
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        trajPurplePixelC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(this::purple)
                .lineToLinearHeading(new Pose2d(11,-40,Math.toRadians(210)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_LEFT))
                .build();

        trajYellowPixelC = drive.trajectorySequenceBuilder(trajPurplePixelC.end())
                .addDisplacementMarker(this::retract)
                .lineToSplineHeading(new Pose2d(11,-55, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1100, true))
                .setTangent(Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(45.4, -41, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajParkC = drive.trajectorySequenceBuilder(trajYellowPixelC.end())
                .forward(5)
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(290))
                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
        drive.followTrajectorySequence(trajParkA);
    }

    private void CaseB() {
        drive.followTrajectorySequence(trajPurplePixelB);
        drive.followTrajectorySequence(trajYellowPixelB);
        drive.followTrajectorySequence(trajParkB);
    }

    private void CaseC() {
        drive.followTrajectorySequence(trajPurplePixelC);
        drive.followTrajectorySequence(trajYellowPixelC);
        drive.followTrajectorySequence(trajParkC);
    }

    public void purple() {
        scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);
        sleep(200);
        scoreSubsystem.usePivot(Constants.PIVOT_SERVO_PURPLE_POSITION);
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

    public void scoreRelease() {
        new Thread(() -> {
            scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH);
            sleep(400);

            scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION);
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION);
            scoreSubsystem.rotateClaw(Constants.ANGLE_INIT_POSITION);

            slideSubsystem.setLevel(0);
        }).start();
    }
}
