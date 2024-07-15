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
public class RedMid extends LinearOpMode {

    private BarcodeUtil webcam;
    private BarCodeDetection.BarcodePosition barcodePosition;

    private SampleMecanumDrive drive;

    private SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private TrajectorySequence trajPurplePixelA;
    private TrajectorySequence trajYellowPixelA;
    private TrajectorySequence trajIntakeA;
    private TrajectorySequence trajScoreA;
    private TrajectorySequence trajIntakeA2;
    private TrajectorySequence trajScoreA2;


    private TrajectorySequence trajPurplePixelB;
    private TrajectorySequence trajYellowPixelB;
    private TrajectorySequence trajIntakeB;
    private TrajectorySequence trajScoreB;
    private TrajectorySequence trajIntakeB2;
    private TrajectorySequence trajScoreB2;

    private TrajectorySequence trajIntakeB3;

    private TrajectorySequence trajScoreB3;

    private TrajectorySequence trajPurplePixelC;
    private TrajectorySequence trajYellowPixelC;

    private TrajectorySequence trajIntakeC;

    private TrajectorySequence trajScoreC;

    private TrajectorySequence trajIntakeC2;

    private TrajectorySequence trajScoreC2;

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
        scoreSubsystem = new ScoreSubsystem(armServoR, armServoL, rotateServo, pivotServo, clawServoL, clawServoR, droneServo,false);
        slideSubsystem = new SlideSubsystem(FtcDashboard.getInstance().getTelemetry(), slideMotorRight, slideMotorLeft, true);

        scoreReleaseThread = new ScoreReleaseThread(slideSubsystem, scoreSubsystem, intakeSubsystem);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(11.6, -61.5, Math.toRadians(270)));

        trajPurplePixelA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(this::purple)
                .lineToSplineHeading(new Pose2d(13.5, -41, Math.toRadians(320)),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_LEFT))
                .build();

        trajYellowPixelA = drive.trajectorySequenceBuilder(trajPurplePixelA.end())
                .addDisplacementMarker(this::retract)
                .lineToLinearHeading(new Pose2d(45,-59,Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1100, true))
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(93, -24.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajIntakeA = drive.trajectorySequenceBuilder(trajYellowPixelA.end())
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(200))
                .splineToSplineHeading(new Pose2d(35, -56, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-36, -59, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::intake)
                .splineToSplineHeading(new Pose2d(-56, -41.8, Math.toRadians(163)), Math.toRadians(120),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajScoreA = drive.trajectorySequenceBuilder(trajIntakeA.end())
                .addDisplacementMarker(this::catchThread)
                .setTangent(Math.toRadians(340))
                .splineToSplineHeading(new Pose2d(-12, -59.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(45, -57.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1200, true))
                .splineToLinearHeading(new Pose2d(93.4, -36, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajIntakeA2 = drive.trajectorySequenceBuilder(trajScoreA.end())
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(35, -56, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-36, -56.3, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::intake)
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_THIRD_PIXEL_POS + 0.06))
                .splineToSplineHeading(new Pose2d(-55.7, -37.1, Math.toRadians(165)), Math.toRadians(120),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajScoreA2 = drive.trajectorySequenceBuilder(trajIntakeA2.end())
                .addDisplacementMarker(this::catchThread)
                .setTangent(Math.toRadians(340))
                .splineToSplineHeading(new Pose2d(-12, -57.4, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(45, -55, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1200, true ))
                .splineToLinearHeading(new Pose2d(93.8, -36.5, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajParkA = drive.trajectorySequenceBuilder(trajScoreA2.end())
                .addDisplacementMarker(this::scoreRelease)
                .forward(3,
                        SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(80))
                .build();

        trajPurplePixelB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(this::purple)
                .lineToLinearHeading(new Pose2d(11.6, -39, Math.toRadians(270)),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_LEFT))
                .build();

        trajYellowPixelB = drive.trajectorySequenceBuilder(trajPurplePixelB.end())
                .addDisplacementMarker(this::retract)
                .lineToLinearHeading(new Pose2d(46,-59,Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1100, true))
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(93.5, -28.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajIntakeB = drive.trajectorySequenceBuilder(trajYellowPixelB.end())
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(200))
                .splineToSplineHeading(new Pose2d(35, -56, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-36, -61.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::intake)
                .splineToSplineHeading(new Pose2d(-56, -41.5, Math.toRadians(163)), Math.toRadians(120),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajScoreB = drive.trajectorySequenceBuilder(trajIntakeB.end())
                .addDisplacementMarker(this::catchThread)
                .setTangent(Math.toRadians(340))
                .splineToSplineHeading(new Pose2d(-12, -62, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(45, -58, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1200, true))
                .splineToLinearHeading(new Pose2d(93.4, -36, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajIntakeB2 = drive.trajectorySequenceBuilder(trajScoreB.end())
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(35, -58, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-36, -57.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::intake)
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_THIRD_PIXEL_POS + 0.06))
                .splineToSplineHeading(new Pose2d(-55.9, -38, Math.toRadians(165)), Math.toRadians(120),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajScoreB2 = drive.trajectorySequenceBuilder(trajIntakeB2.end())
                .addDisplacementMarker(this::catchThread)
                .setTangent(Math.toRadians(340))
                .splineToSplineHeading(new Pose2d(-12, -57.4, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(45, -55, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1200, true ))
                .splineToLinearHeading(new Pose2d(94, -37, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();


        trajParkB = drive.trajectorySequenceBuilder(trajScoreB2.end())
                .addDisplacementMarker(this::scoreRelease)
                .forward(3,
                        SampleMecanumDrive.getVelocityConstraint(100, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        trajPurplePixelC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(this::purple)
                .lineToSplineHeading(new Pose2d(14.6, -39, Math.toRadians(240)),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_LEFT))
                .build();

        trajYellowPixelC = drive.trajectorySequenceBuilder(trajPurplePixelC.end())
                .addDisplacementMarker(this::retract)
                .lineToLinearHeading(new Pose2d(45,-59,Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1100, true))
                .setTangent(Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(93.5, -37.9, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajIntakeC = drive.trajectorySequenceBuilder(trajYellowPixelC.end())
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(200))
                .splineToSplineHeading(new Pose2d(35, -56, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-36, -61.5, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::intake)
                .splineToSplineHeading(new Pose2d(-56, -41.9, Math.toRadians(163)), Math.toRadians(120),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajScoreC = drive.trajectorySequenceBuilder(trajIntakeC.end())
                .addDisplacementMarker(this::catchThread)
                .setTangent(Math.toRadians(340))
                .splineToSplineHeading(new Pose2d(-12, -62, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(45, -58, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1200, true))
                .splineToLinearHeading(new Pose2d(93.8, -37, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajIntakeC2 = drive.trajectorySequenceBuilder(trajScoreC.end())
                .addDisplacementMarker(this::scoreRelease)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(35, -58, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-36, -57, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(this::intake)
                .addDisplacementMarker(() -> intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_THIRD_PIXEL_POS + 0.06))
                .splineToSplineHeading(new Pose2d(-55, -36, Math.toRadians(165)), Math.toRadians(120),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajScoreC2 = drive.trajectorySequenceBuilder(trajIntakeC2.end())
                .addDisplacementMarker(this::catchThread)
                .setTangent(Math.toRadians(340))
                .splineToSplineHeading(new Pose2d(-12, -57.4, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(45, -55, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(70, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> score(1300, true ))
                .splineToLinearHeading(new Pose2d(94.4, -39, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH))
                .build();

        trajParkC = drive.trajectorySequenceBuilder(trajScoreC2.end())
                .addDisplacementMarker(this::scoreRelease)
                .forward(3)
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
        sleep(300);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_SECOND_PIXEL_POS + 0.02);
        sleep(700);
        drive.followTrajectorySequence(trajScoreA);
        drive.followTrajectorySequence(trajIntakeA2);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_THIRD_PIXEL_POS + 0.06);
        sleep(200);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FOURTH_PIXEL_POS + 0.06);
        sleep(550);
        drive.followTrajectorySequence(trajScoreA2);
        drive.followTrajectorySequence(trajParkA);
    }

    private void CaseB() {
        drive.followTrajectorySequence(trajPurplePixelB);
        drive.followTrajectorySequence(trajYellowPixelB);
        drive.followTrajectorySequence(trajIntakeB);
        sleep(250);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_SECOND_PIXEL_POS + 0.02);
        sleep(650);
        drive.followTrajectorySequence(trajScoreB);
        drive.followTrajectorySequence(trajIntakeB2);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_THIRD_PIXEL_POS + 0.06);
        sleep(200);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FOURTH_PIXEL_POS + 0.06);
        sleep(550);
        drive.followTrajectorySequence(trajScoreB2);
        drive.followTrajectorySequence(trajParkB);
    }

    private void CaseC() {
        drive.followTrajectorySequence(trajPurplePixelC);
        drive.followTrajectorySequence(trajYellowPixelC);
        drive.followTrajectorySequence(trajIntakeC);
        sleep(300);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_SECOND_PIXEL_POS + 0.02);
        sleep(700);
        drive.followTrajectorySequence(trajScoreC);
        drive.followTrajectorySequence(trajIntakeC2);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_THIRD_PIXEL_POS + 0.06);
        sleep(200);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FOURTH_PIXEL_POS + 0.06);
        sleep(500);
        drive.followTrajectorySequence(trajScoreC2);
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
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_SETLINE_3_POSITION);
            if (rotate) {
                scoreSubsystem.rotateClaw(Constants.ANGLE_R);
            } else scoreSubsystem.rotateClaw(Constants.ANGLE_INIT_POSITION);
            sleep(300);
            intakeSubsystem.useTrap(Constants.TRAP_BLOCK);
            sleep(300);
            intakeSubsystem.useTrap(Constants.TRAP_BLOCK);
        }).start();
    }

    public void scoreRelease() {
        new Thread(() -> {
            scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH);
            sleep(600);

            scoreSubsystem.rotateClaw(Constants.ANGLE_INIT_POSITION);
            scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION - 0.045);
            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION);

            slideSubsystem.setLevel(0);
        }).start();
    }

    public void intake() {
        intakeSubsystem.setIntakePower(1);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_FIRST_PIXEL_POS + 0.02);
    }

    public void catchThread() {
        new Thread(() -> {
            intakeSubsystem.setIntakePower(-1);
            scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH);

            scoreSubsystem.useArm(0.286);
            sleep(400);
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



