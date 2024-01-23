package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.threads.ScoreThread;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BarCodeDetection;
import org.firstinspires.ftc.teamcode.vision.BarcodeUtil;

@Autonomous
public class StackAutoRed extends LinearOpMode {

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
    private TrajectorySequence trajToIntakePreloadCaseA;
    private TrajectorySequence trajToIntakePreloadCaseB;
    private TrajectorySequence trajToIntakePreloadCaseC;
    private TrajectorySequence trajToIntakeCaseA;
    private TrajectorySequence trajToIntakeCaseB;
    private TrajectorySequence trajToIntakeCaseC;
    private TrajectorySequence trajToIntake2CaseA;
    private TrajectorySequence trajToIntake2CaseB;
    private TrajectorySequence trajToIntake2CaseC;
    private TrajectorySequence trajToScoreCaseA;
    private TrajectorySequence trajToScoreCaseB;
    private TrajectorySequence trajToScoreCaseC;
    private TrajectorySequence trajToScore2CaseA;
    private TrajectorySequence trajToScore2CaseB;
    private TrajectorySequence trajToScore2CaseC;

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

        scoreAutoThread = new Thread(() -> {
            slideSubsystem.setLevel(Constants.SLIDE_POSITIONS[3]);
            sleep(150);

            scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);
            scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_45);
        });

        retractThread = new Thread(() -> {
            scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_INIT_POSITION);
            sleep(100);
            scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION);
            sleep(300);

            slideSubsystem.setLevel(Constants.SLIDE_INTAKE);
        });

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
            CaseA();
        }   else {
            CaseB();
        }
        sleep(3000);
    }

    private void CaseA() {
        // Build the trajectories
        trajPreloadCaseA = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-42,-32,Math.toRadians(0)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntakeCaseA = drive.trajectorySequenceBuilder(trajPreloadCaseA.end())
                .lineTo(new Vector2d(-36,-32),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55,-11, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCaseA = drive.trajectorySequenceBuilder(trajToIntakeCaseA.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49,-29, Math.toRadians(180)),Math.toRadians(320),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntake2CaseA = drive.trajectorySequenceBuilder(trajToScoreCaseA.end())
                .setTangent(Math.toRadians(140))
                .splineToLinearHeading(new Pose2d(-55,-11,Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScore2CaseA = drive.trajectorySequenceBuilder(trajToIntake2CaseA.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49,-29, Math.toRadians(180)), Math.toRadians(320),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        parkSpotA = drive.trajectorySequenceBuilder(trajToScore2CaseA.end())
                .lineTo(new Vector2d(44,-29),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(60,-10, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadCaseA);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS);
        sleep(300);

        drive.followTrajectorySequence(trajToIntakePreloadCaseA);
        //intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseA);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseA);
        intakeRoutine(Constants.INTAKE_SERVO_MID_POS);

        drive.followTrajectorySequence(trajToScoreCaseA);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(parkSpotA);
    }

    private void CaseB() {
        // Build the trajectories
        trajPreloadCaseB = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-45, -24.5, Math.toRadians(180)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntakeCaseB = drive.trajectorySequenceBuilder(trajPreloadCaseB.end())
                .lineTo(new Vector2d(-53,-11),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCaseB = drive.trajectorySequenceBuilder(trajToIntakeCaseB.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49,-35, Math.toRadians(180)),Math.toRadians(280),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntake2CaseB = drive.trajectorySequenceBuilder(trajToScoreCaseB.end())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-53,-11, Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScore2CaseB = drive.trajectorySequenceBuilder(trajToIntake2CaseB.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49,-29, Math.toRadians(180)), Math.toRadians(300),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        parkSpotB = drive.trajectorySequenceBuilder(trajToScore2CaseB.end())
                .lineTo(new Vector2d(44,-29),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(100))
                .splineToLinearHeading(new Pose2d(60,-10, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadCaseB);

        /*preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS);*/
        sleep(500);

        drive.followTrajectorySequence(trajToIntakeCaseB);
        sleep(200);
        //intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseB);
        sleep(200);
        //scoreAutoThread.start();
        //retractThread.start();

        drive.followTrajectorySequence(trajToIntake2CaseB);
        sleep(200);
       // intakeRoutine(Constants.INTAKE_SERVO_MID_POS);

        drive.followTrajectorySequence(trajToScore2CaseB);
        sleep(200);
        //scoreAutoThread.start();
        //retractThread.start();

        drive.followTrajectorySequence(parkSpotB);
    }

    private void CaseC() {
        // Build the trajectories
        trajPreloadCaseC = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(95))
                .splineToLinearHeading(new Pose2d(-30,-35, Math.toRadians(180)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntakeCaseC = drive.trajectorySequenceBuilder(trajPreloadCaseC.end())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-55,-11, Math.toRadians(180)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScoreCaseC = drive.trajectorySequenceBuilder(trajToIntakeCaseC.end())
                .setTangent(Math.toRadians(10))
                .splineToLinearHeading(new Pose2d(49,-43.5, Math.toRadians(180)),Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToIntake2CaseC = drive.trajectorySequenceBuilder(trajToScoreCaseC.end())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-55,-11,Math.toRadians(180)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        trajToScore2CaseC = drive.trajectorySequenceBuilder(trajToIntake2CaseC.end())
                .setTangent(Math.toRadians(10))
                .splineToLinearHeading(new Pose2d(49,-29, Math.toRadians(180)), Math.toRadians(316),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        parkSpotC = drive.trajectorySequenceBuilder(trajToScore2CaseC.end())
                .lineTo(new Vector2d(44, -29),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(60,-10, Math.toRadians(180)),Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        // Follow the trajectories
        drive.followTrajectorySequence(trajPreloadCaseC);

        preloadServo.setPosition(Constants.PRELOAD_SERVO_SCORE_POS);
        sleep(600);

        drive.followTrajectorySequence(trajToIntakePreloadCaseC);
        intakeRoutine(Constants.INTAKE_SERVO_FIRST_PIXEL_POS_AUTO);

        drive.followTrajectorySequence(trajToScoreCaseC);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(trajToIntakeCaseA);
        intakeRoutine(Constants.INTAKE_SERVO_MID_POS);

        drive.followTrajectorySequence(trajToScore2CaseC);
        scoreAutoThread.start();
        retractThread.start();

        drive.followTrajectorySequence(parkSpotC);
    }

    public void intakeRoutine(double intakeLevel) {
        intakeServo.setPosition(intakeLevel);
        intakeSubsystem.setIntakePower(0.8);
        sleep(1000);

        intakeSubsystem.setIntakePower(0);
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
    }
}

