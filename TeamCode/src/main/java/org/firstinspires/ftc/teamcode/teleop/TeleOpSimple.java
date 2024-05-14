package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
public class TeleOpSimple extends LinearOpMode {
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private Motor slideMotorLeft;
    private Motor slideMotorRight;
    private Motor climbMotor;
    private Motor intakeMotor;

    private Servo rotateServo;
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo blockServo;
    private Servo droneServo;
    private Servo intakeServo;
    private Servo preloadServo;

    private Encoder leftFwEncoder;
    private Encoder rightFwEncoder;
    private Encoder strafeFwEncoder;

    double lfPower;
    double rfPower;
    double lbPower;
    double rbPower;

    YawPitchRollAngles orientation;

    IMU imu;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.CLIMB_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        lf = hardwareMap.get(DcMotor.class, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        rf = hardwareMap.get(DcMotor.class, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        lb = hardwareMap.get(DcMotor.class, HardwareConstants.ID_LEFT_BACK_MOTOR);
        rb = hardwareMap.get(DcMotor.class, HardwareConstants.ID_RIGHT_BACK_MOTOR);
        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFwEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_RIGHT_ENCODER));
        leftFwEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LEFT_ENCODER));
        strafeFwEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_STRAFE_ENCODER));

        climbMotor = new Motor(hardwareMap, HardwareConstants.ID_CLIMB_MOTOR);
        intakeMotor = new Motor(hardwareMap, HardwareConstants.ID_INTAKE_MOTOR);

        leftFwEncoder.setDirection(Encoder.Direction.REVERSE);
        strafeFwEncoder.setDirection(Encoder.Direction.REVERSE);

        intakeServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_SERVO);
        rotateServo = hardwareMap.get(Servo.class, HardwareConstants.ID_FLIP_SERVO);
        blockServo = hardwareMap.get(Servo.class, HardwareConstants.ID_BLOCK_SERVO);
        armServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_LEFT);
        armServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_RIGHT);
        droneServo = hardwareMap.get(Servo.class, HardwareConstants.ID_DRONE_SERVO);
        preloadServo = hardwareMap.get(Servo.class, HardwareConstants.ID_PRELOAD_SERVO);

        rf.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        lf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.FORWARD);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armServoLeft.setDirection(Servo.Direction.REVERSE);
        preloadServo.setDirection(Servo.Direction.REVERSE);
        intakeServo.setDirection(Servo.Direction.REVERSE);

        intakeServo.setPosition(0);
        armServoLeft.setPosition(0);
        armServoRight.setPosition(0);
        rotateServo.setPosition(0.22);
        droneServo.setPosition(0);
        preloadServo.setPosition(0);
        blockServo.setPosition(0);

        slideMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        climbMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slideMotorLeft.setInverted(true);
        climbMotor.setInverted(true);

        slideMotorLeft.resetEncoder();
        slideMotorRight.resetEncoder();

        climbMotor.resetEncoder();
        waitForStart();

        while(opModeIsActive()) {
            double drive = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double turn = gamepad2.right_stick_x;

            // Send calculated power to wheels
            lfPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            rfPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            lbPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            rbPower = Range.clip(drive - turn + strafe, -1.0, 1.0);


            lf.setPower(lfPower);
            rf.setPower(rfPower);
            lb.setPower(lbPower);
            rb.setPower(rbPower);

            if (gamepad1.b) {
                rotateServo.setPosition(rotateServo.getPosition() + 0.01);
                sleep(200);
            }
            else if (gamepad1.x) {
                rotateServo.setPosition(rotateServo.getPosition() - 0.01);
                sleep(200);
            }

            if (gamepad2.b) {
                intakeServo.setPosition(0.315);
                sleep(200);

            }
            else if (gamepad2.x) {
                intakeServo.setPosition(0);
                sleep(200);
            }

            if (gamepad1.dpad_up) {
                armServoLeft.setPosition(armServoLeft.getPosition() + 0.01);
                armServoRight.setPosition(armServoRight.getPosition() + 0.01);
                sleep(200);
            }
            else if (gamepad1.dpad_down){
                armServoLeft.setPosition(armServoLeft.getPosition() - 0.01);
                armServoRight.setPosition(armServoRight.getPosition() - 0.01);
                sleep(200);
            }

            if (gamepad1.a) {
                blockServo.setPosition(blockServo.getPosition() + 0.01);
                sleep(200);
            }
            else if (gamepad1.y) {
                blockServo.setPosition(blockServo.getPosition() - 0.01);
                sleep(200);
            }

            if (gamepad2.a) {
                intakeServo.setPosition(intakeServo.getPosition() + 0.01);
                sleep(200);
            }
            else if (gamepad2.y) {
                intakeServo.setPosition(intakeServo.getPosition() - 0.01);
                sleep(200);
            }

           /* if(gamepad2.left_bumper) {
                intakeServo.setPosition(intakeServo.getPosition() + 0.01);
                sleep(200);
            }
            if(gamepad2.right_bumper) {
                intakeServo.setPosition(intakeServo.getPosition() - 0.01);
                sleep(200);
            }*/

           if (gamepad1.dpad_right) {
               intakeMotor.set(1);
            }
            if (gamepad1.dpad_left) {
                intakeMotor.set(0);
            }

            /*lf.setPower(gamepad1.left_trigger);
            rf.setPower(gamepad1.right_trigger);
            lb.setPower(gamepad2.left_trigger);
            rb.setPower(gamepad2.right_trigger);
*/
            slideMotorLeft.set(gamepad1.right_trigger - gamepad1.left_trigger);
            slideMotorRight.set(gamepad1.right_trigger - gamepad1.left_trigger);

            climbMotor.set(gamepad2.right_trigger - gamepad2.left_trigger);

            telemetry.addData("Slide Motor Left", slideMotorLeft.getCurrentPosition());
            telemetry.addData("Slide Motor Right", slideMotorRight.getCurrentPosition());
          //  telemetry.addData("Slide Motor Power", gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("Intake Servo", intakeServo.getPosition());
            telemetry.addData("Rotate Servo", rotateServo.getPosition());
            telemetry.addData("Arm Servo Left", armServoLeft.getPosition());
            telemetry.addData("Arm Servo Right", armServoRight.getPosition());
            telemetry.addData("Drone Servo", droneServo.getPosition());
            telemetry.addData("Right Encoder", rightFwEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder", leftFwEncoder.getCurrentPosition());
            telemetry.addData("Strafe Encoder", strafeFwEncoder.getCurrentPosition());
            telemetry.addData("Preload Servo", preloadServo.getPosition());
            telemetry.addData("Climb Motor", climbMotor.getCurrentPosition());
            //telemetry.addData("Climb Motor METERS", ticksToMeters(climbMotor.getCurrentPosition()));
            telemetry.addData("BLock Servo", blockServo.getPosition());
            telemetry.update();
        }
    }
/*
    public double ticksToMeters(int ticks) {
        return (double) ticks / Constants.CLIMB_MOTOR_MAX_EXTENSION * Constants.CLIMB_MAX_EXTENSION_METERS;
    }*/
}
