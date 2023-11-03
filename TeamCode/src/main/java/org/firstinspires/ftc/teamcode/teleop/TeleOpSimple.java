package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.roadrunner.drive.util.Encoder;

@TeleOp
public class TeleOpSimple extends LinearOpMode {
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private Motor slideMotorLeft;
    private Motor slideMotorRight;

    private Servo clawServo;
    private Servo rotateServo;
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo droneServo;
    private Servo intakeServo;
    private Servo pivotServo;
    private Servo leftClimbRelease;
    private Servo rightClimbRelease;
    private Servo preloadServo;

    private Encoder leftFwEncoder;
    private Encoder rightFwEncoder;
    private Encoder strafeEncoder;

    double lfPower;
    double rfPower;
    double lbPower;
    double rbPower;

    @Override
    public void runOpMode() {
        lf = hardwareMap.get(DcMotor.class, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        rf = hardwareMap.get(DcMotor.class, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        lb = hardwareMap.get(DcMotor.class, HardwareConstants.ID_LEFT_BACK_MOTOR);
        rb = hardwareMap.get(DcMotor.class, HardwareConstants.ID_RIGHT_BACK_MOTOR);
        slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LATERAL_ENCODER));
        rightFwEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_RIGHT_ENCODER));
        leftFwEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LEFT_ENCODER));

        leftFwEncoder.setDirection(Encoder.Direction.REVERSE);
        strafeEncoder.setDirection(Encoder.Direction.REVERSE);

        clawServo = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_SERVO);
        intakeServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_SERVO);
        pivotServo = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_CLAW_SERVO);
        rotateServo = hardwareMap.get(Servo.class, HardwareConstants.ID_FLIP_SERVO);
        armServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_LEFT);
        armServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO_RIGHT);
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

        leftClimbRelease.setDirection(Servo.Direction.REVERSE);
        rightClimbRelease.setDirection(Servo.Direction.REVERSE);
        droneServo.setDirection(Servo.Direction.REVERSE);
        intakeServo.setDirection(Servo.Direction.REVERSE);
        armServoLeft.setDirection(Servo.Direction.REVERSE);
        preloadServo.setDirection(Servo.Direction.REVERSE);

        clawServo.setPosition(Constants.OPEN_CLAW);
        intakeServo.setPosition(Constants.INTAKE_SERVO_INIT_POS);
        pivotServo.setPosition(Constants.PIVOT_INIT_POS);
        armServoLeft.setPosition(Constants.ARM_SERVO_INIT_POSITION);
        armServoRight.setPosition(Constants.ARM_SERVO_INIT_POSITION);
        rotateServo.setPosition(Constants.ROTATE_SERVO_INIT_POSITION);
        droneServo.setPosition(Constants.DRONE_SERVO_INIT_POS);
        preloadServo.setPosition(Constants.PRELOAD_SERVO_INIT_POS);

        slideMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        slideMotorLeft.setInverted(true);
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

            //Position for flipServo
            if (gamepad1.b) {
                rotateServo.setPosition(rotateServo.getPosition() + 0.01);
                sleep(200);
            }
            else if (gamepad1.x) {
                rotateServo.setPosition(rotateServo.getPosition() - 0.01);
                sleep(200);
            }

            //Position for pivotServoLeft and pivotServoRight
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

            //Position for clawServo
            if (gamepad1.a) {
                clawServo.setPosition(clawServo.getPosition() + 0.01);
                sleep(200);
            }
            else if (gamepad1.y) {
                clawServo.setPosition(clawServo.getPosition() - 0.01);
                sleep(200);
            }

            if(gamepad1.left_bumper) {
                intakeServo.setPosition(intakeServo.getPosition() + 0.01);
                sleep(200);
            }
            if(gamepad1.right_bumper) {
                intakeServo.setPosition(intakeServo.getPosition() - 0.01);
                sleep(200);
            }

            if(gamepad2.a) {
                pivotServo.setPosition(pivotServo.getPosition() + 0.01);
                sleep(200);
            }
            if(gamepad2.y) {
                pivotServo.setPosition(pivotServo.getPosition() - 0.01);
                sleep(200);
            }

            if (gamepad1.dpad_right) {
                droneServo.setPosition(droneServo.getPosition() + 0.05);
                sleep(200);
            }
            if (gamepad1.dpad_left) {
                droneServo.setPosition(droneServo.getPosition() - 0.05);
            }

            slideMotorLeft.set(gamepad1.right_trigger - gamepad1.left_trigger);
            slideMotorRight.set(gamepad1.right_trigger - gamepad1.left_trigger);

            telemetry.addData("Slide Motor Left", slideMotorLeft.getCurrentPosition());
            telemetry.addData("Slide Motor Left", slideMotorRight.getCurrentPosition());
            telemetry.addData("Slide Motor Power", gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("Claw Servo", clawServo.getPosition());
            telemetry.addData("Pivot Claw Servo", pivotServo.getPosition());
            telemetry.addData("Intake Servo", intakeServo.getPosition());
            telemetry.addData("Flip Servo", rotateServo.getPosition());
            telemetry.addData("Pivot Servo Left", armServoLeft.getPosition());
            telemetry.addData("Pivot Servo Right", armServoRight.getPosition());
            telemetry.addData("Drone Servo", droneServo.getPosition());
            telemetry.addData("Strafe Encoder", strafeEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", rightFwEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder", leftFwEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
