package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
@TeleOp
public class TeleOpSimple extends LinearOpMode {
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;
    private Motor slideMotorLeft;
    private Motor slideMotorRight;
    private Motor intakeMotor;
    private Motor extendoMotor;

    private Servo angleServo;
    private Servo armServoLeft;
    private Servo armServoRight;
    private Servo pivotServo;
    private Servo droneServo;
    private Servo intakeServo;
    private Servo clawL;
    private Servo clawR;
    private Servo trap;

    public static double angle = 0;
    public static double armLeft = 0;
    public static double armRight = 0;
    public static double pivot = 0;
    public static double drone = 0;
    public static double trapPos = 0;
    public static double intakeServoPos = 0;
    public static double intakeMotorPower = 0;
    public static double clawLeft = 0;
    public static double clawRight = 0;

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

        /*lf = hardwareMap.get(DcMotor.class, HardwareConstants.ID_LEFT_FRONT_MOTOR);
        rf = hardwareMap.get(DcMotor.class, HardwareConstants.ID_RIGHT_FRONT_MOTOR);
        lb = hardwareMap.get(DcMotor.class, HardwareConstants.ID_LEFT_BACK_MOTOR);
        rb = hardwareMap.get(DcMotor.class, HardwareConstants.ID_RIGHT_BACK_MOTOR);*/
        //slideMotorLeft = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_LEFT);
        //slideMotorRight = new Motor(hardwareMap, HardwareConstants.ID_SLIDE_MOTOR_RIGHT);
       // intakeMotor = new Motor(hardwareMap, HardwareConstants.ID_INTAKE_MOTOR);

     /*   rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

       /* rightFwEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_RIGHT_ENCODER));
        leftFwEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_LEFT_ENCODER));
        strafeFwEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, HardwareConstants.ID_STRAFE_ENCODER));*/


        //leftFwEncoder.setDirection(Encoder.Direction.REVERSE);
        //strafeFwEncoder.setDirection(Encoder.Direction.REVERSE);

        //intakeServo = hardwareMap.get(Servo.class, HardwareConstants.ID_INTAKE_SERVO);
       //angleServo = hardwareMap.get(Servo.class, HardwareConstants.ID_ROTATE_SERVO);
        //armServoLeft = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_LEFT);
      // armServoRight = hardwareMap.get(Servo.class, HardwareConstants.ID_ARM_SERVO_RIGHT);
        //pivotServo = hardwareMap.get(Servo.class, HardwareConstants.ID_PIVOT_SERVO);
        droneServo = hardwareMap.get(Servo.class, HardwareConstants.ID_DRONE_SERVO);
        //clawL = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_SERVO_L);
        //clawR = hardwareMap.get(Servo.class, HardwareConstants.ID_CLAW_SERVO_R);
        //trap = hardwareMap.get(Servo.class, HardwareConstants.ID_TRAP_SERVO);

        /*lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        //intakeMotor.setRunMode(Motor.RunMode.RawPower);

       // intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

       //armServoLeft.setDirection(Servo.Direction.REVERSE);
     //  pivotServo.setDirection(Servo.Direction.REVERSE);
        //clawL.setDirection(Servo.Direction.REVERSE);
        droneServo.setDirection(Servo.Direction.REVERSE);
//        angleServo.setDirection(Servo.Direction.REVERSE);

        //intakeServo.setPosition(0);
       // armServoLeft.setPosition(0);
       // armServoRight.setPosition(0);
//        angleServo.setPosition(0.278);
        //pivotServo.setPosition(0);
        droneServo.setPosition(0);
       /*clawR.setPosition(0);
      clawL.setPosition(0);*/
        //trap.setPosition(0);

      /* //slideMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
       slideMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
       slideMotorLeft.setInverted(true);*/

      /* slideMotorLeft.resetEncoder();
       slideMotorRight.resetEncoder();*/
        waitForStart();

        while(opModeIsActive()) {
               /* double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;*/

                // Send calculated power to wheels
//                lfPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
//                rfPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
//                lbPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
//                rbPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

//                lf.setPower(lfPower);
//                rf.setPower(rfPower);
//                lb.setPower(lbPower);
//                rb.setPower(rbPower);

               /*// if (gamepad2.left_bumper) {
                    armServoLeft.setPosition(Constants.ARM_SERVO_PIVOT_POSITION);
                    armServoRight.setPosition(Constants.ARM_SERVO_PIVOT_POSITION);

                    pivotServo.setPosition(Constants.PIVOT_SERVO_PIVOT_POSITION);
                }
*/
            /*if (gamepad2.a) {
                //armServoLeft.setPosition(Constants.ARM_SERVO_INIT_POSITION + 0.08);
              //  armServoRight.setPosition(Constants.ARM_SERVO_PIVOT_POSITION + 0.08);//*

                trap.setPosition(Constants.TRAP_BLOCK);
                intakeServo.setPosition(Constants.INTAKE_SERVO_INTAKE_POS);
                intakeMotor.set(1);
            }

            if (gamepad2.b) {
                trap.setPosition(Constants.TRAP_OPEN);
                intakeMotor.set(0);
            }

            if(gamepad2.y) {
                intakeMotor.set(-1);
            }*/

               /* if (gamepad2.right_bumper) {
                    armServoLeft.setPosition(Constants.ARM_SERVO_INIT_POSITION);
                    armServoRight.setPosition(Constants.ARM_SERVO_INIT_POSITION);

                    pivotServo.setPosition(Constants.PIVOT_SERVO_INIT_POSITION);
                }*/

               //angleServo.setPosition(angle);
               // armServoLeft.setPosition(armLeft);
                //armServoRight.setPosition(armLeft);
               // pivotServo.setPosition(pivot);
              //intakeServo.setPosition(intakeServoPos);
             // intakeMotor.set(intakeMotorPower);
                droneServo.setPosition(drone);
              /*      clawL.setPosition(clawLeft);
                    clawR.setPosition(clawRight);*/
               //trap.setPosition(trapPos);

           /* slideMotorLeft.set(gamepad1.right_trigger - gamepad1.left_trigger);
          slideMotorRight.set(gamepad1.right_trigger - gamepad1.left_trigger);*/

           /* telemetry.addData("Slide Motor Left", slideMotorLeft.getCurrentPosition());
            telemetry.addData("Slide Motor Right", slideMotorRight.getCurrentPosition());*/
           //telemetry.addData("Intake Servo", intakeServo.getPosition());
          //  telemetry.addData("Rotate Servo", angleServo.getPosition());
           // telemetry.addData("Arm Servo Left", armServoLeft.getPosition());
            //telemetry.addData("Arm Servo Right", armServoRight.getPosition());
          // telemetry.addData("Pivot", pivotServo.getPosition());
            //telemetry.addData("Trap", trap.getPosition());
            telemetry.addData("Drone Servo", droneServo.getPosition());
            /*telemetry.addData("Right Encoder", rightFwEncoder.getCurrentPosition());
           telemetry.addData("Left Encoder", leftFwEncoder.getCurrentPosition());
            telemetry.addData("Strafe Encoder", strafeFwEncoder.getCurrentPosition());
           telemetry.addData("Extendo Motor", extendoMotor.getCurrentPosition());*/
            /*telemetry.addData("Claw L", clawL.getPosition());
            telemetry.addData("Claw R", clawR.getPosition());*/
            telemetry.update();
        }
    }
}
