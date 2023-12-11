package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class ScoreSubsystem extends SubsystemBase {
    private Servo clawServo;
    private Servo pivotServo;
    private Servo armServoL;
    private Servo armServoR;
    private Servo rotateServo;
    private Servo droneServo;

    private DigitalChannel touchSensorLeft;
    private DigitalChannel touchSensorRight;

    public ScoreSubsystem(Servo clawServo, Servo pivotServo, Servo armServoL, Servo armServoR, Servo rotateServo, Servo droneServo, DigitalChannel touchSensorLeft, DigitalChannel touchSensorRight, boolean isAuto) {
        this.clawServo = clawServo;
        this.pivotServo = pivotServo;
        this.armServoL = armServoL;
        this.armServoR = armServoR;
        this.rotateServo = rotateServo;
        this.droneServo = droneServo;

        this.touchSensorLeft = touchSensorLeft;
        this.touchSensorRight = touchSensorRight;

        this.armServoL.setDirection(Servo.Direction.REVERSE);
        this.droneServo.setDirection(Servo.Direction.REVERSE);

        if (!isAuto) {
            this.clawServo.setPosition(Constants.OPEN_CLAW);
            this.pivotServo.setPosition(Constants.PIVOT_INIT_POS);
            this.armServoR.setPosition(Constants.ARM_SERVO_INIT_POSITION);
            this.armServoL.setPosition(Constants.ARM_SERVO_INIT_POSITION);
            this.rotateServo.setPosition(Constants.ROTATE_SERVO_INIT_POSITION);
            this.droneServo.setPosition(Constants.DRONE_SERVO_INIT_POS);
        } else {
            this.clawServo.setPosition(Constants.CLOSE_CLAW_AUTO);
            this.pivotServo.setPosition(Constants.PIVOT_INIT_POS);
            this.armServoL.setPosition(Constants.ARM_SERVO_INIT_POSITION);
            this.armServoR.setPosition(Constants.ARM_SERVO_INIT_POSITION);
            this.rotateServo.setPosition(Constants.ROTATE_SERVO_INIT_POSITION);
            this.droneServo.setPosition(Constants.DRONE_SERVO_INIT_POS);
        }
    }
    public void useClaw(double clawPosition) {
        clawServo.setPosition(clawPosition);
    }

    public void pivotClaw(double pos) {
        pivotServo.setPosition(pos);
    }

    public void rotateClaw(double rotatePosition) {
        rotateServo.setPosition(rotatePosition);
    }

    public void useArm(double armPosition) {
        armServoL.setPosition(armPosition);
        armServoR.setPosition(armPosition);
    }

    public void setDroneServoPos(double pos) {
        droneServo.setPosition(pos);
    }
    public boolean isLeftPressed() { return touchSensorLeft.getState(); }
    public boolean isRightPressed() { return touchSensorRight.getState(); }
}
