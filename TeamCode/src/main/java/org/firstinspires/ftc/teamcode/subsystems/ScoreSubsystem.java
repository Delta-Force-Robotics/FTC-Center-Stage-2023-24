package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class ScoreSubsystem extends SubsystemBase {
    private Servo armServoL;
    private Servo armServoR;
    private Servo rotateServo;
    private Servo blockServo;
    private Servo droneServo;

    public ScoreSubsystem(Servo armServoL, Servo armServoR, Servo rotateServo, Servo blockServo, Servo droneServo, boolean isAuto) {
        this.armServoL = armServoL;
        this.armServoR = armServoR;
        this.rotateServo = rotateServo;
        this.blockServo = blockServo;
        this.droneServo = droneServo;

        this.armServoL.setDirection(Servo.Direction.REVERSE);

        if (!isAuto) {
            this.armServoR.setPosition(Constants.ARM_SERVO_INIT_POSITION);
            this.armServoL.setPosition(Constants.ARM_SERVO_INIT_POSITION);
            this.rotateServo.setPosition(Constants.ROTATE_SERVO_INIT_POSITION);
            this.blockServo.setPosition(Constants.BLOCK_SERVO_SCORE_POS);
            this.droneServo.setPosition(Constants.DRONE_SERVO_INIT_POS);
        } else {
            this.armServoL.setPosition(Constants.ARM_SERVO_INIT_POSITION);
            this.armServoR.setPosition(Constants.ARM_SERVO_INIT_POSITION);
            this.rotateServo.setPosition(Constants.ROTATE_SERVO_INIT_POSITION);
            this.blockServo.setPosition(Constants.BLOCK_SERVO_BLOCK_POS);
            this.droneServo.setPosition(Constants.DRONE_SERVO_INIT_POS);
        }
    }

    public void rotateClaw(double rotatePosition) {
        rotateServo.setPosition(rotatePosition);
    }

    public void useArm(double armPosition) {
        armServoL.setPosition(armPosition);
        armServoR.setPosition(armPosition);
    }

    public void useBlock(double position) {
        blockServo.setPosition(position);
    }

    public void setDroneServoPos(double pos) {
        droneServo.setPosition(pos);
    }
}
