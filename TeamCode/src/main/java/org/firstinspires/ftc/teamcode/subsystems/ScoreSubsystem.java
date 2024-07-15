package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;

public class ScoreSubsystem extends SubsystemBase {
    private Servo armServoL;
    private Servo armServoR;
    private Servo pivotServo;
    private Servo rotateServo;
    private Servo clawServoL;
    private Servo clawServoR;
    private Servo droneServo;
    private Telemetry telemetry;

    public boolean retract = false;
    public boolean isScoring = true;

    public ScoreSubsystem(Servo armServoR, Servo armServoL, Servo rotateServo, Servo pivotServo, Servo clawServoL, Servo clawServoR, Servo droneServo, boolean isAuto) {
        this.armServoR = armServoR;
        this.armServoL = armServoL;
        this.rotateServo = rotateServo;
        this.pivotServo = pivotServo;
        this.clawServoL = clawServoL;
        this.clawServoR = clawServoR;
        this.droneServo = droneServo;

        this.armServoL.setDirection(Servo.Direction.REVERSE);
        this.pivotServo.setDirection(Servo.Direction.REVERSE);
        this.clawServoL.setDirection(Servo.Direction.REVERSE);
        this.rotateServo.setDirection(Servo.Direction.REVERSE);
        this.droneServo.setDirection(Servo.Direction.REVERSE);

        this.armServoR.setPosition(Constants.ARM_SERVO_INIT_POSITION - 0.023);
        this.armServoL.setPosition(Constants.ARM_SERVO_INIT_POSITION - 0.023);
        this.rotateServo.setPosition(Constants.ANGLE_INIT_POSITION);
        this.pivotServo.setPosition(Constants.PIVOT_SERVO_INIT_POSITION);
        this.clawServoL.setPosition(Constants.CLAW_SERVO_CLOSE_POS + 0.06);
        this.clawServoR.setPosition(Constants.CLAW_SERVO_CLOSE_POS);
        this.droneServo.setPosition(Constants.DRONE_SERVO_INIT_POS);
    }

    public void useClaw(double pos, int clawP) {
        if (clawP == 0) {
            clawServoL.setPosition(pos + 0.06);
            clawServoR.setPosition(pos);
        } else if (clawP == 1) {
            clawServoL.setPosition(pos + 0.06);
        } else if (clawP == 2) {
            clawServoR.setPosition(pos);
        }
    }
    public boolean clawIsOpend() {
        if (clawServoL.getPosition() == Constants.CLAW_SERVO_OPEN_POS && clawServoR.getPosition() == Constants.CLAW_SERVO_OPEN_POS) {
            return true;
        }
        return false;
    }

    public void usePivot(double pos) {
        pivotServo.setPosition(pos);
    }

    public void useArm(double armPosition) {
        armServoL.setPosition(armPosition);
        armServoR.setPosition(armPosition);
    }
    public void rotateClaw(double pos) {
        rotateServo.setPosition(pos);
    }

    public void useDrone(double pos) {
        droneServo.setPosition(pos);
    }
}
