package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private Motor climbMotor;
    private Servo climbServoLeft;
    private Servo climbServoRight;

    public ClimbSubsystem(Motor climbMotor, Servo climbServoLeft, Servo climbServoRight) {
        this.climbMotor = climbMotor;
        this.climbServoLeft = climbServoLeft;
        this.climbServoRight = climbServoRight;

        climbMotor.setRunMode(Motor.RunMode.PositionControl);
        climbMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        climbServoLeft.setDirection(Servo.Direction.REVERSE);
        climbServoRight.setDirection(Servo.Direction.REVERSE);

        climbServoLeft.setPosition(Constants.CLIMB_SERVO_INIT_POS);
        climbServoRight.setPosition(Constants.CLIMB_SERVO_INIT_POS);
    }

    public void setClimbPos(int pos) { climbMotor.setTargetPosition(pos); }

    public void setClimbServoPos(double pos) {
        climbServoLeft.setPosition(pos);
        climbServoRight.setPosition(pos);
    }
}
