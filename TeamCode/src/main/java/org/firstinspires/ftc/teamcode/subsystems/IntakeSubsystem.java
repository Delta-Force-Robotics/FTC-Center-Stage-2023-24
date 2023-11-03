package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private Motor intakeMotor;
    private Servo intakeServo;

    public IntakeSubsystem(Motor intakeMotor, Servo intakeServo) {
        this.intakeMotor = intakeMotor;
        this.intakeServo = intakeServo;

        this.intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setRunMode(Motor.RunMode.RawPower);

        this.intakeServo.setDirection(Servo.Direction.REVERSE);

        this.intakeServo.setPosition(Constants.INTAKE_SERVO_INIT_POS);
    }

    public void setIntakePower(double power) { intakeMotor.set(power); }
    public void setIntakePos(double pos) { intakeServo.setPosition(pos); }
 }
