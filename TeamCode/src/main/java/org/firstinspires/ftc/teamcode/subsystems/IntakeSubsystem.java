package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private Motor intakeMotor;
    private Servo intakeServo;
    private Servo trap;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public boolean intake = true;

    public IntakeSubsystem(Servo intakeServo, Servo trap, Motor intakeMotor) {
        this.intakeServo = intakeServo;
        this.trap = trap;
        this.intakeMotor = intakeMotor;

        this.intakeMotor.setRunMode(Motor.RunMode.RawPower);
        this.intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.intakeServo.setPosition(Constants.INTAKE_SERVO_INIT_POS - 0.08);
        this.trap.setPosition(Constants.TRAP_BLOCK);

    }
    public void setIntakePower(double power) { intakeMotor.set(power); }
    public void setIntakePos(double pos) { intakeServo.setPosition(pos); }
    public void useTrap(double pos) { trap.setPosition(pos); }
    public double getTrapPos() {
        return trap.getPosition();
    }
 }
