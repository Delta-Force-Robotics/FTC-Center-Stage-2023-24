package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import java.util.function.BooleanSupplier;

public class ClimbSubsystem extends SubsystemBase {
    private Motor climbMotor;
    public BooleanSupplier isInterrupted;

    public ClimbSubsystem(Motor climbMotor) {
        this.climbMotor = climbMotor;

        this.climbMotor.setRunMode(Motor.RunMode.RawPower);
        this.climbMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.climbMotor.setInverted(true);

        this.climbMotor.resetEncoder();
    }

    public void setClimbPower(double power) {
        climbMotor.set(power);
    }

    public void setClimbPos(int pos) {
        climbMotor.setRunMode(Motor.RunMode.PositionControl);
        climbMotor.setTargetPosition(pos);
        climbMotor.set(0);

        climbMotor.setPositionTolerance(10);

        while (!climbMotor.atTargetPosition() && !isInterrupted.getAsBoolean()) {
            climbMotor.set(1);
        }

        climbMotor.stopMotor();
        climbMotor.setRunMode(Motor.RunMode.RawPower);
    }
 }
