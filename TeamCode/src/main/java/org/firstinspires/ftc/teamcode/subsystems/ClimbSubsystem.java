package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;

import java.util.function.BooleanSupplier;

public class ClimbSubsystem extends SubsystemBase {
    private Motor climbMotor;

    private PIDFController pidfClimbMotor;
    private double calculate;
    private Telemetry telemetry;
    public BooleanSupplier isInterrupted;
    private double position = 0;


    public ClimbSubsystem(Motor climbMotor, Telemetry telemetry, boolean resetEncoder) {
        this.climbMotor = climbMotor;

        this.climbMotor.setRunMode(Motor.RunMode.RawPower);
        this.climbMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.climbMotor.setInverted(true);
        this.telemetry = telemetry;

        if (resetEncoder) {
            this.climbMotor.resetEncoder();
        }
    }


    public void setClimbPower(double power) {
        climbMotor.set(power);
    }

    public void setClimbPos(int pos) {
        climbMotor.setRunMode(Motor.RunMode.PositionControl);
        climbMotor.setTargetPosition(pos);
        climbMotor.set(0);

        climbMotor.setPositionTolerance(20);

        while (!climbMotor.atTargetPosition() && !isInterrupted.getAsBoolean()) {
            climbMotor.set(0.2);
        }

        climbMotor.stopMotor();
    }
 }
