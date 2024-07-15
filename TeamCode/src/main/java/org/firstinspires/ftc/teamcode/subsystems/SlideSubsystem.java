package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.constants.HardwareConstants;

import java.util.function.BooleanSupplier;

public class SlideSubsystem extends SubsystemBase {
    public Motor slideMotorLeft;
    public Motor slideMotorRight;
    public BooleanSupplier isInterrupted;
    private PIDFController pidfLeftSlideMotor;
    private PIDFController pidfRightSlideMotor;
    private double[] pidfCoefficientsExtend;
    private double[] pidfCoefficientsRetract;
    public double calculateLeft;
    public double calculateRight;
    private Telemetry telemetry;
    public boolean isAuto;
    HardwareMap hardwareMap;

    public SlideSubsystem(Telemetry telemetry, Motor slideMotorRight, Motor slideMotorLeft, boolean resetEncoders) {
        this.telemetry = telemetry;
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;

        this.slideMotorLeft.setRunMode(Motor.RunMode.RawPower);
        this.slideMotorRight.setRunMode(Motor.RunMode.RawPower);

        this.slideMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.slideMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.slideMotorLeft.setInverted(true);

        if(resetEncoders) {
            this.slideMotorLeft.resetEncoder();
            this.slideMotorRight.resetEncoder();
        }
    }

    /**
     * Sets the extension level for the slides using a PID.
     * @param level Intended level for slide extension, in millimeters.
     */
    public void setLevel(int level) {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.PRESET_POSITIONS;

        slideMotorLeft.setRunMode(Motor.RunMode.PositionControl);
        slideMotorRight.setRunMode(Motor.RunMode.PositionControl);

        slideMotorLeft.set(0);
        slideMotorRight.set(0);

        slideMotorLeft.setTargetPosition(level);
        slideMotorRight.setTargetPosition(level);

        slideMotorLeft.setPositionTolerance(10);
        slideMotorRight.setPositionTolerance(10);

        while (!slideMotorLeft.atTargetPosition() && !slideMotorRight.atTargetPosition() && !isInterrupted.getAsBoolean()) {
            slideMotorLeft.set(1);
            slideMotorRight.set(1);
        }

        slideMotorLeft.stopMotor();
        slideMotorRight.stopMotor();

        slideMotorLeft.setRunMode(Motor.RunMode.RawPower);
        slideMotorRight.setRunMode(Motor.RunMode.RawPower);

        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
    }

    public double getPassivePower() {
        if(slideMotorLeft.getCurrentPosition() <= 0.05) {
            return 0;
        }

        return Constants.SLIDE_MOTOR_PASSIVE_POWER * (double)ticksToMeters(slideMotorLeft.getCurrentPosition())/(double)Constants.SLIDE_MAX_EXTENSION_METERS + 0.1;
    }

    public void setMotorPower( double power ) {
        slideMotorLeft.set(power);
        slideMotorRight.set(power);
    }

    public double ticksToMeters(int ticks) {
        return (double) ticks / Constants.SLIDE_MAX_EXTENSION_TICKS * Constants.SLIDE_MAX_EXTENSION_METERS;
    }

    public double metersToTicks(double meters) {
        return Math.round(meters / Constants.SLIDE_MAX_EXTENSION_METERS * Constants.SLIDE_MAX_EXTENSION_TICKS);
    }

    public int getMotorTicks() {
        return slideMotorLeft.getCurrentPosition();
    }

    public double getSlideExtensionMeters() {
        return ticksToMeters(getMotorTicks());
    }

    public void resetEnc() {
        slideMotorLeft.resetEncoder();
        slideMotorRight.resetEncoder();
    }
}