package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardEx
        ;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficientsEx;
import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;


import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import java.util.function.BooleanSupplier;

public class SlideSubsystem extends SubsystemBase {
    public Motor slideMotorLeft;
    public Motor slideMotorRight;
    public BooleanSupplier isInterrupted;
    public double calculateLeft;
    public double calculateRight;
    private Telemetry telemetry;
    public boolean isAuto;
    public int currLevel = 1;

    private PIDController leftPIDController = new PIDController(Constants.SLIDE_FEEDBACK_KP, 0, 0);
    private PIDController rightPIDController = new PIDController(Constants.SLIDE_FEEDBACK_KP, 0, 0);
    private FeedforwardEx feedForwardController = new FeedforwardEx(new FeedforwardCoefficientsEx(Constants.SLIDE_FF_KV, Constants.SLIDE_FF_KA, Constants.SLIDE_FF_KS, Constants.SLIDE_FF_KG, 0));

    private WPILibMotionProfile motionProfile = getMotionProfile(0, 0);
    private Timer timer;
    private double motionProfileDelay = 0;
    public double lastVelocity = 0;
    public double lastTime = 0;

    public enum SlideState {
        INTAKE(Constants.SLIDE_INTAKE);

        private double id;
        SlideState(double slideLevel) {
            id = slideLevel;
        }

        public double getId() { return id; }
        public void setId(double slideLevel) {
            id = slideLevel;
        }
        public boolean isSameSlideLevel(double slideLevel) { return id == slideLevel; }
    }

    public SlideState slideState = SlideState.INTAKE;

    public SlideSubsystem(Motor slideMotorLeft, Motor slideMotorRight, Telemetry telemetry, boolean resetEncoders, boolean isAuto) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;
        this.telemetry = telemetry;

        this.slideMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.slideMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        this.slideMotorLeft.setRunMode(Motor.RunMode.RawPower);
        this.slideMotorRight.setRunMode(Motor.RunMode.RawPower);

        this.slideMotorLeft.setInverted(true);

        if(resetEncoders) {
            this.slideMotorLeft.resetEncoder();
            this.slideMotorRight.resetEncoder();
        }

        this.telemetry = telemetry;
        this.isAuto = isAuto;
    }

    @Override
    public void periodic() {
        showTelemetry();
        updateControlLoop();
    }

    public void updateControlLoop() {
        WPILibMotionProfile.State state = motionProfile.calculate(Math.max(timer.currentTime() - motionProfileDelay, 0));

        double leftPower = feedForwardController.calculate(
                state.position,
                state.velocity,
                (state.velocity - lastVelocity) / (timer.currentTime() - lastTime)
        ) + leftPIDController.calculate(ticksToMeters(slideMotorLeft.getCurrentPosition()), state.position) + getPassivePower();

        double rightPower = feedForwardController.calculate(
                state.position,
                state.velocity,
                (state.velocity - lastVelocity) / (timer.currentTime() - lastTime)
        ) + rightPIDController.calculate(ticksToMeters(slideMotorRight.getCurrentPosition()), state.position) + getPassivePower();

        lastVelocity = state.velocity;
        lastTime = timer.currentTime();

        slideMotorRight.set(rightPower);
        slideMotorLeft.set(leftPower);
    }

    public void setLevel(double level){
        setLevel(level, 0);
    }
  
    public void setLevel(double level, double delay) {
        timer = new Timer();
        motionProfileDelay = delay;
        motionProfile = getMotionProfile(level, 0);
    }

    public WPILibMotionProfile getMotionProfile(double goalLevel, double goalSpeed) {
        WPILibMotionProfile.Constraints constraints = new WPILibMotionProfile.Constraints(Constants.SLIDE_MAX_VELOCITY, Constants.SLIDE_MAX_ACCEL);
        WPILibMotionProfile.State initialState = new WPILibMotionProfile.State(ticksToMeters(getMotorTicks()), getMotorSpeedsAverage());
        WPILibMotionProfile.State goalState = new WPILibMotionProfile.State(goalLevel, goalSpeed);

        return new WPILibMotionProfile(constraints, initialState, goalState);
    }

    public double getPassivePower() {
        if(slideMotorLeft.getCurrentPosition() <= 0.05) {
            return 0;
        }

        return Constants.SLIDE_MOTOR_PASSIVE_POWER * Math.ceil(ticksToMeters(slideMotorLeft.getCurrentPosition())/Constants.SLIDE_MAX_EXTENSION_METERS*4) + Constants.SLIDE_MOTOR_PASSIVE_POWER_CLAW_WEIGHT;
    }

    public double getMotorSpeedsAverage() {
        double leftSpeed = slideMotorLeft.getCorrectedVelocity() / 60 * 2 * Math.PI * Constants.SPOOL_RADIUS_METERS;
        double rightSpeed = slideMotorRight.getCorrectedVelocity() / 60 * 2 * Math.PI * Constants.SPOOL_RADIUS_METERS;

        return (leftSpeed + rightSpeed) / 2;
    }

    public void setMotorPower( double power ) {
        slideMotorLeft.set(power);
        slideMotorRight.set(power);
    }

    public SlideState getSlideState() {
        return slideState;
    }

    public void setSlideState(SlideState slideState) {
        this.slideState = slideState;
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

    public int getCurrLevel() {
        return currLevel;
    }

    public void showTelemetry() {
        telemetry.addData("Motion Profile Velocity", lastVelocity);
        telemetry.addData("Motor Velocity", getMotorSpeedsAverage());
    }
}