package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FeedbackController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.FeedforwardController;
import com.ThermalEquilibrium.homeostasis.Utils.Timer;
import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;

public class SlideSubsystem extends SubsystemBase {
    private Motor slideMotorLeft;
    private Motor slideMotorRight;
    private int currLevel = 1;
    private Telemetry telemetry;

    private PIDController leftPIDController;
    private PIDController rightPIDController;
    private FeedforwardController feedForwardController;

    private WPILibMotionProfile motionProfile = null;
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

    public SlideSubsystem(Motor slideMotorLeft, Motor slideMotorRight, Telemetry telemetry, boolean resetEncoders) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;
        this.telemetry = telemetry;

        this.slideMotorLeft.setInverted(true);

        if (resetEncoders) {
            slideMotorLeft.resetEncoder();
            slideMotorRight.resetEncoder();
        }
    }

    @Override
    public void periodic() {
        showTelemetry();
        updateControlLoop();
    }

    public void updateControlLoop() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.PRESET_POSITIONS;

        WPILibMotionProfile.State state = motionProfile.calculate(timer.currentTime() - motionProfileDelay);
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
        
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
    }

    public void setLevel(double level){
        setLevel(level, 0);
    }
    public void setLevel(double level, double delay) {
        timer = new Timer();
        motionProfileDelay = delay;

        WPILibMotionProfile.Constraints constraints = new WPILibMotionProfile.Constraints(Constants.SLIDE_MAX_VELOCITY, Constants.SLIDE_MAX_ACCEL);
        WPILibMotionProfile.State initialState = new WPILibMotionProfile.State(ticksToMeters(getMotorTicks()), getMotorSpeedsAverage());
        WPILibMotionProfile.State goalState = new WPILibMotionProfile.State(level, 0);

        motionProfile = new WPILibMotionProfile(constraints, initialState, goalState);
    }

    public double getPassivePower() {
        if(slideMotorLeft.getCurrentPosition() <= 0.05) {
            return 0;
        }

        return Constants.SLIDE_MOTOR_PASSIVE_POWER * Math.ceil(ticksToMeters(slideMotorLeft.getCurrentPosition())/Constants.SLIDE_MAX_EXTENSION_METERS*4) + Constants.SLIDE_MOTOR_PASSIVE_POWER_CLAW_WEIGHT;
    }

    public double getMotorSpeedsAverage() {
        double leftSpeed = slideMotorLeft.getCorrectedVelocity() / 60 * 2 * Math.PI * Constants.SPOOL_SIZE_METERS;
        double rightSpeed = slideMotorRight.getCorrectedVelocity() / 60 * 2 * Math.PI * Constants.SPOOL_SIZE_METERS;


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

    public int metersToTicks(double meters) {
        return (int) Math.round(meters / Constants.SLIDE_MAX_EXTENSION_METERS * Constants.SLIDE_MAX_EXTENSION_TICKS);
    }

    public int getMotorTicks() {
        return slideMotorLeft.getCurrentPosition();
    }

    private void showTelemetry() {
        if (currLevel == 1) {
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - 🟩🟩🟩🟩");
        }
        else if (currLevel == 2) {
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - 🟩🟩🟩🟩\n" +
                            " 1 - ⬜️⬜️⬜️⬜️");
        }
        else if (currLevel == 3){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - 🟩🟩🟩🟩\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 4){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - 🟩🟩🟩🟩\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 5){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - 🟩🟩🟩🟩\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 6){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - 🟩🟩🟩🟩️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 7){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - 🟩🟩🟩🟩️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 8){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - 🟩🟩🟩🟩\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 9){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- ⬜️⬜️⬜️⬜️\n" +
                            " 9 - 🟩🟩🟩🟩️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
        else if (currLevel == 10){
            telemetry.log().clear();
            telemetry.log().add(
                    "10- 🟩🟩🟩🟩️️\n" +
                            " 9 - ⬜️⬜️⬜️⬜️\n" +
                            " 8 - ⬜️⬜️⬜️⬜️\n" +
                            " 7 - ⬜️⬜️⬜️⬜️\n" +
                            " 6 - ⬜️⬜️⬜️⬜️\n" +
                            " 5 - ⬜️⬜️⬜️⬜️\n" +
                            " 4 - ⬜️⬜️⬜️⬜️\n" +
                            " 3 - ⬜️⬜️⬜️⬜️\n" +
                            " 2 - ⬜️⬜️⬜️⬜️\n" +
                            " 1 - ⬜️⬜️⬜️⬜️\n");
        }
    }

    public void setCurrentLevel(int currLevel){
        this.currLevel = Range.clip(currLevel, 1, 10);
    }

    public void changeLevel() {
        setLevel(Constants.SLIDE_POSITIONS[currLevel-1]);
    }

    public int getCurrLevel() {
        return currLevel;
    }
}