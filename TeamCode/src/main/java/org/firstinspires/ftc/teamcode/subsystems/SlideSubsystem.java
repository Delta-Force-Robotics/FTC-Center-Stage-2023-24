package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.threads.SlideLevelThread;

import java.util.function.BooleanSupplier;

public class SlideSubsystem extends SubsystemBase {
    private Motor slideMotorLeft;
    private Motor slideMotorRight;
    private SlideLevelThread slideLevelThread;
    private Telemetry telemetry;

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

    public SlideSubsystem(Motor slideMotorLeft, Motor slideMotorRight, SlideLevelThread slideLevelThread, boolean resetEncoders) {
        this.slideMotorLeft = slideMotorLeft;
        this.slideMotorRight = slideMotorRight;
        this.slideLevelThread = slideLevelThread;

        this.slideMotorLeft.setInverted(true);

        if (resetEncoders) {
            slideMotorLeft.resetEncoder();
            slideMotorRight.resetEncoder();
        }
    }

    public void setLevel(double level) {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.PRESET_POSITIONS;

        // set the run mode
        slideMotorLeft.setRunMode(Motor.RunMode.PositionControl);
        slideMotorRight.setRunMode(Motor.RunMode.PositionControl);

        // set the target position
        slideMotorLeft.setTargetPosition(metersToTicks(level)); // an integer representing desired tick count
        slideMotorRight.setTargetPosition(metersToTicks(level));

        slideMotorLeft.set(0);
        slideMotorRight.set(0);

        // set the tolerance
        slideMotorLeft.setPositionTolerance(13.6);   // allowed maximum error
        slideMotorRight.setPositionTolerance(13.6);

        // perform the control loop
        while (!slideMotorLeft.atTargetPosition()) {
            slideMotorLeft.set(1);
            slideMotorRight.set(1);
        }

        telemetry.addData("slideMotorLeft", ticksToMeters(slideMotorLeft.getCurrentPosition()));
        telemetry.addData("slideMotorRight", ticksToMeters(slideMotorRight.getCurrentPosition()));

        slideMotorLeft.stopMotor();
        slideMotorRight.stopMotor();
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

    public double getSlideExtensionMeters() {
        return ticksToMeters(getMotorTicks());
    }

    public void changeLevel(){
        if (slideLevelThread.currLevel == 1) {
            setLevel(Constants.SLIDE_POSITIONS[0]);
        }

        else if (slideLevelThread.currLevel == 2) {
            setLevel(Constants.SLIDE_POSITIONS[1]);
        }

        else if (slideLevelThread.currLevel == 3){
            setLevel(Constants.SLIDE_POSITIONS[2]);
        }

        else if (slideLevelThread.currLevel == 4){
            setLevel(Constants.SLIDE_POSITIONS[3]);
        }

        else if (slideLevelThread.currLevel == 5){
            setLevel(Constants.SLIDE_POSITIONS[4]);
        }

        else if (slideLevelThread.currLevel == 6){
            setLevel(Constants.SLIDE_POSITIONS[5]);
        }

        else if (slideLevelThread.currLevel == 7){
            setLevel(Constants.SLIDE_POSITIONS[6]);
        }

        else if (slideLevelThread.currLevel == 8){
            setLevel(Constants.SLIDE_POSITIONS[7]);
        }

        else if (slideLevelThread.currLevel == 9){
            setLevel(Constants.SLIDE_POSITIONS[8]);
        }

        else if (slideLevelThread.currLevel == 10){
            setLevel(Constants.SLIDE_POSITIONS[9]);
        }
    }

}