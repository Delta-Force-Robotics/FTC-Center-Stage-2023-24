package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class ScoreThread extends Thread {
    public SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;
    public double slideLevel = 0;

    public boolean selectRotate = false;

    public ScoreThread(SlideSubsystem slideSubsystem, ScoreSubsystem scoreSubsystem) {
        this.slideSubsystem = slideSubsystem;
        this.scoreSubsystem = scoreSubsystem;
        this.slideSubsystem.isInterrupted = this::isInterrupted;
    }

    @Override
    public void run() {
        scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);

        slideSubsystem.setLevel(slideLevel);
        scoreSubsystem.rotateClaw(0.26);
        scoreSubsystem.pivotClaw(0.88);
    }

    public void interrupt() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}