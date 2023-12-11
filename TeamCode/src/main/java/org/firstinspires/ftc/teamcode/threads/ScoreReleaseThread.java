package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class ScoreReleaseThread extends Thread{
    private ScoreSubsystem scoreSubsystem;
    private SlideSubsystem slideSubsystem;
    public double slideLevel = 0;

    public ScoreReleaseThread(ScoreSubsystem scoreSubsystem, SlideSubsystem slideSubsystem) {
        this.scoreSubsystem = scoreSubsystem;
        this.slideSubsystem = slideSubsystem;
    }

    @Override
    public void run() {
        scoreSubsystem.useClaw(Constants.OPEN_CLAW);
        try {
            sleep(800);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        scoreSubsystem.pivotClaw(0);
        scoreSubsystem.rotateClaw(0.5);
        scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_30);

        try {
            sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        slideSubsystem.setLevel(slideLevel);
    }

    public void interrupt() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}
