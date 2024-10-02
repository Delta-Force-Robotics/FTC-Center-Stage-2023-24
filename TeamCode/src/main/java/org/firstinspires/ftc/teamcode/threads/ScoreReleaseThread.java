package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class ScoreReleaseThread extends Thread {
    public SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;
    public double slideLevel = 0;

    public boolean selectRotate = false;

    public ScoreReleaseThread(SlideSubsystem slideSubsystem, ScoreSubsystem scoreSubsystem) {
        this.slideSubsystem = slideSubsystem;
        this.scoreSubsystem = scoreSubsystem;
        this.slideSubsystem.isInterrupted = this::isInterrupted;
    }

    @Override
    public void run() {
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
        try {
            sleep(350);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_INIT_POSITION);
        try {
            sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION);

        try {
            sleep(150);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        slideSubsystem.setLevel(slideLevel);
    }
}