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

        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        try {
            sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        slideSubsystem.setLevel(slideLevel);

        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);
        try {
            sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        if (selectRotate == false)
            scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_INIT_POSITION);
        else
            scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_45);
    }

    public void interrupt() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}