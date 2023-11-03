package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class ScoreThread extends Thread {
    private SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;
    public double levelForSlides = 0;
    public boolean selectRotate = false;

    public ScoreThread(SlideSubsystem slideSubsystem, ScoreSubsystem scoreSubsystem) {
        this.slideSubsystem = slideSubsystem;
        this.scoreSubsystem = scoreSubsystem;
    }

    @Override
    public void run() {
            scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);
            scoreSubsystem.pivotClaw(Constants.PIVOT_PIVOT_POS);

            if(selectRotate = false) {
                scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_45);
            } else if (selectRotate = true) {
                scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_180);
            }

            slideSubsystem.changeLevel();
    }

    public void interrupt() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}
