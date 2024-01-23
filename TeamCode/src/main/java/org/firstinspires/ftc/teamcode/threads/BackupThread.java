package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class BackupThread extends Thread {
    private ScoreSubsystem scoreSubsystem;
    public double slideLevel = 0;

    public boolean selectRotate = false;

    public BackupThread(ScoreSubsystem scoreSubsystem) {
        this.scoreSubsystem = scoreSubsystem;
    }

    @Override
    public void run() {
        scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_POSITION);

        try {
            sleep(300);
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