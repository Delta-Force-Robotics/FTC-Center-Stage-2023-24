package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class ScoreReleaseThread extends Thread{
    private ScoreSubsystem scoreSubsystem;
    private SlideSubsystem slideSubsystem;

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

        scoreSubsystem.rotateClaw(Constants.ROTATE_SERVO_INIT_POSITION);
        scoreSubsystem.pivotClaw(Constants.PIVOT_PIVOT_POS);
        scoreSubsystem.useArm(Constants.ARM_SERVO_PIVOT_30);

        slideSubsystem.setLevel(Constants.SLIDE_INTAKE);
        scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION);
    }
}
