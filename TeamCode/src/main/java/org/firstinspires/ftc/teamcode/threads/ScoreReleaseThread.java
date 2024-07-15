package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideSubsystem;

public class ScoreReleaseThread extends Thread {
    public SlideSubsystem slideSubsystem;
    private ScoreSubsystem scoreSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public ScoreReleaseThread(SlideSubsystem slideSubsystem, ScoreSubsystem scoreSubsystem, IntakeSubsystem intakeSubsystem) {
        this.slideSubsystem = slideSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.scoreSubsystem = scoreSubsystem;
        this.slideSubsystem.isInterrupted = this::isInterrupted;

        this.setPriority(MIN_PRIORITY);
    }

    @Override
    public void run() {
        scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH);
        intakeSubsystem.useTrap(Constants.TRAP_BLOCK);
        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        scoreSubsystem.rotateClaw(Constants.ANGLE_INIT_POSITION);
        scoreSubsystem.usePivot(0);
        scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION - 0.04);
        try {
            sleep(400);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        slideSubsystem.setLevel(0);
    }

    public void interrupt() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}