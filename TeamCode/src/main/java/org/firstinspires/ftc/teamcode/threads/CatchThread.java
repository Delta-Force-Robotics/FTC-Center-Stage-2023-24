package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;

public class CatchThread extends Thread {
    private ScoreSubsystem scoreSubsystem;
    private IntakeSubsystem intakeSubsystem;

    public CatchThread(ScoreSubsystem scoreSubsystem, IntakeSubsystem intakeSubsystem) {
        this.scoreSubsystem = scoreSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.setPriority(MIN_PRIORITY);
    }

    @Override
    public void run() {
        intakeSubsystem.setIntakePower(0);
        scoreSubsystem.useClaw(Constants.CLAW_SERVO_OPEN_POS, Constants.CLAW_BOTH);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS + 0.12);

        scoreSubsystem.useArm(0.23);
        try {
            sleep(150);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intakeSubsystem.setIntakePower(0);
        intakeSubsystem.useTrap(Constants.TRAP_OPEN - 0.03);
        try {
            sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION - 0.08);
        scoreSubsystem.useArm(Constants.ARM_SERVO_INTAKE_POS + 0.08);
        try {
            sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION);
        try {
            sleep(50);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scoreSubsystem.useArm(Constants.ARM_SERVO_INTAKE_POS);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        scoreSubsystem.useClaw(Constants.CLAW_SERVO_CLOSE_POS, Constants.CLAW_BOTH);
        try {
            sleep(300);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION - 0.1);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
        scoreSubsystem.useArm(Constants.ARM_SERVO_INIT_POSITION - 0.04);
        intakeSubsystem.setIntakePower(-1);
        intakeSubsystem.intake = true;
        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        intakeSubsystem.setIntakePower(0);
    }

    public void interrupt() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}