package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;

public class IntakeThread extends Thread {
    private IntakeSubsystem intakeSubsystem;
    private ScoreSubsystem scoreSubsystem;
    public double levelForSlides = 0;
    boolean isAuto;

    public IntakeThread(IntakeSubsystem intakeSubsystem, ScoreSubsystem scoreSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.scoreSubsystem = scoreSubsystem;

        this.setPriority(MIN_PRIORITY);
    }

    @Override
    public void run() {
        if (intakeSubsystem.getTrapPos() != Constants.TRAP_BLOCK) {
            scoreSubsystem.useArm(0.23);
            try {
                sleep(250);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            intakeSubsystem.setIntakePower(0);
            intakeSubsystem.useTrap(Constants.TRAP_BLOCK);
            try {
                sleep(300);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            scoreSubsystem.usePivot(Constants.PIVOT_SERVO_INIT_POSITION - 0.08);
            scoreSubsystem.useArm(Constants.ARM_SERVO_INTAKE_POS + 0.08);
        }

        intakeSubsystem.useTrap(Constants.TRAP_BLOCK);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
        intakeSubsystem.setIntakePower(1);
    }
}
