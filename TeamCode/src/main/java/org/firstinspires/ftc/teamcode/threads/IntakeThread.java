package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;

public class IntakeThread extends Thread {
    private IntakeSubsystem intakeSubsystem;
    private ScoreSubsystem scoreSubsystem;
    public double levelForSlides = 0;
    boolean isAuto;

    public IntakeThread(IntakeSubsystem intakeSubsystem, ScoreSubsystem scoreSubsystem, boolean isAuto) {
        this.intakeSubsystem = intakeSubsystem;
        this.scoreSubsystem = scoreSubsystem;
        this.isAuto = isAuto;
    }

    @Override
    public void run() {

        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_SCORE_POS);
        try {
            sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INTAKE_POS);
        intakeSubsystem.setIntakePower(1);
    }
}
