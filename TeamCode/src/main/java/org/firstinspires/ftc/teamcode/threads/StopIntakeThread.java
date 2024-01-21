package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;

public class StopIntakeThread extends Thread {
    private IntakeSubsystem intakeSubsystem;
    private ScoreSubsystem scoreSubsystem;
    public double levelForSlides = 0;
    boolean isAuto;

    public StopIntakeThread(IntakeSubsystem intakeSubsystem, ScoreSubsystem scoreSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.scoreSubsystem = scoreSubsystem;
    }

    @Override
    public void run() {
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        intakeSubsystem.setIntakePower(0);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);
    }
}
