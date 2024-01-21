package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;

public class OutakeThread extends Thread {
    private IntakeSubsystem intakeSubsystem;
    private ScoreSubsystem scoreSubsystem;

    public OutakeThread(IntakeSubsystem intakeSubsystem, ScoreSubsystem scoreSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.scoreSubsystem = scoreSubsystem;
    }

    @Override
    public void run() {
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        intakeSubsystem.setIntakePower(-1);
        try {
            sleep(1500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS);

        intakeSubsystem.setIntakePower(0);
    }
}
