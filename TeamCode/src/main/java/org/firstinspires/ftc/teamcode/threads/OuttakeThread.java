package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;

public class OuttakeThread extends Thread {
    private IntakeSubsystem intakeSubsystem;
    private ScoreSubsystem scoreSubsystem;

    public OuttakeThread(IntakeSubsystem intakeSubsystem, ScoreSubsystem scoreSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.scoreSubsystem = scoreSubsystem;
    }

    @Override
    public void run() {
        scoreSubsystem.useBlock(Constants.BLOCK_SERVO_BLOCK_POS);
        intakeSubsystem.setIntakePos(0);
        intakeSubsystem.setIntakePower(-1);
        try {
            sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        intakeSubsystem.setIntakePower(0);
    }
}
