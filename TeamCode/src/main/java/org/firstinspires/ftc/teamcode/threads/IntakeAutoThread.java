package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ScoreSubsystem;

public class IntakeAutoThread extends Thread {
    private final IntakeSubsystem intakeSubsystem;
    public double intakeLevel = 0;

    public IntakeAutoThread(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void run() {
        intakeSubsystem.setIntakePower(1);
        intakeSubsystem.setIntakePos(intakeLevel);
    }
}
