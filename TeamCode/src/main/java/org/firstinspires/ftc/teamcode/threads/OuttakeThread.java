package org.firstinspires.ftc.teamcode.threads;

import org.firstinspires.ftc.teamcode.constants.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class OuttakeThread extends Thread {
    public IntakeSubsystem intakeSubsystem;
    public int extendoLevel = 0;

    public OuttakeThread(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        this.setPriority(MIN_PRIORITY);
    }

    @Override
    public void run() {
        intakeSubsystem.setIntakePower(-1);
        intakeSubsystem.setIntakePos(Constants.INTAKE_SERVO_INIT_POS + 0.12);

        try {
            sleep(800);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        intakeSubsystem.setIntakePower(0);
        intakeSubsystem.intake = true;
    }

    public void interrupt() {
        Constants.SLIDE_INPUT_STATE = Constants.InputState.MANUAL_CONTROL;
        super.interrupt();
    }
}